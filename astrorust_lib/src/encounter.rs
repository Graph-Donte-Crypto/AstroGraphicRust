use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::orbit_3d::Orbit3D;
use crate::trajectory::Trajectory;
use nalgebra::{Matrix2, Vector2};

/// Coarse filter: heliocentric distance bounds
pub fn is_encounter_possible(
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
) -> bool {
    let is_possible = spacecraft.apoapsis() + r_soi >= planet.periapsis()
        && spacecraft.periapsis() - r_soi <= planet.apoapsis();

    if is_possible && let Trajectory::Elliptic(spc) = spacecraft {
        return find_encounter(spc, planet, r_soi).is_some();
    }

    is_possible
}

/// Coarse filter: heliocentric distance bounds
pub fn coarse_encounter_interval(
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
) -> ((f64, f64), (f64, f64)) {
    let lo = -(planet.periapsis() - spacecraft.a() - r_soi) / (spacecraft.a() * spacecraft.e());
    let hi = -(planet.apoapsis() - spacecraft.a() + r_soi) / (spacecraft.a() * spacecraft.e());
    if spacecraft.e() < 1.0 {
        let (lo, hi) = (lo.max(-1.0).acos().to_degrees(), hi.min(1.0).acos().to_degrees());
        ((lo, hi), (-lo, -hi))
    } else {
        let (lo, hi) = (lo.max(1.0).acosh(), hi.acosh());
        ((lo, hi), (-lo, -hi))
    }
}

/// Compute the scaled coupling matrix M = diag(a₁,b₁) · C · diag(a₂,b₂)
/// where C = 2 Aᵀ B encodes the mutual orientation of the two orbital planes.
fn scaled_coupling_matrix(
    orbit1: &Orbit3D<EllipticOrbit>,
    orbit2: &Orbit3D<EllipticOrbit>,
) -> Matrix2<f64> {
    let c = orbit1.orb_to_ecl().transpose() * orbit2.orb_to_ecl() * 2.0;

    let a1 = orbit1.orbit_2d.0.a();
    let e1 = orbit1.orbit_2d.0.e();
    let b1 = a1 * (1.0 - e1 * e1).sqrt();

    let a2 = orbit2.orbit_2d.0.a();
    let e2 = orbit2.orbit_2d.0.e();
    let b2 = a2 * (1.0 - e2 * e2).sqrt();

    Matrix2::new(c[(0, 0)] * a1 * a2, c[(0, 1)] * a1 * b2, c[(1, 0)] * b1 * a2, c[(1, 1)] * b1 * b2)
}

const MAX_ITER: usize = 50;
const TOL: f64 = 1e-12;

/// Find eccentric anomalies (E₁, E₂) where the distance between two elliptic
/// orbits equals r_soi, using Newton's method on f = r₁² + r₂² - p₁ᵀ M p₂ - r_soi².
///
/// Returns `None` if Newton's method fails to converge.
pub fn find_encounter(
    orbit1: &Orbit3D<EllipticOrbit>,
    orbit2: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
) -> Option<(f64, f64)> {
    let ((lo, hi), _) =
        coarse_encounter_interval(&Trajectory::Elliptic(orbit1.clone()), orbit2, r_soi);
    let a1 = orbit1.orbit_2d.0.a();
    let e1 = orbit1.orbit_2d.0.e();
    let a2 = orbit2.orbit_2d.0.a();
    let e2 = orbit2.orbit_2d.0.e();

    let b1 = a1 * (1.0 - e1 * e1).sqrt();
    let b2 = a2 * (1.0 - e2 * e2).sqrt();

    let m = scaled_coupling_matrix(orbit1, orbit2);
    let r_soi_sq = r_soi * r_soi;

    // Initial guess: E1 from coarse interval midpoint, E2 by projecting
    // the spacecraft position onto the planet's orbital plane.
    // For nearly circular, coplanar orbits this is close to exact.
    // Try both symmetric branches and pick the one with smaller f.
    let initial_e1_guess = |mid_deg: f64| -> (f64, f64, f64) {
        let ea1 = mid_deg.to_radians();
        let r1_2d = Vector2::new(a1 * (ea1.cos() - e1), b1 * ea1.sin());
        let r1_3d = orbit1.orb_to_ecl() * r1_2d;
        let q = orbit2.orb_to_ecl().transpose() * r1_3d;
        let ea2 = (q.y / b2).atan2(q.x / a2 + e2);
        let p1 = Vector2::new(ea1.cos() - e1, ea1.sin());
        let p2 = Vector2::new(ea2.cos() - e2, ea2.sin());
        let r1 = a1 * (1.0 - e1 * ea1.cos());
        let r2 = a2 * (1.0 - e2 * ea2.cos());
        let f = r1 * r1 + r2 * r2 - p1.dot(&(m * p2));
        (ea1, ea2, f)
    };
    let mid = (lo + hi) / 2.0;
    let (ea1_a, ea2_a, f_a) = initial_e1_guess(mid);
    let (ea1_b, ea2_b, f_b) = initial_e1_guess(-mid);
    let (mut ea1, mut ea2) = if f_a <= f_b {
        (ea1_a, ea2_a)
    } else {
        (ea1_b, ea2_b)
    };

    let mut prev_f = f64::INFINITY;

    for i in 0..MAX_ITER {
        let (sin1, cos1) = ea1.sin_cos();
        let (sin2, cos2) = ea2.sin_cos();

        // Dimensionless position vectors: p_i = (cos E_i - e_i, sin E_i)
        let p1 = Vector2::new(cos1 - e1, sin1);
        let p2 = Vector2::new(cos2 - e2, sin2);

        // Derivative vectors
        let w1 = Vector2::new(-sin1, cos1);
        let w2 = Vector2::new(-sin2, cos2);

        // Second-derivative direction (u_i = (cos E_i, sin E_i))
        let u1 = Vector2::new(cos1, sin1);
        let u2 = Vector2::new(cos2, sin2);

        // Precompute M · p₂, M · ŵ₂, M · û₂ (shared intermediates)
        let m_p2 = m * p2;
        let m_w2 = m * w2;
        let m_u2 = m * u2;

        // Heliocentric distances squared: r_i² = a_i²(1 - e_i cos E_i)²
        let r1 = a1 * (1.0 - e1 * cos1);
        let r1_sq = r1 * r1;
        let r2 = a2 * (1.0 - e2 * cos2);
        let r2_sq = r2 * r2;

        // f = r₁² + r₂² - p₁ᵀ M p₂ - r_soi²
        let f = r1_sq + r2_sq - p1.dot(&m_p2);

        println!("{i:>4}  E1={ea1:>20.12}  E2={ea2:>20.12}  f={f:>16.2}  d={:>.2}", f.sqrt());

        // Early exit: f is above r_soi² and not making progress
        if f > r_soi_sq && f >= prev_f {
            return None;
        }
        prev_f = f;

        // Gradient
        let self_grad1 = 2.0 * a1 * a1 * e1 * sin1 * (1.0 - e1 * cos1);
        let self_grad2 = 2.0 * a2 * a2 * e2 * sin2 * (1.0 - e2 * cos2);
        let f1 = self_grad1 - w1.dot(&m_p2);
        let f2 = self_grad2 - p1.dot(&m_w2);

        // Hessian
        let self_hess1 = 2.0 * a1 * a1 * e1 * (cos1 - e1 + 2.0 * e1 * sin1 * sin1);
        let self_hess2 = 2.0 * a2 * a2 * e2 * (cos2 - e2 + 2.0 * e2 * sin2 * sin2);
        let h11 = self_hess1 + u1.dot(&m_p2);
        let h22 = self_hess2 + p1.dot(&m_u2);
        let h12 = -w1.dot(&m_w2);

        // Cramer's rule: solve H δ = -∇f, with gradient descent fallback
        // when H is not positive definite (saddle point or maximum)
        let det = h11 * h22 - h12 * h12;
        let (d_e1, d_e2) = if det > TOL && h11 > 0.0 {
            // H is positive definite → Newton step
            ((f1 * h22 - f2 * h12) / (-det), (f2 * h11 - f1 * h12) / (-det))
        } else {
            // Gradient descent with step size scaled by 1/|∇f|
            let grad_norm_sq = f1 * f1 + f2 * f2;
            if grad_norm_sq <= f64::EPSILON {
                return None;
            }
            let alpha = f / grad_norm_sq;
            (-alpha * f1, -alpha * f2)
        };

        ea1 += d_e1;
        ea2 += d_e2;

        if d_e1.abs() < TOL && d_e2.abs() < TOL {
            if f <= r_soi_sq {
                return Some((ea1, ea2));
            } else {
                return None;
            }
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Config, StarSystem};
    use crate::orbit::orbit_3d::KeplerianElements;

    #[test]
    fn encounter_voyager2_jupiter() {
        let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
        let config = Config::load_from_yaml(&format!("{config_dir}/config.yml")).unwrap();
        let system = StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

        let spacecraft: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(config.spacecraft.orbit).into();

        let jupiter_cfg = system.planets.iter().find(|p| p.body.name == "Jupiter").unwrap();
        let jupiter: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(jupiter_cfg.orbit.clone()).into();
        let r_soi = jupiter_cfg.soi_radius();

        println!("Jupiter SOI radius: {r_soi:.0} km");

        let traj: Trajectory = Trajectory::Elliptic(spacecraft.clone());

        let result = find_encounter(&spacecraft, &jupiter, r_soi);
        println!("Result: {result:?}");
        assert!(result.is_some(), "Newton's method did not converge");
    }

    #[test]
    fn encounter_voyager2_earth() {
        let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
        let config = Config::load_from_yaml(&format!("{config_dir}/config.yml")).unwrap();
        let system = StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

        let spacecraft: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(config.spacecraft.orbit).into();

        let earth_cfg = system.planets.iter().find(|p| p.body.name == "Earth").unwrap();
        let earth: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(earth_cfg.orbit.clone()).into();
        let r_soi = earth_cfg.soi_radius();

        println!("Earth SOI radius: {r_soi:.0} km");

        let result = find_encounter(&spacecraft, &earth, r_soi);
        println!("Result: {result:?}");
    }
}
