use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::orbit_3d::Orbit3D;
use crate::orbit::{self};
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
        let (lo, hi) = (lo.clamp(-1.0, 1.0).acos().to_degrees(), hi.clamp(-1.0, 1.0).acos().to_degrees());
        dbg!(&(lo, hi));
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
    let (T1, T2) = (orbit1.period().as_secs() / 86400.0 / 365.2425, orbit2.period().as_secs() / 86400.0 / 365.2425);
    let T_syn = T1 * T2 / (T1 - T2).abs();
    dbg!(T1, T2, T_syn);
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

    // Initial guess via parabolic interpolation on the coarse interval.
    // For each ±branch, evaluate f at (lo, mid, hi), fit a quadratic,
    // and use its minimum as E₁. E₂ is estimated by projecting the
    // spacecraft position onto the planet's orbital plane.
    let eval_e1 = |ea1: f64| -> (f64, f64) {
        let r1_2d = Vector2::new(a1 * (ea1.cos() - e1), b1 * ea1.sin());
        let r1_3d = orbit1.orb_to_ecl() * r1_2d;
        let q = orbit2.orb_to_ecl().transpose() * r1_3d;
        let ea2 = (q.y / b2).atan2(q.x / a2 + e2);
        let p1 = Vector2::new(ea1.cos() - e1, ea1.sin());
        let p2 = Vector2::new(ea2.cos() - e2, ea2.sin());
        let r1 = a1 * (1.0 - e1 * ea1.cos());
        let r2 = a2 * (1.0 - e2 * ea2.cos());
        (ea2, r1 * r1 + r2 * r2 - p1.dot(&(m * p2)))
    };
    // Parabolic interpolation: given 3 points (x0,f0),(x1,f1),(x2,f2),
    // fit f(x) ≈ ax²+bx+c and return the minimizer x* = -b/(2a).
    let parabolic_min = |x0: f64, f0: f64, x1: f64, f1: f64, x2: f64, f2: f64| -> f64 {
        // x* = x1 - ½ [(x1-x0)²(f1-f2) - (x1-x2)²(f1-f0)]
        //            / [(x1-x0)(f1-f2) - (x1-x2)(f1-f0)]
        let d10 = x1 - x0;
        let d12 = x1 - x2;
        let num = d10 * d10 * (f1 - f2) - d12 * d12 * (f1 - f0);
        let den = d10 * (f1 - f2) - d12 * (f1 - f0);
        x1 - 0.5 * num / den
    };
    // Try both ±branches, sorted by initial f (best first).
    let mut candidates = Vec::new();
    for sign in [1.0_f64, -1.0] {
        let x0 = sign * lo.to_radians();
        let x1 = sign * ((lo + hi) / 2.0).to_radians();
        let x2 = sign * hi.to_radians();
        let (_, f0) = eval_e1(x0);
        let (_, f1) = eval_e1(x1);
        let (_, f2) = eval_e1(x2);
        let ea1_star = parabolic_min(x0, f0, x1, f1, x2, f2);
        let (ea2, f) = eval_e1(ea1_star);
        candidates.push((ea1_star, ea2, f));
    }
    candidates.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

    for (ea1_init, ea2_init, _) in &candidates {
        if let Some(result) = newton_minimize(
            *ea1_init, *ea2_init, a1, e1, a2, e2, &m, r_soi_sq,
        ) {
            return Some(result);
        }
    }

    None
}

/// Newton's method to minimize f = r₁² + r₂² - p₁ᵀMp₂, returning (E₁, E₂)
/// if the minimum distance is within r_soi.
fn newton_minimize(
    mut ea1: f64,
    mut ea2: f64,
    a1: f64,
    e1: f64,
    a2: f64,
    e2: f64,
    m: &Matrix2<f64>,
    r_soi_sq: f64,
) -> Option<(f64, f64)> {
    let mut prev_f = f64::INFINITY;

    for i in 0..MAX_ITER {
        let (sin1, cos1) = ea1.sin_cos();
        let (sin2, cos2) = ea2.sin_cos();

        let p1 = Vector2::new(cos1 - e1, sin1);
        let p2 = Vector2::new(cos2 - e2, sin2);
        let w1 = Vector2::new(-sin1, cos1);
        let w2 = Vector2::new(-sin2, cos2);
        let u1 = Vector2::new(cos1, sin1);
        let u2 = Vector2::new(cos2, sin2);

        let m_p2 = m * p2;
        let m_w2 = m * w2;
        let m_u2 = m * u2;

        let r1 = a1 * (1.0 - e1 * cos1);
        let r2 = a2 * (1.0 - e2 * cos2);
        let f = r1 * r1 + r2 * r2 - p1.dot(&m_p2);

        println!("{i:>4}  E1={ea1:>20.12}  E2={ea2:>20.12}  f={f:>16.2}  d={:>.2}", f.sqrt());

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

        let det = h11 * h22 - h12 * h12;
        let (d_e1, d_e2) = if det > TOL && h11 > 0.0 {
            ((f1 * h22 - f2 * h12) / (-det), (f2 * h11 - f1 * h12) / (-det))
        } else {
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
            return if f <= r_soi_sq {
                Some((ea1, ea2))
            } else {
                None
            };
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
        let earth: Orbit3D<EllipticOrbit> = KeplerianElements::from(earth_cfg.orbit.clone()).into();
        let r_soi = earth_cfg.soi_radius();

        println!("Earth SOI radius: {r_soi:.0} km");

        let result = find_encounter(&spacecraft, &earth, r_soi);
        println!("Result: {result:?}");
    }

    /// Compare initial guess strategies: midpoint, r₁=a₂ crossing, parabolic.
    #[test]
    fn compare_initial_guesses() {
        let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
        let config = Config::load_from_yaml(&format!("{config_dir}/config.yml")).unwrap();
        let system =
            StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

        let spacecraft: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(config.spacecraft.orbit).into();
        let traj = Trajectory::Elliptic(spacecraft.clone());

        let a1 = spacecraft.orbit_2d.0.a();
        let e1 = spacecraft.orbit_2d.0.e();
        let b1 = a1 * (1.0 - e1 * e1).sqrt();

        for name in ["Earth", "Mars", "Jupiter"] {
            let planet_cfg = system.planets.iter().find(|p| p.body.name == name).unwrap();
            let planet: Orbit3D<EllipticOrbit> =
                KeplerianElements::from(planet_cfg.orbit.clone()).into();
            let r_soi = planet_cfg.soi_radius();

            let a2 = planet.orbit_2d.0.a();
            let e2 = planet.orbit_2d.0.e();
            let b2 = a2 * (1.0 - e2 * e2).sqrt();

            let m = scaled_coupling_matrix(&spacecraft, &planet);

            let eval = |ea1: f64| -> (f64, f64) {
                let r1_2d = Vector2::new(a1 * (ea1.cos() - e1), b1 * ea1.sin());
                let r1_3d = spacecraft.orb_to_ecl() * r1_2d;
                let q = planet.orb_to_ecl().transpose() * r1_3d;
                let ea2 = (q.y / b2).atan2(q.x / a2 + e2);
                let p1 = Vector2::new(ea1.cos() - e1, ea1.sin());
                let p2 = Vector2::new(ea2.cos() - e2, ea2.sin());
                let r1 = a1 * (1.0 - e1 * ea1.cos());
                let r2 = a2 * (1.0 - e2 * ea2.cos());
                let f = r1 * r1 + r2 * r2 - p1.dot(&(m * p2));
                (ea2, f)
            };

            let ((lo, hi), _) = coarse_encounter_interval(&traj, &planet, r_soi);

            // Strategy 1: linear interpolation between endpoints (2 evals per branch)
            let mid = (lo + hi) / 2.0;
            let mut d_linear = [0.0; 2];
            for (i, sign) in [1.0_f64, -1.0].iter().enumerate() {
                let x_lo = sign * lo.to_radians();
                let x_hi = sign * hi.to_radians();
                let (_, f_lo) = eval(x_lo);
                let (_, f_hi) = eval(x_hi);
                // Weight toward the endpoint with smaller f
                let ea1_lerp = x_lo + (x_hi - x_lo) * f_lo / (f_lo + f_hi);
                let (_, f_lerp) = eval(ea1_lerp);
                d_linear[i] = f_lerp.sqrt();
            }

            // Strategy 2: r₁ = a₂ crossing (2 evals)
            let cos_e1 = ((a1 - a2) / (a1 * e1)).clamp(-1.0, 1.0);
            let ea1_cross = cos_e1.acos();
            let (_, f_cross_pos) = eval(ea1_cross);
            let (_, f_cross_neg) = eval(-ea1_cross);

            // Strategy 3: parabolic interpolation (4 evals per branch)
            let parabolic_min =
                |x0: f64, f0: f64, x1: f64, f1: f64, x2: f64, f2: f64| -> f64 {
                    let d10 = x1 - x0;
                    let d12 = x1 - x2;
                    let num = d10 * d10 * (f1 - f2) - d12 * d12 * (f1 - f0);
                    let den = d10 * (f1 - f2) - d12 * (f1 - f0);
                    x1 - 0.5 * num / den
                };
            let mut d_para = [0.0; 2];
            for (i, sign) in [1.0_f64, -1.0].iter().enumerate() {
                let x0 = sign * lo.to_radians();
                let x1 = sign * mid.to_radians();
                let x2 = sign * hi.to_radians();
                let (_, f0) = eval(x0);
                let (_, f1) = eval(x1);
                let (_, f2) = eval(x2);
                let ea1_star = parabolic_min(x0, f0, x1, f1, x2, f2);
                let (_, f_star) = eval(ea1_star);
                d_para[i] = f_star.sqrt();
            }

            // Actual minimum via Newton from parabolic guess
            let r_soi_sq = r_soi * r_soi;
            let mut d_actual = [f64::INFINITY; 2];
            for (i, sign) in [1.0_f64, -1.0].iter().enumerate() {
                let x0 = sign * lo.to_radians();
                let x1 = sign * mid.to_radians();
                let x2 = sign * hi.to_radians();
                let (_, f0) = eval(x0);
                let (_, f1) = eval(x1);
                let (_, f2) = eval(x2);
                let ea1_star = parabolic_min(x0, f0, x1, f1, x2, f2);
                let (ea2_star, _) = eval(ea1_star);
                if let Some((ea1_conv, ea2_conv)) = newton_minimize(
                    ea1_star, ea2_star, a1, e1, a2, e2, &m, f64::INFINITY,
                ) {
                    let (s1, c1) = ea1_conv.sin_cos();
                    let (s2, c2) = ea2_conv.sin_cos();
                    let p1 = Vector2::new(c1 - e1, s1);
                    let p2 = Vector2::new(c2 - e2, s2);
                    let r1 = a1 * (1.0 - e1 * c1);
                    let r2 = a2 * (1.0 - e2 * c2);
                    d_actual[i] = (r1 * r1 + r2 * r2 - p1.dot(&(m * p2))).sqrt();
                }
            }

            println!("{name:>7}:  r_soi={r_soi:.0}");
            println!(
                "         +branch:  linear d={:>12.0}  crossing d={:>12.0}  parabolic d={:>12.0}  actual d={:>12.0}",
                d_linear[0], f_cross_pos.sqrt(), d_para[0], d_actual[0]
            );
            println!(
                "         -branch:  linear d={:>12.0}  crossing d={:>12.0}  parabolic d={:>12.0}  actual d={:>12.0}",
                d_linear[1], f_cross_neg.sqrt(), d_para[1], d_actual[1]
            );
        }
    }
}
