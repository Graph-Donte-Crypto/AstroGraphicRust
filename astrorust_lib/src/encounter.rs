use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::orbit_3d::Orbit3D;
use crate::trajectory::Trajectory;
use nalgebra::{Matrix2, Vector2};

/// Coarse filter: heliocentric distance bounds
pub fn coarse_encounter_interval(
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
) -> ((f64, f64), (f64, f64)) {
    let lo = -(planet.periapsis() - spacecraft.a() - r_soi) / (spacecraft.a() * spacecraft.e());
    let hi = -(planet.apoapsis() - spacecraft.a() + r_soi) / (spacecraft.a() * spacecraft.e());
    if spacecraft.e() < 1.0 {
        let (lo, hi) =
            (lo.clamp(-1.0, 1.0).acos().to_degrees(), hi.clamp(-1.0, 1.0).acos().to_degrees());
        dbg!(&(lo, hi));
        ((lo, hi), (-lo, -hi))
    } else {
        let (lo, hi) = (lo.max(1.0).acosh(), hi.acosh());
        ((lo, hi), (-lo, -hi))
    }
}

/// Compute the scaled coupling matrix M = diag(a₁,b₁) · C · diag(a₂,b₂)
/// where C = 2 Aᵀ B encodes the mutual orientation of the two orbital planes.
fn scaled_coupling_matrix(orbit1: &Trajectory, orbit2: &Orbit3D<EllipticOrbit>) -> Matrix2<f64> {
    let c = orbit1.orb_to_ecl().transpose() * orbit2.orb_to_ecl() * 2.0;

    let a1 = orbit1.a();
    let e1 = orbit1.e();
    let b1 = a1.abs() * (1.0 - e1 * e1).abs().sqrt();

    let a2 = orbit2.orbit_2d.0.a();
    let e2 = orbit2.orbit_2d.0.e();
    let b2 = a2 * (1.0 - e2 * e2).sqrt();

    Matrix2::new(c[(0, 0)] * a1 * a2, c[(0, 1)] * a1 * b2, c[(1, 0)] * b1 * a2, c[(1, 1)] * b1 * b2)
}

const MAX_ITER: usize = 50;
const TOL: f64 = 1e-12;

/// Find anomalies (E₁/H₁, E₂) where the distance between two orbits
/// equals r_soi, using Newton's method on f = r₁² + r₂² - p₁ᵀ M p₂ - r_soi².
///
/// Returns all encounter points (one per coarse interval branch).
pub fn find_encounters(
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
) -> Vec<(f64, f64)> {
    let is_possible = spacecraft.apoapsis() + r_soi >= planet.periapsis()
        && spacecraft.periapsis() - r_soi <= planet.apoapsis();

    if !is_possible {
        return Vec::new();
    }

    let ((lo, hi), _) = coarse_encounter_interval(spacecraft, planet, r_soi);
    let a1 = spacecraft.a();
    let e1 = spacecraft.e();
    let hyp1 = e1 > 1.0;
    let a2 = planet.orbit_2d.0.a();
    let e2 = planet.orbit_2d.0.e();

    let b1 = a1.abs() * (1.0 - e1 * e1).abs().sqrt();
    let b2 = a2 * (1.0 - e2 * e2).sqrt();

    let m = scaled_coupling_matrix(spacecraft, planet);
    let r_soi_sq = r_soi * r_soi;

    // Initial guess via parabolic interpolation on the coarse interval.
    // For each ±branch, evaluate f at (lo, mid, hi), fit a quadratic,
    // and use its minimum as E₁. E₂ is estimated by projecting the
    // spacecraft position onto the planet's orbital plane.
    let eval_e1 = |ea1: f64| -> (f64, f64) {
        let (r1_2d, p1, r1) = if hyp1 {
            let (sh, ch) = (ea1.sinh(), ea1.cosh());
            (Vector2::new(a1 * (ch - e1), b1 * sh), Vector2::new(ch - e1, sh), a1 * (1.0 - e1 * ch))
        } else {
            (
                Vector2::new(a1 * (ea1.cos() - e1), b1 * ea1.sin()),
                Vector2::new(ea1.cos() - e1, ea1.sin()),
                a1 * (1.0 - e1 * ea1.cos()),
            )
        };
        let r1_3d = spacecraft.orb_to_ecl() * r1_2d;
        let q = planet.orb_to_ecl().transpose() * r1_3d;
        let ea2 = (q.y / b2).atan2(q.x / a2 + e2);
        let p2 = Vector2::new(ea2.cos() - e2, ea2.sin());
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
        let (x0, x1, x2) = if hyp1 {
            (sign * lo, sign * ((lo + hi) / 2.0), sign * hi)
        } else {
            (sign * lo.to_radians(), sign * ((lo + hi) / 2.0).to_radians(), sign * hi.to_radians())
        };
        let (_, f0) = eval_e1(x0);
        let (_, f1) = eval_e1(x1);
        let (_, f2) = eval_e1(x2);
        let ea1_star = parabolic_min(x0, f0, x1, f1, x2, f2);
        let (ea2, f) = eval_e1(ea1_star);
        candidates.push((ea1_star, ea2, f));
    }
    candidates.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

    let mut results = Vec::new();
    for (ea1_init, ea2_init, _) in &candidates {
        if let Some(result) =
            newton_minimize(*ea1_init, *ea2_init, a1, e1, a2, e2, &m, r_soi_sq, hyp1)
        {
            results.push(result);
        }
    }

    results.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
    results
}

/// Compute tight encounter intervals [E₁_min, E₁_max] and [E₂_min, E₂_max]
/// by solving {f=0, ∂f/∂E₂=0} and {f=0, ∂f/∂E₁=0} respectively.
///
/// Returns ((E₁_min, E₁_max), (E₂_min, E₂_max)) in radians, or None if
/// no encounter exists.
pub fn encounter_intervals(
    orbit1: &Trajectory,
    orbit2: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
) -> Option<((f64, f64), (f64, f64))> {
    // Quick check: does the SOI ever get entered?
    if find_encounters(orbit1, orbit2, r_soi).is_empty() {
        return None;
    }

    let a1 = orbit1.a();
    let e1 = orbit1.e();
    let hyp1 = e1 > 1.0;
    let b1 = a1.abs() * (1.0 - e1 * e1).abs().sqrt();
    let a2 = orbit2.orbit_2d.0.a();
    let e2 = orbit2.orbit_2d.0.e();
    let b2 = a2 * (1.0 - e2 * e2).sqrt();
    let m = scaled_coupling_matrix(orbit1, orbit2);
    let r_soi_sq = r_soi * r_soi;

    // Coarse interval gives two symmetric E₁ arcs: [hi_rad, lo_rad] and [-lo_rad, -hi_rad]
    // (acos reverses the inequality, so hi_deg < lo_deg but hi_rad < lo_rad).
    let ((lo_deg, hi_deg), _) = coarse_encounter_interval(orbit1, orbit2, r_soi);
    let (coarse_lo, coarse_hi) = if hyp1 {
        (lo_deg, hi_deg) // already in raw hyperbolic anomaly units
    } else {
        (lo_deg.to_radians(), hi_deg.to_radians())
    };

    // Estimate E₂ from E₁ by projecting onto the planet's orbital plane.
    let estimate_e2 = |ea1: f64| -> f64 {
        let r1_2d = if hyp1 {
            let (sh, ch) = (ea1.sinh(), ea1.cosh());
            Vector2::new(a1 * (ch - e1), b1 * sh)
        } else {
            Vector2::new(a1 * (ea1.cos() - e1), b1 * ea1.sin())
        };
        let r1_3d = orbit1.orb_to_ecl() * r1_2d;
        let q = orbit2.orb_to_ecl().transpose() * r1_3d;
        (q.y / b2).atan2(q.x / a2 + e2)
    };

    // Use the coarse interval start/end as initial guesses for both ± branches.
    // For E₁ bounds: solve {f=0, g₂=0} — the E₁ extrema of the constraint curve.
    // For E₂ bounds: solve {f=0, g₁=0} — the E₂ extrema of the constraint curve.
    // The lo endpoint should converge to E₁_min, the hi endpoint to E₁_max (and vice versa).
    let mut e1_results = Vec::new();
    let mut e2_results = Vec::new();

    for sign in [1.0_f64, -1.0] {
        for &ea1_init in &[sign * coarse_lo, sign * coarse_hi] {
            let ea2_init = estimate_e2(ea1_init);

            if let Some(pt) = newton_constraint(
                ea1_init,
                ea2_init,
                a1,
                e1,
                a2,
                e2,
                &m,
                r_soi_sq,
                ConstraintKind::E1Extremum,
                hyp1,
            ) {
                e1_results.push(pt);
            }

            if let Some(pt) = newton_constraint(
                ea1_init,
                ea2_init,
                a1,
                e1,
                a2,
                e2,
                &m,
                r_soi_sq,
                ConstraintKind::E2Extremum,
                hyp1,
            ) {
                e2_results.push(pt);
            }
        }
    }

    if e1_results.is_empty() || e2_results.is_empty() {
        return None;
    }

    let e1_min = e1_results.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let e1_max = e1_results.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
    let e2_min = e2_results.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let e2_max = e2_results.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);

    Some(((e1_min, e1_max), (e2_min, e2_max)))
}

#[derive(Clone, Copy)]
enum ConstraintKind {
    /// Solve {f=0, g₂=0} to find E₁ turning points
    E1Extremum,
    /// Solve {f=0, g₁=0} to find E₂ turning points
    E2Extremum,
}

/// Newton's method on the 2×2 system for encounter interval bounds.
///
/// For E₁ extrema: solves {f=0, g₂=0} with Jacobian [[g₁, g₂], [H₁₂, H₂₂]]
/// For E₂ extrema: solves {f=0, g₁=0} with Jacobian [[g₁, g₂], [H₁₁, H₁₂]]
fn newton_constraint(
    mut ea1: f64,
    mut ea2: f64,
    a1: f64,
    e1: f64,
    a2: f64,
    e2: f64,
    m: &Matrix2<f64>,
    r_soi_sq: f64,
    kind: ConstraintKind,
    hyp1: bool,
) -> Option<(f64, f64)> {
    for _ in 0..MAX_ITER {
        let (sin1, cos1, p1, w1, u1, cross_sign) = if hyp1 {
            let (sh, ch) = (ea1.sinh(), ea1.cosh());
            (
                sh,
                ch,
                Vector2::new(ch - e1, sh),
                Vector2::new(sh, ch), // ŵ₁ = (sinh H, cosh H)
                Vector2::new(ch, sh), // û₁ = (cosh H, sinh H)
                -1.0_f64,
            ) // cross-term sign flip
        } else {
            let (s, c) = ea1.sin_cos();
            (
                s,
                c,
                Vector2::new(c - e1, s),
                Vector2::new(-s, c), // ŵ₁ = (-sin E, cos E)
                Vector2::new(c, s),  // û₁ = (cos E, sin E)
                1.0_f64,
            )
        };
        let (sin2, cos2) = ea2.sin_cos();

        let p2 = Vector2::new(cos2 - e2, sin2);
        let w2 = Vector2::new(-sin2, cos2);
        let u2 = Vector2::new(cos2, sin2);

        let m_p2 = m * p2;
        let m_w2 = m * w2;
        let m_u2 = m * u2;

        let r1 = a1 * (1.0 - e1 * cos1);
        let r2 = a2 * (1.0 - e2 * cos2);
        let f = r1 * r1 + r2 * r2 - p1.dot(&m_p2) - r_soi_sq;

        // Gradient
        let self_grad1 = if hyp1 {
            -2.0 * a1 * a1 * e1 * sin1 * (1.0 - e1 * cos1)
        } else {
            2.0 * a1 * a1 * e1 * sin1 * (1.0 - e1 * cos1)
        };
        let g1 = self_grad1 - w1.dot(&m_p2);
        let g2 = 2.0 * a2 * a2 * e2 * sin2 * (1.0 - e2 * cos2) - p1.dot(&m_w2);

        // Hessian
        let self_hess1 = if hyp1 {
            2.0 * a1 * a1 * e1 * (e1 - cos1 + 2.0 * e1 * sin1 * sin1)
        } else {
            2.0 * a1 * a1 * e1 * (cos1 - e1 + 2.0 * e1 * sin1 * sin1)
        };
        let h11 = self_hess1 + cross_sign * u1.dot(&m_p2);
        let h22 = 2.0 * a2 * a2 * e2 * (cos2 - e2 + 2.0 * e2 * sin2 * sin2) + p1.dot(&m_u2);
        let h12 = -w1.dot(&m_w2);

        let (d_e1, d_e2) = match kind {
            // {f=0, g₂=0}: J = [[g₁, g₂], [H₁₂, H₂₂]]
            // Cramer: Δ = g₁·H₂₂ - g₂·H₁₂
            ConstraintKind::E1Extremum => {
                let det = g1 * h22 - g2 * h12;
                if det.abs() < TOL {
                    return None;
                }
                (-(f * h22 - g2 * g2) / det, -(g2 * g1 - f * h12) / det)
            }
            // {f=0, g₁=0}: J = [[g₁, g₂], [H₁₁, H₁₂]]
            // Cramer: Δ = g₁·H₁₂ - g₂·H₁₁
            ConstraintKind::E2Extremum => {
                let det = g1 * h12 - g2 * h11;
                if det.abs() < TOL {
                    return None;
                }
                (-(f * h12 - g1 * g2) / det, -(g1 * g1 - f * h11) / det)
            }
        };

        ea1 += d_e1;
        ea2 += d_e2;

        if d_e1.abs() < TOL && d_e2.abs() < TOL {
            // Verify f ≈ 0 (distance equals r_soi).
            if f.abs() < r_soi_sq * 1e-6 {
                // Normalize E₂ to [-π, π]. Only normalize E₁ for elliptic.
                if !hyp1 {
                    ea1 = (ea1 + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                        - std::f64::consts::PI;
                }
                ea2 = (ea2 + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                    - std::f64::consts::PI;
                return Some((ea1, ea2));
            }
            return None;
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
    hyp1: bool,
) -> Option<(f64, f64)> {
    let mut prev_f = f64::INFINITY;

    for i in 0..MAX_ITER {
        let (sin1, cos1, p1, w1, u1, cross_sign) = if hyp1 {
            let (sh, ch) = (ea1.sinh(), ea1.cosh());
            (
                sh,
                ch,
                Vector2::new(ch - e1, sh),
                Vector2::new(sh, ch), // ŵ₁ = (sinh H, cosh H)
                Vector2::new(ch, sh), // û₁ = (cosh H, sinh H)
                -1.0_f64,
            ) // cross-term sign flip
        } else {
            let (s, c) = ea1.sin_cos();
            (
                s,
                c,
                Vector2::new(c - e1, s),
                Vector2::new(-s, c), // ŵ₁ = (-sin E, cos E)
                Vector2::new(c, s),  // û₁ = (cos E, sin E)
                1.0_f64,
            )
        };
        let (sin2, cos2) = ea2.sin_cos();

        let p2 = Vector2::new(cos2 - e2, sin2);
        let w2 = Vector2::new(-sin2, cos2);
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
        let self_grad1 = if hyp1 {
            -2.0 * a1 * a1 * e1 * sin1 * (1.0 - e1 * cos1)
        } else {
            2.0 * a1 * a1 * e1 * sin1 * (1.0 - e1 * cos1)
        };
        let self_grad2 = 2.0 * a2 * a2 * e2 * sin2 * (1.0 - e2 * cos2);
        let f1 = self_grad1 - w1.dot(&m_p2);
        let f2 = self_grad2 - p1.dot(&m_w2);

        // Hessian
        let self_hess1 = if hyp1 {
            2.0 * a1 * a1 * e1 * (e1 - cos1 + 2.0 * e1 * sin1 * sin1)
        } else {
            2.0 * a1 * a1 * e1 * (cos1 - e1 + 2.0 * e1 * sin1 * sin1)
        };
        let self_hess2 = 2.0 * a2 * a2 * e2 * (cos2 - e2 + 2.0 * e2 * sin2 * sin2);
        let h11 = self_hess1 + cross_sign * u1.dot(&m_p2);
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
            return if f <= r_soi_sq { Some((ea1, ea2)) } else { None };
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Config, StarSystem};
    use crate::orbit::flat::hyperbolic::HyperbolicOrbit;
    use crate::orbit::orbit_3d::KeplerianElements;

    #[test]
    fn encounter_voyager2_jupiter() {
        let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
        let config = Config::load_from_yaml(&format!("{config_dir}/config.yml")).unwrap();
        let system = StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

        let spacecraft: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(config.spacecraft.orbit).into();
        let traj = Trajectory::Elliptic(spacecraft);

        let jupiter_cfg = system.planets.iter().find(|p| p.body.name == "Jupiter").unwrap();
        let jupiter: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(jupiter_cfg.orbit.clone()).into();
        let r_soi = jupiter_cfg.soi_radius();

        println!("Jupiter SOI radius: {r_soi:.0} km");

        let results = find_encounters(&traj, &jupiter, r_soi);
        println!("Results: {results:?}");
        assert!(!results.is_empty(), "Newton's method did not converge");
    }

    #[test]
    fn encounter_voyager2_earth() {
        let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
        let config = Config::load_from_yaml(&format!("{config_dir}/config.yml")).unwrap();
        let system = StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

        let spacecraft: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(config.spacecraft.orbit).into();
        let traj = Trajectory::Elliptic(spacecraft);

        let earth_cfg = system.planets.iter().find(|p| p.body.name == "Earth").unwrap();
        let earth: Orbit3D<EllipticOrbit> = KeplerianElements::from(earth_cfg.orbit.clone()).into();
        let r_soi = earth_cfg.soi_radius();

        println!("Earth SOI radius: {r_soi:.0} km");

        let results = find_encounters(&traj, &earth, r_soi);
        println!("Results: {results:?}");
    }

    /// Compare initial guess strategies: midpoint, r₁=a₂ crossing, parabolic.
    #[test]
    fn compare_initial_guesses() {
        let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
        let config = Config::load_from_yaml(&format!("{config_dir}/config.yml")).unwrap();
        let system = StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

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

            let m = scaled_coupling_matrix(&traj, &planet);

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
            let parabolic_min = |x0: f64, f0: f64, x1: f64, f1: f64, x2: f64, f2: f64| -> f64 {
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
                if let Some((ea1_conv, ea2_conv)) =
                    newton_minimize(ea1_star, ea2_star, a1, e1, a2, e2, &m, f64::INFINITY, false)
                {
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
                d_linear[0],
                f_cross_pos.sqrt(),
                d_para[0],
                d_actual[0]
            );
            println!(
                "         -branch:  linear d={:>12.0}  crossing d={:>12.0}  parabolic d={:>12.0}  actual d={:>12.0}",
                d_linear[1],
                f_cross_neg.sqrt(),
                d_para[1],
                d_actual[1]
            );
        }
    }

    #[test]
    fn encounter_intervals_vs_coarse() {
        let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
        let config = Config::load_from_yaml(&format!("{config_dir}/config.yml")).unwrap();
        let system = StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

        let spacecraft: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(config.spacecraft.orbit).into();
        let traj = Trajectory::Elliptic(spacecraft.clone());

        for name in ["Earth", "Mars", "Jupiter"] {
            let planet_cfg = system.planets.iter().find(|p| p.body.name == name).unwrap();
            let planet: Orbit3D<EllipticOrbit> =
                KeplerianElements::from(planet_cfg.orbit.clone()).into();
            let r_soi = planet_cfg.soi_radius();

            let ((coarse_lo_deg, coarse_hi_deg), _) =
                coarse_encounter_interval(&traj, &planet, r_soi);
            // acos reverses the inequality, so coarse_hi_deg > coarse_lo_deg
            // but coarse_hi_rad > coarse_lo_rad (larger angle = larger rad).
            let coarse_min_rad = coarse_lo_deg.to_radians(); // smaller angle
            let coarse_max_rad = coarse_hi_deg.to_radians(); // larger angle

            println!("\n{name}:  r_soi = {r_soi:.0}");
            println!(
                "  coarse E₁: [{coarse_min_rad:.4}, {coarse_max_rad:.4}] rad  ({coarse_lo_deg:.2}°, {coarse_hi_deg:.2}°)"
            );

            let result = encounter_intervals(&traj, &planet, r_soi);
            match result {
                Some(((e1_min, e1_max), (e2_min, e2_max))) => {
                    println!(
                        "  tight  E₁: [{e1_min:.4}, {e1_max:.4}] rad  ({:.2}°, {:.2}°)",
                        e1_min.to_degrees(),
                        e1_max.to_degrees()
                    );
                    println!(
                        "  tight  E₂: [{e2_min:.4}, {e2_max:.4}] rad  ({:.2}°, {:.2}°)",
                        e2_min.to_degrees(),
                        e2_max.to_degrees()
                    );

                    let coarse_width = coarse_max_rad - coarse_min_rad;
                    let tight_width = e1_max - e1_min;
                    let reduction = (1.0 - tight_width / coarse_width) * 100.0;
                    println!(
                        "  E₁ reduction: {coarse_width:.4} → {tight_width:.4} rad  ({reduction:.1}% narrower)"
                    );

                    // Tight E₁ bounds must be inside the coarse bounds.
                    assert!(
                        e1_min >= coarse_min_rad - 0.01,
                        "{name}: tight E₁_min {e1_min:.4} < coarse min {coarse_min_rad:.4}"
                    );
                    assert!(
                        e1_max <= coarse_max_rad + 0.01,
                        "{name}: tight E₁_max {e1_max:.4} > coarse max {coarse_max_rad:.4}"
                    );

                    // Tight interval must be non-empty
                    assert!(e1_min < e1_max, "{name}: E₁ interval empty");
                    assert!(e2_min < e2_max, "{name}: E₂ interval empty");
                }
                None => {
                    println!("  tight: no encounter found");
                }
            }
        }
    }

    #[test]
    fn encounter_hyperbolic_saturn() {
        let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
        let system = StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

        // Voyager 2 Jupiter-to-Saturn hyperbolic orbit
        let spacecraft: Orbit3D<HyperbolicOrbit> = KeplerianElements {
            mu: system.star.μ,
            a: -2220315000.0,
            e: 1.338264,
            i: 2.582320_f64.to_radians(),
            Omega: 119.196938_f64.to_radians(),
            omega: (-9.170896_f64).to_radians(),
            M0: 1.65,
        }
        .into();
        let traj = Trajectory::Hyperbolic(spacecraft);

        let saturn_cfg = system.planets.iter().find(|p| p.body.name == "Saturn").unwrap();
        let saturn: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(saturn_cfg.orbit.clone()).into();
        let r_soi = saturn_cfg.soi_radius();

        println!("Saturn SOI radius: {r_soi:.0} km");

        let results = find_encounters(&traj, &saturn, r_soi);
        println!("Results: {results:?}");
        assert!(!results.is_empty(), "Newton's method did not converge for hyperbolic encounter");
    }
}
