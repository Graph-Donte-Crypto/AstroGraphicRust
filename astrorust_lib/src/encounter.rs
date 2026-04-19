use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::orbit_3d::Orbit3D;
use crate::trajectory::Trajectory;
use crate::util::solve_quadratic;
use nalgebra::{Matrix2, Vector2, Vector3};

const MAX_ITER: usize = 50;
const TOL: f64 = 1e-12;

/// Compute initial-guess anomalies from radial overlap. `E₁` (or `H₁`) is
/// taken as the midpoint of the coarse interval where the spacecraft's
/// heliocentric distance `r₁(E₁)` overlaps the planet's radial shell
/// `[r_peri − r_soi, r_apo + r_soi]`. `E₂` is recovered by projecting the
/// spacecraft's 3D position at that anomaly onto the planet's orbital plane.
///
/// Returns the two same-side pairings `[(E₁, E₂), (−E₁, −E₂)]` for the two
/// arcs symmetric about periapsis/apoapsis. All anomalies are in radians.
pub fn radial_initial_guesses(
    craft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
) -> [(f64, f64); 2] {
    let lo = -(planet.periapsis() - craft.a() - r_soi) / (craft.a() * craft.e());
    let hi = -(planet.apoapsis() - craft.a() + r_soi) / (craft.a() * craft.e());
    let (lo, hi) = if craft.is_hyperbolic() {
        (lo.max(1.0).acosh(), hi.acosh())
    } else {
        (lo.clamp(-1.0, 1.0).acos(), hi.clamp(-1.0, 1.0).acos())
    };
    let E1_mid = 0.5 * (lo + hi);
    let project_E2 = |E1: f64| -> f64 {
        let r1_2d = if craft.is_hyperbolic() {
            Vector2::new(craft.a() * (E1.cosh() - craft.e()), craft.b() * E1.sinh())
        } else {
            Vector2::new(craft.a() * (E1.cos() - craft.e()), craft.b() * E1.sin())
        };
        let r1_3d = craft.orb_to_ecl() * r1_2d;
        let q = planet.orb_to_ecl().transpose() * r1_3d;
        (q.y / planet.b()).atan2(q.x / planet.a() + planet.e())
    };
    [(E1_mid, project_E2(E1_mid)), (-E1_mid, project_E2(-E1_mid))]
}

/// Compute initial-guess anomalies from the mutual line of nodes of the two
/// orbital planes. Returns the two same-side pairings:
/// `[(E₁_asc, E₂_asc), (E₁_desc, E₂_desc)]` — i.e. both bodies on the `+ℓ`
/// ray and both on the `−ℓ` ray, where `ℓ = n̂₁ × n̂₂`.
///
/// The spacecraft may be elliptic or hyperbolic; the planet must be elliptic.
/// Hyperbolic spacecraft anomalies are returned as raw `H₁` (no wrapping).
///
/// Returns `None` when the per-orbit nodal-ray equation has no real root —
/// in practice when the orbits are too nearly coplanar for the mutual-node
/// line to be well-defined, or when the hyperbolic branch geometry rules
/// out a `+ℓ`/`−ℓ` crossing. Callers should fall back to the radial-overlap
/// + midpoint initial guess in that case.
///
/// This is an alternative to the radial-overlap + midpoint initial guess.
/// It is typically much closer to the true encounter for non-negligible
/// mutual inclinations because the minimum of the 3D separation sits near
/// the mutual-node line by geometry.
pub fn nodal_initial_guesses(
    craft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
) -> Option<[(f64, f64); 2]> {
    let A_matrix = craft.orb_to_ecl();
    let B_matrix = planet.orb_to_ecl();

    let n1 = A_matrix.column(0).cross(&A_matrix.column(1));
    let n2 = B_matrix.column(0).cross(&B_matrix.column(1));
    let nodal_vec = n1.cross(&n2);

    let (E1_asc, E1_desc) = intersect_nodal_line(craft, &nodal_vec)?;
    let (E2_asc, E2_desc) = intersect_nodal_line(&planet.clone().into(), &nodal_vec)?;
    Some([(E1_asc, E2_asc), (E1_desc, E2_desc)])
}

/// Intersect the mutual nodal line (3D `nodal_vec`, in ecliptic coords) with
/// the trajectory's focus-centered conic `(x/a + e)² ± (y/b)² = 1` (+ ellipse,
/// − hyperbola). The 3D vector is first projected into the trajectory's
/// orbital plane via its `orb_to_ecl` basis. Returns the two anomalies
/// `(plus, minus)` for the `+nodal_vec` and `−nodal_vec` sides, or `None`
/// when the line misses the conic (or its physical branch, for hyperbolic
/// trajectories).
fn intersect_nodal_line(trajectory: &Trajectory, nodal_vec: &Vector3<f64>) -> Option<(f64, f64)> {
    let nodal_in_plane = trajectory.orb_to_ecl().transpose() * nodal_vec;
    let (a, b, e) = (trajectory.a(), trajectory.b(), trajectory.e());
    let x_sq = (nodal_in_plane.x / a).powi(2);
    let y_sq = (nodal_in_plane.y / b).powi(2);
    let (t_plus, t_minus) = solve_quadratic(
        if trajectory.is_hyperbolic() { x_sq - y_sq } else { x_sq + y_sq },
        2.0 * e * nodal_in_plane.x / a,
        e * e - 1.0,
    )?;
    if trajectory.is_hyperbolic() {
        // Physical hyperbolic branch requires cosh H ≥ 1.
        let H_from_t = |t: f64| -> Option<f64> {
            let cosh_h = t * nodal_in_plane.x / a + e;
            (cosh_h >= 1.0).then(|| (t * nodal_in_plane.y / b).asinh())
        };
        Some((H_from_t(t_plus)?, H_from_t(t_minus)?))
    } else {
        let E_from_t = |t: f64| (t * nodal_in_plane.y / b).atan2(t * nodal_in_plane.x / a + e);
        Some((E_from_t(t_plus), E_from_t(t_minus)))
    }
}

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

    let a1 = spacecraft.a();
    let e1 = spacecraft.e();
    let hyp1 = spacecraft.is_hyperbolic();
    let a2 = planet.a();
    let e2 = planet.e();

    let M = scaled_coupling_matrix(spacecraft, planet);
    let r_soi_sq = r_soi * r_soi;

    let f_at = |E1: f64, E2: f64| -> f64 {
        let (p1, r1) = if hyp1 {
            let (sh, ch) = (E1.sinh(), E1.cosh());
            (Vector2::new(ch - e1, sh), a1 * (1.0 - e1 * ch))
        } else {
            let (s, c) = E1.sin_cos();
            (Vector2::new(c - e1, s), a1 * (1.0 - e1 * c))
        };
        let p2 = Vector2::new(E2.cos() - e2, E2.sin());
        let r2 = a2 * (1.0 - e2 * E2.cos());
        r1 * r1 + r2 * r2 - p1.dot(&(M * p2))
    };

    // Seed Newton with both ±branches of the radial-overlap midpoint guess,
    // sorted by initial f (best first).
    let mut candidates: Vec<(f64, f64, f64)> = radial_initial_guesses(spacecraft, planet, r_soi)
        .iter()
        .map(|&(E1, E2)| (E1, E2, f_at(E1, E2)))
        .collect();
    eprintln!(
        "[find_encounters] r_soi_sq={r_soi_sq:.3e}  raw seeds: {:?}",
        candidates.iter().map(|c| (c.0, c.1, c.2)).collect::<Vec<_>>()
    );
    candidates.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
    eprintln!(
        "[find_encounters] sorted seeds (best first): {:?}",
        candidates.iter().map(|c| (c.0, c.1, c.2)).collect::<Vec<_>>()
    );

    let mut results = Vec::new();
    for (E1_init, E2_init, f_init) in &candidates {
        eprintln!(
            "[find_encounters] Newton seed E1={E1_init:.6} E2={E2_init:.6} f_init={f_init:.3e}"
        );
        match newton_minimize(*E1_init, *E2_init, a1, e1, a2, e2, &M, r_soi_sq, hyp1) {
            Some(result) => {
                eprintln!("[find_encounters]   -> CONVERGED inside r_soi: {result:?}");
                results.push(result);
            }
            None => eprintln!("[find_encounters]   -> rejected (None)"),
        }
    }

    results.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
    results
}

/// Newton's method to minimize f = r₁² + r₂² - p₁ᵀMp₂, returning (E₁, E₂)
/// if the minimum distance is within r_soi.
fn newton_minimize(
    mut E1: f64,
    mut E2: f64,
    a1: f64,
    e1: f64,
    a2: f64,
    e2: f64,
    M: &Matrix2<f64>,
    r_soi_sq: f64,
    hyp1: bool,
) -> Option<(f64, f64)> {
    let mut prev_f = f64::INFINITY;

    for i in 0..MAX_ITER {
        let (sin1, cos1, p1, w1, u1, cross_sign) = if hyp1 {
            let (sh, ch) = (E1.sinh(), E1.cosh());
            (
                sh,
                ch,
                Vector2::new(ch - e1, sh),
                Vector2::new(sh, ch), // ŵ₁ = (sinh H, cosh H)
                Vector2::new(ch, sh), // û₁ = (cosh H, sinh H)
                -1.0_f64,
            ) // cross-term sign flip
        } else {
            let (s, c) = E1.sin_cos();
            (
                s,
                c,
                Vector2::new(c - e1, s),
                Vector2::new(-s, c), // ŵ₁ = (-sin E, cos E)
                Vector2::new(c, s),  // û₁ = (cos E, sin E)
                1.0_f64,
            )
        };
        let (sin2, cos2) = E2.sin_cos();

        let p2 = Vector2::new(cos2 - e2, sin2);
        let w2 = Vector2::new(-sin2, cos2);
        let u2 = Vector2::new(cos2, sin2);

        let M_p2 = M * p2;
        let M_w2 = M * w2;
        let M_u2 = M * u2;

        let r1 = a1 * (1.0 - e1 * cos1);
        let r2 = a2 * (1.0 - e2 * cos2);
        let f = r1 * r1 + r2 * r2 - p1.dot(&M_p2);

        println!("{i:>4}  E1={E1:>20.12}  E2={E2:>20.12}  f={f:>16.2}  d={:>.2}", f.sqrt());

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
        let f1 = self_grad1 - w1.dot(&M_p2);
        let f2 = self_grad2 - p1.dot(&M_w2);

        // Hessian
        let self_hess1 = if hyp1 {
            2.0 * a1 * a1 * e1 * (e1 - cos1 + 2.0 * e1 * sin1 * sin1)
        } else {
            2.0 * a1 * a1 * e1 * (cos1 - e1 + 2.0 * e1 * sin1 * sin1)
        };
        let self_hess2 = 2.0 * a2 * a2 * e2 * (cos2 - e2 + 2.0 * e2 * sin2 * sin2);
        let h11 = self_hess1 + cross_sign * u1.dot(&M_p2);
        let h22 = self_hess2 + p1.dot(&M_u2);
        let h12 = -w1.dot(&M_w2);

        let det = h11 * h22 - h12 * h12;
        let (dE1, dE2) = if det > TOL && h11 > 0.0 {
            ((f1 * h22 - f2 * h12) / (-det), (f2 * h11 - f1 * h12) / (-det))
        } else {
            let grad_norm_sq = f1 * f1 + f2 * f2;
            if grad_norm_sq <= f64::EPSILON {
                return None;
            }
            let alpha = f / grad_norm_sq;
            (-alpha * f1, -alpha * f2)
        };

        E1 += dE1;
        E2 += dE2;

        if dE1.abs() < TOL && dE2.abs() < TOL {
            return if f <= r_soi_sq { Some((E1, E2)) } else { None };
        }
    }

    None
}

/// Compute the scaled coupling matrix M = diag(a₁,b₁) · C · diag(a₂,b₂)
/// where C = 2 Aᵀ B encodes the mutual orientation of the two orbital planes.
fn scaled_coupling_matrix(orbit1: &Trajectory, orbit2: &Orbit3D<EllipticOrbit>) -> Matrix2<f64> {
    let C = 2.0 * orbit1.orb_to_ecl().transpose() * orbit2.orb_to_ecl();
    let D = Vector2::new(orbit1.a(), orbit1.b()) * Vector2::new(orbit2.a(), orbit2.b()).transpose();
    C.component_mul(&D)
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

    /// Compare radial-overlap + midpoint initial guess against mutual-nodal guess.
    #[test]
    fn compare_nodal_vs_midpoint_initial_guesses() {
        let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
        let config = Config::load_from_yaml(&format!("{config_dir}/config.yml")).unwrap();
        let system = StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

        let elliptic_sc: Orbit3D<EllipticOrbit> =
            KeplerianElements::from(config.spacecraft.orbit).into();
        let elliptic_traj = Trajectory::Elliptic(elliptic_sc);

        // Voyager 2 Jupiter-to-Saturn hyperbolic orbit (same elements as
        // encounter_hyperbolic_saturn test).
        let hyperbolic_sc: Orbit3D<HyperbolicOrbit> = KeplerianElements {
            mu: system.star.μ,
            a: -2220315000.0,
            e: 1.338264,
            i: 2.582320_f64.to_radians(),
            Omega: 119.196938_f64.to_radians(),
            omega: (-9.170896_f64).to_radians(),
            M0: 1.65,
        }
        .into();
        let hyperbolic_traj = Trajectory::Hyperbolic(hyperbolic_sc);

        let pairs: Vec<(&Trajectory, &[&str])> =
            vec![(&elliptic_traj, &["Earth", "Mars", "Jupiter"]), (&hyperbolic_traj, &["Saturn"])];

        for (traj, names) in pairs {
            let hyp1 = traj.e() > 1.0;
            for name in names {
                let planet_cfg = system.planets.iter().find(|p| p.body.name == *name).unwrap();
                let planet: Orbit3D<EllipticOrbit> =
                    KeplerianElements::from(planet_cfg.orbit.clone()).into();
                let r_soi = planet_cfg.soi_radius();

                let (a1, e1) = (traj.a(), traj.e());
                let (a2, e2) = (planet.a(), planet.e());
                let M = scaled_coupling_matrix(traj, &planet);

                let p1_vec = |E1: f64| -> Vector2<f64> {
                    if hyp1 {
                        Vector2::new(E1.cosh() - e1, E1.sinh())
                    } else {
                        Vector2::new(E1.cos() - e1, E1.sin())
                    }
                };
                let r1_scalar = |E1: f64| -> f64 {
                    if hyp1 { a1 * (1.0 - e1 * E1.cosh()) } else { a1 * (1.0 - e1 * E1.cos()) }
                };

                // Absolute 3D distance at a given (E₁/H₁, E₂).
                let distance = |E1: f64, E2: f64| -> f64 {
                    let (s2, c2) = E2.sin_cos();
                    let p1 = p1_vec(E1);
                    let p2 = Vector2::new(c2 - e2, s2);
                    let r1 = r1_scalar(E1);
                    let r2 = a2 * (1.0 - e2 * c2);
                    (r1 * r1 + r2 * r2 - p1.dot(&(M * p2))).max(0.0).sqrt()
                };

                // --- Midpoint guess: better of the two ±branches ---
                let mut best_mid = (f64::NAN, f64::NAN, f64::INFINITY);
                for (E1, E2) in radial_initial_guesses(traj, &planet, r_soi) {
                    let d = distance(E1, E2);
                    if d < best_mid.2 {
                        best_mid = (E1, E2, d);
                    }
                }

                // --- Nodal guess: best of ±ℓ same-side pairings ---
                let best_nodal = nodal_initial_guesses(traj, &planet).map(|pairs| {
                    pairs
                        .iter()
                        .map(|&(e1g, e2g)| (e1g, e2g, distance(e1g, e2g)))
                        .min_by(|a, b| a.2.partial_cmp(&b.2).unwrap())
                        .unwrap()
                });

                // --- True minimum via Newton on the better seed ---
                let seed = match best_nodal {
                    Some(n) if n.2 < best_mid.2 => (n.0, n.1),
                    _ => (best_mid.0, best_mid.1),
                };
                let (mut E1, mut E2) = seed;
                for _ in 0..200 {
                    let (sin1, cos1, p1, w1, u1, cross_sign) = if hyp1 {
                        let (sh, ch) = (E1.sinh(), E1.cosh());
                        (
                            sh,
                            ch,
                            Vector2::new(ch - e1, sh),
                            Vector2::new(sh, ch),
                            Vector2::new(ch, sh),
                            -1.0_f64,
                        )
                    } else {
                        let (s, c) = E1.sin_cos();
                        (
                            s,
                            c,
                            Vector2::new(c - e1, s),
                            Vector2::new(-s, c),
                            Vector2::new(c, s),
                            1.0_f64,
                        )
                    };
                    let (sin2, cos2) = E2.sin_cos();
                    let p2v = Vector2::new(cos2 - e2, sin2);
                    let w2 = Vector2::new(-sin2, cos2);
                    let u2 = Vector2::new(cos2, sin2);
                    let M_p2 = M * p2v;
                    let M_w2 = M * w2;
                    let M_u2 = M * u2;
                    let r1_s = if hyp1 { a1 * (1.0 - e1 * cos1) } else { a1 * (1.0 - e1 * cos1) };
                    let self_g1 = if hyp1 {
                        -2.0 * a1 * a1 * e1 * sin1 * (1.0 - e1 * cos1)
                    } else {
                        2.0 * a1 * a1 * e1 * sin1 * (1.0 - e1 * cos1)
                    };
                    let g1 = self_g1 - w1.dot(&M_p2);
                    let g2 = 2.0 * a2 * a2 * e2 * sin2 * (1.0 - e2 * cos2) - p1.dot(&M_w2);
                    let self_h11 = if hyp1 {
                        2.0 * a1 * a1 * e1 * (e1 - cos1 + 2.0 * e1 * sin1 * sin1)
                    } else {
                        2.0 * a1 * a1 * e1 * (cos1 - e1 + 2.0 * e1 * sin1 * sin1)
                    };
                    let h11 = self_h11 + cross_sign * u1.dot(&M_p2);
                    let h22 =
                        2.0 * a2 * a2 * e2 * (cos2 - e2 + 2.0 * e2 * sin2 * sin2) + p1.dot(&M_u2);
                    let h12 = -w1.dot(&M_w2);
                    let det = h11 * h22 - h12 * h12;
                    let dE1 = (g1 * h22 - g2 * h12) / (-det);
                    let dE2 = (g2 * h11 - g1 * h12) / (-det);
                    E1 += dE1;
                    E2 += dE2;
                    let _ = r1_s;
                    if dE1.abs() < 1e-14 && dE2.abs() < 1e-14 {
                        break;
                    }
                }
                let (E1_true, E2_true, d_true) = (E1, E2, distance(E1, E2));
                let wrap_pi = |x: f64| {
                    (x + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                        - std::f64::consts::PI
                };
                // Hyperbolic H has no periodicity — don't wrap.
                let sc_diff = |x: f64| if hyp1 { x } else { wrap_pi(x) };

                // Mutual inclination.
                let a_c0 = traj.orb_to_ecl() * Vector2::new(1.0, 0.0);
                let a_c1 = traj.orb_to_ecl() * Vector2::new(0.0, 1.0);
                let b_c0 = planet.orb_to_ecl() * Vector2::new(1.0, 0.0);
                let b_c1 = planet.orb_to_ecl() * Vector2::new(0.0, 1.0);
                let sin_i = a_c0.cross(&a_c1).cross(&b_c0.cross(&b_c1)).norm();
                let i_mut_deg = sin_i.asin().to_degrees();
                let label = if hyp1 { "H1" } else { "E1" };

                println!(
                    "\n{name}:  r_soi = {r_soi:.0} km,  i_mut = {i_mut_deg:.3}°{}",
                    if hyp1 { ",  hyperbolic spacecraft" } else { "" }
                );
                println!(
                    "  true:  {label}={:>8.4} rad  E2={:>8.4} rad  d = {:.3e} km ({:.3} r_soi)",
                    E1_true,
                    E2_true,
                    d_true,
                    d_true / r_soi
                );
                println!(
                    "  mid:   {label}={:>8.4} rad  E2={:>8.4} rad  d = {:.3e} km ({:.3} r_soi)  Δ{label}={:+.4} rad  ΔE2={:+.4} rad",
                    best_mid.0,
                    best_mid.1,
                    best_mid.2,
                    best_mid.2 / r_soi,
                    sc_diff(best_mid.0 - E1_true),
                    wrap_pi(best_mid.1 - E2_true),
                );
                match best_nodal {
                    Some(n) => {
                        println!(
                            "  nodal: {label}={:>8.4} rad  E2={:>8.4} rad  d = {:.3e} km ({:.3} r_soi)  Δ{label}={:+.4} rad  ΔE2={:+.4} rad",
                            n.0,
                            n.1,
                            n.2,
                            n.2 / r_soi,
                            sc_diff(n.0 - E1_true),
                            wrap_pi(n.1 - E2_true),
                        );
                        println!("  ratio d_mid / d_nodal = {:.3}x", best_mid.2 / n.2);
                    }
                    None => println!("  nodal: skipped (coplanar or no real ℓ crossing)"),
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
