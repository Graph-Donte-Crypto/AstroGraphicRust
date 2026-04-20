use crate::angle::{Angle, EccAnomaly, HypAnomaly};
use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::orbit_3d::Orbit3D;
use crate::state_vectors::StateVectors;
use crate::trajectory::Trajectory;
use crate::util::{FloatExt, format_with_thousand_separators, solve_quadratic};
use nalgebra::{Matrix2, Vector2, Vector3};

const MAX_ITER: usize = 50;
const TOL: f64 = 1e-12;

/// 3D ecliptic position of the spacecraft at its natural anomaly
/// (eccentric `E` for elliptic, hyperbolic `H` for hyperbolic).
fn craft_position(craft: &Trajectory, anomaly: f64) -> Vector3<f64> {
    match craft {
        Trajectory::Elliptic(o) => o.position(EccAnomaly::from(Angle::from_rad(anomaly))),
        Trajectory::Hyperbolic(o) => o.position(HypAnomaly::from(anomaly)),
    }
}

/// Squared 3D distance between spacecraft and planet at the given anomalies.
pub fn distance_sq(
    craft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    E1: f64,
    E2: f64,
) -> f64 {
    (craft_position(craft, E1) - planet.position(EccAnomaly::from(Angle::from_rad(E2))))
        .norm_squared()
}

/// Analytic gradient and Hessian of f(E₁, E₂) = r₁² + r₂² − p₁ᵀMp₂ at the given
/// anomalies. Shared between the Levenberg-Marquardt minimiser and the
/// encounter-refinement stage, which needs H at the geometric minimum to build
/// the Taylor ellipse in (M₁, M₂).
#[inline]
pub fn gradient_hessian_at(
    E1: f64,
    E2: f64,
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    coupling: &Matrix2<f64>,
) -> (Vector2<f64>, Matrix2<f64>) {
    let (a1, e1, a2, e2) = (spacecraft.a(), spacecraft.e(), planet.a(), planet.e());
    let hyp1 = spacecraft.is_hyperbolic();
    let (sin1, cos1, p1, w1, u1, cross_sign) = if hyp1 {
        let (sh, ch) = E1.sinh_cosh();
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
        (s, c, Vector2::new(c - e1, s), Vector2::new(-s, c), Vector2::new(c, s), 1.0_f64)
    };
    let (sin2, cos2) = E2.sin_cos();
    let p2 = Vector2::new(cos2 - e2, sin2);
    let w2 = Vector2::new(-sin2, cos2);
    let u2 = Vector2::new(cos2, sin2);
    let M_p2 = coupling * p2;
    let M_w2 = coupling * w2;
    let M_u2 = coupling * u2;

    let self_g1 = 2.0 * a1 * a1 * e1 * sin1 * (1.0 - e1 * cos1);
    let self_g1 = if hyp1 { -self_g1 } else { self_g1 };
    let self_g2 = 2.0 * a2 * a2 * e2 * sin2 * (1.0 - e2 * cos2);
    let grad = Vector2::new(self_g1 - w1.dot(&M_p2), self_g2 - p1.dot(&M_w2));

    let self_h1 = if hyp1 {
        2.0 * a1 * a1 * e1 * (e1 - cos1 + 2.0 * e1 * sin1 * sin1)
    } else {
        2.0 * a1 * a1 * e1 * (cos1 - e1 + 2.0 * e1 * sin1 * sin1)
    };
    let self_h2 = 2.0 * a2 * a2 * e2 * (cos2 - e2 + 2.0 * e2 * sin2 * sin2);
    let h11 = self_h1 + cross_sign * u1.dot(&M_p2);
    let h22 = self_h2 + p1.dot(&M_u2);
    let h12 = -w1.dot(&M_w2);
    let hess = Matrix2::new(h11, h12, h12, h22);
    (grad, hess)
}

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
) -> [((f64, f64), (f64, f64)); 2] {
    let lo = -(planet.periapsis() - craft.a() - r_soi) / (craft.a() * craft.e());
    let hi = -(planet.apoapsis() - craft.a() + r_soi) / (craft.a() * craft.e());
    let (lo, hi) = if craft.is_hyperbolic() {
        (lo.max(1.0).acosh(), hi.acosh())
    } else {
        (lo.clamp(-1.0, 1.0).acos(), hi.clamp(-1.0, 1.0).acos())
    };
    let E1_mid = 0.5 * (lo + hi);
    let project_E2 = |E1: f64| -> f64 {
        let q = planet.orb_to_ecl().transpose() * craft_position(craft, E1);
        (q.y / planet.b()).atan2(q.x / planet.a() + planet.e())
    };
    // Each branch's valid E1 interval — the positive arc is [lo, hi] and the
    // negative arc is [−hi, −lo]. Returned alongside the seed so the solver
    // can bracket E1 to its own branch and avoid basin-hopping into the other.
    [((E1_mid, project_E2(E1_mid)), (lo, hi)), ((-E1_mid, project_E2(-E1_mid)), (-hi, -lo))]
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

/// Locate anomaly pairs (E₁/H₁, E₂) that minimize the squared 3D distance
/// f = r₁² + r₂² − p₁ᵀ M p₂ between the two orbits, via Levenberg-Marquardt
/// on each coarse-interval branch. A minimum is reported only if f ≤ r_soi²
/// at convergence, i.e. the two orbits actually come within r_soi.
///
/// Returns one entry per branch whose minimum satisfies the SOI condition.
pub fn find_soi_minima(
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
) -> Vec<(f64, f64)> {
    let is_possible = spacecraft.apoapsis() + r_soi >= planet.periapsis()
        && spacecraft.periapsis() - r_soi <= planet.apoapsis();

    if !is_possible {
        return Vec::new();
    }

    let M_matrix = scaled_coupling_matrix(spacecraft, planet);
    let r_soi_sq = r_soi * r_soi;

    let f_at = |E1: f64, E2: f64| distance_sq(spacecraft, planet, E1, E2);

    // Seed each ±branch with the radial-overlap midpoint, then swap in a
    // mutual-nodal seed if it falls inside the branch's E1 interval with a
    // smaller f. This is the per-branch best-of-{radial, nodal} dispatch —
    // helps cases like Voyager 2 / Jupiter where the nodal guess is 5×
    // closer to the minimum than the radial midpoint.
    let radial = radial_initial_guesses(spacecraft, planet, r_soi);
    let nodal = nodal_initial_guesses(spacecraft, planet);
    let mut candidates: Vec<(f64, f64, (f64, f64), f64)> = Vec::with_capacity(2);
    for &((E1_r, E2_r), bounds) in &radial {
        let (lo, hi) = bounds;
        let (mut E1, mut E2, mut f) = (E1_r, E2_r, f_at(E1_r, E2_r));
        if let Some(nodal_seeds) = &nodal {
            for &(E1_n, E2_n) in nodal_seeds {
                if E1_n >= lo && E1_n <= hi {
                    let f_n = f_at(E1_n, E2_n);
                    if f_n < f {
                        (E1, E2, f) = (E1_n, E2_n, f_n);
                    }
                }
            }
        }
        candidates.push((E1, E2, bounds, f));
    }
    eprintln!(
        "[find_soi_minima] r_soi_sq={r_soi_sq:.3e}  raw seeds: {:?}",
        candidates.iter().map(|c| (c.0, c.1, c.2, c.3)).collect::<Vec<_>>()
    );
    candidates.sort_by(|a, b| a.3.partial_cmp(&b.3).unwrap());
    eprintln!(
        "[find_soi_minima] sorted seeds (best first): {:?}",
        candidates.iter().map(|c| (c.0, c.1, c.2, c.3)).collect::<Vec<_>>()
    );

    let mut results = Vec::new();
    for (E1_init, E2_init, bounds, _) in &candidates {
        match levenberg_minimize_2d(
            *E1_init, *E2_init, spacecraft, planet, &M_matrix, r_soi_sq, *bounds,
        ) {
            Some(result) => {
                eprintln!("[find_soi_minima]   -> CONVERGED inside r_soi: {result:?}");
                results.push(result);
            }
            None => eprintln!("[find_soi_minima]   -> rejected (None)"),
        }
    }

    results.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
    results
}

/// 2D Levenberg-Marquardt iteration on f(E₁, E₂). Step δ solves
/// (H + λI)·δ = −∇f where H is the analytic 2×2 Hessian of
/// f = r₁² + r₂² − p₁ᵀMp₂. λ=0 gives a pure Newton step; when a step
/// fails to decrease f (singular/indefinite H or poor quadratic model),
/// λ is raised and the step is re-solved from the same iterate.
#[inline]
pub fn levenberg_minimize_2d(
    mut E1: f64,
    mut E2: f64,
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    M: &Matrix2<f64>,
    r_soi_sq: f64,
    e1_bounds: (f64, f64),
) -> Option<(f64, f64)> {
    let (e1_lo, e1_hi) = e1_bounds;
    let eval_f = |e1v: f64, e2v: f64| distance_sq(spacecraft, planet, e1v, e2v);

    // Levenberg-Marquardt damping. λ=0 → pure Newton. When a step increases f
    // (Newton model is untrustworthy, typically because H is near-singular or
    // indefinite), we reject, raise λ, and re-solve (H + λI)δ = −∇f from the
    // same iterate. As λ → ∞ the step becomes short and aligned with −∇f.
    let mut lambda = 0.0_f64;
    let mut f = eval_f(E1, E2);

    for i in 0..MAX_ITER {
        let (grad, hess) = gradient_hessian_at(E1, E2, spacecraft, planet, M);
        let (h11, h22) = (hess.m11, hess.m22);

        // let d_km = format_with_thousand_separators(f.max(0.0).sqrt().round() as u64);
        // println!(
        //     "{i:>4}  E1={E1:>23.16}  E2={E2:>23.16}  f={f:>22.12e}  d={d_km:>16} km  λ={lambda:.2e}"
        // );

        // Gradient-norm convergence: at a stationary point of f, we're done.
        if grad.amax() < TOL {
            return if f <= r_soi_sq { Some((E1, E2)) } else { None };
        }

        // Inner Levenberg loop: try (H + λI)δ = −∇f, accept if f decreases, else raise λ.
        let lambda_base = 1e-6 * h11.abs().max(h22.abs()).max(1.0);
        let mut accepted = false;
        let mut step = Vector2::zeros();
        let (mut new_E1, mut new_E2, mut new_f) = (E1, E2, f);
        for _ in 0..40 {
            let damped = hess + Matrix2::from_diagonal_element(lambda);
            let det = damped.determinant();
            if det.abs() < f64::EPSILON || det <= 0.0 {
                // Damped Hessian still indefinite/singular — raise λ.
                lambda = (lambda * 4.0).max(lambda_base);
                continue;
            }
            // Cramer's rule: δ = −H⁻¹ ∇f with H⁻¹ = adj(H)/det.
            step = Vector2::new(
                damped.m22 * grad.x - damped.m12 * grad.y,
                damped.m11 * grad.y - damped.m21 * grad.x,
            ) / -det;

            // Active-set projection: when E1 sits on a bound and the Newton
            // step wants to push further outside, clamping silently kills dE1
            // and Levenberg thrashes raising λ to compensate. Instead, freeze E1 and
            // solve the 1D damped-Newton subproblem along E2 only.
            let dh22 = h22 + lambda;
            let at_lo = E1 <= e1_lo + TOL && step.x < 0.0;
            let at_hi = E1 >= e1_hi - TOL && step.x > 0.0;
            if (at_lo || at_hi) && dh22.abs() > f64::EPSILON {
                step.x = 0.0;
                step.y = -grad.y / dh22;
            }

            new_E1 = (E1 + step.x).clamp(e1_lo, e1_hi);
            new_E2 = E2 + step.y;
            new_f = eval_f(new_E1, new_E2);
            if new_f < f {
                accepted = true;
                break;
            }
            lambda = (lambda * 4.0).max(lambda_base);
        }
        if !accepted {
            // Can't make progress — return current iterate if admissible.
            return if f <= r_soi_sq { Some((E1, E2)) } else { None };
        }

        let step_at_tol = step.amax() < TOL;
        // Relative-f stall: decrease is below √eps · f. Handles bound-constrained
        // minima where ∇f ≠ 0 but the feasible step no longer reduces f.
        let f_stalled = (f - new_f).powi(2) <= f * f * TOL;
        E1 = new_E1;
        E2 = new_E2;
        f = new_f;
        if step_at_tol || f_stalled {
            return if f <= r_soi_sq { Some((E1, E2)) } else { None };
        }
        // Successful step — relax damping back toward pure Newton.
        lambda *= 0.25;
        if lambda < lambda_base * 1e-3 {
            lambda = 0.0;
        }
    }

    if f <= r_soi_sq { Some((E1, E2)) } else { None }
}

/// Compute the scaled coupling matrix M = diag(a₁,b₁) · C · diag(a₂,b₂)
/// where C = 2 Aᵀ B encodes the mutual orientation of the two orbital planes.
#[inline]
pub fn scaled_coupling_matrix(
    orbit1: &Trajectory,
    orbit2: &Orbit3D<EllipticOrbit>,
) -> Matrix2<f64> {
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

        let results = find_soi_minima(&traj, &jupiter, r_soi);
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

        let results = find_soi_minima(&traj, &earth, r_soi);
        println!("Results: {results:?}");
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

        let results = find_soi_minima(&traj, &saturn, r_soi);
        println!("Results: {results:?}");
        assert!(!results.is_empty(), "Newton's method did not converge for hyperbolic encounter");
    }
}
