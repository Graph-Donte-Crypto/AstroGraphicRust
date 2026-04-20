use std::f64::consts::TAU;

use chrono::{DateTime, TimeZone, Utc};
use nalgebra::{Matrix2, Vector2};

use crate::angle::{Angle, MeanAnomaly};
use crate::kepler_equation::solve_kepler_householder_pade_elliptic;
use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::orbit_3d::Orbit3D;
use crate::soi_minima::{distance_sq, gradient_hessian_at, scaled_coupling_matrix};
use crate::trajectory::Trajectory;
use crate::util::FloatExt;

const NEWTON_TOL: f64 = 1e-12;
const NEWTON_MAX_ITER: usize = 50;
/// Upper bound on integer iterations of the (k1, k2) search. For any realistic
/// admissible encounter the first hit lies well below this; the cap exists to
/// bound runtime when `τ = T1/T2` is (near-)rational and no admissible pair
/// exists at all.
const SEARCH_MAX_ITER: i64 = 1_000_000;

/// Real SOI entry: eccentric/hyperbolic anomalies at crossing, revolution
/// indices `(k1, k2)` satisfying the time-coupling constraint, and the
/// absolute time `t_enc` in seconds from epoch `t = 0`.
#[derive(Debug, Clone, Copy)]
pub struct Encounter {
    pub t_enc: f64,
    pub E1: f64,
    pub E2: f64,
    pub k1: i64,
    pub k2: i64,
}

/// Given a geometric minimum `(E1*, E2*)` of the squared 3D distance (as
/// produced by `soi_minima::find_soi_minima`), locate the earliest real SOI
/// entry by intersecting the time-coupling family of straight lines in
/// `(M1, M2)` with the Taylor ellipse of `f` around the minimum.
///
/// `t_start` is the spacecraft's mission-start epoch (the time at which its
/// `M0` was recorded). The planet's `M0` epoch is hard-coded to J2000
/// (2000-01-01T12:00:00Z), matching the convention in `config/system/*.yml`.
/// The returned `t_enc` is in seconds from J2000.
///
/// Returns `None` when no integer `(k1, k2)` inside the admissibility band
/// `|T1·k1 − T2·k2 − α*| ≤ Δα` is found via the continued-fraction walk
/// (typically: `τ = T1/T2` is rational, or the minimum is a geometric
/// accident that cannot be promoted to a real encounter).
pub fn find_encounter(
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
    minima: &[(f64, f64)],
    t_start: DateTime<Utc>,
) -> Option<Encounter> {
    // The distance² function has a (E1, E2) → (−E1, −E2) symmetry, so each
    // physical encounter shows up as two geometric minima corresponding to
    // different revolution pairings / times. Try every minimum the caller
    // supplies; return the encounter whose `t_enc` is the earliest at or
    // after `t_start` (soonest future encounter). If none are after t_start,
    // fall back to the globally earliest.
    let j2000 = Utc.with_ymd_and_hms(2000, 1, 1, 12, 0, 0).unwrap();
    let t_start_s = (t_start - j2000).num_seconds() as f64;
    let candidates: Vec<_> = minima
        .iter()
        .filter_map(|&m| find_encounter_single(spacecraft, planet, r_soi, m, t_start))
        .collect();
    candidates
        .iter()
        .filter(|e| e.t_enc >= t_start_s)
        .min_by(|a, b| a.t_enc.partial_cmp(&b.t_enc).unwrap())
        .or_else(|| candidates.iter().min_by(|a, b| a.t_enc.partial_cmp(&b.t_enc).unwrap()))
        .copied()
}

fn find_encounter_single(
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
    (E1_star, E2_star): (f64, f64),
    t_start: DateTime<Utc>,
) -> Option<Encounter> {
    let j2000 = Utc.with_ymd_and_hms(2000, 1, 1, 12, 0, 0).unwrap();
    let t_start_s = (t_start - j2000).num_seconds() as f64;
    let (e1, e2) = (spacecraft.e(), planet.e());
    let hyp1 = spacecraft.is_hyperbolic();

    // Hessian at the geometric minimum (∇f ≈ 0 there, so we ignore it).
    let coupling = scaled_coupling_matrix(spacecraft, planet);
    let (_grad, hess) = gradient_hessian_at(E1_star, E2_star, spacecraft, planet, &coupling);

    // f* = d² − r_soi² (< 0 because the minimum is inside the SOI).
    let r_soi_sq = r_soi * r_soi;
    let f_star = distance_sq(spacecraft, planet, E1_star, E2_star) - r_soi_sq;

    // j_i = ∂M_i/∂E_i. Elliptic: 1 − e·cos E. Hyperbolic: e·cosh H − 1.
    let j1 = if hyp1 { e1 * E1_star.cosh() - 1.0 } else { 1.0 - e1 * E1_star.cos() };
    let j2 = 1.0 - e2 * E2_star.cos();

    // H_M = J⁻¹ H J⁻¹ with J = diag(j1, j2). Since J is diagonal this is
    // componentwise scaling by the outer product (1/j)(1/j)ᵀ — no matmul.
    let inv_j = Vector2::new(1.0 / j1, 1.0 / j2);
    let h_m = hess.component_mul(&(inv_j * inv_j.transpose()));

    // Mean anomalies at the minimum.
    let m1_star =
        if hyp1 { e1 * E1_star.sinh() - E1_star } else { E1_star - e1 * E1_star.sin() };
    let m2_star = E2_star - e2 * E2_star.sin();

    let t1 = time_scale(spacecraft);
    let t2 = time_scale_elliptic(planet);
    // Rewind spacecraft's M0 from its own epoch to the planet's epoch (common
    // t=0 for the rest of the calculation). M1(t) = M0 + n1·(t − t_body_epoch),
    // so M1(planet_epoch) = M0 − n1·(spacecraft_epoch − planet_epoch).
    let m1_0 = (m0_of(spacecraft) - TAU * t_start_s / t1).rem_euclid(TAU);
    let m2_0 = planet.orbit_2d.0.M0().as_rad().rem_euclid(TAU);

    let alpha_star = (t2 * (m2_star - m2_0) - t1 * (m1_star - m1_0)) / TAU;
    eprintln!(
        "[encounter] M1*={m1_star:.6} rad  M10={m1_0:.6} rad  (M1*-M10)={:.6} rad  T1·(…)/2π={:.3} days",
        m1_star - m1_0,
        t1 * (m1_star - m1_0) / TAU / 86400.0
    );
    eprintln!(
        "[encounter] M2*={m2_star:.6} rad  M20={m2_0:.6} rad  (M2*-M20)={:.6} rad  T2·(…)/2π={:.3} days",
        m2_star - m2_0,
        t2 * (m2_star - m2_0) / TAU / 86400.0
    );

    let s = t1 / t2;
    let a_coef = h_m.m11 + 2.0 * s * h_m.m12 + s * s * h_m.m22;
    let det_h_m = h_m.determinant();
    if det_h_m <= 0.0 || a_coef <= 0.0 || f_star >= 0.0 {
        return None;
    }
    let delta_alpha = (t2 / TAU) * (-2.0 * a_coef * f_star / det_h_m).sqrt();

    let d_star = (f_star + r_soi_sq).max(0.0).sqrt();
    eprintln!(
        "[encounter] d*={d_star:.0} km (r_soi={r_soi:.0} km)  f*={f_star:.3e}  α*={alpha_star:.3e} s ({:.3} days)  α*/T2={:.6} (Δα/T2={:.6})  Δα={delta_alpha:.3e} s ({:.3} days)  T1={t1:.3e}  T2={t2:.3e}  τ={:.6}",
        alpha_star / 86400.0,
        alpha_star / t2,
        delta_alpha / t2,
        delta_alpha / 86400.0,
        t1 / t2,
    );

    // Find admissible (k1, k2).
    let (k1, k2) = if hyp1 {
        // Hyperbolic: k1 ≡ 0, so α = −T2·k2.
        let k2 = (-alpha_star / t2).round() as i64;
        let alpha = -t2 * k2 as f64;
        if (alpha - alpha_star).abs() > delta_alpha {
            return None;
        }
        (0_i64, k2)
    } else {
        search_k1_k2(t1, t2, alpha_star, delta_alpha)?
    };

    let alpha = t1 * k1 as f64 - t2 * k2 as f64;
    let d0 = TAU * (alpha - alpha_star) / t2;

    // Intersect the time line ΔM2 = s·ΔM1 + d0 with the Taylor ellipse
    // ΔMᵀ H_M ΔM = −2f*. Two real roots (generic), we take the smaller-M1 one
    // as the SOI-entry side.
    let big_a = a_coef;
    let big_b = 2.0 * d0 * (h_m.m12 + s * h_m.m22);
    let big_c = d0 * d0 * h_m.m22 + 2.0 * f_star;
    let disc = big_b * big_b - 4.0 * big_a * big_c;
    if disc < 0.0 {
        return None;
    }
    let dm1 = (-big_b - disc.sqrt()) / (2.0 * big_a);
    let dm2 = s * dm1 + d0;
    let m1_seed = m1_star + dm1;
    let m2_seed = m2_star + dm2;

    // Invert Kepler to get (E1, E2) seeds.
    let e1_seed = if hyp1 {
        invert_kepler_hyperbolic(m1_seed, e1)
    } else {
        solve_kepler_householder_pade_elliptic(e1, MeanAnomaly::from(Angle::from_rad(m1_seed)))
            .as_rad()
    };
    let e2_seed =
        solve_kepler_householder_pade_elliptic(e2, MeanAnomaly::from(Angle::from_rad(m2_seed)))
            .as_rad();

    // Joint 2D Newton on {f = 0, h = 0}.
    let (E1, E2) = newton_fh(
        e1_seed, e2_seed, spacecraft, planet, &coupling, r_soi_sq, t1, t2, m1_0, m2_0, k1, k2,
    )?;

    let t_enc = if hyp1 {
        let m1_enc = e1 * E1.sinh() - E1;
        t1 / TAU * (m1_enc - m1_0)
    } else {
        let m1_enc = E1 - e1 * E1.sin();
        t1 / TAU * (m1_enc + TAU * k1 as f64 - m1_0)
    };

    Some(Encounter { t_enc, E1, E2, k1, k2 })
}

/// 2π · √(|a|³ / μ). Orbital period for elliptic orbits; time scale (same
/// formula, same role in Kepler's equation) for hyperbolic ones.
fn time_scale(traj: &Trajectory) -> f64 {
    let (a, mu) = match traj {
        Trajectory::Elliptic(o) => (o.orbit_2d.0.a(), o.orbit_2d.0.mu()),
        Trajectory::Hyperbolic(o) => (o.orbit_2d.0.a(), o.orbit_2d.0.mu()),
    };
    TAU * (a.abs().powi(3) / mu).sqrt()
}

fn time_scale_elliptic(orb: &Orbit3D<EllipticOrbit>) -> f64 {
    let a = orb.orbit_2d.0.a();
    let mu = orb.orbit_2d.0.mu();
    TAU * (a.abs().powi(3) / mu).sqrt()
}

fn m0_of(traj: &Trajectory) -> f64 {
    match traj {
        Trajectory::Elliptic(o) => o.orbit_2d.0.M0().as_rad(),
        Trajectory::Hyperbolic(o) => o.orbit_2d.0.M0().as_rad(),
    }
}

/// Newton inversion of hyperbolic Kepler: given M, return H such that
/// `e·sinh H − H = M`. Seeded with the asymptotic `H ≈ sign(M)·ln(2|M|/e)`
/// (exact for large |M|) and falling back to `M/(e−1)` for small |M|.
fn invert_kepler_hyperbolic(m: f64, e: f64) -> f64 {
    let seed_log = (2.0 * m.abs() / e).ln();
    let mut h = if seed_log.is_finite() && seed_log > 1.0 {
        m.signum() * seed_log
    } else {
        m / (e - 1.0)
    };
    for _ in 0..50 {
        let (sh, ch) = h.sinh_cosh();
        let dh = -(e * sh - h - m) / (e * ch - 1.0);
        h += dh;
        if dh.abs() < 1e-14 {
            break;
        }
    }
    h
}

/// Search for integer `(k1, k2)` with `|T1·k1 − T2·k2 − α*| ≤ Δα` by plain
/// iteration, stepping outward from 0 in alternating ±. The loop runs on
/// whichever side (`k1` or `k2`) corresponds to the shorter period — for the
/// same physical encounter cycle, integer magnitudes grow faster on that
/// side, so the first admissible hit is reached in fewer iterations.
///
/// Returns `None` after `SEARCH_MAX_ITER` iterations (encounter does not fit
/// on an integer lattice, or `τ` is rational).
fn search_k1_k2(t1: f64, t2: f64, alpha_star: f64, delta_alpha: f64) -> Option<(i64, i64)> {
    let iterate_k1 = t1 <= t2;
    for step in 0..=SEARCH_MAX_ITER {
        for k in [step, -step] {
            let (k1, k2) = if iterate_k1 {
                let k1 = k;
                let k2 = ((t1 * k1 as f64 - alpha_star) / t2).round() as i64;
                (k1, k2)
            } else {
                let k2 = k;
                let k1 = ((t2 * k2 as f64 + alpha_star) / t1).round() as i64;
                (k1, k2)
            };
            if (t1 * k1 as f64 - t2 * k2 as f64 - alpha_star).abs() <= delta_alpha {
                eprintln!(
                    "[search] iterate_{} found (k1, k2) = ({k1}, {k2}) at step {step}",
                    if iterate_k1 { "k1" } else { "k2" },
                );
                return Some((k1, k2));
            }
            if step == 0 {
                break;
            }
        }
    }
    None
}

/// 2D Newton on the system `{f = 0, h = 0}` with analytic Jacobian
/// `[[∂f/∂E1, ∂f/∂E2], [T1·j1, −T2·j2]]` where `j_i = ∂M_i/∂E_i`. Reuses
/// `gradient_hessian_at` for `∇f`. Solves via Cramer's rule.
fn newton_fh(
    mut E1: f64,
    mut E2: f64,
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    coupling: &Matrix2<f64>,
    r_soi_sq: f64,
    t1: f64,
    t2: f64,
    m1_0: f64,
    m2_0: f64,
    k1: i64,
    k2: i64,
) -> Option<(f64, f64)> {
    let (e1, e2) = (spacecraft.e(), planet.e());
    let hyp1 = spacecraft.is_hyperbolic();
    let alpha = t1 * k1 as f64 - t2 * k2 as f64;

    for _ in 0..NEWTON_MAX_ITER {
        let (grad_f, _hess) = gradient_hessian_at(E1, E2, spacecraft, planet, coupling);
        let d2 = distance_sq(spacecraft, planet, E1, E2);
        let f = d2 - r_soi_sq;

        let (m1, j1) = if hyp1 {
            (e1 * E1.sinh() - E1, e1 * E1.cosh() - 1.0)
        } else {
            (E1 - e1 * E1.sin(), 1.0 - e1 * E1.cos())
        };
        let (m2, j2) = (E2 - e2 * E2.sin(), 1.0 - e2 * E2.cos());
        let h = t1 * (m1 - m1_0) - t2 * (m2 - m2_0) + TAU * alpha;

        let jac = Matrix2::new(grad_f.x, grad_f.y, t1 * j1, -t2 * j2);
        let det = jac.determinant();
        if det.abs() < f64::EPSILON {
            return None;
        }
        // Cramer: [dE1; dE2] = −J⁻¹ [f; h].
        let dE1 = -(f * jac.m22 - h * jac.m12) / det;
        let dE2 = -(h * jac.m11 - f * jac.m21) / det;

        E1 += dE1;
        E2 += dE2;

        if dE1.abs() < NEWTON_TOL && dE2.abs() < NEWTON_TOL {
            return Some((E1, E2));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AU_IN_KM;
    use crate::config::{Config, StarSystem};
    use crate::orbit::orbit_3d::KeplerianElements;
    use crate::soi_minima::find_soi_minima;

    #[test]
    fn voyager2_jupiter_encounter() {
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

        let minima = find_soi_minima(&traj, &jupiter, r_soi);
        assert!(!minima.is_empty(), "expected a geometric minimum inside Jupiter's SOI");

        let enc = find_encounter(&traj, &jupiter, r_soi, &minima, config.spacecraft.t0)
            .expect("encounter expected");
        println!("Voyager 2 / Jupiter encounter: {enc:?}");
        println!("t_enc = {:.3} days", enc.t_enc / 86400.0);
        let d = distance_sq(&traj, &jupiter, enc.E1, enc.E2).sqrt();
        println!("distance at encounter: {d:.3} km  (r_soi = {r_soi:.0} km)");
        assert!((d - r_soi).abs() / r_soi < 1e-6, "f=0 residual: d={d}, r_soi={r_soi}");
        // Real Voyager 2 Jupiter arrival: 1979-07-09 → ≈ −7485 days from J2000.
        let t_enc_days = enc.t_enc / 86400.0;
        assert!(
            (t_enc_days - (-7485.0)).abs() < 200.0,
            "expected t_enc ≈ −7485 days (1979-07-09), got {t_enc_days:.1}"
        );
    }

    /// Parker-like spacecraft vs. Venus — stress case from the paper. Here
    /// `Δα = 0.372 days` is very narrow relative to the lattice spacing `T2/2`,
    /// so the admissible `(k1, k2)` is far from the origin (paper reports the
    /// first hit at `(91, 40)` via continued fractions).
    ///
    #[test]
    fn parker_venus_encounter() {
        let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
        let system = StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

        let mu_sun = system.star.μ;
        let venus_cfg = system.planets.iter().find(|p| p.body.name == "Venus").unwrap();
        let venus: Orbit3D<EllipticOrbit> = KeplerianElements::from(venus_cfg.orbit.clone()).into();
        let r_soi = venus_cfg.soi_radius();

        // Parker-like perihelion 0.046 AU, aphelion 0.80 AU, coplanar with Venus.
        let rp = 0.046 * AU_IN_KM;
        let ra = 0.80 * AU_IN_KM;
        let a1 = 0.5 * (rp + ra);
        let e1 = (ra - rp) / (ra + rp);
        let parker: Orbit3D<EllipticOrbit> = KeplerianElements {
            mu: mu_sun,
            a: a1,
            e: e1,
            i: 3.39468_f64.to_radians(),
            Omega: 76.67984_f64.to_radians(),
            omega: 234.92262_f64.to_radians(),
            M0: 0.0,
        }
        .into();
        let traj = Trajectory::Elliptic(parker);

        let minima = find_soi_minima(&traj, &venus, r_soi);
        assert!(!minima.is_empty(), "expected a geometric minimum inside Venus's SOI");

        // Same epoch for both M0s (no offset).
        // Parker's epoch coincides with J2000 → t_start = system.t0.
        let enc = find_encounter(&traj, &venus, r_soi, &minima, system.t0)
            .expect("encounter expected");
        println!("Parker / Venus encounter: {enc:?}");
        println!("t_enc = {:.3} years", enc.t_enc / (365.25 * 86400.0));
        let d = distance_sq(&traj, &venus, enc.E1, enc.E2).sqrt();
        println!("distance at encounter: {d:.3} km  (r_soi = {r_soi:.0} km)");
        assert!((d - r_soi).abs() / r_soi < 1e-6, "f=0 residual: d={d}, r_soi={r_soi}");
        assert!(enc.k1 > 0 && enc.t_enc > 0.0);
    }
}
