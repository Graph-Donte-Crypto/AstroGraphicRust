use std::f64::consts::{PI, TAU};
use std::time::Duration;

use criterion::{criterion_group, criterion_main, Criterion};
use nalgebra::{Matrix2, Matrix3x2, Vector2};

/// Rotation matrix from orbital to ecliptic frame (3×2).
fn rotation_matrix(inc: f64, omega: f64, big_omega: f64) -> Matrix3x2<f64> {
    let (ci, si) = (inc.cos(), inc.sin());
    let (co, so) = (omega.cos(), omega.sin());
    let (c_o, s_o) = (big_omega.cos(), big_omega.sin());
    Matrix3x2::new(
        c_o * co - s_o * so * ci,
        -c_o * so - s_o * co * ci,
        s_o * co + c_o * so * ci,
        -s_o * so + c_o * co * ci,
        so * si,
        co * si,
    )
}

/// Scaled coupling matrix M = diag(a₁,b₁) · 2AᵀB · diag(a₂,b₂).
fn scaled_coupling_matrix(
    a1: f64,
    b1: f64,
    a2: f64,
    b2: f64,
    mat_a: &Matrix3x2<f64>,
    mat_b: &Matrix3x2<f64>,
) -> Matrix2<f64> {
    let c = mat_a.transpose() * mat_b * 2.0;
    Matrix2::new(
        c[(0, 0)] * a1 * a2,
        c[(0, 1)] * a1 * b2,
        c[(1, 0)] * b1 * a2,
        c[(1, 1)] * b1 * b2,
    )
}

/// f(E₁, E₂) = r₁² + r₂² - p₁ᵀ M p₂ - r_soi²
fn f_value(ea1: f64, ea2: f64, a1: f64, e1: f64, a2: f64, e2: f64, m: &Matrix2<f64>, r_soi_sq: f64) -> f64 {
    let p1 = Vector2::new(ea1.cos() - e1, ea1.sin());
    let p2 = Vector2::new(ea2.cos() - e2, ea2.sin());
    let r1 = a1 * (1.0 - e1 * ea1.cos());
    let r2 = a2 * (1.0 - e2 * ea2.cos());
    r1 * r1 + r2 * r2 - p1.dot(&(m * p2)) - r_soi_sq
}

/// Estimate E₂ by projecting spacecraft position onto the planet's orbital plane.
/// Uses the geometric ellipse inversion: E₂ = atan2(q_y/b₂, q_x/a₂ + e₂).
fn atan2_projection(ea1: f64, a1: f64, e1: f64, b1: f64, a2: f64, e2: f64, b2: f64, mat_a: &Matrix3x2<f64>, mat_b: &Matrix3x2<f64>) -> f64 {
    let r1_2d = Vector2::new(a1 * (ea1.cos() - e1), b1 * ea1.sin());
    let r1_3d = mat_a * r1_2d;
    let q = mat_b.transpose() * r1_3d;
    (q.y / b2).atan2(q.x / a2 + e2)
}

/// E₂ estimate from df/dE₂ = 0 with e₂ ≈ 0: just p₁ᵀ M ŵ₂ = 0.
///
/// Gives E₂ = atan2(v₂, v₁) where v = Mᵀ p₁.
/// No 3D projection needed — works directly with the 2×2 coupling matrix.
fn cross_term_estimate(ea1: f64, e1: f64, m: &Matrix2<f64>) -> f64 {
    let p1 = Vector2::new(ea1.cos() - e1, ea1.sin());
    let v = m.transpose() * p1;
    let ea2 = v.y.atan2(v.x);
    // atan2(v₂,v₁) zeros p₁ᵀMŵ₂, but could be a max or min of the cross-term.
    // We want max cross-term (= min f). Second derivative of cross-term is
    // -p₁ᵀMû₂ where û₂=(cosE₂,sinE₂). Cross-term is maximized when p₁ᵀMû₂ > 0.
    let u2 = Vector2::new(ea2.cos(), ea2.sin());
    if p1.dot(&(m * u2)) > 0.0 {
        ea2
    } else {
        // Flip to opposite quadrant
        if ea2 > 0.0 { ea2 - PI } else { ea2 + PI }
    }
}

/// Refine the atan2 projection estimate with Newton iterations on df/dE₂ = 0.
///
/// Each step: E₂ -= g₂ / H₂₂ where
///   g₂ = 2a₂²e₂ sinE₂ (1 - e₂ cosE₂) - p₁ᵀ M ŵ₂
///   H₂₂ = 2a₂²e₂ (cosE₂ - e₂ + 2e₂ sin²E₂) + p₁ᵀ M û₂
fn newton_estimate(
    ea1: f64,
    mut ea2: f64,
    a1: f64,
    e1: f64,
    a2: f64,
    e2: f64,
    m: &Matrix2<f64>,
    iters: usize,
) -> f64 {
    let p1 = Vector2::new(ea1.cos() - e1, ea1.sin());

    for _ in 0..iters {
        let (sin2, cos2) = ea2.sin_cos();
        let w2 = Vector2::new(-sin2, cos2);
        let u2 = Vector2::new(cos2, sin2);

        let g2 = 2.0 * a2 * a2 * e2 * sin2 * (1.0 - e2 * cos2) - p1.dot(&(m * w2));
        let h22 = 2.0 * a2 * a2 * e2 * (cos2 - e2 + 2.0 * e2 * sin2 * sin2)
            + p1.dot(&(m * u2));

        if h22.abs() < 1e-30 { break; }
        ea2 -= g2 / h22;
    }

    ea2
}

/// Estimate E₂ by solving df/dE₂ = 0 exactly via half-angle quartic.
///
/// df/dE₂ = 0 becomes: BB·t⁴ + 2(AA+CC)·t³ + 0·t² + 2(AA−CC)·t − BB = 0
/// where t = tan(E₂/2), AA = 2a₂²e₂ + v₁, BB = v₂, CC = 2a₂²e₂².
fn quartic_estimate(
    ea1: f64,
    ea2_hint: f64,
    a1: f64,
    e1: f64,
    a2: f64,
    e2: f64,
    m: &Matrix2<f64>,
    r_soi_sq: f64,
) -> f64 {
    let p1x = ea1.cos() - e1;
    let p1y = ea1.sin();
    let p1 = Vector2::new(p1x, ea1.sin());

    // v = Mᵀ p₁
    let v1 = m[(0, 0)] * p1x + m[(1, 0)] * p1y;
    let v2 = m[(0, 1)] * p1x + m[(1, 1)] * p1y;

    let aa = 2.0 * a2 * a2 * e2 + v1;
    let bb = v2;
    let cc = 2.0 * a2 * a2 * e2 * e2;

    // Quartic: bb·t⁴ + 2(aa+cc)·t³ + 0·t² + 2(aa−cc)·t − bb = 0
    //
    // When |bb| << |aa+cc| (small v₂), Ferrari's method is ill-conditioned.
    // Fall back to cubic: 2(aa+cc)·t³ + 2(aa−cc)·t − bb = 0, then
    // polish roots with one Newton step on the full quartic.
    let c3 = 2.0 * (aa + cc);
    let c1 = 2.0 * (aa - cc);

    // Collect up to 4 real roots of bb·t⁴ + c3·t³ + c1·t - bb = 0
    let mut ts = [0.0_f64; 4];
    let mut n_roots = 0usize;

    if bb.abs() < 1e-6 * c3.abs() {
        // Degenerate: solve cubic c3·t³ + c1·t - bb = 0
        for &t in roots::find_roots_cubic(c3, 0.0, c1, -bb).as_ref() {
            ts[n_roots] = t;
            n_roots += 1;
        }
    } else {
        // Depressed quartic via Tschirnhaus substitution.
        // Normalize: t⁴ + α·t³ + γ·t + δ = 0 (t² coeff is already 0)
        let alpha = c3 / bb;
        let gamma = c1 / bb;

        // Substitute t = u - α/4 → u⁴ + pu² + qu + r = 0
        let shift = alpha / 4.0;
        let alpha_sq = alpha * alpha;
        let p = -3.0 / 8.0 * alpha_sq;
        let q = alpha * alpha_sq / 8.0 + gamma;
        let r = -3.0 / 256.0 * alpha_sq * alpha_sq - gamma * alpha / 4.0 - 1.0;

        if q.abs() < 1e-12 * (p.abs() + r.abs() + 1.0) {
            // Biquadratic: u⁴ + pu² + r = 0
            let disc = p * p - 4.0 * r;
            if disc >= 0.0 {
                let sqrt_disc = disc.sqrt();
                for v in [(-p + sqrt_disc) / 2.0, (-p - sqrt_disc) / 2.0] {
                    if v >= 0.0 {
                        let u = v.sqrt();
                        ts[n_roots] = u - shift;
                        n_roots += 1;
                        ts[n_roots] = -u - shift;
                        n_roots += 1;
                    }
                }
            }
        } else {
            // Ferrari: resolvent cubic y³ - (p/2)y² - ry + (pr/2 - q²/8) = 0
            let resolvent = roots::find_roots_cubic(
                1.0,
                -p / 2.0,
                -r,
                p * r / 2.0 - q * q / 8.0,
            );

            // Pick the largest root (ensures 2y₀ - p ≥ 0)
            let y0 = resolvent
                .as_ref()
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max);

            let s_sq = 2.0 * y0 - p;
            if s_sq >= 0.0 {
                let s = s_sq.sqrt();
                let q_over_2s = q / (2.0 * s);

                // Quadratic 1: u² - s·u + (y₀ + q/(2s)) = 0
                let disc1 = s_sq - 4.0 * (y0 + q_over_2s);
                if disc1 >= 0.0 {
                    let d1 = disc1.sqrt();
                    ts[n_roots] = (s + d1) / 2.0 - shift;
                    n_roots += 1;
                    ts[n_roots] = (s - d1) / 2.0 - shift;
                    n_roots += 1;
                }

                // Quadratic 2: u² + s·u + (y₀ - q/(2s)) = 0
                let disc2 = s_sq - 4.0 * (y0 - q_over_2s);
                if disc2 >= 0.0 {
                    let d2 = disc2.sqrt();
                    ts[n_roots] = (-s + d2) / 2.0 - shift;
                    n_roots += 1;
                    ts[n_roots] = (-s - d2) / 2.0 - shift;
                    n_roots += 1;
                }
            }
        }
    }

    // Polish each root with Newton steps on the full quartic p(t) = bb·t⁴ + c3·t³ + c1·t - bb
    let polish = |t: f64| -> f64 {
        let mut t = t;
        for _ in 0..4 {
            let p = bb * t * t * t * t + c3 * t * t * t + c1 * t - bb;
            let dp = 4.0 * bb * t * t * t + 3.0 * c3 * t * t + c1;
            if dp.abs() < 1e-30 { break; }
            t -= p / dp;
        }
        t
    };

    // Filter for local minima (H₂₂ > 0), then pick smallest f.
    // Also check E₂ = π (where t → ∞, missed by the half-angle substitution).
    // Fall back to the projection hint if no minimum found.
    let mut best_ea2 = ea2_hint;
    let mut best_f = f64::INFINITY;

    let check = |ea2: f64, best_ea2: &mut f64, best_f: &mut f64| {
        let (sin2, cos2) = ea2.sin_cos();
        let u2 = Vector2::new(cos2, sin2);
        let h22 = 2.0 * a2 * a2 * e2 * (cos2 - e2 + 2.0 * e2 * sin2 * sin2)
            + p1.dot(&(m * u2));
        if h22 <= 0.0 { return; }
        let fv = f_value(ea1, ea2, a1, e1, a2, e2, m, r_soi_sq);
        if fv < *best_f {
            *best_f = fv;
            *best_ea2 = ea2;
        }
    };

    for i in 0..n_roots {
        let t = polish(ts[i]);
        check(2.0 * t.atan(), &mut best_ea2, &mut best_f);
    }

    // E₂ = π is a singularity of tan(E₂/2); check it as extra candidate.
    check(PI, &mut best_ea2, &mut best_f);

    best_ea2
}

/// Estimate E₂ using only the cubic approximation (dropping the bb·t⁴ term).
///
/// Solves 2(AA+CC)·t³ + 2(AA−CC)·t − BB = 0, then polishes on the full quartic.
fn cubic_estimate(
    ea1: f64,
    ea2_hint: f64,
    a1: f64,
    e1: f64,
    a2: f64,
    e2: f64,
    m: &Matrix2<f64>,
    r_soi_sq: f64,
) -> f64 {
    let p1x = ea1.cos() - e1;
    let p1y = ea1.sin();
    let p1 = Vector2::new(p1x, p1y);

    let v1 = m[(0, 0)] * p1x + m[(1, 0)] * p1y;
    let v2 = m[(0, 1)] * p1x + m[(1, 1)] * p1y;

    let aa = 2.0 * a2 * a2 * e2 + v1;
    let bb = v2;
    let cc = 2.0 * a2 * a2 * e2 * e2;

    let c3 = 2.0 * (aa + cc);
    let c1 = 2.0 * (aa - cc);

    let raw_roots = roots::find_roots_cubic(c3, 0.0, c1, -bb);

    let mut best_ea2 = ea2_hint;
    let mut best_f = f64::INFINITY;

    let check = |ea2: f64, best_ea2: &mut f64, best_f: &mut f64| {
        let (sin2, cos2) = ea2.sin_cos();
        let u2 = Vector2::new(cos2, sin2);
        let h22 = 2.0 * a2 * a2 * e2 * (cos2 - e2 + 2.0 * e2 * sin2 * sin2)
            + p1.dot(&(m * u2));
        if h22 <= 0.0 { return; }
        let fv = f_value(ea1, ea2, a1, e1, a2, e2, m, r_soi_sq);
        if fv < *best_f {
            *best_f = fv;
            *best_ea2 = ea2;
        }
    };

    for &t in raw_roots.as_ref() {
        check(2.0 * t.atan(), &mut best_ea2, &mut best_f);
    }
    check(PI, &mut best_ea2, &mut best_f);

    best_ea2
}

/// Wrapped angular error in [0, π].
fn angle_error(a: f64, b: f64) -> f64 {
    let err = (a - b).abs();
    err.min(TAU - err)
}

/// Golden-section search to minimize f over [a, b] (assumes unimodal).
fn golden_section_min(f: impl Fn(f64) -> f64, mut a: f64, mut b: f64) -> f64 {
    const PHI: f64 = 0.618_033_988_749_895;
    let mut x1 = b - PHI * (b - a);
    let mut x2 = a + PHI * (b - a);
    let mut f1 = f(x1);
    let mut f2 = f(x2);
    for _ in 0..100 {
        if f1 < f2 {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = b - PHI * (b - a);
            f1 = f(x1);
        } else {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = a + PHI * (b - a);
            f2 = f(x2);
        }
    }
    (a + b) / 2.0
}

/// Find the global minimum of f over [-π, π] using grid search + golden-section refinement.
fn global_min(f: impl Fn(f64) -> f64) -> f64 {
    let n = 360;
    let mut best_i = 0;
    let mut best_f = f64::INFINITY;
    let samples: Vec<f64> = (0..n).map(|i| -PI + TAU * i as f64 / n as f64).collect();
    for (i, &x) in samples.iter().enumerate() {
        let fv = f(x);
        if fv < best_f {
            best_f = fv;
            best_i = i;
        }
    }
    let lo = if best_i == 0 { samples[0] - TAU / n as f64 } else { samples[best_i - 1] };
    let hi = if best_i == n - 1 { samples[n - 1] + TAU / n as f64 } else { samples[best_i + 1] };
    golden_section_min(&f, lo, hi)
}

/// Orbital parameters for one benchmark case.
struct BenchCase {
    label: String,
    a1: f64,
    e1: f64,
    inc2_deg: f64,
    e2: f64,
    a2: f64,
    r_soi: f64,
    omega1: f64,
    omega2: f64,
    big_omega2: f64,
}

fn bench_encounter(c: &mut Criterion) {
    let a2 = 39.5;
    let e2 = 0.25;
    let r_soi = 0.05;
    let a1_h = (1.0 + a2) / 2.0;
    let e1_h = (a2 - 1.0) / (a2 + 1.0);

    let mut cases = Vec::new();

    // Varying eccentricity (Pluto i=17°)
    for &e1 in &[0.0, 0.2, 0.5, 0.8, e1_h] {
        let a1 = if e1 > 0.0 { (1.0 + a2) / (1.0 + e1) } else { a2 };
        cases.push(BenchCase {
            label: format!("e1={e1:.3}"),
            a1, e1, inc2_deg: 17.0, e2, a2, r_soi,
            omega1: 0.0, omega2: 0.0, big_omega2: 0.0,
        });
    }

    // Varying Omega2 (Hohmann, i=17°)
    for om_deg in [0, 30, 60, 90, 120, 150, 180] {
        cases.push(BenchCase {
            label: format!("Ω2={om_deg}°"),
            a1: a1_h, e1: e1_h, inc2_deg: 17.0, e2, a2, r_soi,
            omega1: 0.0, omega2: 0.0, big_omega2: (om_deg as f64).to_radians(),
        });
    }

    // Varying planet eccentricity (i=17°)
    for &e2t in &[0.0, 0.01, 0.05, 0.1, 0.25] {
        let a1 = (1.0 + a2 * (1.0 - e2t)) / 2.0;
        let e1 = 1.0 - 1.0 / a1;
        if e1 <= 0.0 || e1 >= 1.0 { continue; }
        cases.push(BenchCase {
            label: format!("e2={e2t:.2}"),
            a1, e1, inc2_deg: 17.0, e2: e2t, a2, r_soi,
            omega1: 0.0, omega2: 0.0, big_omega2: 0.0,
        });
    }

    let n = 3600;
    let e1s: Vec<f64> = (0..n).map(|i| -PI + TAU * i as f64 / n as f64).collect();

    // Benchmark only a representative subset (Hohmann Ω₂=0° and worst-case Ω₂=150°)
    {
        let mut group = c.benchmark_group("encounter_e2_estimate");
        group.warm_up_time(Duration::from_millis(500));
        group.measurement_time(Duration::from_millis(2000));

        let bench_indices: Vec<usize> = cases.iter().enumerate()
            .filter(|(_, c)| c.label == format!("e1={e1_h:.3}") || c.label == "Ω2=150°")
            .map(|(i, _)| i)
            .collect();

        for &idx in &bench_indices {
            let case = &cases[idx];
            let inc2 = case.inc2_deg.to_radians();
            let b1 = case.a1 * (1.0 - case.e1 * case.e1).abs().sqrt();
            let b2 = case.a2 * (1.0 - case.e2 * case.e2).sqrt();
            let mat_a = rotation_matrix(0.0, case.omega1, 0.0);
            let mat_b = rotation_matrix(inc2, case.omega2, case.big_omega2);
            let m = scaled_coupling_matrix(case.a1, b1, case.a2, b2, &mat_a, &mat_b);
            let r_soi_sq = case.r_soi * case.r_soi;

            let mut ei = 0usize;

            group.bench_function(format!("atan2/{}", case.label), |b| {
                b.iter(|| {
                    let ea1 = e1s[ei % n];
                    ei = ei.wrapping_add(1);
                    criterion::black_box(atan2_projection(
                        ea1, case.a1, case.e1, b1, case.a2, case.e2, b2, &mat_a, &mat_b,
                    ))
                });
            });

            group.bench_function(format!("cross/{}", case.label), |b| {
                b.iter(|| {
                    let ea1 = e1s[ei % n];
                    ei = ei.wrapping_add(1);
                    criterion::black_box(cross_term_estimate(
                        ea1, case.e1, &m,
                    ))
                });
            });

            for niter in [1, 2, 3] {
                group.bench_function(format!("newton{niter}/{}", case.label), |b| {
                    b.iter(|| {
                        let ea1 = e1s[ei % n];
                        ei = ei.wrapping_add(1);
                        let ea2_hint = atan2_projection(
                            ea1, case.a1, case.e1, b1, case.a2, case.e2, b2, &mat_a, &mat_b,
                        );
                        criterion::black_box(newton_estimate(
                            ea1, ea2_hint, case.a1, case.e1, case.a2, case.e2, &m, niter,
                        ))
                    });
                });
            }

            group.bench_function(format!("quartic/{}", case.label), |b| {
                b.iter(|| {
                    let ea1 = e1s[ei % n];
                    ei = ei.wrapping_add(1);
                    let ea2_hint = atan2_projection(
                        ea1, case.a1, case.e1, b1, case.a2, case.e2, b2, &mat_a, &mat_b,
                    );
                    criterion::black_box(quartic_estimate(
                        ea1, ea2_hint, case.a1, case.e1, case.a2, case.e2, &m, r_soi_sq,
                    ))
                });
            });
        }

        group.finish();
    }

    // Print full accuracy comparison against ground truth (golden-section minimizer)
    println!("\n=== Varying eccentricity (Pluto i=17°) ===");
    println!("{:>12}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}", "case", "proj°", "cross°", "N1°", "N2°", "N3°", "qrt°");
    println!("{}", "-".repeat(70));

    for case in &cases {
        let inc2 = case.inc2_deg.to_radians();
        let b1 = case.a1 * (1.0 - case.e1 * case.e1).abs().sqrt();
        let b2 = case.a2 * (1.0 - case.e2 * case.e2).sqrt();
        let mat_a = rotation_matrix(0.0, case.omega1, 0.0);
        let mat_b = rotation_matrix(inc2, case.omega2, case.big_omega2);
        let m = scaled_coupling_matrix(case.a1, b1, case.a2, b2, &mat_a, &mat_b);
        let r_soi_sq = case.r_soi * case.r_soi;

        let mut max_err_proj = 0.0_f64;
        let mut max_err_cross = 0.0_f64;
        let mut max_err_n = [0.0_f64; 3];
        let mut max_err_qrt = 0.0_f64;

        for &ea1 in &e1s {
            let ea2_true = global_min(|ea2| {
                f_value(ea1, ea2, case.a1, case.e1, case.a2, case.e2, &m, r_soi_sq)
            });

            let ea2_proj = atan2_projection(ea1, case.a1, case.e1, b1, case.a2, case.e2, b2, &mat_a, &mat_b);
            let ea2_cross = cross_term_estimate(ea1, case.e1, &m);
            let ea2_qrt = quartic_estimate(ea1, ea2_proj, case.a1, case.e1, case.a2, case.e2, &m, r_soi_sq);

            max_err_proj = max_err_proj.max(angle_error(ea2_proj, ea2_true));
            max_err_cross = max_err_cross.max(angle_error(ea2_cross, ea2_true));
            max_err_qrt = max_err_qrt.max(angle_error(ea2_qrt, ea2_true));

            // Newton starting from cross-term estimate
            for niter in 0..3 {
                let ea2_n = newton_estimate(ea1, ea2_cross, case.a1, case.e1, case.a2, case.e2, &m, niter + 1);
                max_err_n[niter] = max_err_n[niter].max(angle_error(ea2_n, ea2_true));
            }
        }

        println!(
            "{:>12}  {:>8.2}  {:>8.2}  {:>8.4}  {:>8.4}  {:>8.4}  {:>8.4}",
            case.label,
            max_err_proj.to_degrees(),
            max_err_cross.to_degrees(),
            max_err_n[0].to_degrees(),
            max_err_n[1].to_degrees(),
            max_err_n[2].to_degrees(),
            max_err_qrt.to_degrees(),
        );
    }
}

criterion_group!(benches, bench_encounter);
criterion_main!(benches);
