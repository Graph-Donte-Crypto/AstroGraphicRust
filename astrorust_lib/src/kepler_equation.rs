use std::f64::consts::PI;

use crate::angle::{Angle, EccAnomaly, MeanAnomaly};

/// Elliptic Kepler's equation solver (about 40% faster than naive implementation
/// with Newton's method) Based on the paper by F. Landis Markley "Kepler
/// equation solver" (1995)
pub fn solve_kepler_householder_pade_elliptic(e: f64, M: MeanAnomaly) -> EccAnomaly {
    let M = M.normalize();
    let E = pade_initial_guess(e, M.into()).as_rad();
    let M = M.as_rad();

    // 3rd order Householder's method
    let (sin_E, cos_E) = E.sin_cos();
    let f = E - e * sin_E - M;
    let f_sqr = f * f;
    let Df = 1.0 - e * cos_E;
    let Df_sqr = Df * Df;
    let DDf = e * sin_E;
    let DDDf = e * cos_E;
    let E = E
        - (6.0 * f * Df_sqr - 3.0 * f_sqr * DDf)
            / (6.0 * Df_sqr * Df - 6.0 * f * Df * DDf + f_sqr * DDDf);

    Angle::from_rad(E).into()
}

pub fn solve_kepler_newton_pade(e: f64, M: MeanAnomaly) -> EccAnomaly {
    let M = M.normalize().into();
    let mut E0 = pade_initial_guess(e, M).as_rad();
    let M = M.as_rad();

    let mut E = E0;
    for _ in 0..15 {
        let (sin_E0, cos_E0) = E.sin_cos();
        E = E0 - (E0 - e * sin_E0 - M) / (1.0 - e * cos_E0);
        if (E - E0).abs() < f64::EPSILON {
            break;
        }
        E0 = E;
    }
    Angle::from_rad(E).into()
}

// fn solve_kepler_newton_simple(e: f64, M: f64) -> f64 {
//     let M = normalize_radians(M);
//     let mut E0 = if M < 0.0 { M - e } else { M + e };

//     let mut E = E0;
//     for _ in 0..15 {
//         let (sin_E0, cos_E0) = E.sin_cos();
//         E = E0 - (E0 - e * sin_E0 - M) / (1.0 - e * cos_E0);
//         if (E - E0).abs() < f64::EPSILON {
//             break;
//         }
//         E0 = E;
//     }
//     E
// }

fn pade_initial_guess(e: f64, M: MeanAnomaly) -> EccAnomaly {
    const ALPHA_MULT: f64 = 1.0 / (PI * PI - 6.0);

    let M = M.as_rad();

    // Solving cubic equation based on Pade approximant for sin(E) (more info in
    // the paper)
    let alpha = (3.0 * PI * PI + 1.6 * PI * (PI - M.abs()) / (1.0 + e)) * ALPHA_MULT;
    let d = 3.0 * (1.0 - e) + alpha * e;
    let M_sqr = M * M;
    let alpha_d = alpha * d;
    let q = 2.0 * alpha_d * (1.0 - e) - M_sqr;
    let q_sqr = q * q;
    let r = 3.0 * alpha_d * (d - 1.0 + e) * M + M_sqr * M;
    let pre_w = r.abs() + (q_sqr * q + r * r).sqrt();
    let w = (pre_w * pre_w).cbrt();

    // Finally, get our initial guess
    Angle::from_rad(((2.0 * r * w) / (w * w + w * q + q_sqr) + M) / d).into()
}

#[cfg(test)]
mod tests {
    use crate::angle::{Angle, EccAnomaly, MeanAnomaly};
    use crate::kepler_equation::solve_kepler_householder_pade_elliptic;

    #[test]
    fn highly_elliptic() {
        let e = 0.99999999999;
        let expected_E: EccAnomaly = Angle::from_rad(0.0843533).into();
        let M: MeanAnomaly = Angle::from_rad(expected_E.as_rad() - e * expected_E.sin()).into();
        let E = solve_kepler_householder_pade_elliptic(e, M);
        dbg!(E, expected_E);
        assert!((E.as_rad() - expected_E.as_rad()).abs() < 1e-13);
    }

    #[test]
    fn circular() {
        let e = 0.0;
        let M: MeanAnomaly = Angle::from_rad(3.14).into();
        let expected: EccAnomaly = Angle::from_rad(3.14).into();
        let solution = solve_kepler_householder_pade_elliptic(e, M);
        dbg!(solution, expected);
        assert!((solution.as_rad() - expected.as_rad()).abs() < 1e-15);
    }
}
