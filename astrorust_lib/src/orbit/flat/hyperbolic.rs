use nalgebra::Vector2;
use roots::Roots;

use crate::angle::{HypAnomaly, IntoAnomaly, MeanAnomaly, TrueAnomaly};
use crate::state_vectors::{StateVectorTypes, StateVectors};
use crate::time::Time;

use super::Orbit2D;

#[derive(Debug)]
pub struct HyperbolicOrbit(Orbit2D);

impl HyperbolicOrbit {
    fn r_from_sinh_cosh_H(&self, (sinh_H, cosh_H): (f64, f64)) -> Vector2<f64> {
        let Orbit2D { a, e, e_root, .. } = self.0;
        a * Vector2::new(cosh_H - e, -e_root * sinh_H)
    }

    fn v_from_sinh_cosh_H(&self, (sinh_H, cosh_H): (f64, f64)) -> Vector2<f64> {
        let Orbit2D { e, e_root, speed_root, .. } = self.0;
        let v_mult = -speed_root / (1.0 - e * cosh_H);
        v_mult * Vector2::new(-sinh_H, e_root * cosh_H)
    }

    /// sinh and cosh of hyperbolic anomaly
    fn sinh_cosh_H_from_nu(&self, nu: TrueAnomaly, e: f64) -> (f64, f64) {
        // FIXME: use hyperbolic
        let (sin_nu, cos_nu) = nu.sin_cos();
        let inv_denominator = e.mul_add(cos_nu, 1.0).recip();
        let sin_E = self.0.e_root * sin_nu * inv_denominator;
        let cos_E = (e + cos_nu) * inv_denominator;
        (sin_E, cos_E)
    }
}

impl StateVectorTypes for HyperbolicOrbit {
    type Position = Vector2<f64>;
    type Velocity = Vector2<f64>;
}

impl StateVectors<MeanAnomaly> for HyperbolicOrbit {
    fn position(&self, M: MeanAnomaly) -> Self::Position {
        let H: HypAnomaly = M.into_anomaly(self.0.e);
        self.r_from_sinh_cosh_H(H.sinh_cosh())
    }

    fn velocity(&self, M: MeanAnomaly) -> Self::Velocity {
        let H: HypAnomaly = M.into_anomaly(self.0.e);
        self.v_from_sinh_cosh_H(H.sinh_cosh())
    }

    fn position_and_velocity(&self, M: MeanAnomaly) -> (Self::Position, Self::Velocity) {
        let H: HypAnomaly = M.into_anomaly(self.0.e);
        let sinh_cosh_H = H.sinh_cosh();
        (self.r_from_sinh_cosh_H(sinh_cosh_H), self.v_from_sinh_cosh_H(sinh_cosh_H))
    }
}

impl StateVectors<HypAnomaly> for HyperbolicOrbit {
    fn position(&self, H: HypAnomaly) -> Self::Position {
        self.r_from_sinh_cosh_H(H.sinh_cosh())
    }

    fn velocity(&self, H: HypAnomaly) -> Self::Velocity {
        self.v_from_sinh_cosh_H(H.sinh_cosh())
    }

    fn position_and_velocity(&self, H: HypAnomaly) -> (Self::Position, Self::Velocity) {
        let sinh_cosh_H = H.sinh_cosh();
        (self.r_from_sinh_cosh_H(sinh_cosh_H), self.v_from_sinh_cosh_H(sinh_cosh_H))
    }
}

impl StateVectors<TrueAnomaly> for HyperbolicOrbit {
    fn position(&self, nu: TrueAnomaly) -> Self::Position {
        self.r_from_sinh_cosh_H(self.sinh_cosh_H_from_nu(nu, self.0.e))
    }

    fn velocity(&self, nu: TrueAnomaly) -> Self::Velocity {
        self.v_from_sinh_cosh_H(self.sinh_cosh_H_from_nu(nu, self.0.e))
    }

    fn position_and_velocity(&self, nu: TrueAnomaly) -> (Self::Position, Self::Velocity) {
        let sinh_cosh_H = self.sinh_cosh_H_from_nu(nu, self.0.e);
        (self.r_from_sinh_cosh_H(sinh_cosh_H), self.v_from_sinh_cosh_H(sinh_cosh_H))
    }
}

impl StateVectors<Time> for HyperbolicOrbit {
    fn position(&self, t: Time) -> Self::Position {
        self.position(self.0.M_from_t(t))
    }

    fn velocity(&self, t: Time) -> Self::Velocity {
        self.velocity(self.0.M_from_t(t))
    }

    fn position_and_velocity(&self, t: Time) -> (Self::Position, Self::Position) {
        self.position_and_velocity(self.0.M_from_t(t))
    }
}

// const CUBIC_DELTA_THRESHOLD: f64 = 1.0e-6;

// From eq. 4 in the B. Wu et all paper
// Max value in each interval
const PADE_ECCENTRIC_ANOMALY_THRESHOLDS: [f64; 15] = [
    40.0 / 8.0,
    38.0 / 8.0,
    34.0 / 8.0,
    30.0 / 8.0,
    26.0 / 8.0,
    22.0 / 8.0,
    18.0 / 8.0,
    15.0 / 8.0,
    13.0 / 8.0,
    11.0 / 8.0,
    9.0 / 8.0,
    7.0 / 8.0,
    5.0 / 8.0,
    3.0 / 8.0,
    29.0 / 200.0,
];

const PADE_ORDERS: [f64; 15] = [
    10.0 / 2.0,
    9.0 / 2.0,
    8.0 / 2.0,
    7.0 / 2.0,
    6.0 / 2.0,
    5.0 / 2.0,
    8.0 / 4.0,
    7.0 / 4.0,
    6.0 / 4.0,
    5.0 / 4.0,
    4.0 / 4.0,
    3.0 / 4.0,
    2.0 / 4.0,
    1.0 / 4.0,
    0.0 / 4.0,
];

fn hyperbolic_kepler_equation(eccentricity: f64, eccentric_anomaly: f64) -> f64 {
    eccentricity * f64::sinh(eccentric_anomaly) - eccentric_anomaly
}

fn pade_approximation(ec: f64, mh: f64, a: f64) -> [f64; 4] {
    let ex = f64::exp(a);
    let enx = ex.recip();
    let sa = (ex - enx) * 0.5; // sinh(a)
    let ca = (ex + enx) * 0.5; // cosh(a)

    let d1 = ca.powi(2) + 3.0;
    let d2 = sa.powi(2) + 4.0;
    let p1 = ca * (3.0 * ca.powi(2) + 17.0) / (5.0 * d1);
    let p2 = sa * (3.0 * sa.powi(2) + 28.0) / (20.0 * d2);
    let p3 = ca * (ca.powi(2) + 27.0) / (60.0 * d1);
    let q1 = -2.0 * ca * sa / (5.0 * d1);
    let q2 = (sa.powi(2) - 4.0) / (20.0 * d2);
    let coefficient_f3 = ec * p3 - q2;
    let coefficient_f2 = ec * p2 - (mh + a) * q2 - q1;
    let coefficient_f1 = ec * p1 - (mh + a) * q1 - 1.0;
    let coefficient_f0 = ec * sa - mh - a;
    [coefficient_f3, coefficient_f2, coefficient_f1, coefficient_f0]
}

// fn solve_cubic(coeff: [f64; 4], mh: f64, ec: f64) -> f64 {
//     let mut x = mh / (ec - 1.0); // starting value from series expansion of HKE
//
//     loop {
//         // halley's method
//         let f = ((coeff[0] * x + coeff[1]) * x + coeff[2]) * x + coeff[3];
//         let f_prime = (3.0 * coeff[0] * x + 2.0 * coeff[1]) * x + coeff[2];
//         let f_prime_prime = 6.0 * coeff[0] * x + 2.0 * coeff[1];
//         let delta = -2.0 * f * f_prime / (2.0 * f_prime * f_prime - f * f_prime_prime);
//         if delta.abs() < CUBIC_DELTA_THRESHOLD {
//             break;
//         }
//         x += delta;
//     }
//     x
// }

fn solve_cubic(coeff: [f64; 4], _mh: f64, _ec: f64) -> f64 {
    match roots::find_roots_cubic(coeff[0], coeff[1], coeff[2], coeff[3]) {
        // Roots::No(_) => unreachable!("No roots found!"),
        Roots::One([x]) => x,
        // Roots::Two(roots) => {
        //     dbg!("Two roots");
        //     let x0 = mh / (ec - 1.0); // starting value from series expansion of HKE
        //     *roots
        //         .iter()
        //         .reduce(|l, r| if (r - x0).abs() < (l - x0).abs() { r } else { l })
        //         .unwrap()
        // }
        // Roots::Three(roots) => {
        //     dbg!("Three roots");
        //     let x0 = mh / (ec - 1.0); // starting value from series expansion of HKE
        //     *roots
        //         .iter()
        //         .reduce(|l, r| if (r - x0).abs() < (l - x0).abs() { r } else { l })
        //         .unwrap()
        // }
        // Roots::Four(_) => unreachable!("Cubic equations have max 3 roots"),
        _ => unreachable!(),
    }
}

/// ## Example
/// ```rs
/// use rust_kepler_solver::hyperbola::HyperbolaSolver;
///
/// fn example_hyperbola() {
///     let eccentricity = 1.0;
///     let solver = HyperbolaSolver::new(eccentricity);
///     println!("{}", solver.solve(1.2));
///     println!("{}", solver.solve(100.0));
/// }
/// ```
#[derive(Debug, Clone)]
pub struct HyperbolaSolver {
    eccentricity: f64,
    pade_mean_anomaly_thresholds: [f64; 15],
}

impl HyperbolaSolver {
    pub fn new(eccentricity: f64) -> Self {
        let pade_mean_anomaly_thresholds = PADE_ECCENTRIC_ANOMALY_THRESHOLDS
            .map(|eccentric_anomaly| hyperbolic_kepler_equation(eccentricity, eccentric_anomaly));
        Self { eccentricity, pade_mean_anomaly_thresholds }
    }

    /// Works with all values of mean anomaly 0 to infinity
    pub fn solve(&self, mean_anomaly: f64) -> f64 {
        // Solver assumes mean anomaly > 0
        // The equation is symmetric, so for mean anomaly < 0, we just flip the sign o the output
        let ec = self.eccentricity;
        let mh = mean_anomaly.abs();

        let f0 = if mh <= self.pade_mean_anomaly_thresholds[0] {
            // For mh < 5 we use a 'piecewise pade approximation' to get the starting estimate
            let i = self
                .pade_mean_anomaly_thresholds
                .iter()
                .position(|x| mh >= *x)
                .unwrap_or(self.pade_mean_anomaly_thresholds.len() - 1);
            let a = PADE_ORDERS[i];
            let coefficients = pade_approximation(ec, mh, a);

            solve_cubic(coefficients, mh, ec) + a
        } else {
            // For mh >= 5, we can use this... thing that I copied from the above paper
            // I have no idea how it works, but it works very very very well
            let exp = 2.0 * mh / ec;
            let exp_recip = exp.recip();

            let sa = 0.5 * (exp - exp_recip);
            let ca = 0.5 * (exp + exp_recip);

            let fa = f64::ln(exp);

            let var_name = (ec.powi(2) / (4.0 * mh) + fa) / (ec * ca - 1.0);
            let ec_sa_ec_ca = ec * sa / (ec * ca - 1.0);
            let top = 6.0 * var_name + 3.0 * ec_sa_ec_ca * var_name.powi(2);
            let bottom =
                6.0 + 6.0 * ec_sa_ec_ca * var_name + (ec * ca / (ec * ca - 1.0)) * var_name.powi(2);
            let delta = top / bottom;
            fa + delta
        };

        // Halley method

        let ex = f64::exp(f0);
        let enx = ex.recip();
        let f0_sinh = (ex - enx) * 0.5; // sinh(a)
        let f0_cosh = (ex + enx) * 0.5; // cosh(a)
                                        //
        let f = ec * f0_sinh - f0 - mh;
        let f_prime = ec * f0_cosh - 1.0;
        let f_prime_prime = f_prime + 1.0;

        let f_prime_recip = f_prime.recip();

        let f1 = f0 - (2.0 * f * f_prime_recip) / (2.0 - f * f_prime_prime * f_prime_recip.powi(2));

        f1 * mean_anomaly.signum()
    }
}
const BISECTION_ITERATIONS: usize = 64;

fn bisection(function: &impl Fn(f64) -> f64, min: f64, max: f64) -> f64 {
    let mut low = min;
    let mut high = max;
    let mut mid = (min + max) / 2.0;
    for _ in 0..BISECTION_ITERATIONS {
        if function(mid).is_sign_positive() && function(low).is_sign_positive()
            || function(mid).is_sign_negative() && function(low).is_sign_negative()
        {
            low = mid;
        } else {
            high = mid;
        }
        mid = (low + high) / 2.0;
    }
    mid
}

#[cfg(test)]
mod test {
    use super::{bisection, HyperbolaSolver};
    use std::f64::consts::PI;

    #[test]
    fn test_bisection_1() {
        let function = |x: f64| f64::sin(x);
        assert!(bisection(&function, -PI / 2.0, PI / 2.0).abs() < 1.0e-2);
    }

    #[test]
    fn test_bisection_2() {
        let function = |x: f64| x.powi(2) - 4.0;
        assert!(bisection(&function, 0.0, 10.0).abs() - 2.0 < 1.0e-2);
        assert!(bisection(&function, -10.0, 0.0).abs() - 2.0 < 1.0e-2);
    }

    fn solve_with_bisection(e: f64, m: f64) -> f64 {
        // We don't care about speed here, so just use as wide a range as possible
        let f = |eccentric_anomaly: f64| e * f64::sinh(eccentric_anomaly) - eccentric_anomaly - m;
        bisection(&f, -100000.0, 100000.0)
    }

    #[test]
    fn test_hyperbola() {
        let eccentricites: Vec<f64> =
            (1..999).map(|x| 1.0 + f64::powi(x as f64, 2) / 1000.0).collect();
        let mean_anomalies: Vec<f64> =
            (0..10000).map(|x| f64::powi(x as f64, 2) / 10000.0).collect();

        for e in &eccentricites {
            let solver = HyperbolaSolver::new(*e);
            for m in &mean_anomalies {
                let expected = solve_with_bisection(*e, *m);
                let actual = solver.solve(*m);
                let difference = if actual.abs() < 1.0e-5 {
                    expected - actual
                } else {
                    (expected - actual) / actual
                }
                .abs();
                if difference > 1.0e-4 {
                    dbg!(expected, actual, difference, e, m);
                    panic!()
                }
            }
        }
    }
}

// #[cfg(test)]
// mod tests {
//     use crate::angle::{Angle, TrueAnomaly};
//     use crate::orbit::flat::elliptic::HyperbolicOrbit;
//     use crate::orbit::flat::Orbit2DBuilder;
//     use crate::state_vectors::StateVectors;

//     // #[test]
//     // fn kerbin() {
//     //     let orbit = Orbit2DBuilder::default()
//     //         .std_grav_param(1.1723328e9)
//     //         .semi_major_axis(1.3599840256e7)
//     //         .mean_anomaly_at_t0(Angle::from_rad(3.14).into())
//     //         .build()
//     //         .unwrap();
//     //     let orbit = HyperbolicOrbit(orbit);
//     //     dbg!(&orbit);
//     //     let (r, v) = orbit.position_and_velocity(TrueAnomaly::from(Angle::from_rad(0.0)));
//     //     assert!((r.norm() - 1.3599840256e7).abs() < 1e-8);
//     //     assert!((v.norm() - 9.285).abs() < 1e-3);
//     // }
// }
