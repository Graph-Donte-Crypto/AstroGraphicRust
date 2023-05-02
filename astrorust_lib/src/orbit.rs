#![allow(non_snake_case)]
use std::f64::consts::{TAU, PI};
use nalgebra::{Matrix3x2, Vector2};

#[derive(Builder, CopyGetters, Debug)]
#[builder(build_fn(validate = "Self::validate"))]
pub struct Orbit2D {
	/// Standard gravitational parameter
	#[getset(get_copy = "pub")]
	#[builder(setter(name = "std_grav_param"))]
	mu: f32,

	/// Semi-major axis
	#[getset(get_copy = "pub")]
	#[builder(setter(name = "semi_major_axis"))]
	a: f32,

	/// Eccentricity
	#[getset(get_copy = "pub")]
	#[builder(default, setter(name = "eccentricity"))]
	e: f32,

	/// Mean anomaly at T=0
	#[getset(get_copy = "pub")]
	#[builder(default, setter(name = "mean_anomaly_at_t0"))]
	M0: f32,

	// pre-computed
	#[builder(setter(skip), default = "(self.mu.unwrap() / self.a.unwrap()).sqrt()")]
	speed_root: f32, // sqrt(mu / a)

	#[builder(setter(skip), default = "(1.0 - self.e.unwrap().powi(2)).sqrt()")]
	e_root: f32, // sqrt(1 - e^2)

	#[builder(setter(skip), default = "(self.mu.unwrap() / self.a.unwrap().powi(3)).sqrt()")]
	mean_motion: f32, // sqrt(mu / a^3)

	/// Orbital period
	#[getset(get_copy = "pub")]
	#[builder(setter(skip))]
	period: f32,
}

#[derive(Builder, CopyGetters, Debug)]
#[builder(build_fn(validate = "Self::validate"))]
pub struct Orbit {
	/// Inclination
	#[getset(get_copy = "pub")]
	#[builder(default, setter(name = "inclination"))]
	i: f32,

	/// Longitude of the ascending node
	#[getset(get_copy = "pub")]
	#[builder(default, setter(name = "long_of_asc_node"))]
	Omega: f32,

	/// Argument of periapsis
	#[getset(get_copy = "pub")]
	#[builder(default, setter(name = "arg_of_periapsis"))]
	omega: f32,

	/// Matrix to transform vectors from orbital to ecliptic coordinates
	#[builder(
		setter(skip),
		default = "Orbit::compute_orb_to_ecl(self.i.unwrap(), self.Omega.unwrap(), self.omega.unwrap())"
	)]
	orb_to_ecl: Matrix3x2<f32>,
}

impl Orbit2DBuilder {
	fn validate(&self) -> Result<(), String> {
		Ok(())
	}
}

impl Orbit2D {
	pub fn r_from_nu(&self, nu: f32) -> Vector2<f32> {
		let (e, e_root) = (self.e, self.e_root);
		let (sin_nu, cos_nu) = nu.sin_cos();
		let inv_denominator = 1.0 / (e.mul_add(cos_nu, 1.0));
		let sin_E = e_root * sin_nu * inv_denominator;
		let cos_E = (e + cos_nu) * inv_denominator;
		self.r_from_sin_cos_E((sin_E, cos_E))
	}

	pub fn r_and_v_from_nu(&self, nu: f32) -> (Vector2<f32>, Vector2<f32>) {
		let (e, e_root) = (self.e, self.e_root);
		let (sin_nu, cos_nu) = nu.sin_cos();
		let inv_denominator = 1.0 / (e.mul_add(cos_nu, 1.0));
		let sin_E = e_root * sin_nu * inv_denominator;
		let cos_E = (e + cos_nu) * inv_denominator;
		(self.r_from_sin_cos_E((sin_E, cos_E)), self.v_from_sin_cos_E((sin_E, cos_E)))
	}

	pub fn r_from_E(&self, E: f32) -> Vector2<f32> {
		self.r_from_sin_cos_E(E.sin_cos())
	}

	/// Position and velocity from eccentric anomaly
	pub fn r_and_v_from_E(&self, E: f32) -> (Vector2<f32>, Vector2<f32>) {
		let sin_cos_E = E.sin_cos();
		(self.r_from_sin_cos_E(sin_cos_E), self.v_from_sin_cos_E(sin_cos_E))
	}

	pub fn r_from_t(&self, t: f32) -> Vector2<f32> {
		self.r_from_E(self.E_from_M(self.M_from_t(t)))
	}

	pub fn r_and_v_from_t(&self, t: f32) -> (Vector2<f32>, Vector2<f32>) {
		self.r_and_v_from_E(self.E_from_M(self.M_from_t(t)))
	}

	fn r_from_sin_cos_E(&self, (sin_E, cos_E): (f32, f32)) -> Vector2<f32> {
		let (e, e_root) = (self.e, self.e_root);
		self.a * Vector2::new(cos_E - e, e_root * sin_E)
	}

	fn v_from_sin_cos_E(&self, (sin_E, cos_E): (f32, f32)) -> Vector2<f32> {
		let (e, e_root) = (self.e, self.e_root);
		let v_mult: f32 = self.speed_root / (1.0 - e * cos_E);
		-v_mult * Vector2::new(sin_E, e_root * cos_E)
	}

	fn E_from_M(&self, M: f32) -> f32 {
		std::f32::NAN
	}

	fn M_from_t(&self, t: f32) -> f32 {
		self.mean_motion * t
	}
}

impl OrbitBuilder {
	fn validate(&self) -> Result<(), String> {
		Ok(())
	}
}

impl Orbit {
	fn compute_orb_to_ecl(i: f32, Omega: f32, omega: f32) -> Matrix3x2<f32> {
		let (sin_i, cos_i) = i.sin_cos();
		let (sin_Omega, cos_Omega) = Omega.sin_cos();
		let (sin_omega, cos_omega) = omega.sin_cos();
		Matrix3x2::new(
			cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i,
			-cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i,
			sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i,
			-sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i,
			sin_omega * sin_i,
			cos_omega * sin_i,
		)
	}
}

/// Elliptic Kepler's equation solver (about 40% faster than naive implementation
/// with Newton's method) Based on the paper by F. Landis Markley "Kepler
/// equation solver" (1995)
fn solve_kepler_householder_pade(e: f64, M: f64) -> f64 {
	let M = normalize_radians(M);
	let E = pade_initial_guess(e, M);

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

	E
}

fn solve_kepler_newton_pade(e: f64, M: f64) -> f64 {
	let M = normalize_radians(M);
	let mut E0 = pade_initial_guess(e, M);

	let mut E = E0;
	for _ in 0..15 {
		let (sin_E0, cos_E0) = E.sin_cos();
		E = E0 - (E0 - e * sin_E0 - M) / (1.0 - e * cos_E0);
		if (E - E0).abs() < f64::EPSILON {
			break;
		}
		E0 = E;
	}
	E
}

fn solve_kepler_newton_simple(e: f64, M: f64) -> f64 {
	let M = normalize_radians(M);
	let mut E0 = if M < 0.0 { M - e } else { M + e };

	let mut E = E0;
	for _ in 0..15 {
		let (sin_E0, cos_E0) = E.sin_cos();
		E = E0 - (E0 - e * sin_E0 - M) / (1.0 - e * cos_E0);
		if (E - E0).abs() < f64::EPSILON {
			break;
		}
		E0 = E;
	}
	E
}

/// Normalize angle to [-PI; +PI]
fn normalize_radians(angle: f64) -> f64 {
	// Wrap M to [-PI; +PI]
	let M = (angle + PI) % TAU;
	if M < 0.0 {
		M + PI
	} else {
		M - PI
	}
}

fn pade_initial_guess(e: f64, M: f64) -> f64 {
	const ALPHA_MULT: f64 = 1.0 / (PI * PI - 6.0);

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
	((2.0 * r * w) / (w * w + w * q + q_sqr) + M) / d
}

#[cfg(test)]
mod tests {
	#[test]
	fn test() {}
}
