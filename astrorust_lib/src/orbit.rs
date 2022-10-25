#![allow(non_snake_case)]

use core::f32::consts::TAU;
use nalgebra::{Matrix3x2, Vector2, Vector3};

#[derive(Builder, Getters, Setters, Debug)]
#[builder(build_fn(validate = "Self::validate"))]
pub struct Orbit {
	/// Standard gravitational parameter
	#[getset(get = "pub")]
	#[builder(setter(name = "std_grav_param"))]
	mu: f32,

	/// Semi-major axis
	#[getset(get = "pub")]
	#[builder(setter(name = "semi_major_axis"))]
	a: f32,

	/// Eccentricity
	#[getset(get = "pub")]
	#[builder(default, setter(name = "eccentricity"))]
	e: f32,

	/// Inclination
	#[getset(get = "pub")]
	#[builder(default, setter(name = "inclination"))]
	i: f32,

	/// Longitude of the ascending node
	#[getset(get = "pub")]
	#[builder(default, setter(name = "long_of_asc_node"))]
	Omega: f32,

	/// Argument of periapsis
	#[getset(get = "pub")]
	#[builder(default, setter(name = "arg_of_periapsis"))]
	omega: f32,

	/// Mean anomaly at T=0
	#[getset(get = "pub")]
	#[builder(default, setter(name = "mean_anomaly_at_t0"))]
	M0: f32,

	/// Orbital period
	#[getset(get = "pub")]
	#[builder(setter(skip))]
	period: f32, // orbital period

	// pre-computed
	#[builder(setter(skip), default = "(self.mu.unwrap() / self.a.unwrap()).sqrt()")]
	speed_root: f32, // sqrt(mu / a)

	#[builder(setter(skip), default = "(self.mu.unwrap() / self.a.unwrap().powi(3)).sqrt()")]
	mean_motion: f32, // sqrt(mu / a^3)

	#[builder(setter(skip), default = "(1.0 - self.e.unwrap().powi(2)).sqrt()")]
	e_root: f32, // sqrt(1 - e^2)

	/// Matrix to transform vectors from orbital to ecliptic coordinates
	#[builder(
		setter(skip),
		default = "Orbit::compute_orb_to_ecl(self.i.unwrap(), self.Omega.unwrap(), self.omega.unwrap())"
	)]
	orb_to_ecl: Matrix3x2<f32>,
}

impl OrbitBuilder {
	fn validate(&self) -> Result<(), String> {
		Ok(())
	}
}

impl Orbit {
	pub fn r_from_nu(&self, nu: f32) -> Vector3<f32> {
		let (e, e_root) = (self.e, self.e_root);
		let (sin_nu, cos_nu) = nu.sin_cos();
		let inv_denominator = 1.0 / (e.mul_add(cos_nu, 1.0));
		let sin_E = e_root * sin_nu * inv_denominator;
		let cos_E = (e + cos_nu) * inv_denominator;
		self.r_from_sin_cos_E((sin_E, cos_E))
	}

	pub fn r_and_v_from_nu(&self, nu: f32) -> (Vector3<f32>, Vector3<f32>) {
		let (e, e_root) = (self.e, self.e_root);
		let (sin_nu, cos_nu) = nu.sin_cos();
		let inv_denominator = 1.0 / (e.mul_add(cos_nu, 1.0));
		let sin_E = e_root * sin_nu * inv_denominator;
		let cos_E = (e + cos_nu) * inv_denominator;
		(self.r_from_sin_cos_E((sin_E, cos_E)), self.v_from_sin_cos_E((sin_E, cos_E)))
	}

	pub fn r_from_E(&self, E: f32) -> Vector3<f32> {
		self.r_from_sin_cos_E(E.sin_cos())
	}

	/// Position and velocity from eccentric anomaly
	pub fn r_and_v_from_E(&self, E: f32) -> (Vector3<f32>, Vector3<f32>) {
		let E_sin_cos = E.sin_cos();
		(self.r_from_sin_cos_E(E_sin_cos), self.v_from_sin_cos_E(E_sin_cos))
	}

	pub fn r_from_t(&self, t: f32) -> Vector3<f32> {
		self.r_from_E(self.E_from_M(self.M_from_t(t)))
	}

	pub fn r_and_v_from_t(&self, t: f32) -> (Vector3<f32>, Vector3<f32>) {
		self.r_and_v_from_E(self.E_from_M(self.M_from_t(t)))
	}

	fn r_from_sin_cos_E(&self, (sin_E, cos_E): (f32, f32)) -> Vector3<f32> {
		let (a, e, e_root) = (self.a, self.e, self.e_root);
		let r_orb = a * Vector2::new(cos_E - e, e_root * sin_E);
		self.orb_to_ecl * r_orb
	}

	fn v_from_sin_cos_E(&self, (sin_E, cos_E): (f32, f32)) -> Vector3<f32> {
		let (e, e_root) = (self.e, self.e_root);
		let v_mult: f32 = self.speed_root / (1.0 - e * cos_E);
		let v_orb = -v_mult * Vector2::new(sin_E, e_root * cos_E);
		self.orb_to_ecl * v_orb
	}

	fn E_from_M(&self, _M: f32) -> f32 {
		todo!()
	}

	fn M_from_t(&self, t: f32) -> f32 {
		(t / self.period) * TAU
	}

	fn compute_orb_to_ecl(i: f32, Omega: f32, omega: f32) -> Matrix3x2<f32> {
		let (sin_i, cos_i) = i.sin_cos();
		let (sin_Omega, cos_Omega) = Omega.sin_cos();
		let (sin_omega, cos_omega) = omega.sin_cos();
		let arr = [
			cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i,
			-cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i,
			sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i,
			-sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i,
			sin_omega * sin_i,
			cos_omega * sin_i,
		];
		Matrix3x2::from_row_slice(&arr)
	}
}

#[cfg(test)]
mod tests {
	#[test]
	fn test() {}
}
