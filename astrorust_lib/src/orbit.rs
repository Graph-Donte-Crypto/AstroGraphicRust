#![allow(non_snake_case)]

use core::f32::consts::TAU;
use nalgebra::{Matrix3x2, Vector2, Vector3};

#[derive(Builder, Getters, Setters, Debug)]
pub struct Orbit {
	#[getset(get = "pub")]
	mu: f32,

	#[getset(get = "pub")]
	a: f32,

	#[getset(get = "pub")]
	#[builder(default)]
	e: f32,

	#[getset(get = "pub")]
	#[builder(default)]
	i: f32,

	#[getset(get = "pub")]
	#[builder(default)]
	Omega: f32,

	#[getset(get = "pub")]
	#[builder(default)]
	omega: f32,

	#[getset(get = "pub")]
	#[builder(default)]
	M0: f32,

	#[getset(get = "pub")]
	period: f32, // orbital period

	// pre-computed
	#[builder(setter(skip), default_code = "(mu / a).sqrt()")]
	speed_root: f32,  // sqrt(mu / a)

	#[builder(setter(skip), default_code = "speed_root / a")]
	mean_motion: f32, // sqrt(mu / a^3)

	#[builder(setter(skip), default_code = "(1.0 - e * e).sqrt()")]
	e_root: f32,      // sqrt(1 - e^2)

	// matrix to transform vectors from orbital to ecliptic coordinates
	orb_to_ecl: Matrix3x2<f32>,
}

impl Orbit {
	pub fn new(mu: f32, a: f32, e: f32, i: f32, Omega: f32, omega: f32, M0: f32) -> Self {
		let speed_root = (mu / a).sqrt();
		let mean_motion = speed_root / a;
		let period = a / speed_root;
		let e_root = (1.0 - e * e).sqrt();
		Orbit {
			mu,
			a,
			e,
			i,
			Omega,
			omega,
			M0,
			speed_root,
			mean_motion,
			period,
			e_root,
			orb_to_ecl: Self::orb_to_ecl(i, Omega, omega),
		}
	}

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
		let r_orb = Vector2::from([a * (cos_E - e), a * e_root * sin_E]);
		self.orb_to_ecl * r_orb
	}

	fn v_from_sin_cos_E(&self, (sin_E, cos_E): (f32, f32)) -> Vector3<f32> {
		let (e, e_root) = (self.e, self.e_root);
		let v_mult: f32 = self.speed_root / (1.0 - e * cos_E);
		let v_orb = Vector2::from([-v_mult * sin_E, v_mult * e_root * cos_E]);
		self.orb_to_ecl * v_orb
	}

	fn E_from_M(&self, _M: f32) -> f32 {
		unimplemented!()
	}

	fn M_from_t(&self, t: f32) -> f32 {
		(t / self.period) * TAU
	}

	fn orb_to_ecl(i: f32, Omega: f32, omega: f32) -> Matrix3x2<f32> {
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
