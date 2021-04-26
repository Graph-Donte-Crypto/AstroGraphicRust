#![allow(non_snake_case)]

use core::f32::consts::TAU;
use dimensioned::{si::{Meter, Second, MeterPerSecond, Hertz, Unitless}, ucum::Radian, MapUnsafe};
use nalgebra::{Matrix3x2, Vector2, Vector3};
use crate::units::StdGravParam;

#[derive(TypedBuilder, Getters, Setters, Debug)]
pub struct Orbit {
	#[getset(get = "pub")]
	mu: StdGravParam<f32>,

	#[getset(get = "pub")]
	a: Meter<f32>,

	#[getset(get = "pub")]
	#[builder(default)]
	e: Unitless<f32>,

	#[getset(get = "pub")]
	#[builder(default)]
	i: Radian<f32>,

	#[getset(get = "pub")]
	#[builder(default)]
	Omega: Radian<f32>,

	#[getset(get = "pub")]
	#[builder(default)]
	omega: Radian<f32>,

	#[getset(get = "pub")]
	#[builder(default)]
	M0: Radian<f32>,

	#[builder(setter(skip), default)]
	period: Second<f32>, // orbital period

	// pre-computed
	#[builder(setter(skip), default_code = "(mu / a).sqrt()")]
	speed_root: MeterPerSecond<f32>,  // sqrt(mu / a)

	#[builder(setter(skip), default_code = "speed_root / a")]
	mean_motion: Hertz<f32>, // sqrt(mu / a^3)

	#[builder(setter(skip), default_code = "Unitless::new((1.0 - e * e).sqrt())")]
	e_root: Unitless<f32>,      // sqrt(1 - e^2)

	// matrix to transform vectors from orbital to ecliptic coordinates
	#[builder(setter(skip), default_code = "Orbit::compute_orb_to_ecl(i, Omega, omega)")]
	orb_to_ecl: Matrix3x2<Unitless<f32>>,
}

impl Orbit {
	pub fn r_from_nu(&self, nu: Radian<f32>) -> Vector3<Meter<f32>> {
		let (e, e_root) = (self.e, self.e_root);
		let (sin_nu, cos_nu) = nu.value_unsafe.sin_cos();
		let inv_denominator = 1.0 / (e.mul_add(cos_nu, 1.0));
		let sin_E = e_root * sin_nu * inv_denominator;
		let cos_E = (e + cos_nu) * inv_denominator;
		self.r_from_sin_cos_E((sin_E, cos_E))
	}

	pub fn r_and_v_from_nu(&self, nu: Radian<f32>) -> (Vector3<Meter<f32>>, Vector3<MeterPerSecond<f32>>) {
		let (e, e_root) = (self.e, self.e_root);
		let (sin_nu, cos_nu) = nu.value_unsafe.sin_cos();
		let inv_denominator = 1.0 / (e.mul_add(cos_nu, 1.0));
		let sin_E = e_root * sin_nu * inv_denominator;
		let cos_E = (e + cos_nu) * inv_denominator;
		(self.r_from_sin_cos_E((sin_E, cos_E)), self.v_from_sin_cos_E((sin_E, cos_E)))
	}

	pub fn r_from_E(&self, E: Radian<f32>) -> Vector3<Meter<f32>> {
		let (sin_E, cos_E) = E.value_unsafe.sin_cos();
		self.r_from_sin_cos_E((Unitless::new(sin_E), Unitless::new(cos_E)))
	}

	pub fn r_and_v_from_E(&self, E: Radian<f32>) -> (Vector3<Meter<f32>>, Vector3<MeterPerSecond<f32>>) {
		let (sin_E, cos_E) = E.value_unsafe.sin_cos();
		(
			self.r_from_sin_cos_E((Unitless::new(sin_E), Unitless::new(cos_E))),
			self.v_from_sin_cos_E((Unitless::new(sin_E), Unitless::new(cos_E)))
		)
	}

	pub fn r_from_t(&self, t: Second<f32>) -> Vector3<Meter<f32>> {
		self.r_from_E(self.E_from_M(self.M_from_t(t)))
	}

	pub fn r_and_v_from_t(&self, t: Second<f32>) -> (Vector3<Meter<f32>>, Vector3<MeterPerSecond<f32>>) {
		self.r_and_v_from_E(self.E_from_M(self.M_from_t(t)))
	}

	fn r_from_sin_cos_E(&self, (sin_E, cos_E): (Unitless<f32>, Unitless<f32>)) -> Vector3<Meter<f32>> {
		let (a, e, e_root) = (self.a, self.e, self.e_root);
		let r_orb = Vector2::from([a * (cos_E - e), a * e_root * sin_E]);
		self.orb_to_ecl * r_orb
	}

	fn v_from_sin_cos_E(&self, (sin_E, cos_E): (Unitless<f32>, Unitless<f32>)) -> Vector3<MeterPerSecond<f32>> {
		let (e, e_root) = (self.e, self.e_root);
		let v_mult = self.speed_root / (1.0 - e * cos_E);
		let v_orb = Vector2::from([-v_mult * sin_E, v_mult * e_root * cos_E]);
		self.orb_to_ecl * v_orb
	}

	fn E_from_M(&self, _M: Radian<f32>) -> Radian<f32> {
		unimplemented!()
	}

	fn M_from_t(&self, t: Second<f32>) -> Radian<f32> {
		let test = t.map(|time| (time / self.period) * Radian::new(TAU));
		Radian::new(0.0_f32)
	}

	fn compute_orb_to_ecl(i: Radian<f32>, Omega: Radian<f32>, omega: Radian<f32>) -> Matrix3x2<Unitless<f32>> {
		let (sin_i, cos_i) = i.value_unsafe.sin_cos();
		let (sin_Omega, cos_Omega) = Omega.value_unsafe.sin_cos();
		let (sin_omega, cos_omega) = omega.value_unsafe.sin_cos();
		let arr = [
			Unitless::new(cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i),
			Unitless::new(-cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i),
			Unitless::new(sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i),
			Unitless::new(-sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i),
			Unitless::new(sin_omega * sin_i),
			Unitless::new(cos_omega * sin_i),
		];
		Matrix3x2::from_row_slice(&arr)
	}
}
