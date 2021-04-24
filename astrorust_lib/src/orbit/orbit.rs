#![allow(non_snake_case)]

use core::f32::consts::TAU;
use uom::si::{Quantity, SI, f32::{Length, Angle, Time, Velocity, Frequency, Ratio}};
use uom::si::ratio::ratio;
use uom::si::angle::Unit;
use num_traits::real::Real;
use typenum::{int::Z0, P3, N2};
use nalgebra::{Matrix3x2, Vector2, Vector3, base::Scalar};

type MuDimension = uom::si::ISQ<P3, Z0, N2, Z0, Z0, Z0, Z0, dyn uom::Kind>;
type StdGravParam = Quantity<MuDimension, SI<f32>, f32>;

#[derive(Getters, Builder, Debug)]
#[allow(non_snake_case)]
pub struct Orbit {
    #[getset(get = "pub")]
	mu: StdGravParam,

    #[getset(get = "pub")]
	a: Length,

    #[getset(get = "pub")]
    #[builder(default)]
	e: f32,

    #[getset(get = "pub")]
    #[builder(default)]
	i: Angle,

    #[getset(get = "pub")]
    #[builder(default)]
    #[allow(non_snake_case)]
	Omega: Angle,

    #[getset(get = "pub")]
    #[builder(default)]
	omega: Angle,

    #[getset(get = "pub")]
	period: Time,           // orbital period

	// pre-computed
	speed_root: Velocity,   // sqrt(mu / a)
	mean_motion: Frequency, // sqrt(mu / a^3)
	e_root: f32,            // sqrt(1 - e^2)
	orb_to_ecl: Matrix3x2<Ratio>,
}

#[allow(non_snake_case)]
impl Orbit {
    pub fn new(mu: StdGravParam,
              a: Length,
              e: f32,
              i: Angle,
              Omega: Angle,
              omega: Angle) -> Self {

        let speed_root = (mu / a).sqrt();
		let mean_motion = speed_root / a;
        let period = a / speed_root;
		let e_root = (1.0 - e * e).sqrt();
		Orbit {
            mu, a, e, i, Omega, omega,
            speed_root, mean_motion, period, e_root,
            orb_to_ecl: Self::orb_to_ecl(i, Omega, omega)
        }
    }

    pub fn r_from_nu(&self, nu: Angle) -> Vector3<Length> {
		let (e, a, e_root) = (self.e, self.a, self.e_root);
        let (sin_nu, cos_nu) = nu.sin_cos().into() as (f32, f32);
		let inv_denominator = 1.0 / (e.mul_add(cos_nu, 1.0));
		let sin_E = e_root * sin_nu * inv_denominator;
		let cos_E = (e + cos_nu) * inv_denominator;
        self.r_from_sin_cos_E((sin_E, cos_E))
    }
    
    pub fn r_and_v_from_nu(&self, nu: Angle) -> (Vector3<Length>, Vector3<Velocity>) {
		let (e, a, e_root) = (self.e, self.a, self.e_root);
        let (sin_nu, cos_nu) = nu.sin_cos().into() as (f32, f32);
		let inv_denominator = 1.0 / (e.mul_add(cos_nu, 1.0));
		let sin_E = Ratio::new(e_root * sin_nu * inv_denominator);
		let cos_E = Ratio::new((e + cos_nu) * inv_denominator);
        (self.r_from_sin_cos_E((sin_E, cos_E)),
         self.v_from_sin_cos_E((sin_E, cos_E)))
    }
    
    pub fn r_from_E(&self, E: Angle) -> Vector3<Length> {
		let (a, e, e_root) = (self.a, self.e, self.e_root);
        self.r_from_sin_cos_E(E.sin_cos())
    }

    pub fn r_and_v_from_E(&self, E: Angle) -> (Vector3<Length>, Vector3<Velocity>) {
		let (a, e, e_root) = (self.a, self.e, self.e_root);
        let E_sin_cos = E.sin_cos();
        (self.r_from_sin_cos_E(E_sin_cos),
         self.v_from_sin_cos_E(E_sin_cos))
    }

    pub fn r_from_t(&self, t: Time) -> Vector3<Length> {
        self.r_from_E(E_from_M(M_from_t(t)))
    }
    
    pub fn r_and_v_from_t(&self, t: Time) -> (Vector3<Length>, Vector3<Velocity>) {
        self.r_and_v_from_E(self.E_from_M(self.M_from_t(t)))
    }

    fn r_from_sin_cos_E(&self, (sin_E, cos_E): (Ratio, Ratio)) -> Vector3<Length> {
		let (a, e, e_root) = (self.a, self.e, self.e_root);
		let r_orb = Vector2::new(a * (cos_E.into() - e), a * e_root * sin_E.into());
	    self.orb_to_ecl * r_orb
    }

    fn v_from_sin_cos_E(&self, (sin_E, cos_E): (Ratio, Ratio)) -> Vector3<Velocity> {
		let (a, e, e_root) = (self.a, self.e, self.e_root);
		let v_mult: Velocity = self.speed_root / (1.0 - e * cos_E.into());
		let v_orb = Vector2::new(-v_mult * sin_E, v_mult * e_root * cos_E);
	    self.orb_to_ecl * v_orb
    }

    fn E_from_M(&self, M: Angle) -> Angle {
        unimplemented!()
    }

    fn M_from_t(&self, t: Time) -> Angle {
        (t / self.period) * Angle::new(TAU)
    }

    fn orb_to_ecl(i: Angle, Omega: Angle, omega: Angle) -> Matrix3x2<Ratio> {
        let (sin_i, cos_i) = i.sin_cos();
        let (sin_Omega, cos_Omega) = Omega.sin_cos();
        let (sin_omega, cos_omega) = omega.sin_cos();
        Matrix3x2::new(
        	  cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i,
            - cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i,
              sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i,
            - sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i,
              sin_omega * sin_i,
              cos_omega * sin_i
        )
    }
}
