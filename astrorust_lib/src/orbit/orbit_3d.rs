use crate::state_vectors::{StateVectorTypes, StateVectors};
use nalgebra::Matrix3x2;
use std::ops::Mul;

#[derive(Builder, CopyGetters, Debug)]
// #[builder(build_fn(validate = "Self::validate"))]
pub struct Orbit3D<O> {
    #[builder(setter)]
    pub orbit_2d: O,

    /// Inclination
    #[getset(get_copy = "pub")]
    #[builder(default, setter(name = "inclination"))]
    i: f64,

    /// Longitude of the ascending node
    #[getset(get_copy = "pub")]
    #[builder(default, setter(name = "long_of_asc_node"))]
    Omega: f64,

    /// Argument of periapsis
    #[getset(get_copy = "pub")]
    #[builder(default, setter(name = "arg_of_periapsis"))]
    omega: f64,

    /// Matrix to transform vectors from orbital to ecliptic coordinates
    #[builder(
        setter(skip),
        default = "Orbit3D::<O>::compute_orb_to_ecl(self.i.unwrap_or_default(), self.Omega.unwrap_or_default(), self.omega.unwrap_or_default())"
    )]
    orb_to_ecl: Matrix3x2<f64>,
}

impl<O> Orbit3D<O> {
    pub fn orb_to_ecl(&self) -> &Matrix3x2<f64> {
        &self.orb_to_ecl
    }
}

impl<O: StateVectorTypes> StateVectorTypes for Orbit3D<O>
where
    Matrix3x2<f64>: Mul<<O as StateVectorTypes>::Position>,
    Matrix3x2<f64>: Mul<<O as StateVectorTypes>::Velocity>,
{
    type Position = <Matrix3x2<f64> as Mul<O::Position>>::Output;
    type Velocity = <Matrix3x2<f64> as Mul<O::Velocity>>::Output;
}

impl<E: Copy, O: StateVectors<E>> StateVectors<E> for Orbit3D<O>
where
    Matrix3x2<f64>: Mul<<O as StateVectorTypes>::Position>,
    Matrix3x2<f64>: Mul<<O as StateVectorTypes>::Velocity>,
{
    fn position(&self, anomaly: E) -> Self::Position {
        self.orb_to_ecl * self.orbit_2d.position(anomaly)
    }

    fn velocity(&self, anomaly: E) -> Self::Velocity {
        self.orb_to_ecl * self.orbit_2d.velocity(anomaly)
    }

    fn position_and_velocity(&self, anomaly: E) -> (Self::Position, Self::Velocity) {
        let (r, v) = self.orbit_2d.position_and_velocity(anomaly);
        (self.orb_to_ecl * r, self.orb_to_ecl * v)
    }
}

impl<O> Orbit3D<O> {
    fn compute_orb_to_ecl(i: f64, Omega: f64, omega: f64) -> Matrix3x2<f64> {
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
