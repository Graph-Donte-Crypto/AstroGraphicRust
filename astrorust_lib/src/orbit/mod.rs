use elliptic::Orbit2D;
use nalgebra::{Matrix3x2, Vector3};

use crate::state_vectors::{StateVectorTypes, StateVectors};

pub mod elliptic;
pub mod hyperbolic;

#[derive(Builder, CopyGetters, Debug)]
// #[builder(build_fn(validate = "Self::validate"))]
pub struct Orbit3D {
    #[builder(setter)]
    orbit_2d: Orbit2D,

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
        default = "Orbit3D::compute_orb_to_ecl(self.i.unwrap(), self.Omega.unwrap(), self.omega.unwrap())"
    )]
    orb_to_ecl: Matrix3x2<f64>,
}

impl From<Orbit2D> for Orbit3DBuilder {
    fn from(orbit_2d: Orbit2D) -> Self {
        Self { orbit_2d: Some(orbit_2d), ..Default::default() }
    }
}

impl StateVectorTypes for Orbit3D {
    type Position = Vector3<f64>;
    type Velocity = Vector3<f64>;
}

impl<E: Copy> StateVectors<E> for Orbit3D
where
    Orbit2D: StateVectors<E>,
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

impl Orbit3D {
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
