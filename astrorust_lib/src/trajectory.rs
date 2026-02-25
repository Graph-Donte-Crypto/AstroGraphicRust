use crate::angle::Angle;
use crate::config;
use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::flat::hyperbolic::HyperbolicOrbit;
use crate::orbit::flat::Orbit2DBuilder;
use crate::orbit::orbit_3d::{Orbit3D, Orbit3DBuilder};
use crate::state_vectors::{StateVectorTypes, StateVectors};
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub enum Trajectory {
    Elliptic(Orbit3D<EllipticOrbit>),
    Hyperbolic(Orbit3D<HyperbolicOrbit>),
}

impl StateVectorTypes for Trajectory {
    type Position = Vector3<f64>;
    type Velocity = Vector3<f64>;
}

impl<P: Copy> StateVectors<P> for Trajectory
where
    EllipticOrbit: StateVectors<P>,
    HyperbolicOrbit: StateVectors<P>,
{
    fn position(&self, param: P) -> Self::Position {
        match self {
            Trajectory::Elliptic(orbit) => orbit.position(param),
            Trajectory::Hyperbolic(orbit) => orbit.position(param),
        }
    }

    fn velocity(&self, param: P) -> Self::Velocity {
        match self {
            Trajectory::Elliptic(orbit) => orbit.velocity(param),
            Trajectory::Hyperbolic(orbit) => orbit.velocity(param),
        }
    }

    fn position_and_velocity(&self, param: P) -> (Self::Position, Self::Velocity) {
        match self {
            Trajectory::Elliptic(orbit) => orbit.position_and_velocity(param),
            Trajectory::Hyperbolic(orbit) => orbit.position_and_velocity(param),
        }
    }
}

impl From<Orbit3D<EllipticOrbit>> for Trajectory {
    fn from(value: Orbit3D<EllipticOrbit>) -> Self {
        Self::Elliptic(value)
    }
}

impl From<Orbit3D<HyperbolicOrbit>> for Trajectory {
    fn from(value: Orbit3D<HyperbolicOrbit>) -> Self {
        Self::Hyperbolic(value)
    }
}

impl From<config::Orbit> for Trajectory {
    fn from(value: config::Orbit) -> Self {
        if value.a >= 0.0 {
            let orbit_2d: EllipticOrbit = Orbit2DBuilder::default()
                .std_grav_param(value.mu)
                .semi_major_axis(value.a)
                .eccentricity(value.e)
                .mean_anomaly_at_t0(Angle::from_rad(value.M0).into())
                .build()
                .unwrap()
                .into();
            Orbit3DBuilder::default()
                .orbit_2d(orbit_2d)
                .inclination(value.i.to_radians())
                .long_of_asc_node(value.Ω.to_radians())
                .arg_of_periapsis(value.ω.to_radians())
                .build()
                .unwrap()
                .into()
        } else {
            let orbit_2d: HyperbolicOrbit = Orbit2DBuilder::default()
                .std_grav_param(value.mu)
                .semi_major_axis(value.a)
                .eccentricity(value.e)
                .mean_anomaly_at_t0(Angle::from_rad(value.M0).into())
                .build()
                .unwrap()
                .into();
            Orbit3DBuilder::default()
                .orbit_2d(orbit_2d)
                .inclination(value.i.to_radians())
                .long_of_asc_node(value.Ω.to_radians())
                .arg_of_periapsis(value.ω.to_radians())
                .build()
                .unwrap()
                .into()
        }
    }
}
