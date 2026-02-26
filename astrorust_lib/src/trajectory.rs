use crate::config;
use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::flat::hyperbolic::HyperbolicOrbit;
use crate::orbit::orbit_3d::{KeplerianElements, Orbit3D};
use crate::state_vectors::{StateVectorTypes, StateVectors};
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub enum Trajectory {
    Elliptic(Orbit3D<EllipticOrbit>),
    Hyperbolic(Orbit3D<HyperbolicOrbit>),
}

impl Trajectory {
    pub fn from_state_vectors(mu: f64, r: Vector3<f64>, v: Vector3<f64>, t: f64) -> Self {
        let r_mag = r.magnitude();
        let a = r_mag * mu / (2.0 * mu - v.magnitude_squared() * r_mag);
        if a >= 0.0 {
            Orbit3D::<EllipticOrbit>::from_state_vectors(mu, r, v, t).into()
        } else {
            Orbit3D::<HyperbolicOrbit>::from_state_vectors(mu, r, v, t).into()
        }
    }
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
        let is_elliptic = value.a >= 0.0;
        let elements: KeplerianElements = value.into();
        if is_elliptic {
            Orbit3D::<EllipticOrbit>::from(elements).into()
        } else {
            Orbit3D::<HyperbolicOrbit>::from(elements).into()
        }
    }
}
