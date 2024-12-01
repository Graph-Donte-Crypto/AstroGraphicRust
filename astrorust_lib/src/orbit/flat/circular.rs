use nalgebra::Vector2;

use crate::angle::TrueAnomaly;
use crate::state_vectors::{StateVectorTypes, StateVectors};
use crate::time::Time;

use super::Orbit2D;

pub struct CircularOrbit(Orbit2D);

impl StateVectorTypes for CircularOrbit {
    type Position = Vector2<f64>;
    type Velocity = Vector2<f64>;
}

impl StateVectors<TrueAnomaly> for CircularOrbit {
    fn position(&self, nu: TrueAnomaly) -> Self::Position {
        let (cos_nu, sin_nu) = nu.sin_cos();
        self.0.a * Vector2::new(cos_nu, sin_nu)
    }

    fn velocity(&self, nu: TrueAnomaly) -> Self::Velocity {
        let (cos_nu, sin_nu) = nu.sin_cos();
        self.0.speed_root * Vector2::new(-sin_nu, cos_nu)
    }

    fn position_and_velocity(&self, nu: TrueAnomaly) -> (Self::Position, Self::Velocity) {
        let (cos_nu, sin_nu) = nu.sin_cos();
        let r = self.0.a * Vector2::new(cos_nu, sin_nu);
        let v = self.0.speed_root * Vector2::new(-sin_nu, cos_nu);
        (r, v)
    }
}

impl StateVectors<Time> for CircularOrbit {
    fn position(&self, t: Time) -> Self::Position {
        self.position(TrueAnomaly::from(*self.0.M_from_t(t)))
    }

    fn velocity(&self, t: Time) -> Self::Velocity {
        self.velocity(TrueAnomaly::from(*self.0.M_from_t(t)))
    }

    fn position_and_velocity(&self, t: Time) -> (Self::Position, Self::Position) {
        self.position_and_velocity(TrueAnomaly::from(*self.0.M_from_t(t)))
    }
}
