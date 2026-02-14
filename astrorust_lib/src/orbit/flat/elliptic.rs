use nalgebra::Vector2;

use crate::angle::{EccAnomaly, IntoAnomaly, MeanAnomaly, TrueAnomaly};
use crate::state_vectors::{StateVectorTypes, StateVectors};
use crate::time::Time;

use super::Orbit2D;
use std::f64::consts::TAU;

#[derive(Debug, Clone)]
pub struct EllipticOrbit(pub Orbit2D);

impl From<Orbit2D> for EllipticOrbit {
    fn from(orbit: Orbit2D) -> Self {
        Self(orbit)
    }
}

impl EllipticOrbit {
    fn r_from_sin_cos_E(&self, (sin_E, cos_E): (f64, f64)) -> Vector2<f64> {
        let Orbit2D { a, e, e_root, .. } = self.0;
        a * Vector2::new(cos_E - e, e_root * sin_E)
    }

    fn v_from_sin_cos_E(&self, (sin_E, cos_E): (f64, f64)) -> Vector2<f64> {
        let Orbit2D { e, e_root, speed_root, .. } = self.0;
        let v_mult = speed_root / (1.0 - e * cos_E);
        v_mult * Vector2::new(-sin_E, e_root * cos_E)
    }

    /// sin and cos of eccentric anomaly
    fn sin_cos_E_from_nu(&self, nu: TrueAnomaly, e: f64) -> (f64, f64) {
        let (sin_nu, cos_nu) = nu.sin_cos();
        let inv_denominator = 1.0 / (e.mul_add(cos_nu, 1.0));
        let sin_E = self.0.e_root * sin_nu * inv_denominator;
        let cos_E = (e + cos_nu) * inv_denominator;
        (sin_E, cos_E)
    }

    pub fn period(&self) -> Time {
        Time::from_secs(TAU / self.0.mean_motion)
    }
}

impl StateVectorTypes for EllipticOrbit {
    type Position = Vector2<f64>;
    type Velocity = Vector2<f64>;
}

impl StateVectors<MeanAnomaly> for EllipticOrbit {
    fn position(&self, M: MeanAnomaly) -> Self::Position {
        let E: EccAnomaly = M.into_anomaly(self.0.e);
        self.r_from_sin_cos_E(E.sin_cos())
    }

    fn velocity(&self, M: MeanAnomaly) -> Self::Velocity {
        let E: EccAnomaly = M.into_anomaly(self.0.e);
        self.v_from_sin_cos_E(E.sin_cos())
    }

    fn position_and_velocity(&self, M: MeanAnomaly) -> (Self::Position, Self::Velocity) {
        let E: EccAnomaly = M.into_anomaly(self.0.e);
        let sin_cos_E = E.sin_cos();
        (self.r_from_sin_cos_E(sin_cos_E), self.v_from_sin_cos_E(sin_cos_E))
    }
}

impl StateVectors<EccAnomaly> for EllipticOrbit {
    fn position(&self, E: EccAnomaly) -> Self::Position {
        self.r_from_sin_cos_E(E.sin_cos())
    }

    fn velocity(&self, E: EccAnomaly) -> Self::Velocity {
        self.v_from_sin_cos_E(E.sin_cos())
    }

    fn position_and_velocity(&self, E: EccAnomaly) -> (Self::Position, Self::Velocity) {
        let sin_cos_E = E.sin_cos();
        (self.r_from_sin_cos_E(sin_cos_E), self.v_from_sin_cos_E(sin_cos_E))
    }
}

impl StateVectors<TrueAnomaly> for EllipticOrbit {
    fn position(&self, nu: TrueAnomaly) -> Self::Position {
        self.r_from_sin_cos_E(self.sin_cos_E_from_nu(nu, self.0.e))
    }

    fn velocity(&self, nu: TrueAnomaly) -> Self::Velocity {
        self.v_from_sin_cos_E(self.sin_cos_E_from_nu(nu, self.0.e))
    }

    fn position_and_velocity(&self, nu: TrueAnomaly) -> (Self::Position, Self::Velocity) {
        let sin_cos_E = self.sin_cos_E_from_nu(nu, self.0.e);
        (self.r_from_sin_cos_E(sin_cos_E), self.v_from_sin_cos_E(sin_cos_E))
    }
}

impl StateVectors<Time> for EllipticOrbit {
    fn position(&self, t: Time) -> Self::Position {
        self.position(self.0.M_from_t(t))
    }

    fn velocity(&self, t: Time) -> Self::Velocity {
        self.velocity(self.0.M_from_t(t))
    }

    fn position_and_velocity(&self, t: Time) -> (Self::Position, Self::Position) {
        self.position_and_velocity(self.0.M_from_t(t))
    }
}

#[cfg(test)]
mod tests {
    use crate::angle::{Angle, TrueAnomaly};
    use crate::orbit::flat::elliptic::EllipticOrbit;
    use crate::orbit::flat::Orbit2DBuilder;
    use crate::state_vectors::StateVectors;

    #[test]
    fn kerbin() {
        let orbit = Orbit2DBuilder::default()
            .std_grav_param(1.1723328e9)
            .semi_major_axis(1.3599840256e7)
            .mean_anomaly_at_t0(Angle::from_rad(3.14).into())
            .build()
            .unwrap();
        let orbit = EllipticOrbit(orbit);
        dbg!(&orbit);
        let (r, v) = orbit.position_and_velocity(TrueAnomaly::from(Angle::from_rad(0.0)));
        assert!((r.norm() - 1.3599840256e7).abs() < 1e-8);
        assert!((v.norm() - 9.285).abs() < 1e-3);
    }
}
