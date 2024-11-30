use crate::angle::{Angle, EccAnomaly, IntoAnomaly, MeanAnomaly, TrueAnomaly};
use crate::state_vectors::{StateVectorTypes, StateVectors};
use crate::time::Time;
use nalgebra::Vector2;

#[derive(Clone, Builder, CopyGetters, Debug)]
#[builder(build_fn(validate = "Self::validate"))]
pub struct Orbit2D {
    /// Standard gravitational parameter
    #[getset(get_copy = "pub")]
    #[builder(setter(name = "std_grav_param"))]
    mu: f64,

    /// Semi-major axis
    #[getset(get_copy = "pub")]
    #[builder(setter(name = "semi_major_axis"))]
    a: f64,

    /// Eccentricity
    #[getset(get_copy = "pub")]
    #[builder(default, setter(name = "eccentricity"))]
    e: f64,

    /// Mean anomaly at T=0
    #[getset(get_copy = "pub")]
    #[builder(default, setter(name = "mean_anomaly_at_t0"))]
    M0: MeanAnomaly,

    /// `sqrt(μ / a)`
    #[builder(setter(skip), default = "(self.mu.unwrap() / self.a.unwrap()).sqrt()")]
    speed_root: f64, // sqrt(mu / a)
    //
    /// sqrt(1 - e^2)
    #[builder(setter(skip), default = "(1.0 - self.e.unwrap_or(0.0).powi(2)).sqrt()")]
    e_root: f64,

    #[builder(
        setter(skip),
        default = "(self.mu.unwrap() * self.a.unwrap().recip()).sqrt() * self.a.unwrap().recip()"
    )]
    mean_motion: f64, // sqrt(mu / a^3)
}

impl Orbit2DBuilder {
    pub fn validate(&self) -> Result<(), String> {
        match self.e {
            Some(e) if e <= 0.0 => return Err("Eccentricity must be non-negative".into()),
            _ => (),
        }
        Ok(())
    }
}

impl Orbit2D {
    fn r_from_sin_cos_E(&self, (sin_E, cos_E): (f64, f64)) -> Vector2<f64> {
        let Self { e, e_root, .. } = self;
        self.a * Vector2::new(cos_E - e, e_root * sin_E)
    }

    fn v_from_sin_cos_E(&self, (sin_E, cos_E): (f64, f64)) -> Vector2<f64> {
        let Self { e, e_root, .. } = self;
        let v_mult: f64 = self.speed_root / (1.0 - e * cos_E);
        -v_mult * Vector2::new(sin_E, e_root * cos_E)
    }

    fn M_from_t(&self, t: Time) -> MeanAnomaly {
        (*self.M0 + Angle::from_rad(self.mean_motion * t.as_secs())).into()
    }

    /// sin and cos of eccentric anomaly
    fn sin_cos_E_from_nu(&self, nu: TrueAnomaly, e: f64) -> (f64, f64) {
        let (sin_nu, cos_nu) = nu.sin_cos();
        let inv_denominator = 1.0 / (e.mul_add(cos_nu, 1.0));
        let sin_E = self.e_root * sin_nu * inv_denominator;
        let cos_E = (e + cos_nu) * inv_denominator;
        (sin_E, cos_E)
    }
}

impl StateVectorTypes for Orbit2D {
    type Position = Vector2<f64>;
    type Velocity = Vector2<f64>;
}

impl<E: IntoAnomaly<EccAnomaly> + Copy> StateVectors<E> for Orbit2D {
    fn position(&self, anomaly: E) -> Self::Position {
        self.r_from_sin_cos_E(anomaly.into_anomaly(self.e).sin_cos())
    }

    fn velocity(&self, anomaly: E) -> Self::Velocity {
        self.v_from_sin_cos_E(anomaly.into_anomaly(self.e).sin_cos())
    }

    fn position_and_velocity(&self, anomaly: E) -> (Self::Position, Self::Velocity) {
        let sin_cos_E = anomaly.into_anomaly(self.e).sin_cos();
        (self.r_from_sin_cos_E(sin_cos_E), self.v_from_sin_cos_E(sin_cos_E))
    }
}

impl StateVectors<TrueAnomaly> for Orbit2D {
    fn position(&self, nu: TrueAnomaly) -> Self::Position {
        self.r_from_sin_cos_E(self.sin_cos_E_from_nu(nu, self.e))
    }

    fn velocity(&self, nu: TrueAnomaly) -> Self::Velocity {
        self.v_from_sin_cos_E(self.sin_cos_E_from_nu(nu, self.e))
    }

    fn position_and_velocity(&self, nu: TrueAnomaly) -> (Self::Position, Self::Velocity) {
        let sin_cos_E = self.sin_cos_E_from_nu(nu, self.e);
        (self.r_from_sin_cos_E(sin_cos_E), self.v_from_sin_cos_E(sin_cos_E))
    }
}

impl StateVectors<Time> for Orbit2D {
    fn position(&self, t: Time) -> Self::Position {
        self.position(self.M_from_t(t))
    }

    fn velocity(&self, t: Time) -> Self::Velocity {
        self.velocity(self.M_from_t(t))
    }

    fn position_and_velocity(&self, t: Time) -> (Self::Position, Self::Position) {
        self.position_and_velocity(self.M_from_t(t))
    }
}

#[cfg(test)]
mod tests {
    use super::Orbit2DBuilder;
    use crate::angle::Angle;
    use crate::state_vectors::StateVectors;
    use crate::time::Time;

    #[test]
    fn kerbin() {
        let orbit = Orbit2DBuilder::default()
            .std_grav_param(1.1723328e9)
            .semi_major_axis(1.3599840256e7)
            .mean_anomaly_at_t0(Angle::from_rad(3.14).into())
            .build()
            .unwrap();
        dbg!(&orbit);
        let (r, v) = orbit.position_and_velocity(Time::from_secs(9203545.0));
        assert!((r.norm() - 1.3599840256e7).abs() < 1e-8);
        assert!((v.norm() - 9.285).abs() < 1e-3);
    }
}
