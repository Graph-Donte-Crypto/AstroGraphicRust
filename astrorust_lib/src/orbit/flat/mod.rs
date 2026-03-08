pub mod elliptic;
pub mod hyperbolic;

use crate::angle::{Angle, MeanAnomaly};
use crate::time::Time;

#[derive(Clone, Builder, CopyGetters, Debug, PartialEq)]
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
    #[builder(setter(skip), default = "(self.mu.unwrap() / self.a.unwrap().abs()).sqrt()")]
    speed_root: f64, // sqrt(mu / a)
    //
    /// sqrt(1 - e^2)
    #[builder(setter(skip), default = "((1.0 - self.e.unwrap_or(0.0).powi(2)).abs()).sqrt()")]
    e_root: f64,

    #[builder(setter(skip), default = "(self.mu.unwrap() / self.a.unwrap().abs().powi(3)).sqrt()")]
    mean_motion: f64, // sqrt(mu / a^3)
}

impl Orbit2DBuilder {
    pub fn validate(&self) -> Result<(), String> {
        let a = self.a.unwrap();
        match self.e {
            Some(e) if e < 0.0 => return Err("Eccentricity must be non-negative".into()),
            Some(e) if e > 1.0 && a > 0.0 => {
                return Err("Semi-major axis must be < 0 if eccentricity is > 1 (hyperbolic)".into());
            }
            Some(e) if e < 1.0 && a < 0.0 => {
                return Err("Semi-major axis must be > 0 if eccentricity is < 1 (elliptic)".into());
            }
            _ => (),
        }
        Ok(())
    }
}

impl Orbit2D {
    pub fn periapsis(&self) -> f64 {
        self.a * (1.0 - self.e)
    }

    pub fn apoapsis(&self) -> f64 {
        self.a * (1.0 + self.e)
    }

    pub fn M_from_t(&self, t: Time) -> MeanAnomaly {
        let Self { M0, mean_motion, .. } = self;
        (**M0 + Angle::from_rad(mean_motion * t.as_secs())).into()
    }
}
