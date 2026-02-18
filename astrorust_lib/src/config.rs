use crate::angle::Angle;
use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::flat::Orbit2DBuilder;
use crate::orbit::orbit_3d::{Orbit3D, Orbit3DBuilder};
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub system: String,
    pub spacecraft: Spacecraft,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Spacecraft {
    pub body: String,
    pub orbit: Orbit,
}

#[derive(Debug, Clone, Deserialize)]
pub struct StarSystem {
    pub star: CelestialBody,
    pub planets: Vec<Planet>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct CelestialBody {
    pub name: String,
    /// Standard gravitational parameter
    pub μ: f64,
    pub radius: f64,
    pub atmosphere_height: f64,
    pub siderial_day: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Planet {
    #[serde(flatten)]
    pub body: CelestialBody,
    pub orbit: Orbit,
}

impl Planet {
    pub fn soi_radius(&self) -> f64 {
        self.orbit.a * (self.body.μ / self.orbit.mu).powf(0.4)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct Orbit {
    #[serde(default)]
    pub mu: f64,

    /// Semi-major axis
    pub a: f64,
    /// Eccentricity
    pub e: f64,
    /// Inclination
    pub i: f64,
    /// Longitude of ascending node
    pub Ω: f64,
    /// Argument of periapsis
    pub ω: f64,
    /// Mean anomaly at t=0
    pub M0: f64,
}

impl StarSystem {
    pub fn load_from_yaml(path: &str) -> anyhow::Result<Self> {
        let mut system: Self = serde_saphyr::from_str(&std::fs::read_to_string(path)?)?;
        for planet in &mut system.planets {
            planet.orbit.mu = system.star.μ;
        }
        Ok(system)
    }

    pub fn into_planets(self) -> Vec<(CelestialBody, Orbit3D<EllipticOrbit>)> {
        let μ = self.star.μ;
        self.planets
            .into_iter()
            .map(|planet| {
                let orbit: EllipticOrbit = Orbit2DBuilder::default()
                    .std_grav_param(μ)
                    .semi_major_axis(planet.orbit.a)
                    .eccentricity(planet.orbit.e)
                    .mean_anomaly_at_t0(Angle::from_rad(planet.orbit.M0).into())
                    .build()
                    .unwrap()
                    .into();
                let orbit = Orbit3DBuilder::default()
                    .orbit_2d(orbit)
                    .inclination(planet.orbit.i.to_radians())
                    .long_of_asc_node(planet.orbit.Ω.to_radians())
                    .arg_of_periapsis(planet.orbit.ω.to_radians())
                    .build()
                    .unwrap();
                (planet.body, orbit)
            })
            .collect()
    }
}

impl Config {
    pub fn load_from_yaml(path: &str) -> anyhow::Result<Self> {
        Ok(serde_saphyr::from_str(&std::fs::read_to_string(path)?)?)
    }
}
