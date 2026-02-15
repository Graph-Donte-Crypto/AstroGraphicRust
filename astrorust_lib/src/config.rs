use crate::angle::Angle;
use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::flat::Orbit2DBuilder;
use crate::orbit::orbit_3d::{Orbit3D, Orbit3DBuilder};
use serde::Deserialize;

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

#[derive(Debug, Clone, Deserialize)]
pub struct Orbit {
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
        Ok(serde_saphyr::from_str(&std::fs::read_to_string(path)?)?)
    }

    pub fn into_planets(self) -> Vec<Orbit3D<EllipticOrbit>> {
        let μ = self.star.μ;
        self.planets
            .into_iter()
            .map(|planet| -> Orbit3D<EllipticOrbit> {
                let orbit: EllipticOrbit = Orbit2DBuilder::default()
                    .std_grav_param(μ)
                    .semi_major_axis(planet.orbit.a)
                    .eccentricity(planet.orbit.e)
                    .mean_anomaly_at_t0(Angle::from_rad(planet.orbit.M0).into())
                    .build()
                    .unwrap()
                    .into();
                Orbit3DBuilder::default()
                    .orbit_2d(orbit)
                    .inclination(planet.orbit.i.to_radians())
                    .long_of_asc_node(planet.orbit.Ω.to_radians())
                    .arg_of_periapsis(planet.orbit.ω.to_radians())
                    .build()
                    .unwrap()
            })
            .collect()
    }
}
