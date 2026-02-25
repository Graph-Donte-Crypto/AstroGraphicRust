use crate::angle::Angle;
use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::flat::Orbit2DBuilder;
use crate::orbit::orbit_3d::{Orbit3D, Orbit3DBuilder};
use serde::{Deserialize, Deserializer};

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
    #[serde(deserialize_with = "deserialize_hex_color")]
    pub color: [u8; 3],
}

#[derive(Debug, Clone, Deserialize)]
pub struct Planet {
    #[serde(flatten)]
    pub body: CelestialBody,
    #[serde(default)]
    pub moons: Vec<Moon>,
    pub orbit: Orbit,
}

impl Planet {
    pub fn soi_radius(&self) -> f64 {
        self.orbit.a * (self.body.μ / self.orbit.mu).powf(0.4)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct Moon {
    #[serde(flatten)]
    pub body: CelestialBody,
    pub orbit: Orbit,
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
            for moon in &mut planet.moons {
                moon.orbit.mu = planet.body.μ;
            }
        }
        Ok(system)
    }

    pub fn into_planets(self) -> Vec<(CelestialBody, Orbit3D<EllipticOrbit>)> {
        let μ = self.star.μ;
        self.planets
            .into_iter()
            .map(|planet| (planet.body, build_orbit(μ, &planet.orbit)))
            .collect()
    }
}

impl Config {
    pub fn load_from_yaml(path: &str) -> anyhow::Result<Self> {
        Ok(serde_saphyr::from_str(&std::fs::read_to_string(path)?)?)
    }
}

fn deserialize_hex_color<'de, D>(deserializer: D) -> Result<[u8; 3], D::Error>
where
    D: Deserializer<'de>,
{
    let color = String::deserialize(deserializer)?;
    if color.len() != 7 || !color.starts_with('#') {
        return Err(serde::de::Error::custom("color must be in #RRGGBB format"));
    }
    let r = u8::from_str_radix(&color[1..3], 16)
        .map_err(|_| serde::de::Error::custom("invalid red channel in color"))?;
    let g = u8::from_str_radix(&color[3..5], 16)
        .map_err(|_| serde::de::Error::custom("invalid green channel in color"))?;
    let b = u8::from_str_radix(&color[5..7], 16)
        .map_err(|_| serde::de::Error::custom("invalid blue channel in color"))?;
    Ok([r, g, b])
}

fn build_orbit(mu: f64, orbit: &Orbit) -> Orbit3D<EllipticOrbit> {
    let orbit_2d: EllipticOrbit = Orbit2DBuilder::default()
        .std_grav_param(mu)
        .semi_major_axis(orbit.a)
        .eccentricity(orbit.e)
        .mean_anomaly_at_t0(Angle::from_rad(orbit.M0).into())
        .build()
        .unwrap()
        .into();
    Orbit3DBuilder::default()
        .orbit_2d(orbit_2d)
        .inclination(orbit.i.to_radians())
        .long_of_asc_node(orbit.Ω.to_radians())
        .arg_of_periapsis(orbit.ω.to_radians())
        .build()
        .unwrap()
}
