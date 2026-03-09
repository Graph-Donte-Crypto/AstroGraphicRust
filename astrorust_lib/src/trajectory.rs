use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::flat::hyperbolic::HyperbolicOrbit;
use crate::orbit::orbit_3d::{KeplerianElements, Orbit3D};
use crate::state_vectors::StateVectors;
use crate::{AU_IN_KM, config};
use nalgebra::{Matrix3x2, Vector3};
use std::fmt::{self, Display, Formatter};
use std::ops::Mul;

#[derive(Debug, Clone)]
pub enum Trajectory {
    Elliptic(Orbit3D<EllipticOrbit>),
    Hyperbolic(Orbit3D<HyperbolicOrbit>),
}

impl Trajectory {
    pub fn periapsis(&self) -> f64 {
        match self {
            Trajectory::Elliptic(orbit3_d) => orbit3_d.periapsis(),
            Trajectory::Hyperbolic(orbit3_d) => orbit3_d.periapsis(),
        }
    }

    pub fn apoapsis(&self) -> f64 {
        match self {
            Trajectory::Elliptic(orbit3_d) => orbit3_d.apoapsis(),
            Trajectory::Hyperbolic(_) => f64::INFINITY,
        }
    }

    pub fn a(&self) -> f64 {
        match self {
            Trajectory::Elliptic(orbit3_d) => orbit3_d.orbit_2d.0.a(),
            Trajectory::Hyperbolic(orbit3_d) => orbit3_d.orbit_2d.0.a(),
        }
    }

    pub fn e(&self) -> f64 {
        match self {
            Trajectory::Elliptic(orbit3_d) => orbit3_d.orbit_2d.0.e(),
            Trajectory::Hyperbolic(orbit3_d) => orbit3_d.orbit_2d.0.e(),
        }
    }

    pub fn orb_to_ecl(&self) -> &Matrix3x2<f64> {
        match self {
            Trajectory::Elliptic(orbit3_d) => orbit3_d.orb_to_ecl(),
            Trajectory::Hyperbolic(orbit3_d) => orbit3_d.orb_to_ecl(),
        }
    }

    pub fn t0(&self) -> f64 {
        match self {
            Trajectory::Elliptic(orbit3_d) => orbit3_d.orbit_2d.0.a(),
            Trajectory::Hyperbolic(orbit3_d) => orbit3_d.orbit_2d.0.a(),
        }
    }

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

impl<P: Copy> StateVectors<P> for Trajectory
where
    EllipticOrbit: StateVectors<P>,
    HyperbolicOrbit: StateVectors<P>,
    Matrix3x2<f64>: Mul<<EllipticOrbit as StateVectors<P>>::Position, Output = Vector3<f64>>,
    Matrix3x2<f64>: Mul<<EllipticOrbit as StateVectors<P>>::Velocity, Output = Vector3<f64>>,
    Matrix3x2<f64>: Mul<<HyperbolicOrbit as StateVectors<P>>::Position, Output = Vector3<f64>>,
    Matrix3x2<f64>: Mul<<HyperbolicOrbit as StateVectors<P>>::Velocity, Output = Vector3<f64>>,
{
    type Position = Vector3<f64>;
    type Velocity = Vector3<f64>;

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

impl Display for Orbit3D<EllipticOrbit> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let a = self.orbit_2d.0.a();
        let (a, a_unit) = if a / AU_IN_KM < 0.2 { (a, "km") } else { (a / AU_IN_KM, "au") };
        write!(
            f,
            "pe: {:.6} {a_unit}\nap: {:.6} {a_unit}\na: {a:.6} {a_unit}\ne: {:.6}\ni: {:.3}°\n\u{03A9}: {:.3}°\n\u{03C9}: {:.3}°\nM₀: {:.3}°",
            a * (1.0 - self.orbit_2d.0.e()),
            a * (1.0 + self.orbit_2d.0.e()),
            self.orbit_2d.0.e(),
            self.i().to_degrees(),
            self.Omega().to_degrees(),
            self.omega().to_degrees(),
            self.orbit_2d.0.M0().as_deg(),
        )
    }
}

impl Display for Orbit3D<HyperbolicOrbit> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let a = self.orbit_2d.0.a();
        let (a, a_unit) = if (a / AU_IN_KM).abs() < 0.2 { (a, "km") } else { (a / AU_IN_KM, "au") };
        write!(
            f,
            "pe: {:.6} {a_unit}\n a: {a:.6} {a_unit}\ne: {:.6}\ni: {:.3}°\n\u{03A9}: {:.3}°\n\u{03C9}: {:.3}°\nM₀: {:.3}",
            a * (1.0 - self.orbit_2d.0.e()),
            self.orbit_2d.0.e(),
            self.i().to_degrees(),
            self.Omega().to_degrees(),
            self.omega().to_degrees(),
            self.orbit_2d.0.M0().as_rad(),
        )
    }
}

impl Display for Trajectory {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::Elliptic(orbit) => write!(f, "{orbit}"),
            Self::Hyperbolic(orbit) => write!(f, "{orbit}"),
        }
    }
}
