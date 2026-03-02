use crate::angle::Angle;
use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::flat::hyperbolic::HyperbolicOrbit;
use crate::orbit::flat::Orbit2DBuilder;
use crate::state_vectors::StateVectors;
use nalgebra::{Matrix3x2, Vector3};
use std::f64::consts::TAU;
use std::ops::Mul;

#[derive(Builder, CopyGetters, Debug, Clone, PartialEq)]
// #[builder(build_fn(validate = "Self::validate"))]
pub struct Orbit3D<O> {
    #[builder(setter)]
    pub orbit_2d: O,

    /// Inclination
    #[getset(get_copy = "pub")]
    #[builder(default, setter(name = "inclination"))]
    i: f64,

    /// Longitude of the ascending node
    #[getset(get_copy = "pub")]
    #[builder(default, setter(name = "long_of_asc_node"))]
    Omega: f64,

    /// Argument of periapsis
    #[getset(get_copy = "pub")]
    #[builder(default, setter(name = "arg_of_periapsis"))]
    omega: f64,

    /// Matrix to transform vectors from orbital to ecliptic coordinates
    #[builder(
        setter(skip),
        default = "Orbit3D::<O>::compute_orb_to_ecl(self.i.unwrap_or_default(), self.Omega.unwrap_or_default(), self.omega.unwrap_or_default())"
    )]
    orb_to_ecl: Matrix3x2<f64>,
}

impl<O> Orbit3D<O> {
    pub fn orb_to_ecl(&self) -> &Matrix3x2<f64> {
        &self.orb_to_ecl
    }
}

pub(crate) struct KeplerianElements {
    mu: f64,
    a: f64,
    e: f64,
    i: f64,
    Omega: f64,
    omega: f64,
    M0: f64,
}

const ECCENTRICITY_TOL: f64 = 1.0e-12;

fn elements_from_state_vectors(
    mu: f64,
    r: Vector3<f64>,
    v: Vector3<f64>,
    t: f64,
) -> KeplerianElements {
    let h = r.cross(&v);
    let r_mag = r.magnitude();
    let a = r_mag * mu / (2.0 * mu - v.magnitude_squared() * r_mag);
    let e_vec = v.cross(&r.cross(&v)) / mu - r.normalize();
    let i = (h[2] / h.magnitude()).acos();
    let n = Vector3::z().cross(&h).normalize();
    let Omega = if n[1] >= 0.0 { n[0].acos() } else { TAU - n[0].acos() };
    let omega = if e_vec[2] >= 0.0 {
        (e_vec.dot(&n) / e_vec.magnitude()).acos()
    } else {
        TAU - (e_vec.dot(&n) / e_vec.magnitude()).acos()
    };
    let e = e_vec.magnitude();

    let M0 = if a >= 0.0 {
        let mut E = ((a - r_mag) / (a * e)).acos();
        if r.dot(&v) < 0.0 {
            E = TAU - E;
        }
        let M0 = E - e * E.sin() - t * (mu / (a * a * a).abs()).sqrt();
        M0 % TAU
    } else {
        let mut H = ((a - r_mag) / (a * e)).acosh();
        if r.dot(&v) < 0.0 {
            H = -H;
        }
        e * H.sinh() - H - t * (mu / (a * a * a).abs()).sqrt()
    };

    KeplerianElements { mu, a, e, i, Omega, omega, M0 }
}

fn validate_elliptic_elements(a: f64, e: f64) {
    assert!(a.is_finite() && a >= 0.0, "Elliptic orbit requires finite a >= 0.0");
    assert!(e.is_finite() && e >= 0.0, "Eccentricity must be finite and non-negative");
    assert!(
        e < 1.0 - ECCENTRICITY_TOL,
        "Elliptic orbit requires e < 1.0 (parabolic e ~= 1 is unsupported)",
    );
}

fn validate_hyperbolic_elements(a: f64, e: f64) {
    assert!(a.is_finite() && a < 0.0, "Hyperbolic orbit requires finite a < 0.0");
    assert!(e.is_finite(), "Eccentricity must be finite");
    assert!(
        e > 1.0 + ECCENTRICITY_TOL,
        "Hyperbolic orbit requires e > 1.0 (parabolic e ~= 1 is unsupported)",
    );
}

impl From<crate::config::Orbit> for KeplerianElements {
    fn from(value: crate::config::Orbit) -> Self {
        Self {
            mu: value.mu,
            a: value.a,
            e: value.e,
            i: value.i.to_radians(),
            Omega: value.Ω.to_radians(),
            omega: value.ω.to_radians(),
            M0: value.M0,
        }
    }
}

impl From<KeplerianElements> for Orbit3D<EllipticOrbit> {
    fn from(value: KeplerianElements) -> Self {
        let KeplerianElements { mu, a, e, i, Omega, omega, M0 } = value;
        validate_elliptic_elements(a, e);

        let orbit_2d: EllipticOrbit = Orbit2DBuilder::default()
            .std_grav_param(mu)
            .semi_major_axis(a)
            .eccentricity(e)
            .mean_anomaly_at_t0(Angle::from_rad(M0).into())
            .build()
            .unwrap()
            .into();

        Orbit3DBuilder::default()
            .orbit_2d(orbit_2d)
            .inclination(i)
            .long_of_asc_node(Omega)
            .arg_of_periapsis(omega)
            .build()
            .unwrap()
    }
}

impl From<KeplerianElements> for Orbit3D<HyperbolicOrbit> {
    fn from(value: KeplerianElements) -> Self {
        let KeplerianElements { mu, a, e, i, Omega, omega, M0 } = value;
        validate_hyperbolic_elements(a, e);

        let orbit_2d: HyperbolicOrbit = Orbit2DBuilder::default()
            .std_grav_param(mu)
            .semi_major_axis(a)
            .eccentricity(e)
            .mean_anomaly_at_t0(Angle::from_rad(M0).into())
            .build()
            .unwrap()
            .into();

        Orbit3DBuilder::default()
            .orbit_2d(orbit_2d)
            .inclination(i)
            .long_of_asc_node(Omega)
            .arg_of_periapsis(omega)
            .build()
            .unwrap()
    }
}

impl Orbit3D<EllipticOrbit> {
    pub fn from_state_vectors(mu: f64, r: Vector3<f64>, v: Vector3<f64>, t: f64) -> Self {
        Self::from(elements_from_state_vectors(mu, r, v, t))
    }
}

impl Orbit3D<HyperbolicOrbit> {
    pub fn from_state_vectors(mu: f64, r: Vector3<f64>, v: Vector3<f64>, t: f64) -> Self {
        Self::from(elements_from_state_vectors(mu, r, v, t))
    }
}

impl<E: Copy, O: StateVectors<E>> StateVectors<E> for Orbit3D<O>
where
    Matrix3x2<f64>: Mul<<O as StateVectors<E>>::Position>,
    Matrix3x2<f64>: Mul<<O as StateVectors<E>>::Velocity>,
{
    type Position = <Matrix3x2<f64> as Mul<O::Position>>::Output;
    type Velocity = <Matrix3x2<f64> as Mul<O::Velocity>>::Output;

    fn position(&self, anomaly: E) -> Self::Position {
        self.orb_to_ecl * self.orbit_2d.position(anomaly)
    }

    fn velocity(&self, anomaly: E) -> Self::Velocity {
        self.orb_to_ecl * self.orbit_2d.velocity(anomaly)
    }

    fn position_and_velocity(&self, anomaly: E) -> (Self::Position, Self::Velocity) {
        let (r, v) = self.orbit_2d.position_and_velocity(anomaly);
        (self.orb_to_ecl * r, self.orb_to_ecl * v)
    }
}

impl<O> Orbit3D<O> {
    fn compute_orb_to_ecl(i: f64, Omega: f64, omega: f64) -> Matrix3x2<f64> {
        let (sin_i, cos_i) = i.sin_cos();
        let (sin_Omega, cos_Omega) = Omega.sin_cos();
        let (sin_omega, cos_omega) = omega.sin_cos();
        Matrix3x2::new(
            cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i,
            -cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i,
            sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i,
            -sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i,
            sin_omega * sin_i,
            cos_omega * sin_i,
        )
    }
}
