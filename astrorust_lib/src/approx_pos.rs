//! Approximate positions of the planets using Keplerian elements.
//!
//! Based on <https://ssd.jpl.nasa.gov/planets/approx_pos.html>
//! by E.M. Standish and J.G. Williams (1992).

use nalgebra::Vector3;

const J2000_JD: f64 = 2_451_545.0;
const DAYS_PER_CENTURY: f64 = 36_525.0;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Planet {
    Mercury,
    Venus,
    EarthMoonBary,
    Mars,
    Jupiter,
    Saturn,
    Uranus,
    Neptune,
}

impl Planet {
    pub const ALL: [Planet; 8] = [
        Self::Mercury,
        Self::Venus,
        Self::EarthMoonBary,
        Self::Mars,
        Self::Jupiter,
        Self::Saturn,
        Self::Uranus,
        Self::Neptune,
    ];
}

/// Which table set to use for Keplerian elements.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeRange {
    /// Table 1: 1800 AD -- 2050 AD
    Modern,
    /// Table 2a/2b: 3000 BC -- 3000 AD
    Extended,
}

struct ElementsAndRates {
    a0: f64,
    a_dot: f64,
    e0: f64,
    e_dot: f64,
    I0: f64,
    I_dot: f64,
    L0: f64,
    L_dot: f64,
    long_peri0: f64,
    long_peri_dot: f64,
    long_node0: f64,
    long_node_dot: f64,
}

struct ExtraTerms {
    b: f64,
    c: f64,
    s: f64,
    f: f64,
}

// ── Table 1: 1800 AD -- 2050 AD ──────────────────────────────────────────

const TABLE1: [ElementsAndRates; 8] = [
    // Mercury
    ElementsAndRates {
        a0: 0.38709927,
        a_dot: 0.00000037,
        e0: 0.20563593,
        e_dot: 0.00001906,
        I0: 7.00497902,
        I_dot: -0.00594749,
        L0: 252.25032350,
        L_dot: 149472.67411175,
        long_peri0: 77.45779628,
        long_peri_dot: 0.16047689,
        long_node0: 48.33076593,
        long_node_dot: -0.12534081,
    },
    // Venus
    ElementsAndRates {
        a0: 0.72333566,
        a_dot: 0.00000390,
        e0: 0.00677672,
        e_dot: -0.00004107,
        I0: 3.39467605,
        I_dot: -0.00078890,
        L0: 181.97909950,
        L_dot: 58517.81538729,
        long_peri0: 131.60246718,
        long_peri_dot: 0.00268329,
        long_node0: 76.67984255,
        long_node_dot: -0.27769418,
    },
    // EM Bary
    ElementsAndRates {
        a0: 1.00000261,
        a_dot: 0.00000562,
        e0: 0.01671123,
        e_dot: -0.00004392,
        I0: -0.00001531,
        I_dot: -0.01294668,
        L0: 100.46457166,
        L_dot: 35999.37244981,
        long_peri0: 102.93768193,
        long_peri_dot: 0.32327364,
        long_node0: 0.0,
        long_node_dot: 0.0,
    },
    // Mars
    ElementsAndRates {
        a0: 1.52371034,
        a_dot: 0.00001847,
        e0: 0.09339410,
        e_dot: 0.00007882,
        I0: 1.84969142,
        I_dot: -0.00813131,
        L0: -4.55343205,
        L_dot: 19140.30268499,
        long_peri0: -23.94362959,
        long_peri_dot: 0.44441088,
        long_node0: 49.55953891,
        long_node_dot: -0.29257343,
    },
    // Jupiter
    ElementsAndRates {
        a0: 5.20288700,
        a_dot: -0.00011607,
        e0: 0.04838624,
        e_dot: -0.00013253,
        I0: 1.30439695,
        I_dot: -0.00183714,
        L0: 34.39644051,
        L_dot: 3034.74612775,
        long_peri0: 14.72847983,
        long_peri_dot: 0.21252668,
        long_node0: 100.47390909,
        long_node_dot: 0.20469106,
    },
    // Saturn
    ElementsAndRates {
        a0: 9.53667594,
        a_dot: -0.00125060,
        e0: 0.05386179,
        e_dot: -0.00050991,
        I0: 2.48599187,
        I_dot: 0.00193609,
        L0: 49.95424423,
        L_dot: 1222.49362201,
        long_peri0: 92.59887831,
        long_peri_dot: -0.41897216,
        long_node0: 113.66242448,
        long_node_dot: -0.28867794,
    },
    // Uranus
    ElementsAndRates {
        a0: 19.18916464,
        a_dot: -0.00196176,
        e0: 0.04725744,
        e_dot: -0.00004397,
        I0: 0.77263783,
        I_dot: -0.00242939,
        L0: 313.23810451,
        L_dot: 428.48202785,
        long_peri0: 170.95427630,
        long_peri_dot: 0.40805281,
        long_node0: 74.01692503,
        long_node_dot: 0.04240589,
    },
    // Neptune
    ElementsAndRates {
        a0: 30.06992276,
        a_dot: 0.00026291,
        e0: 0.00859048,
        e_dot: 0.00005105,
        I0: 1.77004347,
        I_dot: 0.00035372,
        L0: -55.12002969,
        L_dot: 218.45945325,
        long_peri0: 44.96476227,
        long_peri_dot: -0.32241464,
        long_node0: 131.78422574,
        long_node_dot: -0.00508664,
    },
];

// ── Table 2a: 3000 BC -- 3000 AD ─────────────────────────────────────────

const TABLE2A: [ElementsAndRates; 8] = [
    // Mercury
    ElementsAndRates {
        a0: 0.38709843,
        a_dot: 0.00000000,
        e0: 0.20563661,
        e_dot: 0.00002123,
        I0: 7.00559432,
        I_dot: -0.00590158,
        L0: 252.25166724,
        L_dot: 149472.67486623,
        long_peri0: 77.45771895,
        long_peri_dot: 0.15940013,
        long_node0: 48.33961819,
        long_node_dot: -0.12214182,
    },
    // Venus
    ElementsAndRates {
        a0: 0.72332102,
        a_dot: -0.00000026,
        e0: 0.00676399,
        e_dot: -0.00005107,
        I0: 3.39777545,
        I_dot: 0.00043494,
        L0: 181.97970850,
        L_dot: 58517.81560260,
        long_peri0: 131.76755713,
        long_peri_dot: 0.05679648,
        long_node0: 76.67261496,
        long_node_dot: -0.27274174,
    },
    // EM Bary
    ElementsAndRates {
        a0: 1.00000018,
        a_dot: -0.00000003,
        e0: 0.01673163,
        e_dot: -0.00003661,
        I0: -0.00054346,
        I_dot: -0.01337178,
        L0: 100.46691572,
        L_dot: 35999.37306329,
        long_peri0: 102.93005885,
        long_peri_dot: 0.31795260,
        long_node0: -5.11260389,
        long_node_dot: -0.24123856,
    },
    // Mars
    ElementsAndRates {
        a0: 1.52371243,
        a_dot: 0.00000097,
        e0: 0.09336511,
        e_dot: 0.00009149,
        I0: 1.85181869,
        I_dot: -0.00724757,
        L0: -4.56813164,
        L_dot: 19140.29934243,
        long_peri0: -23.91744784,
        long_peri_dot: 0.45223625,
        long_node0: 49.71320984,
        long_node_dot: -0.26852431,
    },
    // Jupiter
    ElementsAndRates {
        a0: 5.20248019,
        a_dot: -0.00002864,
        e0: 0.04853590,
        e_dot: 0.00018026,
        I0: 1.29861416,
        I_dot: -0.00322699,
        L0: 34.33479152,
        L_dot: 3034.90371757,
        long_peri0: 14.27495244,
        long_peri_dot: 0.18199196,
        long_node0: 100.29282654,
        long_node_dot: 0.13024619,
    },
    // Saturn
    ElementsAndRates {
        a0: 9.54149883,
        a_dot: -0.00003065,
        e0: 0.05550825,
        e_dot: -0.00032044,
        I0: 2.49424102,
        I_dot: 0.00451969,
        L0: 50.07571329,
        L_dot: 1222.11494724,
        long_peri0: 92.86136063,
        long_peri_dot: 0.54179478,
        long_node0: 113.63998702,
        long_node_dot: -0.25015002,
    },
    // Uranus
    ElementsAndRates {
        a0: 19.18797948,
        a_dot: -0.00020455,
        e0: 0.04685740,
        e_dot: -0.00001550,
        I0: 0.77298127,
        I_dot: -0.00180155,
        L0: 314.20276625,
        L_dot: 428.49512595,
        long_peri0: 172.43404441,
        long_peri_dot: 0.09266985,
        long_node0: 73.96250215,
        long_node_dot: 0.05739699,
    },
    // Neptune
    ElementsAndRates {
        a0: 30.06952752,
        a_dot: 0.00006447,
        e0: 0.00895439,
        e_dot: 0.00000818,
        I0: 1.77005520,
        I_dot: 0.00022400,
        L0: 304.22289287,
        L_dot: 218.46515314,
        long_peri0: 46.68158724,
        long_peri_dot: 0.01009938,
        long_node0: 131.78635853,
        long_node_dot: -0.00606302,
    },
];

// ── Table 2b: extra terms for Jupiter--Neptune (3000 BC -- 3000 AD) ──────

const TABLE2B: [ExtraTerms; 4] = [
    // Jupiter
    ExtraTerms { b: -0.00012452, c: 0.06064060, s: -0.35635438, f: 38.35125000 },
    // Saturn
    ExtraTerms { b: 0.00025899, c: -0.13434469, s: 0.87320147, f: 38.35125000 },
    // Uranus
    ExtraTerms { b: 0.00058331, c: -0.97731848, s: 0.17689245, f: 7.67025000 },
    // Neptune
    ExtraTerms { b: -0.00041348, c: 0.68346318, s: -0.10162547, f: 7.67025000 },
];

fn planet_index(planet: Planet) -> usize {
    planet as usize
}

fn planet_elements(
    planet: Planet,
    range: TimeRange,
) -> (&'static ElementsAndRates, Option<&'static ExtraTerms>) {
    let idx = planet_index(planet);
    match range {
        TimeRange::Modern => (&TABLE1[idx], None),
        TimeRange::Extended => {
            let extra = match planet {
                Planet::Jupiter => Some(&TABLE2B[0]),
                Planet::Saturn => Some(&TABLE2B[1]),
                Planet::Uranus => Some(&TABLE2B[2]),
                Planet::Neptune => Some(&TABLE2B[3]),
                _ => None,
            };
            (&TABLE2A[idx], extra)
        }
    }
}

/// Solve Kepler's equation M = E - e* sin(E) in degrees.
/// e_star = (180/pi) * e ≈ 57.29578 * e.
fn solve_kepler_deg(M: f64, e_star: f64, e: f64) -> f64 {
    let mut E = M + e_star * M.to_radians().sin();
    for _ in 0..100 {
        let dM = M - (E - e_star * E.to_radians().sin());
        let dE = dM / (1.0 - e * E.to_radians().cos());
        E += dE;
        if dE.abs() < 1e-13 {
            break;
        }
    }
    E
}

/// Normalize angle to [-180, +180] degrees.
fn normalize_deg(mut deg: f64) -> f64 {
    deg %= 360.0;
    if deg > 180.0 {
        deg -= 360.0;
    } else if deg < -180.0 {
        deg += 360.0;
    }
    deg
}

/// Keplerian elements evaluated at a specific epoch.
/// Semi-major axis in AU, eccentricity dimensionless, angles in radians.
pub struct EvaluatedElements {
    /// Semi-major axis [AU]
    pub a: f64,
    /// Eccentricity
    pub e: f64,
    /// Inclination [rad]
    pub I: f64,
    /// Longitude of the ascending node [rad]
    pub Omega: f64,
    /// Argument of perihelion [rad]
    pub omega: f64,
    /// Mean anomaly [rad], normalized to [-pi, +pi]
    pub M: f64,
}

/// Compute Keplerian elements for a planet at a given Julian Date.
pub fn orbital_elements(planet: Planet, jd: f64, range: TimeRange) -> EvaluatedElements {
    let T = (jd - J2000_JD) / DAYS_PER_CENTURY;
    let (el, extra) = planet_elements(planet, range);

    let a = el.a0 + el.a_dot * T;
    let e = el.e0 + el.e_dot * T;
    let I = el.I0 + el.I_dot * T;
    let L = el.L0 + el.L_dot * T;
    let long_peri = el.long_peri0 + el.long_peri_dot * T;
    let long_node = el.long_node0 + el.long_node_dot * T;

    let omega = long_peri - long_node;

    let mut M = L - long_peri;
    if let Some(ex) = extra {
        let f_T = (ex.f * T).to_radians();
        M += ex.b * T * T + ex.c * f_T.cos() + ex.s * f_T.sin();
    }
    let M = normalize_deg(M);

    EvaluatedElements {
        a,
        e,
        I: I.to_radians(),
        Omega: long_node.to_radians(),
        omega: omega.to_radians(),
        M: M.to_radians(),
    }
}

/// Heliocentric position in the J2000 ecliptic frame, in AU.
pub fn heliocentric_ecliptic_position(planet: Planet, jd: f64, range: TimeRange) -> Vector3<f64> {
    let el = orbital_elements(planet, jd, range);

    let e_star = 57.29577951308232 * el.e;
    let E_deg = solve_kepler_deg(el.M.to_degrees(), e_star, el.e);
    let E_rad = E_deg.to_radians();

    let (sin_E, cos_E) = E_rad.sin_cos();
    let x_prime = el.a * (cos_E - el.e);
    let y_prime = el.a * (1.0 - el.e * el.e).sqrt() * sin_E;

    let (sin_w, cos_w) = el.omega.sin_cos();
    let (sin_O, cos_O) = el.Omega.sin_cos();
    let (sin_I, cos_I) = el.I.sin_cos();

    let x_ecl = (cos_w * cos_O - sin_w * sin_O * cos_I) * x_prime
        + (-sin_w * cos_O - cos_w * sin_O * cos_I) * y_prime;
    let y_ecl = (cos_w * sin_O + sin_w * cos_O * cos_I) * x_prime
        + (-sin_w * sin_O + cos_w * cos_O * cos_I) * y_prime;
    let z_ecl = (sin_w * sin_I) * x_prime + (cos_w * sin_I) * y_prime;

    Vector3::new(x_ecl, y_ecl, z_ecl)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AU_IN_KM;
    use crate::orbit::flat::elliptic::EllipticOrbit;
    use crate::orbit::orbit_3d::{KeplerianElements, Orbit3D};
    use crate::state_vectors::StateVectors;
    use crate::time::Time;
    use nalgebra::Matrix3x2;
    use similar_asserts::assert_eq;

    /// Build an Orbit3D from JPL approximate elements at the Voyager 2 flyby epoch,
    /// then compare position_and_velocity(t=0) with the Horizons reference.
    #[test]
    fn jupiter_orbit3d_from_jpl_elements_at_voyager_2_flyby() {
        let expected_r = [-6.332981769456728E+08, 4.887724313507157E+08, 1.217737175266561E+07];
        let expected_v = [-8.145624834811183E+00, -9.746651242098165E+00, 2.226261710927915E-01];
        test_state_vectors_at("1979-09-07T22:29:51Z", expected_r, expected_v);
    }

    /// Build an Orbit3D from JPL approximate elements at J2000 epoch
    /// then compare position_and_velocity(t=0) with the Horizons reference.
    #[test]
    fn jupiter_orbit3d_from_jpl_elements_at_j2000() {
        let expected_r = [5.985671169709978E+08, 4.396053959974722E+08, -1.522685327350616E+07];
        let expected_v = [-7.909871473863137E+00, 1.115620857802522E+01, 1.308660270810371E-01];
        test_state_vectors_at("2000-01-01T12:00:00Z", expected_r, expected_v);
    }

    fn datetime_to_jd(datetime_rfc_3339: &str) -> f64 {
        let dt = chrono::DateTime::parse_from_rfc3339(datetime_rfc_3339).unwrap().to_utc();
        let j2000_epoch =
            chrono::DateTime::parse_from_rfc3339("2000-01-01T12:00:00Z").unwrap().to_utc();
        J2000_JD + (dt - j2000_epoch).as_seconds_f64() / 86400.0
    }

    fn test_state_vectors_at(datetime: &str, expected_r: [f64; 3], expected_v: [f64; 3]) {
        let jd = datetime_to_jd(datetime);
        let el = orbital_elements(Planet::Jupiter, jd, TimeRange::Modern);

        // Sun μ + Jupiter μ (km³/s²)
        let mu = 132_712_440_042.0 + 1.266_865_319e8;

        let orbit = Orbit3D::<EllipticOrbit>::from(KeplerianElements {
            mu,
            a: el.a * AU_IN_KM,
            e: el.e,
            i: el.I,
            Omega: el.Omega,
            omega: el.omega,
            M0: el.M,
        });

        let computed = orbit.position_and_velocity(Time::from_secs(0.0));
        let expected = (Vector3::from(expected_r), Vector3::from(expected_v));

        let abs_err_r = (computed.0 - expected.0).magnitude();
        let rel_err_r = abs_err_r / expected.0.magnitude() * 100.0;
        let abs_err_v = (computed.1 - expected.1).magnitude();
        let rel_err_v = abs_err_v / expected.1.magnitude() * 100.0;

        eprintln!("Absolute error for r: {abs_err_r:.6} km");
        eprintln!("Relative error for r: {rel_err_r:.6} %");
        eprintln!("Absolute error for v: {abs_err_v:.6} km/s");
        eprintln!("Relative error for v: {rel_err_v:.6} %");
        let expected = Matrix3x2::from_columns(&[expected.0, expected.1]);
        let computed = Matrix3x2::from_columns(&[computed.0, computed.1]);

        assert_eq!(format!("{computed:.3}"), format!("{expected:.3}"));
    }
}
