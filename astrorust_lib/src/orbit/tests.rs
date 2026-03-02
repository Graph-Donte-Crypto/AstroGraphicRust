//! Reference data from Horizons <https://ssd.jpl.nasa.gov/horizons/app.html>

use crate::config::{Config, Spacecraft, StarSystem};
use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::orbit_3d::{KeplerianElements, Orbit3D};
use crate::state_vectors::StateVectors;
use crate::time::Time;
use nalgebra::Vector3;
use similar_asserts::assert_eq;
use std::f64::consts::TAU;
use std::sync::LazyLock as Lazy;

static SOLAR_SYSTEM: Lazy<StarSystem> =
    Lazy::new(|| StarSystem::load_from_yaml("../config/system/solar.yml").unwrap());
static VOYAGER_2: Lazy<Spacecraft> =
    Lazy::new(|| Config::load_from_yaml("../config/config.yml").unwrap().spacecraft);
static VOYAGER_2_ORBIT: Lazy<Orbit3D<EllipticOrbit>> = Lazy::new(|| {
    let mut orbit = VOYAGER_2.orbit.clone();
    dbg!(orbit.mu);
    orbit.M0 = (orbit.M0
        + ((SOLAR_SYSTEM.t0 - VOYAGER_2.t0).as_seconds_f64()
            * (orbit.mu / orbit.a.powi(3)).sqrt())
            % TAU)
        % TAU;
    Orbit3D::<EllipticOrbit>::from(KeplerianElements::from(orbit))
});

#[test]
fn jupiter_at_time_of_voyager_2_flyby_periapsis() {
    test_jupiter_state_vectors_at_t(
        "1979-09-07T22:29:51Z",
        [-6.332981769456728E+08, 4.887724313507157E+08, 1.217737175266561E+07],
        [-8.145624834811183E+00, -9.746651242098165E+00, 2.226261710927915E-01],
    )
}

#[test]
fn jupiter_at_j2000() {
    test_jupiter_state_vectors_at_t(
        "2000-01-01T12:00:00Z",
        [5.985671169709978E+08, 4.396053959974722E+08, -1.522685327350616E+07],
        [-7.909871473863137E+00, 1.115620857802522E+01, 1.308660270810371E-01],
    )
}

#[test]
fn voyager_2_injection_to_jupiter() {
    test_voyager_2_state_vectors_at_t(
        "1977-08-23T11:29:11Z",
        [1.322849282606923E+08, -7.210582396926185E+07, 8.023135489867255E+05],
        [1.651796685882496E+01, 3.513574057743460E+01, 3.257046011774337E+00],
    )
}

#[test]
fn voyager_2_before_jupiter_encounter() {
    test_voyager_2_state_vectors_at_t(
        "1979-04-15T00:00:00Z",
        [-5.120225704239177E+08, 5.382843226253158E+08, 1.520282900189507E+07],
        [-1.067245524788600E+01, -4.397026196084053E-02, -5.144538804252702E-01],
    )
}

fn test_jupiter_state_vectors_at_t(
    datetime_rfc_3339: &str,
    expected_r: [f64; 3],
    expected_v: [f64; 3],
) {
    let orbit = Orbit3D::<EllipticOrbit>::from(KeplerianElements::from(
        SOLAR_SYSTEM.planets[4].orbit.clone(),
    ));
    test_state_vectors_at_t(datetime_rfc_3339, SOLAR_SYSTEM.t0, &orbit, expected_r, expected_v);
}

fn test_voyager_2_state_vectors_at_t(
    datetime_rfc_3339: &str,
    expected_r: [f64; 3],
    expected_v: [f64; 3],
) {
    // let t = chrono::DateTime::parse_from_rfc3339(datetime_rfc_3339).unwrap().to_utc();
    // let t: Time = (t - SOLAR_SYSTEM.t0).into();
    // let computed_orbit = Orbit3D::<EllipticOrbit>::from_state_vectors(
    //     SOLAR_SYSTEM.star.μ,
    //     expected_r.into(),
    //     expected_v.into(),
    //     t.as_secs(),
    // );
    // assert_eq!(computed_orbit, *VOYAGER_2_ORBIT);
    test_state_vectors_at_t(
        datetime_rfc_3339,
        SOLAR_SYSTEM.t0,
        &VOYAGER_2_ORBIT,
        expected_r,
        expected_v,
    );
}

fn test_state_vectors_at_t(
    datetime_rfc_3339: &str,
    t0: chrono::DateTime<chrono::Utc>,
    orbit: &Orbit3D<EllipticOrbit>,
    expected_r: [f64; 3],
    expected_v: [f64; 3],
) {
    let t = chrono::DateTime::parse_from_rfc3339(datetime_rfc_3339).unwrap().to_utc();
    let t: Time = (t - t0).into();
    let expected = (Vector3::from(expected_r), Vector3::from(expected_v));
    let computed = orbit.position_and_velocity(t);

    let abs_err_r = (computed.0 - expected.0).magnitude();
    let abs_err_v = (computed.1 - expected.1).magnitude();

    eprintln!("Absolute error for r: {abs_err_r:.6} km");
    eprintln!("Relative error for r: {:.6} %", abs_err_r / expected.0.magnitude() * 100.0);
    eprintln!("Absolute error for v: {abs_err_v:.6} km/s");
    eprintln!("Relative error for v: {:.6} %", abs_err_v / expected.1.magnitude() * 100.0);
    assert_eq!(computed, expected);
}
