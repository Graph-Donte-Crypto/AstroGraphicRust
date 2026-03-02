use crate::config::StarSystem;
use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::orbit_3d::{KeplerianElements, Orbit3D};
use crate::state_vectors::StateVectors;
use crate::time::Time;
use nalgebra::Vector3;
use similar_asserts::assert_eq;

/// Reference data from Horizons <https://ssd.jpl.nasa.gov/horizons/app.html>
#[test]
fn jupiter_at_time_of_voyager_2_flyby_periapsis() {
    let system = StarSystem::load_from_yaml(&format!("../config/system/solar.yml")).unwrap();
    let orbit =
        Orbit3D::<EllipticOrbit>::from(KeplerianElements::from(system.planets[4].orbit.clone()));
    let t = chrono::DateTime::parse_from_rfc3339("1979-09-07T22:29:51Z").unwrap().to_utc();
    let t: Time = (t - system.t0).into();
    let expected = (
        Vector3::<f64>::new(-6.332981769456728E+08, 4.887724313507157E+08, 1.217737175266561E+07),
        Vector3::<f64>::new(-8.145624834811183E+00, -9.746651242098165E+00, 2.226261710927915E-01),
    );
    let computed = orbit.position_and_velocity(t);
    eprintln!(
        "Relative error for r, %: {}",
        ((computed.0 - expected.0).magnitude() / expected.0.magnitude()) * 100.0
    );
    eprintln!(
        "Relative error for v, %: {}",
        ((computed.1 - expected.1).magnitude() / expected.1.magnitude()) * 100.0
    );
    assert_eq!(computed, expected);
}

/// Reference data from Horizons <https://ssd.jpl.nasa.gov/horizons/app.html>
#[test]
fn jupiter_at_j2000() {
    let system = StarSystem::load_from_yaml(&format!("../config/system/solar.yml")).unwrap();
    let orbit =
        Orbit3D::<EllipticOrbit>::from(KeplerianElements::from(system.planets[4].orbit.clone()));
    let t = chrono::DateTime::parse_from_rfc3339("2000-01-01T12:00:00Z").unwrap().to_utc();
    let t: Time = (t - system.t0).into();
    let expected = (
        Vector3::<f64>::new(5.985671169709978E+08, 4.396053959974722E+08, -1.522685327350616E+07),
        Vector3::<f64>::new(-7.909871473863137E+00, 1.115620857802522E+01, 1.308660270810371E-01),
    );
    let computed = orbit.position_and_velocity(t);
    eprintln!(
        "Relative error for r, %: {}",
        ((computed.0 - expected.0).magnitude() / expected.0.magnitude()) * 100.0
    );
    eprintln!(
        "Relative error for v, %: {}",
        ((computed.1 - expected.1).magnitude() / expected.1.magnitude()) * 100.0
    );
    assert_eq!(computed, expected);
}
