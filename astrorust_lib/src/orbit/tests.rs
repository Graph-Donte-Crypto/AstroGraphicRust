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
        Vector3::new(-6.332981769456728E+08, 4.887724313507157E+08, 1.217737175266561E+07),
        Vector3::new(-8.145624834811183E+00, -9.746651242098165E+00, 2.226261710927915E-01),
    );
    let computed = orbit.position_and_velocity(t);
    assert_eq!(computed, expected);
}
