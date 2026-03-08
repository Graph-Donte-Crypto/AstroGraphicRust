use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::orbit_3d::Orbit3D;
use crate::trajectory::Trajectory;

/// Coarse filter: heliocentric distance bounds
pub fn is_encounter_possible(
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
) -> bool {
    spacecraft.apoapsis() + r_soi >= planet.periapsis()
        && spacecraft.periapsis() - r_soi <= planet.apoapsis()
}
