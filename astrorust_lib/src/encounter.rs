use crate::orbit::flat::elliptic::EllipticOrbit;
use crate::orbit::orbit_3d::Orbit3D;
use crate::trajectory::Trajectory;

/// Coarse filter: heliocentric radius bounds (triangle inequality). Encounter is possible only if
/// the radial ranges get within `r_soi`. Uses the paper's cos E₁ formulation for elliptic s/c;
/// hyperbolic: r₁ ∈ [|a₁|(e₁−1), ∞), same “gap ≤ r_soi” check. See docs/soi_encounter_derivation.typ.
pub fn is_encounter_possible(
    spacecraft: &Trajectory,
    planet: &Orbit3D<EllipticOrbit>,
    r_soi: f64,
) -> bool {
    let (a1, e1) = (spacecraft.a(), spacecraft.e());
    let (a2, e2) = (planet.orbit_2d.0.a(), planet.orbit_2d.0.e());
    // Periapsis and apoapsis of planet
    let pe2 = a2 * (1.0 - e2);
    let ap2 = a2 * (1.0 + e2);

    match spacecraft {
        Trajectory::Elliptic(_) if e1 < 1e-12 => {
            // Circular: r₁ = a₁ constant. Possible iff a₁ ∈ [r2_lo - r_soi, r2_hi + r_soi].
            a1 >= pe2 - r_soi && a1 <= ap2 + r_soi
        }
        Trajectory::Elliptic(_) => {
            // r₁ = a₁(1 − e₁ cos E₁). Paper: (a₁ − a₂(1+e₂) − r_soi)/(a₁ e₁) ≤ cos E₁ ≤ (a₁ − a₂(1−e₂) + r_soi)/(a₁ e₁).
            // Clamp both sides to [-1, 1]. Interval empty ⇒ no encounter.
            let cos_lo = (a1 - ap2 - r_soi) / (a1 * e1);
            let cos_hi = (a1 - pe2 + r_soi) / (a1 * e1);
            cos_lo.max(-1.0) <= cos_hi.min(1.0)
        }
        Trajectory::Hyperbolic(_) => {
            let cosh_lo = (a1 - pe2 + r_soi) / (a1 * e1);
            let cosh_hi = (a1 - ap2 - r_soi) / (a1 * e1);
            cosh_lo.max(1.0) <= cosh_hi
        }
    }
}
