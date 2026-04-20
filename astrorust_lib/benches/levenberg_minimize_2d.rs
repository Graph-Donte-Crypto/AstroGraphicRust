use std::time::Duration;

use astrorust_lib::config::{Config, StarSystem};
use astrorust_lib::angle::{Angle, EccAnomaly, HypAnomaly};
use astrorust_lib::soi_minima::{
    levenberg_minimize_2d, nodal_initial_guesses, radial_initial_guesses, scaled_coupling_matrix,
};
use astrorust_lib::orbit::flat::elliptic::EllipticOrbit;
use astrorust_lib::orbit::flat::hyperbolic::HyperbolicOrbit;
use astrorust_lib::orbit::orbit_3d::{KeplerianElements, Orbit3D};
use astrorust_lib::state_vectors::StateVectors;
use astrorust_lib::trajectory::Trajectory;
use criterion::{Bencher, Criterion, black_box, criterion_group, criterion_main};
use nalgebra::Matrix2;

/// One benchmark case: a pre-seeded (trajectory, planet, seed, bounds, M, r_soi²)
/// tuple so each iteration just runs the solver on warm inputs.
struct Case {
    name: &'static str,
    traj: Trajectory,
    planet: Orbit3D<EllipticOrbit>,
    seed: (f64, f64),
    bounds: (f64, f64),
    coupling: Matrix2<f64>,
    r_soi_sq: f64,
}

fn cases() -> Vec<Case> {
    let config_dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../config");
    let config = Config::load_from_yaml(&format!("{config_dir}/config.yml")).unwrap();
    let system = StarSystem::load_from_yaml(&format!("{config_dir}/system/solar.yml")).unwrap();

    let elliptic_sc: Orbit3D<EllipticOrbit> =
        KeplerianElements::from(config.spacecraft.orbit).into();
    let elliptic_traj = Trajectory::Elliptic(elliptic_sc);

    // Voyager 2 Jupiter-to-Saturn hyperbolic orbit.
    let hyperbolic_sc: Orbit3D<HyperbolicOrbit> = KeplerianElements {
        mu: system.star.μ,
        a: -2220315000.0,
        e: 1.338264,
        i: 2.582320_f64.to_radians(),
        Omega: 119.196938_f64.to_radians(),
        omega: (-9.170896_f64).to_radians(),
        M0: 1.65,
    }
    .into();
    let hyperbolic_traj = Trajectory::Hyperbolic(hyperbolic_sc);

    let pairs: [(&'static str, &Trajectory, &[&'static str]); 2] = [
        ("voyager2", &elliptic_traj, &["Earth", "Mars", "Jupiter"]),
        ("voyager2_hyp", &hyperbolic_traj, &["Saturn"]),
    ];

    let mut out = Vec::new();
    for (sc_label, traj, planets) in pairs {
        for name in planets {
            let planet_cfg = system.planets.iter().find(|p| p.body.name == *name).unwrap();
            let planet: Orbit3D<EllipticOrbit> =
                KeplerianElements::from(planet_cfg.orbit.clone()).into();
            let r_soi = planet_cfg.soi_radius();
            let coupling = scaled_coupling_matrix(traj, &planet);
            // Bench both the positive and negative arcs — the two seeds land
            // in distinct basins with different convergence behavior. For each
            // branch, use the best-of-{radial, nodal} seed that find_soi_minima
            // would actually pick, so the bench reflects production behavior.
            let radial = radial_initial_guesses(traj, &planet, r_soi);
            let nodal = nodal_initial_guesses(traj, &planet);
            let f_at = |E1: f64, E2: f64| -> f64 {
                let p1 = match traj {
                    Trajectory::Elliptic(o) => o.position(EccAnomaly::from(Angle::from_rad(E1))),
                    Trajectory::Hyperbolic(o) => o.position(HypAnomaly::from(E1)),
                };
                let p2 = planet.position(EccAnomaly::from(Angle::from_rad(E2)));
                (p1 - p2).norm_squared()
            };
            for (arc_label, &((E1_r, E2_r), bounds)) in
                [("pos", &radial[0]), ("neg", &radial[1])]
            {
                let (lo, hi) = bounds;
                let (mut E1, mut E2, mut f) = (E1_r, E2_r, f_at(E1_r, E2_r));
                if let Some(nodal_seeds) = &nodal {
                    for &(E1_n, E2_n) in nodal_seeds {
                        if E1_n >= lo && E1_n <= hi {
                            let f_n = f_at(E1_n, E2_n);
                            if f_n < f {
                                (E1, E2, f) = (E1_n, E2_n, f_n);
                            }
                        }
                    }
                }
                let case_name: &'static str =
                    Box::leak(format!("{sc_label}-{name}/{arc_label}").into_boxed_str());
                out.push(Case {
                    name: case_name,
                    traj: traj.clone(),
                    planet: planet.clone(),
                    seed: (E1, E2),
                    bounds,
                    coupling,
                    r_soi_sq: r_soi * r_soi,
                });
            }
        }
    }
    out
}

pub fn bench(c: &mut Criterion) {
    let cases = cases();

    let mut group = c.benchmark_group("levenberg_minimize_2d");
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_millis(2000));

    for case in &cases {
        group.bench_function(case.name, |b: &mut Bencher| {
            b.iter(|| {
                levenberg_minimize_2d(
                    black_box(case.seed.0),
                    black_box(case.seed.1),
                    &case.traj,
                    &case.planet,
                    &case.coupling,
                    case.r_soi_sq,
                    case.bounds,
                )
            });
        });
    }
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
