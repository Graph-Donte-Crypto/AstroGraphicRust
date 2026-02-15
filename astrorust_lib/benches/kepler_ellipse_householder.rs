use std::time::Duration;

use astrorust_lib::angle::Angle;
use astrorust_lib::kepler_equation;
use criterion::{criterion_group, criterion_main, Bencher, Criterion};

pub fn bench(c: &mut Criterion) {
    let eccentricities = [0.01, 0.1, 0.3, 0.5, 0.7, 0.9, 0.9999999];
    let mean_anomalies = [0.01, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 5.5, 6.0, 6.283];

    let mut group = c.benchmark_group("ellipse");

    group.warm_up_time(Duration::from_millis(1000));
    group.measurement_time(Duration::from_millis(2000));

    for e in eccentricities {
        group.throughput(criterion::Throughput::Elements(mean_anomalies.len() as u64));
        group.bench_function(format!("{}", e).as_str(), |b: &mut Bencher| {
            b.iter(|| {
                for m in mean_anomalies {
                    let m = Angle::from_rad(m).into();
                    kepler_equation::solve_kepler_householder_pade_elliptic(
                        e,
                        criterion::black_box(m),
                    );
                }
            });
        });
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
