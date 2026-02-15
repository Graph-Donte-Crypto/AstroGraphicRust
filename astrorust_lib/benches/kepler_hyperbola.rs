use std::time::Duration;

use astrorust_lib::orbit::flat::hyperbolic::HyperbolaSolver;
use criterion::{criterion_group, criterion_main, Bencher, Criterion};

pub fn bench(c: &mut Criterion) {
    let eccentricities = [1.01, 1.1, 1.3, 1.5, 1.7, 1.9, 1.99];
    let mean_anomalies = [0.01, 0.5, 1.0, 2.0, 4.0, 8.0, 32.0, 128.0, 999.0, 9999999.0];

    let mut group = c.benchmark_group("hyperbola");

    group.warm_up_time(Duration::from_millis(1000));
    group.measurement_time(Duration::from_millis(2000));

    for e in eccentricities {
        let solver = HyperbolaSolver::new(e);
        group.throughput(criterion::Throughput::Elements(mean_anomalies.len() as u64));
        group.bench_function(format!("{}", e).as_str(), |b: &mut Bencher| {
            b.iter(|| {
                for m in &mean_anomalies {
                    solver.solve(criterion::black_box(*m));
                }
            });
        });
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
