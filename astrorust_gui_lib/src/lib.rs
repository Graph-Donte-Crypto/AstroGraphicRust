pub use kiss3d::{self, nalgebra as na};

use astrorust_lib::angle::{Angle, EccAnomaly, HypAnomaly};
use astrorust_lib::orbit::flat::elliptic::EllipticOrbit;
use astrorust_lib::orbit::flat::hyperbolic::HyperbolicOrbit;
use astrorust_lib::orbit::orbit_3d::Orbit3D;
use astrorust_lib::state_vectors::StateVectors;
use astrorust_lib::trajectory::Trajectory;
use core::f64::consts::TAU;
use na::Point3;

pub fn generate_trajectory_points(
    soi_radius: f64,
    trajectory: &Trajectory,
    count: usize,
) -> Vec<Point3<f64>> {
    match trajectory {
        Trajectory::Elliptic(orbit) => generate_ellipse_points(orbit, count),
        Trajectory::Hyperbolic(orbit) => generate_hyperbola_points(orbit, soi_radius, count),
    }
}

/// E -- eccentric anomaly
#[allow(non_snake_case)]
pub fn generate_ellipse_points(orbit: &Orbit3D<EllipticOrbit>, count: usize) -> Vec<Point3<f64>> {
    let mut E: f64 = 0.0;
    let mut points = Vec::with_capacity(count);
    let dE = TAU / (count as f64);
    for _ in 0..count {
        points.push(orbit.position(EccAnomaly::from(Angle::from_rad(E))).into());
        E += dE;
    }
    points
}

/// H -- hyperbolic anomaly
#[allow(non_snake_case)]
pub fn generate_hyperbola_points(
    orbit: &Orbit3D<HyperbolicOrbit>,
    soi_radius: f64,
    count: usize,
) -> Vec<Point3<f64>> {
    let H_max = ((orbit.orbit_2d.0.a() - soi_radius)
        / (orbit.orbit_2d.0.a() * orbit.orbit_2d.0.e()))
    .acosh();
    let H_min = -H_max;
    let mut H = H_min;
    let mut points = Vec::with_capacity(count);
    let dH = 2.0 * H_max / (count as f64);
    for _ in 0..count {
        points.push(orbit.position(HypAnomaly::from(H)).into());
        H += dH;
    }
    points
}

pub fn draw_orbit_points(
    window: &mut kiss3d::window::Window,
    points: &[Point3<f32>],
    color: &Point3<f32>,
    is_hyperbolic: bool,
) {
    for i in 0..points.len() - 1 {
        window.draw_line(&points[i], &points[i + 1], color);
        // window.draw_point(&points[i], color);
    }
    if !is_hyperbolic {
        window.draw_line(&points[0], &points[points.len() - 1], color);
    }
}

pub fn draw_full_axes(w: &mut kiss3d::window::Window, box_size: f32, margin: f32) {
    const COLOR: Point3<f32> = Point3::new(0.2, 0.2, 0.2);
    w.draw_line(&Point3::from([margin, 0.0, 0.0]), &Point3::from([box_size, 0.0, 0.0]), &COLOR);
    w.draw_line(&Point3::from([0.0, margin, 0.0]), &Point3::from([0.0, box_size, 0.0]), &COLOR);
    w.draw_line(&Point3::from([0.0, 0.0, margin]), &Point3::from([0.0, 0.0, box_size]), &COLOR);
    w.draw_line(&Point3::from([-margin, 0.0, 0.0]), &Point3::from([-box_size, 0.0, 0.0]), &COLOR);
    w.draw_line(&Point3::from([0.0, -margin, 0.0]), &Point3::from([0.0, -box_size, 0.0]), &COLOR);
    w.draw_line(&Point3::from([0.0, 0.0, -margin]), &Point3::from([0.0, 0.0, -box_size]), &COLOR);
}

pub fn draw_positive_axes(w: &mut kiss3d::window::Window, box_size: f32, margin: f32) {
    w.draw_line(
        &Point3::from([margin, 0.0, 0.0]),
        &Point3::from([box_size, 0.0, 0.0]),
        &Point3::from([1.0, 0.0, 0.0]),
    );
    w.draw_line(
        &Point3::from([0.0, margin, 0.0]),
        &Point3::from([0.0, box_size, 0.0]),
        &Point3::from([0.0, 1.0, 0.0]),
    );
    w.draw_line(
        &Point3::from([0.0, 0.0, margin]),
        &Point3::from([0.0, 0.0, box_size]),
        &Point3::from([0.0, 0.0, 1.0]),
    );
}
