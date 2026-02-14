pub use kiss3d::nalgebra as na;
pub use {kiss3d, kiss3d_trackball};

use astrorust_lib::angle::{Angle, EccAnomaly};
use astrorust_lib::orbit::flat::elliptic::EllipticOrbit;
use astrorust_lib::orbit::orbit_3d::Orbit3D;
use astrorust_lib::state_vectors::StateVectors;
use core::f64::consts::TAU;
use na::Point3;

type Orbit = Orbit3D<EllipticOrbit>;

/// E -- eccentric anomaly
#[allow(non_snake_case)]
pub fn generate_orbit_points(orbit: &Orbit, count: usize) -> Vec<Point3<f32>> {
    let mut E: f64 = 0.0;
    let mut points = Vec::with_capacity(count);
    let dE = TAU / (count as f64);
    for _ in 0..count {
        let r = orbit.position(EccAnomaly::from(Angle::from_rad(E)));
        points.push(Point3::new(r.x as f32, r.y as f32, r.z as f32));
        E += dE;
    }
    points
}

pub fn draw_orbit_points(
    window: &mut kiss3d::window::Window,
    points: &[Point3<f32>],
    color: &Point3<f32>,
) {
    for i in 0..points.len() - 1 {
        window.draw_line(&points[i], &points[i + 1], color);
        //window.draw_point(&points[i], color);
    }
    window.draw_line(&points[0], &points[points.len() - 1], color);
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
