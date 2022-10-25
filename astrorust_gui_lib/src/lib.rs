pub use kiss3d::nalgebra as na;
pub use kiss3d;
pub use kiss3d_trackball;

use core::f32::consts::TAU;
use na::{Point3};
use astrorust_lib::orbit::Orbit;

pub fn generate_orbit_points(orbit: &Orbit, count: usize) -> Vec<Point3<f32>> {
	let mut nu: f32 = 0.0;
	let mut points = Vec::with_capacity(count);
	let d_nu = TAU / (count as f32);
	for _ in 0..count - 1 {
		let point = Point3::from(orbit.r_from_nu(nu));
		points.push(point);
		nu += d_nu;
	}
	points
}

pub fn draw_orbit_points(
	window: &mut kiss3d::window::Window,
	points: &Vec<Point3<f32>>,
	color: &Point3<f32>,
) {
	for i in 0..points.len() - 1 {
		window.draw_line(&(points[i]), &(points[i + 1]), color);
	}
	window.draw_line(&(points[0]), &(points[points.len() - 1]), color);
}

pub fn draw_full_axes(w: &mut kiss3d::window::Window, box_size: f32, margin: f32) {
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
	w.draw_line(
		&Point3::from([-margin, 0.0, 0.0]),
		&Point3::from([-box_size, 0.0, 0.0]),
		&Point3::from([1.0, 0.0, 0.0]),
	);
	w.draw_line(
		&Point3::from([0.0, -margin, 0.0]),
		&Point3::from([0.0, -box_size, 0.0]),
		&Point3::from([0.0, 1.0, 0.0]),
	);
	w.draw_line(
		&Point3::from([0.0, 0.0, -margin]),
		&Point3::from([0.0, 0.0, -box_size]),
		&Point3::from([0.0, 0.0, 1.0]),
	);
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
