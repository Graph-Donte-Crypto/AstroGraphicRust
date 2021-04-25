// #[macro_use(lazy_static)]
// extern crate lazy_static;

pub use nalgebra;
pub use kiss3d;

use core::f32::consts::TAU;
use kiss3d::{
	event::{Key, MouseButton},
	light::Light,
	nalgebra as na,
	window::Window
};
use na::{Point3, Translation3, UnitQuaternion, Vector3};
use astrorust_lib::orbit::Orbit;

pub trait Drawable {
	fn generate_points(&self, count: usize) -> Vec<Point3<f32>>;
}

impl Drawable for Orbit {
	fn generate_points(&self, count: usize) -> Vec<Point3<f32>> {
		let mut nu: f32 = 0.0;
		let mut points = vec![];
		for _ in 0..count {
			let point = Point3::from(self.r_from_nu(nu));
			points.push(point);
			nu += TAU / (count as f32);
		}
		points
	}
}

pub fn draw_orbit(window: &mut kiss3d::window::Window, orbit: &Orbit, color: &Point3<f32>) {
	let points = orbit.generate_points(100);
	for i in 0..points.len() - 1 {
		window.draw_line(&(points[i]), &(points[i + 1]), &color);
	}
	window.draw_line(&(points[0]), &(points[points.len() - 1]), &color);
}
// 
// pub fn draw_space_body(window: &mut kiss3d::window::Window, space_body: &SpaceBody) {}
// 
// pub fn draw_space_body_with_orbit(
// 	window: &mut kiss3d::window::Window,
// 	space_body: &SpaceBody,
// 	color: &Point3<f32>,
// ) {
// 	draw_orbit(window, &(space_body.orbit), color);
// 	draw_space_body(window, space_body);
// }
