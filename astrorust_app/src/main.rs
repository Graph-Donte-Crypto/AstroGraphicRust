use astrorust_gui_lib as gui_lib;
use astrorust_lib as lib;
use gui_lib::kiss3d;
use gui_lib::kiss3d::{
	light::Light,
	window::Window,
};
use gui_lib::kiss3d::nalgebra::{Point3, UnitQuaternion, Translation3, Vector3};
use std::f32::consts::PI;

#[allow(non_snake_case)]
fn main() {
	let mut window = Window::new("Astro Graphic Rust");

	let mut sphere = window.add_sphere(-1.0);
	let mut planet = window.add_sphere(0.2);

	planet.set_color(0.6, 0.6, 0.6);
	sphere.set_color(1.0, 1.0, 0.0);
	//window.set_light(Light::StickToCamera);
	window.set_light(Light::Absolute(Point3::from([0.0, 0.0, 0.0])));
	
	let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);
	let orbit = lib::orbit::Orbit::new(1172.3328, 5.263138, 0.2, 7_f32.to_radians(), 15_f32.to_radians(), 70_f32.to_radians(), 0_f32.to_radians());
	let points = gui_lib::generate_orbit_points(&orbit, 500);
	window.set_line_width(1.0);

	let mut camera =
		gui_lib::kiss3d_trackball::Trackball::new(&Point3::from([10.0, 10.0, 10.0]), &Point3::origin(), &Vector3::from([0.0, 0.0, PI]));

	let mut index = 0_usize;
	while window.render_with_camera(&mut camera) {
		planet.set_local_translation(Translation3 { vector: points[index].coords });
		gui_lib::draw_orbit_points(&mut window, &points, &Point3::from([1.0, 1.0, 1.0]));
		gui_lib::draw_full_axes(&mut window, 100_f32, 1_f32);
		sphere.prepend_to_local_rotation(&rot);
		//planet.set_local_translation(Translation3::new(20.0 * angle.cos(), 20.0 * angle.sin(), 0.0));
		//angle += PI / 180.0;
		index = (index + 1) % points.len();
	}
}
