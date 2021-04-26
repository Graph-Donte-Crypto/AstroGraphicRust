use astrorust_gui_lib as gui_lib;
use astrorust_lib as lib;
use gui_lib::kiss3d::{
	event::MouseButton,
	light::Light,
	nalgebra as na,
	window::Window,
};
use na::{Point3, UnitQuaternion, Translation3, Vector3};
use core::f32::consts::PI;

#[allow(non_snake_case)]
fn main() {
	let mut window = Window::new("Astro Graphic Rust");

	// kiss3d supports only single point lightsource, so we place it in the center of the star.
	// Negative radius turns the sphere inside out, so the light can pass outside
	let mut sphere = window.add_sphere(-1.0);
	let mut planet = window.add_sphere(0.2);
	planet.set_color(0.6, 0.6, 0.6);
	sphere.set_color(1.0, 1.0, 0.0);
	window.set_light(Light::Absolute(Point3::from([0.0, 0.0, 0.0])));

	let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);
	let orbit = lib::orbit::Orbit::builder()
		.mu(1172.3328)
		.a(5.263138)
		.e(0.2)
		.i(7_f32.to_radians())
		.Omega(15_f32.to_radians())
		.omega(70_f32.to_radians())
		.build();
	dbg!(&orbit);

	let points = gui_lib::generate_orbit_points(&orbit, 500);
	window.set_line_width(1.0);

	let mut camera = gui_lib::kiss3d_trackball::Trackball::new(
		&Point3::from([10.0, 10.0, 10.0]),
		&Point3::origin(),
		&Vector3::from([0.0, 0.0, PI]),
	);

	camera.input.rebind_slide_button(Some(MouseButton::Button3));
	camera.input.rebind_orbit_button(Some(MouseButton::Button2));

	let mut index = 0_usize;
	while window.render_with_camera(&mut camera) {
		planet.set_local_translation(Translation3 { vector: points[index].coords });
		gui_lib::draw_orbit_points(&mut window, &points, &Point3::from([1.0, 1.0, 1.0]));
		gui_lib::draw_full_axes(&mut window, 100_f32, 1_f32);
		sphere.prepend_to_local_rotation(&rot);
		index = (index + 1) % points.len();
	}
}
