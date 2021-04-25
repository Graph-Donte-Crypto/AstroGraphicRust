use astrorust_gui_lib as gui_lib;
use astrorust_lib as lib;
use gui_lib::kiss3d::{
	self,
	event::{Key, MouseButton},
	light::Light,
	window::Window,
};
use gui_lib::kiss3d::nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};
use gui_lib::Drawable;

#[allow(non_snake_case)]
fn main() {
	let mut window = Window::new("Astro Graphic Rust");
	let mut sphere = window.add_sphere(1.0);
	sphere.set_color(1.0, 1.0, 0.0);
	window.set_light(Light::StickToCamera);
	let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);
	let orbit = lib::orbit::Orbit::new(1172.3328, 5.263138, 0.2, 7.0_f32.to_radians(), 15.0_f32.to_radians(), 70.0_f32.to_radians(), 0.0_f32.to_radians());
	window.set_line_width(1.0);
	let rho = 0;
	let box_size: f32 = 500.0;
	let axis_0 = Point3::new(-box_size, -box_size, -box_size);
	let axis_x = Point3::new(box_size, -box_size, -box_size);
	let axis_y = Point3::new(-box_size, box_size, -box_size);
	let axis_z = Point3::new(-box_size, -box_size, box_size);

	let mut camera =
		kiss3d::camera::FirstPerson::new(Point3::new(0.0, 0.0, 0.0), Point3::new(-1.0, -1.0, -1.0));

	camera.rebind_up_key(Some(Key::W));
	camera.rebind_down_key(Some(Key::S));
	camera.rebind_left_key(Some(Key::A));
	camera.rebind_right_key(Some(Key::D));

	camera.rebind_rotate_button(Some(MouseButton::Button2));
	camera.rebind_drag_button(None);

	//window.render_with_camera()

	while window.render_with_camera(&mut camera) {
		//planet.set_local_translation(Translation3::<f32>::from(point_to_vector(&vec[index])));
		//vec[index]));
		gui_lib::draw_orbit(&mut window, &orbit, &Point3::<f32>::new(1.0, 1.0, 1.0));

		window.draw_line(&axis_0, &axis_x, &Point3::new(1.0, 0.0, 0.0));
		window.draw_line(&axis_0, &axis_y, &Point3::new(0.0, 1.0, 0.0));
		window.draw_line(&axis_0, &axis_z, &Point3::new(0.0, 0.0, 1.0));

		sphere.prepend_to_local_rotation(&rot);
		//planet.set_local_translation(Translation3::new(20.0 * angle.cos(), 20.0 * angle.sin(), 0.0));
		//angle += PI / 180.0;
	}
}
