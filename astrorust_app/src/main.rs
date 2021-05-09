use astrorust_gui_lib as gui_lib;
use astrorust_lib as lib;
use cell::RefCell;
use gui_lib::{SpaceBody, kiss3d::{self, camera::Camera, event::{Action, MouseButton, WindowEvent}, light::Light, nalgebra as na, window::Window}};
use na::{Point3, UnitQuaternion, Translation3, Vector3, Point2, Vector2};
use std::{f32::consts::PI, cell};

fn draw_debug_lines(window: &mut Window, point: &Point3<f32>) {
	window.draw_line(
		&point, 
		&Point3::<f32>::from([100.0, 0.0, 0.0]),
		&Point3::from([1.0, 1.0, 1.0])
	);
	window.draw_line(
		&point, 
		&Point3::<f32>::from([0.0, 100.0, 0.0]),
		&Point3::from([1.0, 1.0, 1.0])
	);
	window.draw_line(
		&point, 
		&Point3::<f32>::from([0.0, 0.0, 100.0]),
		&Point3::from([1.0, 1.0, 1.0])
	);
}

#[allow(non_snake_case)]
fn main() {
	let mut window = Window::new("Astro Graphic Rust");
	let kerbol = RefCell::new(gui_lib::SpaceBody::new(&mut window, -1.0));
	let planet = RefCell::new(gui_lib::SpaceBody::new(&mut window, 0.2));

	planet.borrow_mut().body.set_color(0.6, 0.6, 0.6);
	kerbol.borrow_mut().body.set_color(1.0, 1.0, 0.0);

	let space_body_vec = vec![
		&kerbol, 
		&planet
	];

	//sphere.set
	//window.set_light(Light::StickToCamera);
	window.set_light(Light::Absolute(Point3::from([0.0, 0.0, 0.0])));
	
	let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);
	let orbit = lib::orbit::Orbit::new(1172.3328, 5.263138, 0.2, 7_f32.to_radians(), 15_f32.to_radians(), 70_f32.to_radians(), 0_f32.to_radians());
	let points = gui_lib::generate_orbit_points(&orbit, 500);
	window.set_line_width(1.0);

	let mut camera = //kiss3d::camera::ArcBall::new(Point3::from([0.0, 0.0, 0.0]), Point3::from([5.0, 0.0, 5.0]));
		gui_lib::kiss3d_trackball::Trackball::new(Point3::from([0.0, 0.0, 0.0]), &Point3::from([5.0, 0.0, 5.0]), &Vector3::from([0.0, 1.0, 0.0]));

	//camera.input.rebind_slide_button(Some(MouseButton::Button3));
	//camera.input.rebind_orbit_button(Some(MouseButton::Button2));

	let mut current_body = space_body_vec[0];

	let mut index = 0_usize;
	loop {
		/* camera.frame.set_eye(
			&(camera.frame.eye() + current_body.borrow().pos()), 
			&camera.frame.yaw_axis()
		); */

		

		if !window.render_with_camera(&mut camera) {break}

		

		/* camera.frame.set_eye(
			&(camera.frame.eye() - current_body.borrow().pos()), 
			&camera.frame.yaw_axis()
		); */
		/* 
		let c_eye = &camera.eye();
		let c_tar = &camera.at();
*/
		let c_eye = &camera.frame.eye();
		let c_tar = camera.frame.target();
		 
		//let upvec = camera.frame.yaw_axis();
		
		let mouse_line = window.cursor_pos().map(|mouse| camera.unproject(
			&Point2::from([mouse.0 as f32, mouse.1 as f32]), 
			&Vector2::from([window.width() as f32, window.height() as f32])
		));

		let mouse_line = match window.cursor_pos() {
			Some(mouse) =>
				Some(camera.unproject(
					&Point2::from([mouse.0 as f32, mouse.1 as f32]), 
					&Vector2::from([window.width() as f32, window.height() as f32])
				)),
			None => None
		};

		if let Some(mouse) = mouse_line {

			//draw_debug_lines(&mut window, c_eye);
			draw_debug_lines(&mut window, &(&Point3::<f32>::origin() + &(&c_tar.coords - &c_eye.coords)));
			draw_debug_lines(&mut window, &mouse.0);
			draw_debug_lines(&mut window, c_tar);

			//dbg!(c_eye);
			//dbg!(&mouse.0);
			dbg!(&camera.project(c_eye, &Vector2::from([window.width() as f32, window.height() as f32])));

			let line_vec = &mouse.0.coords - &c_eye.coords;

			window.draw_line(
				&(&Point3::<f32>::origin() + &(&c_tar.coords - &c_eye.coords)), 
				&Point3::<f32>::origin(),
				//&(&(mouse_point + vector) - &Vector3::<f32>::from([0.0, 10.0, 0.0])),
				&Point3::from([0.0, 1.0, 1.0])
			);

			window.draw_line(
				c_tar, 
				&Point3::<f32>::origin(),
				//&(&(mouse_point + vector) - &Vector3::<f32>::from([0.0, 10.0, 0.0])),
				&Point3::from([1.0, 1.0, 0.0])
			);

			/* window.draw_line(
				&(&mouse.0 + &(c_tar.coords - c_eye.coords)), 
				&Point3::<f32>::origin(),
				//&(&(mouse_point + vector) - &Vector3::<f32>::from([0.0, 10.0, 0.0])),
				&Point3::from([1.0, 1.0, 1.0])
			); */

			//window.draw_line(&Point3::<f32>::origin(), &(Point3::<f32>::from(SpaceBody::get_real_eye(&c_eye, &c_tar, mouse, (window.width() as f64 / 2.0, window.height() as f64 / 2.0))) - Vector3::<f32>::from([0.0, 0.0, 50.0])), &Point3::from([1.0, 1.0, 1.0]));
			//window.draw_line(&c_tar, &(Point3::<f32>::from(SpaceBody::get_real_eye(&c_eye, &c_tar, mouse, (window.width() as f64 / 2.0, window.height() as f64 / 2.0))) - Vector3::<f32>::from([0.0, 0.0, 50.0])), &Point3::from([1.0, 1.0, 1.0]));
		}
		for i in window.events().iter() {
			if let WindowEvent::MouseButton(button, action, modifiers) = i.value {
				if let Some(mouse) = mouse_line {
					if MouseButton::Button1 == button && Action::Press == action {
						//println!("{} | {}", c_eye.to_string(), &Point3::<f32>::from(SpaceBody::get_real_eye(&c_eye, &c_tar, mouse, (window.width() as f64 / 2.0, window.height() as f64 / 2.0))).to_string());
						for p in &space_body_vec {
							if p.borrow().ui_mouse_check(mouse.0, mouse.1) {
								println!("YES");
								current_body = p;
								break;
							}
						}
					}
				}
			}
		}

		

		//planet.data().local_translation()
		planet.borrow_mut().body.set_local_translation(Translation3 { vector: points[index].coords });
		gui_lib::draw_orbit_points(&mut window, &points, &Point3::from([1.0, 1.0, 1.0]));
		gui_lib::draw_full_axes(&mut window, 100_f32, 1_f32);
		kerbol.borrow_mut().body.prepend_to_local_rotation(&rot);
		//planet.set_local_translation(Translation3::new(20.0 * angle.cos(), 20.0 * angle.sin(), 0.0));
		//angle += PI / 180.0;
		index = (index + 1) % points.len();
	}
}
