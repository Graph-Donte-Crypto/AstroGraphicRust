use astrorust_gui_lib::kiss3d::text::Font;
use astrorust_gui_lib::na::Point2;
use astrorust_lib::angle::{Angle, EccAnomaly, TrueAnomaly};
use astrorust_lib::orbit::flat::elliptic::EllipticOrbit;
use astrorust_lib::state_vectors::StateVectors;
use astrorust_lib::time::Time;
use core::f64::consts::PI;
use gui_lib::kiss3d::light::Light;
use gui_lib::kiss3d::nalgebra as na;
use gui_lib::kiss3d::window::Window;
use na::{Point3, Translation3, Vector3};
use std::f64::consts::FRAC_PI_2;
use std::time::Instant;
use {astrorust_gui_lib as gui_lib, astrorust_lib as lib};

#[allow(non_snake_case)]
fn main() {
    const SPHERE_RADIUS: f32 = 1.0;

    let mut window = Window::new("Astro Graphic Rust");

    // kiss3d supports only one point lightsource, so we place it in the center of the star.
    // Negative radius turns the sphere inside out, so the light can pass outside
    let mut star = window.add_sphere(-SPHERE_RADIUS);
    let mut planet = window.add_sphere(0.5);
    planet.set_color(0.38, 0.52, 0.76);
    star.set_color(1.0, 1.0, 0.0);
    window.set_light(Light::Absolute(Point3::origin()));

    let elliptic_orbit: EllipticOrbit = lib::orbit::flat::Orbit2DBuilder::default()
        .std_grav_param(1172.3328)
        .semi_major_axis(10.0)
        .eccentricity(0.7)
        .build()
        .unwrap()
        .into();
    let orbit = lib::orbit::orbit_3d::Orbit3DBuilder::default()
        .orbit_2d(elliptic_orbit.clone())
        .inclination(17_f64.to_radians())
        .long_of_asc_node(45_f64.to_radians())
        .arg_of_periapsis(70_f64.to_radians())
        .build()
        .unwrap();
    dbg!(&orbit);

    dbg!(elliptic_orbit.period().as_secs() * 60.0);

    let points = gui_lib::generate_orbit_points(&orbit, 100);
    window.set_line_width(3.0);
    window.set_framerate_limit(Some(60));

    let mut camera = gui_lib::kiss3d_trackball::Trackball::new(
        &Point3::from([10.0, 10.0, 10.0]),
        &Point3::origin(),
        &Vector3::from([0.0, 0.0, std::f32::consts::PI]),
    );

    let pe = Point3::from(orbit.position(EccAnomaly::from(Angle::from_rad(0.0)))).map(|x| x as f32);
    let ap = Point3::from(orbit.position(EccAnomaly::from(Angle::from_rad(PI)))).map(|x| x as f32);

    let started_at = Instant::now();
    while window.render_with_camera(&mut camera) {
        gui_lib::draw_full_axes(&mut window, 100.0, SPHERE_RADIUS);
        gui_lib::draw_orbit_points(&mut window, &points, &Point3::from([1.0, 1.0, 1.0]));

        let t = started_at.elapsed();
        let (r, v) = orbit.position_and_velocity(Time::from(t));
        window.draw_line(&pe, &ap, &Point3::new(0.7, 1.0, 0.7));
        window.draw_line(
            &Point3::origin(),
            &r.map(|x| x as f32).into(),
            &Point3::new(0.6, 0.6, 1.0),
        );
        window.draw_text(
            &format!("Distance: {:.1} km\nSpeed:     {:.1} km/s", r.magnitude(), v.magnitude()),
            &Point2::new(0.0, 0.0),
            60.0,
            &Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );
        planet.set_local_translation(Translation3 {
            vector: Vector3::new(r[0] as f32, r[1] as f32, r[2] as f32),
        });
    }
}
