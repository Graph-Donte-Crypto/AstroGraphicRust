use astrorust_gui_lib::kiss3d::scene::SceneNode;
use astrorust_gui_lib::kiss3d::text::Font;
use astrorust_gui_lib::na::{Point, Point2};
use astrorust_lib::angle::{Angle, EccAnomaly};
use astrorust_lib::orbit::flat::elliptic::EllipticOrbit;
use astrorust_lib::orbit::orbit_3d::Orbit3D;
use astrorust_lib::state_vectors::StateVectors;
use astrorust_lib::time::Time;
use core::f64::consts::PI;
use gui_lib::kiss3d::light::Light;
use gui_lib::kiss3d::nalgebra as na;
use gui_lib::kiss3d::window::Window;
use na::{Point3, Translation3, Vector3};
use std::time::Instant;
use {astrorust_gui_lib as gui_lib, astrorust_lib as lib};

#[allow(non_snake_case)]
fn main() {
    const STAR_RADIUS: f32 = 3.0;

    let mut window = Window::new("Astro Graphic Rust");

    // kiss3d supports only one point lightsource, so we place it in the center of the star.
    // Negative radius turns the sphere inside out, so the light can pass outside
    let mut star = window.add_sphere(-STAR_RADIUS);
    let mut planet1 = window.add_sphere(2.0);
    let mut planet2 = window.add_sphere(2.0);
    let mut planet3 = window.add_sphere(1.0);
    planet1.set_color(0.38, 0.52, 0.76);
    planet2.set_color(0.38, 0.52, 0.76);
    planet3.set_color(1.0, 0.0, 0.0);
    star.set_color(1.0, 1.0, 0.0);
    window.set_light(Light::Absolute(Point3::origin()));

    let (orbit1, points1, pe1, ap1) = elliptic_orbit(10.0, 0.0);
    let (orbit2, points2, pe2, ap2) = elliptic_orbit(40.0, 0.0);
    let (orbit3, points3, pe3, ap3) = elliptic_orbit(25.0, 0.6);

    window.set_line_width(3.0);
    window.set_framerate_limit(Some(60));

    let mut camera = gui_lib::kiss3d_trackball::Trackball::new(
        &Point3::from([10.0, 10.0, 10.0]),
        &Point3::origin(),
        &Vector3::from([0.0, 0.0, std::f32::consts::PI]),
    );

    let started_at = Instant::now();
    while window.render_with_camera(&mut camera) {
        gui_lib::draw_full_axes(&mut window, 100.0, STAR_RADIUS);
        let t: Time = started_at.elapsed().into();

        draw_orbit_and_current_position(
            &mut window,
            &mut planet1,
            &orbit1,
            &points1,
            pe1,
            ap1,
            t,
            false,
        );

        draw_orbit_and_current_position(
            &mut window,
            &mut planet2,
            &orbit2,
            &points2,
            pe2,
            ap2,
            t,
            false,
        );

        draw_orbit_and_current_position(
            &mut window,
            &mut planet3,
            &orbit3,
            &points3,
            pe3,
            ap3,
            t,
            true,
        );
    }
}

fn draw_orbit_and_current_position(
    window: &mut Window,
    planet: &mut SceneNode,
    orbit: &Orbit3D<EllipticOrbit>,
    points: &Vec<Point<f32, 3>>,
    pe: Point<f32, 3>,
    ap: Point<f32, 3>,
    t: Time,
    draw_text: bool,
) {
    gui_lib::draw_orbit_points(window, points, &Point3::from([1.0, 1.0, 1.0]));

    let (r, v) = orbit.position_and_velocity(t);
    window.draw_line(&pe, &ap, &Point3::new(0.7, 1.0, 0.7));
    // window.draw_line(&Point3::origin(), &r.map(|x| x as f32).into(), &Point3::new(0.6, 0.6, 1.0));
    planet.set_local_translation(Translation3 {
        vector: Vector3::new(r[0] as f32, r[1] as f32, r[2] as f32),
    });

    if draw_text {
        window.draw_text(
            &format!("Distance: {:.1} km\nSpeed:     {:.1} km/s", r.magnitude(), v.magnitude()),
            &Point2::new(0.0, 0.0),
            60.0,
            &Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );
    }
}

fn elliptic_orbit(
    a: f64,
    e: f64,
) -> (Orbit3D<EllipticOrbit>, Vec<Point<f32, 3>>, Point<f32, 3>, Point<f32, 3>) {
    let orbit: EllipticOrbit = lib::orbit::flat::Orbit2DBuilder::default()
        .std_grav_param(11723.328)
        .semi_major_axis(a)
        .eccentricity(e)
        .build()
        .unwrap()
        .into();
    let orbit = lib::orbit::orbit_3d::Orbit3DBuilder::default().orbit_2d(orbit).build().unwrap();
    let points = gui_lib::generate_orbit_points(&orbit, 100);
    let pe = Point3::from(orbit.position(EccAnomaly::from(Angle::from_rad(0.0)))).map(|x| x as f32);
    let ap = Point3::from(orbit.position(EccAnomaly::from(Angle::from_rad(PI)))).map(|x| x as f32);

    (orbit, points, pe, ap)
}
