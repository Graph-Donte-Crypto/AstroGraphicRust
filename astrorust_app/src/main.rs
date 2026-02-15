use astrorust_gui_lib as gui_lib;
use astrorust_gui_lib::kiss3d::camera::Camera;
use astrorust_gui_lib::kiss3d::scene::SceneNode;
use astrorust_gui_lib::kiss3d::text::Font;
use astrorust_gui_lib::na::Point2;
use astrorust_lib::angle::{Angle, EccAnomaly};
use astrorust_lib::config::StarSystem;
use astrorust_lib::orbit::flat::elliptic::EllipticOrbit;
use astrorust_lib::orbit::orbit_3d::Orbit3D;
use astrorust_lib::state_vectors::StateVectors;
use astrorust_lib::time::Time;
use chrono::NaiveDate;
use core::f64::consts::PI;
use gui_lib::kiss3d::light::Light;
use gui_lib::kiss3d::nalgebra as na;
use gui_lib::kiss3d::window::Window;
use na::{Point3, Translation3, Vector3};
use std::time::{Duration, Instant};

const CAMERA_ACCELERATION: f64 = 0.001;
const TIME_WARP: f64 = 1000_000.0;
// const TIME_WARP: f64 = 1.0;
const STAR_RADIUS: f32 = 15.0;
const PLANET_RADIUS: f32 = 7.0;
const PLANET_COLORS: &[Point3<f32>] = &[
    Point3::new(0.65, 0.65, 0.65), // Mercury (grayish)
    Point3::new(0.95, 0.85, 0.55), // Venus (yellowish-white)
    Point3::new(0.0, 0.3, 0.8),    // Earth (blue with green and brown)
    Point3::new(0.8, 0.4, 0.1),    // Mars (reddish-brown)
    Point3::new(0.9, 0.75, 0.5),   // Jupiter (orange with white and brown)
    Point3::new(0.85, 0.75, 0.4),  // Saturn (pale gold)
    Point3::new(0.5, 0.8, 0.9),    // Uranus (light blue-green)
    Point3::new(0.1, 0.3, 0.8),    // Neptune (deep blue)
    Point3::new(0.95, 0.9, 0.85),  // Pluto (light brown/off-white)
];

fn main() {
    let mut window = Window::new("Astro Graphic Rust");

    // kiss3d supports only one point lightsource, so we place it in the center of the star.
    // Negative radius turns the sphere inside out, so the light can pass outside
    let mut star = window.add_sphere(-STAR_RADIUS);
    star.set_color(1.0, 1.0, 0.0);
    window.set_light(Light::Absolute(Point3::origin()));

    let config = StarSystem::load_from_yaml("config/star_system/solar.yml").unwrap();

    let scale = 100.0 / config.planets.first().unwrap().orbit.a;

    let mut planets: Vec<Planet> = config
        .clone()
        .into_planets()
        .into_iter()
        .zip(PLANET_COLORS)
        .map(|(orbit, color)| create_planet(&mut window, scale, orbit, color.clone()))
        .collect();

    window.set_line_width(3.0);
    window.set_framerate_limit(Some(60));

    let mut camera = gui_lib::kiss3d_trackball::Trackball::new(
        &Point3::from([0.0, -500.0, 300.0]),
        &Point3::origin(),
        &Vector3::from([0.0, 0.0, std::f32::consts::PI]),
    );

    let epoch = NaiveDate::from_ymd_opt(2000, 1, 1).unwrap().and_time(Default::default()).and_utc();
    let started_at_date = chrono::Utc::now();
    let started_at = Instant::now();
    while window.render_with_camera(&mut camera) {
        // gui_lib::draw_full_axes(&mut window, 100.0, STAR_RADIUS);
        let eye = camera.eye();
        let real_t = started_at.elapsed().as_secs_f64();
        let t = Time::from_secs(real_t * TIME_WARP + (started_at_date - epoch).as_seconds_f64());
        for planet in &mut planets {
            draw_orbit_and_current_position(&mut window, &eye, scale, planet, t);
        }

        window.draw_text(
            &format!("Time: {}", (epoch + Duration::from(t)).format("%Y-%m-%d %H:%M")),
            &Point2::new(0.0, 0.0),
            60.0,
            &Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );
        camera.frame.set_eye(
            &(&eye + (0.5 * CAMERA_ACCELERATION * real_t * real_t) as f32 * eye.coords.normalize()),
            &Vector3::new(0.0, 0.0, 1.0),
        );
    }
}

fn draw_orbit_and_current_position(
    window: &mut Window,
    eye: &Point3<f32>,
    scale: f64,
    planet: &mut Planet,
    t: Time,
) {
    gui_lib::draw_orbit_points(window, &planet.points, &planet.color);

    let (r, v) = planet.orbit.position_and_velocity(t);
    // window.draw_line(&planet.pe, &planet.ap, &Point3::new(0.7, 1.0, 0.7));
    // window.draw_line(&Point3::origin(), &r.map(|x| x as f32).into(), &Point3::new(0.6, 0.6, 1.0));

    planet.sphere.set_local_translation(Translation3 { vector: r.map(|x| (scale * x) as f32) });
    let clamp = (20.0, (50.0 * planet.orbit.orbit_2d.0.a().max(149.6 * 1e6) / 90118820.0) as f32);
    let planet_scale = (eye.coords.magnitude() * 0.020).clamp(clamp.0, clamp.1);
    planet.sphere.set_local_scale(planet_scale, planet_scale, planet_scale);

    // if draw_text {
    //     window.draw_text(
    //         &format!("Distance: {:.1} km\nSpeed:     {:.1} km/s", r.magnitude(), v.magnitude()),
    //         &Point2::new(0.0, 0.0),
    //         60.0,
    //         &Font::default(),
    //         &Point3::new(1.0, 1.0, 1.0),
    //     );
    // }
}

fn create_planet(
    window: &mut Window,
    scale: f64,
    orbit: Orbit3D<EllipticOrbit>,
    color: Point3<f32>,
) -> Planet {
    let points = gui_lib::generate_orbit_points(&orbit, 100)
        .iter()
        .map(|point| point.map(|x| (scale * x) as f32))
        .collect();
    let pe = Point3::from(orbit.position(EccAnomaly::from(Angle::from_rad(0.0))))
        .map(|x| (scale * x) as f32);
    let ap = Point3::from(orbit.position(EccAnomaly::from(Angle::from_rad(PI))))
        .map(|x| (scale * x) as f32);

    let mut sphere = window.add_sphere(PLANET_RADIUS as f32);
    sphere.set_color(color.x, color.y, color.z);

    Planet { sphere, orbit, points, pe, ap, color }
}

struct Planet {
    sphere: SceneNode,
    orbit: Orbit3D<EllipticOrbit>,
    points: Vec<Point3<f32>>,
    pe: Point3<f32>,
    ap: Point3<f32>,
    color: Point3<f32>,
}
