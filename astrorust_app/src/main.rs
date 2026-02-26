use astrorust_gui_lib as gui_lib;
use astrorust_gui_lib::kiss3d::camera::Camera;
use astrorust_gui_lib::kiss3d::nalgebra::Point2;
use astrorust_gui_lib::kiss3d::scene::SceneNode;
use astrorust_gui_lib::kiss3d::text::Font;
use astrorust_lib::config::{CelestialBody, Config, StarSystem};
use astrorust_lib::orbit::flat::elliptic::EllipticOrbit;
use astrorust_lib::orbit::orbit_3d::Orbit3D;
use astrorust_lib::state_vectors::StateVectors;
use astrorust_lib::time::Time;
use astrorust_lib::trajectory::Trajectory;
use astrorust_lib::util::format_with_thousand_separators;
use chrono::{DateTime, Duration, NaiveDate, NaiveTime, Utc};
use gui_lib::kiss3d::light::Light;
use gui_lib::kiss3d::nalgebra as na;
use gui_lib::kiss3d::window::Window;
use na::{Point3, Translation3, Vector3};
use std::time::Instant;

const CAMERA_ACCELERATION: f64 = 0.0;
const TIME_WARP: u64 = 5_000_000;
const STAR_RADIUS: f32 = 15.0;
const PLANET_RADIUS: f32 = 7.0;

fn main() {
    let mut window = Window::new("Astro Graphic Rust");

    // kiss3d supports only one point lightsource, so we place it in the center of the star.
    // Negative radius turns the sphere inside out, so the light can pass outside
    let mut star = window.add_sphere(-STAR_RADIUS);
    star.set_color(1.0, 1.0, 0.0);
    window.set_light(Light::Absolute(Point3::origin()));

    let config = Config::load_from_yaml("config/config.yml").unwrap();
    let system = config.system.to_lowercase();
    let system = StarSystem::load_from_yaml(&format!("config/system/{system}.yml")).unwrap();

    let scale = 100.0 / system.planets.first().unwrap().orbit.a;

    let mut planets: Vec<Body> = system
        .clone()
        .into_planets()
        .into_iter()
        .map(|(body, orbit)| create_body(&mut window, scale, body, orbit))
        .collect();
    let (_, spacecraft_μ) = std::iter::once((system.star.name.as_str(), system.star.μ))
        .chain(system.planets.iter().map(|planet| (planet.body.name.as_str(), planet.body.μ)))
        .chain(system.planets.iter().flat_map(|planet| {
            planet.moons.iter().map(|moon| (moon.body.name.as_str(), moon.body.μ))
        }))
        .find(|(name, _)| *name == config.spacecraft.body)
        .expect(&format!("Body `{}` not found", config.spacecraft.body));
    let mut spacecraft_orbit = config.spacecraft.orbit.clone();
    spacecraft_orbit.mu = spacecraft_μ;
    let spacecraft: Trajectory = spacecraft_orbit.into();
    let mut spacecraft =
        create_spacecraft(&mut window, scale, spacecraft, Point3::new(1.0, 1.0, 1.0));

    window.set_line_width(3.0);
    window.set_framerate_limit(Some(60));

    let mut camera = gui_lib::kiss3d_trackball::Trackball::new(
        &Point3::from([0.0, -500.0, 300.0]),
        &Point3::origin(),
        &Vector3::from([0.0, 0.0, std::f32::consts::PI]),
    );

    let epoch = NaiveDate::from_ymd_opt(2000, 1, 1)
        .unwrap()
        .and_time(NaiveTime::from_hms_opt(12, 0, 0).unwrap())
        .and_utc();
    let started_at_date =
        chrono::DateTime::parse_from_rfc3339("1977-08-23T11:29:11Z").unwrap().to_utc();
    let started_at = Instant::now();
    while window.render_with_camera(&mut camera) {
        // gui_lib::draw_full_axes(&mut window, 100.0, STAR_RADIUS);
        let eye = camera.eye();
        let real_t = started_at.elapsed().as_secs_f64();
        let t =
            Time::from_secs(real_t * TIME_WARP as f64 + (started_at_date - epoch).as_seconds_f64());
        for planet in &mut planets {
            draw_orbit_and_current_position(&mut window, &eye, scale, planet, t);
        }
        let r = spacecraft.orbit.position(t);
        if let Some(planet) = planets.iter_mut().find(|planet| {
            (planet.orbit.position(t) - r).magnitude()
                < planet.orbit.orbit_2d.0.a()
                    * (planet.body.μ / planet.orbit.orbit_2d.0.mu()).powf(0.4)
        }) {
            let (orbit, body) = (&planet.orbit, &planet.body);
            // Compute gravity assist
            eprintln!("Spacecraft encounter with planet {}", body.name);
            let (planet_r, planet_v) = orbit.position_and_velocity(t);

            // Convert heliocentric state vectors to planetocentric
            let mut r = r - planet_r;
            let mut v = spacecraft.orbit.velocity(t) - planet_v;

            // Unit vector from planet to periapsis of flyby hyperbola
            let unit_e = (v.cross(&r.cross(&v)) / body.μ - r.normalize()).normalize();

            dbg!("Flyby start: ", &r, &v);

            // Reflect state vectors to get state vectors after the flyby
            r = -r + 2.0 * r.dot(&unit_e) * unit_e;
            let delta_v = 2.0 * v.dot(&unit_e) * unit_e;
            v += delta_v;

            dbg!("Flyby end: ", &r, &v, &delta_v, delta_v.magnitude());

            r = (orbit.orbit_2d.0.a() * (body.μ / orbit.orbit_2d.0.mu()).powf(0.4)) * r.normalize();

            // Convert planetocentric state vectors back to heliocentric
            r += planet_r;
            v += planet_v;
            dbg!(r.magnitude(), v.magnitude());

            // Calculate new orbit from new state vectors
            let new_orbit =
                Trajectory::from_state_vectors(system.star.μ, r, v, t.as_secs());
            dbg!(&spacecraft.orbit, &new_orbit);
            spacecraft.orbit = new_orbit.into();
            // let soi_radius = orbit.orbit_2d.0.a() * (body.μ / orbit.orbit_2d.0.mu()).powf(0.4);
            spacecraft.points = gui_lib::generate_trajectory_points(1e11, &spacecraft.orbit, 360)
                .iter()
                .map(|point| point.map(|x| (scale * x) as f32))
                .collect();
        }
        draw_orbit_and_current_position_of_spacecraft(
            &mut window,
            &eye,
            scale,
            &mut spacecraft,
            t,
            epoch,
        );

        if CAMERA_ACCELERATION > f64::EPSILON {
            camera.frame.set_eye(
                &(&eye
                    + (0.5 * CAMERA_ACCELERATION * real_t * real_t) as f32
                        * eye.coords.normalize()),
                &Vector3::new(0.0, 0.0, 1.0),
            );
        }
    }
}

fn draw_orbit_and_current_position(
    window: &mut Window,
    eye: &Point3<f32>,
    scale: f64,
    body: &mut Body,
    t: Time,
) {
    gui_lib::draw_orbit_points(window, &body.points, &body.color, false);
    let r = body.orbit.position(t);

    body.sphere.set_local_translation(Translation3 { vector: r.map(|x| (scale * x) as f32) });
    let clamp = (20.0, (50.0 * body.orbit.orbit_2d.0.a().max(149.6 * 1e6) / 90118820.0) as f32);
    let planet_scale = (eye.coords.magnitude() * 0.020).clamp(clamp.0, clamp.1);
    body.sphere.set_local_scale(planet_scale, planet_scale, planet_scale);
}

fn draw_orbit_and_current_position_of_spacecraft(
    window: &mut Window,
    eye: &Point3<f32>,
    scale: f64,
    spacecraft: &mut Spacecraft,
    t: Time,
    epoch: DateTime<Utc>,
) {
    let is_hyperbolic = match spacecraft.orbit {
        Trajectory::Elliptic(_) => false,
        Trajectory::Hyperbolic(_) => true,
    };
    gui_lib::draw_orbit_points(window, &spacecraft.points, &spacecraft.color, is_hyperbolic);
    let (r, v) = spacecraft.orbit.position_and_velocity(t);
    // window.draw_line(&planet.pe, &planet.ap, &Point3::new(0.7, 1.0, 0.7));
    // window.draw_line(&Point3::origin(), &r.map(|x| x as f32).into(), &Point3::new(0.6, 0.6, 1.0));

    spacecraft.sphere.set_local_translation(Translation3 { vector: r.map(|x| (scale * x) as f32) });
    let clamp = (20.0, 50.0);
    let planet_scale = (eye.coords.magnitude() * 0.020).clamp(clamp.0, clamp.1);
    spacecraft.sphere.set_local_scale(planet_scale, planet_scale, planet_scale);

    window.draw_text(
        &format!(
            "Warp: {}x\nTime: {}\nDistance: {:.1} au\nSpeed:     {:.1} km/s",
            format_with_thousand_separators(TIME_WARP),
            (epoch + Duration::from(t)).format("%Y-%m-%d %H:%M"),
            r.magnitude() / 149597870.700,
            v.magnitude()
        ),
        &Point2::origin(),
        60.0,
        &Font::default(),
        &Point3::new(1.0, 1.0, 1.0),
    );
}

fn create_body(
    window: &mut Window,
    scale: f64,
    body: CelestialBody,
    orbit: Orbit3D<EllipticOrbit>,
) -> Body {
    let points = gui_lib::generate_orbit_points(&orbit, 100)
        .iter()
        .map(|point| point.map(|x| (scale * x) as f32))
        .collect();
    let color = rgb8_to_color(body.color);

    let mut sphere = window.add_sphere(PLANET_RADIUS as f32);
    sphere.set_color(color.x, color.y, color.z);

    Body { sphere, body, orbit, points, color }
}

fn create_spacecraft(
    window: &mut Window,
    scale: f64,
    orbit: Trajectory,
    color: Point3<f32>,
) -> Spacecraft {
    let points = gui_lib::generate_trajectory_points(1e10, &orbit, 100)
        .iter()
        .map(|point| point.map(|x| (scale * x) as f32))
        .collect();

    let mut sphere = window.add_sphere(PLANET_RADIUS as f32);
    sphere.set_color(color.x, color.y, color.z);

    Spacecraft { sphere, orbit, points, color }
}

fn rgb8_to_color([r, g, b]: [u8; 3]) -> Point3<f32> {
    Point3::<f32>::new(r.into(), g.into(), b.into()) / 255.0
}

struct Body {
    sphere: SceneNode,
    body: CelestialBody,
    orbit: Orbit3D<EllipticOrbit>,
    points: Vec<Point3<f32>>,
    color: Point3<f32>,
}

struct Spacecraft {
    sphere: SceneNode,
    orbit: Trajectory,
    points: Vec<Point3<f32>>,
    color: Point3<f32>,
}
