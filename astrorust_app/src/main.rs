use astrorust_gui_lib as gui_lib;
use astrorust_gui_lib::kiss3d::camera::Camera;
use astrorust_gui_lib::kiss3d::nalgebra::Point2;
use astrorust_gui_lib::kiss3d::scene::SceneNode;
use astrorust_gui_lib::kiss3d::text::Font;
use astrorust_gui_lib::na::UnitQuaternion;
use astrorust_lib::AU_IN_KM;
use astrorust_lib::angle::{Angle, EccAnomaly, HypAnomaly};
use astrorust_lib::config::{CelestialBody, Config, StarSystem};
use astrorust_lib::encounter::{Encounter, find_encounter};
use astrorust_lib::orbit::flat::elliptic::EllipticOrbit;
use astrorust_lib::orbit::orbit_3d::Orbit3D;
use astrorust_lib::state_vectors::StateVectors;
use astrorust_lib::time::Time;
use astrorust_lib::trajectory::Trajectory;
use astrorust_lib::util::*;
use chrono::{DateTime, Duration, Utc};
use gui_lib::kiss3d::event::{Action, Key, WindowEvent};
use gui_lib::kiss3d::light::Light;
use gui_lib::kiss3d::nalgebra as na;
use gui_lib::kiss3d::window::Window;
use na::{Point3, Translation3, Vector3};
use std::f64::consts::TAU;
use std::path::Path;
use std::rc::Rc;
use std::time::Instant;

const CAMERA_ACCELERATION: f64 = 0.0;
const TIME_WARP_STEPS: [i64; 21] = [
    -1_000_000_000,
    -100_000_000,
    -10_000_000,
    -1_000_000,
    -100_000,
    -10_000,
    -1_000,
    -100,
    -10,
    -1,
    0,
    1,
    10,
    100,
    1_000,
    10_000,
    100_000,
    1_000_000,
    10_000_000,
    100_000_000,
    1_000_000_000,
];
const DEFAULT_TIME_WARP_INDEX: usize = 11;
const STAR_RADIUS: f32 = 15.0;
const PLANET_RADIUS: f32 = 7.0;

/// Find the planet whose next SOI encounter is soonest after the current sim
/// time. Returns `(planet_index, Encounter)` so the SOI-entry transition can
/// be driven off the pre-computed `t_enc`, `E1`, `E2` rather than per-frame
/// distance sampling (which misses encounters at high time-warp).
fn first_future_encounter(
    spacecraft: &Spacecraft,
    planets: &[Body],
    t_now: DateTime<Utc>,
) -> Option<(usize, Encounter)> {
    planets
        .iter()
        .enumerate()
        .filter_map(|(i, planet)| {
            let enc =
                find_encounter(&spacecraft.trajectory, &planet.orbit, planet.soi_radius, t_now)?;
            (enc.t_enc >= t_now).then_some((i, enc))
        })
        .min_by_key(|(_, enc)| enc.t_enc)
        .inspect(|(i, enc)| {
            eprintln!("Next encounter: {} at {}", planets[*i].body.name, enc.t_enc);
        })
}

fn main() {
    // TODO: remove after migration to newer winit without wayland bug
    unsafe {
        std::env::set_var("WINIT_UNIX_BACKEND", "x11");
    }
    let mut window = Window::new("Astro Graphic Rust");

    let mut config = Config::load_from_yaml("config/config.yml").unwrap();
    let system = config.system.to_lowercase();
    let system = StarSystem::load_from_yaml(&format!("config/system/{system}.yml")).unwrap();

    // kiss3d supports only one point lightsource, so we place it in the center of the star.
    // Negative radius turns the sphere inside out, so the light can pass outside
    let mut star = window.add_sphere(-STAR_RADIUS);
    let star_color = rgb8_to_color(system.star.color);
    star.set_color(star_color.x, star_color.y, star_color.z);
    window.set_light(Light::Absolute(Point3::origin()));

    let scale = 100.0 / system.planets.first().unwrap().orbit.a;

    let mut planets: Vec<Body> = system
        .clone()
        .into_planets()
        .into_iter()
        .map(|(body, orbit)| create_body(&mut window, scale, body, orbit))
        .collect();

    window.set_line_width(3.0);
    window.set_framerate_limit(Some(60));

    let mut camera = gui_lib::kiss3d_trackball::Trackball::new(
        &Point3::from([0.0, -500.0, 300.0]),
        &Point3::origin(),
        &Vector3::from([0.0, 0.0, std::f32::consts::PI]),
    );
    let hud_font = load_ttf_font_from_current_dir();

    let planets_epoch = system.t0;
    let started_at_date =
        chrono::DateTime::parse_from_rfc3339("1979-04-15T00:00:00Z").unwrap().to_utc();
    // chrono::DateTime::parse_from_rfc3339("1993-08-23T11:29:11Z").unwrap().to_utc();
    let started_at = Instant::now();
    let mut previous_frame = Instant::now();
    let mut simulated_seconds = (started_at_date - planets_epoch).as_seconds_f64();
    let mut t = Time::from_secs(simulated_seconds);
    let mut time_warp_index = DEFAULT_TIME_WARP_INDEX;

    config.spacecraft.orbit.M0 = (config.spacecraft.orbit.M0
        + ((system.t0 - config.spacecraft.t0).as_seconds_f64()
            * (config.spacecraft.orbit.mu / config.spacecraft.orbit.a.powi(3)).sqrt())
            % TAU)
        % TAU;
    let spacecraft: Trajectory = config.spacecraft.orbit.clone().into();
    let mut spacecraft = create_spacecraft(
        &mut window,
        scale,
        spacecraft,
        rgb8_to_color(config.spacecraft.color),
        None,
    );
    spacecraft.next_encounter =
        first_future_encounter(&spacecraft, &planets, planets_epoch + Duration::from(t));
    dbg!(&spacecraft.next_encounter);

    while window.render_with_camera(&mut camera) {
        for event in window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::RBracket, Action::Press, _) => {
                    time_warp_index = (time_warp_index + 1).min(TIME_WARP_STEPS.len() - 1);
                }
                WindowEvent::Key(Key::LBracket, Action::Press, _) => {
                    time_warp_index = time_warp_index.saturating_sub(1);
                }
                _ => {}
            }
        }

        // gui_lib::draw_full_axes(&mut window, 100.0, STAR_RADIUS);
        let eye = camera.eye();
        let real_t = started_at.elapsed().as_secs_f64();
        let now = Instant::now();
        let real_dt = (now - previous_frame).as_secs_f64();
        previous_frame = now;
        let time_warp = TIME_WARP_STEPS[time_warp_index];
        simulated_seconds += real_dt * time_warp as f64;
        t = Time::from_secs(simulated_seconds);
        for planet in &mut planets {
            planet.r = planet.orbit.position(t);
            draw_orbit_and_current_position(&mut window, &eye, scale, planet);
        }

        {
            let (r, v) = spacecraft.trajectory.position_and_velocity(t);
            spacecraft.r = r;
            spacecraft.v = v;
        }
        let now_date = planets_epoch + Duration::from(t);
        if spacecraft.planet_idx.is_none()
            && let Some((i, encounter)) = spacecraft.next_encounter
            && now_date >= encounter.t_enc
        {
            let planet = &planets[i];
            eprintln!(
                "Spacecraft entering SOI of planet {} at {}",
                planet.body.name, encounter.t_enc
            );

            // Use the pre-computed (E1, E2) at SOI entry so the transition
            // is deterministic regardless of frame cadence / time warp.
            let (craft_r, craft_v) = match &spacecraft.trajectory {
                Trajectory::Elliptic(o) => {
                    o.position_and_velocity(EccAnomaly::from(Angle::from_rad(encounter.E1)))
                }
                Trajectory::Hyperbolic(o) => {
                    o.position_and_velocity(HypAnomaly::from(encounter.E1))
                }
            };
            let (planet_r, planet_v) =
                planet.orbit.position_and_velocity(EccAnomaly::from(Angle::from_rad(encounter.E2)));

            // Convert heliocentric state vectors to planetocentric.
            spacecraft.planet_idx = Some(i);
            spacecraft.next_encounter = None;
            // Rebuild as a flyby hyperbola relative to the planet, timestamped
            // at the exact encounter instant (seconds since planets_epoch).
            let t_enc_secs = (encounter.t_enc - planets_epoch).as_seconds_f64();
            spacecraft.trajectory = Trajectory::from_state_vectors(
                planet.body.μ,
                craft_r - planet_r,
                craft_v - planet_v,
                t_enc_secs,
            );
            spacecraft.points =
                gui_lib::generate_trajectory_points(planet.soi_radius, &spacecraft.trajectory, 100)
                    .iter()
                    .map(|point| point.map(|x| (scale * x) as f32))
                    .collect();
            let (craft_r, craft_v) = spacecraft.trajectory.position_and_velocity(t);
            spacecraft.r = craft_r;
            spacecraft.v = craft_v;

            // Pre-compute the deterministic SOI exit: solve r(H) = r_soi on
            // the planetocentric hyperbola, take the positive (post-periapsis)
            // root, convert to mean anomaly, derive the absolute exit time.
            let Trajectory::Hyperbolic(hyp) = &spacecraft.trajectory else {
                unreachable!("planetocentric flyby is always hyperbolic")
            };
            let a = hyp.orbit_2d.0.a();
            let e_hyp = hyp.orbit_2d.0.e();
            let mu = hyp.orbit_2d.0.mu();
            let n_hyp = (mu / a.abs().powi(3)).sqrt();
            // M_from_t uses M0 + n·t, so M0 stored is the mean anomaly at
            // t = 0 (J2000). Advance it to the capture instant `t_enc_secs`
            // to get the actual M at SOI entry.
            let h_exit = ((spacecraft.trajectory.a() - planet.soi_radius)
                / (spacecraft.trajectory.a() * spacecraft.trajectory.e()))
            .acosh();
            let m_exit = e_hyp * h_exit.sinh() - h_exit;
            dbg!(&m_exit);
            let dt_exit_s = 2.0 * m_exit / n_hyp;
            let t_exit = encounter.t_enc + chrono::Duration::nanoseconds((dt_exit_s * 1e9) as i64);
            eprintln!(
                "SOI exit pre-computed: H_exit={h_exit:.4}, Δt={:.2} days → t_exit={t_exit}",
                dt_exit_s / 86400.0,
            );
            spacecraft.soi_exit = Some((h_exit, t_exit));
        }
        if let Some((h_exit, t_exit)) = spacecraft.soi_exit
            && now_date >= t_exit
            && let Some(planet_idx) = spacecraft.planet_idx
        {
            let planet = &planets[planet_idx];
            eprintln!("Spacecraft leaving SOI of planet {} at {t_exit}", planet.body.name);

            // Use pre-computed H_exit for deterministic planetocentric state
            // at the exit instant, regardless of frame cadence / time warp.
            let (craft_r, craft_v) = match &spacecraft.trajectory {
                Trajectory::Hyperbolic(o) => o.position_and_velocity(HypAnomaly::from(h_exit)),
                Trajectory::Elliptic(_) => unreachable!("planetocentric flyby is hyperbolic"),
            };
            let t_exit_secs = (t_exit - planets_epoch).as_seconds_f64();
            let (planet_r, planet_v) = planet.orbit.position_and_velocity(Time::from_secs(t_exit_secs));

            spacecraft.planet_idx = None;
            spacecraft.soi_exit = None;
            // Rebuild the heliocentric trajectory from the new state at the
            // exact SOI-exit instant (not the current frame time).
            spacecraft.trajectory = Trajectory::from_state_vectors(
                system.star.μ,
                craft_r + planet_r,
                craft_v + planet_v,
                t_exit_secs,
            );
            spacecraft.points =
                gui_lib::generate_trajectory_points(1e11, &spacecraft.trajectory, 360)
                    .iter()
                    .map(|point| point.map(|x| (scale * x) as f32))
                    .collect();
            spacecraft.next_encounter =
                first_future_encounter(&spacecraft, &planets, planets_epoch + Duration::from(t));
            let (craft_r, craft_v) = spacecraft.trajectory.position_and_velocity(t);
            spacecraft.r = craft_r;
            spacecraft.v = craft_v;
            dbg!(&spacecraft.next_encounter);
        };
        draw_orbit_and_current_position_of_spacecraft(
            &mut window,
            &eye,
            scale,
            &mut spacecraft,
            t,
            planets_epoch,
            &hud_font,
            time_warp,
            &planets,
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
) {
    gui_lib::draw_orbit_points(window, &body.points, &body.color, false);

    let body_scaled_position = body.r.map(|x| (scale * x) as f32);
    body.sphere.set_local_translation(Translation3 { vector: body_scaled_position });
    //let clamp = (0.01, (50.0 * body.orbit.orbit_2d.0.a().max(AU_IN_KM) / 90118820.0) as f32);
    let planet_scale = (eye.coords - body_scaled_position).magnitude() * 0.02;
    body.sphere.set_local_scale(planet_scale, planet_scale, planet_scale);
}

fn draw_orbit_and_current_position_of_spacecraft(
    window: &mut Window,
    eye: &Point3<f32>,
    scale: f64,
    spacecraft: &mut Spacecraft,
    t: Time,
    epoch: DateTime<Utc>,
    hud_font: &Rc<Font>,
    time_warp: i64,
    planets: &[Body],
) {
    let is_hyperbolic = match spacecraft.trajectory {
        Trajectory::Elliptic(_) => false,
        Trajectory::Hyperbolic(_) => true,
    };
    let heliocentric_r = if let Some(i) = spacecraft.planet_idx {
        let planet_r = planets[i].r;

        let points: Vec<_> =
            spacecraft.points.iter().map(|p| p + (scale * planet_r).map(|x| x as f32)).collect();
        gui_lib::draw_orbit_points(window, &points, &spacecraft.color, is_hyperbolic);
        spacecraft.r + planet_r
    } else {
        gui_lib::draw_orbit_points(window, &spacecraft.points, &spacecraft.color, is_hyperbolic);
        spacecraft.r
    };

    let scaled_position = heliocentric_r.map(|x| (scale * x) as f32);
    spacecraft.node.set_local_translation(Translation3 { vector: scaled_position });
    spacecraft.node.set_local_rotation(UnitQuaternion::face_towards(
        &-Vector3::z(),
        &(planets[2].r - heliocentric_r).normalize().map(|x| x as f32),
    ));
    let scale = (eye.coords - scaled_position).magnitude() * 0.2;
    spacecraft.node.set_local_scale(scale, scale, scale);

    let (distance, distance_unit) = if spacecraft.r.magnitude() < (AU_IN_KM * 0.2) {
        (spacecraft.r.magnitude(), "km")
    } else {
        (spacecraft.r.magnitude() / AU_IN_KM, "au")
    };
    let telemetry_text = format!(
        "Warp: {warp}x\nTime: {time}\nDistance: {distance:.2} {distance_unit}\nSpeed: {speed:.1} km/s",
        warp = format_signed_warp(time_warp),
        time = (epoch + Duration::from(t)).format("%Y-%m-%d %H:%M"),
        speed = spacecraft.v.magnitude(),
    );
    let orbit_text = format!(
        "SOI: {soi}\n{orbit}",
        soi = spacecraft.planet_idx.map_or("Sun", |i| &planets[i].body.name),
        orbit = spacecraft.trajectory
    );
    let text_scale = 50.0;
    window.draw_text(
        &telemetry_text,
        &Point2::new(0.0, 0.0),
        text_scale,
        hud_font,
        &Point3::new(1.0, 1.0, 1.0),
    );
    window.draw_text(
        &orbit_text,
        &Point2::new(0.0, text_scale * 5.0),
        text_scale,
        hud_font,
        &Point3::new(1.0, 1.0, 1.0),
    );
}

fn format_signed_warp(time_warp: i64) -> String {
    if time_warp < 0 {
        format!("-{}", format_with_thousand_separators(time_warp.unsigned_abs()))
    } else {
        format_with_thousand_separators(time_warp.unsigned_abs())
    }
}

fn load_ttf_font_from_current_dir() -> Rc<Font> {
    let font_path = Path::new("OpenSans-Regular.ttf");
    Font::new(font_path).unwrap_or_else(|| panic!("failed to load {}", font_path.display()))
}

fn create_body(
    window: &mut Window,
    scale: f64,
    body: CelestialBody,
    orbit: Orbit3D<EllipticOrbit>,
) -> Body {
    let points = gui_lib::generate_ellipse_points(&orbit, 360)
        .iter()
        .map(|point| point.map(|x| (scale * x) as f32))
        .collect();
    let color = rgb8_to_color(body.color);

    let mut sphere = window.add_sphere(PLANET_RADIUS as f32);
    sphere.set_color(color.x, color.y, color.z);
    let soi_radius = orbit.orbit_2d.0.a() * (body.μ / orbit.orbit_2d.0.mu()).powf(0.4);
    let r = orbit.position(Time::from_secs(0.0));

    Body { sphere, body, orbit, points, color, soi_radius, r }
}

fn create_spacecraft(
    window: &mut Window,
    scale: f64,
    trajectory: Trajectory,
    color: Point3<f32>,
    planet_idx: Option<usize>,
) -> Spacecraft {
    let points: Vec<_> = gui_lib::generate_trajectory_points(1e10, &trajectory, 100)
        .into_iter()
        .map(|point| point.map(|x| (x * scale) as f32))
        .collect();

    let mtl_dir = Path::new("models");
    let obj_path = Path::new("models/voyager.obj");
    let node = window.add_obj(obj_path, mtl_dir, Vector3::new(1.0, 1.0, 1.0));
    let (r, v) = trajectory.position_and_velocity(Time::from_secs(0.0));

    Spacecraft {
        node,
        trajectory,
        points,
        color,
        planet_idx,
        soi_exit: None,
        next_encounter: None,
        r,
        v,
    }
}

struct Body {
    sphere: SceneNode,
    body: CelestialBody,
    orbit: Orbit3D<EllipticOrbit>,
    points: Vec<Point3<f32>>,
    color: Point3<f32>,
    soi_radius: f64,
    r: Vector3<f64>,
}

struct Spacecraft {
    node: SceneNode,
    planet_idx: Option<usize>,
    /// When inside a planet's SOI: the pre-computed (H_exit, t_exit) on the
    /// planetocentric hyperbola. Populated at SOI entry, consumed at exit —
    /// keeps the exit transition deterministic under high time warp.
    soi_exit: Option<(f64, DateTime<Utc>)>,
    trajectory: Trajectory,
    next_encounter: Option<(usize, Encounter)>,
    points: Vec<Point3<f32>>,
    color: Point3<f32>,
    r: Vector3<f64>,
    v: Vector3<f64>,
}
