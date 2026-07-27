use astrorust_gui_lib as gui_lib;
use astrorust_gui_lib::kiss3d::camera::Camera;
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
use gui_lib::kiss3d::event::{Action, Key, MouseButton, WindowEvent};
use gui_lib::kiss3d::light::Light;
use gui_lib::kiss3d::nalgebra as na;
use gui_lib::kiss3d::window::Window;
use kiss3d::camera::ArcBall;
use na::{Point2, Point3, Translation3, Vector2, Vector3};
use std::f64::consts::TAU;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

const TEXT_SIZE: f32 = 30.0;
const CAMERA_ACCELERATION: f64 = 0.0;
/// Zoom-out limit, in multiples of the system radius (outermost apoapsis).
const CAMERA_MAX_DIST_FACTOR: f64 = 2.0;
/// Extra far plane headroom past the zoom-out limit, in multiples of the system radius.
const CAMERA_ZFAR_MARGIN: f64 = 1.5;
/// Near clipping plane. Large enough to keep depth buffer precision at solar-system scale.
const CAMERA_ZNEAR: f32 = 1.0;
/// Radians of camera rotation per pixel of mouse drag. ArcBall's own default is 0.005.
const CAMERA_ROTATE_SENSITIVITY: f32 = 0.002;
/// Distance multiplier per unit of scroll. Below 1 so that scrolling up zooms in.
const CAMERA_ZOOM_STEP: f32 = 1.0 / 1.01;
/// Furthest the cursor may travel between press and release for it to count as a click rather
/// than a camera drag, in px.
const CLICK_SLACK: f32 = 6.0;
/// Screen-space radius around a body within which a click selects it, in px.
const PICK_RADIUS: f32 = 30.0;
/// How long the "Focus: ..." label stays on screen after a body is selected.
const FOCUS_LABEL_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(3);
/// Rough width of one glyph as a fraction of the font size, used to centre the focus label.
const FOCUS_LABEL_CHAR_WIDTH: f32 = 0.5;
/// Name shown for the spacecraft, which unlike the celestial bodies has none in the config.
const SPACECRAFT_NAME: &str = "Spacecraft";
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

#[kiss3d::main]
async fn main() {
    let mut window = Window::new("AstroGraphicRust");
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

    // Default ArcBall frustum (zfar = 1000) clips everything past Mars at our scale, so size the
    // far plane from the apoapsis of the outermost planet. zfar is measured from the camera, not
    // from the origin, so it must cover the zoom-out limit plus the far side of the system.
    // znear is kept well above 0 to preserve depth buffer precision.
    let system_radius =
        scale * planets.iter().map(|planet| planet.orbit.apoapsis()).fold(0.0, f64::max);
    let max_dist = CAMERA_MAX_DIST_FACTOR * system_radius;
    let zfar = (max_dist + CAMERA_ZFAR_MARGIN * system_radius) as f32;
    let mut camera = ArcBall::new_with_frustum(
        std::f32::consts::FRAC_PI_4,
        CAMERA_ZNEAR,
        zfar,
        Point3::from([0.0, -500.0, 300.0]),
        Point3::origin(),
    );
    camera.set_max_dist(max_dist as f32);
    // ArcBall's rotation speed (yaw_step / pitch_step) is not configurable, so unbind its rotate
    // button and drive yaw/pitch from the event loop below at our own sensitivity instead.
    // Cursor tracking stays with ArcBall, so panning is unaffected.
    camera.rebind_rotate_button(None);
    let mut cursor_pos: Option<Point2<f32>> = None;
    let mut is_rotating = false;
    // Total cursor travel since the left button went down. Compared against CLICK_SLACK to tell a
    // click apart from a drag; straight-line distance would misread a drag that happens to end
    // where it started as a click.
    let mut drag_travel = 0.0;
    // Owned rather than borrowed from `planets`, which is mutated every frame.
    let mut focus: Option<(String, Instant)> = None;
    let hud_font = Arc::new(load_ttf_font_from_current_dir());

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

    while window.render_with_camera(&mut camera).await {
        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::RBracket, Action::Press, _) => {
                    time_warp_index = (time_warp_index + 1).min(TIME_WARP_STEPS.len() - 1);
                }
                WindowEvent::Key(Key::LBracket, Action::Press, _) => {
                    time_warp_index = time_warp_index.saturating_sub(1);
                }
                WindowEvent::MouseButton(MouseButton::Button1, action, _) => {
                    is_rotating = action == Action::Press;
                    match action {
                        Action::Press => drag_travel = 0.0,
                        // Releasing without having moved is a click: focus whatever is under it.
                        Action::Release if drag_travel < CLICK_SLACK => {
                            let screen_size = window.size().map(|x| x as f32);
                            let picked = cursor_pos.and_then(|pos| {
                                pick_body(
                                    &camera,
                                    &system.star.name,
                                    &planets,
                                    &spacecraft,
                                    scale,
                                    pos,
                                    screen_size,
                                )
                            });
                            if let Some((name, position)) = picked {
                                camera.set_at(position);
                                focus = Some((name.to_owned(), Instant::now()));
                            }
                        }
                        Action::Release => {}
                    }
                }
                WindowEvent::CursorPos(x, y, _) => {
                    let pos = Point2::new(x as f32, y as f32);
                    if let (true, Some(previous_pos)) = (is_rotating, cursor_pos) {
                        let dpos = pos - previous_pos;
                        drag_travel += dpos.norm();
                        camera.set_yaw(camera.yaw() + dpos.x * CAMERA_ROTATE_SENSITIVITY);
                        camera.set_pitch(camera.pitch() - dpos.y * CAMERA_ROTATE_SENSITIVITY);
                    }
                    cursor_pos = Some(pos);
                }
                WindowEvent::Scroll(_, off, _) => {
                    // ArcBall zooms towards the cursor, which drags `at` sideways whenever the
                    // cursor is off-centre. Inhibit it and scale the distance directly, so the
                    // focus point stays put.
                    event.inhibited = true;
                    let dist = camera.dist() * CAMERA_ZOOM_STEP.powf(off as f32);
                    camera.set_dist(dist);
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
            let (planet_r, planet_v) =
                planet.orbit.position_and_velocity(Time::from_secs(t_exit_secs));

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

        focus = focus.filter(|(_, since)| since.elapsed() < FOCUS_LABEL_TIMEOUT);
        if let Some((name, _)) = &focus {
            draw_focus_label(&mut window, name, &hud_font);
        }

        if CAMERA_ACCELERATION > f64::EPSILON {
            let at = camera.at();
            camera.set_up_axis(Vector3::new(0.0, 0.0, 1.0));
            camera.look_at(
                eye + (0.5 * CAMERA_ACCELERATION * real_t * real_t) as f32 * eye.coords.normalize(),
                at,
            );
        }
    }
}

/// Draws the currently focused body's name across the top of the window. kiss3d cannot measure a
/// string, so the horizontal centring is an estimate based on the glyph count.
fn draw_focus_label(window: &mut Window, name: &str, hud_font: &Arc<Font>) {
    let text = format!("Focus: {name}");
    let width = text.chars().count() as f32 * TEXT_SIZE * FOCUS_LABEL_CHAR_WIDTH;
    let x = (window.size().x as f32 - width).max(0.0) / 2.0;
    window.draw_text(&text, &Point2::new(x, 0.0), TEXT_SIZE, hud_font, &Point3::new(1.0, 1.0, 1.0));
}

/// Position of the spacecraft relative to the star. Inside a planet's SOI its state vector is
/// planetocentric, so the planet's own position has to be added back.
fn heliocentric_r(spacecraft: &Spacecraft, planets: &[Body]) -> Vector3<f64> {
    match spacecraft.planet_idx {
        Some(i) => spacecraft.r + planets[i].r,
        None => spacecraft.r,
    }
}

/// Returns the world position of the star, a planet or the spacecraft drawn closest to `cursor` on
/// screen, as long as it is within [`PICK_RADIUS`]. Bodies behind the camera are ignored: `project`
/// divides by a negative w for those and would otherwise report a bogus, sometimes very close,
/// screen position.
fn pick_body<'a>(
    camera: &ArcBall,
    star_name: &'a str,
    planets: &'a [Body],
    spacecraft: &Spacecraft,
    scale: f64,
    cursor: Point2<f32>,
    screen_size: Vector2<f32>,
) -> Option<(&'a str, Point3<f32>)> {
    let star = std::iter::once((star_name, Point3::origin()));
    let spacecraft = std::iter::once((
        SPACECRAFT_NAME,
        Point3::from(heliocentric_r(spacecraft, planets).map(|x| (scale * x) as f32)),
    ));
    let planets = planets.iter().map(|planet| {
        (planet.body.name.as_str(), Point3::from(planet.r.map(|x| (scale * x) as f32)))
    });
    star.chain(planets)
        .chain(spacecraft)
        .filter(|(_, body)| (camera.view_transform() * body).z < 0.0)
        .map(|(name, body)| {
            let projected = camera.project(&body, &screen_size);
            // `project` measures y upwards from the bottom, cursor coordinates downwards from
            // the top.
            let projected = Point2::new(projected.x, screen_size.y - projected.y);
            ((name, body), (projected - cursor).norm())
        })
        .filter(|&(_, distance)| distance < PICK_RADIUS)
        .min_by(|(_, a), (_, b)| a.total_cmp(b))
        .map(|(body, _)| body)
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
    hud_font: &Arc<Font>,
    time_warp: i64,
    planets: &[Body],
) {
    let is_hyperbolic = match spacecraft.trajectory {
        Trajectory::Elliptic(_) => false,
        Trajectory::Hyperbolic(_) => true,
    };
    if let Some(i) = spacecraft.planet_idx {
        let points: Vec<_> = spacecraft
            .points
            .iter()
            .map(|p| p + (scale * planets[i].r).map(|x| x as f32))
            .collect();
        gui_lib::draw_orbit_points(window, &points, &spacecraft.color, is_hyperbolic);
    } else {
        gui_lib::draw_orbit_points(window, &spacecraft.points, &spacecraft.color, is_hyperbolic);
    };
    let heliocentric_r = heliocentric_r(spacecraft, planets);

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
    window.draw_text(
        &telemetry_text,
        &Point2::new(0.0, 0.0),
        TEXT_SIZE,
        &hud_font,
        &Point3::new(1.0, 1.0, 1.0),
    );
    window.draw_text(
        &orbit_text,
        &Point2::new(0.0, TEXT_SIZE * 5.0),
        TEXT_SIZE,
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

fn load_ttf_font_from_current_dir() -> Font {
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
