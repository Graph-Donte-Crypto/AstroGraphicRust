//extern crate nalgebra as na;
#[macro_use(lazy_static)]
extern crate lazy_static;
use kiss3d::nalgebra as na;

use na::{Vector3, UnitQuaternion, Translation3, Point3};
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::event::{Key, MouseButton};

//pub use kiss3d::nalgebra;
pub use kiss3d;

pub struct Orbit {
    a: f32,
    e: f32,
    omega_big: f32,
    omega_small: f32,
    i: f32,
    mu: f32,
    points : Vec<Point3<f32>>,
    /* pre-computed */
    speed_root: f32, // sqrt(mu / a)
    mean_motion: f32,   // sqrt(mu / a^3)
    period_value: f32,  // orbital period
    e_root: f32,	     // sqrt(1 - e^2)
    a_e_root: f32,	     // a * e_root
    orb_to_ecl: [f32; 6] 
}

impl Orbit {

    pub fn new(a: f32, e: f32, omega_big: f32, omega_small: f32, i: f32, mu:f32, points_count: usize) -> Orbit {
        let speed_root = (mu / a).sqrt();
        let mean_motion = speed_root / a;
        let e_root = (1.0 - e * e).sqrt();
        let i_rad = i.to_radians();
        let om_big = omega_big.to_radians();
        let om_sma = omega_small.to_radians();
        let mut orbit = Orbit {
            a, 
            e, 
            omega_big: om_big, 
            omega_small: om_sma, 
            i: i_rad,
            mu,
            points: Vec::<Point3<f32>>::new(),
            /* pre-computed */
            speed_root,
            mean_motion,
            period_value: core::f32::consts::TAU / mean_motion,
            e_root,
            a_e_root: a * e_root,
            orb_to_ecl : [
                 om_big.cos() * om_sma.cos() - om_big.sin() * om_sma.sin() * i_rad.cos(),
                -om_big.cos() * om_sma.sin() - om_big.sin() * om_sma.cos() * i_rad.cos(),
                 om_big.sin() * om_sma.cos() + om_big.cos() * om_sma.sin() * i_rad.cos(),
                -om_big.sin() * om_sma.sin() + om_big.cos() * om_sma.cos() * i_rad.cos(),
                om_sma.sin() * i_rad.sin(),
                om_sma.cos() * i_rad.sin()
            ]
        };
        orbit.generate_orbit_points(points_count);
        return orbit;
    }

    pub fn get_points(&self) -> &Vec<Point3<f32>> {
        return &self.points;
    }

    //nu in [0; 2*PI]
    fn get_radius_vector(&self, nu: f32) -> Vector3<f32>{  
        let e = self.e;
        let a = self.a;
        let e_root = self.e_root;
        let ote = &self.orb_to_ecl;
        let temp_del = 1.0 + e * nu.cos();

        let e_anomaly_cos = (e + nu.cos()) / temp_del;
        let e_anomaly_sin = e_root * nu.sin() / temp_del;
        
        let r_x = a * (e_anomaly_cos - e);
        let r_y = e_root * a * e_anomaly_sin;
        
        return Vector3::<f32>::new(
            ote[0] * r_x + ote[1] * r_y,  
            ote[2] * r_x + ote[3] * r_y,  
            ote[4] * r_x + ote[5] * r_y  
        );
    } 

    pub fn get_position_and_velocity(&self, nu: f32) -> (Point3<f32>, Vector3<f32>) {
        
        let e = self.e;
        let a = self.a;
        let e_root = self.e_root;
        let speed_root = self.speed_root;
        let ote = &self.orb_to_ecl;
        let temp_del = 1.0 + e * nu.cos();

        let e_anomaly_cos = (e + nu.cos()) / temp_del;
        let e_anomaly_sin = e_root * nu.sin() / temp_del;

        let r_x = a * (e_anomaly_cos - e);
        let r_y = e_root * a * e_anomaly_sin;
        
        let v_mult = speed_root / (1.0 - e * e_anomaly_cos);
        let v_x = -v_mult * e_anomaly_sin;
        let v_y =  v_mult * e_root * e_anomaly_cos;

        return (
            Point3::<f32>::new(
                ote[0] * r_x + ote[1] * r_y,  
                ote[2] * r_x + ote[3] * r_y,  
                ote[4] * r_x + ote[5] * r_y  
            ),
            Vector3::<f32>::new(
                ote[0] * v_x + ote[1] * v_y,  
                ote[2] * v_x + ote[3] * v_y,  
                ote[4] * v_x + ote[5] * v_y
            )
        );
    }

    fn generate_orbit_points(&mut self, count: usize) {
        use core::f32::consts::TAU;

        let mut nu: f32 = 0.0;

        for _ in 0..count {
            let point = Point3::from(self.get_radius_vector(nu));
            self.points.push(point);
            nu += TAU / (count as f32);
        }
    }
}

pub struct SpaceBody {
    orbit: Orbit,
    r: f32,
    nu0: f32,
    nu: f32,
    position : Point3<f32>,
    velocity : Vector3<f32>
}

impl SpaceBody {
    
    pub fn new(orbit: Orbit, r:f32, nu0 : f32, nu: f32) -> SpaceBody { 
        let (position, velocity) = orbit.get_position_and_velocity(nu);

        return SpaceBody {
            orbit,
            r,
            nu0,
            nu,
            position,
            velocity
        };
    }


    pub fn move_to_true_anomaly(&mut self, nu: f32) {
        self.nu = nu;
        let (position, velocity) = self.orbit.get_position_and_velocity(nu);
        self.position = position;
        self.velocity = velocity;
    }

    pub fn get_position(&self) -> &Point3<f32> {
        return &self.position;
    }

    pub fn get_position_as_translation(&self) -> Translation3<f32> {
        return Translation3::<f32>::from(self.position.coords);
    }

    pub fn get_velocity(&self) -> &Vector3<f32> {
        return &self.velocity;
    } 

    pub fn get_true_anomaly(&self) -> f32 {
        return self.nu;
    }
}


lazy_static! {

    static ref MOHO_EXAMPLE: SpaceBody = SpaceBody::new(
        Orbit::new(5.263138, 0.2, 7.0, 15.0, 70.0, 1172.3328, 500),
        2.5,
        180_f32.to_radians(),
        0.0
    );

}


pub fn draw_orbit(window: &mut kiss3d::window::Window, orbit: &Orbit, color: &Point3<f32>) {
    let points = &(orbit.points);
    for i in 0..points.len() - 1 {
        window.draw_line(&(points[i]), &(points[i + 1]), &color);
    }
    window.draw_line(&(points[0]), &(points[points.len() - 1]), &color);
}

pub fn draw_space_body(window: &mut kiss3d::window::Window, space_body: &SpaceBody) {

}

pub fn draw_space_body_with_orbit(window: &mut kiss3d::window::Window, space_body: &SpaceBody, color: &Point3<f32>) {
    draw_orbit(window, &(space_body.orbit), color);
    draw_space_body(window, space_body);
}

