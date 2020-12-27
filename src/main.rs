extern crate kiss3d;
extern crate nalgebra as na;

use na::{Vector3, UnitQuaternion, Translation3, Point3};
use kiss3d::window::Window;
use kiss3d::light::Light;

struct Orbit {
    pub a: f32,
    pub e: f32,
    pub omega_big: f32,
    pub omega_small: f32,
    pub i: f32
}

impl Orbit {

    fn new(a: f32, e: f32, omega_big: f32, omega_small: f32, i: f32) -> Orbit {
        return Orbit {
            a, 
            e, 
            omega_big: omega_big.to_radians(), 
            omega_small: omega_small.to_radians(), 
            i: i.to_radians(),
        }
 
    }

    fn get_radius(&self, nu: f32) -> f32 {
        return (self.a * (1.0 - self.e * self.e)) / (1.0 + self.e * nu.cos());
    }

    //nu in [0; 2*PI]
    pub fn get_radius_vector(&self, nu: f32) -> Vector3<f32>{
        let W = self.omega_big;
        let w = self.omega_small;
        let i = self.i;
        let v = w + nu;

        let r: f32 = self.get_radius(nu);
        let x = r * (W.cos() * v.cos() - W.sin() * v.sin() * i.cos());
        let y = r * (W.sin() * v.cos() + W.cos() * v.sin() * i.cos());
        let z = r * (v.sin() * i.sin());
        return Vector3::new(x, y, z);
    } 

    pub fn get_orbit_points(&self, count: i32) -> Vec<Point3<f32>> {
        use core::f32::consts::TAU;

        let mut nu: f32 = 0.0;
        let mut vec : Vec<Point3<f32>> = Vec::new();

        for i in 0..count {
            vec.push(Point3::from(self.get_radius_vector(nu)));
            nu += TAU / (count as f32);
        }
        return vec;
    }
}

fn main() {
    let mut window = Window::new("Kiss3d: cube");
    //let mut c      = window.add_cube(1.0, 1.0, 1.0);

    let mut sphere = window.add_sphere(1.0);
    let mut planet = window.add_sphere(0.2);

    sphere.set_color(1.0, 1.0, 0.0);

    window.set_light(Light::StickToCamera);

    //let tra = Translation3::new(1.0, 2.0, 2.0);
    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);
    
    //let PI = 3.141592656;
    //let mut nu : f32 = 0.0;


    let orbit = Orbit{
        a: 5.263138,
        e: 0.8,
        i: 7.0_f32.to_radians(),
        omega_small: 15.0_f32.to_radians(),
        omega_big: 70.0_f32.to_radians()
    };

    
    let points_count : usize = 3 * 360;
    let vec = orbit.get_orbit_points(points_count as i32);

    //planet.set_local_translation(Translation3::new(1.0, 0.0, 0.0));
    
    //sphere.prepend_to_local_translation(tra);
    //sphere.set_local_translation(tra);

    window.set_line_width(1.0);

    let mut index : usize = 0;

    fn point_to_vector(point: &Point3<f32>) -> Vector3<f32> {
        return Vector3::new(point.x, point.y, point.z);
    }

    while window.render() {

        planet.set_local_translation(Translation3::<f32>::from(point_to_vector(&vec[index])));        
                //vec[index]));
        index += 1 as usize;
        index = index % points_count;

        for i in 0..vec.len()-1 {
            window.draw_line(&vec[i], &vec[i + 1], &Point3::<f32>::new(0.0, 1.0, 1.0));
        }
        window.draw_line(&vec[0], &vec[vec.len() - 1], &Point3::<f32>::new(0.0, 1.0, 1.0));

        sphere.prepend_to_local_rotation(&rot);
        //planet.set_local_translation(Translation3::new(20.0 * angle.cos(), 20.0 * angle.sin(), 0.0));
        //angle += PI / 180.0;
    }
}
