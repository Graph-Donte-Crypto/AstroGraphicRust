#![allow(non_snake_case)]

#[macro_use]
extern crate getset;
#[macro_use]
extern crate derive_builder;

pub mod angle;
pub mod approx_pos;
pub mod config;
pub mod gravity_assist;
pub mod kepler_equation;
pub mod orbit;
pub mod state_vectors;
pub mod time;
pub mod trajectory;
pub mod util;


pub const AU_IN_KM: f64 = 149_597_870.700;

