use std::f64::consts::{PI, TAU};
use std::ops::{Add, Deref, Sub};

use nalgebra::ComplexField;

use crate::kepler_equation;

#[derive(Debug, Clone, Copy, Default)]
pub struct Angle(f64);

impl Angle {
    pub fn from_rad(rad: f64) -> Self {
        Self(rad)
    }

    pub fn from_deg(deg: f64) -> Self {
        Self(deg.to_radians())
    }

    pub fn as_deg(&self) -> f64 {
        self.0.to_degrees()
    }

    pub fn as_rad(&self) -> f64 {
        self.0
    }

    /// Normalize angle to [-PI; +PI]
    pub fn normalize(&self) -> Self {
        // Wrap M to [-PI; +PI]
        let M = (self.0 + PI) % TAU;
        Self(if M < 0.0 { M + PI } else { M - PI })
    }

    pub fn sin_cos(&self) -> (f64, f64) {
        self.0.sin_cos()
    }

    pub fn sin(&self) -> f64 {
        self.0.sin()
    }
}

impl Add for Angle {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Angle {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct EccAnomaly(Angle);

impl From<Angle> for EccAnomaly {
    fn from(angle: Angle) -> Self {
        Self(angle)
    }
}

impl Deref for EccAnomaly {
    type Target = Angle;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HypAnomaly(f64);

impl HypAnomaly {
    pub fn sinh_cosh(&self) -> (f64, f64) {
        self.0.sinh_cosh()
    }
}

impl From<f64> for HypAnomaly {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl Deref for HypAnomaly {
    type Target = f64;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Clone, Copy)]
pub struct TrueAnomaly(Angle);

impl From<Angle> for TrueAnomaly {
    fn from(angle: Angle) -> Self {
        Self(angle)
    }
}

impl Deref for TrueAnomaly {
    type Target = Angle;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct MeanAnomaly(Angle);

impl From<Angle> for MeanAnomaly {
    fn from(angle: Angle) -> Self {
        Self(angle)
    }
}

impl Deref for MeanAnomaly {
    type Target = Angle;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

pub trait IntoAnomaly<T> {
    fn into_anomaly(self, e: f64) -> T;
}

impl<T> IntoAnomaly<T> for T {
    fn into_anomaly(self, _: f64) -> T {
        self
    }
}

impl IntoAnomaly<EccAnomaly> for MeanAnomaly {
    fn into_anomaly(self, e: f64) -> EccAnomaly {
        kepler_equation::solve_kepler_householder_pade_elliptic(e, self)
    }
}

impl IntoAnomaly<HypAnomaly> for MeanAnomaly {
    fn into_anomaly(self, e: f64) -> HypAnomaly {
        crate::orbit::flat::hyperbolic::HyperbolaSolver::new(e).solve(self.as_rad()).into()
    }
}
