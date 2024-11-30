#[derive(Clone, Copy)]
pub struct Time(f64);

impl Time {
    pub fn from_secs(s: f64) -> Self {
        Self(s)
    }

    pub fn as_secs(&self) -> f64 {
        self.0
    }
}
