use std::time::Duration;

#[derive(Clone, Copy, Debug)]
pub struct Time(f64);

impl Time {
    pub fn from_secs(s: f64) -> Self {
        Self(s)
    }

    pub fn as_secs(&self) -> f64 {
        self.0
    }
}

impl From<Duration> for Time {
    fn from(duration: Duration) -> Self {
        Self::from_secs(duration.as_secs_f64())
    }
}

impl From<Time> for Duration {
    fn from(time: Time) -> Duration {
        Duration::from_secs_f64(time.as_secs())
    }
}

impl From<Time> for chrono::Duration {
    fn from(time: Time) -> chrono::Duration {
        let duration =
            chrono::Duration::from_std(Duration::from_secs_f64(time.as_secs().abs())).unwrap();
        if time.as_secs() >= 0.0 {
            duration
        } else {
            -duration
        }
    }
}
