pub struct AccelerationState {
    pub accel_now: f32,
    pub accel_vel: f32,
    pub accel_alt: f32,
}

impl AccelerationState {
    pub fn new() -> Self {
        Self {
            accel_now: 0.0,
            accel_vel: 0.0,
            accel_alt: 0.0,
        }
    }

    pub fn update(&mut self, accel_now: f32, dt_us: u32) {
        let dt_seconds = dt_us as f32 * 1.0e-6;
        self.accel_now = accel_now;
        self.accel_vel += accel_now * dt_seconds;
        self.accel_alt += self.accel_vel * dt_seconds;
    }
}
