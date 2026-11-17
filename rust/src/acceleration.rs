use crate::baro_samp::{update_average, BarometerState};

pub struct AccelerationState {
    pub accel_now: f32,
    pub accel_vel: f32,
    pub accel_alt: f32,
    pub max_accel: f32,
    pub fusion_vel: f32,
    pub fusion_alt: f32,
    pub max_velocity: f32,
    pub high_g_smooth: i32,
    high_g_buffer: [f32; 30],
    high_g_sum: f32,
    high_g_index: usize,
    high_g_count: usize,
}

impl AccelerationState {
    pub fn new() -> Self {
        Self {
            accel_now: 0.0,
            accel_vel: 0.0,
            accel_alt: 0.0,
            max_accel: 0.0,
            fusion_vel: 0.0,
            fusion_alt: 0.0,
            max_velocity: 0.0,
            high_g_smooth: 0,
            high_g_buffer: [0.0; 30],
            high_g_sum: 0.0,
            high_g_index: 0,
            high_g_count: 0,
        }
    }

    pub fn update_high_g(&mut self, sample: i32) {
        self.high_g_count = (self.high_g_count + 1).min(self.high_g_buffer.len());
        let average;
        (average, self.high_g_sum, self.high_g_index) = update_average(
            &mut self.high_g_buffer,
            self.high_g_sum,
            self.high_g_index,
            sample as f32,
            self.high_g_count,
        );
        self.high_g_smooth = average as i32;
    }

    pub fn update(
        &mut self,
        accel_now: f32,
        dt_us: u32,
        barometer: &BarometerState,
        new_baro_sample: bool,
    ) {
        let dt_seconds = dt_us as f32 * 1.0e-6;
        self.accel_now = accel_now;

        if accel_now > self.max_accel {
            self.max_accel = accel_now;
        }

        let delta_velocity = accel_now * dt_seconds;
        self.accel_vel += delta_velocity;
        self.accel_alt += self.accel_vel * dt_seconds;

        self.fusion_vel += delta_velocity;

        if new_baro_sample
            && barometer.vel < 300.0
            && barometer.vel < barometer.max_vel
            && accel_now < 0.2
            && self.fusion_vel < 300.0
            && barometer.alt < 13_000.0
        {
            self.fusion_vel = 0.99 * self.fusion_vel + 0.01 * barometer.vel;
        }

        if self.fusion_vel > self.max_velocity {
            self.max_velocity = self.fusion_vel;
        }

        self.fusion_alt += self.fusion_vel * dt_seconds;

        if new_baro_sample && barometer.alt < 9_000.0 {
            self.fusion_alt = 0.95 * self.fusion_alt + 0.05 * barometer.alt;
        }
    }
}
