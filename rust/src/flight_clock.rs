pub struct FlightClock {
    liftoff_time_us: Option<u32>,
    previous_update_us: Option<u32>,
    pub flight_time_us: u32,
}

impl FlightClock {
    pub fn new() -> Self {
        Self {
            liftoff_time_us: None,
            previous_update_us: None,
            flight_time_us: 0,
        }
    }

    pub fn start(&mut self, now_us: u32) {
        self.liftoff_time_us = Some(now_us);
        self.previous_update_us = Some(now_us);
        self.flight_time_us = 0;
    }

    pub fn advance(&mut self, now_us: u32) -> Option<u32> {
        let previous_update_us = self.previous_update_us?;
        let dt_us = now_us - previous_update_us;
        self.previous_update_us = Some(now_us);
        self.flight_time_us += dt_us;
        Some(dt_us)
    }
}
