use crate::event_logic::Phase;

fn update_average<const N: usize>(buffer: &mut [f32; N], sum: f32, index: usize, sample: f32) -> (f32, f32, usize) {
    assert!(N > 0);
    let new_sum = sum - buffer[index] + sample;
    buffer[index] = sample;
    let next_index = (index + 1) % N;

    return (new_sum / N as f32, new_sum, next_index);
}

struct AltitudeHistory<const N: usize> {
    buffer: [f32; N],
    sum: f32,
    index: usize,
}

impl<const N: usize> AltitudeHistory<N> {
    fn new() -> Self {
        assert!(N > 0);
        return Self {
            buffer: [0.0; N],
            sum: 0.0,
            index: 0,
        };
    }

    fn update(&mut self, sample: f32) -> f32 {
        let average;
        (average, self.sum, self.index) = update_average(&mut self.buffer, self.sum, self.index, sample);
        return average;
    }
}

struct VelocityHistory<const N: usize> {
    altitudes: [f32; N],
    timestamps_us: [u32; N],
    index: usize,
    updated_index: usize
}

impl<const N: usize> VelocityHistory<N> {
    fn new() -> Self {
        assert!(N > 0);
        return Self {
            altitudes: [0.0; N],
            timestamps_us: [0; N],
            index: 0,
            updated_index:0
        };
    }
    fn update(&mut self, smooth_alt:f32, _sample_time_us: u32) -> f32 {
        let elapsed_us =
            _sample_time_us.wrapping_sub(self.timestamps_us[self.updated_index]);
        let elapsed_s = elapsed_us as f32 * 1e-6;
        let rV = (smooth_alt - self.altitudes[self.updated_index]) / elapsed_s;
        self.altitudes[self.index] = smooth_alt;
        self.timestamps_us[self.index] = _sample_time_us;
        self.index = (self.index + 1) % N;
        return rV;
    }
    fn update_index(&mut self, phase : Phase) {
        if matches!(phase, Phase::Apogee) {
                self.updated_index = self.index;
        }
        else {
            self.updated_index = (N + self.index - 10) % N;
        }

    }
}


pub struct BarometerState {
    base_alt: f32,
    alt: f32,
    smooth_alt: f32,
    vel: f32,
    new_samp: bool,
    max_vel: f32,
    max_alt: f32,

    baseline_history: AltitudeHistory<30>,
    altitude_history: AltitudeHistory<10>,
    velocity_history: VelocityHistory<30>,
}

impl BarometerState {
    pub fn new() -> Self {
        Self {
            //these top fields are from the original baro struct
            base_alt: 10.0,
            alt: 0.0,
            smooth_alt: 0.0,
            vel: 0.0,
            new_samp: false,
            max_vel: 0.0,
            max_alt: 0.0,
            //these bottom fields are static variables from the process_sample method though I don't
            //see any reason why they shouldn't just be added as fields here
    

            baseline_history: AltitudeHistory::new(),
            altitude_history: AltitudeHistory::new(),
            velocity_history: VelocityHistory::new(),
        }
    }
    //maybe establishing invariants around what base_alt_index could be 
    pub fn process_sample(&mut self, raw_altitude_m: f32, _sample_time_us:u32, phase: Phase) {
        if matches!(phase, Phase::Pad | Phase::Landed | Phase::TimedOut){
            self.base_alt = self.baseline_history.update(raw_altitude_m);
            //here the normal code updates the radio struct based on the state of the barometer
            //struct
            //Maybe the telemetry part of the code can handle this, and this part of the code
            //should be focused on updating the barometer state and nothing else
            self.new_samp = false;
        }

        self.alt = raw_altitude_m - self.base_alt;
        if (self.alt > self.max_alt && ! (matches!(phase, Phase::Apogee))) {
            self.max_alt = self.alt;
        }
        self.smooth_alt = self.altitude_history.update(self.alt);

        //Velocity updates go here
        self.velocity_history.update_index(phase);
        self.vel = self.velocity_history.update(self.smooth_alt, _sample_time_us);
        if (self.vel > self.max_vel) {
            self.max_vel = self.vel;
        }
        
        



    }

}
















