use crate::settings::Settings;
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
//states the flight can be in. 
//traking stateful approach
pub enum Phase{
    #[default]
    Pad,
    BoostUnconfirmed,
    Boost,
    BurnoutCheck,
    Coast,
    Apogee,
    Drogue,
    Main,
    Landed,
    TimedOut,
}

pub struct SensorInputs{
    pub flight_time_us: u32,
    pub accel_z: i16,
    pub accel_now: f32,
    pub accel_vel: f32,
    pub fusion_vel: f32,
    pub baro_alt: f32,
    pub baro_vel: f32,
    pub baro_touchdown: u8,
    pub alt_ok: bool,
    pub rotn_ok: bool,
}
//number of pyros in the rocket
//Needs to be updated 
const N: usize = 4; 

pub struct Flight{
    pub flight_phase: Phase,
    pub pyroN: [(bool, u32); N],
    pub syncApogee: bool,
    pub syncMains: bool,
    pub settings: Settings,
    pub flight_end_time_us: Option<u32>,
}
impl Flight{
    /// A new flight sitting on the pad with every pyro off.
    pub fn new(settings: Settings) -> Self {
        Flight {
            flight_phase: Phase::Pad,
            pyroN: [(false, 0); N],
            syncApogee: false,
            syncMains: false,
            settings,
            flight_end_time_us: None,
        }
    }
    fn pyro_global(&self)-> bool{
        for pyro in self.pyroN{
            if pyro.0{
                return true;
            }
        }
        false
    }
    pub fn timeout_check(&mut self, current_input: &SensorInputs){
        let pyro: bool = self.pyro_global();
        if pyro==false && !matches!(self.flight_phase, Phase::Pad | Phase::Landed | Phase::TimedOut) 
        && current_input.flight_time_us > self.settings.rcd_time_us{
            self.flight_phase=Phase::TimedOut;
            self.flight_end_time_us = Some(current_input.flight_time_us);
        }
    }
}


pub fn past_apogee(velocity_mps: f32) -> bool {
    velocity_mps < 0.0
}
pub fn fib(mut val: usize) -> usize {
    if val == 0 || val == 1{
        return val;
    }
    else{ 
        let new_val: i64 = 10;
        let r: &i64 = &new_val;   // r points at new_val
        let copy: i64 = *r;
        val=fib(val-1) + fib(val-2);
    }
    val
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prints_and_checks_apogee() {
        let velocity = -2.5;
        let result = past_apogee(velocity);
        println!("velocity = {velocity} m/s, past apogee = {result}");
        assert!(result);
    }
    #[test]
    fn check_fib(){
        let fib_vals=[0,1,1,2,3,5,8,13,21];
        for i in 0..fib_vals.len(){
            println!("fib val is {} and index is {i}", fib_vals[i]);
            let result=fib(i);
            assert!(result==fib_vals[i]);
        }
    }
}
