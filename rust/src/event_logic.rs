use crate::settings::{AirstartTrigger, FlightProfile, PyroRole, Settings, N, SensorInputs,RadioEvent};
use log::{debug};
//states the flight can be in. 
//traking stateful approach
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
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

//Motor Stage 
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MotorStage {
    fire_check_time: Option<u32>,
    fire_time: Option<u32>,
    ignition_time: Option<u32>,
    burnout_check_time: Option<u32>,
    burnout_time: Option<u32>,
}

//Stage rockets
//FUTURE SHOULD ASSERT EQUIVLANCY BETWEN THE STAGESTAGE AND THE SETTINGS FLIGHT PROFILE
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StageState {
    Single,                                                                  // nothing
    TwoStage { booster_separation_time: Option<u32>, sustainer: MotorStage },
    AirStart { airstart1: MotorStage, airstart2: MotorStage },
    Booster,
}
//for comparing and extracting ouptut for validating equivlance
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TickOutput{
    pub event: Option<RadioEvent>,
    pub sync_sd: bool,
    pub reset: bool,
    pub pyros: [bool; N],
}

#[derive(Clone, Copy)]
pub struct Flight{
    pub flight_phase: Phase,
    stages: StageState,
    pub pyro_n: [(bool, u32); N],
    pub sync_apogee: bool,
    pub sync_mains: bool,
    pub settings: Settings,
    pub flight_end_time_us: Option<u32>,
    sensor_history:  [SensorInputs;2],
    inflight_recovery: u32,
    burnout_time: u32,
    apogee_time: Option<u32>,
    apogee_fire_time: Option<u32>,
    main_deploy_time: Option<u32>,
}
impl Flight{
    /// A new flight sitting on the pad with every pyro off.
    pub fn new(settings: Settings) -> Self {
        Flight {
            flight_phase: Phase::Pad,
            stages: match settings.flight_profile{
                FlightProfile::Single => StageState::Single,
                FlightProfile::Booster => StageState::Booster,
                FlightProfile::TwoStage{ .. } => StageState::TwoStage{
                    booster_separation_time: None,
                    sustainer: MotorStage::default()
                },
                FlightProfile::AirStart{ .. } => StageState::AirStart{
                    airstart1: MotorStage::default(),
                    airstart2: MotorStage::default(),
                },

            },
            pyro_n: [(false, 0); N],
            sync_apogee: false,
            sync_mains: false,
            settings,
            flight_end_time_us: None,
            sensor_history: [SensorInputs::default(); 2],
            inflight_recovery: 0,
            burnout_time: 0,
            apogee_time: None,
            apogee_fire_time: None,
            main_deploy_time: None,
        }
    }
    //global pyro check
    fn pyro_global(&self)-> bool{
        for pyro in self.pyro_n{
            if pyro.0{
                return true;
            }
        }
        false
    }
    //check the timeout logic for flying
    //need to add the GPS logic which will be added in another struct
    //need adequate testing here
    pub fn timeout_check(&mut self) -> Option<RadioEvent>{
        let pyro: bool = self.pyro_global();
        if pyro==false && !matches!(self.flight_phase, Phase::Pad | Phase::Landed | Phase::TimedOut) 
        && self.sensor_history[0].flight_time_us > self.settings.rcd_time_us{
            self.flight_phase=Phase::TimedOut;
            self.flight_end_time_us = Some(self.sensor_history[0].flight_time_us);
            //booster profile sends its own event
            if self.settings.flight_profile == FlightProfile::Booster{
                return Some(RadioEvent::BoosterTimeLimit);
            }
            return Some(RadioEvent::TimeLimit);
        }
        None
    }
    //liftoff detection
    //flight_time_us is expected to count from this liftoff (fltTime.liftoff), the caller zeroes it
    pub fn liftoff_check(&mut self) -> Option<RadioEvent>{
        if self.flight_phase==Phase::Pad && self.sensor_history[0].accel_z > self.settings.g_trigger_counts{
            self.flight_phase=Phase::BoostUnconfirmed;
            return Some(RadioEvent::Liftoff);
        }
        None
    }
    pub fn update_current_info(&mut self, current_input: &SensorInputs){
        self.sensor_history[1]=self.sensor_history[0];
        self.sensor_history[0]=*current_input;
    }
    pub fn false_trigger(&mut self) -> Option<RadioEvent>{
        if self.flight_phase==Phase::BoostUnconfirmed{
            if self.sensor_history[0].flight_time_us > self.settings.detect_liftoff_time_us
            || self.sensor_history[0].accel_vel> self.settings.threshold_vel_mps
            || self.sensor_history[0].accel_alt > self.settings.threshold_alt_m {
                self.flight_phase=Phase::Boost;
            }
            if self.sensor_history[0].accel_z < self.settings.g_trigger_counts
            && self.sensor_history[0].accel_vel < self.settings.threshold_vel_mps
            && self.sensor_history[0].flight_time_us > self.settings.clear_rail_time_us {
                if self.settings.test_mode{
                    debug!("False Trigger Reset");
                }
                *self = Flight::new(self.settings);
                //booster profile sends its own event
                if self.settings.flight_profile == FlightProfile::Booster{
                    return Some(RadioEvent::BoosterPreflight);
                }
                return Some(RadioEvent::Preflight);
            }
        }
        None
    }
    pub fn booster_burnout(&mut self) -> Option<RadioEvent>{
        if self.flight_phase==Phase::Boost && self.sensor_history[0].accel_z<=0{
            self.flight_phase=Phase::BurnoutCheck;
            self.burnout_time=self.sensor_history[0].flight_time_us;
            return Some(RadioEvent::BoosterBurnout);

            }
        None
        }
    pub fn booster_motor_burp(&mut self) -> Option<RadioEvent>{
        if self.flight_phase==Phase::BurnoutCheck{
            if self.sensor_history[0].flight_time_us - self.burnout_time>self.settings.booster_burp_time_us(){
                self.flight_phase=Phase::Coast;
            }
            else if !self.settings.test_mode && self.sensor_history[0].accel_z>0{
                self.flight_phase=Phase::Boost;
                return Some(RadioEvent::Liftoff);
            }
        }
        None
        }
    fn fire_pyros(&mut self, role: PyroRole) {
        let now = self.sensor_history[0].flight_time_us;
        for i in 0..N {
            if self.settings.pyro_roles[i] == role {
                self.pyro_n[i] = (true, now);
            }
        }
    }
    pub fn two_stage_flight_check_fire_seperation_charge(&mut self) -> Option<RadioEvent>{
        if let (FlightProfile::TwoStage{booster_separation_delay_us, ..}, StageState::TwoStage{booster_separation_time, ..}) = (self.settings.flight_profile, &mut self.stages){
            if booster_separation_time.is_none() && self.flight_phase==Phase::Coast
            && self.sensor_history[0].flight_time_us - self.burnout_time > booster_separation_delay_us{
                *booster_separation_time=Some(self.sensor_history[0].flight_time_us);
            }
            else{
                return None;
            }
        }
        else{
            return None;
        }
        self.fire_pyros(PyroRole::BoosterSeparation); // firePyros('B')
        Some(RadioEvent::EjectBooster)
    }
    pub fn trigger(&mut self) -> Option<PyroRole>{
        //Single has no upper motor thus no trigger same for booster
        let now = self.sensor_history[0].flight_time_us;
        if let (FlightProfile::TwoStage{sustainer_fire_delay_us, ..},
            StageState::TwoStage{booster_separation_time, sustainer}) = (self.settings.flight_profile, &self.stages){
                if sustainer.fire_check_time.is_none() && (self.flight_phase < Phase::Apogee || self.settings.test_mode)
                && !self.pyro_global() && booster_separation_time.is_some()
                && now - booster_separation_time.unwrap() > sustainer_fire_delay_us{
                    return Some(PyroRole::SustainerIgnition);
                }
        }
        else if let (FlightProfile::AirStart{airstart1: as1_cfg, airstart2: as2_cfg, ..},
        StageState::AirStart{airstart1, airstart2}) = (self.settings.flight_profile, &self.stages){
            if airstart1.fire_check_time.is_none()
            && ((as1_cfg.trigger == AirstartTrigger::Ignition
                && self.flight_phase != Phase::BoostUnconfirmed
                && now > as1_cfg.delay_us)
            || (as1_cfg.trigger == AirstartTrigger::Burnout
                && self.flight_phase >= Phase::BurnoutCheck
                && now - self.burnout_time > as1_cfg.delay_us)){
                return Some(PyroRole::Airstart1);
            }
            if airstart2.fire_check_time.is_none() && self.flight_phase < Phase::Apogee
            && ((as2_cfg.trigger == AirstartTrigger::Ignition
                && airstart1.ignition_time.is_some()
                && now - airstart1.ignition_time.unwrap() > as2_cfg.delay_us)
            || (as2_cfg.trigger == AirstartTrigger::Burnout
                && airstart1.burnout_time.is_some()
                && now - airstart1.burnout_time.unwrap() > as2_cfg.delay_us)){
                return Some(PyroRole::Airstart2);
            }
        }
        None

    }

    pub fn fire_decision(&mut self) -> Option<RadioEvent>{
        //which motor is ready, None means not time yet (or Single/Booster)
        let role = match self.trigger(){
            Some(role) => role,
            None => return None,
        };
        let now = self.sensor_history[0].flight_time_us;
        let alt_ok = self.sensor_history[0].alt_ok;
        let rotn_ok = self.sensor_history[0].rotn_ok;
        //grab the motor that triggered and its fire event
        let (motor, fire_event) = match (role, &mut self.stages){
            (PyroRole::SustainerIgnition, StageState::TwoStage{sustainer, ..}) => (sustainer, RadioEvent::FireSustainer),
            (PyroRole::Airstart1, StageState::AirStart{airstart1, ..}) => (airstart1, RadioEvent::FireAirstart1),
            (PyroRole::Airstart2, StageState::AirStart{airstart2, ..}) => (airstart2, RadioEvent::FireAirstart2),
            _ => return None,
        };
        motor.fire_check_time = Some(now);
        if alt_ok && rotn_ok{
            motor.fire_time = Some(now);
            self.fire_pyros(role); 
            return Some(fire_event);
        }
        else if !rotn_ok && !alt_ok{
            return Some(RadioEvent::NoFireRotnAltLimit);
        }
        else if !rotn_ok{
            return Some(RadioEvent::NoFireRotnLimit);
        }
        else{
            return Some(RadioEvent::NoFireAltLimit);
        }
    }
    pub fn ignition_check(&mut self) -> Option<RadioEvent>{
        let now = self.sensor_history[0].flight_time_us;
        let accel_now = self.sensor_history[0].accel_now;
        //motors this profile has and their ignition event, Single/Booster have none
        let motors: [Option<(&mut MotorStage, RadioEvent)>; 2] = match &mut self.stages{
            StageState::TwoStage{sustainer, ..} => [Some((sustainer, RadioEvent::SustainerIgnition)), None],
            StageState::AirStart{airstart1, airstart2} => [Some((airstart1, RadioEvent::Airstart1Ignition)), Some((airstart2, RadioEvent::Airstart2Ignition))],
            _ => [None, None],
        };
        for (motor, ignition_event) in motors.into_iter().flatten(){
            if self.flight_phase < Phase::Apogee && motor.fire_time.is_some() && motor.ignition_time.is_none() && accel_now > 10.0{
                motor.ignition_time = Some(now);
                return Some(ignition_event);
            }
        }
        None
    }
    pub fn burnout_check(&mut self) -> Option<RadioEvent>{
        let now = self.sensor_history[0].flight_time_us;
        let accel_now = self.sensor_history[0].accel_now;
        let accel_z = self.sensor_history[0].accel_z;
        let burp_time = self.settings.booster_burp_time_us();
        //motors this profile has, their burnout event, and the burp window (None = sustainer rule)
        let motors: [Option<(&mut MotorStage, RadioEvent, Option<u32>)>; 2] = match &mut self.stages{
            StageState::TwoStage{sustainer, ..} => [Some((sustainer, RadioEvent::SustainerBurnout, None)), None],
            StageState::AirStart{airstart1, airstart2} => [Some((airstart1, RadioEvent::Airstart1Burnout, Some(burp_time))), Some((airstart2, RadioEvent::Airstart2Burnout, Some(burp_time)))],
            _ => [None, None],
        };
        for (motor, burnout_event, burp_window) in motors.into_iter().flatten(){
            if motor.burnout_time.is_some() || motor.ignition_time.is_none(){
                continue;
            }
            match burp_window{
                None => {
                    if self.flight_phase < Phase::Apogee && accel_now < 0.0
                    && now - motor.ignition_time.unwrap() > 100_000{
                        motor.burnout_time = Some(now);
                        return Some(burnout_event);
                    }
                }
                //thrust inside the window cancels it, surviving the window is burnout
                Some(burp) => {
                    if motor.burnout_check_time.is_none() && self.flight_phase < Phase::Apogee && accel_now < 0.0{
                        motor.burnout_check_time = Some(now);
                    }
                    if let Some(t) = motor.burnout_check_time{
                        if now < t + burp && accel_z > 0{
                            motor.burnout_check_time = None;
                        }
                        else if now > t + burp{
                            motor.burnout_check_time = None;
                            motor.burnout_time = Some(t);
                            return Some(burnout_event);
                        }
                    }
                }
            }
        }
        None
    }
    pub fn apogee_functions(&self) -> [bool; 3]{
        let accel_apogee = self.sensor_history[0].accel_vel < -10.0;
        let baro_apogee = self.sensor_history[0].baro_vel < -10.0 && self.sensor_history[0].accel_vel<70.0 && (self.sensor_history[0].baro_alt+self.sensor_history[0].baro_base_alt) <9000.0;
        let fusion_apogee = self.sensor_history[0].fusion_vel<0.0;
        [accel_apogee, baro_apogee,fusion_apogee]
    }
    pub fn check_apogee_event(&mut self) -> Option<RadioEvent>{
        if self.flight_phase == Phase::Coast && !self.pyro_global() && self.apogee_functions().contains(&true){
            self.flight_phase = Phase::Apogee;
            self.apogee_time = Some(self.sensor_history[0].flight_time_us);
            if self.settings.flight_profile == FlightProfile::Booster{
                return Some(RadioEvent::BoosterApogee);
            }
            else{
                return Some(RadioEvent::Apogee);
            }
        }
        None
    }
    pub fn fire_apogee_charge(&mut self) -> Option<RadioEvent>{
        let now = self.sensor_history[0].flight_time_us;
        if self.flight_phase == Phase::Apogee && self.apogee_time.is_some()
        && now - self.apogee_time.unwrap() >= self.settings.apogee_delay_us{
            self.flight_phase = Phase::Drogue;
            self.apogee_fire_time = Some(now);
            self.fire_pyros(PyroRole::Apogee); // firePyros('A')
            if self.settings.flight_profile == FlightProfile::Booster{
                return Some(RadioEvent::FireBoosterApogee);
            }
            else{
                return Some(RadioEvent::FireApogee);
            }
        }
        None
    }
    pub fn sd_sync_check(&mut self) -> bool{
        let now = self.sensor_history[0].flight_time_us;
        let mut sync = false;
        if !self.settings.test_mode{
            if !self.sync_apogee && self.apogee_fire_time.is_some() && now - self.apogee_fire_time.unwrap() >= 3_000_000{
                self.sync_apogee = true;
                sync = true;
            }
            if !self.sync_mains && self.main_deploy_time.is_some() && now - self.main_deploy_time.unwrap() >= 3_000_000{
                self.sync_mains = true;
                sync = true;
            }
        }
        sync
    }
    pub fn apogee_separation_check(&self) -> Option<RadioEvent>{
        let now = self.sensor_history[0].flight_time_us;
        if self.flight_phase == Phase::Drogue && self.apogee_fire_time.is_some()
        && self.sensor_history[0].accel_z as i32 > 4 * self.settings.counts_per_g as i32
        && now - self.apogee_fire_time.unwrap() <= 2_000_000{
            if self.settings.flight_profile == FlightProfile::Booster{
                return Some(RadioEvent::BoosterSeparation);
            }
            else{
                return Some(RadioEvent::Separation);
            }
        }
        None
    }
    pub fn fire_main_charge(&mut self) -> Option<RadioEvent>{
        let now = self.sensor_history[0].flight_time_us;
        if self.flight_phase == Phase::Drogue && self.apogee_fire_time.is_some()
        && self.sensor_history[0].baro_alt < self.settings.main_deploy_alt_m
        && now - self.apogee_fire_time.unwrap() >= 1_000_000{
            self.flight_phase = Phase::Main;
            self.main_deploy_time = Some(now);
            self.fire_pyros(PyroRole::Main); // firePyros('M')
            if self.settings.flight_profile == FlightProfile::Booster{
                return Some(RadioEvent::FireBoosterMains);
            }
            else{
                return Some(RadioEvent::FireMains);
            }
        }
        None
    }
    pub fn under_chute_check(&self) -> Option<RadioEvent>{
        let now = self.sensor_history[0].flight_time_us;
        if self.main_deploy_time.is_some()
        && now - self.main_deploy_time.unwrap() > 50_000
        && now - self.main_deploy_time.unwrap() < 3_000_000
        && self.sensor_history[0].accel_now > 50.0{
            if self.settings.flight_profile == FlightProfile::Booster{
                return Some(RadioEvent::BoosterUnderChute);
            }
            else{
                return Some(RadioEvent::UnderChute);
            }
        }
        None
    }
    pub fn pyro_turn_off(&mut self){
        let now = self.sensor_history[0].flight_time_us;
        for pyro in self.pyro_n.iter_mut(){
            if pyro.0 && now - pyro.1 > self.settings.fire_time_us{
                pyro.0 = false;
            }
        }
    }
    pub fn touchdown_check(&mut self) -> Option<RadioEvent>{
        let now = self.sensor_history[0].flight_time_us;
        if self.flight_phase == Phase::Main && !self.pyro_global() && !self.settings.test_mode
        && self.sensor_history[0].baro_touchdown > self.settings.touchdown_trigger
        && self.sensor_history[0].baro_alt < 46.0{
            self.flight_phase = Phase::Landed;
            self.flight_end_time_us = Some(now);
            if self.settings.flight_profile == FlightProfile::Booster{
                return Some(RadioEvent::BoosterTouchdown);
            }
            else{
                return Some(RadioEvent::Touchdown);
            }
        }
        None
    }
    //single tick of check efvents
    pub fn check_events(&mut self, input: &SensorInputs) -> TickOutput{
        self.update_current_info(input);
        let mut out = TickOutput{event: None, sync_sd: false, reset: false, pyros: [false; N]};

        //liftoff detection runs before checkEvents() in the main loop (harness.cpp:343)
        if self.flight_phase == Phase::Pad{
            out.event = self.liftoff_check().or(out.event);
            if self.flight_phase == Phase::Pad{
                return out;
            }
        }
        if matches!(self.flight_phase, Phase::Landed | Phase::TimedOut){
            return out;
        }
        out.event = self.timeout_check().or(out.event);
        if let Some(event) = self.false_trigger(){
            out.event = Some(event);
            out.reset = true;
            return out;
        }
        out.event = self.booster_burnout().or(out.event);
        out.event = self.booster_motor_burp().or(out.event);
        out.event = self.two_stage_flight_check_fire_seperation_charge().or(out.event);
        out.event = self.fire_decision().or(out.event);
        out.event = self.ignition_check().or(out.event);
        out.event = self.burnout_check().or(out.event);
        out.event = self.check_apogee_event().or(out.event);
        out.event = self.fire_apogee_charge().or(out.event);
        out.sync_sd = self.sd_sync_check();
        out.event = self.apogee_separation_check().or(out.event);
        out.event = self.fire_main_charge().or(out.event);
        out.event = self.under_chute_check().or(out.event);
        self.pyro_turn_off();
        out.event = self.touchdown_check().or(out.event);

        for i in 0..N{
            out.pyros[i] = self.pyro_n[i].0;
        }
        out
    }
}

#[cfg(test)]
mod tests{
    use super::*;
    use std::collections::HashMap;
    use std::fs;

    fn openrocket_file(name: &str) -> String{
        format!("{}/../OpenRocket/{}", env!("CARGO_MANIFEST_DIR"), name)
    }

    fn replay(trace_csv: &str, events_csv: &str, settings: Settings){
        let trace = fs::read_to_string(openrocket_file(trace_csv)).expect("missing trace csv");
        let mut lines = trace.lines();
        let header: Vec<&str> = lines.next().unwrap().split(',').collect();
        let col: HashMap<&str, usize> = header.iter().enumerate().map(|(i, h)| (*h, i)).collect();

        let mut flight = Flight::new(settings);
        let mut current: u8 = RadioEvent::Preflight as u8; //radio.event keeps its value until overwritten
        let mut rust_events: Vec<(String, u8)> = Vec::new();

        for (row, line) in lines.enumerate(){
            let f: Vec<&str> = line.split(',').collect();
            let num = |name: &str| -> f32 { f[col[name]].parse().unwrap() };
            let input = SensorInputs{
                flight_time_us: (f[col["flight_time_s"]].parse::<f64>().unwrap() * 1e6).round() as u32,
                accel_z: num("accel_z_counts") as i16,
                accel_now: num("accelNow"),
                accel_vel: num("accelVel"),
                accel_alt: num("accelAlt"),
                fusion_vel: num("fusionVel"),
                baro_alt: num("baro_Alt"),
                baro_vel: num("baro_Vel"),
                baro_base_alt: num("baro_baseAlt"),
                baro_touchdown: num("baroTouchdown") as u8,
                alt_ok: f[col["altOK"]] == "1",
                rotn_ok: f[col["rotnOK"]] == "1",
            };
            let out = flight.check_events(&input);
            if let Some(event) = out.event{
                if event as u8 != current{
                    current = event as u8;
                    rust_events.push((f[col["time_s"]].to_string(), current));
                }
            }
            let expected: u8 = f[col["radio_event"]].parse().unwrap();
            assert_eq!(current, expected,
                "radio_event mismatch at trace row {} (time_s {}, phase {:?}): rust {} cpp {}",
                row + 2, f[col["time_s"]], flight.flight_phase, current, expected);
        }

        let events = fs::read_to_string(openrocket_file(events_csv)).expect("missing events csv");
        let cpp_events: Vec<(String, u8)> = events.lines()
            .map(|l| l.split(',').collect::<Vec<&str>>())
            .filter(|f| f.len() >= 5 && f[2] == "RADIO_EVENT")
            .map(|f| (f[0].to_string(), f[4].parse().unwrap()))
            .collect();
        println!("rust events: {:?}", rust_events);
        assert_eq!(rust_events, cpp_events);
    }

    //harness run: --profile S --main-ft 656.16
    #[test]
    fn replay_single_stage(){
        let settings = Settings{
            main_deploy_alt_m: 656.16 / 3.2808,
            ..Settings::default()
        };
        replay("Single_Stage_High_Powered_Rocket_Test_sensors_cpp_trace.csv",
               "Single_Stage_High_Powered_Rocket_Test_sensors_cpp_events.csv", settings);
    }

    //harness run: --profile 2 (default --main-ft 700), harness.cpp:242-258
    #[test]
    fn replay_two_stage(){
        let settings = Settings{
            main_deploy_alt_m: 700.0 / 3.2808,
            flight_profile: FlightProfile::TwoStage{booster_separation_delay_us: 500_000, sustainer_fire_delay_us: 500_000},
            pyro_roles: [PyroRole::BoosterSeparation, PyroRole::SustainerIgnition, PyroRole::Apogee, PyroRole::Main],
            ..Settings::default()
        };
        replay("2_Stage_High_Powered_Rocket_Test_sensors_cpp_trace.csv",
               "2_Stage_High_Powered_Rocket_Test_sensors_cpp_events.csv", settings);
    }

    #[test]
    fn liftoff_detected_above_g_trigger(){

    }

    #[test]
    fn check_flight_profile_and_stage_state_eq(){

    }
    // need to come up with more tests am tired atm will write later.
}
