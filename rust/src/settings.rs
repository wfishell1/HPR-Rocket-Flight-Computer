pub const N: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AirstartTrigger {
    Ignition,
    Burnout,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirstartConfig {
    pub trigger: AirstartTrigger,
    pub delay_us: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlightProfile {
    Single,
    TwoStage { booster_separation_delay_us: u32, sustainer_fire_delay_us: u32 },
    AirStart {
        booster_separation_delay_us: u32,
        airstart1: AirstartConfig,
        airstart2: AirstartConfig,
    },
    Booster,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PyroRole {
    Apogee,
    Main,
    BoosterSeparation,
    SustainerIgnition,
    Airstart1,
    Airstart2,
    None,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RadioEvent {
    Preflight = 0,
    Liftoff = 1,
    BoosterBurnout = 2,
    Apogee = 3,
    FireApogee = 4,
    Separation = 5,
    FireMains = 6,
    UnderChute = 7,
    EjectBooster = 8,
    FireSustainer = 9,
    SustainerIgnition = 10,
    SustainerBurnout = 11,
    FireAirstart1 = 12,
    Airstart1Ignition = 13,
    Airstart1Burnout = 14,
    FireAirstart2 = 15,
    Airstart2Ignition = 16,
    Airstart2Burnout = 17,
    NoFireRotnLimit = 18,
    NoFireAltLimit = 19,
    NoFireRotnAltLimit = 20,
    BoosterApogee = 21,
    FireBoosterApogee = 22,
    BoosterSeparation = 23,
    FireBoosterMains = 24,
    BoosterUnderChute = 25,
    TimeLimit = 26,
    Touchdown = 27,
    PowerLossRestart = 28,
    BoosterTouchdown = 29,
    BoosterPreflight = 30,
    BoosterTimeLimit = 31,
    BoosterPowerLoss = 32,
}

#[derive(Clone, Copy, Default)]
pub struct SensorInputs{
    pub flight_time_us: u32,
    pub accel_z: i16,
    pub accel_now: f32,
    pub accel_vel: f32,
    pub accel_alt: f32,
    pub fusion_vel: f32,
    pub baro_alt: f32,
    pub baro_base_alt: f32,
    pub baro_vel: f32,
    pub baro_touchdown: u8,
    pub alt_ok: bool,
    pub rotn_ok: bool,
}

// Flight settings, fixed for the whole flight.
// This is set in the setting.txt file in main code originally
//setting it as a struct to avoid on the fly parsing at runtim
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    pub rcd_time_us: u32,
    pub max_angle_tenths: i64,
    pub main_deploy_alt_m: f32,
    pub apogee_delay_us: u32,
    pub fire_time_us: u32,
    pub counts_per_g: i16,
    pub g_trigger_counts: i16,
    pub detect_liftoff_time_us: u32,
    pub threshold_vel_mps: f32,
    pub threshold_alt_m: f32,
    pub clear_rail_time_us: u32,
    pub touchdown_trigger: u8,
    pub flight_profile: FlightProfile,
    pub pyro_roles: [PyroRole; N],
    pub test_mode: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            rcd_time_us: 900_000_000,
            max_angle_tenths: 200,
            main_deploy_alt_m: 200.0,      
            // harness --main-ft 656.16, for simujlation tranlsation we use ft
            //however the computer uses meters THIS NEEDS TO BE UPDATED
            apogee_delay_us: 0,
            fire_time_us: 500_000, 
            counts_per_g: 2048,
            g_trigger_counts: 5120,
            detect_liftoff_time_us: 500_000,
            threshold_vel_mps: 35.0,
            threshold_alt_m: 61.0,
            clear_rail_time_us: 250_000,
            touchdown_trigger: 5,
            flight_profile: FlightProfile::Single,
            pyro_roles: [PyroRole::None, PyroRole::None, PyroRole::Apogee, PyroRole::Main],
            test_mode: false,
        }
    }
}

impl Settings {
    // Upstream: two-stage burp window follows separation delay; other profiles use 100 ms.
    pub fn booster_burp_time_us(&self) -> u32 {
        match self.flight_profile {
            FlightProfile::TwoStage { booster_separation_delay_us, .. } => {
                1_000_000.min(booster_separation_delay_us.saturating_sub(10_000))
            }
            _ => 100_000,
        }
    }
}
