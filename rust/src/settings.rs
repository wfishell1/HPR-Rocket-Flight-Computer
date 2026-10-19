// Flight settings, fixed for the whole flight.
// This is set in the setting.txt file in main code originally
//setting it as a struct to avoid on the fly parsing at runtim
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    pub rcd_time_us: u32,
    pub main_deploy_alt_m: f32,
    pub apogee_delay_us: u32,
    pub fire_time_us: u32,
    pub counts_per_g: i16,
    pub g_trigger_counts: i16,
    pub detect_liftoff_time_us: u32,
    pub threshold_vel_mps: f32,
    pub threshold_alt_m: f32,
    pub clear_rail_time_us: u32,
    pub booster_burp_time_us: u32,
    pub touchdown_trigger: u8,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            rcd_time_us: 900_000_000,
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
            booster_burp_time_us: 100_000,
            touchdown_trigger: 5,
        }
    }
}
