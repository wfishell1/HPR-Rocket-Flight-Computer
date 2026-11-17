use crate::settings::Settings;
use crate::speed_trig::{speed_arc_sin,speed_atan2, speed_arc_tan, speed_tan};
//THIS MIGHT NEED TO BE MOVED SOMEWHERE ELSE
const ROTATION_FAULT_TENTHS: i64 = 450;
const ROTATION_INTERVAL_US: u32 = 10_000;

////////

pub struct rotation_info{
    dx: i64,
    dy: i64,
    dz: i64,
    last_rotation_us: u32,
    quat: [f64; 5],
    prev_roll_z: i64,
    quat_roll_z: i64,
    full_roll_z: i64,
    rotnOK: bool,
    rotations_fault: bool,
    max_angle_tenths: i64,
}
impl rotation_info{
    pub fn new(settings: Settings)->Self{
        rotation_info{
            dx: 0,
            dy: 0,
            dz: 0,
            last_rotation_us: 0,
            quat: [0.0,1.0,0.0,0.0,0.0],
            prev_roll_z: 0,
            quat_roll_z: 0,
            full_roll_z: 0,
            rotnOK: true,
            rotations_fault: false,
            max_angle_tenths: settings.max_angle_tenths,
        }
    }
    pub fn accumulate(&mut self, gyro: [i32; 3], dt_us: u32) {
        self.dx += i64::from(gyro[0]) * i64::from(dt_us);
        self.dy += i64::from(gyro[1]) * i64::from(dt_us);
        self.dz += i64::from(gyro[2]) * i64::from(dt_us);
    }

    pub fn update(&mut self, flight_time_us: u32, gyro_gain: f64) {
        if flight_time_us.wrapping_sub(self.last_rotation_us) > ROTATION_INTERVAL_US {
            self.getQuatRotn(self.dx as f64, self.dy as f64, self.dz as f64, gyro_gain);
            self.dx = 0;
            self.dy = 0;
            self.dz = 0;
            self.last_rotation_us = flight_time_us;
        }
    }

    pub fn getQuatRotn(&mut self, mut dx: f64, mut dy: f64, mut dz: f64, gyro_gain: f64){
        let q = &mut self.quat;
        let mut quat_diff=[0.0f64; 5];
        let mut rotn1=[0.0f64; 4];
        let mut rotn2=[0.0f64; 4];
        let mut rotn3=[0.0f64; 4];
        let mut quat_len: f64;
        let (
        mut _a, mut _b, mut _c, mut _d,
        mut _a2, mut _b2, mut _c2, mut _d2,
        mut _ab, mut _ac, mut _ad, mut _bc, mut _bd, mut _cd
        ): (f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64);
        
        let rotn_2_rad = gyro_gain * core::f64::consts::PI / 1000000.0;

        //convert to rads 
        dx *=rotn_2_rad;
        dy*= rotn_2_rad;
        dz *= rotn_2_rad;

        //compute quaternion derivative
        //wtf is a quaternion derivative? def: The time derivative of a unit orientation quaternion \(\mathbf{q}\) relates the rate of change of orientation to the angular velocity vector \(\mathbfit{\omega }\) v copied from google
        quat_diff[1]= 0.5 * (-1.0 * dx *q[2] -dy * q[3] - dz * q[4]);
        quat_diff[2] = 0.5 * (dx*q[1] - dy * q[4] + dz * q[3]);
        quat_diff[3]= 0.5 *( dx * q[4] + dy * q[1] - dz* q[2]);
        quat_diff[4]= 0.5 * (-1.0 * dx * q[3] + dy * q[2] + dz * q[1]);

        //Update the quaternion
        q[1]+= quat_diff[1];
        q[2]+= quat_diff[2];
        q[3]+= quat_diff[3];
        q[4]+= quat_diff[4];
        
        //re nomralize
        //note originally had self.quat but AI helped consolidate this here. 

        quat_len = 1.0 / libm::sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3] + q[4]*q[4]);
        q[1]+= quat_len;
        q[2]+= quat_len;
        q[3]+= quat_len;
        q[4]+= quat_len;
        

        _a = q[1];
        _b=q[2];
        _c = q[3];
        _d = q[4];
        _a2 = _a * _a;
        _b2 = _b * _b;
        _c2 = _c * _c;
        _d2 = _d * _d;
        _ab = _a * _b;
        _ac = _a * _c;
        _ad = _a * _d;
        _bc = _b * _c;
        _bd = _b * _d;
        _cd = _c * _d;

        //compiuting roation matrix 
        rotn1[1]= _a2+ _b2 - _c2 -_d2;
        rotn2[1] = 2.0*(_bd+_ad);
        rotn3[1] = 2.0*(_bd-_ac);
        rotn3[2]= 2.0* (_cd +_ab);
        rotn3[3] = _a2-_b2-_c2+_d2;
         //compute 3D orientation returns 10th of a degree as i64
        let pitch_x: i64 = speed_atan2(rotn3[2],rotn3[3]);
        let yaw_y: i64 = speed_arc_sin(-1.0*rotn3[1]);

        self.prev_roll_z=self.quat_roll_z;
        self.quat_roll_z= speed_atan2(rotn2[1],rotn1[1]);

        if self.quat_roll_z - self.prev_roll_z > 1800{
            self.full_roll_z-=1;
        }
        else if self.quat_roll_z - self.prev_roll_z < -1800 {
            self.full_roll_z+=1;
        }

        //compute angle off vertical
        let tan_yaw: f64 = speed_tan(yaw_y);
        let tan_pitch: f64 = speed_tan(pitch_x);
        let hyp1: f64 = tan_yaw * tan_yaw + tan_pitch*tan_pitch;
        let hyp2: f64 = libm::sqrt(hyp1);
        let offvert: i64 = speed_arc_tan(hyp2);

        self.rotnOK = !self.rotations_fault && offvert < self.max_angle_tenths;
        if !self.rotations_fault && offvert> self.max_angle_tenths{
            self.rotations_fault = true;
            self.rotnOK= false;
        }
    }
    //getting rid of the cheap get DCM2DRotn for now it is less accurfate but slightly cheaper.
    // CAN REIMPLEMENT IF WHEN WE GET HARDWARE THIS PROVES PROBLEMANTIC.
}
