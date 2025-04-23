// Define the C types used in the FFI calls
#[repr(C)]
pub struct control_t {
    pub _union_data: [u8; 16], // 16 bytes for the union
    pub control_mode: u32,     // 4 bytes
}

#[repr(C)]
pub struct vector3_t {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
pub struct quaternion_t {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

pub type stab_mode_t = u8;

#[repr(C)]
pub struct setpoint_mode_t {
    pub x: stab_mode_t,
    pub y: stab_mode_t,
    pub z: stab_mode_t,
    pub roll: stab_mode_t,
    pub pitch: stab_mode_t,
    pub yaw: stab_mode_t,
    pub quat: stab_mode_t,
}

#[repr(C)]
pub struct setpoint_t {
    pub timestamp: u32,
    pub attitude: vector3_t,
    pub attitudeRate: vector3_t,
    pub attitudeQuaternion: quaternion_t,
    pub thrust: f32,
    pub position: vector3_t,
    pub velocity: vector3_t,
    pub acceleration: vector3_t,
    pub jerk: vector3_t,
    pub velocity_body: bool,
    pub mode: setpoint_mode_t,
}

extern "C" {
    pub fn vTaskDelay(ticks: u32);
    pub fn consolePutchar(ch: i32) -> i32;

    // pub fn controllerPidInit();
    // pub fn controllerPid(
    //     control: *mut control_t,
    //     setpoint: *const setpoint_t,
    //     sensors: *const sensorData_t,
    //     state: *const state_t,
    //     tick: u32,
    // );
}

pub type baro_t = f32;

#[repr(C)]
pub struct sensorData_t {
    pub acc: vector3_t,
    pub gyro: vector3_t,
    pub mag: vector3_t,
    pub baro: baro_t,
    // Optional: include these only if LOG_SEC_IMU is enabled
    // pub accSec: vector3_t,
    // pub gyroSec: vector3_t,
    pub interruptTimestamp: u64,
}

#[repr(C)]
pub struct state_t {
    pub attitude: vector3_t,           // deg
    pub attitudeQuaternion: quaternion_t,
    pub position: vector3_t,           // m
    pub velocity: vector3_t,           // m/s
    pub acc: vector3_t,                // Gs (Z without gravity)
}
