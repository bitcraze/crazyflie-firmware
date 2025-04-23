#![no_std]

use panic_halt as _;

mod ffi;
mod types;
mod controller;
mod util;
mod pid_constants;

use crate::controller::run_my_controller;
use crate::types::{Setpoint, SensorData, State};
// use crate::ffi::controllerPid;

#[no_mangle]
pub extern "C" fn appMain() -> i32 {
    util::console_print("Hello from Rust!\n");
    loop {
        unsafe { ffi::vTaskDelay(1000); }
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTreeInit() {
    // controller::init();
}

#[no_mangle]
pub extern "C" fn controllerOutOfTreeTest() -> bool {
    true
}

#[no_mangle]
pub extern "C" fn controllerOutOfTree(
    control_ptr: *mut ffi::control_t,
    setpoint_ptr: *const ffi::setpoint_t,
    sensors_ptr: *const ffi::sensorData_t,
    state_ptr: *const ffi::state_t,
    tick: u32,
) {
    let setpoint = unsafe { Setpoint::from_raw(setpoint_ptr) };
    let sensors = unsafe { SensorData::from_raw(sensors_ptr) };
    let state = unsafe { State::from_raw(state_ptr) };

    let result = run_my_controller(&setpoint, &sensors, &state, tick);

    // Write output back to control_ptr
    unsafe { result.write_to_raw(control_ptr) };
}