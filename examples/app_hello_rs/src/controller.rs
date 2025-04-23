use crate::types::{Control, ControlLegacy, ControlMode, Matrix3x3, Vector3, SensorData, Setpoint, State};
use crate::util::{console_print};
// use defmt::{debug, info, warn};
use crate::pid_constants::{PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD,
    PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
    PID_YAW_KP, PID_YAW_KI, PID_YAW_KD};
// use pid_constants::*;

pub fn run_my_controller(
    setpoint: &Setpoint,
    _sensors: &SensorData,
    _state: &State,
    _tick: u32,
) -> Control {
    // define some gains


    // let's implement attitude control first
    let current_attitude = _state.attitude;
    let target_attitude = setpoint.attitude;
    let attitude_error: Vector3 = match target_attitude - current_attitude {
        Ok(value) => value,
        Err(_) => {
            panic!("Failed to calculate attitude error");
        }
    };
    // info!("Vector3: x = {}, y = {}, z = {}", attitude_error.x, attitude_error.y, attitude_error.z);
    let attitude_gains: Matrix3x3 = Matrix3x3::new(
        Vector3::new(
            PID_ROLL_KP, PID_PITCH_KP, PID_YAW_KP,
        ),
        Vector3::new(
            PID_ROLL_KI, PID_PITCH_KI, PID_YAW_KI,
        ),
        Vector3::new(
            PID_ROLL_KD, PID_PITCH_KD, PID_YAW_KD,
        ),
    );

    // calculate the control output
    let control_output: Vector3 = match attitude_gains * attitude_error {
        Ok(value) => value,
        Err(_) => {
            panic!("Failed to calculate control output");
        }
    };


    let legacy = ControlLegacy {
        roll: control_output.x as i16,
        pitch: control_output.y as i16,
        yaw: control_output.z as i16,
        thrust: 0 as f32
    };

    Control {
        mode: ControlMode::Legacy,
        legacy: Some(legacy),
        si: None,
        normalized_forces: None,
    }
}
