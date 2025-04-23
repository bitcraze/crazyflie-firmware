use crate::ffi;
use core::fmt;

#[derive(Debug, Copy, Clone)]
pub enum ControlMode {
    Legacy,
    ForceTorque,
    Force,
    Unknown(u32),
}

#[derive(Debug, Copy, Clone)]
pub struct ControlLegacy {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
    pub thrust: f32,
}

#[derive(Debug, Copy, Clone)]
pub struct ControlSI {
    pub thrust_si: f32,
    pub torque: [f32; 3],
}

#[derive(Debug, Copy, Clone)]
pub struct Control {
    pub mode: ControlMode,
    pub legacy: Option<ControlLegacy>,
    pub si: Option<ControlSI>,
    pub normalized_forces: Option<[f32; 4]>,
}

impl Control {
    pub unsafe fn from_raw(ptr: *const ffi::control_t) -> Self {
        if ptr.is_null() {
            return Control {
                mode: ControlMode::Unknown(0),
                legacy: None,
                si: None,
                normalized_forces: None,
            };
        }

        let raw = &*ptr;

        let mode = match raw.control_mode {
            0 => ControlMode::Legacy,
            1 => ControlMode::ForceTorque,
            2 => ControlMode::Force,
            other => ControlMode::Unknown(other),
        };

        match mode {
            ControlMode::Legacy => {
                let legacy_ptr = ptr as *const LegacyLayout;
                let legacy = &*legacy_ptr;
                Control {
                    mode,
                    legacy: Some(ControlLegacy {
                        roll: legacy.roll,
                        pitch: legacy.pitch,
                        yaw: legacy.yaw,
                        thrust: legacy.thrust,
                    }),
                    si: None,
                    normalized_forces: None,
                }
            }
            ControlMode::ForceTorque => {
                let si_ptr = ptr as *const SIForceLayout;
                let si = &*si_ptr;
                Control {
                    mode,
                    legacy: None,
                    si: Some(ControlSI {
                        thrust_si: si.thrust_si,
                        torque: si.torque,
                    }),
                    normalized_forces: None,
                }
            }
            ControlMode::Force => {
                let forces_ptr = ptr as *const ForceLayout;
                let forces = &*forces_ptr;
                Control {
                    mode,
                    legacy: None,
                    si: None,
                    normalized_forces: Some(forces.normalized_forces),
                }
            }
            ControlMode::Unknown(_) => Control {
                mode,
                legacy: None,
                si: None,
                normalized_forces: None,
            },
        }
    }
    pub unsafe fn write_to_raw(&self, ptr: *mut ffi::control_t) {
        if ptr.is_null() {
            return;
        }

        let raw = &mut *ptr;

        match self.mode {
            ControlMode::Legacy => {
                let legacy_ptr = ptr as *mut LegacyLayout;
                let legacy = &mut *legacy_ptr;
                if let Some(legacy_data) = self.legacy {
                    legacy.roll = legacy_data.roll;
                    legacy.pitch = legacy_data.pitch;
                    legacy.yaw = legacy_data.yaw;
                    legacy.thrust = legacy_data.thrust;
                }
            }
            ControlMode::ForceTorque => {
                let si_ptr = ptr as *mut SIForceLayout;
                let si = &mut *si_ptr;
                if let Some(si_data) = self.si {
                    si.thrust_si = si_data.thrust_si;
                    si.torque = si_data.torque;
                }
            }
            ControlMode::Force => {
                let force_ptr = ptr as *mut ForceLayout;
                let force = &mut *force_ptr;
                if let Some(forces) = self.normalized_forces {
                    force.normalized_forces = forces;
                }
            }
            ControlMode::Unknown(_) => {}
        }

        raw.control_mode = match self.mode {
            ControlMode::Legacy => 0,
            ControlMode::ForceTorque => 1,
            ControlMode::Force => 2,
            ControlMode::Unknown(n) => n,
        };
    }
}

// Dummy layouts to reinterpret the union properly
#[repr(C)]
struct LegacyLayout {
    roll: i16,
    pitch: i16,
    yaw: i16,
    thrust: f32,
    _pad: [u8; 64 - 10], // Fill up to where control_mode is
    control_mode: u32,
}

#[repr(C)]
struct SIForceLayout {
    thrust_si: f32,
    torque: [f32; 3],
    _pad: [u8; 64 - 4 - 12], // Fill up
    control_mode: u32,
}

#[repr(C)]
struct ForceLayout {
    normalized_forces: [f32; 4],
    _pad: [u8; 64 - 16],
    control_mode: u32,
}

#[derive(Debug, Copy, Clone)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    transposed: Option<bool>,
}

impl Vector3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Vector3 { x, y, z, transposed: None }
    }

    pub fn T(mut self) -> Self {
        self.transposed = match self.transposed {
            Some(state) => Some(!state),
            None => Some(true),
        };
        self
    }

    pub fn is_transposed(&self) -> Option<bool> {
        self.transposed
    }
}

impl core::fmt::Display for Vector3 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.transposed {
            Some(true) => write!(f, "[{}, {}, {}]^T", self.x, self.y, self.z),
            Some(false) | None => write!(f, "[{}, {}, {}]", self.x, self.y, self.z),
        }
    }
}

impl core::ops::Sub for Vector3 {
    type Output = Result<Self, &'static str>;

    fn sub(self, other: Self) -> Self::Output {
        if self.transposed != other.transposed {
            return Err("Cannot subtract vectors with different transpose states.");
        }
        Ok(Vector3 {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
            transposed: self.transposed,
        })
    }
}

impl core::ops::Add for Vector3 {
    type Output = Result<Self, &'static str>;

    fn add(self, other: Self) -> Self::Output {
        if self.transposed != other.transposed {
            return Err("Cannot add vectors with different transpose states.");
        }
        Ok(Vector3 {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
            transposed: self.transposed,
        })
    }
}


#[derive(Debug, Copy, Clone)]
pub struct Matrix3x3 {
    pub rows: [Vector3; 3],
}

impl Matrix3x3 {
    pub fn new(row1: Vector3, row2: Vector3, row3: Vector3) -> Self {
        Matrix3x3 {
            rows: [row1, row2, row3],
        }
    }

    pub fn transpose(&self) -> Self {
        Matrix3x3 {
            rows: [
                Vector3 {
                    x: self.rows[0].x,
                    y: self.rows[1].x,
                    z: self.rows[2].x,
                    transposed: Some(false),
                },
                Vector3 {
                    x: self.rows[0].y,
                    y: self.rows[1].y,
                    z: self.rows[2].y,
                    transposed: Some(false),
                },
                Vector3 {
                    x: self.rows[0].z,
                    y: self.rows[1].z,
                    z: self.rows[2].z,
                    transposed: Some(false),
                },
            ],
        }
    }
}

impl fmt::Display for Matrix3x3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{}, {}, {}]",
            self.rows[0], self.rows[1], self.rows[2]
        )
    }
}

impl core::ops::Mul<Vector3> for Matrix3x3 {
    type Output = Result<Vector3, &'static str>;

    fn mul(self, vec: Vector3) -> Self::Output {
        if vec.transposed == Some(true) {
            return Err("Cannot multiply a matrix with a transposed vector.");
        }

        Ok(Vector3 {
            x: self.rows[0].x * vec.x + self.rows[0].y * vec.y + self.rows[0].z * vec.z,
            y: self.rows[1].x * vec.x + self.rows[1].y * vec.y + self.rows[1].z * vec.z,
            z: self.rows[2].x * vec.x + self.rows[2].y * vec.y + self.rows[2].z * vec.z,
            transposed: Some(false),
        })
    }
}

impl core::ops::Mul<Matrix3x3> for Vector3 {
    type Output = Result<Vector3, &'static str>;

    fn mul(self, matrix: Matrix3x3) -> Self::Output {
        if self.transposed != Some(true) {
            return Err("Cannot multiply a non-transposed vector with a matrix.");
        }

        Ok(Vector3 {
            x: self.x * matrix.rows[0].x + self.y * matrix.rows[1].x + self.z * matrix.rows[2].x,
            y: self.x * matrix.rows[0].y + self.y * matrix.rows[1].y + self.z * matrix.rows[2].y,
            z: self.x * matrix.rows[0].z + self.y * matrix.rows[1].z + self.z * matrix.rows[2].z,
            transposed: Some(true),
        })
    }
}

impl core::ops::Mul<Vector3> for Vector3 {
    type Output = Result<Matrix3x3, &'static str>;

    fn mul(self, other: Vector3) -> Self::Output {
        if self.transposed != Some(true) || other.transposed != Some(false) {
            return Err("Multiplying vectors requires a transposed vector on the left and a non-transposed vector on the right.");
        }

        Ok(Matrix3x3 {
            rows: [
                Vector3 {
                    x: self.x * other.x,
                    y: self.x * other.y,
                    z: self.x * other.z,
                    transposed: Some(false),
                },
                Vector3 {
                    x: self.y * other.x,
                    y: self.y * other.y,
                    z: self.y * other.z,
                    transposed: Some(false),
                },
                Vector3 {
                    x: self.z * other.x,
                    y: self.z * other.y,
                    z: self.z * other.z,
                    transposed: Some(false),
                },
            ],
        })
    }
}


#[derive(Debug, Copy, Clone)]
pub struct Quaternion {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

#[derive(Debug, Copy, Clone)]
pub enum SetPointModeT {
    ModeDisable,
    ModeAbs,
    ModeVelocity,
}

impl TryFrom<u8> for SetPointModeT {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(SetPointModeT::ModeDisable),
            1 => Ok(SetPointModeT::ModeAbs),
            2 => Ok(SetPointModeT::ModeVelocity),
            _ => Err(()), // Return an error for invalid values
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct SetpointMode {
    pub x: SetPointModeT,
    pub y: SetPointModeT,
    pub z: SetPointModeT,
    pub roll: SetPointModeT,
    pub pitch: SetPointModeT,
    pub yaw: SetPointModeT,
    pub quat: SetPointModeT,
}

impl fmt::Display for SetpointMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "X: {:?}, Y: {:?}, Z: {:?}, Roll: {:?}, Pitch: {:?}, Yaw: {:?}, Quat: {:?}",
            self.x, self.y, self.z, self.roll, self.pitch, self.yaw, self.quat
        )
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Setpoint {
    pub timestamp: u32,
    pub attitude: Vector3,
    pub attitude_rate: Vector3,
    pub attitude_quaternion: Quaternion,
    pub thrust: f32,
    pub position: Vector3,
    pub velocity: Vector3,
    pub acceleration: Vector3,
    pub jerk: Vector3,
    pub velocity_body: bool,
    pub mode: SetpointMode,
}

impl Setpoint {
    pub unsafe fn from_raw(ptr: *const ffi::setpoint_t) -> Self {
        if ptr.is_null() {
            return Setpoint {
                timestamp: 0,
                attitude: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                attitude_rate: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                attitude_quaternion: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                thrust: 0.0,
                position: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                velocity: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                acceleration: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                jerk: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                velocity_body: false,
                mode: SetpointMode {
                    x: SetPointModeT::ModeDisable,
                    y: SetPointModeT::ModeDisable,
                    z: SetPointModeT::ModeDisable,
                    roll: SetPointModeT::ModeDisable,
                    pitch: SetPointModeT::ModeDisable,
                    yaw: SetPointModeT::ModeDisable,
                    quat: SetPointModeT::ModeDisable,
                },
            };
        }

        let raw = &*ptr;

        Setpoint {
            timestamp: raw.timestamp,
            attitude: Vector3 {
                x: raw.attitude.x,
                y: raw.attitude.y,
                z: raw.attitude.z,
                transposed: Some(false),
            },
            attitude_rate: Vector3 {
                x: raw.attitudeRate.x,
                y: raw.attitudeRate.y,
                z: raw.attitudeRate.z,
                transposed: Some(false),
            },
            attitude_quaternion: Quaternion {
                x: raw.attitudeQuaternion.x,
                y: raw.attitudeQuaternion.y,
                z: raw.attitudeQuaternion.z,
                w: raw.attitudeQuaternion.w,
            },
            thrust: raw.thrust,
            position: Vector3 {
                x: raw.position.x,
                y: raw.position.y,
                z: raw.position.z,
                transposed: Some(false),
            },
            velocity: Vector3 {
                x: raw.velocity.x,
                y: raw.velocity.y,
                z: raw.velocity.z,
                transposed: Some(false),
            },
            acceleration: Vector3 {
                x: raw.acceleration.x,
                y: raw.acceleration.y,
                z: raw.acceleration.z,
                transposed: Some(false),
            },
            jerk: Vector3 {
                x: raw.jerk.x,
                y: raw.jerk.y,
                z: raw.jerk.z,
                transposed: Some(false),
            },
            velocity_body: raw.velocity_body,
            mode: SetpointMode {
                x: SetPointModeT::try_from(raw.mode.x).unwrap_or(SetPointModeT::ModeDisable),
                y: SetPointModeT::try_from(raw.mode.y).unwrap_or(SetPointModeT::ModeDisable),
                z: SetPointModeT::try_from(raw.mode.z).unwrap_or(SetPointModeT::ModeDisable),
                roll: SetPointModeT::try_from(raw.mode.roll).unwrap_or(SetPointModeT::ModeDisable),
                pitch: SetPointModeT::try_from(raw.mode.pitch).unwrap_or(SetPointModeT::ModeDisable),
                yaw: SetPointModeT::try_from(raw.mode.yaw).unwrap_or(SetPointModeT::ModeDisable),
                quat: SetPointModeT::try_from(raw.mode.quat).unwrap_or(SetPointModeT::ModeDisable),
            },
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct SensorData {
    pub acc: Vector3,
    pub gyro: Vector3,
    pub mag: Vector3,
    pub baro: f32,
    pub interrupt_timestamp: u64,
}

impl SensorData {
    pub unsafe fn from_raw(ptr: *const ffi::sensorData_t) -> Self {
        if ptr.is_null() {
            return SensorData {
                acc: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                gyro: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                mag: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                baro: 0.0,
                interrupt_timestamp: 0,
            };
        }

        let raw = &*ptr;

        SensorData {
            acc: Vector3 {
                x: raw.acc.x,
                y: raw.acc.y,
                z: raw.acc.z,
                transposed: Some(false),
            },
            gyro: Vector3 {
                x: raw.gyro.x,
                y: raw.gyro.y,
                z: raw.gyro.z,
                transposed: Some(false),
            },
            mag: Vector3 {
                x: raw.mag.x,
                y: raw.mag.y,
                z: raw.mag.z,
                transposed: Some(false),
            },
            baro: raw.baro,
            interrupt_timestamp: raw.interruptTimestamp,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct State {
    pub attitude: Vector3,
    pub attitude_quaternion: Quaternion,
    pub position: Vector3,
    pub velocity: Vector3,
    pub acceleration: Vector3,
}

impl State {
    pub unsafe fn from_raw(ptr: *const ffi::state_t) -> Self {
        if ptr.is_null() {
            return State {
                attitude: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                attitude_quaternion: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                position: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                velocity: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
                acceleration: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    transposed: Some(false),
                },
            };
        }

        let raw = &*ptr;

        State {
            attitude: Vector3 {
                x: raw.attitude.x,
                y: raw.attitude.y,
                z: raw.attitude.z,
                transposed: Some(false),
            },
            attitude_quaternion: Quaternion {
                x: raw.attitudeQuaternion.x,
                y: raw.attitudeQuaternion.y,
                z: raw.attitudeQuaternion.z,
                w: raw.attitudeQuaternion.w,
            },
            position: Vector3 {
                x: raw.position.x,
                y: raw.position.y,
                z: raw.position.z,
                transposed: Some(false),
            },
            velocity: Vector3 {
                x: raw.velocity.x,
                y: raw.velocity.y,
                z: raw.velocity.z,
                transposed: Some(false),
            },
            acceleration: Vector3 {
                x: raw.acc.x,
                y: raw.acc.y,
                z: raw.acc.z,
                transposed: Some(false),
            },
        }
    }
}
