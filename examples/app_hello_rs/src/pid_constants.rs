// Roll Rate
pub const PID_ROLL_RATE_KP: f32 = 250.0;
pub const PID_ROLL_RATE_KI: f32 = 500.0;
pub const PID_ROLL_RATE_KD: f32 = 2.5;
pub const PID_ROLL_RATE_KFF: f32 = 0.0;
pub const PID_ROLL_RATE_INTEGRATION_LIMIT: f32 = 33.3;

// Pitch Rate
pub const PID_PITCH_RATE_KP: f32 = 250.0;
pub const PID_PITCH_RATE_KI: f32 = 500.0;
pub const PID_PITCH_RATE_KD: f32 = 2.5;
pub const PID_PITCH_RATE_KFF: f32 = 0.0;
pub const PID_PITCH_RATE_INTEGRATION_LIMIT: f32 = 33.3;

// Yaw Rate
pub const PID_YAW_RATE_KP: f32 = 120.0;
pub const PID_YAW_RATE_KI: f32 = 16.7;
pub const PID_YAW_RATE_KD: f32 = 0.0;
pub const PID_YAW_RATE_KFF: f32 = 0.0;
pub const PID_YAW_RATE_INTEGRATION_LIMIT: f32 = 166.7;

// Roll
pub const PID_ROLL_KP: f32 = 6.0;
pub const PID_ROLL_KI: f32 = 3.0;
pub const PID_ROLL_KD: f32 = 0.0;
pub const PID_ROLL_KFF: f32 = 0.0;
pub const PID_ROLL_INTEGRATION_LIMIT: f32 = 20.0;

// Pitch
pub const PID_PITCH_KP: f32 = 6.0;
pub const PID_PITCH_KI: f32 = 3.0;
pub const PID_PITCH_KD: f32 = 0.0;
pub const PID_PITCH_KFF: f32 = 0.0;
pub const PID_PITCH_INTEGRATION_LIMIT: f32 = 20.0;

// Yaw
pub const PID_YAW_KP: f32 = 6.0;
pub const PID_YAW_KI: f32 = 1.0;
pub const PID_YAW_KD: f32 = 0.35;
pub const PID_YAW_KFF: f32 = 0.0;
pub const PID_YAW_INTEGRATION_LIMIT: f32 = 360.0;

// Velocity X
pub const PID_VEL_X_KP: f32 = 25.0;
pub const PID_VEL_X_KI: f32 = 1.0;
pub const PID_VEL_X_KD: f32 = 0.0;
pub const PID_VEL_X_KFF: f32 = 0.0;

// Velocity Y
pub const PID_VEL_Y_KP: f32 = 25.0;
pub const PID_VEL_Y_KI: f32 = 1.0;
pub const PID_VEL_Y_KD: f32 = 0.0;
pub const PID_VEL_Y_KFF: f32 = 0.0;

// Velocity Z
pub const PID_VEL_Z_KP: f32 = 25.0;
pub const PID_VEL_Z_KI: f32 = 15.0;
pub const PID_VEL_Z_KD: f32 = 0.0;
pub const PID_VEL_Z_KFF: f32 = 0.0;

// Velocity Z (Barometer Z Hold)
pub const PID_VEL_Z_KP_BARO_Z_HOLD: f32 = 3.0;
pub const PID_VEL_Z_KI_BARO_Z_HOLD: f32 = 1.0;
pub const PID_VEL_Z_KD_BARO_Z_HOLD: f32 = 1.5;
pub const PID_VEL_Z_KFF_BARO_Z_HOLD: f32 = 0.0;

// Velocity Limits
pub const PID_VEL_ROLL_MAX: f32 = 20.0;
pub const PID_VEL_PITCH_MAX: f32 = 20.0;
pub const PID_VEL_THRUST_BASE: f32 = 36000.0;
pub const PID_VEL_THRUST_BASE_BARO_Z_HOLD: f32 = 38000.0;
pub const PID_VEL_THRUST_MIN: f32 = 20000.0;

// Position X
pub const PID_POS_X_KP: f32 = 2.0;
pub const PID_POS_X_KI: f32 = 0.0;
pub const PID_POS_X_KD: f32 = 0.0;
pub const PID_POS_X_KFF: f32 = 0.0;

// Position Y
pub const PID_POS_Y_KP: f32 = 2.0;
pub const PID_POS_Y_KI: f32 = 0.0;
pub const PID_POS_Y_KD: f32 = 0.0;
pub const PID_POS_Y_KFF: f32 = 0.0;

// Position Z
pub const PID_POS_Z_KP: f32 = 2.0;
pub const PID_POS_Z_KI: f32 = 0.5;
pub const PID_POS_Z_KD: f32 = 0.0;
pub const PID_POS_Z_KFF: f32 = 0.0;

// Position Velocity Limits
pub const PID_POS_VEL_X_MAX: f32 = 1.0;
pub const PID_POS_VEL_Y_MAX: f32 = 1.0;
pub const PID_POS_VEL_Z_MAX: f32 = 1.0;
