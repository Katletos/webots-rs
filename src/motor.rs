use webots_bindings::{
    wb_device_get_node_type, wb_motor_disable_force_feedback,
    wb_motor_disable_torque_feedback, wb_motor_enable_force_feedback,
    wb_motor_enable_torque_feedback, wb_motor_get_acceleration,
    wb_motor_get_available_force, wb_motor_get_available_torque,
    wb_motor_get_brake, wb_motor_get_force_feedback,
    wb_motor_get_force_feedback_sampling_period, wb_motor_get_max_force,
    wb_motor_get_max_position, wb_motor_get_max_torque,
    wb_motor_get_max_velocity, wb_motor_get_min_position,
    wb_motor_get_multiplier, wb_motor_get_position_sensor,
    wb_motor_get_target_position, wb_motor_get_torque_feedback,
    wb_motor_get_torque_feedback_sampling_period, wb_motor_get_type,
    wb_motor_get_velocity, wb_motor_set_acceleration,
    wb_motor_set_available_force, wb_motor_set_available_torque,
    wb_motor_set_control_pid, wb_motor_set_force, wb_motor_set_position,
    wb_motor_set_torque, wb_motor_set_velocity, WbDeviceTag,
    WbNodeType_WB_NODE_LINEAR_MOTOR, WbNodeType_WB_NODE_ROTATIONAL_MOTOR,
};

use crate::{robot::WorldLock, Brake, JointType, PositionSensor};

pub struct Motor {
    tag: WbDeviceTag,
    lock: WorldLock,
}

impl Motor {
    pub(crate) fn new(device: WbDeviceTag, lock: WorldLock) -> Self {
        assert!({
            let node_type = unsafe { wb_device_get_node_type(device) };
            node_type == WbNodeType_WB_NODE_LINEAR_MOTOR
                || node_type == WbNodeType_WB_NODE_ROTATIONAL_MOTOR
        });
        Self { tag: device, lock }
    }
    pub fn set_position(&mut self, position: f64) {
        self.lock.after_step(|| unsafe {
            wb_motor_set_position(self.tag, position)
        });
    }
    pub fn set_acceleration(&mut self, acceleration: f64) {
        self.lock.after_step(|| unsafe {
            wb_motor_set_acceleration(self.tag, acceleration)
        });
    }
    pub fn set_velocity(&mut self, velocity: f64) {
        self.lock.after_step(|| unsafe {
            wb_motor_set_velocity(self.tag, velocity)
        });
    }
    pub fn set_force(&mut self, force: f64) {
        self.lock
            .after_step(|| unsafe { wb_motor_set_force(self.tag, force) });
    }
    pub fn set_torque(&mut self, torque: f64) {
        self.lock
            .after_step(|| unsafe { wb_motor_set_torque(self.tag, torque) });
    }
    pub fn set_available_force(&mut self, available_force: f64) {
        self.lock.after_step(|| unsafe {
            wb_motor_set_available_force(self.tag, available_force)
        });
    }
    pub fn set_available_torque(&mut self, available_torque: f64) {
        self.lock.after_step(|| unsafe {
            wb_motor_set_available_torque(self.tag, available_torque)
        });
    }
    pub fn set_control_pid(&mut self, p: f64, i: f64, d: f64) {
        self.lock.after_step(|| unsafe {
            wb_motor_set_control_pid(self.tag, p, i, d)
        });
    }
    pub fn enable_force_feedback(&mut self, sampling_period: i32) {
        self.lock.after_step(|| unsafe {
            wb_motor_enable_force_feedback(self.tag, sampling_period)
        });
    }
    pub fn disable_force_feedback(&mut self) {
        self.lock.after_step(|| unsafe {
            wb_motor_disable_force_feedback(self.tag)
        });
    }
    pub fn get_force_feedback_sampling_period(&mut self) -> i32 {
        self.lock.after_step(|| unsafe {
            wb_motor_get_force_feedback_sampling_period(self.tag)
        })
    }
    pub fn get_force_feedback(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_force_feedback(self.tag) })
    }
    pub fn enable_torque_feedback(&mut self, sampling_period: i32) {
        self.lock.after_step(|| unsafe {
            wb_motor_enable_torque_feedback(self.tag, sampling_period)
        });
    }
    pub fn disable_torque_feedback(&mut self) {
        self.lock.after_step(|| unsafe {
            wb_motor_disable_torque_feedback(self.tag)
        });
    }
    pub fn get_torque_feedback_sampling_period(&mut self) -> i32 {
        self.lock.after_step(|| unsafe {
            wb_motor_get_torque_feedback_sampling_period(self.tag)
        })
    }
    pub fn get_torque_feedback(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_torque_feedback(self.tag) })
    }
    pub fn get_type(&mut self) -> JointType {
        self.lock
            .after_step(|| unsafe { wb_motor_get_type(self.tag).into() })
    }
    pub fn get_target_position(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_target_position(self.tag) })
    }
    pub fn get_min_position(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_min_position(self.tag) })
    }
    pub fn get_max_position(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_max_position(self.tag) })
    }
    pub fn get_velocity(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_velocity(self.tag) })
    }
    pub fn get_max_velocity(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_max_velocity(self.tag) })
    }
    pub fn get_acceleration(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_acceleration(self.tag) })
    }
    pub fn get_available_force(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_available_force(self.tag) })
    }
    pub fn get_max_force(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_max_force(self.tag) })
    }
    pub fn get_available_torque(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_available_torque(self.tag) })
    }
    pub fn get_max_torque(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_max_torque(self.tag) })
    }
    pub fn get_multiplier(&mut self) -> f64 {
        self.lock
            .after_step(|| unsafe { wb_motor_get_multiplier(self.tag) })
    }
    pub fn get_brake(&mut self) -> Brake {
        Brake::new(
            self.lock
                .after_step(|| unsafe { wb_motor_get_brake(self.tag) }),
        )
    }
    pub fn get_position_sensor(&mut self) -> PositionSensor {
        PositionSensor::new(
            self.lock.after_step(|| unsafe {
                wb_motor_get_position_sensor(self.tag)
            }),
        )
    }
}
