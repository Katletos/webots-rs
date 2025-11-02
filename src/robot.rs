use std::ffi::{CStr, CString};

use webots_bindings::{
    wb_robot_battery_sensor_disable, wb_robot_battery_sensor_enable,
    wb_robot_battery_sensor_get_sampling_period,
    wb_robot_battery_sensor_get_value, wb_robot_cleanup,
    wb_robot_get_basic_time_step, wb_robot_get_custom_data,
    wb_robot_get_device, wb_robot_get_mode, wb_robot_get_model,
    wb_robot_get_name, wb_robot_get_project_path, wb_robot_get_supervisor,
    wb_robot_get_synchronization, wb_robot_get_time, wb_robot_get_urdf,
    wb_robot_get_world_path, wb_robot_init, wb_robot_set_custom_data,
    wb_robot_set_mode, wb_robot_step,
};

use crate::{
    Accelerometer, Brake, Camera, DistanceSensor, Gps, Gyro, InertialUnit,
    Keyboard, Lidar, Motor, PositionSensor, Receiver, RobotMode, TouchSensor,
};

pub fn world_time() -> f64 {
    unsafe { wb_robot_get_time() }
}

static ROBOT: std::sync::OnceLock<Robot> = std::sync::OnceLock::new();

#[derive(Default)]
pub struct WorldLock {
    pub prev_tick: f64,
}

impl WorldLock {
    pub fn new() -> Self {
        //tick is always non-negative
        //therefore -1.0 indicates "no previous tick"
        Self { prev_tick: -1.0 }
    }

    pub fn after_step<F, T>(&mut self, mut f: F) -> T
    where
        F: FnMut() -> T,
    {
        const EPSILON: f64 = 1e-7;

        let robot = Robot::global();

        let output: T;

        let in_same_tick = |prev: f64| -> bool {
            let tick_start = world_time();

            (tick_start - prev).abs() < EPSILON
        };

        {
            let mut tick_guard = robot.world_tick.lock().unwrap();

            if in_same_tick(self.prev_tick) {
                tick_guard.wanted += 1;

                tick_guard = robot
                    .step_cond
                    .wait_while(tick_guard, |_| in_same_tick(self.prev_tick))
                    .unwrap();
            } else {
                tick_guard.started += 1;
            }

            tick_guard = robot
                .step_cond
                .wait_while(tick_guard, |tick| tick.in_tick)
                .unwrap();

            output = f();

            tick_guard.started -= 1;

            self.prev_tick = world_time();
        }

        robot.workers_cond.notify_all();

        output
    }
}

struct WorldTick {
    in_tick: bool,
    wanted: usize,
    started: usize,
}

pub struct Robot {
    init: Option<()>,
    world_tick: std::sync::Mutex<WorldTick>,
    //variable used to notify main thread after all workers are done
    workers_cond: std::sync::Condvar,
    //variable used to notify devices after step
    step_cond: std::sync::Condvar,
}

impl Drop for Robot {
    fn drop(&mut self) {
        if let Some(_) = self.init.take() {
            unsafe {
                wb_robot_cleanup();
            }
            // Cleanup only if it was initialized
        }
    }
}

impl Robot {
    pub fn global() -> &'static Robot {
        ROBOT.get_or_init(|| {
            let _ = unsafe { wb_robot_init() };

            Robot {
                world_tick: std::sync::Mutex::new(WorldTick {
                    in_tick: false,
                    started: 0,
                    wanted: 0,
                }),
                workers_cond: std::sync::Condvar::new(),
                step_cond: std::sync::Condvar::new(),
                init: Some(()),
            }
        })
    }

    pub fn step(&self, duration: i32) -> i32 {
        self.world_tick.lock().unwrap().in_tick = true;

        let status = unsafe { wb_robot_step(duration) };

        {
            let mut tick = self.world_tick.lock().unwrap();
            tick.in_tick = false;
            tick.started += tick.wanted;
            tick.wanted = 0;
        }

        self.world_tick.lock().unwrap().in_tick = false;

        self.step_cond.notify_all();

        let _tick = self
            .workers_cond
            .wait_while(self.world_tick.lock().unwrap(), |tick| {
                tick.started > 0
            })
            .unwrap();

        status
    }

    pub fn get_urdf(prefix: &str) -> &[u8] {
        let prefix = CString::new(prefix).expect("CString::new failed");
        unsafe {
            let urdf = wb_robot_get_urdf(prefix.as_ptr());
            CStr::from_ptr(urdf).to_bytes()
        }
    }

    pub fn get_name<'a>() -> &'a [u8] {
        unsafe {
            let name = wb_robot_get_name();
            CStr::from_ptr(name).to_bytes()
        }
    }

    pub fn get_model<'a>() -> &'a [u8] {
        unsafe {
            let model = wb_robot_get_model();
            CStr::from_ptr(model).to_bytes()
        }
    }

    pub fn get_custom_data<'a>() -> &'a [u8] {
        unsafe {
            let custom_data = wb_robot_get_custom_data();
            CStr::from_ptr(custom_data).to_bytes()
        }
    }

    pub fn set_custom_data(custom_data: &[u8]) {
        let custom_data = CStr::from_bytes_with_nul(custom_data)
            .expect("CStr::from_bytes_with_nul failed");
        unsafe { wb_robot_set_custom_data(custom_data.as_ptr()) }
    }

    pub fn get_mode() -> RobotMode {
        unsafe { wb_robot_get_mode().into() }
    }

    pub fn set_mode(mode: RobotMode, argument: &str) {
        let argument = CString::new(argument).expect("CString::new failed");
        unsafe { wb_robot_set_mode(mode.into(), argument.as_ptr()) }
    }

    pub fn get_synchronization() -> bool {
        unsafe { wb_robot_get_synchronization() != 0 }
    }

    pub fn get_supervisor() -> bool {
        unsafe { wb_robot_get_supervisor() != 0 }
    }

    pub fn get_project_path<'a>() -> &'a str {
        unsafe {
            let project_path = wb_robot_get_project_path();
            CStr::from_ptr(project_path).to_str().expect("CStr::to_str")
        }
    }

    pub fn get_world_path<'a>() -> &'a str {
        unsafe {
            let world_path = wb_robot_get_world_path();
            CStr::from_ptr(world_path).to_str().expect("CStr::to_str")
        }
    }

    pub fn get_basic_time_step() -> f64 {
        unsafe { wb_robot_get_basic_time_step() }
    }

    pub fn battery_sensor_enable(sampling_period: i32) {
        unsafe { wb_robot_battery_sensor_enable(sampling_period) }
    }

    pub fn battery_sensor_disable() {
        unsafe { wb_robot_battery_sensor_disable() }
    }

    pub fn battery_sensor_get_sampling_period() -> i32 {
        unsafe { wb_robot_battery_sensor_get_sampling_period() }
    }

    pub fn battery_sensor_get_value() -> f64 {
        unsafe { wb_robot_battery_sensor_get_value() }
    }

    pub fn get_accelerometer(&self, name: &str) -> Accelerometer {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        Accelerometer::new(device, WorldLock::new())
    }

    pub fn get_brake(name: &str) -> Brake {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        Brake::new(device)
    }

    pub fn get_camera(name: &str) -> Camera {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        Camera::new(device)
    }

    pub fn get_lidar(&self, name: &str) -> Lidar {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        Lidar::new(device, WorldLock::new())
    }

    pub fn get_gps(&self, name: &str) -> Gps {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        Gps::new(device, WorldLock::new())
    }

    pub fn get_distance_sensor(name: &str) -> DistanceSensor {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        DistanceSensor::new(device)
    }

    pub fn get_gyro(&self, name: &str) -> Gyro {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        Gyro::new(device, WorldLock::new())
    }

    pub fn get_inertial_unit(name: &str) -> InertialUnit {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        InertialUnit::new(device)
    }

    pub fn get_keyboard() -> Keyboard {
        Keyboard
    }

    pub fn get_motor(&self, name: &str) -> Motor {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        Motor::new(device, WorldLock::new())
    }

    pub fn get_position_sensor(name: &str) -> PositionSensor {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        PositionSensor::new(device)
    }

    pub fn get_receiver(name: &str) -> Receiver {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        Receiver::new(device)
    }

    pub fn get_touch_sensor(name: &str) -> TouchSensor {
        let name = CString::new(name).expect("CString::new failed");
        let device = unsafe { wb_robot_get_device(name.as_ptr()) };
        TouchSensor::new(device)
    }
}
