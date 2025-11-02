use thiserror::Error;
use webots_bindings::{
    wb_device_get_node_type, wb_gps_disable, wb_gps_enable, wb_gps_get_values,
    WbDeviceTag, WbNodeType_WB_NODE_GPS,
};

use crate::robot::WorldLock;

#[derive(Debug, Error)]
pub enum GpsError {
    #[error("failed to get values: value data is NULL")]
    ValueIsNull,
}

pub struct Gps {
    tag: WbDeviceTag,
    lock: WorldLock,
}

impl Gps {
    pub(crate) fn new(device: WbDeviceTag, lock: WorldLock) -> Self {
        assert_eq!(WbNodeType_WB_NODE_GPS, unsafe {
            wb_device_get_node_type(device)
        });
        Self { tag: device, lock }
    }
    pub fn enable(&mut self, sampling_period: i32) {
        self.lock
            .after_step(|| unsafe { wb_gps_enable(self.tag, sampling_period) });
    }
    pub fn disable(&mut self) {
        self.lock.after_step(|| unsafe { wb_gps_disable(self.tag) });
    }
    pub fn get_values(&mut self) -> Result<[f64; 3], GpsError> {
        self.lock.after_step(|| unsafe {
            let values = wb_gps_get_values(self.tag);
            if values.is_null() {
                return Err(GpsError::ValueIsNull);
            }
            Ok([*values.offset(0), *values.offset(1), *values.offset(2)])
        })
    }
}
