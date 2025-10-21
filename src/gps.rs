use thiserror::Error;
use webots_bindings::{
    wb_device_get_node_type, wb_gps_disable, wb_gps_enable, wb_gps_get_values,
    WbDeviceTag, WbNodeType_WB_NODE_GPS,
};

#[derive(Debug, Error)]
pub enum GpsError {
    #[error("failed to get values: value data is NULL")]
    ValueIsNull,
}

pub struct Gps(WbDeviceTag);

impl Gps {
    pub(crate) fn new(device: WbDeviceTag) -> Self {
        assert_eq!(WbNodeType_WB_NODE_GPS, unsafe {
            wb_device_get_node_type(device)
        });
        Self(device)
    }

    pub fn enable(&mut self, sampling_period: i32) {
        unsafe { wb_gps_enable(self.0, sampling_period) }
    }

    pub fn disable(&mut self) {
        unsafe { wb_gps_disable(self.0) }
    }

    pub fn get_values(&self) -> Result<[f64; 3], GpsError> {
        unsafe {
            let values = wb_gps_get_values(self.0);
            if values.is_null() {
                return Err(GpsError::ValueIsNull);
            }
            Ok([*values.offset(0), *values.offset(1), *values.offset(2)])
        }
    }
}
