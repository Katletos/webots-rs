use thiserror::Error;
use webots_bindings::{
    wb_device_get_node_type, wb_lidar_disable, wb_lidar_enable,
    wb_lidar_get_horizontal_resolution, wb_lidar_get_range_image, WbDeviceTag,
    WbNodeType_WB_NODE_LIDAR,
};

#[derive(Debug, Error)]
pub enum LidarError {
    #[error("failed to get values: value data is NULL")]
    ValueIsNull,
}

pub struct Lidar(WbDeviceTag);

impl Lidar {
    pub(crate) fn new(device: WbDeviceTag) -> Self {
        assert_eq!(WbNodeType_WB_NODE_LIDAR, unsafe {
            wb_device_get_node_type(device)
        });
        Self(device)
    }

    pub fn enable(&self, sampling_period: i32) {
        unsafe { wb_lidar_enable(self.0, sampling_period) }
    }

    pub fn disable(&self) {
        unsafe { wb_lidar_disable(self.0) }
    }

    pub fn get_values(&self) -> Result<Vec<f32>, LidarError> {
        unsafe {
            let size = wb_lidar_get_horizontal_resolution(self.0);
            let ptr = wb_lidar_get_range_image(self.0);
            if ptr.is_null() {
                return Err(LidarError::ValueIsNull);
            }
            let slice = std::slice::from_raw_parts(ptr, size as usize);
            Ok(slice.to_vec())
        }
    }
}
