use thiserror::Error;
use webots_bindings::{
    wb_device_get_node_type, wb_lidar_disable, wb_lidar_enable,
    wb_lidar_get_horizontal_resolution, wb_lidar_get_range_image, WbDeviceTag,
    WbNodeType_WB_NODE_LIDAR,
};

use crate::robot::WorldLock;

#[derive(Debug, Error)]
pub enum LidarError {
    #[error("failed to get values: value data is NULL")]
    ValueIsNull,
}

pub struct Lidar {
    pub tag: WbDeviceTag,
    pub lock: WorldLock,
}

impl Lidar {
    pub(crate) fn new(tag: WbDeviceTag, lock: WorldLock) -> Self {
        assert_eq!(WbNodeType_WB_NODE_LIDAR, unsafe {
            wb_device_get_node_type(tag)
        });

        Self { tag, lock }
    }

    pub fn enable(&self, sampling_period: i32) {
        self.lock.after_step(|| {
            unsafe { wb_lidar_enable(self.tag, sampling_period) };
        });
    }

    pub fn disable(&self) {
        self.lock
            .after_step(|| unsafe { wb_lidar_disable(self.tag) });
    }

    pub fn get_values(&self) -> Result<Vec<f32>, LidarError> {
        self.lock.after_step(|| unsafe {
            let size = wb_lidar_get_horizontal_resolution(self.tag);
            let ptr = wb_lidar_get_range_image(self.tag);
            if ptr.is_null() {
                return Err(LidarError::ValueIsNull);
            }
            let slice = std::slice::from_raw_parts(ptr, size as usize);
            Ok(slice.to_vec())
        })
    }
}
