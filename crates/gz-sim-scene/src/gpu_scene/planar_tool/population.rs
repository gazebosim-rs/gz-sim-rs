// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_gpu::{FreeSphere, PlanarObjectBatch};

pub(super) trait Population {
    fn identity(&self) -> (MixedBatchOwner, usize);
    fn tool(&self, index: usize) -> &PlanarToolLane;
    fn object(&self, index: usize) -> Option<&FreeSphere>;
    fn target(&mut self, index: usize, angles: [f64; 2]) -> Result<(), GpuError>;
    fn motor(&mut self, index: usize, axis: usize, speed: f64) -> Result<(), GpuError>;
}
impl Population for PlanarToolBatch {
    fn identity(&self) -> (MixedBatchOwner, usize) {
        (self.owner(), self.lanes().len())
    }
    fn tool(&self, index: usize) -> &PlanarToolLane {
        &self.lanes()[index]
    }
    fn object(&self, _: usize) -> Option<&FreeSphere> {
        None
    }
    fn target(&mut self, i: usize, a: [f64; 2]) -> Result<(), GpuError> {
        self.set_target_angles(i, a)
    }
    fn motor(&mut self, i: usize, axis: usize, speed: f64) -> Result<(), GpuError> {
        self.set_motor_speed(i, axis, speed)
    }
}
impl Population for PlanarObjectBatch {
    fn identity(&self) -> (MixedBatchOwner, usize) {
        (self.owner(), self.lanes().len())
    }
    fn tool(&self, index: usize) -> &PlanarToolLane {
        &self.lanes()[index].tool
    }
    fn object(&self, index: usize) -> Option<&FreeSphere> {
        Some(&self.lanes()[index].object)
    }
    fn target(&mut self, i: usize, a: [f64; 2]) -> Result<(), GpuError> {
        self.set_target_angles(i, a)
    }
    fn motor(&mut self, i: usize, axis: usize, speed: f64) -> Result<(), GpuError> {
        self.set_motor_speed(i, axis, speed)
    }
}
pub(super) fn get(world: &World) -> Option<&dyn Population> {
    match (
        world.get_resource::<PlanarToolBatch>(),
        world.get_resource::<PlanarObjectBatch>(),
    ) {
        (Some(b), None) => Some(b),
        (None, Some(b)) => Some(b),
        _ => None,
    }
}
