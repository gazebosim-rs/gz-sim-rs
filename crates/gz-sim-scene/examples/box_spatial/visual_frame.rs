// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_gpu::CarryGripperBatch;
use serde_json::{Value, json};
pub fn visual_frame(tick: usize, batch: &CarryGripperBatch) -> Value {
    let lane = &batch.lanes()[0];
    let h = lane.hand;
    let pose = |p: DVec3, q: DQuat| json!({"translation":p.to_array(),"rotation":q.to_array()});
    let fixed = |x, y, z| pose(DVec3::new(x, y, z), DQuat::IDENTITY);
    let (p, q) = batch.free_box_link().map_or(
        (
            DVec3::new(h.ball[0], 0., h.ball[1]),
            DQuat::from_rotation_y(-h.angle),
        ),
        |b| (DVec3::from(b.position), DQuat::from_array(b.orientation)),
    );
    let [z, left, right] = h.position;
    json!({"time_ns":tick*1_000_000,"poses":{
        "floor":fixed(0.13,0.,-0.009),"track_guide":fixed(0.105,0.,0.263),"shelf":fixed(lane.shelf_center[0],0.,lane.shelf_center[1]),
        "cart":fixed(lane.track,0.,0.25),"guide":fixed(lane.track,0.,0.177),"carriage":fixed(lane.track,0.,z+0.08),
        "left":fixed(left,0.,z),"right":fixed(right,0.,z),"left_stem":fixed(left,0.,z+0.044),"right_stem":fixed(right,0.,z+0.044),
        "sphere":pose(p,q),"marker":pose(p+q*DVec3::new(0.014,0.015,0.),q)}})
}
