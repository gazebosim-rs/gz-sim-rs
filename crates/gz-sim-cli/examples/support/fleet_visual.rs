// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
pub fn box_visual(
    id: u64,
    name: &str,
    p: [f64; 3],
    rotation: [f64; 4],
    size: [f64; 3],
    color: [f64; 3],
) -> Value {
    json!({"id":id,"name":name,"translation":p,"rotation":rotation,"shape":{"kind":"box","size":size},"color":[color[0],color[1],color[2],1.]})
}
pub fn catalog(id: u64, name: &str, ids: &[u64]) -> Value {
    json!({"model_id":id,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false})
}
