// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
/// Last native substep's shared-plane contact manifold for one distributed body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GroundContactPatch {
    pub body: u64,
    pub normal_on_body: [f64; 3],
    pub points: Vec<gz_physics_multifidelity::GroundPatchPoint>,
}
impl Partition {
    pub(crate) fn ground_contact_patches(&self) -> Vec<GroundContactPatch> {
        let mut result = Vec::new();
        for island in &self.solvers {
            let patches = island.native_backend().partition_ground_patches();
            if patches.is_empty() {
                continue;
            }
            let ids = island.public_body_ids();
            result.extend(patches.into_iter().map(|p| GroundContactPatch {
                body: ids[&p.body],
                normal_on_body: p.normal_on_body,
                points: p.points,
            }));
        }
        result.sort_by_key(|p| p.body);
        result
    }
}
