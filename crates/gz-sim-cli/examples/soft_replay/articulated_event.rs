// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::deformable::articulated_impact::ArticulatedMaterialImpact;
use serde_json::{Value, json};
pub fn event(r: &ArticulatedMaterialImpact) -> Value {
    let n = r.material.loads.len();
    let loads=r.material.loads.iter().map(|l|json!({"source":l.site.sphere,"vertices":l.site.feature.vertices(),"weights":l.site.weights,"normal":l.site.normal,"point":l.site.point,"clearance":l.site.clearance,"on_link":l.on_rigid,"friction":l.friction.map(|f|json!({"coefficient":f.coefficient,"on_material":f.on_material,"before":f.initial_slip,"after":f.final_slip})),"on_particles":l.on_particles,"v0":l.initial_normal_velocity,"v1":l.final_normal_velocity})).collect::<Vec<_>>();
    let k = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| r.native.before.mobility(i, j).unwrap())
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    json!({"detection":r.detection.map(|d|json!({"public_start_ns":d.public_start.as_nanos() as u64,"seconds_after_start":d.seconds_after_start,"rounding_distance":d.rounding_distance})),"links":r.links,"local_points":r.local_points,"loaded_particles":r.loaded_particles,"epoch":r.epoch,"time_ns":r.time.as_nanos() as u64,"restitution":r.restitution,"dofs":r.native.before.dofs,"loads":loads,"native_v0":r.native.before.velocities,"native_v1":r.native.after_velocities,"mobility":k,"native_work":r.native.kinetic_energy_change,"loss":r.material.dissipated_energy,"support_work":r.material.prescribed_support_work})
}
