// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::JointVelocityMotor;
use gz_physics_distributed::floating_tree::{
    AdaptiveGraspFleetPool, CoupledGraspFleetPool, GraspFleetSpec, GraspReplayBudget,
};
use std::{error::Error, path::Path, process::Command};
pub type Result<T> = std::result::Result<T, Box<dyn Error>>;
pub const XML: &str = include_str!("../../../../examples/distributed_grasp_fleet.sdf");
pub const FILE: &str = "barrier.gzadaptivegraspfleet";
pub fn exe() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
pub fn recipe(xml: &str) -> Result<GraspFleetSpec> {
    Ok(GraspFleetSpec {
        assemblies: gz_sim_scene::distributed_hinge::distributed_grasp_fleet_specs(
            xml,
            "fleet.sdf",
            "default",
        )?,
    })
}
pub fn held() -> String {
    XML.replace("-0.95 0.58", "-0.52 0.58")
        .replace(" 0.95 0.58", " 0.52 0.58")
        .replace("-0.946 0.58", "-0.516 0.58")
        .replace(" 0.954 0.58", " 0.524 0.58")
}
pub fn free() -> String {
    XML.replace("0.317 0.954 0.58", "2.0 0.954 0.58")
        .replace("0.317 -0.946 0.58", "2.0 -0.946 0.58")
}
pub fn budget() -> GraspReplayBudget {
    GraspReplayBudget {
        max_intervals: 4096,
        max_events: 8192,
    }
}
pub fn motor(s: &GraspFleetSpec, group: usize, open: bool) -> Vec<(u64, JointVelocityMotor)> {
    let g = &s.assemblies[group].payload;
    let palm = g.tree.links[0].position[1];
    g.tree
        .joints
        .iter()
        .map(|j| {
            let side = g
                .tree
                .links
                .iter()
                .find(|l| l.id == j.child_id)
                .unwrap()
                .position[1]
                - palm;
            (
                j.joint_id,
                JointVelocityMotor {
                    target_velocity: side.signum() * if open { 6. } else { -6. },
                    max_force: 4.,
                },
            )
        })
        .collect()
}
pub fn forces(s: &GraspFleetSpec, y: f64, z: f64) -> Vec<(u64, [f64; 3])> {
    s.assemblies
        .iter()
        .map(|g| {
            let sign = -g.payload.payload.position[1].signum();
            (g.payload.payload.id, [0., sign * y, sign * z])
        })
        .collect()
}
pub fn kill(pid: u32) -> Result<()> {
    assert!(
        Command::new("kill")
            .args(["-KILL", &pid.to_string()])
            .status()?
            .success()
    );
    Ok(())
}
pub fn exact(p: &AdaptiveGraspFleetPool, q: &CoupledGraspFleetPool, native_bytes: bool) {
    for s in &p.spec().assemblies {
        let id = s.payload.tree.model_id;
        assert_eq!(p.observation(id), q.observation(id));
        assert_eq!(p.payload_state(id), q.payload_state(id));
        assert_eq!(p.payload_reactions(id), q.payload_reactions(id));
        if p.is_coupled() && native_bytes {
            assert_eq!(
                p.image(id),
                q.image(),
                "complete replayed native graph at tick {}",
                p.tick()
            );
        }
    }
    if p.is_coupled() {
        assert_eq!(p.cross_reactions(), q.cross_reactions());
    }
}
