// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "distributed-physics")]
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine,
    distributed_hinge::distributed_grounded_robot_spec,
    joint_posture::{JointPostures, PostureMode},
    load_world_with_sdf_step,
};
use std::error::Error;
#[path = "../../../examples/support/grounded_box_array.rs"]
mod grounded_box_array;
use grounded_box_array::boxes;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const HUMAN: &str = include_str!("../../../examples/humanoid_stand.sdf");
fn load(xml: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        xml,
        "robot.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn body(s: &Simulation, name: &str) -> BodyId {
    s.world()
        .iter_entities()
        .find_map(|e| {
            if e.get::<Name>()?.0 == name {
                Some(e.get::<BodyHandle>()?.id())
            } else {
                None
            }
        })
        .unwrap()
}
fn image(s: &Simulation, root: BodyId, ground: BodyId) -> Result<Vec<u8>> {
    Ok(s.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .checkpoint_grounded_robot_tree(root, ground)?)
}
#[test]
fn authored_torque_control_has_stable_public_ids_rejects_malformed_actuators_and_resets()
-> Result<()> {
    let spec = distributed_grounded_robot_spec(HUMAN, "human.sdf", "default")?;
    assert_eq!(spec.links.len(), 31);
    assert_eq!(spec.joints.len(), 30);
    assert_eq!(spec.limits.len(), 30);
    assert!((spec.links.iter().map(|x| x.mass).sum::<f64>() - 75.).abs() < 1e-12);
    let mut sim = load(HUMAN)?;
    let root = body(&sim, "default::humanoid::pelvis");
    let floor = body(&sim, "default::floor::ground");
    sim.step_exact(190)?;
    let ctl = sim
        .world()
        .resource::<JointPostures>()
        .get(spec.model_id)
        .unwrap();
    assert_eq!(ctl.last_commands().len(), 30);
    let row = ctl.axes()[0];
    let saved = image(&sim, root, floor)?;
    let mut current = sim.world_mut().resource_mut::<JointPostures>();
    let ctl = current.get_mut(spec.model_id).unwrap();
    assert!(ctl.set_target(row.joint, f64::NEG_INFINITY).is_err());
    assert!(ctl.set_target(row.joint, 20.).is_err());
    ctl.set_target(row.joint, 0.28)?;
    sim.step_exact(100)?;
    assert_eq!(
        sim.world()
            .resource::<JointPostures>()
            .get(spec.model_id)
            .unwrap()
            .last_commands()[0]
            .target_rad,
        0.28
    );
    sim.reset()?;
    sim.step_exact(190)?;
    assert_eq!(saved, image(&sim, root, floor)?);
    sim.world_mut()
        .resource_mut::<JointPostures>()
        .get_mut(spec.model_id)
        .unwrap()
        .set_mode(PostureMode::Passive);
    sim.step_exact(400)?;
    let changed = image(&sim, root, floor)?;
    assert_ne!(saved, changed);
    for (from, to) in [
        ("max_effort=\"150\"", "max_effort=\"-1\""),
        ("stiffness=\"1000\"", "stiffness=\"NaN\""),
        ("joint=\"neck_yaw\"", "joint=\"neck_pitch\""),
        ("joint=\"left_ankle_pitch\"", "joint=\"other_model_joint\""),
        ("feedback=\"-y\"", "feedback=\"typo\""),
        ("velocity_gain=\"0.58\"", "velocity_x=\"0.58\""),
        (
            "<model>default::humanoid</model>",
            "<model>default::floor</model>",
        ),
    ] {
        assert!(
            load(&HUMAN.replacen(from, to, 1)).is_err(),
            "{from} -> {to}"
        );
    }
    Ok(())
}

#[test]
fn capacity_includes_four_loaded_points_per_box_and_rejects_changed_recipe() -> Result<()> {
    let text = boxes(32);
    let mut sim = load(&text)?;
    let root = body(&sim, "default::feet::b0");
    let floor = body(&sim, "default::floor::ground");
    let first = distributed_grounded_robot_spec(&text, "32.sdf", "default")?;
    assert_eq!(first.links.len(), 32);
    assert_eq!(
        first.model_id,
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::feet"))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get()
    );
    let mut maximum = 0;
    let mut points = 0;
    let mut snapshot = None;
    for t in 0..85 {
        sim.step_exact(1)?;
        let be = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let solved = be
            .fixed_ground_patches(floor)?
            .iter()
            .map(|p| p.points.iter().filter(|x| x.normal_impulse > 0.).count())
            .sum::<usize>();
        let graph = image(&sim, root, floor)?;
        maximum = maximum.max(graph.len());
        if solved > points {
            points = solved;
            snapshot = Some((t, graph));
        }
    }
    assert_eq!(points, 128);
    let (t, graph) = snapshot.unwrap();
    for other in [
        text.clone(),
        text.replace("0.12 0.12 0.12", "0.13 0.12 0.12"),
        text.replace("<upper>0.22</upper>", "<upper>0.12</upper>"),
    ] {
        let matches = other == text;
        let mut fresh = load(&other)?;
        let mut backend = fresh.world_mut().resource_mut::<PhysicsBackend>();
        let target = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
        assert_eq!(
            target
                .restore_grounded_robot_tree(root, floor, &graph)
                .is_ok(),
            matches
        );
        if matches {
            assert_eq!(target.checkpoint_grounded_robot_tree(root, floor)?, graph);
        }
    }
    assert!(distributed_grounded_robot_spec(&boxes(33), "too-many.sdf", "default").is_err());
    println!(
        "32-link star, tick {}: {points} strictly loaded corner rows; maximum warm graph {maximum} bytes",
        t + 1
    );
    Ok(())
}
