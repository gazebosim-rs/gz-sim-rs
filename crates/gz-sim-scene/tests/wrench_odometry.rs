// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    APPLY_LINK_WRENCH_CAPACITY, ApplyLinkWrenchError, ApplyLinkWrenchMode,
    GroundTruthOdometryPublication, GroundTruthOdometrySample, InputError, Name, PhysicsEngine,
    Velocity, WorldPose, apply_link_wrench_active, clear_apply_link_wrench,
    load_world_with_sdf_step, queue_apply_link_wrench, queue_link_world_wrench, remove_model,
    reset_loaded_world,
};
use std::time::Duration;

const XML: &str = include_str!("../../../examples/m36_wrench_odometry.sdf");
type Wrench = ([f64; 3], [f64; 3]);
const FORCE: Wrench = ([4.0, 0.0, 0.0], [0.0; 3]);
const FORCE_A: Wrench = ([1.0, 0.0, 0.0], [0.0; 3]);
const FORCE_B: Wrench = ([3.0, 0.0, 0.0], [0.0; 3]);
const TORQUE: Wrench = ([0.0; 3], [0.0, 0.0, 2.0]);
const TORQUE_B: Wrench = ([0.0; 3], [0.0, 0.0, 4.0]);

fn load() -> Simulation {
    load_world_with_sdf_step(
        XML,
        "m36.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn entity(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .get::<EntityId>(entity(sim, name))
        .unwrap()
        .get()
}
fn body(sim: &Simulation) -> u64 {
    id(sim, "default::rig::body")
}
fn pose_bits(pose: WorldPose) -> [u64; 7] {
    [
        pose.translation[0],
        pose.translation[1],
        pose.translation[2],
        pose.rotation[0],
        pose.rotation[1],
        pose.rotation[2],
        pose.rotation[3],
    ]
    .map(f64::to_bits)
}
#[derive(Debug, PartialEq, Eq)]
struct Reading {
    time: Duration,
    revision: u64,
    pose: [u64; 7],
    linear: [u64; 3],
    angular: [u64; 3],
}
#[derive(Debug, PartialEq, Eq)]
struct Frame {
    time: Duration,
    model: [u64; 7],
    linear: [u64; 3],
    angular: [u64; 3],
    reading: Option<Reading>,
}
fn frame(sim: &Simulation) -> Frame {
    let model = entity(sim, "default::rig");
    let link = entity(sim, "default::rig::body");
    let velocity = sim.world().get::<Velocity>(link).unwrap();
    let reading = sim
        .world()
        .get::<GroundTruthOdometrySample>(model)
        .map(|s| {
            assert_eq!(s.epoch, sim.epoch());
            assert!(s.time <= sim.elapsed());
            Reading {
                time: s.time,
                revision: s.revision,
                pose: pose_bits(s.pose),
                linear: s.linear.map(f64::to_bits),
                angular: s.angular.map(f64::to_bits),
            }
        });
    Frame {
        time: sim.elapsed(),
        model: pose_bits(*sim.world().get::<WorldPose>(model).unwrap()),
        linear: velocity.linear.map(f64::to_bits),
        angular: velocity.angular.map(f64::to_bits),
        reading,
    }
}
fn queue(sim: &mut Simulation, wrench: Wrench, mode: ApplyLinkWrenchMode) {
    let target = body(sim);
    queue_apply_link_wrench(sim, target, wrench.0, wrench.1, mode).unwrap();
}
fn step_pair(actual: &mut Simulation, direct: &mut Simulation, inputs: &[Wrench]) -> Frame {
    let target = body(direct);
    for &(force, torque) in inputs {
        queue_link_world_wrench(direct, target, force, torque).unwrap();
    }
    actual.step_exact(1).unwrap();
    direct.step_exact(1).unwrap();
    let observed = frame(actual);
    assert_eq!(
        observed,
        frame(direct),
        "individual input order must preserve actual pose and filtered history"
    );
    observed
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1e-12, "{actual} != {expected}");
}
fn velocity(sim: &Simulation, linear_x: f64, angular_z: f64) {
    let v = sim
        .world()
        .get::<Velocity>(entity(sim, "default::rig::body"))
        .unwrap();
    close(v.linear[0], linear_x);
    close(v.angular[2], angular_z);
    for value in [v.linear[1], v.linear[2], v.angular[0], v.angular[1]] {
        close(value, 0.0);
    }
}
fn warmup(actual: &mut Simulation, direct: &mut Simulation) {
    let publication = actual
        .world()
        .get::<GroundTruthOdometryPublication>(entity(actual, "default::rig"))
        .unwrap();
    assert_eq!(
        (
            &publication.odom_topic,
            &publication.covariance_topic,
            &publication.tf_topic
        ),
        (
            &"/m36/odometry".to_owned(),
            &"/m36/odometry_covariance".to_owned(),
            &"/m36/transform".to_owned()
        )
    );
    assert_eq!(
        (&publication.odom_frame, &publication.robot_base_frame),
        (&"m36/odom".to_owned(), &"m36/base".to_owned())
    );
    let first = step_pair(actual, direct, &[]);
    assert!(first.reading.is_none());
    for _ in 1..20 {
        step_pair(actual, direct, &[]);
    }
    let f = frame(actual);
    let sample = f.reading.unwrap();
    assert_eq!(sample.time, Duration::from_millis(20));
    assert_eq!(sample.revision, 1);
    assert_eq!(sample.linear.map(f64::from_bits), [0.0; 3]);
    assert_eq!(sample.angular.map(f64::from_bits), [0.0; 3]);
    assert_eq!(sample.pose[..3], [2.0_f64, -1.0, 0.0].map(f64::to_bits));
    velocity(actual, 0.0, 0.0);
}

#[test]
fn once_additive_force_and_paused_clear_match_direct_pose_and_filtered_history() {
    let (mut actual, mut direct) = (load(), load());
    warmup(&mut actual, &mut direct);
    let stationary = frame(&actual);
    queue(&mut actual, FORCE, ApplyLinkWrenchMode::Once);
    for _ in 0..3 {
        assert!(actual.tick().unwrap().dt.is_zero());
        assert!(direct.tick().unwrap().dt.is_zero());
        assert_eq!(frame(&actual), stationary);
        assert_eq!(frame(&actual), frame(&direct));
    }
    step_pair(&mut actual, &mut direct, &[FORCE]);
    velocity(&actual, 0.002, 0.0);
    step_pair(&mut actual, &mut direct, &[]);
    velocity(&actual, 0.002, 0.0);
    queue(&mut actual, FORCE_A, ApplyLinkWrenchMode::Persistent);
    queue(&mut actual, FORCE_B, ApplyLinkWrenchMode::Persistent);
    for n in 1..=18 {
        step_pair(&mut actual, &mut direct, &[FORCE_A, FORCE_B]);
        velocity(&actual, 0.002 + f64::from(n) * 0.002, 0.0);
    }
    let moved = frame(&actual);
    assert!(f64::from_bits(moved.model[0]) > 2.0);
    assert_eq!(
        moved.reading.as_ref().unwrap().time,
        Duration::from_millis(40)
    );
    assert_ne!(moved.reading, stationary.reading);
    let once = ([2.0, 0.0, 0.0], [0.0; 3]);
    queue(&mut actual, once, ApplyLinkWrenchMode::Once);
    let target = body(&actual);
    clear_apply_link_wrench(&mut actual, target).unwrap();
    // Clear-first maintenance removes this later persistent addition as well.
    queue(
        &mut actual,
        ([8.0, 0.0, 0.0], [0.0; 3]),
        ApplyLinkWrenchMode::Persistent,
    );
    actual.tick().unwrap();
    direct.tick().unwrap();
    assert_eq!(frame(&actual), moved);
    step_pair(&mut actual, &mut direct, &[once]);
    velocity(&actual, 0.039, 0.0);
    for _ in 0..19 {
        step_pair(&mut actual, &mut direct, &[]);
    }
    velocity(&actual, 0.039, 0.0);
    assert_eq!(
        frame(&actual).reading.unwrap().time,
        Duration::from_millis(60)
    );
}

#[test]
fn torque_lifetimes_and_body_frame_feedback_match_actual_ordered_world_inputs() {
    let (mut actual, mut direct) = (load(), load());
    warmup(&mut actual, &mut direct);
    let stationary = frame(&actual);
    queue(&mut actual, TORQUE, ApplyLinkWrenchMode::Once);
    step_pair(&mut actual, &mut direct, &[TORQUE]);
    velocity(&actual, 0.0, 0.002);
    step_pair(&mut actual, &mut direct, &[]);
    velocity(&actual, 0.0, 0.002);
    queue(&mut actual, TORQUE, ApplyLinkWrenchMode::Persistent);
    queue(&mut actual, TORQUE_B, ApplyLinkWrenchMode::Persistent);
    for n in 1..=18 {
        step_pair(&mut actual, &mut direct, &[TORQUE, TORQUE_B]);
        velocity(&actual, 0.0, 0.002 + f64::from(n) * 0.006);
    }
    let turning = frame(&actual);
    assert_ne!(turning.model[3..], stationary.model[3..]);
    let target = body(&actual);
    clear_apply_link_wrench(&mut actual, target).unwrap();
    actual.tick().unwrap();
    direct.tick().unwrap();
    assert_eq!(frame(&actual), turning);
    for _ in 0..20 {
        step_pair(&mut actual, &mut direct, &[]);
    }
    velocity(&actual, 0.0, 0.110);
    assert_eq!(
        frame(&actual).reading.unwrap().time,
        Duration::from_millis(60)
    );
}

fn finite_sequence(actual: &mut Simulation, direct: &mut Simulation) -> Vec<Frame> {
    warmup(actual, direct);
    queue(actual, FORCE, ApplyLinkWrenchMode::Once);
    let mut history = vec![step_pair(actual, direct, &[FORCE])];
    queue(actual, FORCE_A, ApplyLinkWrenchMode::Persistent);
    queue(actual, FORCE_B, ApplyLinkWrenchMode::Persistent);
    for _ in 0..19 {
        history.push(step_pair(actual, direct, &[FORCE_A, FORCE_B]));
    }
    history
}

#[test]
fn loaded_reset_and_removal_clear_both_owners_without_retargeting_pending_commands() {
    let (mut actual, mut direct) = (load(), load());
    let owner = id(&actual, "default::rig");
    let link = body(&actual);
    let first = finite_sequence(&mut actual, &mut direct);
    queue(&mut actual, FORCE, ApplyLinkWrenchMode::Once);
    queue(&mut actual, TORQUE, ApplyLinkWrenchMode::Persistent);
    for sim in [&mut actual, &mut direct] {
        reset_loaded_world(sim, |_| ()).unwrap();
    }
    assert_eq!(actual.epoch(), 1);
    assert_eq!(id(&actual, "default::rig"), owner);
    assert_eq!(body(&actual), link);
    assert!(frame(&actual).reading.is_none());
    assert_eq!(finite_sequence(&mut actual, &mut direct), first);
    queue(&mut actual, FORCE, ApplyLinkWrenchMode::Once);
    queue(&mut actual, TORQUE, ApplyLinkWrenchMode::Persistent);
    remove_model(&mut actual, owner).unwrap();
    assert!(
        matches!(queue_apply_link_wrench(&mut actual, link, FORCE.0, FORCE.1, ApplyLinkWrenchMode::Once), Err(ApplyLinkWrenchError::Input(InputError::MissingEntity(id))) if id == link)
    );
    assert!(clear_apply_link_wrench(&mut actual, link).is_err());
    actual.step_exact(1).unwrap();
    assert!(
        !actual
            .world()
            .iter_entities()
            .any(|e| e.contains::<GroundTruthOdometryPublication>()
                || e.contains::<GroundTruthOdometrySample>())
    );
    assert!(!apply_link_wrench_active(&actual));
    for sim in [&mut actual, &mut direct] {
        reset_loaded_world(sim, |_| ()).unwrap();
    }
    assert_eq!(actual.epoch(), 2);
    assert_eq!(body(&actual), link);
    assert_eq!(id(&actual, "default::rig"), owner);
    warmup(&mut actual, &mut direct);
    velocity(&actual, 0.0, 0.0);
    assert!(apply_link_wrench_active(&actual));
}

#[test]
fn rejected_commands_preserve_combined_history_and_prior_work_without_capacity_eviction() {
    let (mut actual, mut direct) = (load(), load());
    warmup(&mut actual, &mut direct);
    let held = frame(&actual);
    queue(&mut actual, FORCE, ApplyLinkWrenchMode::Once);
    let link = body(&actual);
    let model = id(&actual, "default::rig");
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(
            queue_apply_link_wrench(
                &mut actual,
                link,
                [value, 0.0, 0.0],
                [0.0; 3],
                ApplyLinkWrenchMode::Persistent
            )
            .is_err()
        );
    }
    assert!(
        matches!(queue_apply_link_wrench(&mut actual, model, FORCE.0, FORCE.1, ApplyLinkWrenchMode::Once), Err(ApplyLinkWrenchError::Input(InputError::NotFreeLink(id))) if id == model)
    );
    assert!(
        matches!(queue_apply_link_wrench(&mut actual, u64::MAX, FORCE.0, FORCE.1, ApplyLinkWrenchMode::Once), Err(ApplyLinkWrenchError::Input(InputError::MissingEntity(id))) if id == u64::MAX)
    );
    // Fill remaining one-shot slots with exact zero commands. An overflow
    // refusal cannot evict the first physical command or alter odometry history.
    for _ in 1..APPLY_LINK_WRENCH_CAPACITY {
        queue(&mut actual, ([0.0; 3], [0.0; 3]), ApplyLinkWrenchMode::Once);
    }
    assert!(matches!(
        queue_apply_link_wrench(
            &mut actual,
            link,
            FORCE.0,
            FORCE.1,
            ApplyLinkWrenchMode::Once
        ),
        Err(ApplyLinkWrenchError::Capacity)
    ));
    assert_eq!(frame(&actual), held);
    let mut inputs = vec![FORCE];
    inputs.resize(APPLY_LINK_WRENCH_CAPACITY, ([0.0; 3], [0.0; 3]));
    step_pair(&mut actual, &mut direct, &inputs);
    velocity(&actual, 0.002, 0.0);
    for _ in 0..19 {
        step_pair(&mut actual, &mut direct, &[]);
    }
    velocity(&actual, 0.002, 0.0);
    assert_eq!(
        frame(&actual).reading.unwrap().time,
        Duration::from_millis(40)
    );
}
