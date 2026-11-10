// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Existing world-hinge effort and unconditional JointStatePublisher together.
use bevy_ecs::entity::Entity;
use gz_physics::PhysicsError;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    InputError, JointForceCommand, JointSample, JointStatePublication, JointStateSample, Name,
    PhysicsEngine, load_world_with_sdf_step, queue_joint_effort, remove_model, reset_loaded_world,
    set_joint_force,
};
use std::time::Duration;
const XML: &str = include_str!("../../../examples/m42_joint_force_feedback.sdf");
const FORCE: &str = "<plugin filename=\"gz-sim-apply-joint-force-system\" name=\"gz::sim::systems::ApplyJointForce\"><joint_name>hinge</joint_name></plugin>";
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m42.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world().get::<EntityId>(named(sim, name)).unwrap().get()
}
fn joint(sim: &Simulation, public: u64) -> (f64, f64) {
    let s = sim
        .world()
        .get::<JointSample>(entity_by_id(sim.world(), public).unwrap())
        .unwrap();
    (s.state.position, s.state.velocity)
}
fn publication(sim: &Simulation) -> Option<JointStateSample> {
    sim.world()
        .get::<JointStateSample>(named(sim, "default::rotor"))
        .cloned()
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1e-12, "{actual} != {expected}");
}
fn matches_joint(sim: &Simulation, public: u64, sample: &JointStateSample) {
    assert_eq!(sample.time, sim.elapsed());
    assert_eq!(sample.epoch, sim.epoch());
    assert_eq!(sample.joints.len(), 1);
    assert_eq!(sample.joints[0].id, public);
    assert_eq!(
        (sample.joints[0].position, sample.joints[0].velocity),
        joint(sim, public)
    );
}
#[test]
fn native_six_pulses_keep_their_values_with_unconditional_equal_time_and_100hz_feedback() {
    let mut sim = load(XML);
    let public = id(&sim, "default::rotor::hinge");
    let meta = sim
        .world()
        .get::<JointStatePublication>(named(&sim, "default::rotor"))
        .unwrap()
        .clone();
    assert_eq!(
        (meta.model_name.as_str(), meta.topic.as_str()),
        ("rotor", "/m42/joint_state")
    );
    assert_eq!(meta.joints.len(), 1);
    assert_eq!(
        (
            meta.joints[0].parent.as_str(),
            meta.joints[0].child.as_str()
        ),
        ("world", "body")
    );
    assert_eq!(meta.joints[0].axis, [0.0, 1.0, 0.0]);
    assert_eq!(
        (
            meta.joints[0].lower,
            meta.joints[0].upper,
            meta.joints[0].damping
        ),
        (f64::NEG_INFINITY, f64::INFINITY, 0.0)
    );
    assert!(publication(&sim).is_none());
    sim.tick().unwrap();
    let first = publication(&sim).unwrap();
    matches_joint(&sim, public, &first);
    assert_eq!((first.time, first.revision), (Duration::ZERO, 1));
    sim.tick().unwrap();
    let second = publication(&sim).unwrap();
    assert_eq!((second.time, second.revision), (Duration::ZERO, 2));
    assert_eq!(second.joints, first.joints);
    let fixture: serde_json::Value = serde_json::from_str(include_str!(
        "../../gz-physics-dart/tests/fixtures/joint_effort.json"
    ))
    .unwrap();
    let rows = fixture["observations"].as_array().unwrap();
    assert_eq!(rows.len(), 6);
    let mut unthrottled = load(&XML.replace(
        "<update_rate>100</update_rate>",
        "<update_rate>0</update_rate>",
    ));
    let other = id(&unthrottled, "default::rotor::hinge");
    assert_eq!(XML.matches(FORCE).count(), 1);
    let mut direct = load(&XML.replace(FORCE, ""));
    let direct_joint = id(&direct, "default::rotor::hinge");
    for (index, row) in rows.iter().enumerate() {
        let effort = row["effort"].as_f64().unwrap();
        set_joint_force(&mut sim, public, effort).unwrap();
        set_joint_force(&mut unthrottled, other, effort).unwrap();
        queue_joint_effort(&mut direct, direct_joint, effort).unwrap();
        sim.step_exact(1).unwrap();
        unthrottled.step_exact(1).unwrap();
        direct.step_exact(1).unwrap();
        let actual = joint(&sim, public);
        // The accepted cross-engine native fixture qualifies rates, not a
        // shared DART/Rapier position integration rule. Position stays exact
        // against the same Rapier body driven by the existing one-shot API.
        assert_eq!(actual, joint(&direct, direct_joint));
        assert_eq!(sim.elapsed(), direct.elapsed());
        close(actual.1, row["dq"].as_f64().unwrap());
        assert_eq!(
            actual,
            joint(&unthrottled, other),
            "cadence does not alter physics"
        );
        assert_eq!(
            publication(&sim).unwrap().revision,
            second.revision,
            "pulse {index} is before10ms"
        );
    }
    sim.step_exact(3).unwrap();
    direct.step_exact(3).unwrap();
    assert_eq!(publication(&sim).unwrap().revision, second.revision);
    sim.step_exact(1).unwrap();
    direct.step_exact(1).unwrap();
    assert_eq!(joint(&sim, public), joint(&direct, direct_joint));
    let at10 = publication(&sim).unwrap();
    assert_eq!((at10.time, at10.revision), (Duration::from_millis(10), 3));
    matches_joint(&sim, public, &at10);
    sim.tick().unwrap();
    let equal = publication(&sim).unwrap();
    assert_eq!((equal.time, equal.revision), (at10.time, at10.revision + 1));
    assert_eq!(equal.joints, at10.joints);
}
#[test]
fn force_zero_coasts_opposite_force_reverses_and_loaded_reset_clears_commands_and_history() {
    let mut sim = load(XML);
    let public = id(&sim, "default::rotor::hinge");
    let model = id(&sim, "default::rotor");
    set_joint_force(&mut sim, public, 1.5).unwrap();
    sim.step_exact(10).unwrap();
    let driven = joint(&sim, public);
    assert!(driven.0 > 0.0 && driven.1 > 0.0);
    matches_joint(&sim, public, &publication(&sim).unwrap());
    set_joint_force(&mut sim, public, 0.0).unwrap();
    sim.step_exact(10).unwrap();
    let coast = joint(&sim, public);
    assert!(coast.0 > driven.0);
    close(coast.1, driven.1);
    set_joint_force(&mut sim, public, -1.5).unwrap();
    sim.step_exact(20).unwrap();
    assert!(joint(&sim, public).1 < 0.0);
    let epoch = sim.epoch();
    set_joint_force(&mut sim, public, 1.5).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!((sim.elapsed(), sim.epoch()), (Duration::ZERO, epoch + 1));
    assert_eq!(
        (
            id(&sim, "default::rotor"),
            id(&sim, "default::rotor::hinge")
        ),
        (model, public)
    );
    assert!(publication(&sim).is_none());
    let current = entity_by_id(sim.world(), public).unwrap();
    assert_eq!(
        sim.world()
            .get::<JointForceCommand>(current)
            .unwrap()
            .torque(),
        0.0
    );
    sim.tick().unwrap();
    let fresh = publication(&sim).unwrap();
    assert_eq!(
        (fresh.time, fresh.epoch, fresh.revision),
        (Duration::ZERO, epoch + 1, 1)
    );
    matches_joint(&sim, public, &fresh);
    sim.step_exact(10).unwrap();
    assert_eq!(joint(&sim, public), (0.0, 0.0));
    set_joint_force(&mut sim, public, 1.5).unwrap();
    remove_model(&mut sim, model).unwrap();
    assert!(entity_by_id(sim.world(), public).is_none());
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<JointStatePublication>()
                || e.contains::<JointStateSample>()
                || e.contains::<JointForceCommand>())
    );
    assert!(matches!(
        set_joint_force(&mut sim, public, 1.5),
        Err(InputError::MissingEntity(_))
    ));
    sim.step_exact(2).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(id(&sim, "default::rotor::hinge"), public);
    sim.step_exact(10).unwrap();
    assert_eq!(joint(&sim, public), (0.0, 0.0));
}
#[test]
fn feedback_does_not_expand_the_moving_parent_effort_domain() {
    let parent = "<link name='anchor'><pose>1 0 0 0 0 0</pose><inertial><mass>1</mass><inertia><ixx>0.016</ixx><iyy>0.016</iyy><izz>0.016</izz></inertia></inertial><collision name='sphere'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link>";
    let xml = XML
        .replace(FORCE, "")
        .replace(
            "<link name=\"body\">",
            &format!("{parent}<link name=\"body\">"),
        )
        .replace("<parent>world</parent>", "<parent>anchor</parent>");
    let mut sim = load(&xml);
    let public = id(&sim, "default::rotor::hinge");
    sim.tick().unwrap();
    let before = joint(&sim, public);
    let revision = publication(&sim).unwrap().revision;
    assert!(matches!(
        set_joint_force(&mut sim, public, 1.5),
        Err(InputError::Physics(PhysicsError::Unsupported {
            engine: "rapier",
            capability: "moving-parent joint effort commands"
        }))
    ));
    assert_eq!(joint(&sim, public), before);
    assert_eq!(publication(&sim).unwrap().revision, revision);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert!(
        sim.world()
            .get::<JointForceCommand>(entity_by_id(sim.world(), public).unwrap())
            .is_none()
    );
}
