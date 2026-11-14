// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, StepError, StepFault, entity_by_id};
use gz_sim_scene::{
    BodyHandle, InputError, JointHandle, JointSample, JointStatePublication, JointStateSample,
    LoadError, LocalPose, Name, PhysicsEngine, SceneResetError, WorldPose,
    initial_joint_state_publications, load_world_with_sdf_step, queue_joint_effort,
    queue_link_world_wrench, remove_model, reset_loaded_world,
};
use std::time::Duration;

const ORIGINAL: &str = include_str!("fixtures/m18_joint_state.sdf");
type JointBits = (u64, u64, u64);
type Snapshot = (Duration, u64, u64, [u64; 3], [u64; 4], Vec<JointBits>);
fn model(index: usize, rate: &str, selection: &str) -> String {
    let begin = ORIGINAL.find("  <model ").unwrap();
    let end = ORIGINAL.rfind("</model>").unwrap() + "</model>".len();
    ORIGINAL[begin..end]
        .replace("name=\"arm\"", &format!("name=\"arm{index}\""))
        .replace(
            "<pose>0 0 1 0 0 0</pose>",
            &format!("<pose>0 {} 1 0 0 0</pose>", index * 4),
        )
        .replace(
            "<topic>/m18/joint_state</topic>",
            &format!("<topic>/m57/{index}/joint_state</topic>{selection}"),
        )
        .replace(
            "<update_rate>100</update_rate>",
            &format!("<update_rate>{rate}</update_rate>"),
        )
}
fn world(models: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='step' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn try_load(source: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        source,
        "m57-multiple-joint-states.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
}
fn load(source: &str) -> Simulation {
    try_load(source).unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let node = sim
        .world()
        .iter_entities()
        .find(|node| node.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap();
    (node.id(), node.get::<EntityId>().unwrap().get())
}
fn owner(sim: &Simulation, index: usize) -> (Entity, u64) {
    named(sim, &format!("default::arm{index}"))
}
fn snapshot(sim: &Simulation, entity: Entity) -> Option<Snapshot> {
    let value = sim.world().get::<JointStateSample>(entity)?;
    Some((
        value.time,
        value.epoch,
        value.revision,
        value.pose.translation.map(f64::to_bits),
        value.pose.rotation.map(f64::to_bits),
        value
            .joints
            .iter()
            .map(|joint| (joint.id, joint.position.to_bits(), joint.velocity.to_bits()))
            .collect(),
    ))
}
fn geometry(sim: &Simulation, entity: Entity) {
    let publication = sim.world().get::<JointStatePublication>(entity).unwrap();
    let sample = sim.world().get::<JointStateSample>(entity).unwrap();
    let local = sim.world().get::<LocalPose>(entity).unwrap().0;
    assert_eq!((sample.time, sample.epoch), (sim.elapsed(), sim.epoch()));
    assert_eq!(sample.pose.translation, local.translation);
    assert_eq!(sample.pose.rotation, local.rotation);
    assert_eq!(sample.joints.len(), publication.joints.len());
    assert!(
        publication
            .joints
            .windows(2)
            .all(|pair| pair[0].id < pair[1].id)
    );
    for (metadata, actual) in publication.joints.iter().zip(&sample.joints) {
        assert_eq!(actual.id, metadata.id);
        let joint = entity_by_id(sim.world(), metadata.id).unwrap();
        let backend = sim.world().get::<JointSample>(joint).unwrap();
        assert_eq!((backend.time, backend.epoch), (sample.time, sample.epoch));
        assert_eq!(
            (actual.position.to_bits(), actual.velocity.to_bits()),
            (
                backend.state.position.to_bits(),
                backend.state.velocity.to_bits()
            )
        );
        let handle = sim.world().get::<JointHandle>(joint).unwrap();
        let solver = sim
            .world()
            .resource::<PhysicsBackend>()
            .joint_state(handle.id())
            .unwrap();
        assert_eq!(
            (actual.position.to_bits(), actual.velocity.to_bits()),
            (solver.position.to_bits(), solver.velocity.to_bits())
        );
        assert_eq!(metadata.axis, [0.0, 0.0, 1.0]);
        assert_eq!(
            (metadata.lower, metadata.upper, metadata.damping),
            (f64::NEG_INFINITY, f64::INFINITY, 0.0)
        );
        assert_eq!(metadata.pose, sdformat::Transform::default());
    }
}

#[test]
fn distinct_owners_real_articulated_force_torque_and_motors_preserve_joint_values_and_loaded_reset()
{
    let source = world(&(model(0, "100", "") + &model(1, "50", "")));
    let mut sim = load(&source);
    let owners = [owner(&sim, 0), owner(&sim, 1)];
    let publications = owners.map(|(entity, _)| {
        sim.world()
            .get::<JointStatePublication>(entity)
            .unwrap()
            .clone()
    });
    let tips = [
        named(&sim, "default::arm0::tip"),
        named(&sim, "default::arm1::tip"),
    ];
    let initial = tips.map(|(entity, _)| *sim.world().get::<WorldPose>(entity).unwrap());
    let mut motors = Vec::new();
    for (index, (entity, identity)) in tips.iter().enumerate() {
        let force = [2.0, 3.0, 0.0];
        let torque = [0.0, 0.0, 0.5 + index as f64];
        assert!(
            matches!(queue_link_world_wrench(&mut sim, *identity, force, torque), Err(InputError::NotFreeLink(id)) if id == *identity)
        );
        let body = sim.world().get::<BodyHandle>(*entity).unwrap().id();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_world_wrench(body, force, torque)
            .unwrap();
        let joint = named(&sim, &format!("default::arm{index}::tip_hinge")).0;
        let handle = sim.world().get::<JointHandle>(joint).unwrap().id();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_joint_velocity_motor(
                handle,
                Some(JointVelocityMotor {
                    target_velocity: 0.5 + index as f64 * 0.25,
                    max_force: 0.5,
                }),
            )
            .unwrap();
        motors.push(handle);
    }
    let mut held = [None, None];
    let mut times = [Vec::new(), Vec::new()];
    for _ in 0..80 {
        sim.step_exact(1).unwrap();
        for (index, old) in held.iter_mut().enumerate() {
            let value = snapshot(&sim, owners[index].0);
            if value.as_ref().is_some_and(|value| value.0 == sim.elapsed()) {
                geometry(&sim, owners[index].0);
                times[index].push(sim.elapsed());
            } else {
                assert_eq!(&value, old);
            }
            *old = value;
        }
    }
    assert_eq!(
        times[0],
        (1..=8)
            .map(|value| Duration::from_millis(value * 10))
            .collect::<Vec<_>>()
    );
    assert_eq!(times[1], [20, 40, 60, 80].map(Duration::from_millis));
    for (index, handle) in motors.into_iter().enumerate() {
        let actual = sim
            .world()
            .resource::<PhysicsBackend>()
            .joint_state(handle)
            .unwrap();
        assert!(actual.position.abs() > 0.0 && actual.velocity.abs() > 0.0);
        let current = sim.world().get::<WorldPose>(tips[index].0).unwrap();
        assert_ne!(current.translation, initial[index].translation);
        assert_ne!(current.rotation, initial[index].rotation);
        assert_eq!(
            sim.world()
                .get::<JointStatePublication>(owners[index].0)
                .unwrap(),
            &publications[index]
        );
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let mut fresh = load(&source);
    sim.step_exact(20).unwrap();
    fresh.step_exact(20).unwrap();
    for (index, (old, id)) in owners.into_iter().enumerate() {
        let restored = entity_by_id(sim.world(), id).unwrap();
        assert_ne!(restored, old);
        assert_eq!(
            sim.world().get::<JointStatePublication>(restored).unwrap(),
            &publications[index]
        );
        geometry(&sim, restored);
        let a = snapshot(&sim, restored).unwrap();
        let b = snapshot(&fresh, owner(&fresh, index).0).unwrap();
        assert_eq!((a.0, a.2, a.3, a.4, a.5), (b.0, b.2, b.3, b.4, b.5));
        assert_eq!((a.1, b.1), (1, 0));
    }
}

#[test]
fn absent_pair_and_eight_owners_keep_scoped_filters_and_one_family_schedule() {
    let empty = load(&world(""));
    assert_eq!(initial_joint_state_publications(&empty).unwrap().count(), 0);
    assert!(!empty.world().iter_entities().any(
        |node| node.contains::<JointStatePublication>() || node.contains::<JointStateSample>()
    ));
    for count in [1, 2, 8] {
        let models = (0..count)
            .map(|index| {
                model(
                    index,
                    "0",
                    if index.is_multiple_of(2) {
                        "<joint_name>tip_hinge</joint_name><joint_name>tip_hinge</joint_name>"
                    } else {
                        "<joint_name>base_hinge</joint_name>"
                    },
                )
            })
            .collect::<String>();
        // Removing only complete plugin elements retains the same physical graph.
        let mut no_plugins = models.clone();
        while let Some(begin) = no_plugins.find("    <plugin ") {
            let end = begin + no_plugins[begin..].find("</plugin>").unwrap() + "</plugin>".len();
            no_plugins.replace_range(begin..end, "");
        }
        let baseline = load(&world(&no_plugins)).scheduled_system_counts();
        let mut sim = load(&world(&models));
        assert_eq!(
            sim.scheduled_system_counts(),
            (baseline.0 + 1, baseline.1 + 1)
        );
        assert_eq!(
            initial_joint_state_publications(&sim).unwrap().count(),
            count
        );
        sim.step_exact(1).unwrap();
        let mut ids = std::collections::BTreeSet::new();
        for index in 0..count {
            let (entity, id) = owner(&sim, index);
            assert!(ids.insert(id));
            geometry(&sim, entity);
            let publication = sim.world().get::<JointStatePublication>(entity).unwrap();
            assert_eq!(publication.model_name, format!("arm{index}"));
            assert_eq!(publication.topic, format!("/m57/{index}/joint_state"));
            assert_eq!(publication.joints.len(), 1);
            let selected = if index.is_multiple_of(2) {
                "tip_hinge"
            } else {
                "base_hinge"
            };
            assert_eq!(publication.joints[0].name, selected);
            assert_eq!(
                publication.joints[0].id,
                named(&sim, &format!("default::arm{index}::{selected}")).1
            );
            assert!(ids.insert(publication.joints[0].id));
        }
    }
}

#[test]
fn equal_time_paused_and_core_reset_remain_distinct_from_loaded_reconstruction() {
    let source = world(&(model(0, "100", "") + &model(1, "50", "")));
    let mut sim = load(&source);
    sim.step_exact(40).unwrap();
    let owners = [owner(&sim, 0), owner(&sim, 1)];
    let before = owners.map(|(entity, _)| snapshot(&sim, entity).unwrap());
    assert!(sim.is_paused());
    sim.tick().unwrap();
    sim.tick().unwrap();
    for (index, (entity, _)) in owners.iter().enumerate() {
        geometry(&sim, *entity);
        let after = snapshot(&sim, *entity).unwrap();
        assert_eq!(
            (after.0, after.2),
            (Duration::from_millis(40), before[index].2 + 2)
        );
        assert_eq!(
            (after.3, after.4, after.5),
            (before[index].3, before[index].4, before[index].5.clone())
        );
    }
    let revisions = owners.map(|(entity, _)| {
        sim.world()
            .get::<JointStateSample>(entity)
            .unwrap()
            .revision
    });
    sim.reset().unwrap();
    assert!(
        owners
            .iter()
            .all(|(entity, _)| snapshot(&sim, *entity).is_none())
    );
    sim.tick().unwrap();
    for (index, (entity, _)) in owners.iter().enumerate() {
        let value = sim.world().get::<JointStateSample>(*entity).unwrap();
        assert_eq!(
            (value.time, value.epoch, value.revision),
            (Duration::ZERO, 1, revisions[index] + 1)
        );
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for (old, id) in owners {
        let restored = entity_by_id(sim.world(), id).unwrap();
        assert_ne!(restored, old);
        assert!(snapshot(&sim, restored).is_none());
    }
    sim.tick().unwrap();
    for index in 0..2 {
        let value = sim
            .world()
            .get::<JointStateSample>(owner(&sim, index).0)
            .unwrap();
        assert_eq!(
            (value.time, value.epoch, value.revision),
            (Duration::ZERO, 2, 1)
        );
    }
}

#[test]
fn removal_preserves_peer_progress_and_original_metadata_then_retires_last_family() {
    let source = world(&(model(0, "0", "") + &model(1, "0", "")));
    let mut sim = load(&source);
    let initial = initial_joint_state_publications(&sim)
        .unwrap()
        .map(|(id, value)| (id, value.clone()))
        .collect::<Vec<_>>();
    let owners = [owner(&sim, 0), owner(&sim, 1)];
    sim.step_exact(2).unwrap();
    remove_model(&mut sim, owners[0].1).unwrap();
    assert!(entity_by_id(sim.world(), owners[0].1).is_none());
    let peer_revision = sim
        .world()
        .get::<JointStateSample>(owners[1].0)
        .unwrap()
        .revision;
    sim.step_exact(1).unwrap();
    geometry(&sim, owners[1].0);
    assert_eq!(
        sim.world()
            .get::<JointStateSample>(owners[1].0)
            .unwrap()
            .revision,
        peer_revision + 1
    );
    assert_eq!(
        initial_joint_state_publications(&sim)
            .unwrap()
            .map(|(id, value)| (id, value.clone()))
            .collect::<Vec<_>>(),
        initial
    );
    let with_peer = sim.scheduled_system_counts();
    remove_model(&mut sim, owners[1].1).unwrap();
    assert!(!sim.world().iter_entities().any(
        |node| node.contains::<JointStatePublication>() || node.contains::<JointStateSample>()
    ));
    // Both publication callbacks retire, alongside any independently retired
    // joint callbacks. Compare against the genuinely empty loaded world.
    assert_eq!(
        sim.scheduled_system_counts(),
        load(&world("")).scheduled_system_counts()
    );
    assert!(
        with_peer.0 > sim.scheduled_system_counts().0
            && with_peer.1 > sim.scheduled_system_counts().1
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    for (old, id) in owners {
        let entity = entity_by_id(sim.world(), id).unwrap();
        assert_ne!(entity, old);
        geometry(&sim, entity);
    }
}

#[test]
fn selected_and_unselected_joint_binding_loss_remains_global_terminal_with_cached_values() {
    for victim in ["base_hinge", "tip_hinge"] {
        let source = world(
            &(model(0, "0", "<joint_name>tip_hinge</joint_name>")
                + &model(1, "0", "<joint_name>tip_hinge</joint_name>")),
        );
        let mut sim = load(&source);
        sim.step_exact(2).unwrap();
        let owners = [owner(&sim, 0), owner(&sim, 1)];
        let before = owners.map(|(entity, _)| snapshot(&sim, entity));
        let joint = named(&sim, &format!("default::arm1::{victim}")).0;
        sim.world_mut().entity_mut(joint).remove::<JointSample>();
        assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
        assert!(sim.world().resource::<StepFault>().error().is_some());
        // Global preflight runs before publication sampling, even for the
        // unselected joint; it does not promise per-owner fault isolation.
        assert_eq!(owners.map(|(entity, _)| snapshot(&sim, entity)), before);
        let clock = (sim.elapsed(), sim.epoch());
        let mut called = false;
        assert!(matches!(
            reset_loaded_world(&mut sim, |_| called = true),
            Err(SceneResetError::Clock(StepError::Faulted))
        ));
        assert!(!called);
        assert_eq!((sim.elapsed(), sim.epoch()), clock);
        assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
        assert_eq!(owners.map(|(entity, _)| snapshot(&sim, entity)), before);
    }
}

#[test]
fn ninth_owner_late_invalid_and_original_graph_configuration_negatives_stay_atomic() {
    let eight = (0..8)
        .map(|index| model(index, "0", ""))
        .collect::<String>();
    assert!(matches!(
        try_load(&world(&(eight.clone() + &model(8, "0", "")))),
        Err(LoadError::Unsupported {
            feature: "multiple joint-state models",
            ..
        })
    ));
    for invalid in [
        model(7, "NaN", ""),
        model(7, "0", "<joint_name>missing</joint_name>"),
        model(7, "0", "").replace("type=\"revolute\"", "type=\"prismatic\""),
        model(7, "0", "").replace("<parent>base</parent>", "<parent>missing</parent>"),
    ] {
        let first = (0..7)
            .map(|index| model(index, "0", ""))
            .collect::<String>();
        assert!(try_load(&world(&(first + &invalid))).is_err());
    }
    let single = model(0, "0", "");
    let begin = single.find("    <plugin ").unwrap();
    let end = begin + single[begin..].find("</plugin>").unwrap() + "</plugin>".len();
    let duplicate = format!(
        "{}{}{}",
        &single[..end],
        &single[begin..end],
        &single[end..]
    );
    assert!(matches!(
        try_load(&world(&duplicate)),
        Err(LoadError::Unsupported {
            feature: "multiple joint-state publishers",
            ..
        })
    ));
    assert!(
        try_load(&world(&single.replace(
            "<model name=\"arm0\" canonical_link=\"base\">",
            "<model name=\"arm0\" canonical_link=\"base\"><static>true</static>"
        )))
        .is_err()
    );
    let mut fresh = load(&world(&single));
    let id = named(&fresh, "default::arm0::base_hinge").1;
    queue_joint_effort(&mut fresh, id, 1.0).unwrap();
    fresh.step_exact(1).unwrap();
    geometry(&fresh, owner(&fresh, 0).0);
}
