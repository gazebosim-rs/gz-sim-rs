// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, StepError, StepFault, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, LoadError, LocalPose, Name, PhysicsEngine, PosePublication,
    PoseSample, SceneResetError, StaticPoseSample, WorldPose, initial_pose_publications,
    load_world_with_sdf_step, queue_link_world_wrench, remove_model, reset_loaded_world,
};
use std::time::Duration;

const ARTICULATED: &str = include_str!("fixtures/m31_pose_static_publisher.sdf");
type Frame = (u64, [f64; 3], [f64; 4]);
type Snapshot = (Duration, u64, u64, Vec<Frame>);

fn world(models: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='step' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn plugin(owner: usize, rate: &str, static_rate: Option<&str>, selection: &str) -> String {
    let split = static_rate.map(|rate| format!("<static_publisher>true</static_publisher><static_update_frequency>{rate}</static_update_frequency>")).unwrap_or_default();
    format!(
        "<plugin filename='gz-sim-pose-publisher-system' name='gz::sim::systems::PosePublisher'><use_pose_vector_msg>true</use_pose_vector_msg>{selection}<topic>/m55/pose{owner}</topic><update_frequency>{rate}</update_frequency>{split}</plugin>"
    )
}
fn articulated(owner: usize, rate: &str, static_rate: &str) -> String {
    let start = ARTICULATED.find("  <model ").unwrap();
    let end = ARTICULATED.rfind("</model>").unwrap() + "</model>".len();
    ARTICULATED[start..end]
        .replace("rig", &format!("rig{owner}"))
        .replace(
            "<pose>1 2 3 0 0 0.6</pose>",
            &format!("<pose>1 {} 3 0 0 0.6</pose>", 2 + owner * 4),
        )
        .replace("/m31/pose", &format!("/m55/pose{owner}"))
        .replace(
            "<update_frequency>100</update_frequency>",
            &format!("<update_frequency>{rate}</update_frequency>"),
        )
        .replace(
            "<static_update_frequency>25</static_update_frequency>",
            &format!("<static_update_frequency>{static_rate}</static_update_frequency>"),
        )
}
fn jointless(owner: usize, links: usize, fixed: bool, config: &str) -> String {
    let mut body = String::new();
    for link in 0..links {
        body.push_str(&format!("<link name='body{link}'><pose>0 {} 0 0 0 0</pose><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='box'><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link>", link as f64 * 0.2));
    }
    format!(
        "<model name='rig{owner}'><static>{fixed}</static><pose>1 {} 3 0 0 .3</pose>{body}{config}</model>",
        owner * 40
    )
}
fn try_load(xml: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "m55-multiple-pose-publishers",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
}
fn load(xml: &str) -> Simulation {
    try_load(xml).unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap();
    (entity.id(), entity.get::<EntityId>().unwrap().get())
}
fn owner(sim: &Simulation, index: usize) -> (Entity, u64) {
    named(sim, &format!("default::rig{index}"))
}
fn snapshot(sim: &Simulation, entity: Entity, fixed: bool) -> Option<Snapshot> {
    let value = if fixed {
        &sim.world().get::<StaticPoseSample>(entity)?.0
    } else {
        sim.world().get::<PoseSample>(entity)?
    };
    Some((
        value.time,
        value.epoch,
        value.revision,
        value
            .poses
            .iter()
            .map(|frame| (frame.identity, frame.pose.translation, frame.pose.rotation))
            .collect(),
    ))
}
fn geometry(sim: &Simulation, entity: Entity, fixed: bool) {
    let publication = sim.world().get::<PosePublication>(entity).unwrap();
    let frames = if fixed {
        &publication.static_publication.as_ref().unwrap().poses
    } else {
        &publication.poses
    };
    let sample = snapshot(sim, entity, fixed).unwrap();
    assert_eq!(sample.0, sim.elapsed());
    assert_eq!(sample.3.len(), frames.len());
    assert!(
        frames
            .windows(2)
            .all(|pair| pair[0].child_frame_id < pair[1].child_frame_id)
    );
    for (expected, (identity, translation, rotation)) in frames.iter().zip(sample.3) {
        assert_eq!(expected.identity, identity);
        let local = sim
            .world()
            .get::<LocalPose>(entity_by_id(sim.world(), identity).unwrap())
            .unwrap()
            .0;
        // Parent-relative values are copied by the unchanged sampler, not
        // reconstructed from WORLD poses or compared at a weaker tolerance.
        assert_eq!(translation, local.translation);
        assert_eq!(rotation, local.rotation);
    }
}

#[test]
fn distinct_owners_real_wrenches_and_joint_motors_preserve_fresh_local_frames_and_loaded_reset() {
    let xml = world(&(articulated(0, "100", "25") + &articulated(1, "50", "100")));
    let mut sim = load(&xml);
    let owners = [owner(&sim, 0), owner(&sim, 1)];
    let metadata: Vec<_> = owners
        .iter()
        .map(|(entity, _)| sim.world().get::<PosePublication>(*entity).unwrap().clone())
        .collect();
    let tips = [
        named(&sim, "default::rig0::tip"),
        named(&sim, "default::rig1::tip"),
    ];
    let initial: Vec<_> = tips
        .iter()
        .map(|(entity, _)| *sim.world().get::<WorldPose>(*entity).unwrap())
        .collect();
    let mut joints = Vec::new();
    for (index, (_, tip)) in tips.iter().enumerate() {
        // The queued Scene API intentionally admits only free single-link models.
        assert!(matches!(
            queue_link_world_wrench(
                &mut sim,
                *tip,
                [2.0, 3.0, 0.0],
                [0.0, 0.0, 0.5 + index as f64],
            ),
            Err(gz_sim_scene::InputError::NotFreeLink(id)) if id == *tip
        ));
        // This articulated-motion witness uses the existing public physics API,
        // not an expansion of queued Scene input admission.
        let body = sim.world().get::<BodyHandle>(tips[index].0).unwrap().id();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_world_wrench(body, [2.0, 3.0, 0.0], [0.0, 0.0, 0.5 + index as f64])
            .unwrap();
        let joint = named(&sim, &format!("default::rig{index}::tip_hinge")).0;
        let handle = sim.world().get::<JointHandle>(joint).unwrap().id();
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_joint_velocity_motor(
                handle,
                Some(JointVelocityMotor {
                    target_velocity: 0.5,
                    max_force: 0.5,
                }),
            )
            .unwrap();
        joints.push(handle);
    }
    let mut held = [[None, None], [None, None]];
    let mut stamps = [[Vec::new(), Vec::new()], [Vec::new(), Vec::new()]];
    for _ in 0..80 {
        sim.step_exact(1).unwrap();
        for (index, (entity, _)) in owners.iter().enumerate() {
            for (channel, fixed) in [false, true].into_iter().enumerate() {
                let value = snapshot(&sim, *entity, fixed);
                if value.as_ref().is_some_and(|value| value.0 == sim.elapsed()) {
                    geometry(&sim, *entity, fixed);
                    stamps[index][channel].push(sim.elapsed());
                } else {
                    assert_eq!(value, held[index][channel]);
                }
                held[index][channel] = value;
            }
        }
    }
    assert_eq!(
        stamps[0][0],
        (1..=8)
            .map(|n| Duration::from_millis(n * 10))
            .collect::<Vec<_>>()
    );
    assert_eq!(stamps[0][1], [40, 80].map(Duration::from_millis));
    assert_eq!(stamps[1][0], [20, 40, 60, 80].map(Duration::from_millis));
    assert_eq!(
        stamps[1][1],
        (1..=8)
            .map(|n| Duration::from_millis(n * 10))
            .collect::<Vec<_>>()
    );
    for (index, handle) in joints.into_iter().enumerate() {
        let state = sim
            .world()
            .resource::<PhysicsBackend>()
            .joint_state(handle)
            .unwrap();
        assert!(state.position.abs() > 0.0 && state.velocity.abs() > 0.0);
        let moved = sim.world().get::<WorldPose>(tips[index].0).unwrap();
        assert_ne!(moved.translation, initial[index].translation);
        assert_ne!(moved.rotation, initial[index].rotation);
        assert_eq!(
            sim.world().get::<PosePublication>(owners[index].0).unwrap(),
            &metadata[index]
        );
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let mut fresh = load(&xml);
    for (old, identity) in owners {
        let restored = entity_by_id(sim.world(), identity).unwrap();
        assert_ne!(restored, old);
        assert!(
            snapshot(&sim, restored, false).is_none() && snapshot(&sim, restored, true).is_none()
        );
    }
    sim.step_exact(40).unwrap();
    fresh.step_exact(40).unwrap();
    for index in 0..2 {
        for fixed in [false, true] {
            let reset = snapshot(&sim, owner(&sim, index).0, fixed).unwrap();
            let reference = snapshot(&fresh, owner(&fresh, index).0, fixed).unwrap();
            assert_eq!(
                (reset.0, reset.2, &reset.3),
                (reference.0, reference.2, &reference.3)
            );
            assert_eq!((reset.1, reference.1), (1, 0));
        }
    }
}

#[test]
fn absent_single_eight_and_maximum_1024_selected_records_keep_one_schedule_pair() {
    let absent = load(&world(""));
    let base = absent.scheduled_system_counts();
    for count in [1, 2, 8] {
        let models: String = (0..count)
            .map(|index| {
                jointless(
                    index,
                    127,
                    true,
                    &plugin(
                        index,
                        "0",
                        Some("0"),
                        "<publish_model_pose>true</publish_model_pose>",
                    ),
                )
            })
            .collect();
        let mut sim = load(&world(&models));
        assert_eq!(sim.scheduled_system_counts(), (base.0 + 1, base.1 + 1));
        assert_eq!(initial_pose_publications(&sim).unwrap().count(), count);
        sim.step_exact(1).unwrap();
        let mut selected = 0;
        for index in 0..count {
            let entity = owner(&sim, index).0;
            let metadata = sim.world().get::<PosePublication>(entity).unwrap();
            assert!(metadata.poses.is_empty());
            assert_eq!(
                metadata.static_publication.as_ref().unwrap().poses.len(),
                128
            );
            assert!(snapshot(&sim, entity, false).is_none());
            geometry(&sim, entity, true);
            selected += snapshot(&sim, entity, true).unwrap().3.len();
        }
        assert_eq!(selected, count * 128);
    }
    assert_eq!(initial_pose_publications(&absent).unwrap().count(), 0);
}

#[test]
fn per_owner_clocks_ignore_interest_preserve_pause_and_distinguish_core_from_loaded_reset() {
    let xml = world(&(articulated(0, "100", "25") + &articulated(1, "50", "100")));
    // There is deliberately no subscriber-interest API or transport consumer.
    let mut sim = load(&xml);
    sim.step_exact(40).unwrap();
    let before: Vec<_> = (0..2)
        .map(|index| {
            [
                snapshot(&sim, owner(&sim, index).0, false),
                snapshot(&sim, owner(&sim, index).0, true),
            ]
        })
        .collect();
    for _ in 0..3 {
        sim.tick().unwrap();
    }
    sim.step_exact(0).unwrap();
    for (index, values) in before.iter().enumerate() {
        assert_eq!(
            [
                snapshot(&sim, owner(&sim, index).0, false),
                snapshot(&sim, owner(&sim, index).0, true)
            ],
            *values
        );
    }
    sim.reset().unwrap();
    for index in 0..2 {
        assert!(
            snapshot(&sim, owner(&sim, index).0, false).is_none()
                && snapshot(&sim, owner(&sim, index).0, true).is_none()
        );
    }
    sim.step_exact(1).unwrap();
    for index in 0..2 {
        for fixed in [false, true] {
            let value = snapshot(&sim, owner(&sim, index).0, fixed).unwrap();
            assert_eq!((value.0, value.1), (Duration::from_millis(1), 1));
            geometry(&sim, owner(&sim, index).0, fixed);
        }
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(9).unwrap();
    for index in 0..2 {
        assert!(
            snapshot(&sim, owner(&sim, index).0, false).is_none()
                && snapshot(&sim, owner(&sim, index).0, true).is_none()
        );
    }
    sim.step_exact(1).unwrap();
    assert!(snapshot(&sim, owner(&sim, 0).0, false).is_some());
    assert!(snapshot(&sim, owner(&sim, 0).0, true).is_none());
    assert!(snapshot(&sim, owner(&sim, 1).0, false).is_none());
    assert!(snapshot(&sim, owner(&sim, 1).0, true).is_some());
}

#[test]
fn selections_and_static_empty_groups_preserve_entities_and_single_owner_values() {
    for (selection, count, split) in [
        ("", 1, false),
        (
            "<publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose>",
            1,
            false,
        ),
        ("<publish_model_pose>true</publish_model_pose>", 2, false),
        ("<publish_model_pose>true</publish_model_pose>", 2, true),
    ] {
        let first = jointless(
            0,
            1,
            false,
            &plugin(0, "0", split.then_some("0"), selection),
        );
        let second = jointless(
            1,
            1,
            false,
            &plugin(
                1,
                "0",
                Some("0"),
                "<publish_model_pose>true</publish_model_pose>",
            ),
        );
        let mut single = load(&world(&first));
        let mut pair = load(&world(&(first + &second)));
        let original = owner(&single, 0).1;
        assert_eq!(original, owner(&pair, 0).1);
        let link = named(&single, "default::rig0::body0").1;
        queue_link_world_wrench(&mut single, link, [3.0, 2.0, 1.0], [0.0, 0.0, 2.0]).unwrap();
        queue_link_world_wrench(&mut pair, link, [3.0, 2.0, 1.0], [0.0, 0.0, 2.0]).unwrap();
        single.step_exact(20).unwrap();
        pair.step_exact(20).unwrap();
        let entity = owner(&pair, 0).0;
        assert!(
            pair.world()
                .get::<Name>(named(&pair, "default::rig0::body0").0)
                .is_some()
        );
        assert_eq!(
            snapshot(&single, owner(&single, 0).0, split),
            snapshot(&pair, entity, split)
        );
        assert_eq!(snapshot(&pair, entity, split).unwrap().3.len(), count);
        geometry(&pair, entity, split);
        if split {
            assert!(snapshot(&pair, entity, false).is_none());
        }
        assert!(snapshot(&pair, owner(&pair, 1).0, false).is_none());
        assert_eq!(snapshot(&pair, owner(&pair, 1).0, true).unwrap().3.len(), 2);
    }
}

#[test]
fn removal_restores_initial_owners_and_metadata_fault_keeps_global_terminal_nonmutation() {
    let a = jointless(
        0,
        1,
        false,
        &plugin(
            0,
            "0",
            Some("0"),
            "<publish_model_pose>true</publish_model_pose>",
        ),
    );
    let b = jointless(
        1,
        1,
        false,
        &plugin(
            1,
            "0",
            Some("0"),
            "<publish_model_pose>true</publish_model_pose>",
        ),
    );
    let xml = world(&(a + &b));
    let mut sim = load(&xml);
    let base = load(&world("")).scheduled_system_counts();
    let originals = [owner(&sim, 0), owner(&sim, 1)];
    let initial: Vec<_> = initial_pose_publications(&sim)
        .unwrap()
        .map(|(id, config)| (id, config.clone()))
        .collect();
    sim.step_exact(1).unwrap();
    remove_model(&mut sim, originals[0].1).unwrap();
    assert!(entity_by_id(sim.world(), originals[0].1).is_none());
    assert_eq!(sim.scheduled_system_counts(), (base.0 + 1, base.1 + 1));
    sim.step_exact(1).unwrap();
    geometry(&sim, originals[1].0, true);
    assert_eq!(snapshot(&sim, originals[1].0, true).unwrap().2, 2);
    remove_model(&mut sim, originals[1].1).unwrap();
    assert_eq!(sim.scheduled_system_counts(), base);
    assert_eq!(
        initial_pose_publications(&sim)
            .unwrap()
            .map(|(id, config)| (id, config.clone()))
            .collect::<Vec<_>>(),
        initial
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    for (old, identity) in originals {
        assert_ne!(old, entity_by_id(sim.world(), identity).unwrap());
    }
    sim.step_exact(1).unwrap();
    let victim = owner(&sim, 0).0;
    sim.world_mut()
        .get_mut::<PosePublication>(victim)
        .unwrap()
        .topic = "/forged".into();
    // A real advancing tick reaches the ordinary invalid-owner path.
    assert!(sim.step_exact(1).is_err());
    assert!(sim.world().resource::<StepFault>().error().is_some());
    assert!(snapshot(&sim, victim, false).is_none() && snapshot(&sim, victim, true).is_none());
    let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
        before
    );
}

#[test]
fn ninth_owner_late_invalid_selection_duplicate_plugin_and_names_remain_closed_admission() {
    let models: String = (0..9)
        .map(|index| jointless(index, 1, true, &plugin(index, "0", None, "")))
        .collect();
    assert!(matches!(
        try_load(&world(&models)),
        Err(LoadError::Unsupported {
            feature: "multiple pose-publisher models",
            ..
        })
    ));
    for invalid in [
        plugin(
            1,
            "0",
            None,
            "<publish_sensor_pose>true</publish_sensor_pose>",
        ),
        plugin(1, "0", None, "").replace(
            "<use_pose_vector_msg>true</use_pose_vector_msg>",
            "<use_pose_vector_msg>false</use_pose_vector_msg>",
        ),
        plugin(1, "NaN", None, ""),
    ] {
        let a = jointless(0, 1, true, &plugin(0, "0", None, ""));
        let b = jointless(1, 1, true, &invalid);
        assert!(try_load(&world(&(a + &b))).is_err());
    }
    let config = plugin(0, "0", None, "");
    assert!(try_load(&world(&jointless(0, 1, true, &(config.clone() + &config)))).is_err());
    let model = jointless(0, 1, true, &config);
    assert!(try_load(&world(&(model.clone() + &model))).is_err());
    assert!(
        try_load(&world(&jointless(
            0,
            128,
            true,
            &plugin(
                0,
                "0",
                None,
                "<publish_model_pose>true</publish_model_pose>"
            )
        )))
        .is_err()
    );
}
