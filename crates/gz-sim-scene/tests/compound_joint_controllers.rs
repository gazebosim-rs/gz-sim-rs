// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Existing motor ownership and reduced chains compose with flat primitive unions.
use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, JointState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, EntityKind, JointControllerCommandError, JointControllerConfiguration, JointHandle,
    JointSample, LoadError, LocalPose, ModelState, Name, PhysicsEngine, WorldEditError, WorldPose,
    initial_joint_controller_configurations_with_identity, joint_controller_binding_matches,
    load_world_with_sdf_step, model_state, prepare_model_template, remove_model,
    reset_loaded_world, set_joint_controller_command, set_model_state,
};
use std::{collections::BTreeMap, time::Duration};

const POSITION: &str = "<plugin name='gz::sim::systems::JointPositionController' filename='gz-sim-joint-position-controller-system'><joint_name>ROOT</joint_name><use_velocity_commands>true</use_velocity_commands><cmd_max>2</cmd_max><initial_position>0</initial_position></plugin>";
const VELOCITY: &str = "<plugin name='gz::sim::systems::JointController' filename='gz-sim-joint-controller-system'><joint_name>ROOT</joint_name><initial_velocity>0</initial_velocity></plugin>";
fn inertia(moment: f64) -> String {
    format!(
        "<inertial><mass>2</mass><inertia><ixx>{moment}</ixx><iyy>{moment}</iyy><izz>{moment}</izz></inertia></inertial>"
    )
}
fn collisions(count: usize, sign: f64, density: f64) -> String {
    (0..count).map(|i| {
        let geometry = match i % 4 {
            0 => "<sphere><radius>0.5</radius></sphere>",
            1 => "<cylinder><radius>0.5</radius><length>3</length></cylinder>",
            2 => "<box><size>1 0.5 1.5</size></box>",
            _ => "<capsule><radius>0.5</radius><length>1.5</length></capsule>",
        };
        format!("<collision name='c{i}'><pose>2 0 {} 0 0 0.3</pose><density>{density}</density><geometry>{geometry}</geometry></collision>",sign*4.*i as f64)
    }).collect()
}
fn model(
    name: &str,
    counts: [usize; 2],
    swapped: bool,
    moment: f64,
    density: f64,
    pose: &str,
) -> String {
    let root = if swapped { VELOCITY } else { POSITION }.replace("ROOT", "root");
    let tip = if swapped { POSITION } else { VELOCITY }.replace("ROOT", "tip");
    format!(
        "<model name='{name}' canonical_link='base'><pose>{pose}</pose><link name='base'>{}{}<visual name='v'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></visual></link><link name='child'><pose>0 0 4 0 0 0</pose>{}{}<visual name='v'><geometry><sphere><radius>0.1</radius></sphere></geometry></visual></link><joint name='root' type='revolute'><parent>world</parent><child>base</child><axis><xyz>0 0 1</xyz><limit><effort>100</effort><velocity>4</velocity></limit></axis></joint><joint name='tip' type='revolute'><parent>base</parent><child>child</child><axis><xyz>0 0 1</xyz><limit><effort>100</effort><velocity>4</velocity></limit></axis></joint>{root}{tip}</model>",
        inertia(moment),
        collisions(counts[0], -1., density),
        inertia(moment),
        collisions(counts[1], 1., density)
    )
}
fn world(models: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn load(text: &str) -> Simulation {
    load_world_with_sdf_step(
        text,
        "compound-controller",
        "w",
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
fn body(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .get::<BodyHandle>(named(sim, name))
        .unwrap()
        .id()
}
fn state(sim: &Simulation, model: &str, joint: &str) -> JointState {
    let e = named(sim, &format!("w::{model}::{joint}"));
    sim.world()
        .resource::<PhysicsBackend>()
        .joint_state(sim.world().get::<JointHandle>(e).unwrap().id())
        .unwrap()
}
fn command(sim: &mut Simulation, model: &str, joint: &str, target: f64) {
    let key = id(sim, &format!("w::{model}::{joint}"));
    set_joint_controller_command(sim, key, target).unwrap();
}
fn ids(sim: &Simulation) -> BTreeMap<u64, Entity> {
    sim.world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(|id| (id.get(), e.id())))
        .collect()
}
fn bodies(sim: &Simulation) -> BTreeMap<u64, BodyState> {
    sim.world()
        .iter_entities()
        .filter_map(|e| {
            Some((
                e.get::<EntityId>()?.get(),
                sim.world()
                    .resource::<PhysicsBackend>()
                    .state(e.get::<BodyHandle>()?.id())?,
            ))
        })
        .collect()
}
fn near(a: f64, b: f64) {
    assert!(
        a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
        "{a} != {b}"
    );
}
fn vector(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        near(a, b);
    }
}
fn query(
    sim: &Simulation,
    origin: DVec3,
    direction: DVec3,
    max: f64,
    exclude: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin.to_array(), direction.to_array(), max, exclude)
        .unwrap()
}
fn check_members(
    sim: &Simulation,
    name: &str,
    counts: [usize; 2],
    initial_position: [f64; 3],
    source_q: DQuat,
) {
    let model_pose = *sim
        .world()
        .get::<WorldPose>(named(sim, &format!("w::{name}")))
        .unwrap();
    let pivot = DVec3::from_array(initial_position);
    vector(model_pose.translation, initial_position);
    // Compare the canonical anchor to the authored model frame and measured
    // joint coordinate, not to the commanded target or to itself.
    vector(
        (DQuat::from_array(model_pose.rotation) * DVec3::X).to_array(),
        (source_q * DQuat::from_rotation_z(state(sim, name, "root").position) * DVec3::X)
            .to_array(),
    );
    for (link, count, sign, angle, z) in [
        (
            "base",
            counts[0],
            -1.,
            state(sim, name, "root").position,
            0.,
        ),
        (
            "child",
            counts[1],
            1.,
            state(sim, name, "root").position + state(sim, name, "tip").position,
            4.,
        ),
    ] {
        let link_name = format!("w::{name}::{link}");
        let link_pose = *sim
            .world()
            .get::<WorldPose>(named(sim, &link_name))
            .unwrap();
        let q = DQuat::from_array(link_pose.rotation);
        let expected_q = source_q * DQuat::from_rotation_z(angle);
        vector(
            link_pose.translation,
            (pivot + source_q * DVec3::Z * z).to_array(),
        );
        vector(
            (q * DVec3::X).to_array(),
            (expected_q * DVec3::X).to_array(),
        );
        let handle = body(sim, &link_name);
        for i in 0..count {
            let e = named(sim, &format!("{link_name}::c{i}"));
            let local = sim.world().get::<LocalPose>(e).unwrap().0;
            vector(local.translation, [2., 0., sign * 4. * i as f64]);
            let center = DVec3::from_array(link_pose.translation)
                + q * DVec3::new(2., 0., sign * 4. * i as f64);
            let shape_q = q * DQuat::from_rotation_z(0.3);
            let actual = sim.world().get::<WorldPose>(e).unwrap();
            vector(actual.translation, center.to_array());
            vector(
                (DQuat::from_array(actual.rotation) * DVec3::X).to_array(),
                (shape_q * DVec3::X).to_array(),
            );
            // Radial rays remain in this child's axial lane. All children of
            // both links are separated by four units along the shared axis.
            let origin = center + shape_q * DVec3::X * 2.;
            let direction = shape_q * (-DVec3::X);
            let hit = query(sim, origin, direction, 3., None).unwrap();
            near(hit, 1.5);
            // Reuse the represented geometric hit as the inclusive bound; an
            // ideal rounded endpoint under a composed frame is not that bound.
            assert_eq!(query(sim, origin, direction, hit, None), Some(hit));
            assert!(query(sim, origin, direction, hit.next_down(), None).is_none());
            near(
                query(sim, center, shape_q * DVec3::X, 0.125, None).unwrap(),
                0.,
            );
            assert!(
                query(
                    sim,
                    center + shape_q * DVec3::X * 2.,
                    shape_q * (-DVec3::X),
                    3.,
                    Some(handle)
                )
                .is_none()
            );
            if i % 4 == 1 {
                near(
                    query(
                        sim,
                        center + shape_q * DVec3::X * 4. + shape_q * DVec3::Y * 0.3,
                        shape_q * (-DVec3::X),
                        5.,
                        None,
                    )
                    .unwrap(),
                    3.6,
                );
                near(
                    query(sim, center + q * DVec3::Z * 2., q * (-DVec3::Z), 3., None).unwrap(),
                    0.5,
                );
                near(query(sim, center, q * DVec3::X, 0.125, None).unwrap(), 0.);
            } else if i % 4 == 3 {
                near(
                    query(sim, center + q * DVec3::Z * 2., q * (-DVec3::Z), 3., None).unwrap(),
                    0.75,
                );
            }
        }
        if count >= 2 {
            let gap = DVec3::from_array(link_pose.translation) + q * DVec3::new(5., 0., sign * 2.);
            assert!(query(sim, gap, q * (-DVec3::X), 6., None).is_none());
        }
    }
}

#[test]
fn compound_controller_modes_move_mixed_endpoints_and_all_child_query_frames() {
    for counts in [[2, 1], [1, 4], [4, 4], [16, 16]] {
        for swapped in [false, true] {
            let text = world(&model(
                "arm",
                counts,
                swapped,
                2.,
                1000.,
                "3 -2 8 0.2 -0.3 0.4",
            ));
            let mut sim = load(&text);
            let saved = ids(&sim);
            assert_eq!(
                initial_joint_controller_configurations_with_identity(&sim)
                    .unwrap()
                    .count(),
                2
            );
            check_members(
                &sim,
                "arm",
                counts,
                [3., -2., 8.],
                DQuat::from_euler(glam::EulerRot::ZYX, 0.4, -0.3, 0.2),
            );
            let before = bodies(&sim);
            for joint in ["root", "tip"] {
                assert!(joint_controller_binding_matches(
                    &sim,
                    id(&sim, &format!("w::arm::{joint}"))
                ));
                command(
                    &mut sim,
                    "arm",
                    joint,
                    if joint == "root" { 0.4 } else { -0.5 },
                );
            }
            assert_eq!(before, bodies(&sim));
            sim.step_exact(1000).unwrap();
            assert!(state(&sim, "arm", "root").position > 0.05);
            assert!(state(&sim, "arm", "tip").position < -0.05);
            for joint in ["root", "tip"] {
                let sample = sim
                    .world()
                    .get::<JointSample>(named(&sim, &format!("w::arm::{joint}")))
                    .unwrap();
                assert_eq!(sample.state, state(&sim, "arm", joint));
                assert_eq!((sample.time, sample.epoch), (Duration::from_secs(1), 0));
            }
            assert_eq!(ids(&sim), saved);
            check_members(
                &sim,
                "arm",
                counts,
                [3., -2., 8.],
                DQuat::from_euler(glam::EulerRot::ZYX, 0.4, -0.3, 0.2),
            );
        }
    }
}

#[test]
fn compound_controller_continuous_position_and_velocity_targets_keep_real_pivots() {
    for sign in [-1., 1.] {
        let mut sim = load(&world(&model(
            "arm",
            [4, 4],
            false,
            2.,
            1000.,
            "0 0 0 0 0 0",
        )));
        let wanted = sign * (4. * std::f64::consts::PI + 0.3);
        let before = bodies(&sim);
        command(&mut sim, "arm", "root", wanted);
        command(&mut sim, "arm", "tip", sign * 0.5);
        assert_eq!(state(&sim, "arm", "root").position, 0.);
        assert_eq!(bodies(&sim), before);
        sim.step_exact(30_000).unwrap();
        assert!((state(&sim, "arm", "root").position - wanted).abs() < 0.01);
        assert!((state(&sim, "arm", "tip").velocity - sign * 0.5).abs() < 0.01);
        assert!(state(&sim, "arm", "tip").position * sign > std::f64::consts::TAU);
        vector(
            sim.world()
                .get::<WorldPose>(named(&sim, "w::arm::base"))
                .unwrap()
                .translation,
            [0.; 3],
        );
        vector(
            sim.world()
                .get::<WorldPose>(named(&sim, "w::arm::child"))
                .unwrap()
                .translation,
            [0., 0., 4.],
        );
        check_members(&sim, "arm", [4, 4], [0.; 3], DQuat::IDENTITY);
    }
}

#[test]
fn compound_motor_caps_use_explicit_inertia_and_ignore_child_density() {
    let source = model("arm", [4, 4], true, 2., 1., "0 0 0 0 0 0");
    // Drive only the world-root motor; the unpowered second DoF is free.
    let start = source.rfind("<plugin").unwrap();
    let mut root_only = format!("{}</model>", &source[..start]);
    let tip_start = root_only.find("<joint name='tip'").unwrap();
    let tip_end = tip_start + root_only[tip_start..].find("</joint>").unwrap() + "</joint>".len();
    let passive_tip = root_only[tip_start..tip_end].replace(
        "<limit><effort>100</effort><velocity>4</velocity></limit>",
        "",
    );
    assert_ne!(passive_tip, &root_only[tip_start..tip_end]);
    root_only.replace_range(tip_start..tip_end, &passive_tip);
    let low = world(&root_only.replace("<effort>100</effort>", "<effort>0.2</effort>"));
    let dense = low.replace("<density>1</density>", "<density>1000000</density>");
    let heavy = low.replace(
        "<ixx>2</ixx><iyy>2</iyy><izz>2</izz>",
        "<ixx>4</ixx><iyy>4</iyy><izz>4</izz>",
    );
    let mut a = load(&low);
    let mut b = load(&dense);
    let mut c = load(&heavy);
    for sim in [&mut a, &mut b, &mut c] {
        command(sim, "arm", "root", 100.);
        sim.step_exact(1).unwrap();
    }
    let speed = state(&a, "arm", "root").velocity;
    let heavier = state(&c, "arm", "root").velocity;
    assert!(speed > 0. && speed <= 0.2 / 2. * 0.001 + 1e-12, "{speed}");
    assert!(
        heavier > 0. && heavier <= 0.2 / 4. * 0.001 + 1e-12,
        "{heavier}"
    );
    assert!(heavier < speed);
    near(
        state(&a, "arm", "root").position,
        state(&b, "arm", "root").position,
    );
    near(speed, state(&b, "arm", "root").velocity);
    assert_eq!(bodies(&a), bodies(&b));
    assert_eq!(
        ids(&a).keys().collect::<Vec<_>>(),
        ids(&b).keys().collect::<Vec<_>>()
    );
}

#[test]
fn compound_controller_pause_invalid_commands_and_resets_restore_original_targets() {
    let text = world(&model("arm", [4, 4], false, 2., 1000., "0 0 0 0 0 0"));
    let mut sim = load(&text);
    let keys = ids(&sim).keys().copied().collect::<Vec<_>>();
    let original = bodies(&sim);
    sim.set_paused(true);
    for joint in ["root", "tip"] {
        command(&mut sim, "arm", joint, -1.);
        command(&mut sim, "arm", joint, 0.6);
        let key = id(&sim, &format!("w::arm::{joint}"));
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(matches!(
                set_joint_controller_command(&mut sim, key, value),
                Err(JointControllerCommandError::InvalidCommand)
            ));
        }
    }
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(bodies(&sim), original);
    sim.set_paused(false);
    sim.step_exact(1000).unwrap();
    assert!(
        state(&sim, "arm", "root").position > 0.05 && state(&sim, "arm", "tip").position > 0.05
    );
    sim.reset().unwrap();
    assert_eq!(ids(&sim).keys().copied().collect::<Vec<_>>(), keys);
    near(state(&sim, "arm", "root").position, 0.);
    near(state(&sim, "arm", "tip").velocity, 0.);
    check_members(&sim, "arm", [4, 4], [0.; 3], DQuat::IDENTITY);
    sim.step_exact(100).unwrap();
    assert!(
        state(&sim, "arm", "root").position.abs() < 1e-11
            && state(&sim, "arm", "tip").position.abs() < 1e-11
    );
    command(&mut sim, "arm", "root", 0.6);
    command(&mut sim, "arm", "tip", -0.5);
    sim.step_exact(100).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(ids(&sim).keys().copied().collect::<Vec<_>>(), keys);
    assert_eq!(sim.epoch(), 2);
    check_members(&sim, "arm", [4, 4], [0.; 3], DQuat::IDENTITY);
    sim.step_exact(100).unwrap();
    near(state(&sim, "arm", "root").position, 0.);
    near(state(&sim, "arm", "tip").velocity, 0.);
    // Bare backend reconstruction proves collision geometry and clear motors;
    // Scene/controller target reconstruction is covered by the resets above.
    let initial_physics = sim.world().resource::<PhysicsBackend>().initial_state();
    let original_handles = [body(&sim, "w::arm::base"), body(&sim, "w::arm::child")];
    command(&mut sim, "arm", "root", 0.5);
    command(&mut sim, "arm", "tip", 0.5);
    sim.step_exact(100).unwrap();
    assert!(state(&sim, "arm", "root").position > 0.);
    let prepared = sim
        .world()
        .resource::<PhysicsBackend>()
        .prepare_initial(&initial_physics)
        .unwrap();
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_initial(prepared)
        .unwrap();
    assert_eq!(
        [body(&sim, "w::arm::base"), body(&sim, "w::arm::child")],
        original_handles
    );
    near(state(&sim, "arm", "root").position, 0.);
    near(state(&sim, "arm", "tip").velocity, 0.);
    let q = DQuat::from_rotation_z(0.3);
    near(
        query(
            &sim,
            DVec3::new(2., 0., 0.) + q * DVec3::X * 2.,
            q * (-DVec3::X),
            3.,
            None,
        )
        .unwrap(),
        1.5,
    );
}

#[test]
fn compound_controller_removal_keeps_other_owners_ids_and_reset_reservations() {
    let text = world(
        &(model("left", [4, 4], false, 2., 1000., "0 0 0 0 0 0")
            + &model("right", [2, 4], true, 2., 1000., "100 0 0 0 0 0")),
    );
    let mut sim = load(&text);
    let originals = ids(&sim).keys().copied().collect::<Vec<_>>();
    let left = id(&sim, "w::left");
    let right = id(&sim, "w::right");
    let old_joint = id(&sim, "w::left::root");
    let survivor = id(&sim, "w::right::root");
    command(&mut sim, "left", "root", 0.5);
    command(&mut sim, "right", "root", -0.5);
    sim.step_exact(100).unwrap();
    remove_model(&mut sim, left).unwrap();
    assert!(matches!(
        set_joint_controller_command(&mut sim, old_joint, 1.),
        Err(JointControllerCommandError::MissingController(_))
    ));
    assert!(joint_controller_binding_matches(&sim, survivor));
    assert!(
        initial_joint_controller_configurations_with_identity(&sim)
            .unwrap()
            .any(|row| row.id == old_joint)
    );
    let before = state(&sim, "right", "root");
    sim.step_exact(100).unwrap();
    assert!(state(&sim, "right", "root").position < before.position);
    check_members(&sim, "right", [2, 4], [100., 0., 0.], DQuat::IDENTITY);
    remove_model(&mut sim, right).unwrap();
    sim.step_exact(1).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(ids(&sim).keys().copied().collect::<Vec<_>>(), originals);
    assert!(joint_controller_binding_matches(&sim, old_joint));
    assert!(joint_controller_binding_matches(&sim, survivor));
    let fresh = sim.world_mut().spawn_empty().id();
    let assigned = gz_sim_core::assign_id(sim.world_mut(), fresh).unwrap();
    assert!(assigned > *originals.last().unwrap());
}

#[test]
fn compound_joint_motors_move_external_contacts_while_quiet_controls_stay_put() {
    let arm = model("arm", [2, 2], true, 2., 1000., "0 0 0 0 0 0");
    let probe = |x: f64| {
        format!(
            "<model name='probe'><pose>{x} 0.8 0 0 0 0</pose><link name='body'>{}<collision name='c'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision><visual name='v'><geometry><sphere><radius>0.25</radius></sphere></geometry></visual></link></model>",
            inertia(2.)
        )
    };
    let mut touching = load(&world(&(arm.clone() + &probe(2.))));
    let mut quiet = load(&world(&(arm + &probe(20.))));
    for sim in [&mut touching, &mut quiet] {
        command(sim, "arm", "root", 0.5);
        sim.step_exact(1500).unwrap();
    }
    let p = touching
        .world()
        .get::<WorldPose>(named(&touching, "w::probe::body"))
        .unwrap()
        .translation;
    assert!(p[1] > 0.8001, "{p:?}");
    vector(
        quiet
            .world()
            .get::<WorldPose>(named(&quiet, "w::probe::body"))
            .unwrap()
            .translation,
        [20., 0.8, 0.],
    );
    vector(
        touching
            .world()
            .get::<WorldPose>(named(&touching, "w::arm::base"))
            .unwrap()
            .translation,
        [0.; 3],
    );
    assert!(state(&touching, "arm", "root").position > 0.05);
}

#[test]
fn compound_controller_closed_domains_and_binding_faults_preserve_pending_physics() {
    let source = model("arm", [4, 4], false, 2., 1000., "0 0 0 0 0 0");
    let valid = world(&source);
    for bad in [
        valid.replace("<pose>0 0 4 0 0 0</pose>", "<pose>0 0 4 0.1 0 0</pose>"),
        valid.replace("<xyz>0 0 1</xyz>", "<xyz>1 0 0</xyz>"),
        valid.replace("<effort>100</effort>", "<effort>0</effort>"),
        valid.replace("<velocity>4</velocity>", "<velocity>0</velocity>"),
        valid.replace("<parent>world</parent>", "<parent>child</parent>"),
        valid.replace("<inertial>", "<inertial><pose>0.1 0 0 0 0 0</pose>"),
        valid.replace(
            "<link name='base'>",
            "<static>true</static><link name='base'>",
        ),
        world(&model("arm", [17, 2], false, 2., 1000., "0 0 0 0 0 0")),
        valid.replace(
            "<sphere><radius>0.5</radius></sphere>",
            "<plane><normal>0 0 1</normal><size>1 1</size></plane>",
        ),
        valid.replace(
            "<capsule><radius>0.5</radius><length>1.5</length></capsule>",
            "<mesh><uri>missing-m88.gltf</uri></mesh>",
        ),
        valid.replace(
            "<joint_name>tip</joint_name>",
            "<joint_name>root</joint_name>",
        ),
        valid.replace(
            "<use_velocity_commands>true</use_velocity_commands>",
            "<use_velocity_commands>false</use_velocity_commands>",
        ),
    ] {
        assert_ne!(bad, valid);
        assert!(
            load_world_with_sdf_step(
                &bad,
                "m88-refusal",
                "w",
                &sdformat::NoResources,
                PhysicsEngine::Rapier
            )
            .is_err()
        );
    }
    assert!(
        load_world_with_sdf_step(
            &valid,
            "m88-tpe",
            "w",
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        )
        .is_err()
    );
    for fault in 0..5 {
        let mut sim = load(&valid);
        let joint = named(&sim, "w::arm::root");
        let key = id(&sim, "w::arm::root");
        let saved = sim
            .world()
            .get::<JointControllerConfiguration>(joint)
            .unwrap()
            .clone();
        command(&mut sim, "arm", "root", 0.6);
        command(&mut sim, "arm", "tip", 0.5);
        let before = bodies(&sim);
        let clock = (sim.elapsed(), sim.epoch());
        match fault {
            0 => {
                let mut changed = saved.clone();
                changed.topic = "/forged/controller".into();
                sim.world_mut().entity_mut(joint).insert(changed);
            }
            1 => {
                let other = *sim
                    .world()
                    .get::<JointHandle>(named(&sim, "w::arm::tip"))
                    .unwrap();
                sim.world_mut().entity_mut(joint).insert(other);
            }
            2 => {
                sim.world_mut()
                    .entity_mut(joint)
                    .insert(Name("w::arm::forged".into()));
            }
            3 => {
                sim.world_mut().entity_mut(joint).remove::<JointSample>();
            }
            _ => {
                sim.world_mut()
                    .entity_mut(joint)
                    .insert(EntityKind::Collision);
            }
        }
        assert!(matches!(
            set_joint_controller_command(&mut sim, key, -0.5),
            Err(JointControllerCommandError::InvalidPhysics)
        ));
        assert_eq!(bodies(&sim), before);
        assert_eq!((sim.elapsed(), sim.epoch()), clock);
        assert!(sim.step_exact(1).is_err());
        assert_eq!(bodies(&sim), before);
        assert_eq!((sim.elapsed(), sim.epoch()), clock);
    }
    let peers = world(
        &(model("left", [2, 2], false, 2., 1000., "0 0 0 0 0 0")
            + &model("right", [2, 2], false, 2., 1000., "100 0 0 0 0 0")),
    );
    let mut pair = load(&peers);
    command(&mut pair, "left", "root", 0.4);
    command(&mut pair, "right", "root", 0.5);
    let later = named(&pair, "w::right::root");
    let saved = pair
        .world()
        .get::<JointControllerConfiguration>(later)
        .unwrap()
        .clone();
    let mut drift = saved.clone();
    drift.topic = "/forged/later".into();
    pair.world_mut().entity_mut(later).insert(drift);
    let before = bodies(&pair);
    let earlier = id(&pair, "w::left::root");
    assert!(matches!(
        set_joint_controller_command(&mut pair, earlier, -0.5),
        Err(JointControllerCommandError::InvalidPhysics)
    ));
    assert_eq!(bodies(&pair), before);
    assert_eq!(pair.elapsed(), Duration::ZERO);
    pair.world_mut().entity_mut(later).insert(saved);
    pair.step_exact(1000).unwrap();
    assert!(
        state(&pair, "left", "root").position > 0.05
            && state(&pair, "right", "root").position > 0.05
    );
    let mut sim = load(&valid);
    let owner = id(&sim, "w::arm");
    let parsed = sdformat::load_str(&valid, "m88-templates", &sdformat::NoResources).unwrap();
    assert!(matches!(
        prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert!(matches!(
        model_state(&sim, owner),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert!(matches!(
        set_model_state(
            &mut sim,
            owner,
            ModelState {
                translation: [0.; 3],
                rotation: [0., 0., 0., 1.],
                linear_velocity: [0.; 3],
                angular_velocity: [0.; 3]
            }
        ),
        Err(WorldEditError::UnsupportedModel)
    ));
    let unknown = u64::MAX;
    assert!(matches!(
        set_joint_controller_command(&mut sim, unknown, 1.),
        Err(JointControllerCommandError::MissingController(_))
    ));
    assert!(matches!(
        load_world_with_sdf_step(
            &world(&model("arm", [0, 2], false, 2., 1000., "0 0 0 0 0 0")),
            "m88-zero",
            "w",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported { .. })
    ));
}
