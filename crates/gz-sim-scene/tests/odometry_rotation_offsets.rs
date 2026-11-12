// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation, StepError, StepFault};
use gz_sim_scene::{
    GroundTruthOdometryDimensions as Dimensions, GroundTruthOdometryError,
    GroundTruthOdometryOffsets as Offsets, GroundTruthOdometryPublication as Publication,
    GroundTruthOdometrySample as Sample, GroundTruthOdometryScheduleFault, Name, PhysicsEngine,
    SceneResetError, Velocity, WorldPose, load_world_with_sdf_step, queue_link_world_wrench,
    remove_model, reset_loaded_world,
};
use std::time::Duration;
const XML: &str = include_str!("../../../examples/m46_odometry_rotation_offsets.sdf");
const TRANSLATION_XML: &str =
    include_str!("../../../examples/m39_odometry_translation_offsets.sdf");
const PARSER_XML: &str = include_str!("../../../examples/m34_odometry_publisher.sdf");
fn xml(dimensions: &str, local: &str, global: &str) -> String {
    XML.replace("<dimensions>3</dimensions>", dimensions)
        .replace("<rpy_offset>0.3 -0.4 0.5</rpy_offset>", local)
        .replace(
            "<rpy_global_offset>-0.2 0.25 -0.35</rpy_global_offset>",
            global,
        )
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m46.sdf",
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
fn model(sim: &Simulation) -> bevy_ecs::entity::Entity {
    entity(sim, "default::rig")
}
fn body(sim: &Simulation) -> bevy_ecs::entity::Entity {
    entity(sim, "default::rig::body")
}
fn sample(sim: &Simulation) -> Sample {
    *sim.world().get::<Sample>(model(sim)).unwrap()
}
fn bits(s: Sample) -> ([u64; 3], [u64; 4], [u64; 3], [u64; 3]) {
    (
        s.pose.translation.map(f64::to_bits),
        s.pose.rotation.map(f64::to_bits),
        s.linear.map(f64::to_bits),
        s.angular.map(f64::to_bits),
    )
}
fn rotation(rpy: [f64; 3]) -> DQuat {
    DQuat::from_rotation_z(rpy[2]) * DQuat::from_rotation_y(rpy[1]) * DQuat::from_rotation_x(rpy[0])
}
fn geometry(sim: &Simulation) {
    let metadata = sim.world().get::<Publication>(model(sim)).unwrap();
    let offsets = metadata.offsets;
    let world = sim.world().get::<WorldPose>(model(sim)).unwrap();
    // Independent point/basis geometry. This is not a second native scalar
    // implementation or filtered velocity estimator; native replay comes later.
    let inverse_global = rotation(offsets.global_rotation_rpy()).inverse();
    let point = inverse_global
        * (DVec3::from_array(world.translation)
            + DQuat::from_array(world.rotation) * DVec3::from_array(offsets.local_translation())
            - DVec3::from_array(offsets.global_translation()));
    let orientation =
        inverse_global * DQuat::from_array(world.rotation) * rotation(offsets.local_rotation_rpy());
    let actual = sample(sim);
    for (i, value) in point.to_array().into_iter().enumerate() {
        if i == 2 && metadata.dimensions == Dimensions::Two {
            assert_eq!(actual.pose.translation[i], 0.0);
        } else {
            assert!((actual.pose.translation[i] - value).abs() < 1e-12);
        }
    }
    for (a, b) in actual.pose.rotation.into_iter().zip(orientation.to_array()) {
        assert!((a - b).abs() < 1e-12);
    }
    assert!((DQuat::from_array(actual.pose.rotation).length_squared() - 1.0).abs() < 1e-12);
    if metadata.dimensions == Dimensions::Two {
        assert_eq!(
            (actual.linear[2], actual.angular[0], actual.angular[1]),
            (0.0, 0.0, 0.0)
        );
    }
}
fn drive(sim: &mut Simulation) {
    let id = sim.world().get::<EntityId>(body(sim)).unwrap().get();
    queue_link_world_wrench(sim, id, [4.0, -2.0, 6.0], [2.0, 4.0, 6.0]).unwrap();
    sim.step_exact(20).unwrap();
    let velocity = sim.world().get::<Velocity>(body(sim)).unwrap();
    assert!(velocity.linear.into_iter().all(f64::is_finite));
    assert!(velocity.angular.into_iter().all(f64::is_finite));
    assert!(velocity.linear.into_iter().any(|v| v.abs() > 0.0001));
    assert!(velocity.angular.into_iter().any(|v| v.abs() > 0.0001));
}
#[test]
fn all_four_exact_old_rotation_inputs_load_move_and_healthy_reset() {
    // Preserve configured()'s exact M34 MODEL and raw plugin contents. The only
    // extra WORLD plugin supplies the already supported real wrench stimulus.
    let start = PARSER_XML.find("      <plugin").unwrap();
    let end = PARSER_XML[start..].find("</plugin>").unwrap() + start + "</plugin>".len();
    let parser_case = |content: &str| {
        let plugin = format!(
            "<plugin name='gz::sim::systems::OdometryPublisher' filename='gz-sim-odometry-publisher-system'>{content}</plugin>"
        );
        let model_xml = format!("{}{}{}", &PARSER_XML[..start], plugin, &PARSER_XML[end..]);
        model_xml.replace("<model name=\"rig\">", "<plugin name=\"gz::sim::systems::ApplyLinkWrench\" filename=\"gz-sim-apply-link-wrench-system\"/><model name=\"rig\">")
    };
    let cases = [
        (
            parser_case("<rpy_offset>0 0 1</rpy_offset>"),
            Dimensions::Two,
            [0.0, 0.0, 1.0],
            [0.0; 3],
            [0.0; 3],
            [0.0; 3],
        ),
        (
            parser_case("<rpy_global_offset>1 0 0</rpy_global_offset>"),
            Dimensions::Two,
            [0.0; 3],
            [1.0, 0.0, 0.0],
            [0.0; 3],
            [0.0; 3],
        ),
        // These two are the literal old public expressions against ORIGINAL M39.
        (
            TRANSLATION_XML.replace(
                "<dimensions>3</dimensions>",
                "<dimensions>3</dimensions><rpy_offset>0 0 0.1</rpy_offset>",
            ),
            Dimensions::Three,
            [0.0, 0.0, 0.1],
            [0.0; 3],
            [0.4, -0.3, 0.2],
            [-0.7, 0.5, -0.6],
        ),
        (
            TRANSLATION_XML.replace(
                "<dimensions>3</dimensions>",
                "<dimensions>3</dimensions><rpy_global_offset>0.1 0 0</rpy_global_offset>",
            ),
            Dimensions::Three,
            [0.0; 3],
            [0.1, 0.0, 0.0],
            [0.4, -0.3, 0.2],
            [-0.7, 0.5, -0.6],
        ),
    ];
    for (source, dimension, local, global, local_xyz, global_xyz) in cases {
        let mut sim = load(&source);
        let owner = model(&sim);
        let id = sim.world().get::<EntityId>(owner).unwrap().get();
        let metadata = sim.world().get::<Publication>(owner).unwrap().clone();
        assert_eq!(metadata.dimensions, dimension);
        assert_eq!(
            metadata.offsets,
            Offsets::from_translations(local_xyz, global_xyz)
                .unwrap()
                .with_rotations(local, global)
                .unwrap()
        );
        sim.step_exact(20).unwrap();
        let initial = sample(&sim);
        geometry(&sim);
        drive(&mut sim);
        geometry(&sim);
        assert_ne!(bits(sample(&sim)), bits(initial));
        assert_eq!(sample(&sim).time, Duration::from_millis(40));
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let owner = model(&sim);
        assert_eq!(sim.world().get::<EntityId>(owner).unwrap().get(), id);
        assert_eq!(sim.world().get::<Publication>(owner).unwrap(), &metadata);
        assert!(sim.world().get::<Sample>(owner).is_none());
        sim.step_exact(20).unwrap();
        assert_eq!(bits(sample(&sim)), bits(initial));
        assert_eq!(
            (sample(&sim).epoch, sample(&sim).revision),
            (1, initial.revision)
        );
    }
}
#[test]
fn local_global_axes_and_combined_rotations_follow_independent_geometry_in_both_modes() {
    for dimensions in [
        "",
        "<dimensions>2</dimensions>",
        "<dimensions>3</dimensions>",
    ] {
        for (local, global) in [
            ([0.4, 0.0, 0.0], [0.0; 3]),
            ([0.0, -0.3, 0.0], [0.0; 3]),
            ([0.0, 0.0, 0.6], [0.0; 3]),
            ([0.0; 3], [0.4, 0.0, 0.0]),
            ([0.0; 3], [0.0, -0.3, 0.0]),
            ([0.0; 3], [0.0, 0.0, 0.6]),
            ([0.3, -0.4, 0.5], [-0.2, 0.25, -0.35]),
        ] {
            let vector = |name, v: [f64; 3]| format!("<{name}>{} {} {}</{name}>", v[0], v[1], v[2]);
            let mut sim = load(&xml(
                dimensions,
                &vector("rpy_offset", local),
                &vector("rpy_global_offset", global),
            ));
            let metadata = sim.world().get::<Publication>(model(&sim)).unwrap();
            assert_eq!(metadata.offsets.local_rotation_rpy(), local);
            assert_eq!(metadata.offsets.global_rotation_rpy(), global);
            sim.step_exact(20).unwrap();
            geometry(&sim);
            drive(&mut sim);
            geometry(&sim);
        }
    }
}
#[test]
fn rotated_feedback_changes_coordinates_without_changing_actual_physics() {
    let mut rotated = load(XML);
    let mut untranslated_rotation = load(&xml("<dimensions>3</dimensions>", "", ""));
    rotated.step_exact(20).unwrap();
    untranslated_rotation.step_exact(20).unwrap();
    drive(&mut rotated);
    drive(&mut untranslated_rotation);
    let a = rotated.world().get::<WorldPose>(model(&rotated)).unwrap();
    let b = untranslated_rotation
        .world()
        .get::<WorldPose>(model(&untranslated_rotation))
        .unwrap();
    assert_eq!(
        a.translation.map(f64::to_bits),
        b.translation.map(f64::to_bits)
    );
    assert_eq!(a.rotation.map(f64::to_bits), b.rotation.map(f64::to_bits));
    let a = rotated.world().get::<Velocity>(body(&rotated)).unwrap();
    let b = untranslated_rotation
        .world()
        .get::<Velocity>(body(&untranslated_rotation))
        .unwrap();
    assert_eq!(a.linear.map(f64::to_bits), b.linear.map(f64::to_bits));
    assert_eq!(a.angular.map(f64::to_bits), b.angular.map(f64::to_bits));
    let reading = sample(&rotated);
    assert_ne!(
        reading.pose.rotation.map(f64::to_bits),
        sample(&untranslated_rotation)
            .pose
            .rotation
            .map(f64::to_bits)
    );
    assert_ne!(
        reading.linear.map(f64::to_bits),
        sample(&untranslated_rotation).linear.map(f64::to_bits)
    );
    assert_ne!(
        reading.angular.map(f64::to_bits),
        sample(&untranslated_rotation).angular.map(f64::to_bits)
    );
    assert_ne!(reading.linear.map(f64::to_bits), a.linear.map(f64::to_bits));
    geometry(&rotated);
}
#[test]
fn each_zero_rotation_vector_keeps_translation_only_bits_and_nonzero_signed_identity() {
    for dimensions in ["<dimensions>2</dimensions>", "<dimensions>3</dimensions>"] {
        for (local, global, signed_local, signed_global) in [
            (
                "",
                "",
                "<rpy_offset>-0 0 -0</rpy_offset>",
                "<rpy_global_offset>0 -0 -0</rpy_global_offset>",
            ),
            (
                "",
                "<rpy_global_offset>0 -0 0.3</rpy_global_offset>",
                "<rpy_offset>-0 0 -0</rpy_offset>",
                "<rpy_global_offset>0 -0 0.3</rpy_global_offset>",
            ),
            (
                "<rpy_offset>0.3 -0 0</rpy_offset>",
                "",
                "<rpy_offset>0.3 -0 0</rpy_offset>",
                "<rpy_global_offset>-0 0 -0</rpy_global_offset>",
            ),
        ] {
            let mut a = load(&xml(dimensions, local, global));
            let mut b = load(&xml(dimensions, signed_local, signed_global));
            assert_eq!(
                a.world().get::<Publication>(model(&a)).unwrap(),
                b.world().get::<Publication>(model(&b)).unwrap()
            );
            a.step_exact(20).unwrap();
            b.step_exact(20).unwrap();
            assert_eq!(bits(sample(&a)), bits(sample(&b)));
            drive(&mut a);
            drive(&mut b);
            assert_eq!(bits(sample(&a)), bits(sample(&b)));
        }
    }
    let original = Offsets::from_translations([1.0, -0.0, 0.0], [0.0; 3]).unwrap();
    assert_eq!(
        original
            .with_rotations([-0.0, 0.0, -0.0], [0.0; 3])
            .unwrap(),
        original
    );
    let signed = original.with_rotations([0.3, -0.0, 0.0], [0.0; 3]).unwrap();
    assert_eq!(
        signed.local_rotation_rpy()[1].to_bits(),
        (-0.0_f64).to_bits()
    );
    assert_ne!(
        signed,
        original.with_rotations([0.3, 0.0, 0.0], [0.0; 3]).unwrap()
    );
    assert_eq!(
        signed.local_translation().map(f64::to_bits),
        original.local_translation().map(f64::to_bits)
    );
}
#[test]
fn rotated_history_advances_before_throttle_while_pause_holds_publication() {
    for dimensions in ["<dimensions>2</dimensions>", "<dimensions>3</dimensions>"] {
        let source = XML.replace("<dimensions>3</dimensions>", dimensions);
        let mut limited = load(&source);
        let mut every = load(&source.replace(
            "<odom_publish_frequency>50</odom_publish_frequency>",
            "<odom_publish_frequency>0</odom_publish_frequency>",
        ));
        limited.step_exact(20).unwrap();
        every.step_exact(20).unwrap();
        drive(&mut limited);
        drive(&mut every);
        assert_eq!(bits(sample(&limited)), bits(sample(&every)));
        assert_ne!(sample(&limited).revision, sample(&every).revision);
        limited.set_paused(true);
        let held = sample(&limited);
        for _ in 0..3 {
            limited.tick().unwrap();
        }
        assert_eq!(bits(sample(&limited)), bits(held));
        assert_eq!(
            (sample(&limited).time, sample(&limited).revision),
            (held.time, held.revision)
        );
        assert_eq!(limited.elapsed(), Duration::from_millis(40));
    }
}
#[test]
fn bit_only_local_or_global_rotation_mutation_faults_and_terminal_reset_refuses() {
    for global in [false, true] {
        let mut sim = load(&xml(
            "<dimensions>3</dimensions>",
            "<rpy_offset>0.3 -0 0</rpy_offset>",
            "<rpy_global_offset>0.2 -0 0</rpy_global_offset>",
        ));
        sim.step_exact(20).unwrap();
        let owner = model(&sim);
        let id = sim.world().get::<EntityId>(owner).unwrap().get();
        let old = sim.world().get::<Publication>(owner).unwrap().offsets;
        let replacement = if global {
            old.with_rotations(old.local_rotation_rpy(), [0.2, 0.0, 0.0])
        } else {
            old.with_rotations([0.3, 0.0, 0.0], old.global_rotation_rpy())
        }
        .unwrap();
        assert_ne!(old, replacement);
        sim.world_mut()
            .get_mut::<Publication>(owner)
            .unwrap()
            .offsets = replacement;
        assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
        assert!(sim.world().get::<Sample>(owner).is_none());
        let fault = sim
            .world()
            .resource::<StepFault>()
            .error()
            .unwrap()
            .downcast_ref::<GroundTruthOdometryScheduleFault>()
            .unwrap();
        assert_eq!(
            (fault.entity, fault.source),
            (id, GroundTruthOdometryError::Identity)
        );
        let clock = (sim.epoch(), sim.elapsed());
        let metadata = sim.world().get::<Publication>(owner).unwrap().clone();
        let mut called = false;
        assert!(matches!(
            reset_loaded_world(&mut sim, |_| called = true),
            Err(SceneResetError::Clock(StepError::Faulted))
        ));
        assert!(!called);
        assert_eq!((sim.epoch(), sim.elapsed()), clock);
        assert_eq!(sim.world().get::<EntityId>(owner).unwrap().get(), id);
        assert_eq!(sim.world().get::<Publication>(owner).unwrap(), &metadata);
        assert!(sim.world().get::<Sample>(owner).is_none());
    }
}
#[test]
fn removal_and_healthy_reset_restore_rotation_identity_and_empty_histories() {
    let mut sim = load(XML);
    sim.step_exact(20).unwrap();
    let owner = model(&sim);
    let id = sim.world().get::<EntityId>(owner).unwrap().get();
    let link = sim.world().get::<EntityId>(body(&sim)).unwrap().get();
    let metadata = sim.world().get::<Publication>(owner).unwrap().clone();
    let first = sample(&sim);
    queue_link_world_wrench(&mut sim, link, [1.0; 3], [2.0; 3]).unwrap();
    remove_model(&mut sim, id).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<Publication>() || e.contains::<Sample>())
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let owner = model(&sim);
    assert_eq!(sim.world().get::<EntityId>(owner).unwrap().get(), id);
    assert_eq!(sim.world().get::<EntityId>(body(&sim)).unwrap().get(), link);
    assert_eq!(sim.world().get::<Publication>(owner).unwrap(), &metadata);
    assert!(sim.world().get::<Sample>(owner).is_none());
    sim.step_exact(20).unwrap();
    assert_eq!(bits(sample(&sim)), bits(first));
    assert_eq!(
        (sample(&sim).epoch, sample(&sim).revision),
        (1, first.revision)
    );
}
#[test]
fn malformed_nonfinite_rotation_and_other_configuration_refusals_remain_closed() {
    for tag in ["rpy_offset", "rpy_global_offset"] {
        for text in [
            "0 0",
            "0 0 0 1",
            "NaN 0 0",
            "0 inf 0",
            "0 0 -inf",
            "0 0 bad",
            "0<!-- gap -->0 1",
            "<![CDATA[0 0 1]]>",
            "<nested/>",
        ] {
            let bad = format!("<{tag}>{text}</{tag}>");
            assert!(
                load_world_with_sdf_step(
                    &xml("<dimensions>3</dimensions>", &bad, ""),
                    "bad.sdf",
                    "default",
                    &sdformat::NoResources,
                    PhysicsEngine::Rapier
                )
                .is_err(),
                "{bad}"
            );
        }
        for bad in [
            format!("<{tag}>0 0 1</{tag}><{tag}>0 0 1</{tag}>"),
            format!("<{tag} value='1'>0 0 1</{tag}>"),
        ] {
            assert!(
                load_world_with_sdf_step(
                    &xml("", &bad, ""),
                    "bad.sdf",
                    "default",
                    &sdformat::NoResources,
                    PhysicsEngine::Rapier
                )
                .is_err()
            );
        }
    }
    for extra in [
        "<gaussian_noise>0.01</gaussian_noise>",
        "<dimensions>4</dimensions>",
        "<unknown/>",
    ] {
        assert!(
            load_world_with_sdf_step(
                &XML.replace(
                    "<dimensions>3</dimensions>",
                    &format!("<dimensions>3</dimensions>{extra}")
                ),
                "bad.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::Rapier
            )
            .is_err()
        );
    }
    assert!(
        load_world_with_sdf_step(
            XML,
            "bad.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        )
        .is_err()
    );
}
