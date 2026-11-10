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
const XML: &str = include_str!("../../../examples/m39_odometry_translation_offsets.sdf");
fn xml(dimensions: &str, local: &str, global: &str) -> String {
    XML.replace("<dimensions>3</dimensions>", dimensions)
        .replace("<xyz_offset>0.4 -0.3 0.2</xyz_offset>", local)
        .replace(
            "<xyz_global_offset>-0.7 0.5 -0.6</xyz_global_offset>",
            global,
        )
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m39.sdf",
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
fn body_id(sim: &Simulation) -> u64 {
    sim.world().get::<EntityId>(body(sim)).unwrap().get()
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
fn geometry(sim: &Simulation, local: [f64; 3], global: [f64; 3], dimensions: Dimensions) {
    let world = sim.world().get::<WorldPose>(model(sim)).unwrap();
    // Independent rigid-body point geometry, not the native scalar expression
    // or a second velocity/filter estimator. Exact bits are covered by native replay.
    let point = DVec3::from_array(world.translation)
        + DQuat::from_array(world.rotation) * DVec3::from_array(local)
        - DVec3::from_array(global);
    let reading = sample(sim);
    for (index, expected) in point.to_array().into_iter().enumerate() {
        if index == 2 && dimensions == Dimensions::Two {
            assert_eq!(reading.pose.translation[index], 0.0);
        } else {
            assert!((reading.pose.translation[index] - expected).abs() < 1e-12);
        }
    }
    assert_eq!(
        reading.pose.rotation.map(f64::to_bits),
        world.rotation.map(f64::to_bits)
    );
}
fn drive(sim: &mut Simulation) {
    let id = body_id(sim);
    queue_link_world_wrench(sim, id, [4.0, -2.0, 6.0], [2.0, 4.0, 6.0]).unwrap();
    sim.step_exact(1).unwrap();
    let velocity = sim.world().get::<Velocity>(body(sim)).unwrap();
    for (actual, expected) in velocity.linear.into_iter().zip([0.002, -0.001, 0.003]) {
        assert!((actual - expected).abs() < 1e-12);
    }
    for (actual, expected) in velocity.angular.into_iter().zip([0.002, 0.004, 0.006]) {
        assert!((actual - expected).abs() < 1e-12);
    }
    sim.step_exact(19).unwrap();
}
#[test]
fn every_previously_refused_finite_input_loads_moves_and_healthy_resets_exact_configuration() {
    // The first case deliberately OMITS dimensions: it replaces the private
    // parser's default-Two refusal, independently of explicit Three below.
    for (dimension_xml, local_xml, local, mode) in [
        (
            "",
            "<xyz_offset>0 0 1</xyz_offset>",
            [0.0, 0.0, 1.0],
            Dimensions::Two,
        ),
        (
            "<dimensions>2</dimensions>",
            "<xyz_offset>0 1 0</xyz_offset>",
            [0.0, 1.0, 0.0],
            Dimensions::Two,
        ),
        (
            "<dimensions>3</dimensions>",
            "<xyz_offset>0 0 1</xyz_offset>",
            [0.0, 0.0, 1.0],
            Dimensions::Three,
        ),
    ] {
        let mut sim = load(&xml(dimension_xml, local_xml, ""));
        let owner = model(&sim);
        let id = sim.world().get::<EntityId>(owner).unwrap().get();
        let metadata = sim.world().get::<Publication>(owner).unwrap().clone();
        assert_eq!(metadata.dimensions, mode);
        assert_eq!(
            metadata.offsets,
            Offsets::from_translations(local, [0.0; 3]).unwrap()
        );
        sim.step_exact(20).unwrap();
        let initial = sample(&sim);
        geometry(&sim, local, [0.0; 3], mode);
        drive(&mut sim);
        let moved = sample(&sim);
        geometry(&sim, local, [0.0; 3], mode);
        assert_ne!(bits(moved), bits(initial));
        assert_eq!(moved.time, Duration::from_millis(40));
        assert!(
            moved
                .linear
                .into_iter()
                .chain(moved.angular)
                .all(f64::is_finite)
        );
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
fn local_global_and_combined_translations_follow_actual_point_geometry_in_both_modes() {
    for mode in [Dimensions::Two, Dimensions::Three] {
        let dimensions = if mode == Dimensions::Two {
            "<dimensions>2</dimensions>"
        } else {
            "<dimensions>3</dimensions>"
        };
        for (local, global, local_xml, global_xml) in [
            (
                [0.4, -0.3, 0.2],
                [0.0; 3],
                "<xyz_offset>0.4 -0.3 0.2</xyz_offset>",
                "",
            ),
            (
                [0.0; 3],
                [-0.7, 0.5, -0.6],
                "",
                "<xyz_global_offset>-0.7 0.5 -0.6</xyz_global_offset>",
            ),
            (
                [0.4, -0.3, 0.2],
                [-0.7, 0.5, -0.6],
                "<xyz_offset>0.4 -0.3 0.2</xyz_offset>",
                "<xyz_global_offset>-0.7 0.5 -0.6</xyz_global_offset>",
            ),
        ] {
            let mut sim = load(&xml(dimensions, local_xml, global_xml));
            sim.step_exact(20).unwrap();
            geometry(&sim, local, global, mode);
            drive(&mut sim);
            geometry(&sim, local, global, mode);
            assert_eq!(
                sim.world().get::<Publication>(model(&sim)).unwrap().offsets,
                Offsets::from_translations(local, global).unwrap()
            );
            assert_ne!(
                sample(&sim).pose.translation,
                sim.world()
                    .get::<WorldPose>(model(&sim))
                    .unwrap()
                    .translation
            );
        }
    }
}
#[test]
fn translated_point_feedback_does_not_change_physics_or_equal_instantaneous_body_velocity() {
    let mut shifted = load(XML);
    let mut origin = load(&xml("<dimensions>3</dimensions>", "", ""));
    shifted.step_exact(20).unwrap();
    origin.step_exact(20).unwrap();
    drive(&mut shifted);
    drive(&mut origin);
    let a = shifted.world().get::<WorldPose>(model(&shifted)).unwrap();
    let b = origin.world().get::<WorldPose>(model(&origin)).unwrap();
    assert_eq!(
        a.translation.map(f64::to_bits),
        b.translation.map(f64::to_bits)
    );
    assert_eq!(a.rotation.map(f64::to_bits), b.rotation.map(f64::to_bits));
    let av = shifted.world().get::<Velocity>(body(&shifted)).unwrap();
    let bv = origin.world().get::<Velocity>(body(&origin)).unwrap();
    assert_eq!(av.linear.map(f64::to_bits), bv.linear.map(f64::to_bits));
    assert_eq!(av.angular.map(f64::to_bits), bv.angular.map(f64::to_bits));
    assert_ne!(
        sample(&shifted).linear.map(f64::to_bits),
        sample(&origin).linear.map(f64::to_bits)
    );
    assert_ne!(
        sample(&shifted).linear.map(f64::to_bits),
        av.linear.map(f64::to_bits)
    );
    assert_eq!(
        sample(&shifted).angular.map(f64::to_bits),
        sample(&origin).angular.map(f64::to_bits)
    );
    geometry(
        &shifted,
        [0.4, -0.3, 0.2],
        [-0.7, 0.5, -0.6],
        Dimensions::Three,
    );
}
#[test]
fn each_legacy_signed_zero_vector_matches_omission_without_losing_nonzero_bits() {
    for dimensions in ["<dimensions>2</dimensions>", "<dimensions>3</dimensions>"] {
        for (a_local, a_global, b_local, b_global) in [
            (
                "",
                "",
                "<xyz_offset>-0 0 -0</xyz_offset>",
                "<xyz_global_offset>0 -0 -0</xyz_global_offset>",
            ),
            (
                "",
                "<xyz_global_offset>0 -0 1</xyz_global_offset>",
                "<xyz_offset>-0 0 -0</xyz_offset>",
                "<xyz_global_offset>0 -0 1</xyz_global_offset>",
            ),
            (
                "<xyz_offset>1 -0 0</xyz_offset>",
                "",
                "<xyz_offset>1 -0 0</xyz_offset>",
                "<xyz_global_offset>-0 0 -0</xyz_global_offset>",
            ),
        ] {
            let mut a = load(&xml(dimensions, a_local, a_global));
            let mut b = load(&xml(dimensions, b_local, b_global));
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
    let mixed = Offsets::from_translations([1.0, -0.0, 0.0], [0.0; 3]).unwrap();
    assert_eq!(mixed.local_translation()[1].to_bits(), (-0.0_f64).to_bits());
    assert_ne!(
        mixed,
        Offsets::from_translations([1.0, 0.0, 0.0], [0.0; 3]).unwrap()
    );
}
#[test]
fn translated_history_continues_before_throttle_and_pause_holds_publication() {
    let mut limited = load(XML);
    let mut every = load(&XML.replace(
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
#[test]
fn bit_only_offset_mutation_faults_and_terminal_reset_preserves_state() {
    let initial = xml(
        "<dimensions>3</dimensions>",
        "<xyz_offset>1 -0 0</xyz_offset>",
        "",
    );
    let mut sim = load(&initial);
    sim.step_exact(20).unwrap();
    let owner = model(&sim);
    let id = sim.world().get::<EntityId>(owner).unwrap().get();
    sim.world_mut()
        .get_mut::<Publication>(owner)
        .unwrap()
        .offsets = Offsets::from_translations([1.0, 0.0, 0.0], [0.0; 3]).unwrap();
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
    let mut observed = false;
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| observed = true),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert!(!observed);
    assert_eq!((sim.epoch(), sim.elapsed()), clock);
    assert_eq!(sim.world().get::<Publication>(owner).unwrap(), &metadata);
    assert_eq!(sim.world().get::<EntityId>(owner).unwrap().get(), id);
    assert!(sim.world().get::<Sample>(owner).is_none());
}
#[test]
fn removal_reconstructs_original_offsets_only_through_healthy_loaded_reset() {
    let mut sim = load(XML);
    sim.step_exact(20).unwrap();
    let owner = model(&sim);
    let id = sim.world().get::<EntityId>(owner).unwrap().get();
    let link = body_id(&sim);
    let metadata = sim.world().get::<Publication>(owner).unwrap().clone();
    let initial = sample(&sim);
    queue_link_world_wrench(&mut sim, link, [1.0; 3], [2.0; 3]).unwrap();
    remove_model(&mut sim, id).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<Publication>() || e.contains::<Sample>())
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.world().get::<EntityId>(model(&sim)).unwrap().get(), id);
    assert_eq!(body_id(&sim), link);
    assert_eq!(
        sim.world().get::<Publication>(model(&sim)).unwrap(),
        &metadata
    );
    sim.step_exact(20).unwrap();
    assert_eq!(bits(sample(&sim)), bits(initial));
    assert_eq!(sample(&sim).epoch, 1);
}
#[test]
fn malformed_nonfinite_translation_and_unrelated_refusals_remain_closed() {
    for local in [
        "<xyz_offset>0 0</xyz_offset>",
        "<xyz_offset>0 0 bad</xyz_offset>",
        "<xyz_offset>NaN 0 0</xyz_offset>",
        "<xyz_offset>0 0 inf</xyz_offset>",
        "<xyz_offset>0 0 1 2</xyz_offset>",
        "<xyz_offset>0 0 1</xyz_offset><xyz_offset>0 0 1</xyz_offset>",
        "<xyz_offset>0<!-- gap -->0 1</xyz_offset>",
        "<xyz_offset><![CDATA[0 0 1]]></xyz_offset>",
        "<xyz_offset value='1'>0 0 1</xyz_offset>",
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml("<dimensions>3</dimensions>", local, ""),
                "bad.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::Rapier
            )
            .is_err(),
            "{local}"
        );
    }
    for global in [
        "<xyz_global_offset>0 0</xyz_global_offset>",
        "<xyz_global_offset>0 0 bad</xyz_global_offset>",
        "<xyz_global_offset>0 NaN 0</xyz_global_offset>",
        "<xyz_global_offset>0 0 -inf</xyz_global_offset>",
        "<xyz_global_offset>1 2 3</xyz_global_offset><xyz_global_offset>1 2 3</xyz_global_offset>",
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml("<dimensions>3</dimensions>", "", global),
                "bad.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::Rapier
            )
            .is_err(),
            "{global}"
        );
    }
    for extra in [
        "<rpy_offset>0 0 0.1</rpy_offset>",
        "<rpy_global_offset>0.1 0 0</rpy_global_offset>",
        "<gaussian_noise>0.01</gaussian_noise>",
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
