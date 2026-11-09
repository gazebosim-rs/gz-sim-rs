// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn configured(contents: &str) -> Model {
    let mut model = sdformat::load_str(
        include_str!("../fixtures/m27_pose_publisher.sdf"),
        "m31-config.sdf",
        &sdformat::NoResources,
    )
    .unwrap()
    .worlds
    .remove(0)
    .models
    .remove(0);
    model.plugins[0].xml = format!(
        "<plugin filename='{}' name='{}'>{contents}</plugin>",
        model.plugins[0].filename, model.plugins[0].name
    );
    model
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m31-private.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn owner(sim: &Simulation) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.contains::<PosePublication>())
        .unwrap()
        .id()
}
fn absent_xml() -> String {
    let first = XML.find("    <plugin ").unwrap();
    let last = first + XML[first..].find("</plugin>").unwrap() + 9;
    format!("{}{}", &XML[..first], &XML[last..])
}
const XML: &str = include_str!("../fixtures/m31_pose_static_publisher.sdf");
#[test]
fn split_configuration_requires_all_links_direct_canonical_names_and_checked_static_rate() {
    let plan = parse(&configured("<use_pose_vector_msg>true</use_pose_vector_msg><static_publisher>true</static_publisher><update_frequency>100</update_frequency>")).unwrap().unwrap();
    assert_eq!(plan.static_period, Some(Duration::from_millis(10)));
    assert!(plan.links);
    for rate in ["NaN", "inf", "1e-300", "bad"] {
        assert!(parse(&configured(&format!("<use_pose_vector_msg>true</use_pose_vector_msg><static_publisher>true</static_publisher><static_update_frequency>{rate}</static_update_frequency>"))).is_err());
    }
    for rate in ["0", "-100", "2000000000"] {
        let p = parse(&configured(&format!("<use_pose_vector_msg>true</use_pose_vector_msg><static_publisher>true</static_publisher><static_update_frequency>{rate}</static_update_frequency>"))).unwrap().unwrap();
        assert_eq!(p.static_period, Some(Duration::ZERO));
    }
    let too_long = format!("/{}", "x".repeat(65530));
    assert!(parse(&configured(&format!("<use_pose_vector_msg>true</use_pose_vector_msg><static_publisher>true</static_publisher><topic>{too_long}</topic>"))).is_err());
    for edit in [0, 1, 2] {
        let mut m = configured(
            "<use_pose_vector_msg>true</use_pose_vector_msg><static_publisher>true</static_publisher>",
        );
        match edit {
            0 => m.canonical_link = "missing".into(),
            1 => m.joints[1].parent = "rig::base".into(),
            _ => m.joints[1].child = "missing".into(),
        }
        let p = parse(&m).unwrap().unwrap();
        let sim = load(&absent_xml());
        assert!(p.validate(&m, &sim, Some("rapier"), (true, false)).is_err());
    }
}
#[test]
fn split_independent_clocks_hold_samples_and_preserve_both_reset_policies() {
    let mut sim = load(XML);
    let entity = owner(&sim);
    let metadata = sim.world().get::<PosePublication>(entity).unwrap().clone();
    assert_eq!(
        metadata
            .poses
            .iter()
            .map(|p| p.child_frame_id.as_str())
            .collect::<Vec<_>>(),
        ["rig::tip"]
    );
    let fixed = metadata.static_publication.as_ref().unwrap();
    assert_eq!(fixed.topic, "/m31/pose_static");
    assert_eq!(
        fixed
            .poses
            .iter()
            .map(|p| p.child_frame_id.as_str())
            .collect::<Vec<_>>(),
        ["rig", "rig::base"]
    );
    sim.step_exact(100).unwrap();
    let dynamic = sim.world().get::<PoseSample>(entity).unwrap();
    assert_eq!(
        (dynamic.time, dynamic.revision),
        (Duration::from_millis(100), 10)
    );
    let fixed = &sim.world().get::<StaticPoseSample>(entity).unwrap().0;
    assert_eq!((fixed.time, fixed.revision), (Duration::from_millis(80), 2));
    sim.tick().unwrap();
    assert_eq!(
        sim.world()
            .get::<StaticPoseSample>(entity)
            .unwrap()
            .0
            .revision,
        2
    );
    sim.world_mut().resource_mut::<UpdateInfo>().paused = false;
    sample(sim.world_mut());
    assert_eq!(sim.world().get::<PoseSample>(entity).unwrap().revision, 11);
    assert_eq!(
        sim.world()
            .get::<StaticPoseSample>(entity)
            .unwrap()
            .0
            .revision,
        2
    );
    let live = sim.world().get::<Publisher>(entity).unwrap().clone();
    sim.reset().unwrap();
    assert!(sim.world().get::<PoseSample>(entity).is_none());
    assert!(sim.world().get::<StaticPoseSample>(entity).is_none());
    assert_eq!(
        sim.world().get::<Publisher>(entity).unwrap().last,
        live.last
    );
    assert_eq!(
        sim.world()
            .get::<Publisher>(entity)
            .unwrap()
            .static_clock
            .as_ref()
            .unwrap()
            .last,
        live.static_clock.as_ref().unwrap().last
    );
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().get::<StaticPoseSample>(entity).unwrap().0.time,
        Duration::from_millis(1)
    );
    let id = sim.world().get::<EntityId>(entity).unwrap().get();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
    assert_ne!(entity, restored);
    assert_eq!(
        sim.world().get::<PosePublication>(restored).unwrap(),
        &metadata
    );
    sim.step_exact(39).unwrap();
    assert!(sim.world().get::<StaticPoseSample>(restored).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().get::<PoseSample>(restored).unwrap().revision, 4);
    assert_eq!(
        sim.world()
            .get::<StaticPoseSample>(restored)
            .unwrap()
            .0
            .revision,
        1
    );
}
#[test]
fn split_static_only_ticks_do_not_replay_dynamic_and_invalid_due_frame_clears_both() {
    let xml = XML.replace(
        "<static_update_frequency>25",
        "<static_update_frequency>200",
    );
    let mut sim = load(&xml);
    let entity = owner(&sim);
    sim.step_exact(10).unwrap();
    let revision = sim.world().get::<PoseSample>(entity).unwrap().revision;
    sim.step_exact(5).unwrap();
    assert_eq!(
        sim.world().get::<PoseSample>(entity).unwrap().revision,
        revision
    );
    assert_eq!(
        sim.world()
            .get::<StaticPoseSample>(entity)
            .unwrap()
            .0
            .revision,
        3
    );
    for case in 0..4 {
        let mut sim = load(XML);
        let entity = owner(&sim);
        sim.step_exact(40).unwrap();
        match case {
            0 => {
                sim.world_mut()
                    .get_mut::<Publisher>(entity)
                    .unwrap()
                    .static_clock
                    .as_mut()
                    .unwrap()
                    .revision = u64::MAX
            }
            1 => {
                sim.world_mut()
                    .get_mut::<PosePublication>(entity)
                    .unwrap()
                    .static_publication
                    .as_mut()
                    .unwrap()
                    .topic = "/forged".into();
            }
            2 => {
                let id = sim
                    .world()
                    .get::<PosePublication>(entity)
                    .unwrap()
                    .static_publication
                    .as_ref()
                    .unwrap()
                    .poses[0]
                    .identity;
                let node = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
                sim.world_mut()
                    .get_mut::<LocalPose>(node)
                    .unwrap()
                    .0
                    .rotation = [0.0; 4];
            }
            _ => {
                sim.world_mut()
                    .get_mut::<Publisher>(entity)
                    .unwrap()
                    .revision = u64::MAX;
            }
        }
        sim.world_mut().resource_mut::<UpdateInfo>().paused = false;
        sample(sim.world_mut());
        assert!(sim.world().get::<PoseSample>(entity).is_none());
        assert!(sim.world().get::<StaticPoseSample>(entity).is_none());
        assert!(sim.world().resource::<StepFault>().error().is_some());
    }
}
#[test]
fn split_inherited_rate_fixed_metadata_and_jointless_acquisition_preserve_backend_domain() {
    let inherited = XML.replace("<static_update_frequency>25</static_update_frequency>", "");
    let mut sim = load(&inherited);
    let entity = owner(&sim);
    sim.step_exact(100).unwrap();
    assert_eq!(sim.world().get::<PoseSample>(entity).unwrap().revision, 10);
    assert_eq!(
        sim.world()
            .get::<StaticPoseSample>(entity)
            .unwrap()
            .0
            .revision,
        10
    );
    let fixed = XML.replace("type=\"revolute\"", "type=\"fixed\"");
    assert!(matches!(
        load_world_with_sdf_step(
            &fixed,
            "m31-fixed-domain.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        ),
        Err(LoadError::Unsupported { entity, feature })
            if entity == "default::rig::base_hinge" && feature == "joints (unsupported type)"
    ));
    {
        // Exercise the publication constructor's fixed classification, not an
        // unsupported physical fixed-joint world. This fixture is never stepped.
        let sdf =
            sdformat::load_str(&fixed, "m31-fixed-metadata.sdf", &sdformat::NoResources).unwrap();
        let model = &sdf.worlds[0].models[0];
        assert_eq!(model.joints.len(), 2);
        assert!(
            model
                .joints
                .iter()
                .all(|joint| joint.kind == sdformat::JointKind::Fixed)
        );
        let plan = parse(model).unwrap().unwrap();
        let mut construction = load(&absent_xml());
        let entity = construction
            .world()
            .iter_entities()
            .find(|node| {
                node.get::<Name>()
                    .is_some_and(|name| name.0 == "default::rig")
            })
            .unwrap()
            .id();
        plan.validate(model, &construction, Some("rapier"), (true, false))
            .unwrap();
        let counts = construction.scheduled_system_counts();
        let backend = construction
            .world_mut()
            .remove_resource::<PhysicsBackend>()
            .unwrap();
        let handles: Vec<_> = construction
            .world()
            .iter_entities()
            .filter_map(|node| node.get::<JointHandle>().map(JointHandle::id))
            .collect();
        assert_eq!(handles.len(), 2);
        let before: Vec<_> = handles.iter().map(|&id| backend.joint_state(id)).collect();
        assert!(before.iter().all(Option::is_some));
        let result = build(&sdf, &mut construction, model, entity, &backend, &plan);
        assert_eq!(
            handles
                .iter()
                .map(|&id| backend.joint_state(id))
                .collect::<Vec<_>>(),
            before
        );
        construction.world_mut().insert_resource(backend);
        result.unwrap();
        let metadata = construction.world().get::<PosePublication>(entity).unwrap();
        assert!(metadata.poses.is_empty());
        assert_eq!(
            metadata
                .static_publication
                .as_ref()
                .unwrap()
                .poses
                .iter()
                .map(|frame| frame.child_frame_id.as_str())
                .collect::<Vec<_>>(),
            ["rig", "rig::base", "rig::tip"]
        );
        assert_eq!(construction.elapsed(), Duration::ZERO);
        assert_eq!(construction.epoch(), 0);
        assert_eq!(construction.scheduled_system_counts(), counts);
        assert!(construction.world().get::<PoseSample>(entity).is_none());
        assert!(
            construction
                .world()
                .get::<StaticPoseSample>(entity)
                .is_none()
        );
    }
    // The admitted jointless world exercises the same all-static runtime path.
    let start = XML.find("    <joint ").unwrap();
    let end = XML.find("    <plugin ").unwrap();
    let jointless = format!("{}{}", &XML[..start], &XML[end..]);
    let mut sim = load(&jointless);
    let entity = owner(&sim);
    let metadata = sim.world().get::<PosePublication>(entity).unwrap();
    assert!(metadata.poses.is_empty());
    assert_eq!(metadata.static_publication.as_ref().unwrap().poses.len(), 3);
    sim.step_exact(40).unwrap();
    assert!(sim.world().get::<PoseSample>(entity).is_none());
    assert_eq!(
        sim.world()
            .get::<StaticPoseSample>(entity)
            .unwrap()
            .0
            .poses
            .len(),
        3
    );
    let id = sim.world().get::<EntityId>(entity).unwrap().get();
    remove_model(&mut sim, id).unwrap();
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<Publisher>() || e.contains::<StaticPoseSample>())
    );
}
