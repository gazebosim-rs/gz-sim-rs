// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use sdformat::NoResources;

const XML: &str = include_str!("../fixtures/m18_joint_state.sdf");
fn dom() -> Sdf {
    sdformat::load_str(XML, "m18.sdf", &NoResources).expect("fixture DOM")
}
fn model() -> Model {
    dom().worlds.remove(0).models.remove(0)
}
fn configured(contents: &str) -> Model {
    let mut model = model();
    let plugin = &mut model.plugins[0];
    plugin.xml = format!(
        "<plugin filename='{}' name='{}'>{contents}</plugin>",
        plugin.filename, plugin.name
    );
    model
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m18.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .expect("admitted joint-state fixture")
}
fn model_entity(sim: &Simulation) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::arm"))
        .expect("fixture model")
        .id()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let entry = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .expect("fixture entity");
    (entry.id(), entry.get::<EntityId>().expect("identity").get())
}
fn without_publisher() -> String {
    let start = XML.find("    <plugin ").expect("plugin start");
    let end = start + XML[start..].find("</plugin>").expect("plugin end") + "</plugin>".len();
    format!("{}{}", &XML[..start], &XML[end..])
}

#[test]
fn closed_configuration_filters_deduplicates_defaults_and_truncates_native_rate() {
    assert!(
        parse(&{
            let mut m = model();
            m.plugins.clear();
            m
        })
        .unwrap()
        .is_none()
    );
    let plan = parse(&configured("")).unwrap().unwrap();
    assert_eq!(plan.topic, "/world/default/model/arm/joint_state");
    assert_eq!(plan.period, Duration::ZERO);
    assert_eq!(
        plan.names.into_iter().collect::<Vec<_>>(),
        ["base_hinge", "tip_hinge"]
    );
    let plan = parse(&configured("<joint_name>tip_hinge</joint_name><joint_name>tip_hinge</joint_name><update_rate>3</update_rate><topic>/custom/state</topic>")).unwrap().unwrap();
    assert_eq!(plan.names.into_iter().collect::<Vec<_>>(), ["tip_hinge"]);
    assert_eq!(plan.period, Duration::from_nanos(333_333_333));
    assert_eq!(plan.topic, "/custom/state");
    assert_eq!(period(2e9), Some(Duration::ZERO));
    assert!(period(1.1e-10).is_some_and(|value| value.as_nanos() < i64::MAX as u128));
    for rate in [f64::NAN, f64::INFINITY, -1.0, 1e-300, 1e-10] {
        assert!(period(rate).is_none());
    }
    for contents in [
        "<joint_name>missing</joint_name>",
        "<joint_name/>",
        "<topic/>",
        "<topic>/</topic>",
        "<topic>/bad topic</topic>",
        "<topic>/a//b</topic>",
        "<topic>/a</topic><topic>/b</topic>",
        "<update_rate>NaN</update_rate>",
        "<update_rate>-1</update_rate>",
        "<update_rate>0</update_rate><update_rate>1</update_rate>",
        "<unknown>1</unknown>",
        "<joint_name extra='x'>base_hinge</joint_name>",
        "<joint_name><![CDATA[base_hinge]]></joint_name>",
        "<joint_name><name>base_hinge</name></joint_name>",
    ] {
        assert!(parse(&configured(contents)).is_err(), "{contents}");
    }
    let mut duplicate = model();
    duplicate.plugins.push(duplicate.plugins[0].clone());
    assert!(parse(&duplicate).is_err());
    let mut malformed = model();
    malformed.plugins[0].xml = "<plugin".to_owned();
    assert!(parse(&malformed).is_err());
}

#[test]
fn actual_values_and_all_metadata_bind_loaded_ids_with_native_child_pose() {
    let mut sim = load(&XML.replace(
        "<update_rate>100</update_rate>",
        "<update_rate>0</update_rate>",
    ));
    let entity = model_entity(&sim);
    let publication = sim
        .world()
        .get::<JointStatePublication>(entity)
        .expect("metadata")
        .clone();
    assert_eq!(publication.model_name, "arm");
    assert_eq!(publication.topic, "/m18/joint_state");
    assert_eq!(publication.joints.len(), 2);
    let (_, base) = named(&sim, "default::arm::base_hinge");
    let (_, tip) = named(&sim, "default::arm::tip_hinge");
    assert_eq!(
        publication.joints.iter().map(|v| v.id).collect::<Vec<_>>(),
        [base, tip]
    );
    assert_eq!(publication.joints[0].parent, "world");
    assert_eq!(publication.joints[0].child, "base");
    assert_eq!(publication.joints[1].parent, "base");
    assert_eq!(publication.joints[1].child, "tip");
    for joint in &publication.joints {
        assert_eq!(joint.pose, Transform::default());
        assert_eq!(joint.axis, [0.0, 0.0, 1.0]);
        assert_eq!(
            (joint.lower, joint.upper, joint.damping),
            (f64::NEG_INFINITY, f64::INFINITY, 0.0)
        );
    }
    queue_joint_effort(&mut sim, base, 1.0).expect("pending effort");
    sim.step_exact(1).expect("moving joint");
    let sample = sim.world().get::<JointStateSample>(entity).expect("sample");
    assert_eq!(
        (sample.time, sample.epoch, sample.revision),
        (Duration::from_millis(1), 0, 1)
    );
    let model_pose = sim.world().get::<LocalPose>(entity).unwrap().0;
    assert_eq!(sample.pose.translation, model_pose.translation);
    assert_eq!(sample.pose.rotation, model_pose.rotation);
    for value in &sample.joints {
        let joint = gz_sim_core::entity_by_id(sim.world(), value.id).unwrap();
        let actual = sim.world().get::<JointSample>(joint).unwrap();
        assert_eq!(
            (value.position, value.velocity),
            (actual.state.position, actual.state.velocity)
        );
    }
    assert!(sample.joints[0].position.abs() > 0.0 && sample.joints[0].velocity.abs() > 0.0);
    // The native pose metadata is fixed child-frame construction data, not the
    // model-relative dynamic Joint LocalPose (tip starts .8m from the model).
    let tip_entity = gz_sim_core::entity_by_id(sim.world(), tip).unwrap();
    assert!(
        sim.world()
            .get::<LocalPose>(tip_entity)
            .unwrap()
            .0
            .translation[0]
            > 0.7
    );
}

#[test]
fn native_cadence_includes_equal_time_pause_and_reset_without_sensor_suppression() {
    let mut sim = load(XML);
    let entity = model_entity(&sim);
    sim.step_exact(1).unwrap();
    assert!(sim.world().get::<JointStateSample>(entity).is_none());
    sim.step_exact(9).unwrap();
    assert_eq!(
        sim.world().get::<JointStateSample>(entity).unwrap().time,
        Duration::from_millis(10)
    );
    assert_eq!(
        sim.world()
            .get::<JointStateSample>(entity)
            .unwrap()
            .revision,
        1
    );
    let previous = sim
        .world()
        .get::<JointStateSample>(entity)
        .unwrap()
        .revision;
    assert!(sim.is_paused());
    sim.tick().unwrap();
    sim.tick().unwrap();
    assert_eq!(
        sim.world()
            .get::<JointStateSample>(entity)
            .unwrap()
            .revision,
        previous + 2
    );
    assert_eq!(
        sim.world().get::<JointStateSample>(entity).unwrap().time,
        Duration::from_millis(10)
    );
    let (_, joint_id) = named(&sim, "default::arm::base_hinge");
    let joint = gz_sim_core::entity_by_id(sim.world(), joint_id).unwrap();
    let actual = sim.world().get::<JointSample>(joint).unwrap();
    assert_eq!((actual.time, actual.epoch), (Duration::from_millis(10), 0));
    let local = sim.world().get::<LocalPose>(entity).unwrap().0;
    let published = sim.world().get::<JointStateSample>(entity).unwrap();
    assert_eq!(published.pose.translation, local.translation);
    assert_eq!(published.pose.rotation, local.rotation);
    // Direct core reset clears the public sample but keeps live scheduling state.
    let retained_revision = sim.world().get::<Publisher>(entity).unwrap().revision;
    sim.reset().unwrap();
    assert!(sim.world().get::<JointStateSample>(entity).is_none());
    sim.tick().unwrap();
    assert_eq!(
        sim.world()
            .get::<JointStateSample>(entity)
            .unwrap()
            .revision,
        retained_revision + 1
    );
    // Loaded reset reconstructs the original Publisher and physical state.
    let model_id = sim.world().get::<EntityId>(entity).unwrap().get();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored =
        gz_sim_core::entity_by_id(sim.world(), model_id).expect("restored model identity");
    assert_ne!(restored, entity);
    assert_eq!(
        sim.world().get::<EntityId>(restored).unwrap().get(),
        model_id
    );
    assert!(sim.world().get::<JointStateSample>(restored).is_none());
    sim.tick().unwrap();
    let sample = sim.world().get::<JointStateSample>(restored).unwrap();
    assert_eq!((sample.time, sample.epoch), (Duration::ZERO, 2));
    assert_eq!(sample.revision, 1);
    assert!(
        sample
            .joints
            .iter()
            .all(|j| j.position == 0.0 && j.velocity == 0.0)
    );
}

#[test]
fn absent_and_removed_family_owns_no_callbacks_and_loaded_ids_restore() {
    let absent = load(&without_publisher());
    let mut live = load(XML);
    let baseline = absent.scheduled_system_counts();
    assert_eq!(
        live.scheduled_system_counts(),
        (baseline.0 + 1, baseline.1 + 1)
    );
    assert!(!absent.world().contains_resource::<Installed>());
    let entity = model_entity(&live);
    let id = live.world().get::<EntityId>(entity).unwrap().get();
    remove_model(&mut live, id).unwrap();
    assert!(!live.world().contains_resource::<Installed>());
    assert!(
        !live
            .world()
            .iter_entities()
            .any(|e| e.contains::<JointStateSample>() || e.contains::<Publisher>())
    );
    // Loaded reset restores owned identities/publication, not a live manual edit.
    reset_loaded_world(&mut live, |_| ()).unwrap();
    let restored = model_entity(&live);
    assert_eq!(live.world().get::<EntityId>(restored).unwrap().get(), id);
    assert!(live.world().contains_resource::<Installed>());
    live.tick().unwrap();
    assert!(live.world().get::<JointStateSample>(restored).is_some());
}

#[test]
fn corrupt_publication_nonfinite_sample_and_revision_exhaustion_fault_without_packet() {
    for failure in 0..5 {
        let mut sim = load(XML);
        let entity = model_entity(&sim);
        match failure {
            0 => {
                sim.world_mut()
                    .get_mut::<JointStatePublication>(entity)
                    .unwrap()
                    .topic = "/forged".to_owned()
            }
            1 => {
                let (joint, _) = named(&sim, "default::arm::base_hinge");
                sim.world_mut()
                    .get_mut::<JointSample>(joint)
                    .unwrap()
                    .state
                    .position = f64::NAN;
            }
            2 => {
                sim.world_mut()
                    .get_mut::<Publisher>(entity)
                    .unwrap()
                    .revision = u64::MAX
            }
            3 => {
                sim.world_mut()
                    .entity_mut(entity)
                    .remove::<JointStatePublication>();
            }
            _ => {
                let (joint, _) = named(&sim, "default::arm::base_hinge");
                sim.world_mut().entity_mut(joint).remove::<JointSample>();
            }
        }
        // Exercise the publication boundary directly: physics would overwrite a
        // forged JointSample, and the boundary must independently reject it.
        sample(sim.world_mut());
        assert!(sim.world().resource::<StepFault>().error().is_some());
        assert!(sim.world().get::<JointStateSample>(entity).is_none());
    }
}

#[test]
fn plan_domain_and_configuration_resource_bounds_are_explicit() {
    let absent = load(&without_publisher());
    let original = model();
    let plan = parse(&original).unwrap().unwrap();
    plan.validate(&original, &absent, Some("rapier"), (true, false))
        .expect("admitted domain");
    for engine in [None, Some("tpe"), Some("dart")] {
        assert!(
            plan.validate(&original, &absent, engine, (true, false))
                .is_err()
        );
    }
    for placement in [(false, false), (true, true)] {
        assert!(
            plan.validate(&original, &absent, Some("rapier"), placement)
                .is_err()
        );
    }
    let mut nested = model();
    nested.models.push(model());
    assert!(
        plan.validate(&nested, &absent, Some("rapier"), (true, false))
            .is_err()
    );
    let existing = load(XML);
    assert!(matches!(
        plan.validate(&original, &existing, Some("rapier"), (true, false)),
        Err(LoadError::Unsupported {
            feature: "multiple joint-state models",
            ..
        })
    ));
    let mut namespace = configured("");
    namespace.plugins[0].xml = namespace.plugins[0]
        .xml
        .replace("<plugin ", "<plugin xmlns='urn:m18' ");
    assert!(parse(&namespace).is_err());
    assert!(
        parse(&configured(
            "<joint_name xmlns='urn:m18'>base_hinge</joint_name>"
        ))
        .is_err()
    );
    let mut bounded = configured("");
    let padding = 65536 - bounded.plugins[0].xml.len() - 7;
    bounded.plugins[0].xml = bounded.plugins[0].xml.replace(
        "</plugin>",
        &format!("<!--{}--></plugin>", "x".repeat(padding)),
    );
    assert_eq!(bounded.plugins[0].xml.len(), 65536);
    assert!(parse(&bounded).is_ok());
    let offset = bounded.plugins[0].xml.find("<!--").unwrap() + 4;
    bounded.plugins[0].xml.insert(offset, 'x');
    assert_eq!(bounded.plugins[0].xml.len(), 65537);
    assert!(parse(&bounded).is_err());
    let template = original.joints[0].clone();
    let many = |count| {
        let mut m = configured("");
        m.joints = (0..count)
            .map(|index| {
                let mut joint = template.clone();
                joint.name = format!("hinge_{index}");
                joint
            })
            .collect();
        m
    };
    assert_eq!(parse(&many(128)).unwrap().unwrap().names.len(), 128);
    assert!(parse(&many(129)).is_err());
    let mut no_joints = configured("");
    no_joints.joints.clear();
    assert!(parse(&no_joints).is_err());
}

#[test]
fn nonidentity_child_frame_and_expressed_axis_metadata_do_not_alias_model_frame() {
    // Existing Rapier admission allows a child-relative revolute frame and an
    // expressed axis; this does not add a new physical joint/geometry capability.
    let xml=XML.replace("<pose relative_to=\"tip\">0 0 0 0 0 0</pose><axis><xyz>0 0 1</xyz>",
        "<pose relative_to=\"tip\">0.1 -0.2 0.3 1.5707963267948966 0 0</pose><axis><xyz expressed_in=\"__model__\">0 0 1</xyz>");
    assert_ne!(xml, XML);
    let sim = load(&xml);
    let entity = model_entity(&sim);
    let publication = sim.world().get::<JointStatePublication>(entity).unwrap();
    let tip = publication
        .joints
        .iter()
        .find(|joint| joint.name == "tip_hinge")
        .unwrap();
    for (actual, expected) in tip.pose.translation.into_iter().zip([0.1, -0.2, 0.3]) {
        assert!((actual - expected).abs() < 1e-15);
    }
    let h = std::f64::consts::FRAC_1_SQRT_2;
    for (actual, expected) in tip.pose.rotation.into_iter().zip([h, 0.0, 0.0, h]) {
        assert!((actual - expected).abs() < 1e-15);
    }
    // Inverse child-joint Rx(pi/2) rotates the model's +Z into joint +Y.
    for (actual, expected) in tip.axis.into_iter().zip([0.0, 1.0, 0.0]) {
        assert!((actual - expected).abs() < 1e-15);
    }
    let joint = named(&sim, "default::arm::tip_hinge").0;
    let model_relative = sim.world().get::<LocalPose>(joint).unwrap().0;
    assert!((model_relative.translation[0] - tip.pose.translation[0]).abs() > 0.7);
}
