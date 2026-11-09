// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::entity_by_id;
use sdformat::NoResources;
const XML: &str = include_str!("../fixtures/m30_logical_camera.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "logical-camera",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (e.id(), e.get::<EntityId>().unwrap().get())
}
fn sensor(sim: &Simulation) -> Entity {
    named(sim, "default::rig::body::logical_camera").0
}
fn absent() -> String {
    let start = XML.find("    <sensor ").unwrap();
    let end = start + XML[start..].find("</sensor>").unwrap() + 9;
    format!("{}{}", &XML[..start], &XML[end..])
}
#[test]
fn absence_demand_phase_aligned_rate_pause_and_removal_reset_ownership() {
    let mut no = load(&absent());
    let mut sim = load(XML);
    let counts = no.scheduled_system_counts();
    assert_eq!(sim.scheduled_system_counts(), (counts.0 + 1, counts.1 + 1));
    set_logical_camera_interest(&mut no, []);
    assert!(
        !no.world().contains_resource::<Installed>() && !no.world().contains_resource::<Interest>()
    );
    no.step_exact(3).unwrap();
    no.reset().unwrap();
    assert_eq!(no.scheduled_system_counts(), counts);
    let e = sensor(&sim);
    let identity = sim.world().get::<EntityId>(e).unwrap().get();
    set_logical_camera_interest(&mut sim, []);
    sim.step_exact(20).unwrap();
    assert!(sim.world().get::<LogicalCameraSample>(e).is_none());
    assert!(
        sim.world()
            .get::<LogicalCamera>(e)
            .unwrap()
            .schedule
            .is_due(sim.elapsed())
    );
    set_logical_camera_interest(&mut sim, [e, e]);
    assert_eq!(sim.world().resource::<Interest>().0.len(), 1);
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().get::<LogicalCameraSample>(e).unwrap().time,
        Duration::from_millis(21)
    );
    let first = sim
        .world()
        .get::<LogicalCameraSample>(e)
        .unwrap()
        .reading
        .clone();
    sim.tick().unwrap();
    sim.step_exact(8).unwrap();
    assert_eq!(
        sim.world().get::<LogicalCameraSample>(e).unwrap().reading,
        first
    );
    assert_eq!(
        sim.world().get::<LogicalCameraSample>(e).unwrap().time,
        Duration::from_millis(21)
    );
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().get::<LogicalCameraSample>(e).unwrap().time,
        Duration::from_millis(30)
    );
    set_logical_camera_interest(&mut sim, []);
    assert!(sim.world().get::<LogicalCameraSample>(e).is_none());
    clear_logical_camera_interest(&mut sim);
    let rig = named(&sim, "default::rig").1;
    remove_model(&mut sim, rig).unwrap();
    assert!(entity_by_id(sim.world(), identity).is_none());
    assert!(!sim.world().contains_resource::<Installed>());
    assert_eq!(sim.scheduled_system_counts(), counts);
    initial::reset_loaded_world(&mut sim, |sim| {
        let fresh = entity_by_id(sim.world(), identity).unwrap();
        assert_ne!(fresh, e);
        assert!(sim.world().get::<LogicalCameraSample>(fresh).is_none());
    })
    .unwrap();
    let fresh = entity_by_id(sim.world(), identity).unwrap();
    sim.step_exact(1).unwrap();
    let s = sim.world().get::<LogicalCameraSample>(fresh).unwrap();
    assert_eq!((s.epoch, s.time), (1, Duration::from_millis(1)));
    assert_eq!(
        s.reading
            .as_ref()
            .unwrap()
            .models
            .iter()
            .map(|m| m.name.as_str())
            .collect::<Vec<_>>(),
        ["rig", "target"]
    );
}
#[test]
fn strict_raw_configuration_and_metadata_refuse_before_creation() {
    for (old, new) in [
        ("<near>0.5</near>", ""),
        ("<near>0.5</near>", "<near>0.5</near><near>1</near>"),
        ("<far>10</far>", "<far>0.25</far>"),
        ("<near>0.5</near>", "<near>NaN</near>"),
        ("<logical_camera>", "<logical_camera extra='x'>"),
        ("<near>0.5</near>", "<near><bad/>0.5</near>"),
        ("<near>0.5</near>", "<near>0.5</near><noise type='none'/>"),
        (
            "<aspect_ratio>1</aspect_ratio>",
            "<aspect_ratio>0</aspect_ratio>",
        ),
        (
            "<horizontal_fov>1.5707963267948966</horizontal_fov>",
            "<horizontal_fov>3.141592653589793</horizontal_fov>",
        ),
        ("<always_on>true</always_on>", "<visualize>true</visualize>"),
        (
            "<update_rate>100</update_rate>",
            "<update_rate>1e-300</update_rate>",
        ),
        (
            "<topic>/m30/logical_camera</topic>",
            "<topic>bad topic</topic>",
        ),
    ] {
        assert!(
            load_world_with_sdf_step(
                &XML.replace(old, new),
                "bad",
                "default",
                &NoResources,
                PhysicsEngine::Rapier
            )
            .is_err(),
            "{old} -> {new}"
        );
    }
    let camera =
        XML[XML.find("    <sensor ").unwrap()..XML.find("</sensor>").unwrap() + 9].to_owned();
    assert!(
        load_world_with_sdf_step(
            &XML.replace("  </link></model>", &format!("{camera}  </link></model>")),
            "two",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
    assert!(matches!(
        load_world_with_sdf_step(XML, "tpe", "default", &NoResources, PhysicsEngine::Tpe),
        Err(LoadError::Unsupported { .. })
    ));
    let plain = XML
        .replace("<topic>/m30/logical_camera</topic>", "")
        .replace("<frame_id>rig/body/logical_camera</frame_id>", "");
    let sim = load(&plain);
    let m = sim
        .world()
        .get::<LogicalCameraPublication>(sensor(&sim))
        .unwrap();
    assert_eq!(
        m.topic,
        "/world/default/model/rig/link/body/sensor/logical_camera/logical_camera"
    );
    assert_eq!(m.frustum_topic, format!("{}/frustum", m.topic));
    assert_eq!(m.frame_id, "rig::body::logical_camera");
}
#[test]
fn late_invalid_input_and_metadata_drift_replace_entire_frame() {
    for field in 0..4 {
        let mut sim = load(XML);
        let e = sensor(&sim);
        sim.step_exact(1).unwrap();
        assert!(
            sim.world()
                .get::<LogicalCameraSample>(e)
                .unwrap()
                .reading
                .is_ok()
        );
        match field {
            0 => sim.world_mut().get_mut::<WorldPose>(e).unwrap().translation[0] = f64::NAN,
            1 => {
                let target = named(&sim, "default::target").0;
                sim.world_mut()
                    .get_mut::<WorldPose>(target)
                    .unwrap()
                    .rotation = [0.0; 4];
            }
            2 => {
                sim.world_mut()
                    .get_mut::<LogicalCameraPublication>(e)
                    .unwrap()
                    .frame_id = "drift".into()
            }
            _ => {
                let target = named(&sim, "default::target").0;
                sim.world_mut().get_mut::<Name>(target).unwrap().0 = "default::rig".into();
            }
        }
        let value = reading(sim.world(), e, sim.world().get::<LogicalCamera>(e).unwrap());
        assert_eq!(value, Err(InvalidLogicalCameraInput));
        // Exclusive sampling uses this same whole-frame oracle. A due invalid
        // publication drift survives pose synchronization and must clear old success.
        if field == 2 {
            sim.step_exact(9).unwrap();
            assert!(
                sim.world()
                    .get::<LogicalCameraSample>(e)
                    .unwrap()
                    .reading
                    .is_err()
            );
        }
    }
}
#[test]
fn schedule_overflow_fault_is_typed_and_never_retains_a_previous_frame() {
    let mut sim = load(&XML.replace(
        "<update_rate>100</update_rate>",
        "<update_rate>1000000000</update_rate>",
    ));
    let e = sensor(&sim);
    sim.step_exact(1).unwrap();
    let info = UpdateInfo {
        sim_time: Duration::MAX,
        dt: Duration::from_nanos(1),
        ..*sim.world().resource::<UpdateInfo>()
    };
    sim.world_mut().insert_resource(info);
    sample(sim.world_mut());
    assert!(sim.world().get::<LogicalCameraSample>(e).is_none());
    let fault = sim.world().resource::<StepFault>().error().unwrap();
    let fault = fault.downcast_ref::<LogicalCameraScheduleFault>().unwrap();
    assert_eq!(fault.entity, e);
    assert_eq!(fault.source, schedule::ScheduleError::TimeOverflow);
}
#[test]
fn raw_config_drift_namespace_noise_and_world_envelope_are_closed() {
    let parsed = sdformat::load_str(XML, "raw", &NoResources).unwrap();
    let original = &parsed.worlds[0].models[0].links[0].sensors[0];
    assert!(config(original).is_ok());
    let mut mixed = original.clone();
    mixed.configuration_xml = mixed
        .configuration_xml
        .replace("<near>0.5</near>", "<near>0.5<!-- gap -->garbage</near>");
    assert!(config(&mixed).is_err());
    let mut spaced = original.clone();
    spaced.configuration_xml = spaced.configuration_xml.replace(
        "<near>0.5</near>",
        "<!-- root comment --><near>  0.5\n </near>",
    );
    assert!(config(&spaced).is_ok());
    for raw in [
        "<logical_camera/>",
        "<logical_camera xmlns='urn:x'><near>1</near></logical_camera>",
        "<logical_camera><near><![CDATA[1]]></near></logical_camera>",
        "<logical_camera><?hidden x?><near>1</near></logical_camera>",
        "<logical_camera><near extra='x'>1</near></logical_camera>",
    ] {
        let mut s = original.clone();
        s.configuration_xml = raw.into();
        assert!(config(&s).is_err());
    }
    let mut s = original.clone();
    s.kind = "air_speed".into();
    assert!(config(&s).is_err());
    let mut s = original.clone();
    s.config = sdformat::SensorConfig::AirSpeed(Box::default());
    assert!(config(&s).is_err());
    for edit in 0..4 {
        let sdf = sdformat::load_str(XML, "world", &NoResources).unwrap();
        let mut world = sdformat::load_str(XML, "owned", &NoResources)
            .unwrap()
            .worlds
            .remove(0);
        match edit {
            0 => world.models[1].name = "rig".into(),
            1 => {
                let child = world.models[2].clone();
                world.models[1].models.push(child);
            }
            2 => world.models[0].links[0].sensors[0].visualize = true,
            _ => world.models[0].links[0].sensors[0].topic = format!("/{}", "a".repeat(65535)),
        };
        assert!(preflight(&sdf, &world, PhysicsEngine::Rapier).is_err());
    }
}
#[test]
fn rates_continuous_and_very_fast_preserve_one_sample_per_tick_and_removed_target_is_not_cached() {
    for rate in ["0", "-1", "1000", "1000000000000"] {
        let mut sim = load(&XML.replace(
            "<update_rate>100</update_rate>",
            &format!("<update_rate>{rate}</update_rate>"),
        ));
        let e = sensor(&sim);
        for tick in 1..=3 {
            sim.step_exact(1).unwrap();
            assert_eq!(
                sim.world().get::<LogicalCameraSample>(e).unwrap().time,
                Duration::from_millis(tick)
            );
        }
    }
    let mut sim = load(XML);
    let e = sensor(&sim);
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world()
            .get::<LogicalCameraSample>(e)
            .unwrap()
            .reading
            .as_ref()
            .unwrap()
            .models
            .len(),
        2
    );
    let target = named(&sim, "default::target").1;
    remove_model(&mut sim, target).unwrap();
    sim.step_exact(9).unwrap();
    assert_eq!(
        sim.world()
            .get::<LogicalCameraSample>(e)
            .unwrap()
            .reading
            .as_ref()
            .unwrap()
            .models
            .iter()
            .map(|m| m.name.as_str())
            .collect::<Vec<_>>(),
        ["rig"]
    );
}
