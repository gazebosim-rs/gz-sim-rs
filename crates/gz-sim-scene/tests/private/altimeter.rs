// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::entity_by_id;
use sdformat::NoResources;

const XML: &str = include_str!("../fixtures/m13_altimeter.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m13-altimeter.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .expect("admitted altimeter fixture")
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .expect("fixture entity");
    (
        entity.id(),
        entity.get::<EntityId>().expect("public identity").get(),
    )
}
fn sensor(sim: &Simulation) -> Entity {
    named(sim, "default::rig::body::altimeter").0
}
fn without_sensor() -> String {
    let start = XML.find("    <sensor ").expect("sensor start");
    let end = start + XML[start..].find("</sensor>").expect("sensor end") + "</sensor>".len();
    format!("{}{}", &XML[..start], &XML[end..])
}

#[test]
fn absent_altimeter_owns_no_callbacks_marker_interest_or_samples() {
    let mut absent = load(&without_sensor());
    let present = load(XML);
    let counts = absent.scheduled_system_counts();
    assert_eq!(
        present.scheduled_system_counts(),
        (counts.0 + 1, counts.1 + 1)
    );
    assert!(!absent.world().contains_resource::<Installed>());
    set_altimeter_interest(&mut absent, []);
    assert!(!absent.world().contains_resource::<Interest>());
    absent.step_exact(3).expect("advancing absence");
    absent.reset().expect("reset absence");
    assert_eq!(absent.scheduled_system_counts(), counts);
    assert!(!absent.world().contains_resource::<Installed>());
    assert!(!absent.world().contains_resource::<Interest>());
}

#[test]
fn native_world_z_reference_and_actual_offset_point_velocity_are_preserved() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    let (link, link_id) = named(&sim, "default::rig::body");
    let metadata = sim
        .world()
        .get::<AltimeterPublication>(entity)
        .expect("metadata");
    assert_eq!(metadata.vertical_reference, 3.5);
    assert_eq!(
        (
            metadata.entity_name.as_str(),
            metadata.frame_id.as_str(),
            metadata.topic.as_str()
        ),
        (
            "rig::body::altimeter",
            "rig/body/altimeter",
            "/m13/altimeter"
        )
    );
    queue_link_world_velocity(&mut sim, link_id, [0.0, 0.0, 3.0], [0.0, 2.0, 0.0])
        .expect("finite offset-point motion");
    sim.step_exact(1).expect("real Rapier advance");
    let world = sim.world();
    let pose = world.get::<WorldPose>(link).expect("link pose");
    let velocity = world.get::<Velocity>(link).expect("link velocity");
    let [x, y, z, w] = pose.rotation;
    // Independent scalar expansion of R*[1,0,0.5], not the sensor's orientation.
    let offset_x = 1.0 - 2.0 * (y * y + z * z) + (x * z + y * w);
    let offset_y = 2.0 * (x * y + z * w) + (y * z - x * w);
    let offset_z = 2.0 * (x * z - y * w) + 0.5 * (1.0 - 2.0 * (x * x + y * y));
    let expected_vz =
        velocity.linear[2] + velocity.angular[0] * offset_y - velocity.angular[1] * offset_x;
    let sample = world.get::<AltimeterSample>(entity).expect("fresh sample");
    assert_eq!((sample.epoch, sample.time), (0, Duration::from_millis(1)));
    let reading = sample.reading.as_ref().expect("valid point measurement");
    assert!((reading.vertical_velocity - expected_vz).abs() < 1e-12);
    assert!((reading.vertical_velocity - velocity.linear[2]).abs() > 1.9);
    assert!((reading.vertical_position - (pose.translation[2] + offset_z - 3.5)).abs() < 1e-12);
    assert_eq!(reading.vertical_reference, 3.5);
    assert_eq!(
        world
            .get::<AltimeterPublication>(entity)
            .expect("fixed metadata")
            .vertical_reference,
        3.5
    );
}

#[test]
fn demand_fixed_cadence_and_pause_do_not_advance_unrequested_measurements() {
    let mut sim = load(XML);
    let entity = sensor(&sim);
    set_altimeter_interest(&mut sim, []);
    sim.step_exact(20).expect("no demand advance");
    assert!(sim.world().get::<AltimeterSample>(entity).is_none());
    assert!(
        sim.world()
            .get::<Altimeter>(entity)
            .expect("sensor")
            .schedule
            .is_due(sim.elapsed())
    );
    set_altimeter_interest(&mut sim, [entity, entity]);
    assert_eq!(sim.world().resource::<Interest>().0.len(), 1);
    sim.step_exact(1).expect("first demanded acquisition");
    let first = sim
        .world()
        .get::<AltimeterSample>(entity)
        .expect("due sample")
        .time;
    assert_eq!(first, Duration::from_millis(21));
    sim.tick().expect("paused poll");
    assert_eq!(
        sim.world()
            .get::<AltimeterSample>(entity)
            .expect("retained sample")
            .time,
        first
    );
    sim.step_exact(8).expect("before phase-aligned deadline");
    assert_eq!(
        sim.world()
            .get::<AltimeterSample>(entity)
            .expect("retained sample")
            .time,
        first
    );
    sim.step_exact(1).expect("phase-aligned acquisition");
    assert_eq!(
        sim.world()
            .get::<AltimeterSample>(entity)
            .expect("sample")
            .time,
        Duration::from_millis(30)
    );
    set_altimeter_interest(&mut sim, []);
    assert!(sim.world().get::<AltimeterSample>(entity).is_none());
    clear_altimeter_interest(&mut sim);
    sim.step_exact(10).expect("local manual demand");
    assert_eq!(
        sim.world()
            .get::<AltimeterSample>(entity)
            .expect("sample")
            .time,
        Duration::from_millis(40)
    );
}

#[test]
fn last_model_removal_retires_callbacks_and_loaded_reset_restores_identity_reference() {
    let mut sim = load(XML);
    let (entity, identity) = named(&sim, "default::rig::body::altimeter");
    let model = named(&sim, "default::rig").1;
    let counts = sim.scheduled_system_counts();
    sim.step_exact(12).expect("sample before removal");
    remove_model(&mut sim, model).expect("remove owned model");
    assert!(entity_by_id(sim.world(), identity).is_none());
    assert!(!sim.world().contains_resource::<Installed>());
    assert!(!sim.world().contains_resource::<Interest>());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    set_altimeter_interest(&mut sim, [entity]);
    assert!(!sim.world().contains_resource::<Interest>());
    initial::reset_loaded_world(&mut sim, |sim| {
        let restored = entity_by_id(sim.world(), identity).expect("same public identity");
        assert!(sim.world().get::<AltimeterSample>(restored).is_none());
        assert_eq!(
            sim.world()
                .get::<AltimeterPublication>(restored)
                .expect("metadata")
                .vertical_reference,
            3.5
        );
    })
    .expect("loaded scene reconstruction");
    assert_eq!(sim.scheduled_system_counts(), counts);
    let restored = entity_by_id(sim.world(), identity).expect("restored sensor");
    sim.step_exact(1).expect("new epoch acquisition");
    let sample = sim
        .world()
        .get::<AltimeterSample>(restored)
        .expect("reset sample");
    assert_eq!((sample.epoch, sample.time), (1, Duration::from_millis(1)));
    assert_eq!(
        sample.reading.as_ref().expect("reading").vertical_position,
        0.0
    );
}

#[test]
fn admission_refuses_nondefault_noise_metadata_second_sensor_and_other_engines() {
    let original = "<altimeter/>";
    for replacement in [
        "<altimeter><vertical_position><noise type='gaussian'><stddev>0.1</stddev></noise></vertical_position></altimeter>",
        "<altimeter><vertical_velocity><noise type='gaussian'><mean>1</mean></noise></vertical_velocity></altimeter>",
        "<altimeter/><visualize>true</visualize>",
        "<altimeter/><enable_metrics>true</enable_metrics>",
        "<altimeter/><alwaysOn>true</alwaysOn>",
        "<altimeter/><plugin name='custom' filename='custom'/>",
    ] {
        let xml = XML.replace(original, replacement);
        sdformat::load_str(&xml, "altimeter-parsed-negative", &NoResources)
            .expect("negative remains a parsed typed sensor, not an XML rejection");
        assert!(matches!(
            load_world_with_sdf_step(
                &xml,
                "altimeter-refusal",
                "default",
                &NoResources,
                PhysicsEngine::Rapier
            ),
            Err(LoadError::Unsupported { .. })
        ));
    }
    let tiny = XML.replace(
        "<update_rate>100</update_rate>",
        "<update_rate>1e-300</update_rate>",
    );
    sdformat::load_str(&tiny, "altimeter-tiny-rate", &NoResources)
        .expect("finite positive SDF rate");
    assert!(matches!(
        load_world_with_sdf_step(
            &tiny,
            "altimeter-tiny-rate",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "unrepresentable sensor rate",
            ..
        })
    ));
    let twice = XML.replace(
        "</sensor>",
        "</sensor><sensor name='second' type='altimeter'><altimeter/></sensor>",
    );
    assert!(matches!(
        load_world_with_sdf_step(
            &twice,
            "two-altimeters",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "multiple altimeters",
            ..
        })
    ));
    assert!(matches!(
        load_world_with_sdf_step(
            XML,
            "altimeter-engine",
            "default",
            &NoResources,
            PhysicsEngine::Tpe
        ),
        Err(LoadError::Unsupported {
            feature: "Rapier ideal altimeter",
            ..
        })
    ));
}

#[test]
fn omitted_topic_and_frame_use_native_names_and_canonical_frame_wins_over_legacy() {
    let plain = XML
        .replace("<topic>/m13/altimeter</topic>", "")
        .replace("<frame_id>rig/body/altimeter</frame_id>", "");
    let sim = load(&plain);
    let metadata = sim
        .world()
        .get::<AltimeterPublication>(sensor(&sim))
        .expect("default metadata");
    assert_eq!(
        metadata.topic,
        "/world/default/model/rig/link/body/sensor/altimeter/altimeter"
    );
    assert_eq!(metadata.frame_id, "rig::body::altimeter");
    let legacy = plain.replace(
        "<altimeter/>",
        "<gz_frame_id>legacy</gz_frame_id><altimeter/>",
    );
    let sim = load(&legacy);
    assert_eq!(
        sim.world()
            .get::<AltimeterPublication>(sensor(&sim))
            .expect("legacy metadata")
            .frame_id,
        "legacy"
    );
    let both = XML.replace(
        "<altimeter/>",
        "<gz_frame_id>legacy</gz_frame_id><altimeter/>",
    );
    let sim = load(&both);
    assert_eq!(
        sim.world()
            .get::<AltimeterPublication>(sensor(&sim))
            .expect("canonical metadata")
            .frame_id,
        "rig/body/altimeter"
    );
}
