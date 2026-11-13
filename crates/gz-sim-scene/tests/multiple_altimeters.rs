// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec3};
use gz_sensors::altimeter::AltimeterReading;
use gz_sim_core::{EntityId, Simulation, StepError, StepFault, entity_by_id};
use gz_sim_scene::{
    AltimeterPublication, AltimeterSample, LoadError, Name, PhysicsEngine, SceneResetError,
    Velocity, WorldPose, clear_altimeter_interest, load_world_seeded, queue_link_world_wrench,
    remove_model, reset_loaded_world, set_altimeter_interest,
};
use std::time::Duration;

const XML: &str = include_str!("fixtures/m13_altimeter.sdf");
const OFFSETS: [[f64; 3]; 2] = [[1.0, 0.0, 0.5], [-0.5, 0.25, 1.25]];
fn sensor_xml(index: usize, rate: u32, offset: [f64; 3], noise: &str) -> String {
    format!(
        "<sensor name='altimeter{index}' type='altimeter'><pose>{} {} {} 0 0 0</pose><update_rate>{rate}</update_rate><topic>/m49/altimeter{index}</topic><frame_id>rig/body/altimeter{index}</frame_id><altimeter>{noise}</altimeter></sensor>",
        offset[0], offset[1], offset[2]
    )
}
fn world(sensors: &str) -> String {
    let start = XML.find("    <sensor ").unwrap();
    let end = start + XML[start..].find("</sensor>").unwrap() + "</sensor>".len();
    format!("{}{}{}", &XML[..start], sensors, &XML[end..])
}
fn pair(noise0: &str, noise1: &str) -> String {
    world(&format!(
        "{}{}",
        sensor_xml(0, 100, OFFSETS[0], noise0),
        sensor_xml(1, 50, OFFSETS[1], noise1)
    ))
}
fn try_load(xml: &str, seed: u8) -> Result<Simulation, LoadError> {
    load_world_seeded(
        xml,
        "m49.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [seed; 32],
    )
}
fn load(xml: &str, seed: u8) -> Simulation {
    try_load(xml, seed).unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (e.id(), e.get::<EntityId>().unwrap().get())
}
fn sensor(sim: &Simulation, index: usize) -> Entity {
    named(sim, &format!("default::rig::body::altimeter{index}")).0
}
fn sample_at(sim: &Simulation, entity: Entity) -> (Duration, u64, AltimeterReading) {
    let sample = sim.world().get::<AltimeterSample>(entity).unwrap();
    (sample.time, sample.epoch, *sample.reading.as_ref().unwrap())
}
fn sample(sim: &Simulation, index: usize) -> (Duration, u64, AltimeterReading) {
    sample_at(sim, sensor(sim, index))
}
fn pose(sim: &Simulation, name: &str) -> ([f64; 3], [f64; 4]) {
    let p = sim.world().get::<WorldPose>(named(sim, name).0).unwrap();
    (p.translation, p.rotation)
}
fn drive(sim: &mut Simulation, name: &str) {
    let id = named(sim, name).1;
    queue_link_world_wrench(sim, id, [8.0, -4.0, 2.0], [2.0, -3.0, 4.0]).unwrap();
}
fn geometry(sim: &Simulation, index: usize, means: [f64; 2]) {
    let entity = sensor(sim, index);
    let current = sample(sim, index);
    assert_eq!(
        current.0,
        sim.elapsed(),
        "geometry uses an actual fresh acquisition"
    );
    let p = sim.world().get::<WorldPose>(entity).unwrap();
    let body = named(sim, "default::rig::body").0;
    let link = sim.world().get::<WorldPose>(body).unwrap();
    let velocity = sim.world().get::<Velocity>(body).unwrap();
    // Independent geometric point-velocity expansion, not a scalar-sensor oracle.
    let arm = DQuat::from_array(link.rotation) * DVec3::from_array(OFFSETS[index]);
    let vz = velocity.linear[2] + velocity.angular[0] * arm.y - velocity.angular[1] * arm.x;
    let reference = sim
        .world()
        .get::<AltimeterPublication>(entity)
        .unwrap()
        .vertical_reference;
    assert!(
        (current.2.vertical_position - (p.translation[2] - reference + means[0])).abs() < 1e-12
    );
    assert!((current.2.vertical_velocity - (vz + means[1])).abs() < 1e-12);
    assert_eq!(current.2.vertical_reference, reference);
}

#[test]
fn distinct_references_rates_and_offset_geometry_follow_real_motion_and_reset() {
    let mean = "<vertical_position><noise type='gaussian'><mean>0.25</mean></noise></vertical_position><vertical_velocity><noise type='gaussian'><mean>-0.5</mean></noise></vertical_velocity>";
    let mut sim = load(&pair("", mean), 49);
    let identities = [0, 1].map(|i| sim.world().get::<EntityId>(sensor(&sim, i)).unwrap().get());
    let metadata = [0, 1].map(|i| {
        sim.world()
            .get::<AltimeterPublication>(sensor(&sim, i))
            .unwrap()
            .clone()
    });
    assert_eq!(
        [
            metadata[0].vertical_reference,
            metadata[1].vertical_reference
        ],
        [3.5, 4.25]
    );
    let initial = pose(&sim, "default::rig::body");
    let counts = sim.scheduled_system_counts();
    drive(&mut sim, "default::rig::body");
    let mut previous = [None, None];
    let mut stamps = [Vec::new(), Vec::new()];
    for _ in 0..41 {
        sim.step_exact(1).unwrap();
        for (index, (last, times)) in previous.iter_mut().zip(&mut stamps).enumerate() {
            let value = sample(&sim, index);
            if value.0 == sim.elapsed() {
                geometry(
                    &sim,
                    index,
                    if index == 0 { [0.0; 2] } else { [0.25, -0.5] },
                );
                times.push(value.0);
            } else {
                assert_eq!(Some(value), *last);
            }
            *last = Some(value);
        }
    }
    assert_eq!(stamps[0], [1, 10, 20, 30, 40].map(Duration::from_millis));
    assert_eq!(stamps[1], [1, 20, 40].map(Duration::from_millis));
    assert_ne!(pose(&sim, "default::rig::body").0, initial.0);
    assert_ne!(pose(&sim, "default::rig::body").1, initial.1);
    assert!(sample(&sim, 0).2.vertical_velocity.abs() > 0.001);
    let final_samples = [sample(&sim, 0), sample(&sim, 1)];
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(pose(&sim, "default::rig::body"), initial);
    assert_eq!(sim.scheduled_system_counts(), counts);
    for (index, (identity, metadata)) in identities.into_iter().zip(&metadata).enumerate() {
        assert_eq!(
            entity_by_id(sim.world(), identity),
            Some(sensor(&sim, index))
        );
        assert!(
            sim.world()
                .get::<AltimeterSample>(sensor(&sim, index))
                .is_none()
        );
        assert_eq!(
            sim.world().get::<AltimeterPublication>(sensor(&sim, index)),
            Some(metadata)
        );
    }
    drive(&mut sim, "default::rig::body");
    sim.step_exact(41).unwrap();
    for (index, first) in final_samples.into_iter().enumerate() {
        let value = sample(&sim, index);
        assert_eq!((value.0, value.2), (first.0, first.2));
        assert_eq!(value.1, first.1 + 1);
    }
}

#[test]
fn independent_demand_preserves_seeded_draws_deadlines_pause_and_equal_time() {
    let noise = "<vertical_position><noise type='gaussian'><stddev>0.3</stddev><bias_mean>0.2</bias_mean><bias_stddev>0.1</bias_stddev></noise></vertical_position><vertical_velocity><noise type='gaussian'><stddev>0.2</stddev></noise></vertical_velocity>";
    let xml = pair(noise, noise);
    let mut selected = load(&xml, 49);
    let mut all = load(&xml, 49);
    let mut first_b = load(&xml, 49);
    let mut other_seed = load(&xml, 50);
    let a = sensor(&selected, 0);
    let b = sensor(&selected, 1);
    set_altimeter_interest(&mut selected, [a]);
    first_b.step_exact(1).unwrap();
    assert_ne!(
        (
            sample(&first_b, 0).2.vertical_position,
            sample(&first_b, 0).2.vertical_velocity
        ),
        (
            sample(&first_b, 1).2.vertical_position,
            sample(&first_b, 1).2.vertical_velocity
        )
    );
    for _ in 0..50 {
        selected.step_exact(1).unwrap();
        all.step_exact(1).unwrap();
        other_seed.step_exact(1).unwrap();
        assert_eq!(sample(&selected, 0), sample(&all, 0));
        assert!(selected.world().get::<AltimeterSample>(b).is_none());
    }
    assert_ne!(sample(&selected, 0).2, sample(&other_seed, 0).2);
    set_altimeter_interest(&mut selected, [a, b]);
    selected.step_exact(1).unwrap();
    assert_eq!(sample(&selected, 1).0, Duration::from_millis(51));
    assert_eq!(sample(&selected, 1).2, sample(&first_b, 1).2);
    // Clear only B; A continues to match an all-connected same-seed peer.
    set_altimeter_interest(&mut selected, [a]);
    assert!(selected.world().get::<AltimeterSample>(b).is_none());
    let held = sample(&selected, 0);
    selected.set_paused(true);
    selected.tick().unwrap();
    selected.tick().unwrap();
    assert_eq!(selected.elapsed(), Duration::from_millis(51));
    assert_eq!(sample(&selected, 0), held);
    selected.set_paused(false);
    selected.step_exact(8).unwrap();
    all.step_exact(9).unwrap();
    assert_eq!(sample(&selected, 0), held);
    selected.step_exact(1).unwrap();
    all.step_exact(1).unwrap();
    assert_eq!(sample(&selected, 0), sample(&all, 0));
    assert_eq!(sample(&selected, 0).0, Duration::from_millis(60));
    set_altimeter_interest(&mut selected, [a, b]);
    selected.step_exact(1).unwrap();
    first_b.step_exact(19).unwrap();
    assert_eq!(sample(&selected, 1).0, Duration::from_millis(61));
    assert_eq!(sample(&selected, 1).2, sample(&first_b, 1).2);
    clear_altimeter_interest(&mut selected);
    selected.step_exact(19).unwrap();
    assert_eq!(sample(&selected, 1).0, Duration::from_millis(80));
    reset_loaded_world(&mut selected, |_| ()).unwrap();
    selected.step_exact(1).unwrap();
    assert_eq!(sample(&selected, 1).2, sample(&load_first(&xml, 49), 1).2);
}
fn load_first(xml: &str, seed: u8) -> Simulation {
    let mut sim = load(xml, seed);
    sim.step_exact(1).unwrap();
    sim
}

#[test]
fn eight_sensors_share_one_callback_pair_and_ninth_or_late_invalid_sensor_refuses() {
    let sensors: String = (0..8)
        .map(|i| sensor_xml(i, 100, [0.0, 0.0, i as f64 * 0.1], ""))
        .collect();
    let mut eight = load(&world(&sensors), 49);
    let one = load(&world(&sensor_xml(0, 100, [0.0; 3], "")), 49);
    let absent = load(&world(""), 49);
    assert_eq!(
        eight.scheduled_system_counts(),
        one.scheduled_system_counts()
    );
    let counts = absent.scheduled_system_counts();
    assert_eq!(one.scheduled_system_counts(), (counts.0 + 1, counts.1 + 1));
    eight.step_exact(1).unwrap();
    for index in 0..8 {
        let value = sample(&eight, index);
        assert_eq!(value.0, Duration::from_millis(1));
        assert_eq!(
            (value.2.vertical_position, value.2.vertical_velocity),
            (0.0, 0.0)
        );
        assert_eq!(value.2.vertical_reference, 3.0 + index as f64 * 0.1);
    }
    let ninth = world(&format!("{sensors}{}", sensor_xml(8, 100, [0.0; 3], "")));
    assert!(matches!(
        try_load(&ninth, 49),
        Err(LoadError::Unsupported {
            feature: "multiple altimeters",
            ..
        })
    ));
    let invalid = sensor_xml(1, 100, [0.0; 3], "").replace(
        "</sensor>",
        "<enable_metrics>true</enable_metrics></sensor>",
    );
    assert!(matches!(
        try_load(
            &world(&format!("{}{invalid}", sensor_xml(0, 100, [0.0; 3], ""))),
            49
        ),
        Err(LoadError::Unsupported {
            feature: "sensor metrics",
            ..
        })
    ));
    // A failed fresh load does not mutate the already-owned world.
    assert_eq!(sample(&eight, 0).0, Duration::from_millis(1));
    assert_eq!(
        eight.scheduled_system_counts(),
        one.scheduled_system_counts()
    );
}

#[test]
fn removing_one_owner_preserves_peer_then_last_retirement_and_loaded_restoration() {
    let single = world(&sensor_xml(0, 100, OFFSETS[0], ""));
    let begin = single.find("  <model ").unwrap();
    let end = single.find("</model>").unwrap() + "</model>".len();
    let second = single[begin..end]
        .replace("name='rig'", "name='peer'")
        .replace("<pose>0 0 3 0 0 0</pose>", "<pose>5 0 7 0 0 0</pose>")
        .replace("/m49/altimeter0", "/m49/peer");
    let xml = format!("{}{}{}", &single[..end], second, &single[end..]);
    let mut sim = load(&xml, 49);
    let first_id = named(&sim, "default::rig::body::altimeter0").1;
    let peer_id = named(&sim, "default::peer::body::altimeter0").1;
    let first_model = named(&sim, "default::rig").1;
    let peer_model = named(&sim, "default::peer").1;
    let counts = sim.scheduled_system_counts();
    drive(&mut sim, "default::peer::body");
    sim.step_exact(10).unwrap();
    let before = sample_at(&sim, entity_by_id(sim.world(), peer_id).unwrap());
    remove_model(&mut sim, first_model).unwrap();
    assert!(entity_by_id(sim.world(), first_id).is_none());
    assert_eq!(sim.scheduled_system_counts(), counts);
    sim.step_exact(10).unwrap();
    let peer = entity_by_id(sim.world(), peer_id).unwrap();
    assert_eq!(sample_at(&sim, peer).0, Duration::from_millis(20));
    assert_ne!(
        sample_at(&sim, peer).2.vertical_position,
        before.2.vertical_position
    );
    remove_model(&mut sim, peer_model).unwrap();
    assert!(entity_by_id(sim.world(), peer_id).is_none());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    set_altimeter_interest(&mut sim, []);
    sim.step_exact(2).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.scheduled_system_counts(), counts);
    for id in [first_id, peer_id] {
        let e = entity_by_id(sim.world(), id).unwrap();
        assert!(sim.world().get::<AltimeterSample>(e).is_none());
    }
    sim.step_exact(1).unwrap();
    assert_eq!(sample(&sim, 0).2.vertical_reference, 3.5);
    assert_eq!(
        sample_at(&sim, entity_by_id(sim.world(), peer_id).unwrap())
            .2
            .vertical_reference,
        7.5
    );
}

#[test]
fn first_sensor_default_and_seeded_paths_remain_exact_when_a_peer_is_added() {
    for noise in [
        "",
        "<vertical_position><noise type='gaussian'><stddev>0.2</stddev></noise></vertical_position>",
    ] {
        let mut single = load(&world(&sensor_xml(0, 100, OFFSETS[0], noise)), 49);
        let mut multi = load(&pair(noise, ""), 49);
        assert_eq!(
            named(&single, "default::rig::body::altimeter0").1,
            named(&multi, "default::rig::body::altimeter0").1
        );
        drive(&mut single, "default::rig::body");
        drive(&mut multi, "default::rig::body");
        for _ in 0..31 {
            single.step_exact(1).unwrap();
            multi.step_exact(1).unwrap();
            assert_eq!(sample(&single, 0), sample(&multi, 0));
            assert_eq!(
                pose(&single, "default::rig::body"),
                pose(&multi, "default::rig::body")
            );
        }
    }
}

#[test]
fn terminal_reset_refusal_preserves_all_sensor_samples_metadata_and_poses() {
    let mut sim = load(&pair("", ""), 49);
    drive(&mut sim, "default::rig::body");
    sim.step_exact(20).unwrap();
    let before = [sample(&sim, 0), sample(&sim, 1)];
    let metadata = [0, 1].map(|i| {
        sim.world()
            .get::<AltimeterPublication>(sensor(&sim, i))
            .unwrap()
            .clone()
    });
    let body = pose(&sim, "default::rig::body");
    let clock = (sim.elapsed(), sim.epoch(), sim.is_paused());
    sim.world()
        .resource::<StepFault>()
        .report(std::io::Error::other("M49 terminal reset refusal"));
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    let mut called = false;
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| called = true),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert!(!called);
    assert_eq!((sim.elapsed(), sim.epoch(), sim.is_paused()), clock);
    assert_eq!(pose(&sim, "default::rig::body"), body);
    for (index, (before, metadata)) in before.into_iter().zip(&metadata).enumerate() {
        assert_eq!(sample(&sim, index), before);
        assert_eq!(
            sim.world().get::<AltimeterPublication>(sensor(&sim, index)),
            Some(metadata)
        );
    }
}
