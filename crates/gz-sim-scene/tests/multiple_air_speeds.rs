// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec3};
use gz_sensors::air_speed::{
    AirSpeedInput, AirSpeedReading, air_speed, air_speed_unscaled, scale_air_speed,
};
use gz_sim_core::{EntityId, Simulation, StepError, StepFault, entity_by_id};
use gz_sim_scene::{
    AirSpeedPublication, AirSpeedSample, LoadError, Name, PhysicsEngine, SceneResetError, Velocity,
    WorldPose, clear_air_speed_interest, load_world_seeded, queue_link_world_wrench, remove_model,
    reset_loaded_world, set_air_speed_interest,
};
use std::time::Duration;

const XML: &str = include_str!("fixtures/m26_air_speed.sdf");
const OFFSETS: [[f64; 3]; 2] = [[0.0, 0.5, 0.5], [1.0, -0.25, -0.5]];
const WIND: [f64; 3] = [1.0, -2.0, 0.5];
fn wind(xml: &str, vector: [f64; 3]) -> String {
    xml.replace(
        "<gravity>0 0 0</gravity>",
        &format!(
            "<gravity>0 0 0</gravity><wind><linear_velocity>{} {} {}</linear_velocity></wind>",
            vector[0], vector[1], vector[2]
        ),
    )
}
fn sensor_xml(index: usize, rate: u32, offset: [f64; 3], noise: &str) -> String {
    let rpy = if index.is_multiple_of(2) {
        "0 0 0.7853981633974483"
    } else {
        "0.2 -0.3 -0.4"
    };
    format!(
        "<sensor name='air_speed{index}' type='air_speed'><pose>{} {} {} {rpy}</pose><update_rate>{rate}</update_rate><topic>/m51/air_speed{index}</topic><frame_id>rig/body/air_speed{index}</frame_id><air_speed><pressure>{noise}</pressure></air_speed></sensor>",
        offset[0], offset[1], offset[2]
    )
}
fn world(sensors: &str) -> String {
    let start = XML.find("    <sensor ").unwrap();
    let end = start + XML[start..].find("</sensor>").unwrap() + "</sensor>".len();
    format!("{}{}{}", &XML[..start], sensors, &XML[end..])
}
fn pair(noise0: &str, noise1: &str) -> String {
    wind(
        &world(&format!(
            "{}{}",
            sensor_xml(0, 100, OFFSETS[0], noise0),
            sensor_xml(1, 50, OFFSETS[1], noise1)
        )),
        WIND,
    )
}
fn try_load(xml: &str, seed: u8) -> Result<Simulation, LoadError> {
    load_world_seeded(
        xml,
        "m51.sdf",
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
    named(sim, &format!("default::rig::body::air_speed{index}")).0
}
fn sample_at(sim: &Simulation, entity: Entity) -> (Duration, u64, AirSpeedReading) {
    let sample = sim.world().get::<AirSpeedSample>(entity).unwrap();
    (sample.time, sample.epoch, *sample.reading.as_ref().unwrap())
}
fn sample(sim: &Simulation, index: usize) -> (Duration, u64, AirSpeedReading) {
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
// Independent point/attachment geometry retains M41's established 1e-6
// pressure bound. Exact native scalar/wire bits remain separate fixture controls.
fn expected(
    sim: &Simulation,
    entity: Entity,
    offset: [f64; 3],
    wind: [f64; 3],
    noise: Option<[f64; 2]>,
) -> AirSpeedReading {
    let pose = sim.world().get::<WorldPose>(entity).unwrap();
    let link = sim
        .world()
        .get::<WorldPose>(named(sim, "default::rig::body").0)
        .unwrap();
    let velocity = sim
        .world()
        .get::<Velocity>(named(sim, "default::rig::body").0)
        .unwrap();
    let arm = DQuat::from_array(link.rotation) * DVec3::from_array(offset);
    assert!((pose.translation[2] - (link.translation[2] + arm.z)).abs() < 1e-12);
    let point = DVec3::from_array(velocity.linear) + DVec3::from_array(velocity.angular).cross(arm);
    let frame = DQuat::from_array(pose.rotation).inverse();
    let relative = frame * point - frame * DVec3::from_array(wind);
    let input = AirSpeedInput {
        world_z: pose.translation[2],
        velocity_sensor: relative.to_array(),
    };
    if let Some([mean, precision]) = noise {
        let (precursor, temperature) = air_speed_unscaled(input).unwrap();
        let value = f64::from(precursor) + mean;
        let quantized = if precision > 1e-6 {
            (value / precision).round() * precision
        } else {
            value
        };
        scale_air_speed(quantized as f32, temperature).unwrap()
    } else {
        air_speed(input).unwrap()
    }
}
fn geometry(sim: &Simulation, index: usize, noise: Option<[f64; 2]>) {
    let current = sample(sim, index);
    assert_eq!(
        current.0,
        sim.elapsed(),
        "geometry uses an actual fresh acquisition"
    );
    let ideal = expected(sim, sensor(sim, index), OFFSETS[index], WIND, noise);
    assert!((current.2.diff_pressure - ideal.diff_pressure).abs() < 1e-6);
    assert_eq!(current.2.temperature, ideal.temperature);
}

#[test]
fn distinct_frames_rates_and_rotated_point_airflow_follow_real_motion_and_reset() {
    let mean =
        "<noise type='gaussian_quantized'><mean>0.1</mean><precision>0.001</precision></noise>";
    let mut sim = load(&pair("", mean), 51);
    let identities = [0, 1].map(|i| sim.world().get::<EntityId>(sensor(&sim, i)).unwrap().get());
    let metadata = [0, 1].map(|i| {
        sim.world()
            .get::<AirSpeedPublication>(sensor(&sim, i))
            .unwrap()
            .clone()
    });
    assert_eq!(
        [0, 1].map(|i| sim
            .world()
            .get::<WorldPose>(sensor(&sim, i))
            .unwrap()
            .translation[2]),
        [3.5, 2.5]
    );
    for (index, meta) in metadata.iter().enumerate() {
        assert_eq!(meta.entity_name, format!("rig::body::air_speed{index}"));
        assert_eq!(meta.topic, format!("/m51/air_speed{index}"));
        assert_eq!(meta.frame_id, format!("rig/body/air_speed{index}"));
    }
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
                    if index == 0 { None } else { Some([0.1, 0.001]) },
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
    let stationary = load_first(&pair("", mean), 51);
    assert_ne!(
        sample(&sim, 0).2.diff_pressure,
        sample(&stationary, 0).2.diff_pressure
    );
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
                .get::<AirSpeedSample>(sensor(&sim, index))
                .is_none()
        );
        assert_eq!(
            sim.world().get::<AirSpeedPublication>(sensor(&sim, index)),
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
    let noise = "<noise type='gaussian'><stddev>0.02</stddev><bias_mean>0.01</bias_mean><bias_stddev>0.005</bias_stddev><dynamic_bias_stddev>100</dynamic_bias_stddev><dynamic_bias_correlation_time>2</dynamic_bias_correlation_time></noise>";
    let xml = pair(noise, noise);
    let mut selected = load(&xml, 51);
    let mut all = load(&xml, 51);
    let mut first_b = load(&xml, 51);
    let mut other_seed = load(&xml, 52);
    let a = sensor(&selected, 0);
    let b = sensor(&selected, 1);
    set_air_speed_interest(&mut selected, [a]);
    first_b.step_exact(1).unwrap();
    // Different identity streams are compared on a stationary world; delayed
    // acquisitions use the same physical inputs, not an extrapolated trajectory.
    let residual = |index| {
        sample(&first_b, index).2.diff_pressure
            - expected(
                &first_b,
                sensor(&first_b, index),
                OFFSETS[index],
                WIND,
                None,
            )
            .diff_pressure
    };
    assert_ne!(residual(0), residual(1));
    for (index, offset) in OFFSETS.into_iter().enumerate() {
        assert_eq!(
            sample(&first_b, index).2.temperature,
            expected(&first_b, sensor(&first_b, index), offset, WIND, None).temperature
        );
    }
    for _ in 0..50 {
        selected.step_exact(1).unwrap();
        all.step_exact(1).unwrap();
        other_seed.step_exact(1).unwrap();
        assert_eq!(sample(&selected, 0), sample(&all, 0));
        assert!(selected.world().get::<AirSpeedSample>(b).is_none());
    }
    assert_ne!(sample(&selected, 0).2, sample(&other_seed, 0).2);
    set_air_speed_interest(&mut selected, [a, b]);
    selected.step_exact(1).unwrap();
    assert_eq!(sample(&selected, 1).0, Duration::from_millis(51));
    assert_eq!(sample(&selected, 1).2, sample(&first_b, 1).2);
    // Clear only B; A continues to match an all-connected same-seed peer.
    set_air_speed_interest(&mut selected, [a]);
    assert!(selected.world().get::<AirSpeedSample>(b).is_none());
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
    set_air_speed_interest(&mut selected, [a, b]);
    selected.step_exact(1).unwrap();
    first_b.step_exact(19).unwrap();
    assert_eq!(sample(&selected, 1).0, Duration::from_millis(61));
    assert_eq!(sample(&selected, 1).2, sample(&first_b, 1).2);
    clear_air_speed_interest(&mut selected);
    selected.step_exact(19).unwrap();
    assert_eq!(sample(&selected, 1).0, Duration::from_millis(80));
    reset_loaded_world(&mut selected, |_| ()).unwrap();
    selected.step_exact(1).unwrap();
    let fresh = load_first(&xml, 51);
    for index in 0..2 {
        assert_eq!(sample(&selected, index).2, sample(&fresh, index).2);
        assert_eq!(sample(&selected, index).0, Duration::from_millis(1));
        assert_eq!(sample(&selected, index).1, 1);
    }
    // Bare core reset restarts streams/deadlines without reconstructing entities.
    let handle = sensor(&selected, 0);
    selected.reset().unwrap();
    assert_eq!(sensor(&selected, 0), handle);
    assert!(selected.world().get::<AirSpeedSample>(handle).is_none());
    selected.step_exact(1).unwrap();
    for index in 0..2 {
        assert_eq!(sample(&selected, index).2, sample(&fresh, index).2);
        assert_eq!(sample(&selected, index).1, 2);
    }
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
    let mut eight = load(&wind(&world(&sensors), WIND), 51);
    let one = load(&world(&sensor_xml(0, 100, [0.0; 3], "")), 51);
    let absent = load(&world(""), 51);
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
        let ideal = expected(
            &eight,
            sensor(&eight, index),
            [0.0, 0.0, index as f64 * 0.1],
            WIND,
            None,
        );
        assert!((value.2.diff_pressure - ideal.diff_pressure).abs() < 1e-6);
        assert_eq!(value.2.temperature, ideal.temperature);
    }
    let ninth = world(&format!("{sensors}{}", sensor_xml(8, 100, [0.0; 3], "")));
    assert!(matches!(
        try_load(&ninth, 51),
        Err(LoadError::Unsupported {
            feature: "multiple air_speeds",
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
            51
        ),
        Err(LoadError::Unsupported {
            feature: "air_speed metadata",
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
    let single = wind(&world(&sensor_xml(0, 100, OFFSETS[0], "")), WIND);
    let begin = single.find("  <model ").unwrap();
    let end = single.find("</model>").unwrap() + "</model>".len();
    let second = single[begin..end]
        .replace("name='rig'", "name='peer'")
        .replace("<pose>0 0 3 0 0 0</pose>", "<pose>5 0 7 0 0 0</pose>")
        .replace("/m51/air_speed0", "/m51/peer");
    let xml = format!("{}{}{}", &single[..end], second, &single[end..]);
    let mut sim = load(&xml, 51);
    let first_id = named(&sim, "default::rig::body::air_speed0").1;
    let peer_id = named(&sim, "default::peer::body::air_speed0").1;
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
        sample_at(&sim, peer).2.diff_pressure,
        before.2.diff_pressure
    );
    remove_model(&mut sim, peer_model).unwrap();
    assert!(entity_by_id(sim.world(), peer_id).is_none());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    set_air_speed_interest(&mut sim, []);
    sim.step_exact(2).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.scheduled_system_counts(), counts);
    for id in [first_id, peer_id] {
        let e = entity_by_id(sim.world(), id).unwrap();
        assert!(sim.world().get::<AirSpeedSample>(e).is_none());
    }
    sim.step_exact(1).unwrap();
    geometry(&sim, 0, None);
    let peer = entity_by_id(sim.world(), peer_id).unwrap();
    let peer_pose = sim.world().get::<WorldPose>(peer).unwrap();
    assert_eq!(peer_pose.translation[2], 7.5);
    let relative = DQuat::from_array(peer_pose.rotation).inverse() * -DVec3::from_array(WIND);
    let ideal = air_speed(AirSpeedInput {
        world_z: peer_pose.translation[2],
        velocity_sensor: relative.to_array(),
    })
    .unwrap();
    assert!((sample_at(&sim, peer).2.diff_pressure - ideal.diff_pressure).abs() < 1e-6);
    assert_eq!(sample_at(&sim, peer).2.temperature, ideal.temperature);
}

#[test]
fn first_sensor_default_and_seeded_paths_remain_exact_when_a_peer_is_added() {
    for world_wind in [[0.0; 3], WIND] {
        for noise in ["", "<noise type='gaussian'><stddev>0.2</stddev></noise>"] {
            let mut single = load(
                &wind(&world(&sensor_xml(0, 100, OFFSETS[0], noise)), world_wind),
                51,
            );
            let mut multi = load(
                &wind(
                    &world(&format!(
                        "{}{}",
                        sensor_xml(0, 100, OFFSETS[0], noise),
                        sensor_xml(1, 50, OFFSETS[1], "")
                    )),
                    world_wind,
                ),
                51,
            );
            assert_eq!(
                named(&single, "default::rig::body::air_speed0").1,
                named(&multi, "default::rig::body::air_speed0").1
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
}

#[test]
fn terminal_reset_refusal_preserves_all_sensor_samples_metadata_and_poses() {
    let mut sim = load(&pair("", ""), 51);
    drive(&mut sim, "default::rig::body");
    sim.step_exact(20).unwrap();
    let before = [sample(&sim, 0), sample(&sim, 1)];
    let metadata = [0, 1].map(|i| {
        sim.world()
            .get::<AirSpeedPublication>(sensor(&sim, i))
            .unwrap()
            .clone()
    });
    let body = pose(&sim, "default::rig::body");
    let clock = (sim.elapsed(), sim.epoch(), sim.is_paused());
    sim.world()
        .resource::<StepFault>()
        .report(std::io::Error::other("M51 terminal reset refusal"));
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
            sim.world().get::<AirSpeedPublication>(sensor(&sim, index)),
            Some(metadata)
        );
    }
}

#[test]
fn zero_two_and_eight_consumers_keep_closed_wind_admission_and_signed_flow() {
    let absent_counts = load(&world(""), 51).scheduled_system_counts();
    for zero in [[0.0; 3], [-0.0, 0.0, -0.0]] {
        let absent = load(&wind(&world(""), zero), 51);
        assert_eq!(absent.scheduled_system_counts(), absent_counts);
        assert!(
            !absent
                .world()
                .iter_entities()
                .any(|entity| entity.contains::<AirSpeedPublication>())
        );
    }
    assert!(matches!(
        try_load(&wind(&world(""), WIND), 51),
        Err(LoadError::Unsupported {
            feature: "world wind",
            ..
        })
    ));
    let base = world(&format!(
        "{}{}",
        sensor_xml(0, 100, OFFSETS[0], ""),
        sensor_xml(1, 50, OFFSETS[1], "")
    ));
    let zero = load_first(&wind(&base, [0.0; 3]), 51);
    for vector in [
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, -1.0, 0.0],
        [0.0; 3],
        [-0.0, 0.0, -0.0],
    ] {
        let mut sim = load(&wind(&base, vector), 51);
        let original_pose = pose(&sim, "default::rig::body");
        sim.step_exact(1).unwrap();
        assert_eq!(
            pose(&sim, "default::rig::body"),
            original_pose,
            "sensor wind does not apply physical body forces"
        );
        for (index, offset) in OFFSETS.into_iter().enumerate() {
            let actual = sample(&sim, index);
            assert_eq!(actual.0, Duration::from_millis(1));
            let ideal = expected(&sim, sensor(&sim, index), offset, vector, None);
            assert!((actual.2.diff_pressure - ideal.diff_pressure).abs() < 1e-6);
            assert_eq!(actual.2.temperature, ideal.temperature);
            if vector == [0.0; 3] {
                assert_eq!(
                    [
                        actual.2.diff_pressure.to_bits(),
                        actual.2.temperature.to_bits()
                    ],
                    [
                        sample(&zero, index).2.diff_pressure.to_bits(),
                        sample(&zero, index).2.temperature.to_bits()
                    ]
                );
            }
        }
        if vector == [1.0, 0.0, 0.0] {
            assert!(sample(&sim, 0).2.diff_pressure < 0.0);
        }
        if vector == [-1.0, 0.0, 0.0] {
            assert!(sample(&sim, 0).2.diff_pressure > 0.0);
        }
        if vector == [0.0, 0.0, 1.0] {
            assert_eq!(sample(&sim, 0).2.diff_pressure, 0.0);
        }
        if vector == [1.0, -1.0, 0.0] {
            assert!(sample(&sim, 0).2.diff_pressure.abs() < 1e-6);
        }
    }
    // The separate eight-owner control above uses nonzero WORLD wind as well;
    // it would fail if preflight still returned count==1 after lifting the cap.
}
