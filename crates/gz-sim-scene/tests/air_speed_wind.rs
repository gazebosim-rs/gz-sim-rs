// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_sensors::air_speed::{
    AirSpeedInput, AirSpeedReading, air_speed, air_speed_unscaled, scale_air_speed,
};
use gz_sim_core::{EntityId, Simulation, StepFault, entity_by_id};
use gz_sim_scene::{
    AirSpeedSample, AirSpeedSampleError, Name, PhysicsEngine, Velocity, WorldPose,
    load_world_with_sdf_step, queue_link_world_velocity, remove_model, reset_loaded_world,
    set_air_speed_interest,
};
use std::time::Duration;

const XML: &str = include_str!("../../../examples/m41_air_speed_wind.sdf");
const OLD: &str = include_str!("fixtures/m26_air_speed.sdf");
fn wind(xml: &str, value: &str) -> String {
    xml.replace(
        "<gravity>0 0 0</gravity>",
        &format!("<gravity>0 0 0</gravity><wind><linear_velocity>{value}</linear_velocity></wind>"),
    )
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m41.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn named(sim: &Simulation, name: &str) -> (bevy_ecs::entity::Entity, u64) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (e.id(), e.get::<EntityId>().unwrap().get())
}
fn sensor(sim: &Simulation) -> bevy_ecs::entity::Entity {
    named(sim, "default::rig::body::air_speed").0
}
fn reading(sim: &Simulation) -> (Duration, u64, AirSpeedReading) {
    let s = sim.world().get::<AirSpeedSample>(sensor(sim)).unwrap();
    (s.time, s.epoch, *s.reading.as_ref().unwrap())
}
fn bits(value: AirSpeedReading) -> [u64; 2] {
    [value.diff_pressure.to_bits(), value.temperature.to_bits()]
}
fn drive(sim: &mut Simulation) {
    let body = named(sim, "default::rig::body").1;
    queue_link_world_velocity(sim, body, [3.0, -1.0, 0.5], [0.4, -0.2, 2.0]).unwrap();
}
// Independent point geometry uses the established 1e-6 pressure bound. Native
// scalar/wire bit equality requires the separately captured component fixture.
fn geometry(sim: &Simulation, wind: [f64; 3]) {
    let sensor_pose = sim.world().get::<WorldPose>(sensor(sim)).unwrap();
    let link = named(sim, "default::rig::body").0;
    let link_pose = sim.world().get::<WorldPose>(link).unwrap();
    let velocity = sim.world().get::<Velocity>(link).unwrap();
    let offset = DQuat::from_array(link_pose.rotation) * DVec3::new(0.0, 0.5, 0.5);
    let point =
        DVec3::from_array(velocity.linear) + DVec3::from_array(velocity.angular).cross(offset);
    let frame = DQuat::from_array(sensor_pose.rotation).inverse();
    let relative = frame * point - frame * DVec3::from_array(wind);
    let expected = air_speed(AirSpeedInput {
        world_z: sensor_pose.translation[2],
        velocity_sensor: relative.to_array(),
    })
    .unwrap();
    let actual = reading(sim).2;
    assert!((actual.diff_pressure - expected.diff_pressure).abs() < 1e-6);
    assert_eq!(actual.temperature, expected.temperature);
}

#[test]
fn formerly_refused_world_x_wind_loads_moves_and_healthy_reset_restores_public_identity() {
    // Exact old M26 + WORLD(1,0,0) negative input, now a positive workflow.
    let mut sim = load(&wind(OLD, "1 0 0"));
    let id = named(&sim, "default::rig::body::air_speed").1;
    sim.step_exact(1).unwrap();
    let first = reading(&sim).2;
    geometry(&sim, [1.0, 0.0, 0.0]);
    assert!(first.diff_pressure < 0.0);
    drive(&mut sim);
    sim.step_exact(19).unwrap();
    geometry(&sim, [1.0, 0.0, 0.0]);
    assert_ne!(bits(reading(&sim).2), bits(first));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let fresh = entity_by_id(sim.world(), id).unwrap();
    assert_eq!(fresh, sensor(&sim));
    assert!(sim.world().get::<AirSpeedSample>(fresh).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(bits(reading(&sim).2), bits(first));
    assert_eq!(
        (reading(&sim).0, reading(&sim).1),
        (Duration::from_millis(1), 1)
    );
}

#[test]
fn full_world_wind_at_rotating_offset_sensor_changes_airflow_without_changing_physics() {
    let mut windy = load(XML);
    let calm_xml = XML.replace(
        "<wind><linear_velocity>1 -2 0.5</linear_velocity></wind>",
        "",
    );
    let mut calm = load(&calm_xml);
    drive(&mut windy);
    drive(&mut calm);
    let mut held = None;
    let mut due = Vec::new();
    for tick in 1..=30 {
        windy.step_exact(1).unwrap();
        calm.step_exact(1).unwrap();
        let a = named(&windy, "default::rig::body").0;
        let b = named(&calm, "default::rig::body").0;
        let pa = windy.world().get::<WorldPose>(a).unwrap();
        let pb = calm.world().get::<WorldPose>(b).unwrap();
        assert_eq!(
            pa.translation.map(f64::to_bits),
            pb.translation.map(f64::to_bits)
        );
        assert_eq!(pa.rotation.map(f64::to_bits), pb.rotation.map(f64::to_bits));
        assert_eq!(
            windy.world().get::<Velocity>(a).unwrap().linear,
            calm.world().get::<Velocity>(b).unwrap().linear
        );
        assert_eq!(
            windy.world().get::<Velocity>(a).unwrap().angular,
            calm.world().get::<Velocity>(b).unwrap().angular
        );
        let current = reading(&windy);
        if held.is_none_or(|(time, _, _)| time != current.0) {
            geometry(&windy, [1.0, -2.0, 0.5]);
            due.push(tick);
        } else {
            assert_eq!(Some(current), held);
        }
        held = Some(current);
    }
    assert_eq!(due, [1, 10, 20, 30]);
    assert_ne!(bits(reading(&windy).2), bits(reading(&calm).2));
}

#[test]
fn head_tail_cross_and_cancelling_wind_keep_signed_sensor_x_pressure() {
    let base = OLD.replace(
        "<pose>0 0.5 0.5 0 0 0.7853981633974483</pose>",
        "<pose>0 0.5 0.5 0 0 0</pose>",
    );
    for (world_wind, expected_sign) in [
        ("4 0 0", -1.0),
        ("-1 0 0", 1.0),
        ("3 0 0", 0.0),
        ("0 4 5", 1.0),
    ] {
        let mut sim = load(&wind(&base, world_wind));
        let body = named(&sim, "default::rig::body").1;
        queue_link_world_velocity(&mut sim, body, [3.0, 0.0, 0.0], [0.0; 3]).unwrap();
        sim.step_exact(1).unwrap();
        let pressure = reading(&sim).2.diff_pressure;
        if expected_sign == 0.0 {
            assert_eq!(pressure, 0.0);
        } else {
            assert_eq!(pressure.signum(), expected_sign);
        }
    }
}

#[test]
fn tilted_world_z_and_mixed_zero_wind_use_composite_sensor_frame() {
    let tilted = OLD.replace("<pose>0 0 3 0 0 0</pose>", "<pose>0 0 3 0.2 0.3 0.6</pose>");
    for (text, vector) in [("0 0 2", [0.0, 0.0, 2.0]), ("-0 2 -0", [-0.0, 2.0, -0.0])] {
        let mut sim = load(&wind(&tilted, text));
        drive(&mut sim);
        sim.step_exact(1).unwrap();
        geometry(&sim, vector);
    }
}

#[test]
fn whole_signed_zero_matches_legacy_and_zero_stddev_noise_keeps_float_order() {
    for noise in [
        "<air_speed/>",
        "<air_speed><pressure><noise type='gaussian'><mean>0.1</mean><stddev>0</stddev></noise></pressure></air_speed>",
    ] {
        let base = OLD.replace("<air_speed/>", noise);
        let mut omitted = load(&base);
        let mut explicit = load(&wind(&base, "-0 0 -0"));
        drive(&mut omitted);
        drive(&mut explicit);
        for _ in 0..20 {
            omitted.step_exact(1).unwrap();
            explicit.step_exact(1).unwrap();
            assert_eq!(reading(&omitted), reading(&explicit));
        }
    }
    let config = XML.replace("<air_speed/>", "<air_speed><pressure><noise type='gaussian'><mean>0.1</mean><stddev>0</stddev></noise></pressure></air_speed>");
    let mut sim = load(&config);
    sim.step_exact(1).unwrap();
    let pose = sim.world().get::<WorldPose>(sensor(&sim)).unwrap();
    let frame = DQuat::from_array(pose.rotation).inverse();
    let relative = -(frame * DVec3::new(1.0, -2.0, 0.5));
    let (p, t) = air_speed_unscaled(AirSpeedInput {
        world_z: pose.translation[2],
        velocity_sensor: relative.to_array(),
    })
    .unwrap();
    let expected = scale_air_speed((f64::from(p) + 0.1) as f32, t).unwrap();
    assert_eq!(bits(reading(&sim).2), bits(expected));
}

#[test]
fn missing_demand_and_pause_hold_deadlines_and_loaded_removal_reconstructs_wind() {
    let mut sim = load(XML);
    let id = named(&sim, "default::rig::body::air_speed").1;
    let owner = named(&sim, "default::rig").1;
    set_air_speed_interest(&mut sim, []);
    sim.step_exact(20).unwrap();
    assert!(sim.world().get::<AirSpeedSample>(sensor(&sim)).is_none());
    let e = sensor(&sim);
    set_air_speed_interest(&mut sim, [e]);
    sim.step_exact(1).unwrap();
    assert_eq!(reading(&sim).0, Duration::from_millis(21));
    let held = reading(&sim);
    sim.set_paused(true);
    for _ in 0..3 {
        sim.tick().unwrap();
    }
    assert_eq!(reading(&sim), held);
    remove_model(&mut sim, owner).unwrap();
    assert!(entity_by_id(sim.world(), id).is_none());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(entity_by_id(sim.world(), id), Some(sensor(&sim)));
    assert!(sim.world().get::<AirSpeedSample>(sensor(&sim)).is_none());
    sim.set_paused(false);
    sim.step_exact(1).unwrap();
    geometry(&sim, [1.0, -2.0, 0.5]);
    assert_eq!(reading(&sim).1, 1);
}

#[test]
fn finite_overflow_is_a_sample_error_without_a_terminal_clock_fault() {
    let mut sim = load(&wind(OLD, "1e308 0 0"));
    sim.step_exact(1).unwrap();
    let sample = sim.world().get::<AirSpeedSample>(sensor(&sim)).unwrap();
    assert!(matches!(sample.reading, Err(AirSpeedSampleError::Ideal(_))));
    assert!(sim.world().resource::<StepFault>().error().is_none());
    sim.step_exact(9).unwrap();
    assert_eq!(
        sim.world()
            .get::<AirSpeedSample>(sensor(&sim))
            .unwrap()
            .time,
        Duration::from_millis(10)
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        sim.world()
            .get::<AirSpeedSample>(sensor(&sim))
            .unwrap()
            .reading
            .is_err()
    );
}

#[test]
fn nonfinite_and_unconsumed_wind_and_physical_wind_modes_remain_refused() {
    let no_sensor = "<sdf version='1.12'><world name='default'><wind><linear_velocity>1 0 0</linear_velocity></wind></world></sdf>";
    for xml in [
        wind(OLD, "NaN 0 0"),
        wind(OLD, "inf 0 0"),
        wind(OLD, "0 0"),
        no_sensor.to_owned(),
        XML.replace(
            "<model name='rig'>",
            "<model name='rig'><enable_wind>true</enable_wind>",
        ),
        XML.replace(
            "<link name='body'>",
            "<link name='body'><enable_wind>true</enable_wind>",
        ),
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "bad-wind",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::Rapier
            )
            .is_err()
        );
    }
}
