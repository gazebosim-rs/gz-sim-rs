// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "gpu-batch")]
use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    AltimeterSample, ImuSample, Name, PhysicsEngine, WorldLoadOptions,
    gpu_scene::{batch_boundary, queue_link_impulse},
    load_world_on_adapter, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;
const SCENE: &str = include_str!("../../../examples/gpu_sensor_world.sdf");
fn entity(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 3e-10, "{a} vs {b}");
}
#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn sensor_samples_match_their_reference_frames_and_repeat_after_reset() {
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut sim = load_world_on_adapter(
        SCENE,
        "sensor.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            step: None,
            noise_seed: [0; 32],
            engine: PhysicsEngine::GpuBatch,
            distributed_worker: None,
        },
        Some(&adapter),
    )
    .unwrap();
    let mut reference = vec![];
    for epoch in 0..=1 {
        let rolling = entity(&sim, "default::rolling::body::imu");
        let supported = entity(&sim, "default::supported_b::action_supported::imu");
        let free_alt = entity(&sim, "default::free_a::action_free::altimeter");
        let supported_alt = entity(&sim, "default::supported_b::action_supported::altimeter");
        let free_id = sim
            .world()
            .get::<EntityId>(entity(&sim, "default::free_a::action_free"))
            .unwrap()
            .get();
        for imu in [rolling, supported] {
            assert!(sim.world().get::<ImuSample>(imu).is_none());
        }
        for alt in [free_alt, supported_alt] {
            assert!(sim.world().get::<AltimeterSample>(alt).is_none());
        }
        let mut readings = vec![];
        for tick in 1..=40u64 {
            if tick == 2 {
                let at = batch_boundary(&sim);
                queue_link_impulse(&mut sim, at, free_id, [0., 0., 0.17]).unwrap();
            }
            sim.step_exact(1).unwrap();
            for (e, period, g, spin) in [(rolling, 10, 0., 2.2), (supported, 5, 1.2, -1.6)] {
                let s = sim.world().get::<ImuSample>(e).unwrap();
                let last = 1.max(tick / period * period);
                assert_eq!(s.time, Duration::from_millis(last));
                assert_eq!(s.epoch, epoch);
                let imu = s.reading.as_ref().unwrap();
                for i in 0..2 {
                    near(imu.specific_force[i], 0.);
                    near(imu.angular_velocity[i], 0.);
                }
                near(imu.specific_force[2], g);
                near(imu.angular_velocity[2], spin);
                if last == tick {
                    readings.extend(imu.orientation);
                    readings.extend(imu.specific_force);
                    readings.extend(imu.angular_velocity);
                }
            }
            for (e, period, base) in [(free_alt, 20, 0.85), (supported_alt, 8, 0.28)] {
                let s = sim.world().get::<AltimeterSample>(e).unwrap();
                let last = 1.max(tick / period * period);
                assert_eq!(s.time, Duration::from_millis(last));
                assert_eq!(s.epoch, epoch);
                let alt = s.reading.as_ref().unwrap();
                near(alt.vertical_reference, base);
                if e == supported_alt {
                    near(alt.vertical_position, 0.);
                    near(alt.vertical_velocity, 0.);
                } else {
                    let t = s.time.as_secs_f64();
                    let omega = DVec3::new(0.4, 1.8, 1.);
                    // The GPU integrates the normalized quaternion Euler increment.
                    // Its closed-form fixed-axis angle is 2 atan(|omega| h / 2).
                    let local = DVec3::new(0.07, 0.02, 0.03);
                    let angle = 2. * (omega.length() * 0.0005).atan() * last as f64;
                    let r = DQuat::from_axis_angle(omega.normalize(), angle) * local;
                    let analytic = DQuat::from_scaled_axis(omega * t) * local;
                    assert!(
                        r.distance(analytic)
                            <= local.length() * omega.length().powi(3) * t * 1e-6 / 12. + 1e-15
                    );
                    let vz = if last >= 2 { 0.2 } else { 0. };
                    near(alt.vertical_velocity, -1.2 * t + vz + omega.cross(r).z);
                    near(
                        alt.vertical_position,
                        -0.6 * t * t + vz * (t - 0.001) + r.z - 0.03,
                    );
                }
                if last == tick {
                    readings.extend([
                        alt.vertical_position,
                        alt.vertical_velocity,
                        alt.vertical_reference,
                    ]);
                }
            }
        }
        if epoch == 0 {
            reference = readings;
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        } else {
            assert_eq!(readings, reference);
        }
    }
}
