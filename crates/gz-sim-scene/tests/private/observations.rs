// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::{reserve_id, restore_id};

fn assert_vector(actual: [f64; 3], expected: [f64; 3]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert!((actual - expected).abs() < 1e-9, "{actual} != {expected}");
    }
}

#[test]
fn observers_consume_committed_link_state_and_resolve_restored_public_ids() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    sim.world_mut().insert_resource(Gravity([0.0, 0.0, -9.81]));
    let pose = WorldPose {
        translation: [1.0, 2.0, 3.0],
        rotation: DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2).to_array(),
    };
    let velocity = Velocity {
        linear: [3.0, -4.0, 5.0],
        angular: [0.0, 0.0, 2.0],
    };
    let link = sim.world_mut().spawn((pose, velocity)).id();
    let link_id = assign_id(sim.world_mut(), link).unwrap();
    let reserved = reserve_id(sim.world(), link).unwrap();
    let model = sim
        .world_mut()
        .spawn((
            ModelAnchor {
                link: link_id,
                link_to_model: WorldPose {
                    translation: [1.0, 0.0, 0.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                },
            },
            pose,
        ))
        .id();
    let sensor_rotation = DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
    let sensor = sim
        .world_mut()
        .spawn(Imu {
            link: link_id,
            rotation: sensor_rotation,
            reference: DQuat::from_array(pose.rotation) * sensor_rotation,
            previous_velocity: DVec3::ZERO,
            schedule: SampleSchedule::new(0.0).unwrap(),
            sdf_rate_hz: 0.0,
            rate_hz: 0.0,
            active: true,
            noise: None,
        })
        .id();
    sim.add_systems(Phase::PhysicsCommit, sync_models);
    sim.add_systems(Phase::PostStep, sample_imus);
    sim.add_systems(Phase::Reset, (sync_models, reset_imus).chain());
    assert!(!sim.world().contains_resource::<RapierBackend>());
    sim.set_paused(false);
    sim.tick().unwrap();
    assert_vector(
        sim.world().get::<WorldPose>(model).unwrap().translation,
        [1.0, 3.0, 3.0],
    );
    let sample = sim.world().get::<ImuSample>(sensor).unwrap();
    let reading = sample.reading.as_ref().unwrap();
    assert_vector(reading.specific_force, [-4000.0, 5009.81, 3000.0]);
    assert_vector(reading.angular_velocity, [0.0, 2.0, 0.0]);
    for (actual, expected) in reading.orientation.into_iter().zip([0.0, 0.0, 0.0, 1.0]) {
        assert!((actual - expected).abs() < 1e-12);
    }
    sim.tick().unwrap();
    assert_vector(
        sim.world()
            .get::<ImuSample>(sensor)
            .unwrap()
            .reading
            .as_ref()
            .unwrap()
            .specific_force,
        [0.0, 9.81, 0.0],
    );
    sim.world_mut().despawn(link);
    let replacement = sim
        .world_mut()
        .spawn((
            WorldPose {
                translation: [4.0, 5.0, 6.0],
                rotation: DQuat::from_rotation_z(std::f64::consts::PI).to_array(),
            },
            Velocity {
                linear: [0.0; 3],
                angular: [0.0, 0.0, 4.0],
            },
        ))
        .id();
    restore_id(sim.world_mut(), replacement, &reserved).unwrap();
    assert_ne!(replacement, link);
    sim.tick().unwrap();
    assert_vector(
        sim.world().get::<WorldPose>(model).unwrap().translation,
        [3.0, 5.0, 6.0],
    );
    let sample = sim.world().get::<ImuSample>(sensor).unwrap();
    let reading = sample.reading.as_ref().unwrap();
    assert_vector(reading.specific_force, [3000.0, -4990.19, 4000.0]);
    assert_vector(reading.angular_velocity, [0.0, 4.0, 0.0]);
    for (actual, expected) in
        reading
            .orientation
            .into_iter()
            .zip([0.0, 0.5f64.sqrt(), 0.0, 0.5f64.sqrt()])
    {
        assert!((actual - expected).abs() < 1e-12);
    }
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(
        sim.world().get::<ImuSample>(sensor).unwrap().time,
        Duration::from_millis(3)
    );
    sim.reset().unwrap();
    assert!(sim.world().get::<ImuSample>(sensor).is_none());
    sim.set_paused(false);
    sim.tick().unwrap();
    let sample = sim.world().get::<ImuSample>(sensor).unwrap();
    assert_eq!((sample.epoch, sample.time), (1, Duration::from_millis(1)));
    assert_vector(
        sample.reading.as_ref().unwrap().specific_force,
        [0.0, 9.81, 0.0],
    );
}
