// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{
    ImuPublication, ImuSample, PhysicsEngine, load_world_with_engine, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

#[test]
fn original_disabled_orientation_xml_loads_and_survives_reset_without_losing_motion() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for enabled in [false, true] {
            let xml = include_str!("fixtures/fall.sdf").replace(
                "<always_on>true</always_on>",
                if enabled {
                    "<imu><enable_orientation>true</enable_orientation></imu>"
                } else {
                    "<imu><enable_orientation>false</enable_orientation></imu>"
                },
            );
            let mut sim = load_world_with_engine(
                &xml,
                "orientation.sdf",
                "default",
                Duration::from_millis(1),
                &NoResources,
                engine,
            )
            .unwrap();
            for epoch in 0..2 {
                sim.step_exact(1).unwrap();
                let (metadata, sample) = sim
                    .world_mut()
                    .query::<(&ImuPublication, &ImuSample)>()
                    .single(sim.world())
                    .unwrap();
                assert_eq!(metadata.enable_orientation, enabled);
                assert_eq!(sample.time, Duration::from_millis(1));
                assert_eq!(sample.epoch, epoch);
                let reading = sample.reading.as_ref().unwrap();
                assert_eq!(reading.angular_velocity, [0.0; 3]);
                assert!(reading.specific_force[..2].iter().all(|v| v.abs() < 1e-12));
                assert!(
                    (reading.specific_force[2]
                        - if engine == PhysicsEngine::Tpe {
                            9.81
                        } else {
                            0.0
                        })
                    .abs()
                        < 1e-10
                );
                assert_eq!(reading.angular_velocity_covariance, [0.0; 3]);
                assert_eq!(reading.linear_acceleration_covariance, [0.0; 3]);
                reset_loaded_world(&mut sim, |_| ()).unwrap();
            }
        }
    }
}
