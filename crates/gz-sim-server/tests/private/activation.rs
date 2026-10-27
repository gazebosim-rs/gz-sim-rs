// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_msgs::msgs::WorldReset;
use gz_sim_core::EntityId;
use gz_sim_scene::{PhysicsEngine, imu_active, load_world_with_engine};

#[test]
fn local_activation_keeps_cached_packets_quiet_and_restores_on_both_resets() {
    for engine in [PhysicsEngine::Tpe, PhysicsEngine::Rapier] {
        let simulation = load_world_with_engine(
            include_str!("../../../gz-sim-scene/tests/fixtures/fall.sdf"),
            "activation.sdf",
            "default",
            Duration::from_millis(1),
            &sdformat::NoResources,
            engine,
        )
        .unwrap();
        let mut server = Server::new(
            simulation,
            "default",
            config(),
            NonZeroUsize::new(2).unwrap(),
        )
        .unwrap();
        server.set_interest_policy(InterestPolicy::Always);
        server.set_pose_publication_period(Duration::ZERO);
        server.simulation.set_paused(false);
        server.tick().unwrap();
        let id = server
            .simulation
            .world()
            .get::<EntityId>(server.imus[0].entity)
            .unwrap()
            .get();
        assert_eq!(server.counters().imus_queued, 1);
        assert_eq!(server.imus[0].sequence, 1);
        assert!(matches!(
            server.set_imu_active(0, false),
            Err(Error::ImuActivation(_))
        ));
        assert!(imu_active(server.simulation(), id).unwrap());
        server.set_imu_active(id, false).unwrap();
        for _ in 0..3 {
            server.tick().unwrap();
        }
        assert_eq!(server.counters().imus_queued, 1);
        assert_eq!(server.imus[0].sequence, 1);
        server.set_imu_active(id, true).unwrap();
        server.tick().unwrap();
        assert_eq!(server.counters().imus_queued, 2);
        assert_eq!(server.imus[0].sequence, 2);
        for all in [true, false] {
            server.set_imu_active(id, false).unwrap();
            server.simulation.set_paused(true);
            let count = server.counters().imus_queued;
            server.tick().unwrap();
            assert_eq!(server.counters().imus_queued, count);
            assert!(!imu_active(server.simulation(), id).unwrap());
            assert!(server.apply_control(WorldControl {
                pause: true,
                reset: Some(WorldReset {
                    all,
                    time_only: !all,
                    ..Default::default()
                }),
                ..Default::default()
            }));
            server.tick().unwrap();
            assert!(imu_active(server.simulation(), id).unwrap());
            assert!(server.imus[0].last_sample.is_none());
            assert_eq!(server.imus[0].sequence, 0);
            server.simulation.set_paused(false);
            server.tick().unwrap();
            assert_eq!(server.counters().imus_queued, count + 1);
            assert_eq!(server.imus[0].sequence, 1);
        }
        server.shutdown();
    }
}
