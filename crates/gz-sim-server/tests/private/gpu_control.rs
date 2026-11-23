// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[test]
fn collisions_with_sensor_control_root_clock_and_remapped_topics_are_rejected() {
    let mut s = Simulation::new(std::time::Duration::from_millis(1)).unwrap();
    s.world_mut().spawn(gz_sim_scene::ImuPublication {
        enable_orientation: true,
        entity_name: "model::link::imu".into(),
        frame_id: "imu".into(),
        topic: "/imu".into(),
    });
    let mut options = crate::ServerOptions {
        partition: "gpu-scene-test".into(),
        ..Default::default()
    };
    for source in ["/clock", "/imu", "/imu/set_rate"] {
        let key = options.topic_key(source).unwrap();
        assert!(validate_key(&s, &options, &key).is_err());
    }
    options.add_topic_remap("/imu", "/alias").unwrap();
    assert!(validate_key(&s, &options, &options.topic_key("/alias").unwrap()).is_err());
    validate_key(&s, &options, &options.topic_key("/distinct").unwrap()).unwrap();
}

#[test]
fn placement_refuses_malformed_unstamped_unbounded_or_unowned_commands() {
    use crate::gpu_control::placement::{self, EnvironmentPlacement, Target};
    use gz_sim_proto::prost::Message;
    let mut sim = Simulation::new(std::time::Duration::from_millis(1)).unwrap();
    let valid = EnvironmentPlacement {
        epoch: Some(0),
        tick: Some(0),
        environments: vec![8],
        target: Target::Cpu.into(),
    };
    let rejected = |ack: placement::EnvironmentPlacementAck, reason: &str| {
        assert!(!ack.accepted && ack.reason.contains(reason), "{ack:?}");
        assert_eq!(
            (ack.cpu, ack.gpu, ack.moved, ack.reseeded_gpu_rows),
            (0, 0, 0, 0)
        );
    };
    rejected(placement::command(&mut sim, &[0xff], false), "malformed");
    rejected(
        placement::command(&mut sim, &valid.encode_to_vec(), true),
        "reset or fault",
    );
    for (request, reason) in [
        (
            EnvironmentPlacement {
                epoch: None,
                ..valid.clone()
            },
            "missing epoch",
        ),
        (
            EnvironmentPlacement {
                tick: None,
                ..valid.clone()
            },
            "missing tick",
        ),
        (
            EnvironmentPlacement {
                environments: vec![],
                ..valid.clone()
            },
            "between 1 and 256",
        ),
        (
            EnvironmentPlacement {
                environments: vec![1; 257],
                ..valid.clone()
            },
            "between 1 and 256",
        ),
        (
            EnvironmentPlacement {
                target: 0,
                ..valid.clone()
            },
            "CPU or GPU",
        ),
        (
            EnvironmentPlacement {
                target: 42,
                ..valid.clone()
            },
            "CPU or GPU",
        ),
    ] {
        rejected(
            placement::command(&mut sim, &request.encode_to_vec(), false),
            reason,
        );
    }
    let ack = placement::command(&mut sim, &valid.encode_to_vec(), false);
    assert!(!ack.accepted && !ack.reason.is_empty());
}

#[test]
#[ignore = "requires selected Vulkan f64 hardware"]
fn placement_ack_reports_cohort_and_reseed_counts() {
    use crate::gpu_control::placement::{self, EnvironmentPlacement, Target};
    use gz_sim_proto::prost::Message;
    use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_on_adapter};
    let xml = include_str!("../../../../examples/gpu_mixed_world.sdf");
    let mut sim = load_world_on_adapter(
        xml,
        "placement.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(&std::env::var("GZ_GPU_ADAPTER").unwrap()),
    )
    .unwrap();
    let request = EnvironmentPlacement {
        epoch: Some(0),
        tick: Some(0),
        environments: vec![9, 31, 80],
        target: Target::Cpu as i32,
    };
    let ack = placement::command(&mut sim, &request.encode_to_vec(), false);
    assert!(ack.accepted);
    assert_eq!(
        (ack.cpu, ack.gpu, ack.moved, ack.reseeded_gpu_rows),
        (3, 0, 3, 0)
    );
    let ack = placement::command(
        &mut sim,
        &EnvironmentPlacement {
            target: Target::Gpu as i32,
            ..request
        }
        .encode_to_vec(),
        false,
    );
    assert!(ack.accepted);
    assert_eq!(
        (ack.cpu, ack.gpu, ack.moved, ack.reseeded_gpu_rows),
        (0, 3, 3, 3)
    );
}
