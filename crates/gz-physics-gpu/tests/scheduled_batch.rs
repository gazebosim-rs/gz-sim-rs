// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{
    GpuBatch, Lane,
    simulation::{BatchTiming, batch_simulation},
};
use std::time::Duration;

/// Run with GZ_GPU_ADAPTER set to the hardware under test and --ignored.
#[test]
#[ignore = "requires an explicitly selected Vulkan device with shader f64"]
fn paused_steps_partial_workgroups_reset_and_clock_fault() {
    let name = std::env::var("GZ_GPU_ADAPTER").expect("set GZ_GPU_ADAPTER");
    let initial: Vec<_> = (0..65)
        .map(|i| Lane {
            anchor_z: 0.1,
            anchor_v: 0.1,
            bob_z: 0.4,
            bob_v: 0.,
            rigid_z: 0.8 + 0.002 * f64::from(i),
            rigid_v: -0.05,
            rest_length: 0.3,
            compliance: 0.012,
            bob_inverse_mass: 4.,
            rigid_inverse_mass: 2.,
            combined_radius: 0.11,
            last_receiver_impulse_z: 0.,
        })
        .collect();
    let mut sim = batch_simulation(&name, initial.clone()).unwrap();
    sim.tick().unwrap();
    assert!(sim.world().resource::<BatchTiming>().last.is_none());
    assert_eq!(sim.world().resource::<GpuBatch>().tick(), 0);
    sim.step_exact(1).unwrap();
    let observed = sim.world().resource::<GpuBatch>();
    assert_eq!(sim.elapsed(), Duration::from_millis(1));
    for (i, lane) in observed.lanes().iter().enumerate() {
        let z = initial[i].rigid_z + 0.001 * (-0.05 - 0.625 * 0.001 * 9.81);
        assert!((lane.rigid_z - z).abs() < 2e-14);
        assert_eq!(lane.last_receiver_impulse_z, 0.);
    }
    assert!(sim.world().resource::<BatchTiming>().last.is_some());
    sim.reset().unwrap();
    assert_eq!(
        sim.world().resource::<GpuBatch>().lanes()[64].rigid_z,
        initial[64].rigid_z
    );
    assert!(sim.world().resource::<BatchTiming>().last.is_none());
    // A caller that advanced the public batch manually must not get a second
    // physical tick committed against an earlier Gazebo timestamp.
    sim.world_mut().resource_mut::<GpuBatch>().step().unwrap();
    assert!(sim.step_exact(1).is_err());
    assert!(sim.fault().unwrap().to_string().contains("clocks disagree"));
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.world().resource::<GpuBatch>().tick(), 1);
}

/// Failure of one lane must block downstream Gazebo observers for the entire batch.
#[test]
#[ignore = "requires an explicitly selected Vulkan device with shader f64"]
fn numerical_fault_preserves_epoch_observation_and_blocks_publication() {
    use bevy_ecs::prelude::{ResMut, Resource};
    use gz_sim_core::Phase;
    #[derive(Default, Resource)]
    struct Publications(u64);

    let initial = Lane {
        anchor_z: 0.,
        anchor_v: 0.,
        bob_z: 1.,
        bob_v: 0.,
        rigid_z: 2.,
        rigid_v: 0.,
        rest_length: 1.,
        compliance: 0.,
        bob_inverse_mass: 1.,
        rigid_inverse_mass: 1.,
        combined_radius: 0.11,
        last_receiver_impulse_z: 0.,
    };
    // A finite rest length may still need an unrepresentable correction velocity.
    let lanes = vec![
        initial,
        Lane {
            rest_length: 1.0e308,
            ..initial
        },
    ];
    let mut sim =
        batch_simulation(&std::env::var("GZ_GPU_ADAPTER").unwrap(), lanes.clone()).unwrap();
    sim.world_mut().init_resource::<Publications>();
    sim.add_systems(Phase::Publish, |mut count: ResMut<Publications>| {
        count.0 += 1
    });
    assert!(sim.step_exact(1).is_err());
    assert!(matches!(
        sim.fault()
            .unwrap()
            .downcast_ref::<gz_physics_gpu::GpuError>(),
        Some(gz_physics_gpu::GpuError::Nonfinite(1))
    ));
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.world().resource::<GpuBatch>().lanes(), lanes);
    assert!(sim.world().resource::<BatchTiming>().last.is_none());
    assert_eq!(sim.world().resource::<Publications>().0, 0);
}
