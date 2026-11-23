// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
pub(super) fn initial() -> Vec<Environment> {
    let plane = SpherePlaneLane {
        position: [0., 0., 0.22],
        velocity: [1., 0., -0.8],
        angular_velocity: [0.; 3],
        orientation: [0., 0., 0., 1.],
        gravity: [0., 0., -9.81],
        plane_normal: [0., 0., 1.],
        plane_offset: 0.,
        radius: 0.2,
        inverse_mass: 1.,
        friction: 0.2,
        normal_impulse: 0.,
        tangential_impulse: [0.; 3],
    };
    let a = SolidSphere {
        position: [-0.11, 0., 0.1],
        velocity: [1., 0., 0.],
        angular_velocity: [0., 0., 2.],
        orientation: plane.orientation,
        radius: 0.1,
        inverse_mass: 1.,
    };
    let b = SolidSphere {
        position: [0.11, 0., 0.1],
        velocity: [-1., 0., 0.],
        ..a
    };
    let pair = SpherePairLane {
        a,
        b,
        gravity: plane.gravity,
        friction: 0.4,
        restitution: 0.5,
        normal_impulse: 0.,
        tangential_impulse: [0.; 3],
        contact_normal: [0.; 3],
        impact_time: -1.,
    };
    vec![
        Environment {
            id: 70,
            state: EnvironmentState::SupportedSpherePair(SupportedSpherePairLane {
                pair,
                plane_normal: plane.plane_normal,
                plane_offset: 0.,
                support_normal_impulses: [0.; 2],
            }),
        },
        Environment {
            id: 0,
            state: EnvironmentState::SpherePair(pair),
        },
        Environment {
            id: 9,
            state: EnvironmentState::SpherePlane(plane),
        },
    ]
}
#[test]
fn admission_refuses_invalid_registration_without_device() {
    let mut seed = initial();
    assert!(matches!(
        MixedBatch::new("", seed.clone()),
        Err(GpuError::EmptySelector)
    ));
    assert!(matches!(
        MixedBatch::new("GPU", vec![]),
        Err(GpuError::EmptyBatch)
    ));
    seed[0].id = 9;
    assert!(matches!(
        MixedBatch::new("GPU", seed),
        Err(GpuError::DuplicateEnvironment(9))
    ));
    for mut bad in initial() {
        let profile = bad.state.profile();
        bad.state.bytes_mut()[0..8].copy_from_slice(&f64::NAN.to_le_bytes());
        assert!(
            matches!(MixedBatch::new("GPU",vec![bad]),Err(GpuError::InvalidEnvironment{id,profile:p}) if id==bad.id && p==profile)
        );
    }
}
#[test]
fn combined_layout_checks_full_copy_capacity_and_separate_binding_extents() {
    let seeds = initial();
    let lim = wgpu::Limits {
        min_storage_buffer_offset_alignment: 256,
        max_storage_buffer_binding_size: 400,
        max_buffer_size: 2048,
        ..Default::default()
    };
    let plan = |items: &[Environment], limits: &wgpu::Limits| {
        Layout::new(
            items,
            wgpu::Features::SHADER_F64,
            wgpu::DownlevelFlags::COMPUTE_SHADERS,
            limits,
        )
    };
    let l = plan(&seeds, &lim).unwrap();
    assert_eq!(
        l.groups.iter().map(|g| g.offset).collect::<Vec<_>>(),
        [0, 256, 768]
    );
    assert_eq!(l.offsets, [768, 256, 0]);
    assert_eq!(l.plan.state_bytes, 1160);
    assert!(matches!(
        plan(
            &seeds,
            &wgpu::Limits {
                max_buffer_size: 1159,
                ..lim.clone()
            }
        ),
        Err(GpuError::Capacity)
    ));
    assert!(matches!(plan(&[], &lim), Err(GpuError::EmptyBatch)));
    let l = plan(&seeds[1..2], &lim).unwrap();
    assert_eq!(l.offsets, [0]);
    assert_eq!(l.groups.len(), 1);
}
fn selector() -> String {
    std::env::var("GZ_GPU_ADAPTER").expect("select native f64 GPU")
}
#[test]
#[ignore = "requires selected f64 Vulkan hardware"]
fn later_family_failure_keeps_all_public_states_and_retry_reseeds_all_groups() {
    let mut batch = MixedBatch::new(&selector(), initial()).unwrap();
    let seed = batch.environments().to_vec();
    assert!(
        batch.host_state_bytes()
            >= 3 * size_of_val(batch.environments()) + batch.device_info().state_bytes as usize
    );
    assert_eq!(seed.iter().map(|s| s.id).collect::<Vec<_>>(), [0, 9, 70]);
    assert!(batch.get(71).is_none());
    let first = batch.step().unwrap();
    let committed = batch.environments().to_vec();
    for (a, b) in seed.iter().zip(&committed) {
        assert_ne!(a.state, b.state);
    }
    assert_eq!(first.uploaded_bytes, batch.device_info().state_bytes);
    assert_eq!(batch.get(70), Some(&committed[2].state));
    let group = batch.groups()[2];
    assert_eq!(group.profile, Profile::SupportedSpherePair);
    batch
        .gpu
        .queue
        .write_buffer(&batch.gpu.state, group.offset, &f64::NAN.to_le_bytes());
    assert!(matches!(
        batch.step(),
        Err(GpuError::NonfiniteEnvironment {
            id: 70,
            profile: Profile::SupportedSpherePair
        })
    ));
    assert_eq!(batch.environments(), committed);
    assert_eq!(batch.sim_time(), STEP);
    assert_eq!(batch.step().unwrap().uploaded_bytes, first.uploaded_bytes);
    let recovered = batch.environments().to_vec();
    batch.reset();
    assert_eq!(batch.environments(), seed);
    assert_eq!(batch.tick(), 0);
    batch.step().unwrap();
    assert_eq!(batch.step().unwrap().uploaded_bytes, 0);
    assert_eq!(batch.environments(), recovered);
    if let Some((allocated, reserved)) = batch.allocator_usage() {
        assert!(reserved >= allocated && allocated >= batch.device_info().allocated_buffer_bytes);
    }
}
#[test]
#[ignore = "requires selected f64 Vulkan hardware"]
fn shared_bevy_fault_retains_previous_epoch_clock_and_all_three_profiles() {
    use crate::simulation::{BatchTiming, mixed_batch_simulation};
    use gz_sim_core::UpdateInfo;
    let mut sim = mixed_batch_simulation(&selector(), initial()).unwrap();
    sim.tick().unwrap();
    assert_eq!(sim.world().resource::<MixedBatch>().tick(), 0);
    sim.step_exact(2).unwrap();
    let expected = sim.world().resource::<MixedBatch>().environments().to_vec();
    sim.reset().unwrap();
    assert_eq!(sim.world().resource::<UpdateInfo>().epoch, 1);
    assert!(sim.world().resource::<BatchTiming>().last.is_none());
    sim.step_exact(2).unwrap();
    let batch = sim.world().resource::<MixedBatch>();
    assert_eq!(batch.environments(), expected);
    let time = *sim.world().resource::<UpdateInfo>();
    batch.gpu.queue.write_buffer(
        &batch.gpu.state,
        batch.groups()[2].offset,
        &f64::NAN.to_le_bytes(),
    );
    let previous = sim.world().resource::<BatchTiming>().last.unwrap().latency;
    assert!(sim.step_exact(1).is_err());
    assert_eq!(*sim.world().resource::<UpdateInfo>(), time);
    assert_eq!(
        sim.world().resource::<BatchTiming>().last.unwrap().latency,
        previous
    );
    assert_eq!(
        sim.world().resource::<MixedBatch>().environments(),
        expected
    );
}

#[test]
#[ignore = "requires selected f64 Vulkan hardware"]
fn a_finite_registration_can_overflow_on_future_contact_without_publishing_siblings() {
    let mut seed = initial();
    let EnvironmentState::SupportedSpherePair(s) = &mut seed[0].state else {
        unreachable!()
    };
    // A finite inverse mass just above 1/f64::MAX admits a supported rest load.
    // With 0.11 m separation between surfaces, the second 1 ms tick reaches
    // impact at 80 m/s. Its normal impulse is outside the f64 output range.
    s.pair.a.inverse_mass = 9e-309;
    s.pair.b.inverse_mass = 9e-309;
    s.pair.a.position[0] = -0.155;
    s.pair.b.position[0] = 0.155;
    s.pair.a.velocity[0] = 40.;
    s.pair.b.velocity[0] = -40.;
    let mut batch = MixedBatch::new(&selector(), seed).unwrap();
    let init = batch.environments().to_vec();
    batch.step().unwrap();
    let prev = batch.environments().to_vec();
    for _ in 0..2 {
        assert!(matches!(
            batch.step(),
            Err(GpuError::NonfiniteEnvironment {
                id: 70,
                profile: Profile::SupportedSpherePair
            })
        ));
        assert_eq!(batch.environments(), prev);
        assert_eq!(batch.tick(), 1);
    }
    batch.reset();
    assert_eq!(batch.environments(), init);
    batch.step().unwrap();
    assert_eq!(batch.environments(), prev);
}

#[path = "episodes.rs"]
mod episodes;
