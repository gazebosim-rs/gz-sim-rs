// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{
    GpuError, MAX_BLOCK_TICKS, SpherePlaneBatch, SpherePlaneBlockBatch, SpherePlaneLane as Lane,
    simulation::sphere_plane_block_simulation,
};
use std::{error::Error, time::Duration};
fn lane() -> Lane {
    Lane {
        position: [0., 0., 0.15],
        radius: 0.1,
        velocity: [0.7, 0.3, -0.5],
        angular_velocity: [2., -0.1, 0.3],
        orientation: [0., 0., 0., 1.],
        gravity: [0.4, -0.2, -9.81],
        plane_normal: [0., 0., 1.],
        plane_offset: 0.,
        inverse_mass: 1.6,
        friction: 0.18,
        normal_impulse: 0.,
        tangential_impulse: [0.; 3],
    }
}

#[test]
fn invalid_bevy_cadence_and_state_do_not_need_a_device() {
    for n in [0, MAX_BLOCK_TICKS + 1, u32::MAX] {
        assert!(
            matches!(sphere_plane_block_simulation("unused",vec![lane()],n),Err(GpuError::BlockLength(v)) if v==n)
        );
    }
    let mut bad = lane();
    bad.friction = -1.;
    assert!(matches!(
        SpherePlaneBlockBatch::new("unused", vec![lane(), bad]),
        Err(GpuError::InvalidLane(1))
    ));
}

#[test]
#[ignore = "requires explicitly selected native-f64 Vulkan hardware"]
fn variable_blocks_publish_observations_and_reset_at_bevy_boundaries() -> Result<(), Box<dyn Error>>
{
    let gpu = std::env::var("GZ_GPU_ADAPTER")?;
    let initial = vec![lane(); 67];
    let mut dense = SpherePlaneBatch::new(&gpu, initial.clone())?;
    let mut resident = SpherePlaneBlockBatch::new(&gpu, initial.clone())?;
    let mut total = 0;
    for k in [1, 5, 27, 93, 4, 15] {
        let mut normal = 0.;
        let mut tangent = [0.; 3];
        let mut range = None;
        let mut count = 0;
        let mut peak = 0f64;
        for tick in 1..=k {
            dense.step()?;
            let s = dense.lanes()[0];
            normal += s.normal_impulse;
            for (sum, value) in tangent.iter_mut().zip(s.tangential_impulse) {
                *sum += value;
            }
            peak = peak.max(s.normal_impulse);
            if s.normal_impulse > 0. {
                count += 1;
                let (_, end) = range.get_or_insert((tick, tick));
                *end = tick;
            }
        }
        let m = resident.step_block(k)?;
        total += u64::from(k);
        assert_eq!(resident.tick(), total);
        assert_eq!(resident.sim_time(), Duration::from_millis(total));
        assert_eq!(
            m.uploaded_bytes,
            4 + if total == 1 {
                resident.device_info().state_bytes
            } else {
                0
            }
        );
        for (obs, reference) in resident.lanes().iter().zip(dense.lanes()) {
            assert_eq!(
                bytemuck::bytes_of(&obs.state),
                bytemuck::bytes_of(reference)
            );
            assert_eq!(obs.normal_impulse_sum, normal);
            assert_eq!(obs.tangential_impulse_sum, tangent);
            assert_eq!(obs.contact_ticks(), count);
            assert_eq!(obs.contact_range(), range);
            assert_eq!(obs.maximum_normal_impulse, peak);
        }
    }
    let committed = resident.lanes().to_vec();
    assert!(matches!(
        resident.step_block(0),
        Err(GpuError::BlockLength(0))
    ));
    assert!(matches!(
        resident.step_block(MAX_BLOCK_TICKS + 1),
        Err(GpuError::BlockLength(_))
    ));
    assert_eq!(resident.lanes(), committed);
    assert_eq!(resident.tick(), total);
    resident.reset();
    resident.step()?;
    assert_eq!(resident.tick(), 1);

    let mut sim = sphere_plane_block_simulation(&gpu, initial, 17)?;
    sim.tick()?;
    assert_eq!(sim.world().resource::<SpherePlaneBlockBatch>().tick(), 0);
    sim.step_exact(2)?;
    assert_eq!(sim.world().resource::<SpherePlaneBlockBatch>().tick(), 34);
    sim.reset()?;
    assert_eq!(sim.world().resource::<SpherePlaneBlockBatch>().tick(), 0);
    Ok(())
}
#[test]
#[ignore = "requires explicitly selected native-f64 Vulkan hardware"]
fn failed_intermediate_is_indexed_and_the_previous_barrier_is_reusable()
-> Result<(), Box<dyn Error>> {
    let gpu = std::env::var("GZ_GPU_ADAPTER")?;
    // First 1 ms free step is finite; the second reaches the plane and its
    // required stopping impulse is larger than f64::MAX.
    let extreme = Lane {
        position: [0., 0., 1.5e117],
        velocity: [0., 0., -1e120],
        gravity: [0.; 3],
        inverse_mass: 1e-200,
        ..lane()
    };
    let mut b = SpherePlaneBlockBatch::new(&gpu, vec![lane(), extreme])?;
    let initial = b.lanes().to_vec();
    assert!(matches!(
        b.step_block(3),
        Err(GpuError::InvalidIntermediate { lane: 1, offset: 2 })
    ));
    assert_eq!(b.tick(), 0);
    assert_eq!(b.lanes(), initial);
    b.step_block(1)?;
    let saved = b.lanes().to_vec();
    assert!(matches!(
        b.step_block(2),
        Err(GpuError::InvalidIntermediate { lane: 1, offset: 1 })
    ));
    assert_eq!(b.tick(), 1);
    assert_eq!(b.lanes(), saved);
    b.reset();
    assert_eq!(b.lanes(), initial);
    Ok(())
}
