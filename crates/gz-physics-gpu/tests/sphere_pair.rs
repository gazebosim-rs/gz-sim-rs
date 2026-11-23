// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{
    GpuError, SolidSphere as Body, SpherePairBatch, SpherePairLane as Lane,
    simulation::sphere_pair_simulation,
};
use std::error::Error;
fn pair() -> Lane {
    let a = Body {
        position: [-0.101, 0., 0.],
        velocity: [2., 0., 0.],
        angular_velocity: [0.; 3],
        orientation: [0., 0., 0., 1.],
        radius: 0.1,
        inverse_mass: 1.,
    };
    let b = Body {
        position: [0.101, 0., 0.],
        velocity: [-2., 0., 0.],
        ..a
    };
    Lane {
        a,
        b,
        gravity: [0.; 3],
        friction: 0.,
        restitution: 1.,
        normal_impulse: 0.,
        tangential_impulse: [0.; 3],
        contact_normal: [0.; 3],
        impact_time: -1.,
    }
}
#[test]
fn invalid_pair_is_rejected_before_creating_device() {
    let p = pair();
    for s in [
        Lane {
            a: Body {
                position: p.b.position,
                ..p.a
            },
            ..p
        },
        Lane {
            a: Body {
                position: [-0.04, 0., 0.],
                ..p.a
            },
            ..p
        },
        Lane {
            b: Body { radius: 0., ..p.b },
            ..p
        },
        Lane {
            a: Body {
                inverse_mass: 0.,
                ..p.a
            },
            ..p
        },
        Lane {
            b: Body {
                orientation: [0.; 4],
                ..p.b
            },
            ..p
        },
        Lane {
            gravity: [0., 0., f64::NAN],
            ..p
        },
        Lane {
            friction: -0.1,
            ..p
        },
        Lane {
            restitution: 1.01,
            ..p
        },
        Lane {
            restitution: -0.1,
            ..p
        },
        Lane {
            impact_time: 0.,
            ..p
        },
        Lane {
            contact_normal: [1., 0., 0.],
            ..p
        },
        Lane {
            normal_impulse: 2.,
            ..p
        },
        Lane {
            tangential_impulse: [0., 1., 0.],
            ..p
        },
        Lane {
            a: Body {
                velocity: [1e305, 0., 0.],
                ..p.a
            },
            ..p
        },
    ] {
        assert!(matches!(
            SpherePairBatch::new("unused", vec![p, s]),
            Err(GpuError::InvalidLane(1))
        ));
    }
}
#[test]
#[ignore = "requires an explicitly selected native-f64 Vulkan adapter"]
fn elastic_swap_ccd_coasting_and_grazing_are_distinct() -> Result<(), Box<dyn Error>> {
    let n = std::env::var("GZ_GPU_ADAPTER")?;
    let p = pair();
    let bullet = Lane {
        a: Body {
            position: [-1., 0., 0.],
            velocity: [3000., 0., 0.],
            ..p.a
        },
        b: Body {
            position: [1., 0., 0.],
            velocity: [-3000., 0., 0.],
            ..p.b
        },
        ..p
    };
    let miss = Lane {
        a: Body {
            position: [-1., 0.2001, 0.],
            ..bullet.a
        },
        ..bullet
    };
    let graze = Lane {
        a: Body {
            position: [-1., 0.1999, 0.],
            ..bullet.a
        },
        ..bullet
    };
    let static_pair = Lane {
        a: Body {
            velocity: [2., 3., 4.],
            ..p.a
        },
        b: Body {
            velocity: [2., 3., 4.],
            ..p.b
        },
        gravity: [1., -2., -9.],
        ..p
    };
    let spin = Lane {
        a: Body {
            position: [-0.1, 0., 0.],
            angular_velocity: [0., 0., 10.],
            ..p.a
        },
        b: Body {
            position: [0.1, 0., 0.],
            ..p.b
        },
        friction: 1.,
        restitution: 0.,
        ..p
    };
    let endpoint = Lane {
        a: Body {
            position: [-0.102, 0., 0.],
            ..p.a
        },
        b: Body {
            position: [0.102 + f64::EPSILON, 0., 0.],
            ..p.b
        },
        ..p
    };
    let beyond = Lane {
        b: Body {
            position: [0.102 + 1e-9, 0., 0.],
            ..endpoint.b
        },
        ..endpoint
    };
    let initial = vec![p, bullet, miss, graze, static_pair, spin, endpoint, beyond];
    let mut sim = sphere_pair_simulation(&n, initial.clone())?;
    sim.tick()?;
    assert_eq!(sim.world().resource::<SpherePairBatch>().tick(), 0);
    sim.step_exact(1)?;
    let states = sim.world().resource::<SpherePairBatch>().lanes();
    let close = |x: f64, y: f64| assert!((x - y).abs() < 1e-10, "{x} versus {y}");
    close(states[0].a.velocity[0], -2.);
    close(states[0].b.velocity[0], 2.);
    close(states[0].normal_impulse, 4.);
    close(states[0].impact_time, 0.0005);
    close(states[1].impact_time, 0.0003);
    close(states[1].a.velocity[0], -3000.);
    close(states[1].b.velocity[0], 3000.);
    assert_eq!(states[2].impact_time, -1.);
    assert!(states[3].normal_impulse > 0.);
    assert_eq!(states[4].normal_impulse, 0.);
    for j in 0..3 {
        close(
            states[4].b.position[j] - states[4].a.position[j],
            static_pair.b.position[j] - static_pair.a.position[j],
        );
    }
    close(states[6].impact_time, 0.001);
    assert_eq!(states[7].impact_time, -1.);
    let stick = states[5];
    close(stick.impact_time, 0.);
    // Surfaces co-move tangentially after the impulse.
    close(
        stick.a.velocity[1] + 0.1 * stick.a.angular_velocity[2],
        stick.b.velocity[1] - 0.1 * stick.b.angular_velocity[2],
    );
    let first = states.to_vec();
    sim.reset()?;
    assert_eq!(sim.world().resource::<SpherePairBatch>().lanes(), initial);
    sim.step_exact(1)?;
    assert_eq!(sim.world().resource::<SpherePairBatch>().lanes(), first);
    Ok(())
}

#[test]
fn invalid_pair_observation_is_refused_before_device_creation() {
    use gz_physics_gpu::{
        MAX_BLOCK_TICKS, SpherePairBlockBatch, simulation::sphere_pair_block_simulation,
    };
    for n in [0, MAX_BLOCK_TICKS + 1] {
        assert!(
            matches!(sphere_pair_block_simulation("unused", vec![pair()], n), Err(GpuError::BlockLength(v)) if v==n)
        );
    }
    let x = Lane {
        restitution: -1.,
        ..pair()
    };
    assert!(matches!(
        SpherePairBlockBatch::new("unused", vec![pair(), x]),
        Err(GpuError::InvalidLane(1))
    ));
}

#[test]
#[ignore = "requires an explicitly selected native-f64 Vulkan adapter"]
fn resident_pair_observes_subtick_impacts_and_restores_a_failed_block() -> Result<(), Box<dyn Error>>
{
    use gz_physics_gpu::{SpherePairBlockBatch, simulation::sphere_pair_block_simulation};
    let adapter = std::env::var("GZ_GPU_ADAPTER")?;
    let x = pair();
    let mut lanes = vec![x; 71];
    for (i, lane) in lanes.iter_mut().enumerate() {
        lane.a.position[0] -= 0.001 * i as f64;
        lane.a.position[1] += if i % 3 == 0 { 0.205 } else { 0.074 };
        lane.a.angular_velocity = [3., -0.4, -7.2];
        lane.b.angular_velocity = [1.2, 9., 2.3];
        lane.friction = 0.3;
    }
    let mut dense = SpherePairBatch::new(&adapter, lanes.clone())?;
    let mut block = SpherePairBlockBatch::new(&adapter, lanes.clone())?;
    let mut hits = 0;
    for k in [1, 3, 7, 13, 30, 1] {
        let mut sums = vec![([0.; 3], 0., 0f64, 0, None); lanes.len()];
        for i in 0..k {
            dense.step()?;
            for (sum, s) in sums.iter_mut().zip(dense.lanes()) {
                if s.normal_impulse > 0. {
                    let t = 0.001 * (i as f64) + s.impact_time;
                    let (_, last) = sum.4.get_or_insert((t, t));
                    *last = t;
                    sum.3 += 1;
                    hits += 1;
                }
                for axis in 0..3 {
                    sum.0[axis] +=
                        s.contact_normal[axis] * s.normal_impulse + s.tangential_impulse[axis];
                }
                sum.1 += s.normal_impulse;
                sum.2 = sum.2.max(s.normal_impulse);
            }
        }
        block.step_block(k)?;
        for ((obs, state), (j, jn, peak, count, times)) in
            block.lanes().iter().zip(dense.lanes()).zip(sums)
        {
            assert_eq!(bytemuck::bytes_of(&obs.state), bytemuck::bytes_of(state));
            for (actual, want) in obs.impulse_on_b.into_iter().zip(j) {
                assert!((actual - want).abs() <= 4. * f64::EPSILON * want.abs().max(1.));
            }
            assert_eq!(
                (
                    obs.normal_impulse_sum,
                    obs.maximum_normal_impulse,
                    obs.contact_ticks()
                ),
                (jn, peak, count)
            );
            assert_eq!(obs.contact_times(), times);
        }
    }
    assert_eq!(hits, 47);
    let mut sim = sphere_pair_block_simulation(&adapter, lanes, 17)?;
    sim.tick()?;
    assert_eq!(sim.world().resource::<SpherePairBlockBatch>().tick(), 0);
    sim.step_exact(2)?;
    assert_eq!(sim.world().resource::<SpherePairBlockBatch>().tick(), 34);
    sim.reset()?;
    let reset = sim.world().resource::<SpherePairBlockBatch>();
    assert_eq!(reset.tick(), 0);
    assert!(reset.lanes().iter().all(|o| o.contact_times().is_none()));

    // Finite trajectories up to the second tick. At impact the impulse for
    // these two very massive bodies overflows f64.
    let extreme = Lane {
        a: Body {
            position: [0., 0., 0.],
            velocity: [0.; 3],
            radius: 1000.,
            inverse_mass: 1e-300,
            ..x.a
        },
        b: Body {
            position: [1.5e17, 0., 0.],
            velocity: [-1e20, 0., 0.],
            radius: 1000.,
            inverse_mass: 1e-300,
            ..x.b
        },
        ..x
    };
    let mut fail = SpherePairBlockBatch::new(&adapter, vec![x, extreme])?;
    let start = fail.lanes().to_vec();
    assert!(matches!(
        fail.step_block(3),
        Err(GpuError::InvalidIntermediate { lane: 1, offset: 2 })
    ));
    assert_eq!(fail.lanes(), start);
    fail.step_block(1)?;
    let first = fail.lanes().to_vec();
    assert!(matches!(
        fail.step_block(2),
        Err(GpuError::InvalidIntermediate { lane: 1, offset: 1 })
    ));
    assert_eq!((fail.tick(), fail.lanes()), (1, first.as_slice()));
    Ok(())
}

#[test]
#[ignore = "requires an explicitly selected native-f64 Vulkan adapter"]
fn warm_block_checkpoint_reopens_on_new_device_and_restores_bevy_epoch()
-> Result<(), Box<dyn Error>> {
    use gz_physics_gpu::{
        SpherePairBlockBatch, SpherePairBlockCheckpoint as Checkpoint, SphereRotation,
        simulation::sphere_pair_block_simulation_with_rotation,
    };
    use sha2::{Digest, Sha256};
    let name = std::env::var("GZ_GPU_ADAPTER")?;
    let mut lane = pair();
    lane.a.angular_velocity = [9., 17., -14.];
    lane.friction = 0.4;
    for mode in [SphereRotation::Midpoint, SphereRotation::FourthOrder] {
        let mut s = sphere_pair_block_simulation_with_rotation(&name, vec![lane; 67], 7, mode)?;
        s.reset()?;
        s.step_exact(1)?;
        let saved = *s.world().resource::<gz_sim_core::UpdateInfo>();
        s.world_mut()
            .resource_mut::<gz_sim_core::UpdateInfo>()
            .epoch += 1;
        assert!(Checkpoint::capture_simulation(&s).is_err());
        s.world_mut().insert_resource(saved);
        let warm = Checkpoint::capture_simulation(&s)?;
        assert_eq!(
            (
                warm.tick(),
                warm.last_block_ticks(),
                warm.lanes(),
                warm.rotation()
            ),
            (7, 7, 67, mode)
        );
        assert_eq!(warm.observations()[0].contact_ticks(), 1);
        let bytes = warm.to_bytes()?;
        let decoded = Checkpoint::from_bytes(&bytes)?;
        assert_eq!(decoded.to_bytes()?, bytes);
        for pos in [1, 8, 24, 48, 95, bytes.len() - 1] {
            let mut corrupt = bytes.clone();
            corrupt[pos] ^= 1;
            assert!(Checkpoint::from_bytes(&corrupt).is_err());
        }
        // Valid checksum but inconsistent public clock and immutable lane ABI.
        for pos in [
            8,
            9,
            10,
            12,
            16,
            24,
            28,
            40,
            80 + 13 * 8,
            80 + 67 * std::mem::size_of::<Lane>() + 28 * 8,
        ] {
            let mut wrong = bytes.clone();
            wrong[pos] ^= 0x40;
            let end = wrong.len() - 32;
            let digest = Sha256::digest(&wrong[..end]);
            wrong[end..].copy_from_slice(&digest);
            assert!(
                Checkpoint::from_bytes(&wrong).is_err(),
                "malformed offset {pos}"
            );
        }
        let mut other = decoded.restore_simulation(&name)?;
        assert_eq!(s.elapsed(), other.elapsed());
        assert_eq!(s.is_paused(), other.is_paused());
        assert_eq!(
            other.world().resource::<SpherePairBlockBatch>().lanes(),
            warm.observations()
        );
        assert_eq!(Checkpoint::capture_simulation(&other)?.to_bytes()?, bytes);
        for _ in 0..3 {
            s.step_exact(1)?;
            other.step_exact(1)?;
            assert_eq!(
                Checkpoint::capture_simulation(&s)?.to_bytes()?,
                Checkpoint::capture_simulation(&other)?.to_bytes()?
            );
        }
        s.set_paused(true);
        s.tick()?;
        let paused = Checkpoint::capture_simulation(&s)?;
        let mut reset = paused.restore_simulation(&name)?;
        assert!(reset.is_paused());
        reset.reset()?;
        let cold = Checkpoint::capture_simulation(&reset)?;
        assert!(
            cold.observations()
                .iter()
                .all(|r| r.state == lane && r.contact_ticks() == 0)
        );
        assert_eq!(Checkpoint::from_bytes(&cold.to_bytes()?)?.tick(), 0);
        let mut direct = decoded.restore(&name)?;
        direct.step_block(19)?;
        let direct = Checkpoint::capture(&direct)?;
        assert_eq!(direct.tick(), 26);
        assert!(direct.restore_simulation(&name).is_err());
        let mut wrong = bytes;
        wrong[48] ^= 1;
        let end = wrong.len() - 32;
        let digest = Sha256::digest(&wrong[..end]);
        wrong[end..].copy_from_slice(&digest);
        assert!(Checkpoint::from_bytes(&wrong)?.restore(&name).is_err());
    }
    Ok(())
}

#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn disjoint_device_workers_reassemble_out_of_order_across_epoch_and_width_changes()
-> Result<(), Box<dyn Error>> {
    use gz_physics_gpu::{
        PairBlockAssignment as Assignment, PairBlockBarrier as Barrier,
        PairBlockReceipt as Receipt, SpherePairBlockCheckpoint as Cut, SphereRotation,
        simulation::sphere_pair_block_simulation_with_rotation,
    };
    let name = std::env::var("GZ_GPU_ADAPTER")?;
    let seeds = (0..11)
        .map(|i| {
            let mut s = pair();
            s.a.position[2] += 0.11 * i as f64;
            s.b.position[2] += 0.11 * i as f64;
            s
        })
        .collect();
    let mut oracle =
        sphere_pair_block_simulation_with_rotation(&name, seeds, 7, SphereRotation::FourthOrder)?;
    oracle.reset()?;
    let mut cut = Cut::capture_simulation(&oracle)?;
    assert!(Barrier::new(&cut, 8, 2).is_err());
    assert!(Barrier::new(&cut, 7, 0).is_err());
    assert!(Barrier::new(&cut, 7, 12).is_err());
    let mut previous: Option<Vec<u8>> = None;
    for width in [3, 2, 4] {
        let step = Barrier::new(&cut, 7, width)?;
        assert!(step.assignment(width).is_err());
        assert!(step.commit(Vec::new()).is_err());
        if let Some(old) = previous.take() {
            assert!(step.commit([Receipt::from_bytes(&old)?]).is_err());
        }
        let mut receipts = Vec::new();
        for i in (0..width).rev() {
            let task = step.assignment(i)?;
            let bytes = task.to_bytes()?;
            assert!(Receipt::from_bytes(&bytes).is_err());
            let receipt = Assignment::from_bytes(&bytes)?.execute(&name)?.to_bytes()?;
            let mut corrupt = receipt.clone();
            corrupt[40] ^= 1;
            assert!(Receipt::from_bytes(&corrupt).is_err());
            // Same range and exact same acknowledged result may be delivered twice.
            if i == 0 {
                receipts.push(Receipt::from_bytes(&receipt)?);
                previous = Some(receipt.clone());
            }
            receipts.push(Receipt::from_bytes(&receipt)?);
        }
        if width == 3 {
            let wrong = Barrier::new(&cut, 7, 2)?.assignment(0)?.execute(&name)?;
            assert!(step.commit([wrong]).is_err());
        }
        let next = step.commit(receipts)?;
        oracle.step_exact(1)?;
        assert_eq!(
            next.to_bytes()?,
            Cut::capture_simulation(&oracle)?.to_bytes()?
        );
        assert_eq!(next.restore_simulation(&name)?.epoch(), oracle.epoch());
        cut = next;
    }
    Ok(())
}

#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn compact_continuations_reject_old_or_reassigned_ranges_without_advancing_the_worker()
-> Result<(), Box<dyn Error>> {
    use gz_physics_gpu::{
        PairBlockBarrier as Barrier, PairBlockContinuation as Cue, PairBlockExecutor,
        SpherePairBlockBatch, SpherePairBlockCheckpoint as Cut,
    };
    use sha2::{Digest, Sha256};
    let name = std::env::var("GZ_GPU_ADAPTER")?;
    // Identical lane physics also exercises the independent range-identity check.
    let mut oracle = SpherePairBlockBatch::new(&name, vec![pair(); 4])?;
    let cold = Cut::capture(&oracle)?;
    let first = Barrier::new(&cold, 7, 2)?;
    let mut worker = PairBlockExecutor::new(&name);
    let old = first.continuation(0)?;
    assert!(worker.try_continue(old)?.is_none());
    let (receipt, _, reused) = worker.execute(first.assignment(0)?)?;
    assert!(!reused);
    let cut = first.commit([receipt, first.assignment(1)?.execute(&name)?])?;
    let next = Barrier::new(&cut, 7, 2)?;
    assert!(worker.try_continue(old)?.is_none());
    assert!(worker.try_continue(next.continuation(1)?)?.is_none());
    let bytes = next.continuation(0)?.to_bytes();
    assert!(Cue::from_bytes(&bytes[1..]).is_err());
    for offset in [0, 8, 40, 72, 76, 111] {
        let mut broken = bytes;
        broken[offset] ^= 1;
        assert!(Cue::from_bytes(&broken).is_err());
    }
    let mut invalid = bytes;
    invalid[76..80].fill(0);
    let digest = Sha256::digest(&invalid[..80]);
    invalid[80..].copy_from_slice(&digest);
    assert!(Cue::from_bytes(&invalid).is_err());
    let (continued, metrics) = worker
        .try_continue(Cue::from_bytes(&bytes)?)?
        .expect("original input is still warm");
    assert_eq!(metrics.uploaded_bytes, 4);
    assert!(worker.try_continue(next.continuation(0)?)?.is_none());
    let committed = next.commit([continued, next.assignment(1)?.execute(&name)?])?;
    oracle.step_block(7)?;
    oracle.step_block(7)?;
    assert_eq!(committed.to_bytes()?, Cut::capture(&oracle)?.to_bytes()?);
    Ok(())
}

#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn process_pool_migrates_at_bevy_barriers_and_publishes_only_the_full_interval()
-> Result<(), Box<dyn Error>> {
    use gz_physics_gpu::{
        SpherePairBlockBatch, SpherePairBlockCheckpoint as Cut,
        pair_process::{self, PairProcessPool},
        simulation::{self, BatchTiming},
    };
    use std::{path::Path, time::Duration};
    let adapter = std::env::var("GZ_GPU_ADAPTER")?;
    let exe = Path::new(env!("CARGO_BIN_EXE_gz_pair_worker"));
    let lanes = vec![pair(); 5];
    let batch = SpherePairBlockBatch::new(&adapter, lanes.clone())?;
    let mut scene = gz_sim_core::Simulation::new(Duration::from_millis(7))?;
    let pool = PairProcessPool::new(exe, &adapter, 2)?;
    pair_process::install(&mut scene, batch, 7, pool)?;
    let mut oracle = simulation::sphere_pair_block_simulation(&adapter, lanes, 7)?;
    for k in 0..12 {
        let at = simulation::batch_boundary(&scene);
        if k == 3 {
            assert!(pair_process::set_width(&mut scene, at, 6).is_err());
            pair_process::set_width(&mut scene, at, 3)?;
        }
        if k == 7 {
            pair_process::set_width(&mut scene, at, 1)?;
        }
        scene.step_exact(1)?;
        oracle.step_exact(1)?;
        assert_eq!(
            Cut::capture_simulation(&scene)?.to_bytes()?,
            Cut::capture_simulation(&oracle)?.to_bytes()?
        );
        assert!(pair_process::set_width(&mut scene, at, 2).is_err());
        let metrics = scene.world().resource::<PairProcessPool>().last().unwrap();
        assert_eq!(
            metrics.workers,
            if k < 3 {
                2
            } else if k < 7 {
                3
            } else {
                1
            }
        );
        if ![0, 3, 7].contains(&k) {
            assert_eq!(metrics.reused, metrics.workers);
            assert_eq!(metrics.upload_bytes, 4 * metrics.workers as u64);
        }
    }
    scene.set_paused(true);
    oracle.set_paused(true);
    scene.tick()?;
    oracle.tick()?;
    assert_eq!(
        Cut::capture_simulation(&scene)?.to_bytes()?,
        Cut::capture_simulation(&oracle)?.to_bytes()?
    );
    scene.reset()?;
    oracle.reset()?;
    assert!(scene.world().resource::<BatchTiming>().last.is_none());
    assert!(scene.world().resource::<PairProcessPool>().last().is_none());
    assert_eq!(
        Cut::capture_simulation(&scene)?.to_bytes()?,
        Cut::capture_simulation(&oracle)?.to_bytes()?
    );
    scene.step_exact(1)?;
    oracle.step_exact(1)?;
    assert_eq!(
        Cut::capture_simulation(&scene)?.to_bytes()?,
        Cut::capture_simulation(&oracle)?.to_bytes()?
    );
    Ok(())
}

#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn measured_rows_reconstruct_only_the_correct_full_worker_successor() -> Result<(), Box<dyn Error>>
{
    use gz_physics_gpu::{
        PairBlockBarrier as Barrier, SpherePairBlockBatch, SpherePairBlockCheckpoint as Cut,
    };
    let adapter = std::env::var("GZ_GPU_ADAPTER")?;
    let mut oracle = SpherePairBlockBatch::new(&adapter, vec![pair(); 9])?;
    let mut cut = Cut::capture(&oracle)?;
    let mut last: Option<Vec<u8>> = None;
    for span in [7, 3, 8] {
        let b = Barrier::new(&cut, span, 2)?;
        let mut receipts = Vec::new();
        for i in 0..2 {
            let full = b.assignment(i)?.execute(&adapter)?;
            let rows = full.observation_bytes()?;
            assert!(rows.len() < full.to_bytes()?.len());
            assert_eq!(
                b.observation_receipt(i, &rows)?.to_bytes()?,
                full.to_bytes()?
            );
            assert!(b.observation_receipt(1 - i, &rows).is_err());
            if i == 0 {
                if let Some(old) = last.as_ref() {
                    assert!(b.observation_receipt(0, old).is_err());
                }
                for index in [0, 8, 40, 44, 48, 56, 60, 64, 96, rows.len() - 1] {
                    let mut corrupt = rows.clone();
                    corrupt[index] ^= 1;
                    assert!(
                        b.observation_receipt(i, &corrupt).is_err(),
                        "corruption at {index}"
                    );
                }
                assert!(b.observation_receipt(i, &rows[..rows.len() - 1]).is_err());
                last = Some(rows.clone());
            }
            receipts.push(b.observation_receipt(i, &rows)?);
        }
        cut = b.commit(receipts)?;
        oracle.step_block(span)?;
        assert_eq!(cut.to_bytes()?, Cut::capture(&oracle)?.to_bytes()?);
    }
    Ok(())
}
