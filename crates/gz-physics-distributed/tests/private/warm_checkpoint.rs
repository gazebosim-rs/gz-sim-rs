// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn even_integrity_checked_archives_must_match_the_actual_solver_and_command_domain() {
    let world = Island::restore(&Snapshot {
        bodies: fixture().iter().map(Body::promoted).collect(),
        ..Snapshot::default()
    })
    .unwrap();
    let archive = world.checkpoint().unwrap();
    fn mismatched(original: &WarmCheckpoint, change: impl FnOnce(&mut WarmCheckpoint)) {
        let mut candidate = original.clone();
        change(&mut candidate);
        candidate.digest = candidate.checksum().unwrap();
        assert!(Island::restore_warm(&candidate).is_err());
    }
    mismatched(&archive, |s| s.snapshot.step = SimulationStep::Ms1);
    mismatched(&archive, |s| s.bindings.swap(0, 1));
    mismatched(&archive, |s| s.bindings[0] = 4);
    mismatched(&archive, |s| s.snapshot.bodies[1].position[1] = 1.);
    mismatched(&archive, |s| s.snapshot.bodies[0].tier = Tier::Prescribed);
    mismatched(&archive, |s| s.snapshot.bodies.truncate(1));
    mismatched(&archive, |s| s.manifold_count = 1);
    mismatched(&archive, |s| s.warmstart_bits.push(0));
    mismatched(&archive, |s| {
        s.pending.push(PendingImpulse {
            id: 4,
            impulse: [1.; 3],
        })
    });
    mismatched(&archive, |s| {
        s.native.pop().unwrap();
    });
}

#[test]
fn native_speed_limit_and_tick_overflow_preserve_queued_commands_at_the_last_barrier() {
    let mut world = Island::restore(&Snapshot {
        bodies: fixture().iter().map(Body::promoted).collect(),
        ..Snapshot::default()
    })
    .unwrap();
    world.queue_impulse(1, [398., 0., 0.]).unwrap();
    let before = world.checkpoint().unwrap();
    assert!(
        world
            .queue_impulse(1, [400.0_f64.next_up() - 400., 0., 0.])
            .is_err()
    );
    assert_eq!(world.checkpoint().unwrap(), before);
    let mut end = before.clone();
    end.snapshot.tick = u64::MAX;
    end.digest = end.checksum().unwrap();
    let mut restored = Island::restore_warm(&end).unwrap();
    assert!(matches!(restored.step(), Err(WorldError::TimeOverflow)));
    assert_eq!(restored.checkpoint().unwrap(), end);
}

#[test]
fn group_bindings_cover_exactly_the_native_world_and_restore_refuses_wrong_counts() {
    let prototype = fixture()[0].promoted();
    let bodies: Vec<_> = (0..3)
        .map(|id| Body {
            id,
            position: [id as f64 * 3., 0., 0.],
            ..prototype.clone()
        })
        .collect();
    let island = Island::restore(&Snapshot {
        bodies,
        ..Snapshot::default()
    })
    .unwrap();
    let capture = island.checkpoint().unwrap();
    fn reject(source: &WarmCheckpoint, edit: impl FnOnce(&mut WarmCheckpoint)) {
        let mut c = source.clone();
        edit(&mut c);
        c.digest = c.checksum().unwrap();
        assert!(Island::restore_warm(&c).is_err());
    }
    reject(&capture, |c| {
        c.bindings.pop();
    });
    reject(&capture, |c| c.bindings[2] = c.bindings[0]);
    reject(&capture, |c| {
        c.snapshot.bodies.pop();
        c.bindings.pop();
    });
    reject(&capture, |c| {
        c.snapshot.bodies[2].id = c.snapshot.bodies[1].id
    });
    reject(&capture, |c| c.manifold_count = 4);
    reject(&capture, |c| {
        c.snapshot
            .bodies
            .resize(MAX_CHECKPOINT_BODIES + 1, prototype.clone())
    });
}

#[test]
fn support_cache_cannot_be_relabelled_as_free_flight_or_another_elevation() {
    let g = GroundPlane::new(0.125).unwrap();
    let mut bodies = fixture();
    bodies[0] = bodies[0].promoted();
    for b in &mut bodies {
        b.position[2] = 0.45;
    }
    let mut local = Island::restore(&Snapshot {
        ground: Some(g),
        step: SimulationStep::Ms2,
        bodies,
        ..Default::default()
    })
    .unwrap();
    for _ in 0..80 {
        local.advance_native_field([0., 0., -9.81]).unwrap();
    }
    let image = local.checkpoint().unwrap();
    assert_eq!(image.snapshot.supports.len(), 2);
    assert!(image.warmstart_normal_impulses().any(|j| j > 0.));
    let mut resumed = Island::restore_warm(&image).unwrap();
    assert_eq!(
        resumed.checkpoint().unwrap().warmstart_bits,
        image.warmstart_bits
    );
    for _ in 0..8 {
        resumed.advance_native_field([0., 0., -9.81]).unwrap();
        local.advance_native_field([0., 0., -9.81]).unwrap();
        assert_eq!(resumed.committed, local.committed);
    }
    for ground in [
        None,
        Some(GroundPlane::new(0.).unwrap()),
        Some(GroundPlane::oriented([0., 0., g.height()], [1., 0., 0.], g.friction()).unwrap()),
    ] {
        let mut other = image.clone();
        other.snapshot.ground = ground;
        other.digest = other.checksum().unwrap();
        assert!(Island::restore_warm(&other).is_err());
    }
    let mut missing = image.clone();
    missing.snapshot.supports.clear();
    missing.digest = missing.checksum().unwrap();
    assert!(Island::restore_warm(&missing).is_err());
}

#[test]
fn corrected_loaded_observations_survive_restore_and_a_stale_force_witness_is_rejected() {
    let bodies: Vec<_> = [(1., 0.64 + 0.03 * 2.), (2., 2.14 + 0.06 * 5.)]
        .into_iter()
        .enumerate()
        .map(|(id, (mass, z))| Body {
            id: id as u64,
            mass,
            position: [-28., 32., z],
            friction: 1.,
            size: [0.55, 0.55, 0.4],
            linear: [0.; 3],
            angular: [0.; 3],
            ..fixture()[0].promoted()
        })
        .collect();
    let forces: Vec<_> = bodies
        .iter()
        .map(|b| BodyWrench {
            id: b.id,
            force: [0., 0., -9.81 * b.mass],
            torque: [0.; 3],
        })
        .collect();
    let mut source = Island::restore(&Snapshot {
        bodies,
        step: SimulationStep::Ms1,
        ground: Some(GroundPlane::with_friction(0., 0.5).unwrap()),
        ..Default::default()
    })
    .unwrap();
    for _ in 0..2500 {
        source.step_with_wrenches(&forces).unwrap();
    }
    let image = source.checkpoint().unwrap();
    let mut recovered = Island::restore_warm(&image).unwrap();
    assert!(image.warmstart_normal_impulses().any(|v| v > 0.));
    assert_eq!(source.snapshot(), recovered.snapshot());
    let mut stale = image.clone();
    stale.snapshot.impulses[0].normal_impulse += 0.005;
    stale.digest = stale.checksum().unwrap();
    assert!(matches!(
        Island::restore_warm(&stale),
        Err(WorldError::Checkpoint)
    ));
    let mut top_j = 0.;
    for _ in 0..500 {
        source.step_with_wrenches(&forces).unwrap();
        recovered.step_with_wrenches(&forces).unwrap();
        assert_eq!(source.snapshot(), recovered.snapshot());
        top_j += source.snapshot().impulses[0].normal_impulse;
    }
    assert!((top_j / 0.5 - 19.62).abs() < 1e-4);
}

#[test]
fn refined_checkpoint_rejects_invalid_or_mismatched_earlier_impulses() {
    let mut left = fixture()[0].promoted();
    left.position = [-0.34, 0., 0.];
    left.linear = [10., 0., 0.];
    let mut right = left.clone();
    right.id = 2;
    right.position[0] = 0.34;
    right.linear[0] = -10.;
    let mut island = Island::restore(&Snapshot {
        bodies: vec![left, right],
        ..Default::default()
    })
    .unwrap();
    island.step_refined_with_wrenches(&[], 10).unwrap();
    let good = island.checkpoint().unwrap();
    assert!(good.prefix.pairs[0].normal_impulse > 1.);
    assert_eq!(
        Island::restore_warm(&good).unwrap().snapshot(),
        good.snapshot
    );
    for modify in [
        |s: &mut WarmCheckpoint| s.prefix.pairs[0].normal_impulse = -1.,
        |s: &mut WarmCheckpoint| s.prefix.pairs[0].b = u64::MAX,
        |s: &mut WarmCheckpoint| s.prefix.pairs[0].normal_impulse += 1.,
        |s: &mut WarmCheckpoint| s.snapshot.impulses[0].normal_impulse = f64::INFINITY,
        |s: &mut WarmCheckpoint| s.prefix.pairs.clear(),
    ] {
        let mut bad = good.clone();
        modify(&mut bad);
        // The envelope must detect alteration even before the cross-check
        // against the final solve's native receipt.
        assert!(Island::restore_warm(&bad).is_err());
        bad.digest = bad.checksum().unwrap();
        assert!(Island::restore_warm(&bad).is_err());
    }
}
