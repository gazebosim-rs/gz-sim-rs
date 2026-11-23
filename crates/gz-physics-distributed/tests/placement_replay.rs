// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::Error,
    physics::{Body, BodyWrench, Primitive, SimulationStep, fixture},
    pool::{PlacementMember, PlacementMove, PlacementRecord, PoolNativeRecovery, WorkerPool},
};
use std::{error::Error as StdError, path::Path};
type Test = Result<(), Box<dyn StdError>>;
const G: [f64; 3] = [0.; 3];
fn exe() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn body(id: u64, x: f64) -> Body {
    Body {
        id,
        position: [x, 0., 2.],
        linear: [0.; 3],
        primitive: Primitive::Sphere,
        ..fixture()[1].clone()
    }
}
fn spawn(workers: Vec<Vec<Body>>, limit: usize) -> Result<WorkerPool, Error> {
    WorkerPool::spawn(exe(), 92157, workers, SimulationStep::Ms10, None, limit)
}
fn simple(limit: usize) -> Result<WorkerPool, Error> {
    spawn(
        vec![
            vec![body(11, -5.), body(12, 5.)],
            vec![body(13, 15.)],
            vec![],
            vec![],
        ],
        limit,
    )
}
fn stay(tick: u64) -> PlacementRecord {
    PlacementRecord {
        target_tick: tick,
        optional_move: None,
    }
}
fn movement(p: &WorkerPool, id: u64, destination: u8) -> PlacementRecord {
    let source = p.frame().body(id).unwrap().0;
    let owner = &p.frame().workers[source as usize];
    let ids = &owner
        .components
        .iter()
        .find(|c| c.ids.contains(&id))
        .unwrap()
        .ids;
    PlacementRecord {
        target_tick: p.frame().tick + 1,
        optional_move: Some(PlacementMove {
            source,
            destination,
            members: ids
                .iter()
                .map(|&id| PlacementMember {
                    id,
                    generation: owner.generations[&id],
                })
                .collect(),
        }),
    }
}
fn counters(p: &WorkerPool) -> Vec<(u64, u64, u64, u64)> {
    p.traffic()
        .into_iter()
        .map(|t| (t.requests, t.replies, t.request_bytes, t.reply_bytes))
        .collect()
}
fn refused(p: &mut WorkerPool, r: &PlacementRecord, g: [f64; 3], loads: &[BodyWrench]) -> Test {
    let traffic = counters(p);
    let frame = p.frame().clone();
    let receipt = p.last_island_placement().cloned();
    assert!(p.advance_islands_with_placement(g, loads, r).is_err());
    assert_eq!(p.frame(), &frame);
    assert_eq!(p.last_island_placement(), receipt.as_ref());
    assert_eq!(counters(p), traffic);
    assert!(!p.rollback_interrupted_placement()?);
    Ok(())
}
#[test]
fn full_canonical_membership_generations_and_tick_are_fenced_before_any_request() -> Test {
    let mut pool = WorkerPool::spawn_canonical_scene(
        exe(),
        92158,
        vec![body(11, -5.), body(12, 5.)],
        4,
        SimulationStep::Ms10,
        None,
        32,
    )?;
    let valid = movement(&pool, 12, 2);
    assert_eq!(valid.optional_move.as_ref().unwrap().members.len(), 2);
    for change in 0..10 {
        let mut bad = valid.clone();
        let m = bad.optional_move.as_mut().unwrap();
        match change {
            0 => bad.target_tick = 0,
            1 => bad.target_tick = 2,
            2 => m.source = 1,
            3 => m.destination = 0,
            4 => m.destination = 4,
            5 => {
                m.members.remove(1);
            }
            6 => m.members.swap(0, 1),
            7 => m.members[1].id = m.members[0].id,
            8 => m.members[1].generation = 1,
            _ => m.members.clear(),
        }
        refused(&mut pool, &bad, G, &[])?;
    }
    pool.advance_islands_with_placement(G, &[], &valid)?;
    assert_eq!(pool.last_island_placement(), Some(&valid));
    assert_eq!(pool.frame().workers[2].generations[&11], 1);
    let mut stale = movement(&pool, 12, 0);
    stale.optional_move.as_mut().unwrap().members[0].generation = 0;
    refused(&mut pool, &stale, G, &[])?;
    refused(&mut pool, &valid, G, &[])?;
    let back = movement(&pool, 12, 0);
    pool.advance_islands_with_placement(G, &[], &back)?;
    assert_eq!(pool.frame().workers[0].generations[&12], 2);
    pool.advance()?;
    assert!(pool.last_island_placement().is_none());
    Ok(())
}
#[test]
fn journal_residency_and_queued_input_budgets_refuse_replay_without_locking() -> Test {
    let mut tight = simple(3)?;
    let r = movement(&tight, 11, 2);
    assert!(matches!(
        tight.advance_islands_with_placement(G, &[], &r),
        Err(Error::Capacity)
    ));
    refused(&mut tight, &r, G, &[])?;
    tight.advance_islands_with_placement(G, &[], &stay(1))?;
    let mut inputs = simple(128)?;
    for _ in 0..32 {
        inputs.queue_impulse(13, [0.001, 0., 0.])?;
    }
    inputs.queue_impulse(11, [0.001, 0., 0.])?;
    let r = movement(&inputs, 11, 1);
    refused(&mut inputs, &r, G, &[])?;
    inputs.advance_islands_with_placement(G, &[], &stay(1))?;
    let full = (0..256).map(|i| body(i + 1, 4. * i as f64)).collect();
    let mut ids = spawn(vec![vec![body(1001, -9.)], full, vec![]], 32)?;
    let r = movement(&ids, 1001, 1);
    refused(&mut ids, &r, G, &[])?;
    ids.advance_islands_with_placement(G, &[], &stay(1))?;
    Ok(())
}
#[test]
fn actual_field_and_queued_approaches_override_an_optional_record_but_never_mandatory_routing()
-> Test {
    // The 0.10 m face gap is cold-admissible; a queued center impulse or
    // sustained force closes it within the upcoming 10 ms public interval.
    for (split, use_impulse) in [(false, false), (false, true), (true, false), (true, true)] {
        let a = body(11, 0.);
        let b = body(12, 0.70);
        let workers = if split {
            vec![vec![a], vec![b], vec![], vec![]]
        } else {
            vec![vec![a, b], vec![], vec![], vec![]]
        };
        let mut p = spawn(workers, 48)?;
        let candidate = movement(&p, 11, 2);
        let field = BodyWrench {
            id: 11,
            force: [5000., 0., 0.],
            torque: [0.; 3],
        };
        let loads = if use_impulse {
            p.queue_impulse(11, [24., 0., 0.])?;
            &[][..]
        } else {
            std::slice::from_ref(&field)
        };
        assert!(matches!(
            p.advance_islands_with_placement(G, loads, &candidate),
            Err(Error::Colocation)
        ));
        refused(&mut p, &candidate, G, loads)?;
        p.advance_islands_with_placement(G, loads, &stay(1))?;
        assert_eq!(p.frame().body(11).unwrap().0, p.frame().body(12).unwrap().0);
        assert_eq!(p.last_island_placement(), Some(&stay(1)));
    }
    Ok(())
}
#[test]
fn a_lost_export_or_final_reply_fences_the_entire_original_replay_until_commit_or_rollback() -> Test
{
    for final_reply in [false, true] {
        let mut p = simple(32)?;
        let mut control = simple(32)?;
        let dir = tempfile::tempdir()?;
        let archive = dir.path().join("step.gzpool");
        let r = movement(&p, 11, 1);
        let before = p.frame().clone();
        let field = [BodyWrench {
            id: 11,
            force: [0., 1., 0.],
            torque: [0.; 3],
        }];
        let lost = if final_reply { 3 } else { 0 };
        p.drop_next_acknowledgement(lost)?;
        assert!(matches!(
            p.advance_islands_with_placement(G, &field, &r),
            Err(Error::Transport(_))
        ));
        assert_eq!(p.frame(), &before);
        assert!(p.last_island_placement().is_none());
        let after = counters(&p);
        assert!(matches!(
            p.advance_islands_with_placement(G, &field, &stay(1)),
            Err(Error::Pending)
        ));
        assert!(matches!(
            p.advance_islands_with_placement([-0., 0., 0.], &field, &r),
            Err(Error::Pending)
        ));
        assert!(matches!(
            p.advance_islands_with_placement(G, &[], &r),
            Err(Error::Pending)
        ));
        assert!(matches!(
            p.advance_islands_in_field(G, &field),
            Err(Error::Pending)
        ));
        assert!(matches!(
            p.queue_impulse(11, [1., 0., 0.]),
            Err(Error::Pending)
        ));
        assert!(p.save_native_recovery(&archive).is_err());
        assert!(!archive.exists());
        assert_eq!(counters(&p), after);
        let pid = p.worker_pid(1);
        p.terminate_worker(1)?;
        if !final_reply {
            p.recover(1)?;
            assert!(matches!(
                p.advance_islands_with_placement(G, &field, &stay(1)),
                Err(Error::Pending)
            ));
        }
        p.advance_islands_with_placement(G, &field, &r)?;
        control.advance_islands_with_placement(G, &field, &r)?;
        assert_ne!(p.worker_pid(1), pid);
        assert_eq!(p.frame(), control.frame());
        assert_eq!(p.last_island_placement(), Some(&r));
        assert_eq!(p.frame().workers[1].generations[&11], 1);
        p.save_native_recovery(&archive)?;
        let mut restored = PoolNativeRecovery::open(&archive)?.resume(exe())?;
        assert_eq!(restored.frame(), p.frame());
        assert!(restored.last_island_placement().is_none());
        let back = movement(&restored, 11, 0);
        restored.drop_next_acknowledgement(3)?;
        assert!(
            restored
                .advance_islands_with_placement(G, &[], &back)
                .is_err()
        );
        assert!(restored.rollback_interrupted_placement()?);
        assert!(!restored.rollback_interrupted_placement()?);
        restored.advance_islands_with_placement(G, &[], &stay(2))?;
        control.advance_islands_with_placement(G, &[], &stay(2))?;
        assert_eq!(restored.frame(), control.frame());
    }
    Ok(())
}
#[test]
fn explicit_no_move_bypasses_live_measurements_and_interrupts_prior_automatic_windows() -> Test {
    let mut p = simple(64)?;
    p.set_periodic_placement(Some("2,2,64,0".parse()?))?;
    p.set_local_helper_limit(Some(0));
    for tick in 1..=8 {
        p.advance_islands_with_placement(G, &[], &stay(tick))?;
    }
    let status = p.placement_status().unwrap();
    assert_eq!(
        (
            status.windows_started,
            status.windows_completed,
            status.committed_moves
        ),
        (0, 0, 0)
    );
    // An advisory 25 ms probe may legitimately time out on a busy host. Verify an
    // explicit record interrupts a *real* window, retrying only the documented timeout.
    let parallel = std::thread::available_parallelism()?.get() > 1;
    for attempt in 0..16 {
        let previously_unavailable = p.placement_status().unwrap().unavailable_probes;
        p.advance_islands_in_field(G, &[])?;
        let status = p.placement_status().unwrap();
        if !parallel || status.windows_started == 1 {
            break;
        }
        assert_eq!(status.unavailable_probes, previously_unavailable + 1);
        assert!(
            attempt < 15,
            "no advisory sample opened in 16 eligible periods"
        );
        p.advance_islands_with_placement(G, &[], &stay(p.frame().tick + 1))?;
    }
    assert_eq!(
        p.placement_status().unwrap().windows_started,
        u64::from(parallel)
    );
    p.advance_islands_with_placement(G, &[], &stay(p.frame().tick + 1))?;
    let status = p.placement_status().unwrap();
    assert_eq!(status.windows_completed, 0);
    assert_eq!(status.windows_interrupted, u64::from(parallel));
    Ok(())
}

#[test]
fn encoded_records_require_an_explicit_decision_and_bound_sorted_membership() -> Test {
    let stay = stay(1);
    for bytes in [
        r#"{}"#,
        r#"{"target_tick":1}"#,
        r#"{"target_tick":1,"optional_move":null,"unknown":true}"#,
    ] {
        assert!(serde_json::from_str::<PlacementRecord>(bytes).is_err());
    }
    assert_eq!(
        serde_json::from_slice::<PlacementRecord>(&serde_json::to_vec(&stay)?)?,
        stay
    );
    assert_eq!(
        bincode::deserialize::<PlacementRecord>(&bincode::serialize(&stay)?)?,
        stay
    );
    let mut move64 = PlacementRecord {
        target_tick: 1,
        optional_move: Some(PlacementMove {
            source: 0,
            destination: 1,
            members: (1..=64)
                .map(|id| PlacementMember { id, generation: 0 })
                .collect(),
        }),
    };
    for encode in [false, true] {
        let roundtrip = |rec: &PlacementRecord| -> Result<PlacementRecord, Box<dyn StdError>> {
            if encode {
                Ok(bincode::deserialize(&bincode::serialize(rec)?)?)
            } else {
                Ok(serde_json::from_slice(&serde_json::to_vec(rec)?)?)
            }
        };
        assert_eq!(roundtrip(&move64)?, move64);
        let mut malformed = move64.clone();
        malformed
            .optional_move
            .as_mut()
            .unwrap()
            .members
            .push(PlacementMember {
                id: 65,
                generation: 0,
            });
        assert!(roundtrip(&malformed).is_err());
        malformed
            .optional_move
            .as_mut()
            .unwrap()
            .members
            .truncate(2);
        malformed.optional_move.as_mut().unwrap().members.reverse();
        assert!(roundtrip(&malformed).is_err());
        malformed.optional_move.as_mut().unwrap().members.clear();
        assert!(roundtrip(&malformed).is_err());
    }
    let mut pool = WorkerPool::spawn_canonical_scene(
        exe(),
        92159,
        (1..=64).map(|id| body(id, id as f64)).collect(),
        4,
        SimulationStep::Ms10,
        None,
        32,
    )?;
    pool.advance_islands_with_placement(G, &[], &move64)?;
    assert_eq!(pool.last_island_placement(), Some(&move64));
    assert_eq!(
        pool.frame().workers[1]
            .generations
            .values()
            .copied()
            .collect::<Vec<_>>(),
        vec![1; 64]
    );
    move64.target_tick = 2;
    refused(&mut pool, &move64, G, &[])?;
    Ok(())
}

fn sampling_after_replayed_window(fresh: bool, parallel: bool) -> Result<bool, Box<dyn StdError>> {
    let mut p = simple(64)?;
    let config = "4,8,1,4294967295".parse()?;
    p.set_periodic_placement(Some(config))?;
    p.set_local_helper_limit(Some(0));
    p.advance_islands_in_field(G, &[])?;
    for t in 2..=4 {
        p.advance_islands_with_placement(G, &[], &stay(t))?;
    }
    let status = p.placement_status().unwrap();
    if parallel && status.unavailable_probes == 1 {
        assert_eq!((status.windows_started, status.windows_interrupted), (0, 0));
        return Ok(false);
    }
    assert_eq!(
        (
            status.windows_started,
            status.windows_completed,
            status.windows_interrupted
        ),
        (u64::from(parallel), 0, u64::from(parallel))
    );
    if fresh {
        let dir = tempfile::tempdir()?;
        let file = dir.path().join("mid-interrupted-sample.gzpool");
        let old = p.worker_pid(0);
        p.save_native_recovery(&file)?;
        drop(p);
        p = PoolNativeRecovery::open(&file)?.resume(exe())?;
        assert_ne!(old, p.worker_pid(0));
        assert!(p.placement_status().is_none());
        p.set_periodic_placement(Some(config))?;
        p.set_local_helper_limit(Some(0));
    }
    for t in 5..=8 {
        p.advance_islands_with_placement(G, &[], &stay(t))?;
    }
    let old = p.placement_status().unwrap().clone();
    for t in 9..=13 {
        p.advance_islands_in_field(G, &[])?;
        assert_eq!(p.last_island_placement(), Some(&stay(t)));
        if t == 9 {
            let status = p.placement_status().unwrap();
            if parallel && status.unavailable_probes == old.unavailable_probes + 1 {
                assert_eq!(status.windows_started, old.windows_started);
                return Ok(false);
            }
            assert_eq!(
                p.placement_status().unwrap().windows_started,
                old.windows_started + u64::from(parallel)
            );
        }
    }
    let new = p.placement_status().unwrap();
    if parallel && new.unavailable_probes == old.unavailable_probes + 1 {
        assert_eq!(new.windows_completed, 0);
        return Ok(false);
    }
    assert_eq!(
        (
            new.windows_completed,
            new.windows_interrupted,
            new.committed_moves
        ),
        (u64::from(parallel), old.windows_interrupted, 0)
    );
    Ok(true)
}

#[test]
fn fresh_and_same_session_sampling_restart_on_cadence_after_a_replayed_window() -> Test {
    let parallel = std::thread::available_parallelism()?.get() > 1;
    for fresh in [false, true] {
        // An advisory read may expire. Start a fresh trial only after the pool reports
        // the unavailable probe, and require a real uninterrupted sample on this schedule.
        for attempt in 1..=16 {
            if sampling_after_replayed_window(fresh, parallel)? {
                break;
            }
            assert!(attempt < 16, "advisory sample repeatedly timed out");
        }
    }
    Ok(())
}

#[test]
fn retired_members_are_refused_and_an_independent_move_survives_a_same_tick_birth_and_durable_restore()
-> Test {
    let mut p = simple(32)?;
    let deleted = movement(&p, 13, 3);
    let retained = movement(&p, 11, 1);
    p.retire(13)?;
    p.insert_body(body(14, 25.))?;
    let dir = tempfile::tempdir()?;
    let path = dir.path().join("edited.gzpool");
    p.save_native_recovery(&path)?;
    drop(p);
    let mut p = PoolNativeRecovery::open(&path)?.resume(exe())?;
    refused(&mut p, &deleted, G, &[])?;
    assert!(p.insert_body(body(13, 30.)).is_err());
    p.advance_islands_with_placement(G, &[], &retained)?;
    assert_eq!(p.frame().body(11).unwrap().0, 1);
    assert!(p.frame().body(13).is_none());
    assert!(p.frame().body(14).is_some());
    Ok(())
}

#[test]
fn loaded_ordered_handoffs_replay_every_lost_reply_and_peer_replacement_in_both_encodings() -> Test
{
    use gz_physics_distributed::physics::GroundPlane;
    let gravity = [0., 0., -9.81];
    let slab = Body {
        position: [0., 0., 0.205],
        size: [2., 2., 0.4],
        primitive: Primitive::Box,
        ..body(11, 0.)
    };
    let cargo = Body {
        position: [0., 0., 0.765],
        ..body(12, 0.)
    };
    let neighbor = Body {
        position: [64., 0., 0.3],
        ..body(13, 64.)
    };
    let mut base = WorkerPool::spawn(
        exe(),
        92160,
        vec![vec![slab, cargo], vec![neighbor], vec![], vec![]],
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        128,
    )?;
    for _ in 0..50 {
        base.advance_islands_in_field(gravity, &[])?;
    }
    let warm = &base.frame().workers[0];
    assert!(
        warm.snapshot
            .impulses
            .iter()
            .any(|r| r.a == 11 && r.b == 12 && r.normal_impulse > 0.)
    );
    assert!(
        warm.snapshot
            .supports
            .iter()
            .any(|r| r.body == 11 && r.normal_impulse > 0.)
    );
    let field = [BodyWrench {
        id: 12,
        force: [0., 0., -1.],
        torque: [0.; 3],
    }];
    // The incoming impulses for the slab and cargo straddle a resident
    // impulse on the destination. Warm preparation must preserve that order.
    for (id, magnitude) in [(12, 0.003), (13, 0.004), (11, -0.002)] {
        base.queue_impulse_in_field(id, [magnitude, 0., 0.], gravity, &field)?;
    }
    let record = movement(&base, 12, 1);
    assert_eq!(record.optional_move.as_ref().unwrap().members.len(), 2);
    let dir = tempfile::tempdir()?;
    let archive = dir.path().join("ordered-warm.gzpool");
    base.save_native_recovery(&archive)?;
    let initial = bincode::serialize(&(base.frame().tick, &base.frame().workers))?;
    drop(base);
    let bytes = |p: &WorkerPool| {
        bincode::serialize(&(
            p.frame().tick,
            &p.frame().workers,
            p.last_substeps(),
            p.last_island_refined_bodies(),
        ))
    };
    let mut reference = PoolNativeRecovery::open(&archive)?.resume(exe())?;
    reference.advance_islands_with_placement(gravity, &field, &record)?;
    assert!(
        reference.frame().workers[1]
            .snapshot
            .impulses
            .iter()
            .any(|r| r.a == 11 && r.b == 12 && r.normal_impulse > 0.)
    );
    let transferred = bytes(&reference)?;
    reference.advance_islands_with_placement(gravity, &[], &stay(52))?;
    let following = bytes(&reference)?;
    drop(reference);
    // Source commands are export, release, observe, step. Receiver commands
    // are ordered prepare, activate, step. The other two peers only step.
    for packed in [false, true] {
        for (affected, earlier) in [
            (0, 0),
            (1, 0),
            (0, 1),
            (1, 1),
            (0, 2),
            (0, 3),
            (1, 2),
            (2, 0),
            (3, 0),
        ] {
            let mut p = PoolNativeRecovery::open(&archive)?.resume(exe())?;
            assert_eq!(p.set_packed_handoffs(packed), if packed { 4 } else { 0 });
            p.drop_acknowledgement_after(affected, earlier)?;
            assert!(
                matches!(
                    p.advance_islands_with_placement(gravity, &field, &record),
                    Err(Error::Transport(_))
                ),
                "case {packed} {affected} {earlier}"
            );
            assert_eq!(
                bincode::serialize(&(p.frame().tick, &p.frame().workers))?,
                initial
            );
            assert!(p.last_island_placement().is_none());
            assert!(matches!(
                p.advance_islands_with_placement(gravity, &field, &stay(51)),
                Err(Error::Pending)
            ));
            // Kill the other handoff participant, even if its last reply was seen.
            let killed = u8::from(affected == 0);
            let pid = p.worker_pid(killed);
            p.terminate_worker(killed)?;
            if (affected, earlier) == (0, 0) {
                // The failed export never touched the receiver. With raw handoff
                // supervision disabled, its subsequent death is a second real
                // exchange failure, which must preserve the same retry fence.
                assert!(matches!(
                    p.advance_islands_with_placement(gravity, &field, &record),
                    Err(Error::Transport(_))
                ));
                assert_eq!(
                    bincode::serialize(&(p.frame().tick, &p.frame().workers))?,
                    initial
                );
                assert!(matches!(
                    p.advance_islands_with_placement(gravity, &field, &stay(51)),
                    Err(Error::Pending)
                ));
            }
            p.advance_islands_with_placement(gravity, &field, &record)?;
            assert_ne!(p.worker_pid(killed), pid);
            assert_eq!(p.last_island_placement(), Some(&record));
            assert_eq!(
                bytes(&p)?,
                transferred,
                "committed case {packed} {affected} {earlier}"
            );
            p.advance_islands_with_placement(gravity, &[], &stay(52))?;
            assert_eq!(bytes(&p)?, following);
        }
    }
    Ok(())
}
