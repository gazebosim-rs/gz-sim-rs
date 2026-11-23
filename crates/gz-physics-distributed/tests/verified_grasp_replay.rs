// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    articulation::ArticulationError,
    floating_tree::{AdaptiveGraspFleetPool, CoupledGraspFleetPool, GraspReplayBudget},
};
use std::{fs, path::Path, time::Instant};
#[path = "support/grasp_fleet_join.rs"]
mod shared;
use shared::*;
#[test]
fn bounded_replay_joins_two_already_loaded_grippers_without_losing_warm_contact() -> Result<()> {
    let xml = held();
    let s = recipe(&xml)?;
    let a = tempfile::tempdir()?;
    let b = tempfile::tempdir()?;
    let mut p =
        AdaptiveGraspFleetPool::create_with_replay(exe(), a.path(), 3, s.clone(), budget())?;
    let mut q = CoupledGraspFleetPool::create(exe(), b.path(), 3, s.clone())?;
    for i in 0..2 {
        for (j, m) in motor(&s, i, false) {
            p.set_velocity_motor(j, Some(m))?;
            q.set_velocity_motor(j, Some(m))?;
        }
    }
    let j = s.assemblies[0].payload.tree.joints[0].joint_id;
    let mut moves = 0;
    let mut warm = 0;
    let mut join = None;
    let mut joined_steps = 0;
    let mut release = 0;
    let mut peak_before = 0;
    let mut first_timing = None;
    for t in 0..2700 {
        let f = if (650..1980).contains(&t) {
            forces(&s, 3., 0.)
        } else if t >= 1980 {
            forces(&s, -3., 0.)
        } else {
            vec![]
        };
        if t >= 650 && join.is_none() {
            p.queue_effort(j, 0.0002)?;
            q.queue_effort(j, 0.0002)?;
        }
        if t == 73 || t == 738 {
            let dest = (p.owners()[1] + 1) % 3;
            kill(
                p.worker_pid(if t == 73 { p.owners()[1] } else { dest })
                    .unwrap(),
            )?;
            p.transfer(s.assemblies[1].payload.tree.model_id, dest)?;
            drop(p);
            p = AdaptiveGraspFleetPool::resume(exe(), a.path())?;
            assert_eq!(p.replay_budget(), Some(budget()));
            moves += 1;
        }
        if t == 900 {
            p.drop_next_acknowledgement(p.owners()[1])?;
        }
        if t == 999
            && let Some(d) = std::env::var_os("GZ_VERIFIED_GRASP_WITNESS")
        {
            let d = Path::new(&d);
            fs::create_dir_all(d)?;
            fs::copy(a.path().join(FILE), d.join("prior.gzadaptivegraspfleet"))?;
            fs::write(d.join("scene.sdf"), &xml)?;
        }
        p.advance_with_inputs_at(0, t, &[], &f)?;
        q.advance_with_inputs_at(0, t, &[], &f)?;
        exact(&p, &q, true);
        let own = s
            .assemblies
            .iter()
            .map(|g| {
                p.payload_reactions(g.payload.tree.model_id)
                    .unwrap()
                    .iter()
                    .filter(|r| r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
                    .count()
            })
            .sum::<usize>();
        if p.is_coupled() {
            joined_steps += 1;
            if join.is_none() {
                join = p.joined_at();
                assert_eq!(join, Some(t));
                assert_eq!(own, 4);
                assert!(p.advance_with_inputs_at(0, t, &[], &f).is_err());
                first_timing = p.replay_timing();
                if let Some(d) = std::env::var_os("GZ_VERIFIED_GRASP_WITNESS") {
                    let d = Path::new(&d);
                    fs::copy(a.path().join(FILE), d.join("joined.gzadaptivegraspfleet"))?;
                }
            }
            if p.cross_reactions()
                .iter()
                .any(|r| r.reaction.normal_impulse > 0. && r.reaction.warmstart_normal_impulse > 0.)
            {
                assert_eq!(own, 4);
                warm += 1;
                if moves == 2 || (moves == 3 && t % 63 == 0) {
                    let dest = (p.owners()[0] + 1) % 3;
                    kill(
                        p.worker_pid(if moves == 2 { p.owners()[0] } else { dest })
                            .unwrap(),
                    )?;
                    p.transfer(s.assemblies[1].payload.tree.model_id, dest)?;
                    drop(p);
                    p = AdaptiveGraspFleetPool::resume(exe(), a.path())?;
                    exact(&p, &q, true);
                    moves += 1;
                }
            } else if warm > 0 {
                release += 1;
            }
        } else {
            peak_before = peak_before.max(fs::metadata(a.path().join(FILE))?.len());
        }
    }
    let timing = first_timing.expect("recorded private replay timing");
    eprintln!(
        "already-loaded verified replay input={join:?} events={} sim_verification_us={} destination_rebuild_us={} first_step_us={} exact_whole_native_frames_after_join={joined_steps} both_bilateral_plus_cross={warm} released_after_cross={release} source_history_peak_B={peak_before} managed_deaths_and_transfers={moves}",
        timing.events, timing.replay_us, timing.install_us, timing.first_step_us
    );
    assert!(warm >= 300 && release >= 300 && moves == 4);
    p.reset()?;
    assert_eq!(p.epoch(), 1);
    assert_eq!(p.replay_budget(), Some(budget()));
    assert!(!p.is_coupled());
    assert!(p.joined_at().is_none());
    assert!(p.replay_timing().is_none());
    Ok(())
}
#[test]
fn expired_provenance_refuses_only_the_uncertified_interval_after_resume() -> Result<()> {
    let s = recipe(&free())?;
    let d = tempfile::tempdir()?;
    let small = GraspReplayBudget {
        max_intervals: 24,
        max_events: 80,
    };
    let mut p = AdaptiveGraspFleetPool::create_with_replay(exe(), d.path(), 2, s.clone(), small)?;
    let j = s.assemblies[0].payload.tree.joints[0].joint_id;
    let f = forces(&s, 4., 0.);
    for t in 0..800 {
        p.queue_effort(j, 0.0002)?;
        let bytes = fs::read(d.path().join(FILE))?;
        match p.advance_with_inputs_at(0, t, &[], &f) {
            Ok(()) => (),
            Err(ArticulationError::GraspReplayExhausted {
                epoch,
                tick,
                recorded_intervals,
                recorded_events,
            }) => {
                assert_eq!((epoch, tick, recorded_intervals), (0, 500, 24));
                assert_eq!(tick, t);
                assert!(recorded_events <= small.max_events);
                assert_eq!(p.pending_joint_effort(j), Some(0.0002));
                assert_eq!(bytes, fs::read(d.path().join(FILE))?);
                drop(p);
                p = AdaptiveGraspFleetPool::resume(exe(), d.path())?;
                assert!(matches!(
                    p.advance_with_inputs_at(0, t, &[], &f),
                    Err(ArticulationError::GraspReplayExhausted { .. })
                ));
                assert_eq!(bytes, fs::read(d.path().join(FILE))?);
                return Ok(());
            }
            Err(e) => return Err(e.into()),
        }
    }
    panic!("missing expected replay refusal")
}
#[test]
fn divergent_history_names_first_different_physical_tick_and_does_not_publish() -> Result<()> {
    let s = recipe(XML)?;
    let d = tempfile::tempdir()?;
    let mut p =
        AdaptiveGraspFleetPool::create_with_replay(exe(), d.path(), 2, s.clone(), budget())?;
    let mut first_diff = None;
    for t in 0u64..3400 {
        let mut f = Vec::new();
        for i in 0..2 {
            let local = t as i64 - if i == 1 { 350 } else { 0 };
            let open = if i == 0 { 600 } else { 1100 };
            if local == 0 || local == open || (i == 0 && local == 1450) {
                for (j, m) in motor(&s, i, local == open) {
                    p.set_velocity_motor(j, Some(m))?;
                }
            }
            let z = if i == 0 {
                match local {
                    650..850 | 1250..1450 => 1.2,
                    850..1250 => -1.2,
                    _ => 0.,
                }
            } else if (650..1500).contains(&local) {
                1.2
            } else {
                0.
            };
            let y = if t >= 1600 {
                -8. * s.assemblies[i].payload.payload.position[1].signum()
            } else {
                0.
            };
            if [y, z] != [0.; 2] {
                f.push((s.assemblies[i].payload.payload.id, [0., y, z]));
            }
        }
        let before = fs::read(d.path().join(FILE))?;
        match p.advance_with_inputs_at(0, t, &[], &f) {
            Ok(()) => (),
            Err(ArticulationError::GraspReplayDiverged {
                epoch,
                tick,
                first_event,
                first_physical_tick,
            }) => {
                assert_eq!(epoch, 0);
                assert_eq!(tick, t);
                assert_eq!(first_physical_tick, 1012);
                assert!(first_event > 1012 && tick > 1600);
                assert_eq!(before, fs::read(d.path().join(FILE))?);
                drop(p);
                p = AdaptiveGraspFleetPool::resume(exe(), d.path())?;
                assert!(matches!(
                    p.advance_with_inputs_at(0, t, &[], &f),
                    Err(ArticulationError::GraspReplayDiverged {
                        first_physical_tick: 1012,
                        ..
                    })
                ));
                assert_eq!(before, fs::read(d.path().join(FILE))?);
                if let Some(d) = std::env::var_os("GZ_VERIFIED_GRASP_WITNESS") {
                    let d = Path::new(&d);
                    fs::create_dir_all(d)?;
                    fs::write(d.join("first-divergent.gzadaptivegraspfleet"), before)?;
                }
                eprintln!(
                    "would_join_tick={tick} independent_solver_first_diverged_at={first_physical_tick} event={first_event} old_global_file_preserved=true"
                );
                first_diff = Some(tick);
                break;
            }
            Err(e) => return Err(e.into()),
        }
    }
    assert!(first_diff.is_some());
    Ok(())
}

#[test]
#[ignore = "observed steady-state timing and bounded promotion cost need a matched host run"]
fn benchmark_clear_and_replay_on_identical_free_assembly_approach() -> Result<()> {
    let s = recipe(&free())?;
    let dst = std::env::var_os("GZ_REPLAY_TIMING_DIR").ok_or("supply GZ_REPLAY_TIMING_DIR")?;
    let dst = Path::new(&dst);
    fs::create_dir_all(dst)?;
    let mut csv = String::from("repeat,tick,clear_us,replay_us,phase\n");
    let mut total = Vec::new();
    for repeat in 0..8 {
        let a = tempfile::tempdir_in("/dev/shm")?;
        let b = tempfile::tempdir_in("/dev/shm")?;
        let mut clear = AdaptiveGraspFleetPool::create(exe(), a.path(), 2, s.clone())?;
        let mut replay =
            AdaptiveGraspFleetPool::create_with_replay(exe(), b.path(), 2, s.clone(), budget())?;
        let f = forces(&s, 4., 0.03);
        let j = s.assemblies[0].payload.tree.joints[0].joint_id;
        for t in 0..750 {
            let time = |p: &mut AdaptiveGraspFleetPool| -> Result<u128> {
                let start = Instant::now();
                p.advance_with_inputs_at(0, t, &[(j, 0.0002)], &f)?;
                Ok(start.elapsed().as_micros())
            };
            let (c, r) = if (repeat + t) % 2 == 0 {
                let c = time(&mut clear)?;
                (c, time(&mut replay)?)
            } else {
                let r = time(&mut replay)?;
                (time(&mut clear)?, r)
            };
            for g in &s.assemblies {
                let m = g.payload.tree.model_id;
                assert_eq!(clear.observation(m), replay.observation(m));
                assert_eq!(clear.payload_reactions(m), replay.payload_reactions(m));
            }
            assert_eq!(clear.cross_reactions(), replay.cross_reactions());
            let phase = if clear.joined_at() == Some(t) {
                "join"
            } else if clear.is_coupled() {
                "together"
            } else {
                "separate"
            };
            csv.push_str(&format!("{repeat},{t},{c},{r},{phase}\n"));
            if phase == "join" {
                let stats = replay.replay_timing().unwrap();
                total.push((c, r, stats.replay_us, stats.install_us, stats.first_step_us));
            }
        }
    }
    fs::write(dst.join("free-paired-us.csv"), csv)?;
    eprintln!(
        "eight real clear/replay promotions (clear_total_us,replay_total_us,replay_verify_us,restore_us,first_solve_and_receipt_us): {total:?}"
    );
    Ok(())
}
