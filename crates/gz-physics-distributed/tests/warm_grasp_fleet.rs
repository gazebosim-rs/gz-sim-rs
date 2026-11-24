// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::floating_tree::{
    AdaptiveGraspFleetPool, CoupledGraspFleetPool, GraspFleetMerge, GraspFleetSpec,
};
#[allow(dead_code)]
#[path = "support/grasp_fleet_join.rs"]
mod shared;
use shared::*;
use std::{fs, path::Path, time::Instant};

fn held_inputs(s: &GraspFleetSpec, t: u64) -> Vec<(u64, [f64; 3])> {
    if (650..1980).contains(&t) {
        forces(s, 3., 0.)
    } else if t >= 1980 {
        forces(s, -3., 0.)
    } else {
        Vec::new()
    }
}
fn closed(s: &GraspFleetSpec) -> impl Iterator<Item = (u64, gz_physics::JointVelocityMotor)> + '_ {
    (0..2).flat_map(|i| motor(s, i, false))
}
#[test]
fn two_held_member_packets_form_real_foreign_contact_without_replay() -> Result<()> {
    let xml = held();
    let s = recipe(&xml)?;
    let a = tempfile::tempdir()?;
    let b = tempfile::tempdir()?;
    let mut p = AdaptiveGraspFleetPool::create_with_policy(
        exe(),
        a.path(),
        3,
        s.clone(),
        GraspFleetMerge::WarmMembers,
    )?;
    let mut q = CoupledGraspFleetPool::create(exe(), b.path(), 3, s.clone())?;
    assert_eq!(p.merge_policy(), GraspFleetMerge::WarmMembers);
    for (j, m) in closed(&s) {
        p.set_velocity_motor(j, Some(m))?;
        q.set_velocity_motor(j, Some(m))?;
    }
    let j = s.assemblies[0].payload.tree.joints[0].joint_id;
    let mut killed = 0;
    let mut foreign = 0;
    let mut released = 0;
    let mut joined = 0;
    let mut exact_images = 0;
    let mut timing = None;
    let mut digest = receipt_digest::Trail::new();
    for t in 0..2700 {
        if t == 73 || t == 738 || t == 1250 || t == 1460 {
            let dest = (p.owners()[0] + 1) % 3;
            kill(
                p.worker_pid(if killed % 2 == 0 { p.owners()[0] } else { dest })
                    .unwrap(),
            )?;
            p.transfer(s.assemblies[0].payload.tree.model_id, dest)?;
            drop(p);
            p = AdaptiveGraspFleetPool::resume(exe(), a.path())?;
            assert_eq!(p.merge_policy(), GraspFleetMerge::WarmMembers);
            killed += 1;
        }
        let f = held_inputs(&s, t);
        if (650..=999).contains(&t) {
            p.queue_effort(j, 0.0002)?;
            q.queue_effort(j, 0.0002)?;
        }
        let dest = std::env::var_os("GZ_WARM_GRASP_WITNESS");
        if t == 999
            && let Some(d) = &dest
        {
            let d = Path::new(d);
            fs::create_dir_all(d)?;
            fs::copy(a.path().join(FILE), d.join("prior.gzadaptivegraspfleet"))?;
            fs::write(d.join("scene.sdf"), &xml)?;
        }
        p.advance_with_inputs_at(0, t, &[], &f)?;
        q.advance_with_inputs_at(0, t, &[], &f)?;
        exact(&p, &q, false);
        if p.is_coupled() {
            let m = s.assemblies[0].payload.tree.model_id;
            let img = p.image(m).unwrap();
            assert!(img.starts_with(b"GZFCPL02"));
            digest.add(img);
            exact_images += usize::from(Some(img) == q.image());
            joined += 1;
            if p.joined_at() == Some(t) {
                assert_eq!(t, 999);
                timing = p.member_timing();
                if let Some(d) = &dest {
                    fs::copy(
                        a.path().join(FILE),
                        Path::new(d).join("joined.gzadaptivegraspfleet"),
                    )?;
                }
                assert!(p.advance_with_inputs_at(0, t, &[], &f).is_err());
            }
            if p.cross_reactions()
                .iter()
                .any(|r| r.reaction.normal_impulse > 0. && r.reaction.warmstart_normal_impulse > 0.)
            {
                assert!(s.assemblies.iter().all(|s| {
                    p.payload_reactions(s.payload.tree.model_id)
                        .unwrap()
                        .iter()
                        .filter(|r| r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
                        .count()
                        >= 2
                }));
                foreign += 1;
            } else if foreign > 0 {
                released += 1;
            }
        }
    }
    let timing = timing.ok_or("missing real direct promotion")?;
    assert_eq!(killed, 4);
    assert!(foreign >= 600 && released >= 600 && joined >= 1600);
    eprintln!(
        "direct_pair: joined_from=999 complete_public_ordinary_controls=2700 native_ordinary_bytes_equal={exact_images}/{joined} first_and_last_loaded_contacts={foreign} released={released} four_process_faults={killed} packet_bytes={:?} reused_sources={} export_us={} rebuild_install_us={} first_worker_interval_us={} own_native_file_chain={}",
        timing.source_packet_bytes,
        timing.reused_source_workers,
        timing.export_us,
        timing.install_us,
        timing.first_step_us,
        digest.finish()
    );
    p.reset()?;
    assert_eq!(p.epoch(), 1);
    assert!(!p.is_coupled());
    assert!(p.member_timing().is_none());
    Ok(())
}
mod receipt_digest {
    use sha2::{Digest, Sha256};
    pub struct Trail(Sha256);
    impl Trail {
        pub fn new() -> Self {
            Self(Sha256::new())
        }
        pub fn add(&mut self, b: &[u8]) {
            self.0.update((b.len() as u64).to_le_bytes());
            self.0.update(b);
        }
        pub fn finish(self) -> String {
            format!("{:x}", self.0.finalize())
        }
    }
}

#[test]
#[ignore = "paired costs on the same host, source state and active workpiece contacts"]
fn benchmark_direct_verified_and_pinned_on_identical_four_contact_approach() -> Result<()> {
    let d = std::env::var_os("GZ_WARM_GRASP_TIMING_DIR").ok_or("set timing output directory")?;
    let out = Path::new(&d);
    fs::create_dir_all(out)?;
    let xml = held();
    let s = recipe(&xml)?;
    let j = s.assemblies[0].payload.tree.joints[0].joint_id;
    let direct = tempfile::tempdir()?;
    let replay = tempfile::tempdir()?;
    let pinned = tempfile::tempdir()?;
    let mut a = AdaptiveGraspFleetPool::create_with_policy(
        exe(),
        direct.path(),
        3,
        s.clone(),
        GraspFleetMerge::WarmMembers,
    )?;
    let mut b =
        AdaptiveGraspFleetPool::create_with_replay(exe(), replay.path(), 3, s.clone(), budget())?;
    let mut c = CoupledGraspFleetPool::create(exe(), pinned.path(), 3, s.clone())?;
    for (j, m) in closed(&s) {
        a.set_velocity_motor(j, Some(m))?;
        b.set_velocity_motor(j, Some(m))?;
        c.set_velocity_motor(j, Some(m))?;
    }
    for t in 0..999 {
        if t >= 650 {
            a.queue_effort(j, 0.0002)?;
            b.queue_effort(j, 0.0002)?;
            c.queue_effort(j, 0.0002)?;
        }
        let f = held_inputs(&s, t);
        a.advance_with_inputs(&[], &f)?;
        b.advance_with_inputs(&[], &f)?;
        c.advance_with_inputs(&[], &f)?;
    }
    a.queue_effort(j, 0.0002)?;
    b.queue_effort(j, 0.0002)?;
    c.queue_effort(j, 0.0002)?;
    let sizes = [
        fs::metadata(direct.path().join(FILE))?.len(),
        fs::metadata(replay.path().join(FILE))?.len(),
        fs::metadata(pinned.path().join("barrier.gzgraspcluster"))?.len(),
    ];
    drop((a, b, c));
    let mut csv = String::from(
        "repeat,profile,tick,resume_us,total_step_us,source_export_or_replay_us,install_us,first_step_us,source_archive_bytes,accepted_archive_bytes\n",
    );
    for rep in 0..16 {
        for offset in 0..3 {
            let which = (rep + offset) % 3;
            let (dir, file, label) = match which {
                0 => (direct.path(), FILE, "direct"),
                1 => (replay.path(), FILE, "replay"),
                _ => (pinned.path(), "barrier.gzgraspcluster", "pinned"),
            };
            let tmp = tempfile::tempdir()?;
            fs::copy(dir.join(file), tmp.path().join(file))?;
            let start = Instant::now();
            if which < 2 {
                let mut p = AdaptiveGraspFleetPool::resume(exe(), tmp.path())?;
                let resume = start.elapsed().as_micros();
                for t in 999..1019 {
                    let f = held_inputs(&s, t);
                    let t0 = Instant::now();
                    p.advance_with_inputs(&[], &f)?;
                    let total = t0.elapsed().as_micros();
                    let (prepare, install, solve) = if t != 999 {
                        (0, 0, 0)
                    } else if which == 0 {
                        let stats = p.member_timing().unwrap();
                        (stats.export_us, stats.install_us, stats.first_step_us)
                    } else {
                        let stats = p.replay_timing().unwrap();
                        (stats.replay_us, stats.install_us, stats.first_step_us)
                    };
                    csv.push_str(&format!(
                        "{rep},{label},{t},{resume},{total},{prepare},{install},{solve},{},{}\n",
                        sizes[which],
                        fs::metadata(tmp.path().join(file))?.len()
                    ));
                }
            } else {
                let mut p = CoupledGraspFleetPool::resume(exe(), tmp.path())?;
                let resume = start.elapsed().as_micros();
                for t in 999..1019 {
                    let f = held_inputs(&s, t);
                    let t0 = Instant::now();
                    p.advance_with_inputs(&[], &f)?;
                    let total = t0.elapsed().as_micros();
                    csv.push_str(&format!(
                        "{rep},{label},{t},{resume},{total},0,0,0,{},{}\n",
                        sizes[which],
                        fs::metadata(tmp.path().join(file))?.len()
                    ));
                }
            }
        }
    }
    fs::write(out.join("loaded-paired-us.csv"), csv)?;
    eprintln!(
        "960 paired direct/replay/full-world promotion and steady observations, prior byte counts {sizes:?}"
    );
    Ok(())
}

#[test]
#[ignore = "paired previous-v2 vs proof-capable process comparison using the exact accepted four-contact archive"]
fn proof_export_matches_original_worker_bytes_and_measures_process_reuse() -> Result<()> {
    let worker = std::env::var_os("GZ_WARM_BASELINE_WORKER").ok_or("set original v2 executable")?;
    let older = Path::new(&worker);
    let src = std::env::var_os("GZ_WARM_GRASP_PRIOR").ok_or("set prior native archive")?;
    let dst = std::env::var_os("GZ_PROOF_GRASP_TIMING_DIR").ok_or("set output directory")?;
    let out = Path::new(&dst);
    fs::create_dir_all(out)?;
    let mut rows = String::from(
        "repeat,profile,tick,resume_us,step_us,export_us,install_us,first_solve_us,reused\n",
    );
    for rep in 0..48 {
        let a = tempfile::tempdir()?;
        let b = tempfile::tempdir()?;
        for d in [&a, &b] {
            fs::copy(&src, d.path().join(FILE))?;
        }
        let run = |exe: &Path, d: &Path, reused: bool, rows: &mut String| -> Result<Vec<Vec<u8>>> {
            let start = Instant::now();
            let mut p = AdaptiveGraspFleetPool::resume(exe, d)?;
            let resume = start.elapsed().as_micros();
            let mut imgs = Vec::new();
            for t in 999..1015 {
                let f = held_inputs(p.spec(), t);
                let t0 = Instant::now();
                p.advance_with_inputs_at(0, t, &[], &f)?;
                let time = t0.elapsed().as_micros();
                let (export, install, solve) = if t == 999 {
                    let s = p.member_timing().unwrap();
                    assert_eq!(s.reused_source_workers, reused);
                    (s.export_us, s.install_us, s.first_step_us)
                } else {
                    (0, 0, 0)
                };
                imgs.push(fs::read(d.join(FILE))?);
                let name = if reused { "proof" } else { "original" };
                rows.push_str(&format!(
                    "{rep},{name},{t},{resume},{time},{export},{install},{solve},{reused}\n"
                ));
            }
            Ok(imgs)
        };
        let (new, old) = if rep % 2 == 0 {
            let n = run(exe(), a.path(), true, &mut rows)?;
            (n, run(older, b.path(), false, &mut rows)?)
        } else {
            let o = run(older, b.path(), false, &mut rows)?;
            (run(exe(), a.path(), true, &mut rows)?, o)
        };
        assert_eq!(new.len(), old.len());
        for (i, (n, o)) in new.iter().zip(old).enumerate() {
            assert!(n == &o, "full hydrated archive differed after {}", 999 + i);
        }
    }
    fs::write(out.join("paired-proof-us.csv"), rows)?;
    eprintln!(
        "48 alternated promotions from old V3 archive, 768 pairs of complete old/new graph files matched exactly, negotiated prior export true on new worker only"
    );
    Ok(())
}

#[test]
fn one_loaded_and_one_clear_worker_match_both_mixed_orientations() -> Result<()> {
    for clear in 0..2 {
        let xml = if clear == 0 {
            held().replace("0.317 -0.516", "2.0 -0.516")
        } else {
            held().replace("0.317 0.524", "2.0 0.524")
        };
        let s = recipe(&xml)?;
        let a = tempfile::tempdir()?;
        let b = tempfile::tempdir()?;
        let mut p = AdaptiveGraspFleetPool::create_with_policy(
            exe(),
            a.path(),
            3,
            s.clone(),
            GraspFleetMerge::WarmMembers,
        )?;
        let mut q = CoupledGraspFleetPool::create(exe(), b.path(), 3, s.clone())?;
        for (j, m) in closed(&s) {
            p.set_velocity_motor(j, Some(m))?;
            q.set_velocity_motor(j, Some(m))?;
        }
        let mut warm = 0;
        let mut first = None;
        let mut moved = false;
        let mut sizes = None;
        for t in 0..2200 {
            let f = held_inputs(&s, t);
            p.advance_with_inputs_at(0, t, &[], &f)?;
            q.advance_with_inputs_at(0, t, &[], &f)?;
            exact(&p, &q, false);
            if p.joined_at() == Some(t) {
                first = Some(t);
                sizes = Some(p.member_timing().unwrap().source_packet_bytes);
                let by_group = s
                    .assemblies
                    .iter()
                    .map(|g| {
                        p.payload_reactions(g.payload.tree.model_id)
                            .unwrap()
                            .iter()
                            .filter(|r| r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
                            .count()
                    })
                    .collect::<Vec<_>>();
                assert_eq!(by_group[clear], 0);
                assert!(
                    by_group[1 - clear] >= 1,
                    "retained source has no loaded contact at adaptive tick {t}"
                );
                eprintln!("adaptive mixed cut {t}: actually loaded contact count {by_group:?}");
            }
            if p.is_coupled() && !moved {
                kill(p.worker_pid(p.owners()[0]).unwrap())?;
                let dst = (p.owners()[0] + 1) % 3;
                p.transfer(s.assemblies[0].payload.tree.model_id, dst)?;
                drop(p);
                p = AdaptiveGraspFleetPool::resume(exe(), a.path())?;
                moved = true;
            }
            if p.is_coupled()
                && p.cross_reactions().iter().any(|r| {
                    r.reaction.normal_impulse > 0. && r.reaction.warmstart_normal_impulse > 0.
                })
            {
                warm += 1;
            }
        }
        assert!(first.is_some() && moved);
        eprintln!(
            "clear_source={clear} joined_at={first:?} source_bytes={sizes:?} ordinary_physical_frames=2200 recovered_after_active_owner_kill=true foreign_warm={warm}"
        );
    }
    Ok(())
}
