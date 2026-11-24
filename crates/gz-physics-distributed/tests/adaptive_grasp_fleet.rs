// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics::JointVelocityMotor;
use gz_physics_distributed::{
    articulation::ArticulationError,
    floating_tree::{AdaptiveGraspFleetPool, CoupledGraspFleetPool, GraspFleetSpec},
};
use sha2::{Digest, Sha256};
use std::{error::Error, fs, path::Path, process::Command};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/distributed_grasp_fleet.sdf");
const FILE: &str = "barrier.gzadaptivegraspfleet";
fn exe() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn spec(x: &str) -> Result<GraspFleetSpec> {
    Ok(GraspFleetSpec {
        assemblies: gz_sim_scene::distributed_hinge::distributed_grasp_fleet_specs(
            x,
            "fleet.sdf",
            "default",
        )?,
    })
}
fn free() -> String {
    XML.replace("0.317 0.954 0.58", "2.0 0.954 0.58")
        .replace("0.317 -0.946 0.58", "2.0 -0.946 0.58")
}
fn yload(s: &GraspFleetSpec, a: f64, z: f64) -> Vec<(u64, [f64; 3])> {
    s.assemblies
        .iter()
        .map(|g| {
            let sign = -g.payload.payload.position[1].signum();
            (g.payload.payload.id, [0., a * sign, z * sign])
        })
        .collect()
}
fn kill(pid: u32) -> Result<()> {
    assert!(
        Command::new("kill")
            .args(["-KILL", &pid.to_string()])
            .status()?
            .success()
    );
    Ok(())
}
fn compare(p: &AdaptiveGraspFleetPool, q: &CoupledGraspFleetPool) -> Result<()> {
    assert_eq!((p.epoch(), p.tick()), (q.epoch(), q.tick()));
    for g in &q.spec().assemblies {
        let m = g.payload.tree.model_id;
        let a = p.observation(m).unwrap();
        let b = q.observation(m).unwrap();
        for (l, r) in a
            .links
            .iter()
            .chain([p.payload_state(m).unwrap()])
            .zip(b.links.iter().chain([q.payload_state(m).unwrap()]))
        {
            assert_eq!(
                l.position.map(f64::to_bits),
                r.position.map(f64::to_bits),
                "model {m} link {} position tick {}",
                l.id,
                p.tick()
            );
            assert_eq!(
                l.velocity.map(f64::to_bits),
                r.velocity.map(f64::to_bits),
                "model {m} link {} velocity tick {}",
                l.id,
                p.tick()
            );
            assert_eq!(
                l.orientation.map(f64::to_bits),
                r.orientation.map(f64::to_bits)
            );
            assert_eq!(
                l.angular_velocity.map(f64::to_bits),
                r.angular_velocity.map(f64::to_bits)
            );
        }
        assert_eq!(a, b);
        assert_eq!(p.payload_reactions(m), q.payload_reactions(m));
        for j in &g.payload.tree.joints {
            assert_eq!(p.velocity_motor(j.joint_id), q.velocity_motor(j.joint_id));
        }
    }
    if p.is_coupled() {
        assert_eq!(p.cross_reactions().len(), q.cross_reactions().len());
        for (actual, wanted) in p.cross_reactions().iter().zip(q.cross_reactions()) {
            assert_eq!(
                actual,
                wanted,
                "cross pair {}<-{} at tick {}",
                actual.first_body,
                actual.second_body,
                p.tick()
            );
        }
    } else {
        assert!(p.cross_reactions().is_empty());
        assert!(q.cross_reactions().iter().all(|r| !r.reaction.active));
    }
    Ok(())
}
#[test]
fn released_grasps_merge_before_oblique_impact_and_continue_through_physical_release() -> Result<()>
{
    let s = spec(&free())?;
    let a = tempfile::tempdir()?;
    let b = tempfile::tempdir()?;
    let ref_dir = tempfile::tempdir()?;
    let mut p = AdaptiveGraspFleetPool::create(exe(), a.path(), 3, s.clone())?;
    let mut clone = AdaptiveGraspFleetPool::create(exe(), b.path(), 3, s.clone())?;
    let mut q = CoupledGraspFleetPool::create(exe(), ref_dir.path(), 3, s.clone())?;
    assert!(!p.is_coupled());
    assert_eq!(p.owners(), [0, 1]);
    let joint = s.assemblies[0].payload.tree.joints[0].joint_id;
    let model = s.assemblies[0].payload.tree.model_id;
    let model_b = s.assemblies[1].payload.tree.model_id;
    let mut join = None;
    let mut warm = 0;
    let mut first = None;
    let mut moves = 0;
    let mut shear = 0f64;
    let mut digest = Sha256::new();
    for tick in 0..1450 {
        let forces = yload(
            &s,
            if tick < 900 { 4. } else { -4. },
            if tick < 900 { 0.03 } else { 0. },
        );
        p.queue_effort(joint, 0.0002)?;
        clone.queue_effort(joint, 0.0002)?;
        q.queue_effort(joint, 0.0002)?;
        if tick == 130 || tick == 450 {
            let dst = (p.owners()[1] + 1) % 3;
            kill(
                p.worker_pid(if tick == 130 { p.owners()[1] } else { dst })
                    .unwrap(),
            )?;
            p.transfer(model_b, dst)
                .map_err(|e| format!("transfer at {tick}: {e:?}"))?;
            drop(p);
            p = AdaptiveGraspFleetPool::resume(exe(), a.path())
                .map_err(|e| format!("resume at {tick}: {e:?}"))?;
            assert!(p.pending_joint_effort(joint).is_some());
            moves += 1;
        }
        if tick == 500 {
            p.drop_next_acknowledgement(p.owners()[0])?;
            if let Some(path) = std::env::var_os("GZ_ADAPTIVE_GRASP_WITNESS") {
                let out = Path::new(&path);
                fs::create_dir_all(out)?;
                fs::copy(a.path().join(FILE), out.join("prior.gzadaptivegraspfleet"))?;
            }
        }
        p.advance_with_inputs_at(0, tick, &[], &forces)
            .map_err(|e| format!("p prior={tick} advance: {e:?}"))?;
        clone
            .advance_with_inputs_at(0, tick, &[], &forces)
            .map_err(|e| format!("clone prior={tick} advance: {e:?}"))?;
        q.advance_with_inputs_at(0, tick, &[], &forces)
            .map_err(|e| format!("q prior={tick} advance: {e:?}"))?;
        compare(&p, &q)?;
        for m in [model, model_b] {
            assert_eq!(p.image(m), clone.image(m));
            digest.update(p.image(m).unwrap());
        }
        if p.is_coupled() && join.is_none() {
            join = p.joined_at();
            assert_eq!(join, Some(tick));
            assert_eq!(p.owners()[0], p.owners()[1]);
            assert_eq!(p.pending_joint_effort(joint), None);
            assert!(p.advance_with_inputs_at(0, tick, &[], &forces).is_err());
            if let Some(path) = std::env::var_os("GZ_ADAPTIVE_GRASP_WITNESS") {
                let out = Path::new(&path);
                fs::create_dir_all(out)?;
                fs::copy(a.path().join(FILE), out.join("joined.gzadaptivegraspfleet"))?;
            }
        }
        if let Some(pair) = p
            .cross_reactions()
            .iter()
            .find(|r| r.reaction.normal_impulse > 0. && r.reaction.warmstart_normal_impulse > 0.)
        {
            first.get_or_insert(tick + 1);
            warm += 1;
            shear = shear.max(DVec3::from_array(pair.reaction.tangent_impulse).length());
            if moves == 2 || (moves == 3 && tick % 53 == 0) {
                let dst = (p.owners()[0] + 1) % 3;
                kill(
                    p.worker_pid(if moves == 2 { p.owners()[0] } else { dst })
                        .unwrap(),
                )?;
                p.transfer(model_b, dst)
                    .map_err(|e| format!("transfer at {tick}: {e:?}"))?;
                drop(p);
                p = AdaptiveGraspFleetPool::resume(exe(), a.path())
                    .map_err(|e| format!("resume at {tick}: {e:?}"))?;
                compare(&p, &q)?;
                moves += 1;
            }
        }
    }
    assert!(join.is_some_and(|v| v < first.unwrap()));
    assert!(warm > 70 && shear > 0.001 && moves == 4);
    assert!(!a.path().join("barrier.gzgraspfleet").exists());
    assert!(!a.path().join("barrier.gzgraspcluster").exists());
    let before = p.generation();
    p.reset().map_err(|e| format!("post-merge reset: {e:?}"))?;
    assert_eq!((p.epoch(), p.tick(), p.generation()), (1, 0, before + 1));
    assert!(!p.is_coupled());
    assert!(p.joined_at().is_none());
    assert!(p.advance_with_inputs_at(0, 0, &[], &[]).is_err());
    eprintln!(
        "1450 public and native-worker-control frames; first_joint_input_tick={join:?} first_warm={first:?} warm={warm} peak_shear_Ns={shear} real_owner_or_destination_kills={moves} final_reset_separated=true native_sequence_sha256={:x}",
        digest.finalize()
    );
    Ok(())
}
#[test]
fn active_warm_members_refuse_unsafe_merge_and_retain_last_global_cut() -> Result<()> {
    let xml = XML
        .replace("-0.95 0.58", "-0.52 0.58")
        .replace(" 0.95 0.58", " 0.52 0.58")
        .replace("-0.946 0.58", "-0.516 0.58")
        .replace(" 0.954 0.58", " 0.524 0.58");
    let s = spec(&xml)?;
    let d = tempfile::tempdir()?;
    let mut p = AdaptiveGraspFleetPool::create(exe(), d.path(), 3, s.clone())?;
    assert!(!p.is_coupled());
    for g in &s.assemblies {
        let palm = g.payload.tree.links[0].position[1];
        for j in &g.payload.tree.joints {
            let child = g
                .payload
                .tree
                .links
                .iter()
                .find(|l| l.id == j.child_id)
                .unwrap()
                .position[1];
            p.set_velocity_motor(
                j.joint_id,
                Some(JointVelocityMotor {
                    target_velocity: -(child - palm).signum() * 6.,
                    max_force: 4.,
                }),
            )?;
        }
    }
    let id = s.assemblies[0].payload.tree.joints[0].joint_id;
    for tick in 0..1650 {
        let f = if tick >= 650 {
            yload(&s, 3., 0.)
        } else {
            vec![]
        };
        if tick >= 650 {
            p.queue_effort(id, 0.0002)?;
        }
        let old = fs::read(d.path().join(FILE))?;
        match p.advance_with_inputs_at(0, tick, &[], &f) {
            Ok(()) => (),
            Err(ArticulationError::GraspMergeUnavailable {
                epoch,
                tick: old_tick,
                model,
            }) => {
                assert_eq!((epoch, old_tick), (0, tick));
                assert_eq!(old, fs::read(d.path().join(FILE))?);
                assert_eq!(p.pending_joint_effort(id), Some(0.0002));
                for g in &s.assemblies {
                    assert_eq!(
                        p.payload_reactions(g.payload.tree.model_id)
                            .unwrap()
                            .iter()
                            .filter(|r| r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
                            .count(),
                        2
                    );
                }
                drop(p);
                p = AdaptiveGraspFleetPool::resume(exe(), d.path())?;
                assert!(matches!(
                    p.advance_with_inputs_at(0, tick, &[], &f),
                    Err(ArticulationError::GraspMergeUnavailable { .. })
                ));
                assert_eq!(old, fs::read(d.path().join(FILE))?);
                if let Some(path) = std::env::var_os("GZ_ADAPTIVE_GRASP_WITNESS") {
                    let out = Path::new(&path);
                    fs::create_dir_all(out)?;
                    fs::write(out.join("unmergeable.gzadaptivegraspfleet"), old)?;
                }
                eprintln!(
                    "warm native member {model} at {tick} refused clear-only import, complete old global archive and held torque preserved on retry and reopen"
                );
                return Ok(());
            }
            Err(e) => return Err(e.into()),
        }
    }
    panic!("both loaded grippers did not approach")
}
#[test]
fn already_close_construction_starts_coupled_and_resets_to_the_same_solver() -> Result<()> {
    let xml = XML
        .replace("-0.95 0.58", "-0.28 0.58")
        .replace(" 0.95 0.58", " 0.28 0.58")
        .replace("-0.946 0.58", "-0.276 0.58")
        .replace(" 0.954 0.58", " 0.284 0.58");
    let s = spec(&xml)?;
    let a = tempfile::tempdir()?;
    let b = tempfile::tempdir()?;
    let mut p = AdaptiveGraspFleetPool::create(exe(), a.path(), 2, s.clone())?;
    let mut q = CoupledGraspFleetPool::create(exe(), b.path(), 2, s.clone())?;
    assert!(p.is_coupled());
    assert_eq!(p.joined_at(), None);
    for _ in 0..50 {
        p.advance()?;
        q.advance()?;
        compare(&p, &q)?;
        assert_eq!(p.image(s.assemblies[0].payload.tree.model_id), q.image());
    }
    p.reset()?;
    q.reset()?;
    assert!(p.is_coupled());
    assert_eq!(p.joined_at(), None);
    p.advance()?;
    q.advance()?;
    compare(&p, &q)?;
    Ok(())
}
