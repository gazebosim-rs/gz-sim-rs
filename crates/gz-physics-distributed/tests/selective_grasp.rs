// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Split an actually released part, turn it back toward the fingers and inspect the next squeeze.
use gz_physics::{JointVelocityMotor, PhysicsBackend};
use gz_physics_distributed::floating_tree::{GraspPlacement, GraspTreeSpec, SplitGraspPool};
use gz_physics_multifidelity::{ArticulationTier, MovingReaction, MultiFidelityBackend};
use gz_sim_core::entity_by_id;
use gz_sim_scene::{BodyHandle, JointDefinition, JointHandle, PhysicsEngine};
use std::{error::Error, path::Path, process::Command, time::Instant};
const XML: &str = include_str!("../../../examples/distributed_gripper_payload.sdf");
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn motor(t: u64, sign: f64) -> JointVelocityMotor {
    JointVelocityMotor {
        target_velocity: if t == 600 { -6. * sign } else { 6. * sign },
        max_force: 4.,
    }
}
fn force(t: u64) -> [f64; 3] {
    [
        0.,
        0.,
        match t {
            650..850 | 1250..1450 => 1.2,
            850..1250 => -1.2,
            _ => 0.,
        },
    ]
}
fn reaction_bits(r: &MovingReaction) -> (bool, [u64; 3], u64, u64, [u64; 3]) {
    (
        r.active,
        r.normal.map(f64::to_bits),
        r.normal_impulse.to_bits(),
        r.warmstart_normal_impulse.to_bits(),
        r.tangent_impulse.map(f64::to_bits),
    )
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
fn summary(mut t: Vec<u128>) -> [u128; 3] {
    t.sort_unstable();
    if t.is_empty() {
        return [0; 3];
    }
    [50, 95, 99].map(|p| t[(t.len() - 1) * p / 100])
}
#[test]
fn dual_owners_release_and_recapture_match_ordinary_bevy_through_migration() -> Result<()> {
    let spec = gz_sim_scene::distributed_payload_tree_spec(XML, "gripper.sdf", "default")?;
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let a = tempfile::tempdir()?;
    let b = tempfile::tempdir()?;
    let mut adaptive = SplitGraspPool::create(
        exe,
        a.path(),
        3,
        GraspTreeSpec {
            payload: spec.clone(),
        },
        GraspPlacement::Adaptive { release_dwell: 12 },
    )?;
    let mut keep = SplitGraspPool::create(
        exe,
        b.path(),
        3,
        GraspTreeSpec {
            payload: spec.clone(),
        },
        GraspPlacement::KeepTogether,
    )?;
    let mut native = gz_sim_scene::load_world_with_sdf_step(
        XML,
        "gripper.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    gz_sim_scene::set_model_fidelity(&mut native, spec.tree.model_id, ArticulationTier::Dynamic)?;
    let w = native.world();
    let body = |id| {
        w.get::<BodyHandle>(entity_by_id(w, id).unwrap())
            .unwrap()
            .id()
    };
    let part = body(spec.payload.id);
    let links: Vec<_> = spec.tree.links.iter().map(|l| body(l.id)).collect();
    let joints: Vec<_> = spec
        .tree
        .joints
        .iter()
        .map(|j| {
            let e = entity_by_id(w, j.joint_id).unwrap();
            (
                j.joint_id,
                w.get::<JointHandle>(e).unwrap().id(),
                if w.get::<JointDefinition>(e).unwrap().0.name == "right_close" {
                    1.
                } else {
                    -1.
                },
            )
        })
        .collect();
    let (mut before, mut after, mut splits, mut merges, mut split_frames, mut moving_deaths) =
        (0, 0, 0, 0, 0, 0);
    let mut times: [Vec<u128>; 4] = Default::default();
    for t in 0..2600 {
        if matches!(t, 0 | 600 | 1450) {
            for &(id, handle, sign) in &joints {
                let m = Some(motor(t, sign));
                adaptive.set_velocity_motor(id, m)?;
                keep.set_velocity_motor(id, m)?;
                native
                    .world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .set_joint_velocity_motor(handle, m)?;
            }
        }
        let mut load = force(t);
        let inputs = if t == 1887 {
            let id = joints[0].0;
            vec![(id, -0.0003)]
        } else {
            vec![]
        };
        if t == 1887 {
            load[0] = 0.015;
            let id = joints[0].0;
            adaptive.queue_effort(id, 0.0007)?;
            keep.queue_effort(id, 0.0007)?;
            gz_sim_scene::queue_joint_effort(&mut native, id, 0.0007)?;
            for slot in adaptive.owners() {
                adaptive.drop_next_acknowledgement(slot)?;
            }
        }
        for &(id, e) in &inputs {
            gz_sim_scene::queue_joint_effort(&mut native, id, e)?;
        }
        if load != [0.; 3] {
            native
                .world_mut()
                .resource_mut::<PhysicsBackend>()
                .add_world_wrench(part, load, [0.; 3])?;
        }
        native.step_exact(1)?;
        let old = adaptive.owners();
        let was_split = old[0] != old[1];
        let draw = |p: &mut SplitGraspPool| -> Result<u128> {
            let start = Instant::now();
            p.advance_with_payload_force_at(0, t, &inputs, load)?;
            Ok(start.elapsed().as_nanos())
        };
        let (x, y) = if t % 2 == 0 {
            (draw(&mut adaptive)?, draw(&mut keep)?)
        } else {
            let y = draw(&mut keep)?;
            (draw(&mut adaptive)?, y)
        };
        let now = adaptive.owners();
        let is_split = now[0] != now[1];
        if is_split != was_split {
            eprintln!(
                "tick={} split={is_split} owners={now:?} stats={:?}",
                t + 1,
                adaptive.stats()
            );
        }
        splits += usize::from(is_split && !was_split);
        merges += usize::from(!is_split && was_split);
        split_frames += usize::from(is_split);
        times[usize::from(is_split) * 2].push(x);
        times[usize::from(is_split) * 2 + 1].push(y);
        assert_eq!(
            adaptive.observation(),
            keep.observation(),
            "tree tick {}",
            t + 1
        );
        assert_eq!(
            adaptive.payload_state(),
            keep.payload_state(),
            "part tick {}",
            t + 1
        );
        let solver = native.world().resource::<PhysicsBackend>();
        let full = solver.downcast_ref::<MultiFidelityBackend>().unwrap();
        assert!(
            keep.image()
                .is_some_and(|img| img == full.checkpoint_grasp_revolute_tree().unwrap()),
            "full native tick {}",
            t + 1
        );
        let states = adaptive
            .observation()
            .links
            .iter()
            .zip(&links)
            .chain(std::iter::once((adaptive.payload_state(), &part)));
        for (s, &id) in states {
            let other = solver.state(id).unwrap();
            assert_eq!(
                s.position.map(f64::to_bits),
                other.position.map(f64::to_bits)
            );
            assert_eq!(
                s.orientation.map(f64::to_bits),
                other.orientation.map(f64::to_bits)
            );
            assert_eq!(
                s.velocity.map(f64::to_bits),
                other.velocity.map(f64::to_bits)
            );
            assert_eq!(
                s.angular_velocity.map(f64::to_bits),
                other.angular_velocity.map(f64::to_bits)
            );
        }
        let mut loaded = 0;
        for ((&id, r), k) in links
            .iter()
            .zip(adaptive.payload_reactions())
            .zip(keep.payload_reactions())
        {
            assert_eq!(reaction_bits(r), reaction_bits(k), "pair tick {}", t + 1);
            assert_eq!(
                reaction_bits(r),
                reaction_bits(&full.payload_tree_reaction(id, part)?)
            );
            loaded +=
                usize::from(r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.);
        }
        if loaded >= 2 {
            assert!(!is_split);
            if t < 600 {
                before += 1;
            }
            if t > 1450 {
                after += 1;
            }
        }
        if is_split && (moving_deaths == 0 || t % 39 == 0) && moving_deaths < 4 {
            let free = (0..3).find(|slot| !now.contains(slot)).unwrap();
            let saved = adaptive.observation().clone();
            let part_before = *adaptive.payload_state();
            let which = moving_deaths % 2;
            kill(adaptive.worker_pid(now[which]).unwrap())?;
            if which == 0 {
                adaptive.transfer(free)?;
            } else {
                adaptive.transfer_payload(free)?;
            }
            let owners = adaptive.owners();
            drop(adaptive);
            adaptive = SplitGraspPool::resume(exe, a.path())?;
            assert_eq!(adaptive.owners(), owners);
            assert_eq!(adaptive.observation(), &saved);
            assert_eq!(adaptive.payload_state(), &part_before);
            moving_deaths += 1;
        }
    }
    eprintln!(
        "frames=2600 released_intervals={split_frames} splits={splits} rejoins={merges} bits_exact_before_bilateral={before} bits_exact_after_bilateral={after} split_worker_kills_migrations_and_reopens={moving_deaths} stats={:?}",
        adaptive.stats()
    );
    for (i, t) in times.into_iter().enumerate() {
        let n = t.len();
        eprintln!(
            "timing_group={i} samples={n} p50_p95_p99_ns={:?}",
            summary(t)
        );
    }
    assert!(
        before > 150
            && after > 150
            && split_frames > 50
            && splits >= 1
            && merges >= 1
            && moving_deaths == 4
    );
    assert!(adaptive.stats().rejected_previews >= 1 && adaptive.stats().split_steps >= 50);
    Ok(())
}

fn tick_program(p: &mut SplitGraspPool, t: u64) -> Result<()> {
    if matches!(t, 0 | 600 | 1450) {
        let joints = p.topology().joints.clone();
        for j in joints {
            let l = p
                .topology()
                .links
                .iter()
                .find(|l| l.id == j.child_id)
                .unwrap();
            p.set_velocity_motor(j.joint_id, Some(motor(t, -l.position[1].signum())))?;
        }
    }
    p.advance_with_payload_force_at(0, t, &[], force(t))?;
    Ok(())
}
#[test]
#[ignore = "matched durable storage, layout and release dwell benchmark"]
fn measured_storage_and_dwell_for_separate_grasp_owners() -> Result<()> {
    use std::{fs, io::Write};
    const WINDOWS: [(usize, usize, &str); 4] = [
        (250, 120, "bilateral"),
        (1105, 160, "release"),
        (1500, 180, "free"),
        (1830, 160, "return"),
    ];
    let spec = GraspTreeSpec {
        payload: gz_sim_scene::distributed_payload_tree_spec(XML, "gripper.sdf", "default")?,
    };
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let out = std::path::PathBuf::from(std::env::var("GZ_GRASP_BENCH_DIR")?);
    fs::create_dir_all(&out)?;
    let root = Path::new("/dev/shm");
    let baseline = tempfile::tempdir_in(root)?;
    let candidate = tempfile::tempdir_in(root)?;
    let file = "barrier.gzsplitgrasp";
    let mut keep = SplitGraspPool::create(
        exe,
        baseline.path(),
        3,
        spec.clone(),
        GraspPlacement::KeepTogether,
    )?;
    let mut split = SplitGraspPool::create(
        exe,
        candidate.path(),
        3,
        spec.clone(),
        GraspPlacement::Adaptive { release_dwell: 12 },
    )?;
    let mut seeds = Vec::new();
    for tick in 0..1990 {
        if WINDOWS.iter().any(|w| w.0 == tick) {
            seeds.push((
                fs::read(baseline.path().join(file))?,
                fs::read(candidate.path().join(file))?,
            ));
        }
        tick_program(&mut keep, tick as u64)?;
        tick_program(&mut split, tick as u64)?;
    }
    drop(keep);
    drop(split);
    let mut raw = fs::File::create(out.join("matched-storage.csv"))?;
    writeln!(
        raw,
        "storage,window,repetition,tick,split,adaptive_ns,keep_ns,adaptive_bytes,keep_bytes"
    )?;
    for (fsname, base) in [("tmpfs", root), ("local-fsync", Path::new("/var/tmp"))] {
        for ((start, len, window), (keep_seed, adapt_seed)) in WINDOWS.iter().zip(&seeds) {
            let (mut k_times, mut a_times) = (Vec::new(), Vec::new());
            let mut two = 0;
            let mut bytes = [0; 2];
            for rep in 0..4 {
                let a = tempfile::tempdir_in(base)?;
                let b = tempfile::tempdir_in(base)?;
                fs::write(a.path().join(file), adapt_seed)?;
                fs::write(b.path().join(file), keep_seed)?;
                let mut adaptive = SplitGraspPool::resume(exe, a.path())?;
                let mut keep = SplitGraspPool::resume(exe, b.path())?;
                for tick in *start..(start + len) {
                    let tx = |p: &mut SplitGraspPool| -> Result<u128> {
                        let timer = Instant::now();
                        tick_program(p, tick as u64)?;
                        Ok(timer.elapsed().as_nanos())
                    };
                    let (x, y) = if (tick + rep) % 2 == 0 {
                        (tx(&mut adaptive)?, tx(&mut keep)?)
                    } else {
                        let y = tx(&mut keep)?;
                        (tx(&mut adaptive)?, y)
                    };
                    assert_eq!(adaptive.observation(), keep.observation());
                    assert_eq!(adaptive.payload_state(), keep.payload_state());
                    assert_eq!(adaptive.payload_reactions(), keep.payload_reactions());
                    let split = adaptive.owners()[0] != adaptive.owners()[1];
                    two += usize::from(split);
                    let ab = fs::metadata(a.path().join(file))?.len();
                    let kb = fs::metadata(b.path().join(file))?.len();
                    bytes[0] += ab;
                    bytes[1] += kb;
                    writeln!(
                        raw,
                        "{fsname},{window},{rep},{tick},{split},{x},{y},{ab},{kb}"
                    )?;
                    k_times.push(y);
                    a_times.push(x);
                }
            }
            let count = k_times.len();
            eprintln!(
                "storage={fsname} window={window} paired_intervals={count} split={two} average_bytes_adaptive_keep={:?} p50_p95_p99_ns_adaptive={:?} p50_p95_p99_ns_keep={:?}",
                bytes.map(|b| b / count as u64),
                summary(a_times),
                summary(k_times)
            );
        }
    }
    let mut dwells = std::fs::File::create(out.join("dwell.csv"))?;
    writeln!(
        dwells,
        "dwell,first_split,first_rejoin,split_steps,rejected_previews"
    )?;
    for dwell in [1, 12, 48, 120, 800] {
        let d = tempfile::tempdir_in(root)?;
        let mut pool = SplitGraspPool::create(
            exe,
            d.path(),
            3,
            spec.clone(),
            GraspPlacement::Adaptive {
                release_dwell: dwell,
            },
        )?;
        let mut first = 0;
        let mut returned = 0;
        for tick in 0..2200 {
            tick_program(&mut pool, tick)?;
            let joined = pool.owners()[0] == pool.owners()[1];
            if !joined && first == 0 {
                first = tick + 1;
            }
            if joined && first != 0 && returned == 0 {
                returned = tick + 1;
            }
        }
        let st = pool.stats();
        writeln!(
            dwells,
            "{dwell},{first},{returned},{},{}",
            st.split_steps, st.rejected_previews
        )?;
        eprintln!("release_dwell={dwell} first_split={first} first_rejoin={returned} stats={st:?}");
    }
    Ok(())
}
