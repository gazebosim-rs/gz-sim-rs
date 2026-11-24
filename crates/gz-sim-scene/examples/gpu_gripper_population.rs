// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Profile the full SDF/Bevy/device/sensor route for a mixed load-bearing gripper population.
use gz_physics_gpu::{ParallelGripperBatch, ParallelGripperLane, simulation::BatchTiming};
use gz_sim_scene::{
    JointStatePublication, JointStateSample, PhysicsEngine, WorldLoadOptions,
    gpu_scene::parallel_gripper::{observations, set_population_targets},
    load_world_on_adapter, reset_loaded_world,
};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    io::Write as _,
    path::Path,
    time::{Duration, Instant},
};
#[path = "support/gripper_population.rs"]
mod fixture;
#[path = "../../gz-physics-gpu/examples/support/parallel_gripper/native.rs"]
mod reference;

fn kib(key: &str) -> Option<u64> {
    let text = fs::read_to_string("/proc/self/status").ok()?;
    text.lines()
        .find(|s| s.starts_with(key))?
        .split_whitespace()
        .nth(1)?
        .parse()
        .ok()
}
fn quantile(values: &[f64], p: usize) -> f64 {
    let mut v = values.to_vec();
    v.sort_by(f64::total_cmp);
    v[(v.len() * p).div_ceil(100) - 1]
}
fn pose(x: f64, y: f64, z: f64, angle: f64) -> Value {
    let (s, c) = (-angle * 0.5).sin_cos();
    json!({"translation":[x,y,z],"rotation":[0.,s,0.,c]})
}
fn visual(lanes: &[ParallelGripperLane], tick: u64) -> Value {
    let mut p = serde_json::Map::new();
    p.insert("floor".into(), pose(0.24, 0.20, -0.009, 0.));
    for (i, l) in lanes.iter().enumerate() {
        let x = (i % 4) as f64 * 0.16;
        let y = i as f64 * 0.08;
        let [z, a, b] = l.position;
        let [bx, bz] = l.ball;
        for (k, v) in [
            ("rail", pose(x, y, z + 0.08, 0.)),
            ("left", pose(a, y, z, 0.)),
            ("left_stem", pose(a, y, z + 0.044, 0.)),
            ("right", pose(b, y, z, 0.)),
            ("right_stem", pose(b, y, z + 0.044, 0.)),
            ("ball", pose(bx, y, bz, l.angle)),
            (
                "marker",
                pose(
                    bx + 0.014 * l.angle.cos(),
                    y + 0.015,
                    bz + 0.014 * l.angle.sin(),
                    l.angle,
                ),
            ),
        ] {
            p.insert(format!("{k}_{i}"), v);
        }
    }
    json!({"time_ns":tick*1_000_000,"poses":p})
}
fn summarize_cpu(
    ids: &[u64],
    initial: &[ParallelGripperLane],
    history: &[Vec<ParallelGripperLane>],
) -> Vec<Value> {
    initial
        .iter()
        .enumerate()
        .map(|(i, initial)| {
            let mut native = reference::Native::new(initial, 16);
            let mut max_error = 0.0_f64;
            let mut loads = [None; 2];
            let mut min_clear = [f64::INFINITY; 2];
            let mut times = Vec::new();
            for (t, states) in history.iter().enumerate() {
                let l = &states[i];
                let start = Instant::now();
                let (n, _) = native.step(l);
                times.push(start.elapsed().as_secs_f64() * 1e6);
                let (_, b) = native.state();
                max_error = max_error.max((b[0] - l.ball[0]).hypot(b[1] - l.ball[1]));
                for (j, (height, contacts)) in [(l.ball[1], l.normal_impulse), (b[1], n)]
                    .into_iter()
                    .enumerate()
                {
                    if contacts[0].min(contacts[1]) > 1e-7 {
                        loads[j].get_or_insert(t + 1);
                    }
                    if (499..640).contains(&t) {
                        min_clear[j] = min_clear[j].min(height - l.radius - l.ground);
                    }
                }
            }
            json!({
                "id": ids[i],
                "mass_kg": initial.ball_mass,
                "pad_mu": initial.friction[0],
                "first_grip_ms": loads,
                "minimum_hold_clearance_gpu_native_m": min_clear,
                "max_path_error_m": max_error,
                "native_fine_p95_us": quantile(&times,95)
            })
        })
        .collect()
}
fn run(adapter: &str, count: usize, dir: &Path, streamed: bool) -> Result<(), Box<dyn Error>> {
    if count == 0 {
        return Err("population must be positive".into());
    }
    fs::create_dir_all(dir)?;
    let baseline = kib("VmRSS:");
    let mut source = fixture::xml(count);
    if streamed {
        source = fixture::streamed(&source);
    }
    fs::write(dir.join("world.sdf"), &source)?;
    let now = Instant::now();
    let mut sim = load_world_on_adapter(
        &source,
        "gripper_population.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(adapter),
    )?;
    let load_s = now.elapsed().as_secs_f64();
    let entities = sim.world().entities().len();
    let memory = kib("VmRSS:");
    let dev = sim
        .world()
        .resource::<ParallelGripperBatch>()
        .device_info()
        .clone();
    let is_reference = |id: u64| id < 8 || (id as usize) >= count.saturating_sub(8);
    let (reference_ids, source_lanes): (Vec<_>, Vec<_>) = observations(&sim)?
        .filter(|(id, _)| is_reference(*id))
        .map(|(id, l)| (id, *l))
        .unzip();
    let visible = source_lanes.len().min(8);
    let mut history = Vec::new();
    let mut poses = vec![visual(&source_lanes[..visible], 0)];
    let mut epochs = Vec::new();
    let mut previous = None;
    let mut csv = fs::File::create(dir.join("ticks.csv"))?;
    writeln!(
        csv,
        "epoch,tick,schedule_us,observed_us,gpu_dispatch_us,uploaded_bytes,readback_bytes"
    )?;
    for epoch in 0..2 {
        if epoch > 0 {
            reset_loaded_world(&mut sim, |_| ()).map_err(|e| format!("reset: {e:?}"))?;
        }
        let mut models: Vec<_> = sim
            .world()
            .iter_entities()
            .filter_map(|e| {
                let meta = e.get::<JointStatePublication>()?;
                let id = meta
                    .model_name
                    .strip_prefix("gripper_")?
                    .parse::<usize>()
                    .ok()?;
                Some((id, e.id()))
            })
            .collect();
        models.sort_by_key(|(id, _)| *id);
        assert_eq!(models.len(), count);
        let mut hash = Sha256::new();
        let mut held = vec![f64::INFINITY; count];
        let mut held_floor = vec![0.0_f64; count];
        let mut public = Vec::new();
        let mut sched = Vec::new();
        let mut gpu = Vec::new();
        let mut upload = 0u64;
        let mut read = 0u64;
        let mut actions = vec![[0.; 3]; count];
        let mut bytes = Vec::with_capacity(8 * 24 * count);
        let episode_start = Instant::now();
        for tick in 1..=1000u64 {
            let start = Instant::now();
            if streamed {
                for (i, target) in actions.iter_mut().enumerate() {
                    *target = fixture::action(i, tick);
                }
                set_population_targets(&mut sim, &actions)?;
            }
            sim.step_exact(1)?;
            let schedule = start.elapsed();
            let timing = sim.world().resource::<BatchTiming>().last.unwrap();
            bytes.clear();
            let mut ref_row = Vec::with_capacity(source_lanes.len());
            for ((id, l), (expected, e)) in observations(&sim)?.zip(&models) {
                assert_eq!(id, *expected as u64);
                if (500..=640).contains(&tick) {
                    held[*expected] = held[*expected].min(l.ball[1] - l.radius - l.ground);
                    held_floor[*expected] = held_floor[*expected].max(l.normal_impulse[2]);
                }
                let sample = sim
                    .world()
                    .get::<JointStateSample>(*e)
                    .ok_or("missing committed joint sensor")?;
                assert_eq!(sample.time, Duration::from_millis(tick));
                assert_eq!(sample.epoch, epoch);
                let origin = [
                    0.028,
                    (id % 4) as f64 * 0.16 - 0.05,
                    (id % 4) as f64 * 0.16 + 0.05,
                ];
                assert_eq!(sample.joints.len(), 3);
                for (j, read) in sample.joints.iter().enumerate() {
                    assert!((read.position - (l.position[j] - origin[j])).abs() < 1e-13);
                    assert_eq!(read.velocity, l.speed[j]);
                    bytes.extend_from_slice(&read.position.to_bits().to_le_bytes());
                    bytes.extend_from_slice(&read.velocity.to_bits().to_le_bytes());
                }
                for n in l
                    .ball
                    .into_iter()
                    .chain(l.velocity)
                    .chain([l.angle, l.spin])
                    .chain(l.target)
                    .chain(l.normal_impulse)
                    .chain(l.tangent_impulse)
                    .chain(l.support_impulse)
                {
                    bytes.extend_from_slice(&n.to_bits().to_le_bytes());
                }
                if epoch == 0 && is_reference(id) {
                    ref_row.push(*l);
                }
            }
            hash.update(&bytes);
            let wall = start.elapsed().as_secs_f64() * 1e6;
            let step = schedule.as_secs_f64() * 1e6;
            if epoch == 0 {
                poses.push(visual(&ref_row[..visible], tick));
                history.push(ref_row);
            }
            let dispatch = timing.gpu_dispatch.map(|d| d.as_secs_f64() * 1e6);
            public.push(wall);
            sched.push(step);
            if let Some(d) = dispatch {
                gpu.push(d);
            }
            upload += timing.uploaded_bytes;
            read += timing.readback_bytes;
            writeln!(
                csv,
                "{epoch},{tick},{step},{wall},{},{},{}",
                dispatch.unwrap_or_default(),
                timing.uploaded_bytes,
                timing.readback_bytes
            )?;
        }
        let episode_seconds = episode_start.elapsed().as_secs_f64();
        let cohort: Vec<_> = (0..8.min(count))
            .map(|case| {
                let clear: Vec<_> = held.iter().skip(case).step_by(8).copied().collect();
                let max_floor = held_floor
                    .iter()
                    .skip(case)
                    .step_by(8)
                    .copied()
                    .fold(0., f64::max);
                let lifted = clear.iter().filter(|&&z| z > 0.1).count();
                if matches!(case, 0 | 1 | 2 | 7) {
                    assert_eq!(lifted, clear.len(), "cohort {case}");
                    assert_eq!(max_floor, 0.);
                } else {
                    assert_eq!(lifted, 0);
                }
                json!({
                    "case": case,
                    "count": clear.len(),
                    "clear_100mm": lifted,
                    "minimum_held_m": clear.iter().copied().fold(f64::INFINITY,f64::min),
                    "maximum_held_m": clear.iter().copied().fold(f64::NEG_INFINITY,f64::max),
                    "max_hold_floor_impulse_ns": max_floor
                })
            })
            .collect();
        let digest = format!("{:x}", hash.finalize());
        if let Some(ref first) = previous {
            assert_eq!(first, &digest);
        } else {
            previous = Some(digest.clone());
        }
        let wall_s = public.iter().sum::<f64>() / 1e6;
        epochs.push(json!({
            "epoch": epoch,
            "environment_steps_s": 1000.*count as f64/episode_seconds,
            "elapsed_episode_seconds": episode_seconds,
            "observed_environment_steps_s": 1000.*count as f64/wall_s,
            "public_wall_seconds": wall_s,
            "per_environment_simulated_seconds_per_wall_second": 1./episode_seconds,
            "public_p50_us": quantile(&public,50),
            "public_p95_us": quantile(&public,95),
            "public_p99_us": quantile(&public,99),
            "schedule_p99_us": quantile(&sched,99),
            "gpu_dispatch_p99_us": (!gpu.is_empty()).then(||quantile(&gpu,99)),
            "uploaded_bytes": upload,
            "readback_bytes": read,
            "publication_digest": digest,
            "cohorts": cohort,
            "sensor_values": count*3*1000
        }));
        eprintln!(
            "epoch {epoch}, {} complete robot-steps/s",
            1000. * count as f64 / episode_seconds
        );
    }
    let cpu = summarize_cpu(&reference_ids, &source_lanes, &history);
    for row in &cpu {
        let case = row["id"].as_u64().unwrap() % 8;
        let heights = &row["minimum_hold_clearance_gpu_native_m"];
        if matches!(case, 0 | 1 | 2 | 7) {
            assert!(
                heights
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|n| n.as_f64().unwrap() > 0.1)
            );
            assert!(row["max_path_error_m"].as_f64().unwrap() < 0.004);
            assert_eq!(row["first_grip_ms"][0], row["first_grip_ms"][1]);
        } else {
            assert!(
                heights
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|n| n.as_f64().unwrap() < 0.0001)
            );
        }
    }
    let summary = json!({
        "action_source": if streamed {"dense_live"} else {"SDF"},
        "adapter": dev.name,
        "driver": dev.driver_info,
        "state_bytes": dev.state_bytes,
        "wgpu_allocated_buffer_bytes": dev.allocated_buffer_bytes,
        "workgroups": dev.workgroups,
        "environments": count,
        "entities": entities,
        "loaded_seconds": load_s,
        "baseline_rss_kib": baseline,
        "loaded_rss_kib": memory,
        "process_peak_rss_kib": kib("VmHWM:"),
        "public_dt_ms": 1,
        "ticks_per_epoch": 1000,
        "reference_substep_us": 62.5,
        "device_substep_us": 250,
        "epochs": epochs,
        "native_reference": cpu
    });
    fs::write(dir.join("source-poses.json"), serde_json::to_vec(&poses)?)?;
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&summary)?);
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    match args.as_slice() {
        [flag, count, file] if flag == "--write-sdf" => {
            fs::write(file, fixture::xml(count.parse()?))?;
            Ok(())
        }
        [adapter, count, dir] => run(adapter, count.parse()?, Path::new(dir), false),
        [flag, adapter, count, dir] if flag == "--live-actions" => {
            run(adapter, count.parse()?, Path::new(dir), true)
        }
        _ => Err("expected ADAPTER COUNT RECEIPT_DIRECTORY or --write-sdf COUNT FILE".into()),
    }
}
