// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare bounded native staging under the normal SDF, ECS pose and sensor pipeline.
#[allow(dead_code)]
#[path = "support/gpu_sdf_fixture.rs"]
mod generated;
#[path = "support/native_staging_host.rs"]
mod host;
use gz_physics_gpu::{MixedBatch, simulation::BatchTiming};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    AltimeterSample, ImuSample, PhysicsEngine, WorldLoadOptions, WorldPose,
    gpu_scene::{self, GpuSceneObservations},
    load_world_on_adapter, reset_loaded_world,
};
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const COUNT: usize = 2048;
const WORKERS: [usize; 3] = [1, 2, 4];
const STEPS: usize = 180;
const WARM: usize = 30;
fn authored(native: usize, workers: usize, overlap: bool) -> String {
    let original = include_str!("../../../examples/gpu_mixed_world.sdf")
        .replace("0 2.1 0.65 0", "0 2.1 0.41 0")
        .replace("0.035 0.39 0.82", "0.035 0.08 0.82");
    let mut world = generated::fixture_from(&original, COUNT, 10);
    let mut edits = vec![];
    let doc = roxmltree::Document::parse(&world).unwrap();
    let mut registered = 0;
    for node in doc.descendants() {
        match node.tag_name().name() {
            "environment"
                if node.attribute("profile") != Some("supported-pair") && registered < native =>
            {
                edits.push((
                    node.range().start + "<environment".len(),
                    " native_handoff='true' execution='native'".to_string(),
                ));
                registered += 1;
            }
            "observe" if workers > 1 || overlap => edits.push((
                node.range().start,
                format!("<native_staging workers='{workers}' overlap_gpu='{overlap}'/>"),
            )),
            "model" => {
                let name = node.attribute("name").unwrap_or("");
                let ty = match name {
                    "cell_0_rolling" | "cell_2_supported_a" => "imu",
                    "cell_1_free_a" | "cell_3_rolling" => "altimeter",
                    _ => continue,
                };
                let link = node.children().find(|n| n.has_tag_name("link")).unwrap();
                let pos = link.range().end - "</link>".len();
                assert!(world[pos..].starts_with("</link>"));
                edits.push((pos, format!("<sensor name='{ty}' type='{ty}'><always_on>true</always_on><update_rate>100</update_rate><topic>/{name}/{ty}</topic><{ty}/></sensor>")));
            }
            _ => {}
        }
    }
    assert_eq!(registered, native);
    edits.sort_unstable_by_key(|e| e.0);
    for (pos, data) in edits.into_iter().rev() {
        world.insert_str(pos, &data);
    }
    world
}
fn load(adapter: &str, native: usize, workers: usize, overlap: bool) -> Result<Simulation> {
    let xml = authored(native, workers, overlap);
    let now = Instant::now();
    let sim = load_world_on_adapter(
        &xml,
        "native_staging.sdf",
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
    println!(
        "native={native} workers={workers} overlap={overlap} SDF_load_s={:.3}",
        now.elapsed().as_secs_f64()
    );
    let device = sim.world().resource::<MixedBatch>();
    println!(
        "buffer_B={} host_staging_B={} Vulkan_allocator_allocated_reserved_B={:?}",
        device.device_info().allocated_buffer_bytes,
        device.host_state_bytes(),
        device.allocator_usage()
    );
    assert_eq!(
        gpu_scene::execution_counts(&sim),
        Some((native, COUNT - native))
    );
    assert_eq!(
        sim.world()
            .resource::<MixedBatch>()
            .native_staging_workers(),
        workers
    );
    assert_eq!(
        sim.world().resource::<MixedBatch>().overlaps_native_gpu(),
        overlap
    );
    Ok(sim)
}
// Exact comparison is performed after the timed step. Store one digest of the
// full sampled state per checkpoint, and retain endpoints for every Gazebo pose.
#[derive(Default, PartialEq, Debug)]
struct Observation {
    state: Vec<u64>,
    endpoint: Vec<(u64, Vec<u64>)>,
}
fn capture(sim: &Simulation, tick: usize, observed: &mut Observation) {
    let sample = sim.world().resource::<GpuSceneObservations>();
    assert_eq!(sample.tick(), tick as u64);
    let mut hash = 0xcbf29ce484222325u64;
    for e in sample.environments() {
        for word in std::iter::once(e.id).chain(e.state.values().iter().map(|v| v.to_bits())) {
            hash = (hash ^ word).wrapping_mul(0x100000001b3);
        }
    }
    observed.state.push(hash);
    if tick != STEPS {
        return;
    }
    let mut sensors = 0;
    for entity in sim.world().iter_entities() {
        let Some(id) = entity.get::<EntityId>() else {
            continue;
        };
        let mut row = vec![];
        if let Some(p) = entity.get::<WorldPose>() {
            row.extend(
                p.translation
                    .into_iter()
                    .chain(p.rotation)
                    .map(f64::to_bits),
            );
        }
        if let Some(s) = entity.get::<ImuSample>() {
            assert_eq!(s.time.as_millis(), tick as u128);
            let r = s.reading.as_ref().unwrap();
            row.extend(
                r.orientation
                    .into_iter()
                    .chain(r.angular_velocity)
                    .chain(r.specific_force)
                    .map(f64::to_bits),
            );
            sensors += 1;
        }
        if let Some(s) = entity.get::<AltimeterSample>() {
            assert_eq!(s.time.as_millis(), tick as u128);
            let r = s.reading.as_ref().unwrap();
            row.extend(
                [
                    r.vertical_position,
                    r.vertical_velocity,
                    r.vertical_reference,
                ]
                .map(f64::to_bits),
            );
            sensors += 1;
        }
        if !row.is_empty() {
            observed.endpoint.push((id.get(), row));
        }
    }
    observed.endpoint.sort_unstable_by_key(|e| e.0);
    assert_eq!(sensors, 4);
    assert!(observed.endpoint.len() > COUNT * 4);
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("provide ADAPTER NEW_DIRECTORY")?;
    let path = args.next().ok_or("provide NEW_DIRECTORY")?;
    let mode = args.next();
    let matched = matches!(mode.as_deref(), Some("--overlap" | "--overlap-reverse"));
    let reverse = mode.as_deref() == Some("--overlap-reverse");
    if mode.is_some() && !matched || args.next().is_some() {
        return Err("unexpected arguments".into());
    }
    let path = Path::new(&path);
    std::fs::create_dir(path)?;
    let mut raw = BufWriter::new(File::create_new(path.join("steps.csv"))?);
    let mut summary = BufWriter::new(File::create_new(path.join("timing.csv"))?);
    writeln!(
        raw,
        "rep,native,workers,tick,wall_ns,native_stage_ns,gpu_dispatch_ns,upload_B,read_B,overlap,submit_ns,submit_to_fence_cpu_ns,wait_map_ns,decode_ns,gpu_copy_ns"
    )?;
    writeln!(
        summary,
        "rep,native,workers,p50_us,p95_us,p99_us,mean_us,env_steps_s,overlap"
    )?;
    let loads: &[usize] = if reverse {
        &[96, 24, 4]
    } else if matched {
        &[4, 24, 96]
    } else {
        &[24, 96]
    };
    let orders: &[bool] = if matched { &[false, true] } else { &[false] };
    let treatments: Vec<_> = WORKERS
        .into_iter()
        .flat_map(|cap| orders.iter().map(move |&overlap| (cap, overlap)))
        .collect();
    let mut host = BufWriter::new(File::create_new(path.join("host.csv"))?);
    writeln!(
        host,
        "rep,native,workers,overlap,threads,thread_run_ns,thread_runqueue_wait_ns"
    )?;
    for &native in loads {
        let mut runs = treatments
            .iter()
            .map(|&(cap, overlap)| load(&adapter, native, cap, overlap))
            .collect::<Result<Vec<_>>>()?;
        let mut expected = None;
        let reps = if matched { 6 } else { 3 };
        for rep in 0..reps {
            // Pair the two orders at each worker count, reverse the pair order on
            // alternate trials, and rotate the first worker cap between trials.
            for turn in 0..treatments.len() {
                let cycle = (turn / orders.len() + rep) % WORKERS.len();
                let worker = if reverse {
                    WORKERS.len() - 1 - cycle
                } else {
                    cycle
                };
                let order = (turn + rep + usize::from(reverse)) % orders.len();
                let i = worker * orders.len() + order;
                let (cap, overlap) = treatments[i];
                let sim = &mut runs[i];
                if rep > 0 {
                    reset_loaded_world(sim, |_| ())?;
                }
                sim.tick()?;
                let mut observed = Observation::default();
                let mut times = vec![];
                let mut loaded = 0;
                let mut scheduler = None;
                for t in 1..=STEPS {
                    if t == WARM + 1 {
                        scheduler = host::snapshot();
                    }
                    let start = Instant::now();
                    sim.step_exact(1)?;
                    let ns = start.elapsed().as_nanos();
                    let m = sim.world().resource::<BatchTiming>().last.unwrap();
                    assert_eq!(m.cpu_between_submit_and_fence.is_zero(), !overlap);
                    writeln!(
                        raw,
                        "{rep},{native},{cap},{t},{ns},{},{},{},{},{overlap},{},{},{},{},{}",
                        m.cpu_solve.as_nanos(),
                        m.gpu_dispatch.map_or(0, |x| x.as_nanos()),
                        m.uploaded_bytes,
                        m.readback_bytes,
                        m.encode_submit.as_nanos(),
                        m.cpu_between_submit_and_fence.as_nanos(),
                        m.wait_map.as_nanos(),
                        m.decode.as_nanos(),
                        m.gpu_copy.map_or(0, |x| x.as_nanos())
                    )?;
                    if t > WARM {
                        times.push(ns as f64 / 1000.);
                    }
                    if t.is_multiple_of(10) {
                        capture(sim, t, &mut observed);
                        if gpu_scene::native_contact(sim, 0)?
                            .unwrap()
                            .warmstart_normal_impulse
                            > 0.
                        {
                            loaded += 1;
                        }
                    }
                }
                if let Some((exec, wait, threads)) =
                    host::elapsed(scheduler.as_ref(), host::snapshot().as_ref())
                {
                    writeln!(
                        host,
                        "{rep},{native},{cap},{overlap},{threads},{exec},{wait}"
                    )?;
                }
                assert!(
                    loaded > 8,
                    "expected supported native warm starts during the timed interval"
                );
                if let Some(ref reference) = expected {
                    assert!(
                        observed == *reference,
                        "ordinary scene sample or endpoint mismatch native={native} workers={cap} rep={rep}"
                    )
                } else {
                    expected = Some(observed);
                }
                times.sort_by(f64::total_cmp);
                let percentile = |p: usize| times[(p * times.len()).div_ceil(100) - 1];
                let mean = times.iter().sum::<f64>() / times.len() as f64;
                writeln!(
                    summary,
                    "{rep},{native},{cap},{},{},{},{mean},{},{overlap}",
                    percentile(50),
                    percentile(95),
                    percentile(99),
                    COUNT as f64 * 1e6 / mean
                )?;
                println!(
                    "rep={rep} native={native} workers={cap} overlap={overlap} SDF whole p50/p95/p99_us={:.1}/{:.1}/{:.1}; {loaded} supported checkpoints; {} stable Gazebo pose/sensor endpoints",
                    percentile(50),
                    percentile(95),
                    percentile(99),
                    expected.as_ref().unwrap().endpoint.len()
                );
            }
        }
    }
    Ok(())
}
