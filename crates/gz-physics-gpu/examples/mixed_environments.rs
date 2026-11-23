// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! cargo run --release -p gz-physics-gpu --example mixed_environments -- --adapter NAME [--capture NEW_REPLAY] [--out NEW_DIRECTORY]
#[path = "support/sphere_pair/mod.rs"]
pub mod pair;
#[path = "support/sphere_plane/mod.rs"]
pub mod plane;
pub use pair::reference;
#[path = "support/mixed/capture.rs"]
mod capture;
#[path = "support/mixed/qualification.rs"]
mod qualification;
#[path = "support/mixed/report.rs"]
mod report;
#[path = "support/supported_pair/fixture.rs"]
pub mod support;
#[path = "support/times.rs"]
mod timing;
use gz_physics_gpu::{
    Environment as Env, EnvironmentState as State, MixedBatch, SpherePairBatch, SpherePlaneBatch,
    SupportedSpherePairBatch,
    simulation::{BatchTiming, mixed_batch_simulation},
};
use qualification::{Dense, N, Native, TICKS, cpu_step, fixture, registered, signature};
use std::{
    error::Error,
    hint::black_box,
    path::PathBuf,
    time::{Duration, Instant},
};
use timing::Times;

struct Separate {
    plane: SpherePlaneBatch,
    pair: SpherePairBatch,
    supported: SupportedSpherePairBatch,
}
impl Separate {
    fn new(adapter: &str, items: &[Env]) -> Result<Self, Box<dyn Error>> {
        let (mut planes, mut pairs, mut supported) = (vec![], vec![], vec![]);
        for env in items {
            match env.state {
                State::SpherePlane(s) => planes.push(s),
                State::SpherePair(s) => pairs.push(s),
                State::SupportedSpherePair(s) => supported.push(s),
            }
        }
        Ok(Self {
            plane: SpherePlaneBatch::new(adapter, planes)?,
            pair: SpherePairBatch::new(adapter, pairs)?,
            supported: SupportedSpherePairBatch::new(adapter, supported)?,
        })
    }
    fn step(&mut self) -> Result<(), Box<dyn Error>> {
        self.plane.step()?;
        self.pair.step()?;
        self.supported.step()?;
        Ok(())
    }
    fn state(&self, i: usize) -> State {
        match i % 3 {
            0 => State::SpherePlane(self.plane.lanes()[i / 3]),
            1 => State::SpherePair(self.pair.lanes()[i / 3]),
            _ => State::SupportedSpherePair(self.supported.lanes()[i / 3]),
        }
    }
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let (mut adapter, mut capture, mut output) = (None, None, None);
    while let Some(k) = args.next() {
        let v = args.next().ok_or("missing value")?;
        match k.as_str() {
            "--adapter" if adapter.is_none() => adapter = Some(v),
            "--capture" if capture.is_none() => capture = Some(PathBuf::from(v)),
            "--out" if output.is_none() => output = Some(PathBuf::from(v)),
            _ => {
                return Err(
                    "expected --adapter NAME [--capture NEW_REPLAY] [--out NEW_DIRECTORY]".into(),
                );
            }
        }
    }
    let adapter = adapter.ok_or("select a hardware adapter with --adapter NAME")?;
    let mut report = output.as_ref().map(|_| report::Report::default());
    let mut archive = capture.as_ref().map(|_| capture::Capture::default());
    let initial = fixture();
    let setup = Instant::now();
    let mut sim = mixed_batch_simulation(&adapter, registered(&initial))?;
    let mixed_setup = setup.elapsed();
    let m = sim.world().resource::<MixedBatch>();
    assert_eq!(m.environments(), initial);
    let info = m.device_info();
    println!(
        "adapter {} {} {}; {} independent environments, 1 ms public cadence; groups {:?}; aligned state {}, allocated buffers {} bytes; allocator {:?}; first shared setup {mixed_setup:?}",
        info.name,
        info.backend,
        info.driver_info,
        N,
        m.groups(),
        info.state_bytes,
        info.allocated_buffer_bytes,
        m.allocator_usage()
    );
    println!(
        "Host state snapshots, packing buffer and ID-to-offset storage: {} bytes",
        m.host_state_bytes()
    );
    let payload: u64 = m.groups().iter().map(|g| g.bytes).sum();
    println!(
        "profile payload {payload} + {} B binding alignment. One storage slab, one staging slab, three compute dispatches, one queue submission/copy/fence per epoch.",
        info.state_bytes - payload
    );
    sim.tick()?;
    assert_eq!(sim.world().resource::<MixedBatch>().tick(), 0);
    if let Some(c) = &mut archive {
        c.sample(0, &initial);
    }
    let mut scalar = initial.clone();
    let mut dense = Dense::default();
    let mut native = Native::new(&initial);
    let mut digests = Vec::with_capacity(TICKS);
    for k in 1..=TICKS {
        sim.step_exact(1)?;
        let envs = sim.world().resource::<MixedBatch>().environments();
        for (cpu, dev) in scalar.iter_mut().zip(envs) {
            cpu_step(cpu);
            dense.add(dev, cpu);
        }
        native.step(k, envs);
        if let Some(c) = &mut archive {
            c.sample(k, envs);
        }
        digests.push(signature(envs));
    }
    dense.verify();
    native.verify();
    let final_frame = sim.world().resource::<MixedBatch>().environments().to_vec();
    sim.reset()?;
    assert_eq!(sim.world().resource::<MixedBatch>().environments(), initial);
    let setup = Instant::now();
    let mut separate = Separate::new(&adapter, &initial)?;
    let separate_setup = setup.elapsed();
    println!("subsequent setup of three separate single-profile batch devices {separate_setup:?}");
    let mut cpu = initial.clone();
    let mut observed = initial.clone();
    let (mut cpu_time, mut gpu_time, mut groups_time) =
        (Times::default(), Times::default(), Times::default());
    let (mut submit, mut wait, mut decode, mut observe, mut dispatch, mut copy) = (
        Times::default(),
        Times::default(),
        Times::default(),
        Times::default(),
        Times::default(),
        Times::default(),
    );
    for k in 1..=TICKS {
        let start = Instant::now();
        for s in &mut cpu {
            cpu_step(s);
        }
        black_box(signature(&cpu));
        let c = start.elapsed();
        let start = Instant::now();
        sim.step_exact(1)?;
        let batch = sim.world().resource::<MixedBatch>();
        let read = Instant::now();
        let hash = black_box(signature(batch.environments()));
        let read_time = read.elapsed();
        let g = start.elapsed();
        let stats = sim.world().resource::<BatchTiming>().last.unwrap();
        assert_eq!(
            hash,
            digests[k - 1],
            "reset trajectory diverged at tick {k}"
        );
        let start = Instant::now();
        separate.step()?;
        for (i, s) in observed.iter_mut().enumerate() {
            s.state = separate.state(i);
        }
        black_box(signature(&observed));
        let separate_time = start.elapsed();
        // Check exact field bits after every grouping, including signed zeros and
        // copies into the non-homogeneous public order.
        for (a, b) in batch.environments().iter().zip(&observed) {
            assert_eq!(a.id, b.id);
            assert_eq!(
                bytemuck::cast_slice::<f64, u8>(a.state.values()),
                bytemuck::cast_slice::<f64, u8>(b.state.values()),
                "device mismatch at environment {} tick {k}",
                a.id
            );
        }
        if k == 1 {
            println!(
                "reset-upload first tick: pack {:?}, write {:?}, upload {} B, readback {} B",
                stats.pack, stats.upload, stats.uploaded_bytes, stats.readback_bytes
            );
        }
        if k > 100 {
            assert_eq!(stats.uploaded_bytes, 0);
            if let Some(report) = &mut report {
                report.sample(k, g, separate_time, c);
            }
            cpu_time.add(c);
            gpu_time.add(g);
            groups_time.add(separate_time);
            submit.add(stats.encode_submit);
            wait.add(stats.wait_map);
            decode.add(stats.decode);
            observe.add(read_time);
            if let Some(t) = stats.gpu_dispatch {
                dispatch.add(t);
            }
            if let Some(t) = stats.gpu_copy {
                copy.add(t);
            }
        }
    }
    assert_eq!(
        sim.world().resource::<MixedBatch>().environments(),
        final_frame
    );
    assert_eq!(scalar, cpu);
    for (name, mut t) in [
        ("shared mixed GPU incl Bevy and full consumer", gpu_time),
        (
            "three independent GPU devices serial incl gather and full consumer",
            groups_time,
        ),
        ("single-thread scalar incl full consumer", cpu_time),
    ] {
        let seconds = t.total();
        println!(
            "{name}: {:.3} M environment steps/s; {:.3} world sim s/wall s; {:.3} aggregate sim s/wall s",
            (TICKS - 100) as f64 * N as f64 / seconds / 1e6,
            Duration::from_millis((TICKS - 100) as u64).as_secs_f64() / seconds,
            0.001 * (TICKS - 100) as f64 * N as f64 / seconds
        );
        t.report(name);
    }
    for (name, mut t) in [
        ("encode_submit", submit),
        ("fence_map", wait),
        ("decode", decode),
        ("ordered_consumer", observe),
        ("device dispatch", dispatch),
        ("device copy", copy),
    ] {
        t.report(name);
    }
    println!(
        "600/600 frames matched three homogeneous GPU kernels byte for byte. Reset repeat matched every frame digest and full final state. Benchmarks include 100 warmup + 500 observed intervals."
    );
    if let (Some(report), Some(path)) = (report, output) {
        report.write(&path)?;
    }
    if let (Some(c), Some(path)) = (archive, capture) {
        c.write(&path)?;
        println!("published native mixed replay {}", path.display());
    }
    Ok(())
}
