// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify and time 2,048 independent oblique f64 sphere/plane environments.
//! cargo run --release -p gz-physics-gpu --example oblique_sphere_plane -- --adapter NAME [--capture NEW_DIRECTORY]
#[path = "support/sphere_plane/capture.rs"]
mod capture;
#[path = "support/sphere_plane/check.rs"]
mod check;
#[path = "support/sphere_plane/reference.rs"]
pub mod reference;
#[path = "support/times.rs"]
mod timing;
use capture::Capture;
use check::{Balance, Errors, NativeAgreement};
use gz_physics_gpu::{
    SpherePlaneBatch, SpherePlaneLane as Lane,
    simulation::{BatchTiming, sphere_plane_simulation},
};
use reference::{Native, analytical, fixture};
use std::{error::Error, hint::black_box, path::PathBuf, time::Instant};
use timing::Times;

const COUNT: usize = 2048;
const TICKS: usize = 600;
const WARMUP: usize = 100;
fn parse() -> Result<(String, Option<PathBuf>), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let mut adapter = None;
    let mut capture = None;
    while let Some(flag) = args.next() {
        let value = args.next().ok_or("missing option value")?;
        match flag.as_str() {
            "--adapter" if adapter.is_none() => adapter = Some(value),
            "--capture" if capture.is_none() => capture = Some(PathBuf::from(value)),
            _ => return Err("expected --adapter NAME [--capture NEW_DIRECTORY]".into()),
        }
    }
    Ok((
        adapter.ok_or("select a physical f64 Vulkan adapter with --adapter NAME")?,
        capture,
    ))
}
fn signature(lanes: &[Lane]) -> u64 {
    // Examine every observed lane and every observable, on CPU and GPU alike.
    let mut acc = 0u64;
    for s in lanes {
        for value in s
            .position
            .iter()
            .chain(&s.velocity)
            .chain(&s.angular_velocity)
            .chain(&s.orientation)
            .chain(std::iter::once(&s.normal_impulse))
            .chain(&s.tangential_impulse)
        {
            acc = acc.rotate_left(7) ^ value.to_bits();
        }
    }
    black_box(acc)
}
fn main() -> Result<(), Box<dyn Error>> {
    let (adapter, output) = parse()?;
    let initial = fixture(COUNT);
    let mut previous = initial.clone();
    let mut scalar = initial.clone();
    let mut sim = sphere_plane_simulation(&adapter, initial.clone())?;
    let info = sim.world().resource::<SpherePlaneBatch>().device_info();
    println!(
        "{} via {} · {} · {} lanes, {} bytes, {} workgroups, CPU {}",
        info.name,
        info.backend,
        info.driver_info,
        COUNT,
        info.state_bytes,
        info.workgroups,
        std::env::consts::ARCH
    );
    let mut native: Vec<_> = (0..64)
        .map(|i| {
            let j = (i * 29) % COUNT;
            (j, Native::new(initial[j]))
        })
        .collect();
    let mut agreement = Errors::default();
    let mut invariants = Balance::default();
    let mut rapier = NativeAgreement::default();
    let mut archive = output.as_ref().map(|_| Capture::default());
    if let Some(c) = archive.as_mut() {
        c.sample(0, &initial);
    }
    for tick in 1..=TICKS {
        sim.step_exact(1)?;
        let gpu = sim.world().resource::<SpherePlaneBatch>().lanes();
        for ((cpu, old), measured) in scalar.iter_mut().zip(&previous).zip(gpu) {
            analytical(cpu);
            agreement.add(measured, cpu);
            invariants.add(old, measured);
        }
        for (j, test) in &mut native {
            test.step();
            rapier.add(tick, &gpu[*j], test);
        }
        if tick % 10 == 0
            && let Some(c) = archive.as_mut()
        {
            c.sample(tick, gpu);
        }
        previous.copy_from_slice(gpu);
    }
    println!("all-lane GPU vs independent directional-mobility CPU maxima {agreement:?}");
    println!("dense impulse/energy checks {invariants:?}");
    println!("64 independent Rapier worlds at 8x finer dt, compared every public tick: {rapier:?}");
    assert!(
        agreement.p < 2e-9
            && agreement.v < 5e-9
            && agreement.w < 2e-7
            && agreement.q < 2e-8
            && agreement.jn < 5e-9
            && agreement.jt < 5e-9,
        "dense analytic agreement"
    );
    assert!(
        invariants.linear < 2e-10
            && invariants.angular < 2e-10
            && invariants.energy_gain < 2e-10
            && invariants.penetration < 2e-10
            && invariants.cone_excess < 2e-10
            && invariants.normal_leak < 2e-10
            && invariants.normal_speed < 2e-10
            && invariants.sticking_slip_m_s < 2e-10
            && invariants.friction_work_gain_j < 2e-10
            && invariants.normal_spin_change_rad_s < 2e-10,
        "impulse, energy, cone or support balance"
    );
    assert!(
        [
            invariants.sliding,
            invariants.sticking,
            invariants.frictionless,
            invariants.twisting
        ]
        .into_iter()
        .all(|n| n > 1000)
    );
    assert!(
        native
            .iter()
            .all(|(_, c)| c.gpu_first.is_some() && c.native_first.is_some()),
        "unexercised native contact"
    );
    // Hard contact is discontinuous. Position is compared throughout; velocity and
    // rim-speed budgets outside impact correspond to about 6 gravity kicks at 1ms.
    assert!(
        rapier.raw.p < 0.012
            && rapier.max_impact_offset_ticks <= 2
            && rapier.outside_impact.v < 0.065
            && rapier.synchronized_surface_omega < 0.065
            && rapier.outside_impact.q < 0.06
            // A one-tick support impulse error above the largest fixture
            // gravity kick would exceed the time-resolution budget.
            && rapier.postimpact_impulse_speed_m_s < 0.011,
        "native trajectory outside finite-step envelope"
    );

    // All correctness gates precede timing. Repeat exactly the same mixed batch.
    sim.reset()?;
    assert_eq!(sim.world().resource::<SpherePlaneBatch>().lanes(), initial);
    let mut cpu = initial.clone();
    let mut cpu_times = Times::default();
    let mut total_board = Times::default();
    let mut pack = Times::default();
    let mut upload = Times::default();
    let mut submit = Times::default();
    let mut wait = Times::default();
    let mut decode = Times::default();
    let mut observe = Times::default();
    let mut dispatch = Times::default();
    let mut copy = Times::default();
    let mut cold = None;
    let mut readback_bytes = 0;
    let mut upload_bytes = 0;
    for tick in 1..=TICKS {
        let start = Instant::now();
        for lane in &mut cpu {
            analytical(lane);
        }
        signature(&cpu);
        if tick > WARMUP {
            cpu_times.add(start.elapsed());
        }
    }
    for tick in 1..=TICKS {
        let start = Instant::now();
        sim.step_exact(1)?;
        let m = sim.world().resource::<BatchTiming>().last.unwrap();
        let observation = Instant::now();
        signature(sim.world().resource::<SpherePlaneBatch>().lanes());
        let host_observe = observation.elapsed();
        if tick == 1 {
            cold = Some(m);
        }
        if tick > WARMUP {
            total_board.add(start.elapsed());
            pack.add(m.pack);
            upload.add(m.upload);
            submit.add(m.encode_submit);
            wait.add(m.wait_map);
            decode.add(m.decode);
            observe.add(host_observe);
            if let Some(t) = m.gpu_dispatch {
                dispatch.add(t);
            }
            if let Some(t) = m.gpu_copy {
                copy.add(t);
            }
            readback_bytes += m.readback_bytes;
            upload_bytes += m.uploaded_bytes;
        }
    }
    assert_eq!(
        sim.world().resource::<SpherePlaneBatch>().lanes(),
        previous,
        "device repeatability after reset"
    );
    assert_eq!(cpu, scalar);
    let first = cold.unwrap();
    println!(
        "cold/reseed pack {:?}, upload {:?}, {} bytes; timed uploads {upload_bytes} bytes, dense readback {readback_bytes} bytes",
        first.pack, first.upload, first.uploaded_bytes
    );
    for (name, times) in [
        ("CPU scalar incl dense observations", &mut cpu_times),
        ("GPU Bevy/fence/readback/observation", &mut total_board),
        ("GPU host pack", &mut pack),
        ("GPU host upload", &mut upload),
        ("GPU host encode and submit", &mut submit),
        ("GPU host fence and map", &mut wait),
        ("GPU host decode", &mut decode),
        ("GPU dense observation", &mut observe),
        ("device dispatch", &mut dispatch),
        ("device copy", &mut copy),
    ] {
        times.report(name);
    }
    let amount = (COUNT * (TICKS - WARMUP)) as f64;
    println!(
        "measured lane-steps/s scalar {:.0} versus GPU {:.0}; Vulkan allocated/reserved bytes {:?}",
        amount / cpu_times.total(),
        amount / total_board.total(),
        sim.world().resource::<SpherePlaneBatch>().allocator_usage()
    );
    if let (Some(c), Some(path)) = (archive, output) {
        c.write(&path)?;
        println!(
            "saved initial to 600 ms, six observed lanes, 10 ms interval to {}",
            path.display()
        );
    }
    Ok(())
}
