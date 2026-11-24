// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify swept finite-mass, finite-inertia spinning sphere pairs in free space.
//! cargo run --release -p gz-physics-gpu --example spinning_spheres -- --adapter NAME [--capture NEW_DIR]
#[path = "support/sphere_pair/capture.rs"]
pub mod capture;
#[path = "support/sphere_pair/check.rs"]
mod check;
#[path = "support/sphere_pair/reference.rs"]
pub mod reference;
#[path = "support/times.rs"]
mod timing;
use capture::Capture;
use check::{Balance, Errors, NativeAgreement};
use gz_physics_gpu::{
    SpherePairBatch, SpherePairLane as Lane,
    simulation::{BatchTiming, sphere_pair_simulation},
};
use reference::{Native, analytical, fixture};
use std::{error::Error, hint::black_box, path::PathBuf, time::Instant};
use timing::Times;
const N: usize = 2048;
const TICKS: usize = 600;
fn signature(s: &[Lane]) -> u64 {
    let mut h = 0u64;
    for lane in s {
        for x in bytemuck::cast_slice::<Lane, f64>(std::slice::from_ref(lane)) {
            h = h.rotate_left(7) ^ x.to_bits();
        }
    }
    black_box(h)
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let (mut adapter, mut out) = (None, None);
    while let Some(x) = args.next() {
        let v = args.next().ok_or("missing option value")?;
        match x.as_str() {
            "--adapter" if adapter.is_none() => adapter = Some(v),
            "--capture" if out.is_none() => out = Some(PathBuf::from(v)),
            _ => return Err("expected --adapter NAME [--capture NEW_DIR]".into()),
        }
    }
    let name = adapter.ok_or("select --adapter NAME")?;
    let initial = fixture(N);
    let mut cpu = initial.clone();
    let mut prior = initial.clone();
    let mut sim = sphere_pair_simulation(&name, initial.clone())?;
    let device = sim.world().resource::<SpherePairBatch>().device_info();
    println!(
        "{} {} {} | {} independent two-body lanes, {} bytes, {} workgroups",
        device.name, device.backend, device.driver_info, N, device.state_bytes, device.workgroups
    );
    let mut native: Vec<_> = (0..64)
        .map(|k| {
            let i = k * 29;
            (i, Native::new(initial[i]))
        })
        .collect();
    let mut contact = vec![0; N];
    let mut agreement = Errors::default();
    let mut balance = Balance::default();
    let mut rapier = NativeAgreement::default();
    let mut archive = out.as_ref().map(|_| Capture::default());
    if let Some(c) = &mut archive {
        c.observe(0, &initial);
    }
    for t in 1..=TICKS {
        sim.step_exact(1)?;
        let states = sim.world().resource::<SpherePairBatch>().lanes();
        for (i, s) in states.iter().enumerate() {
            analytical(&mut cpu[i]);
            assert_eq!(
                s.normal_impulse > 1e-8,
                cpu[i].normal_impulse > 1e-8,
                "lane {i}, tick {t}, GPU time {} impulse {} vs CPU time {} impulse {}; previous GPU {:?}, initial {:?}",
                s.impact_time,
                s.normal_impulse,
                cpu[i].impact_time,
                cpu[i].normal_impulse,
                prior[i],
                initial[i]
            );
            agreement.dense(s, &cpu[i]);
            balance.add(&prior[i], s);
            if s.normal_impulse > 1e-8 {
                contact[i] += 1;
            }
        }
        for (i, n) in &mut native {
            n.step();
            rapier.add(t, &states[*i], n);
        }
        if let Some(c) = &mut archive {
            c.observe(t, states);
        }
        prior.copy_from_slice(states);
    }
    println!("analytic quadratic/directional-mobility GPU agreement {agreement:?}");
    println!(
        "pair momentum, angular momentum, point impulse, contact energy and Coulomb checks {balance:?}"
    );
    println!(
        "native f64 Rapier at 16x finer dt, full + synchronized + common-friction-law errors {rapier:?}"
    );
    for (i, &count) in contact.iter().enumerate() {
        assert_eq!(
            count,
            usize::from(i % 8 != 7),
            "lane {i} physical contact count"
        );
    }
    assert!(
        agreement.motion.p < 2e-9
            && agreement.motion.v < 1e-8
            && agreement.motion.w < 2e-7
            && agreement.motion.q < 2e-8
            && agreement.impulse < 1e-8
            && agreement.normal < 2e-8
            && agreement.toi < 2e-10
    );
    assert!(
        balance.momentum < 1e-10
            && balance.angular < 1e-10
            && balance.energy_gain < 1e-10
            && balance.impact_energy_residual < 1e-10
            && balance.point_impulse < 1e-10
            && balance.free_com < 1e-10
            && balance.gap_violation < 1e-10
            && balance.restitution < 1e-10
            && balance.slip_sticking < 1e-10
            && balance.cone_excess < 1e-10
    );
    assert!(balance.sliding > 128 && balance.sticking > 128 && balance.impacts == 7 * N / 8);
    for (i, n) in &native {
        assert_eq!(n.native_impact.is_some(), i % 8 != 7);
        assert_eq!(n.native_impact.is_some(), n.device_impact.is_some());
    }
    // Rapier's pinned solver applies restitution after friction. Sliding friction
    // can differ by the exact difference between the two caps; track full raw
    // errors and qualify the remaining geometric/time integration discrepancy.
    assert!(
        rapier.max_offset <= 1
            && rapier.capped_velocity_residual < 0.025
            && rapier.capped_rim_residual < 0.040
            && rapier.capped_position_residual < 0.009
            && rapier.equal_law.q < 0.05
            && rapier.mixed_sliding > 0
            && rapier.matched_impact == 56
    );
    sim.reset()?;
    let mut repeat = initial.clone();
    let mut board = Times::default();
    let mut serial = Times::default();
    let mut device_time = Times::default();
    let mut copying = Times::default();
    let mut wait = Times::default();
    let mut host_decode = Times::default();
    let mut observe = Times::default();
    let mut submit = Times::default();
    for t in 1..=TICKS {
        let start = Instant::now();
        for s in &mut repeat {
            analytical(s);
        }
        signature(&repeat);
        if t > 100 {
            serial.add(start.elapsed());
        }
    }
    let (mut transferred, mut uploaded) = (0, 0);
    for t in 1..=TICKS {
        let start = Instant::now();
        sim.step_exact(1)?;
        let m = sim.world().resource::<BatchTiming>().last.unwrap();
        let obs = Instant::now();
        signature(sim.world().resource::<SpherePairBatch>().lanes());
        let host = obs.elapsed();
        if t == 1 {
            println!(
                "cold state {} bytes, pack {:?}, upload {:?}",
                m.uploaded_bytes, m.pack, m.upload
            );
        }
        if t > 100 {
            board.add(start.elapsed());
            submit.add(m.encode_submit);
            wait.add(m.wait_map);
            host_decode.add(m.decode);
            observe.add(host);
            if let Some(v) = m.gpu_dispatch {
                device_time.add(v);
            }
            if let Some(v) = m.gpu_copy {
                copying.add(v);
            }
            transferred += m.readback_bytes;
            uploaded += m.uploaded_bytes;
        }
    }
    assert_eq!(
        bytemuck::cast_slice::<Lane, u8>(sim.world().resource::<SpherePairBatch>().lanes()),
        bytemuck::cast_slice::<Lane, u8>(&prior)
    );
    assert_eq!(repeat, cpu);
    for (title, t) in [
        ("serial scalar incl observations", &mut serial),
        ("GPU Bevy submit/readback/observations", &mut board),
        ("host submit", &mut submit),
        ("host wait and map", &mut wait),
        ("host decode", &mut host_decode),
        ("host consume", &mut observe),
        ("device kernel", &mut device_time),
        ("device copy", &mut copying),
    ] {
        t.report(title);
    }
    let count = (N * (TICKS - 100)) as f64;
    println!(
        "{} timed pair-ticks, {} B uploaded and {} B dense readback, {:.0} CPU and {:.0} GPU pair-ticks/s",
        count,
        uploaded,
        transferred,
        count / serial.total(),
        count / board.total()
    );
    if let (Some(a), Some(path)) = (archive, out) {
        a.write(&path)?;
        println!("first three GPU pair traces saved to {}", path.display());
    }
    Ok(())
}
