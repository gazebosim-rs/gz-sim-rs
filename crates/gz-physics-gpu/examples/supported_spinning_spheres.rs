// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify simultaneous sphere-pair/plane contact on the invariant, frictionless support family.
//! cargo run --release -p gz-physics-gpu --example supported_spinning_spheres -- --adapter NAME --out NEW_DIRECTORY [--capture NEW_REPLAY]
#[path = "support/sphere_pair/capture.rs"]
pub mod capture;
#[path = "support/sphere_pair/check.rs"]
mod check;
#[path = "support/supported_pair/convergence.rs"]
mod convergence;
#[path = "support/supported_pair/fixture.rs"]
mod fixture;
#[path = "support/sphere_pair/reference.rs"]
pub mod reference;
#[path = "support/times.rs"]
mod timing;
use capture::Capture;
use check::{Balance, Errors, NativeAgreement};
use fixture::{cpu_step, fixture, tangent_gravity};
use gz_physics_gpu::{
    SupportedSpherePairBatch, SupportedSpherePairLane as Lane,
    simulation::{BatchTiming, supported_sphere_pair_simulation},
};
use reference::{Native, V};
use std::{error::Error, fs::File, hint::black_box, path::PathBuf, time::Instant};
use timing::Times;
const COUNT: usize = 2048;
const TICKS: usize = 600;
fn consume(lanes: &[Lane]) {
    let mut h = 0u64;
    for x in bytemuck::cast_slice::<Lane, f64>(lanes) {
        h = h.rotate_left(7) ^ x.to_bits();
    }
    black_box(h);
}
#[derive(Default, Debug)]
struct Support {
    external_linear: f64,
    normal_velocity: f64,
    tangent_spin: f64,
    surface_gap: f64,
    external_work: f64,
    per_tick_weight_error: f64,
    simultaneous_events: usize,
    max_native_step_impulse: f64,
    max_native_integrated_weight: f64,
}
impl Support {
    fn add(&mut self, old: &Lane, new: &Lane, expected: &Lane) {
        let n = V::from(new.plane_normal);
        let g = V::from(new.pair.gravity);
        let mut p = V::ZERO;
        for (i, (a, b)) in [(old.pair.a, new.pair.a), (old.pair.b, new.pair.b)]
            .into_iter()
            .enumerate()
        {
            let j = new.support_normal_impulses[i];
            assert!(j > 0.);
            p += (V::from(b.velocity) - V::from(a.velocity) - g * 0.001) / a.inverse_mass - n * j;
            self.normal_velocity = self.normal_velocity.max(V::from(b.velocity).dot(n).abs());
            let w = V::from(b.angular_velocity);
            self.tangent_spin = self.tangent_spin.max((w - w.dot(n) * n).length());
            self.surface_gap = self
                .surface_gap
                .max((V::from(b.position).dot(n) - new.plane_offset - b.radius).abs());
            self.external_work = self
                .external_work
                .max((V::from(b.position) - V::from(a.position)).dot(n).abs() * j / 0.001);
            self.per_tick_weight_error = self
                .per_tick_weight_error
                .max((j - expected.support_normal_impulses[i]).abs());
        }
        self.external_linear = self.external_linear.max(p.length());
        if new.pair.normal_impulse > 1e-8 {
            self.simultaneous_events += 1;
        }
    }
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let (mut adapter, mut out, mut capture) = (None, None, None);
    while let Some(f) = args.next() {
        let v = args.next().ok_or("missing value")?;
        match f.as_str() {
            "--adapter" if adapter.is_none() => adapter = Some(v),
            "--out" if out.is_none() => out = Some(PathBuf::from(v)),
            "--capture" if capture.is_none() => capture = Some(PathBuf::from(v)),
            _ => {
                return Err(
                    "expected --adapter NAME --out NEW_DIRECTORY [--capture NEW_REPLAY]".into(),
                );
            }
        }
    }
    let adapter = adapter.ok_or("missing explicit hardware --adapter")?;
    let out = out.ok_or("missing --out")?;
    std::fs::create_dir(&out)?;
    let initial = fixture(COUNT);
    let mut cpu = initial.clone();
    let mut previous = initial.clone();
    let mut sim = supported_sphere_pair_simulation(&adapter, initial.clone())?;
    let info = sim
        .world()
        .resource::<SupportedSpherePairBatch>()
        .device_info();
    println!(
        "{} via {} {} | {} supported pairs ({} finite bodies), {} resident bytes",
        info.name,
        info.backend,
        info.driver_info,
        COUNT,
        2 * COUNT,
        info.state_bytes
    );
    let mut native: Vec<_> = (0..64)
        .map(|j| {
            let i = j * 29;
            let s = initial[i];
            (
                i,
                Native::with_support(s.pair, V::from(s.plane_normal), s.plane_offset, 16),
                [0.; 2],
            )
        })
        .collect();
    let mut errors = Errors::default();
    let mut balances = Balance::default();
    let mut support = Support::default();
    let mut rapier = NativeAgreement::default();
    let mut hist = vec![0; COUNT];
    let mut archive = capture.as_ref().map(|_| Capture::default());
    if let Some(c) = &mut archive {
        c.supported(0, &initial);
    }
    for tick in 1..=TICKS {
        sim.step_exact(1)?;
        let states = sim.world().resource::<SupportedSpherePairBatch>().lanes();
        for (i, s) in states.iter().enumerate() {
            cpu_step(&mut cpu[i]);
            errors.dense(&s.pair, &cpu[i].pair);
            support.add(&previous[i], s, &cpu[i]);
            let mut old = previous[i].pair;
            old.gravity = tangent_gravity(&previous[i]).to_array();
            balances.add(&old, &s.pair);
            if s.pair.normal_impulse > 1e-8 {
                hist[i] += 1;
            }
        }
        for (i, model, summed) in &mut native {
            model.step();
            rapier.add(tick, &states[*i].pair, model);
            for ((total, native_j), gpu_j) in summed
                .iter_mut()
                .zip(model.support_impulses)
                .zip(states[*i].support_normal_impulses)
            {
                let error = native_j - gpu_j;
                *total += error;
                support.max_native_step_impulse = support.max_native_step_impulse.max(error.abs());
                support.max_native_integrated_weight =
                    support.max_native_integrated_weight.max(total.abs());
            }
        }
        if let Some(c) = &mut archive {
            c.supported(tick, states);
        }
        previous.copy_from_slice(states);
    }
    println!(
        "analytic tangent-space CPU {errors:?}\n2-body internal momentum/energy {balances:?}\nexternal static support {support:?}\nrefined 3-collider Rapier {rapier:?}"
    );
    for (i, v) in hist.into_iter().enumerate() {
        assert_eq!(v, usize::from(i % 8 != 7), "supported impacts, lane {i}");
    }
    assert!(
        errors.motion.p < 2e-8
            && errors.motion.v < 2e-8
            && errors.motion.w < 2e-7
            && errors.motion.q < 2e-8
            && errors.impulse < 2e-8
    );
    assert!(
        support.external_linear < 2e-10
            && support.surface_gap < 2e-10
            && support.normal_velocity < 2e-10
            && support.tangent_spin < 2e-10
            && support.external_work < 2e-10
            && support.per_tick_weight_error < 2e-10
            && support.simultaneous_events == 7 * COUNT / 8
    );
    assert!(
        balances.angular < 2e-10
            && balances.energy_gain < 2e-10
            && balances.cone_excess < 2e-10
            && balances.impact_energy_residual < 2e-10
            && balances.slip_sticking < 2e-10
            && balances.gap_violation < 2e-10
    );
    for (i, n, _) in &native {
        assert_eq!(n.native_impact.is_some(), i % 8 != 7);
    }
    assert!(
        rapier.max_offset <= 1
            && rapier.capped_velocity_residual < 0.025
            && rapier.capped_rim_residual < 0.040
            && rapier.capped_position_residual < 0.009
            && rapier.equal_law.q < 0.05
            && rapier.mixed_sliding > 0
            && rapier.matched_impact == 56
    );
    // Integrated native support error below a tenth of a nominal 1 ms load for
    // the largest mass. This includes the native manifold's startup settlement.
    assert!(support.max_native_integrated_weight < 0.003);
    convergence::run(&adapter, &mut File::create(out.join("convergence.csv"))?)?;
    convergence::plot(&out)?;
    sim.reset()?;
    let mut scalar = initial;
    let mut cpu_times = Times::default();
    let mut board = Times::default();
    let mut submit = Times::default();
    let mut wait = Times::default();
    let mut decode = Times::default();
    let mut observe = Times::default();
    let mut compute = Times::default();
    let mut copy = Times::default();
    let mut bytes = 0;
    for tick in 1..=TICKS {
        let start = Instant::now();
        for s in &mut scalar {
            cpu_step(s);
        }
        consume(&scalar);
        if tick > 100 {
            cpu_times.add(start.elapsed());
        }
    }
    for tick in 1..=TICKS {
        let start = Instant::now();
        sim.step_exact(1)?;
        let m = sim.world().resource::<BatchTiming>().last.unwrap();
        let consuming = Instant::now();
        consume(sim.world().resource::<SupportedSpherePairBatch>().lanes());
        let consumed = consuming.elapsed();
        if tick == 1 {
            println!(
                "reseed {} bytes, pack {:?}, upload {:?}",
                m.uploaded_bytes, m.pack, m.upload
            );
        }
        if tick > 100 {
            assert_eq!(m.uploaded_bytes, 0);
            board.add(start.elapsed());
            submit.add(m.encode_submit);
            wait.add(m.wait_map);
            decode.add(m.decode);
            observe.add(consumed);
            if let Some(v) = m.gpu_dispatch {
                compute.add(v);
            }
            if let Some(v) = m.gpu_copy {
                copy.add(v);
            }
            bytes += m.readback_bytes;
        }
    }
    assert_eq!(
        bytemuck::cast_slice::<Lane, u8>(
            sim.world().resource::<SupportedSpherePairBatch>().lanes()
        ),
        bytemuck::cast_slice::<Lane, u8>(&previous)
    );
    assert_eq!(scalar, cpu);
    for (label, t) in [
        ("serial CPU + dense consume", &mut cpu_times),
        ("GPU + Bevy/fence/readback/consume", &mut board),
        ("host submit", &mut submit),
        ("wait/map", &mut wait),
        ("decode", &mut decode),
        ("host observation", &mut observe),
        ("device dispatch", &mut compute),
        ("device copy", &mut copy),
    ] {
        t.report(label);
    }
    let count = (COUNT * (TICKS - 100)) as f64;
    println!(
        "{} measured pair-ticks: {} B read, {:.0} CPU versus {:.0} GPU pair-ticks/s",
        count,
        bytes,
        count / cpu_times.total(),
        count / board.total()
    );
    if let (Some(c), Some(path)) = (archive, capture) {
        c.write(&path)?;
        println!("saved supported pair archive to {}", path.display());
    }
    Ok(())
}
