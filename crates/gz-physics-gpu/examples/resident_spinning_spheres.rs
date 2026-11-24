// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare finite-sphere observation barriers with dense GPU ticks and independent
//! continuous-time mechanics. Optional replay contains actual block endpoints.
#[path = "support/sphere_pair/attitude.rs"]
mod attitude;
#[allow(dead_code)]
#[path = "../../gz-physics-distributed/examples/support/primitive_replay_view.rs"]
mod primitive_replay_view;
#[allow(dead_code)]
#[path = "support/sphere_pair/reference.rs"]
mod reference;
use gz_physics_gpu::{
    MAX_BLOCK_TICKS, SpherePairBatch, SpherePairBlockBatch,
    SpherePairBlockObservation as Observation, SpherePairLane as Lane,
};
use primitive_replay_view::View;
use reference::{Q, V, exact_interval, fixture};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

struct Rng(u64);
impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) * (1. / 9007199254740992.)
    }
}
fn population(seed: u64, n: usize) -> Vec<Lane> {
    let mut rng = Rng(seed);
    let mut rows = fixture(n);
    for (i, s) in rows.iter_mut().enumerate() {
        // Rotate the pair as a rigid frame, boost its center and vary its
        // approach speed without changing the impact parameter.
        let axis = V::new(0.5 + rng.unit(), rng.unit(), rng.unit()).normalize();
        let rot = Q::from_axis_angle(axis, 5. * rng.unit());
        let scale = 0.35 + 2.8 * rng.unit();
        let boost = V::new(rng.unit(), rng.unit(), rng.unit()) - V::splat(0.5);
        let offset = V::new(rng.unit(), rng.unit(), rng.unit()) - V::splat(0.5);
        if i % 11 == 6 || i % 11 == 8 {
            // Separate control pairs on either side of the tangent.
            let r = s.a.radius + s.b.radius;
            s.a.position = [0.; 3];
            s.b.position = [0.4, r * (if i % 11 == 6 { 0.999999 } else { 1.000001 }), 0.];
            s.a.velocity = [1., 0., 0.];
            s.b.velocity = [-1., 0., 0.];
        }
        for b in [&mut s.a, &mut s.b] {
            b.position = (rot * V::from(b.position) + offset).to_array();
            b.velocity = (rot * V::from(b.velocity) * scale + boost).to_array();
            b.angular_velocity = (rot * V::from(b.angular_velocity)).to_array();
            b.orientation = (rot * Q::from_array(b.orientation)).normalize().to_array();
        }
        s.gravity = (rot * V::from(s.gravity)).to_array();
        // Use nonzero restitution for these randomized event-time comparisons.
        // A separate dense exercise below includes perfectly inelastic controls.
        s.restitution = 0.04 + 0.96 * rng.unit();
        if rng.unit() < 0.5 {
            std::mem::swap(&mut s.a, &mut s.b);
        }
    }
    rows
}
#[derive(Default)]
struct Errors {
    position: f64,
    velocity: f64,
    spin: f64,
    angle: f64,
    angle_bound_fraction: f64,
    time: f64,
    impulse: f64,
    hits: u64,
    misses: u64,
}
impl Errors {
    fn compare(
        &mut self,
        initial: Lane,
        obs: &Observation,
        seconds: f64,
        seed: u64,
        i: usize,
    ) -> Result<()> {
        let mut exact = initial;
        exact_interval(&mut exact, seconds);
        let j = V::from(exact.contact_normal) * exact.normal_impulse
            + V::from(exact.tangential_impulse);
        self.impulse = self
            .impulse
            .max(j.distance(V::from(obs.impulse_on_b)))
            .max((exact.normal_impulse - obs.maximum_normal_impulse).abs())
            .max((exact.normal_impulse - obs.normal_impulse_sum).abs());
        match (exact.normal_impulse > 0., obs.contact_times()) {
            (false, None) if obs.contact_ticks() == 0 => self.misses += 1,
            (true, Some((t, last))) if obs.contact_ticks() == 1 && t == last => {
                self.hits += 1;
                self.time = self.time.max((t - exact.impact_time).abs());
            }
            _ => {
                return Err(format!(
                    "seed {seed}, lane {i}: {} contacts vs independent impulse {}, {:?}",
                    obs.contact_ticks(),
                    exact.normal_impulse,
                    obs.contact_times()
                )
                .into());
            }
        }
        for (start, x, y) in [
            (initial.a, exact.a, obs.state.a),
            (initial.b, exact.b, obs.state.b),
        ] {
            self.position = self
                .position
                .max(V::from(x.position).distance(V::from(y.position)));
            self.velocity = self
                .velocity
                .max(V::from(x.velocity).distance(V::from(y.velocity)));
            self.spin = self
                .spin
                .max(V::from(x.angular_velocity).distance(V::from(y.angular_velocity)));
            let (a, b) = (Q::from_array(x.orientation), Q::from_array(y.orientation));
            let chord = (a - b).length().min((a + b).length());
            let angle = 4. * (0.5 * chord).min(1.).asin();
            self.angle = self.angle.max(angle);
            // Each normalized midpoint update rotates by 2 atan(h |w| / 2).
            // x - 2 atan(x/2) <= x^3/12, including either side of impact.
            let pre = if exact.impact_time < 0. {
                seconds
            } else {
                exact.impact_time
            };
            let bound = 1e-6 / 12.
                * (pre * V::from(start.angular_velocity).length().powi(3)
                    + (seconds - pre) * V::from(x.angular_velocity).length().powi(3))
                + 2e-8;
            self.angle_bound_fraction = self.angle_bound_fraction.max(angle / bound);
        }
        if self.position > 2e-7
            || self.velocity > 5e-8
            || self.spin > 5e-7
            || self.time > 2e-8
            || self.impulse > 2e-7
            || self.angle_bound_fraction > 1.
        {
            return Err(format!("seed {seed} lane {i}: {}", self.row()).into());
        }
        Ok(())
    }
    fn row(&self) -> Value {
        json!({"hits":self.hits,"misses":self.misses,"position_m":self.position,"velocity_mps":self.velocity,
            "spin_radps":self.spin,"orientation_rad":self.angle,"orientation_error_over_bound":self.angle_bound_fraction,"toi_s":self.time,"impulse_ns":self.impulse})
    }
}
fn seeded(adapter: &str, seed: u64, n: usize, ticks: u32) -> Result<()> {
    let lanes = population(seed, n);
    let mut b = SpherePairBlockBatch::new(adapter, lanes.clone())?;
    let metrics = b.step_block(ticks)?;
    let mut errors = Errors::default();
    for (i, (init, obs)) in lanes.iter().zip(b.lanes()).enumerate() {
        errors.compare(*init, obs, 0.001 * f64::from(ticks), seed, i)?;
    }
    let first = b.lanes().to_vec();
    b.reset();
    b.step_block(ticks)?;
    assert_eq!(
        bytemuck::cast_slice::<_, u8>(b.lanes()),
        bytemuck::cast_slice::<_, u8>(&first)
    );
    println!(
        "{}",
        json!({"seed":seed,"lanes":n,"ticks":ticks,"reset_verified":true,
        "device":b.device_info().name,"latency_us":metrics.latency.as_micros(),"errors":errors.row()})
    );
    Ok(())
}

fn capture(path: &Path, adapter: &str) -> Result<()> {
    let initial = fixture(3);
    let mut block = SpherePairBlockBatch::new(adapter, initial)?;
    let mut frames = Vec::new();
    let mut index = Vec::new();
    let mut native_hits = [None; 3];
    for step in 0..=60 {
        if step > 0 {
            block.step_block(8)?;
        }
        let mut v = View::default();
        let mut readings = Vec::new();
        let tick = block.tick();
        for (i, obs) in block.lanes().iter().enumerate() {
            let origin = V::new(0., 0., (1. - i as f64) * 0.5);
            // Turn approach into the camera's horizontal direction.
            let turn = Q::from_rotation_z(-std::f64::consts::FRAC_PI_2);
            let base = 100 + 40 * i as u64;
            for (k, (label, body, c)) in [
                ("a", obs.state.a, [0.12, 0.69, 0.86, 1.]),
                ("b", obs.state.b, [0.98, 0.59, 0.17, 1.]),
            ]
            .into_iter()
            .enumerate()
            {
                let rot = turn * Q::from_array(body.orientation);
                let center = origin + turn * V::from(body.position);
                let name = format!("pair_{i}_{label}");
                v.add(
                    base + k as u64 * 8,
                    &name,
                    center.to_array(),
                    rot.to_array(),
                    json!({"kind":"sphere","radius":body.radius}),
                    c,
                );
                v.add(
                    base + k as u64 * 8 + 2,
                    &format!("{name}_spin"),
                    (center + rot * V::Y * body.radius).to_array(),
                    rot.to_array(),
                    json!({"kind":"sphere","radius":body.radius*0.18}),
                    [0.95, 0.95, 0.95, 1.],
                );
            }
            if let Some((t, _)) = obs.contact_times() {
                native_hits[i] = Some(((tick - 8) as f64 * 0.001 + t, obs.impulse_on_b));
            }
            let mag = native_hits[i].map_or(0., |(_, j)| V::from(j).length());
            let y = -0.36 + 0.70 * (mag / 4.).min(1.);
            v.cuboid(
                base + 18,
                &format!("pair_{i}_impulse_scale"),
                [0., -0.01, origin.z - 0.20],
                [0.015, 0.7, 0.01],
                [0.48, 0.53, 0.59, 1.],
            );
            v.add(
                base + 20,
                &format!("pair_{i}_impulse"),
                [0., y, origin.z - 0.20],
                [0., 0., 0., 1.],
                json!({"kind":"sphere","radius":0.026}),
                [0.92, 0.29, 0.45, 1.],
            );
            readings.push(json!({"state":obs.state,"block_impulse_on_b":obs.impulse_on_b,"block_contact_times":obs.contact_times(),"last_impact":native_hits[i]}));
        }
        frames.push(v.finish_at(step + 1, tick * 1_000_000));
        index.push(json!({"tick":tick,"pairs":readings}));
    }
    fs::create_dir(path)?;
    gz_sim_cli::export_archive(&path.join("replay"), Default::default(), frames)?;
    fs::write(
        path.join("index.json"),
        serde_json::to_vec(
            &json!({"device":block.device_info().name,"impulse_scale_ns":4.,"ticks_per_observation":8,"frames":index}),
        )?,
    )?;
    Ok(())
}
fn dense_and_timing(adapter: &str, n: usize) -> Result<()> {
    let x = fixture(n);
    let mut dense = SpherePairBatch::new(adapter, x.clone())?;
    let mut block = SpherePairBlockBatch::new(adapter, x)?;
    for stride in [1, 8, 32, 128, 512, MAX_BLOCK_TICKS] {
        let mut duration = 0.;
        let mut db = 0;
        let mut scalar = vec![[0.; 3]; n];
        let mut times = vec![None; n];
        let mut counts = vec![0u32; n];
        for k in 0..stride {
            let m = dense.step()?;
            duration += m.latency.as_secs_f64();
            db += m.readback_bytes;
            for (i, s) in dense.lanes().iter().enumerate() {
                if s.normal_impulse > 0. {
                    let t = k as f64 * 0.001 + s.impact_time;
                    let (_, last) = times[i].get_or_insert((t, t));
                    *last = t;
                    counts[i] += 1;
                }
                for (j, sum) in scalar[i].iter_mut().enumerate() {
                    *sum += s.contact_normal[j] * s.normal_impulse + s.tangential_impulse[j];
                }
            }
        }
        let m = block.step_block(stride)?;
        for (i, (a, b)) in block.lanes().iter().zip(dense.lanes()).enumerate() {
            assert_eq!(
                bytemuck::bytes_of(&a.state),
                bytemuck::bytes_of(b),
                "dense lane {i}"
            );
            assert_eq!(a.contact_ticks(), counts[i]);
            if let (Some((s, t)), Some((u, v))) = (a.contact_times(), times[i]) {
                assert!((s - u).abs().max((t - v).abs()) < 2e-15);
            } else {
                assert_eq!(a.contact_times(), times[i]);
            }
            assert!(V::from(a.impulse_on_b).distance(V::from(scalar[i])) < 1e-12);
        }
        println!(
            "{}",
            json!({"kind":"dense_validation","lanes":n,"stride":stride,"dense_seconds":duration,"block_seconds":m.latency.as_secs_f64(),"dense_readback_bytes":db,"block_readback_bytes":m.readback_bytes,"contacts":counts.into_iter().sum::<u32>()})
        );
    }
    Ok(())
}
fn bench(adapter: &str, n: usize, iterations: u32) -> Result<()> {
    let x = fixture(n);
    let mut block = SpherePairBlockBatch::new(adapter, x.clone())?;
    let mut dense = SpherePairBatch::new(adapter, x.clone())?;
    for stride in [1, 4, 16, 64, 256] {
        for repeat in 0..2 {
            for arm in if repeat == 0 {
                ["dense", "block", "scalar"]
            } else {
                ["scalar", "block", "dense"]
            } {
                let mut cpu = x.clone();
                block.reset();
                dense.reset();
                let mut samples = Vec::new();
                let mut downloads = 0;
                let mut kernel_ns = 0;
                for sample in 0..=iterations {
                    let t = Instant::now();
                    let mut collect = |m: gz_physics_gpu::StepMetrics| {
                        if sample > 0 {
                            downloads += m.readback_bytes;
                            kernel_ns += m.gpu_dispatch.unwrap_or_default().as_nanos();
                        }
                    };
                    match arm {
                        "block" => collect(block.step_block(stride)?),
                        "dense" => {
                            for _ in 0..stride {
                                collect(dense.step()?);
                            }
                        }
                        _ => {
                            for _ in 0..stride {
                                for s in &mut cpu {
                                    reference::analytical(s);
                                }
                                std::hint::black_box(&cpu);
                            }
                        }
                    }
                    if sample > 0 {
                        samples.push(t.elapsed().as_secs_f64());
                    }
                }
                samples.sort_by(f64::total_cmp);
                let sum: f64 = samples.iter().sum();
                println!(
                    "{}",
                    json!({"kind":"timing","device":block.device_info().name,"lanes":n,"stride":stride,"arm":arm,"repeat":repeat,"windows":iterations,
                    "mean_ms":1000.*sum/iterations as f64,"p95_ms":samples[(samples.len()*95).div_ceil(100)-1]*1000.,
                    "environment_ticks_per_s":(n as f64*f64::from(stride)*f64::from(iterations))/sum,
                    "gpu_dispatch_ns":kernel_ns,"readback_bytes":downloads})
                );
            }
        }
    }
    Ok(())
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("need adapter")?;
    match args.next().as_deref() {
        Some("attitude") => attitude::compare(
            &adapter,
            args.next().ok_or("seed")?.parse()?,
            args.next().ok_or("lanes")?.parse()?,
            args.next().ok_or("ticks")?.parse()?,
        ),
        Some("rotor") => attitude::rotor(
            &adapter,
            &PathBuf::from(args.next().ok_or("new view directory")?),
        ),
        Some("seed") => seeded(
            &adapter,
            args.next().ok_or("seed")?.parse()?,
            args.next().ok_or("lanes")?.parse()?,
            args.next().ok_or("ticks")?.parse()?,
        ),
        Some("dense") => dense_and_timing(&adapter, args.next().ok_or("lanes")?.parse()?),
        Some("view") => capture(
            &PathBuf::from(args.next().ok_or("view directory")?),
            &adapter,
        ),
        Some("bench") => bench(
            &adapter,
            args.next().ok_or("lanes")?.parse()?,
            args.next().ok_or("windows")?.parse()?,
        ),
        _ => Err("expected ADAPTER seed|dense|bench|view ...".into()),
    }
}
