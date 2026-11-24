// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent Rapier and Vulkan trials for a three-prismatic-joint lift.
use gz_physics_gpu::{ParallelGripperBatch, ParallelGripperLane};
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path, time::Instant};
#[path = "support/parallel_gripper/native.rs"]
mod native;

fn fixture(mass: f64, mu: f64, open: bool) -> ParallelGripperLane {
    ParallelGripperLane {
        position: [0.028, -0.05, 0.05],
        speed: [0.; 3],
        mass: [0.38, 0.12, 0.12],
        half_size: [[0.006, 0.018]; 2],
        ball: [0., 0.022],
        velocity: [0.; 2],
        angle: 0.,
        spin: 0.,
        radius: 0.022,
        ball_mass: mass,
        gravity: 9.81,
        ground: 0.,
        target: if open {
            [0.028, -0.05, 0.05]
        } else {
            [0.028, -0.022, 0.022]
        },
        stiffness: [270., 1100., 1100.],
        damping: [23., 16., 16.],
        force_limit: [25., 9., 9.],
        friction: [mu, mu, 0.3],
        normal_impulse: [0.; 3],
        tangent_impulse: [0.; 3],
        support_impulse: [0.; 2],
        lower: [0.02, -0.07, 0.015],
        upper: [0.22, -0.015, 0.07],
    }
}
fn command(tick: usize, b: &mut ParallelGripperBatch) -> Result<(), Box<dyn Error>> {
    if matches!(tick, 201 | 651) {
        let targets: Vec<_> = b
            .lanes()
            .iter()
            .map(|l| {
                if tick == 201 {
                    [0.15, l.target[1], l.target[2]]
                } else {
                    [0.15, -0.05, 0.05]
                }
            })
            .collect();
        for (i, t) in targets.into_iter().enumerate() {
            b.set_targets(i, t)?;
        }
    }
    Ok(())
}
fn poses(tick: usize, l: &ParallelGripperLane) -> Value {
    let pose = |x: f64, y: f64, h: f64, a: f64| {
        let (s, c) = (-a * 0.5).sin_cos();
        json!({"translation":[x,y,h],"rotation":[0.,s,0.,c]})
    };
    let [h, left, right] = l.position;
    let [x, z] = l.ball;
    json!({"time_ns":tick*1_000_000,"poses":{
        "floor":pose(0.,0.,-0.009,0.),"carriage":pose(0.,0.,h+0.08,0.),
        "left":pose(left,0.,h,0.),"right":pose(right,0.,h,0.),
        "left_stem":pose(left,0.,h+0.044,0.),"right_stem":pose(right,0.,h+0.044,0.),
        "sphere":pose(x,0.,z,l.angle),"marker":pose(x+0.014*l.angle.cos(),0.015,z+0.014*l.angle.sin(),l.angle)
    }})
}
#[derive(Default)]
struct Trial {
    first: [Option<usize>; 2],
    last_contact: [Option<usize>; 2],
    first_normal: [[f64; 2]; 2],
    landing: [Option<usize>; 2],
    max_clearance: [f64; 2],
    min_held_clearance: [f64; 2],
    max_error: f64,
    max_jaw_error: f64,
    min_jaw_ns: [f64; 2],
    average_jaw_n: [f64; 2],
    average_native_up_n: f64,
    max_balance: f64,
    support_ticks: usize,
    max_floor_hold: [f64; 2],
    max_friction_excess: f64,
    total_normal: [[f64; 2]; 2],
    native_us: Vec<f64>,
}
fn p95(x: &[f64]) -> f64 {
    let mut x = x.to_vec();
    x.sort_by(f64::total_cmp);
    x[(95 * x.len()).div_ceil(100) - 1]
}
fn run(adapter: &str, dir: &Path) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(dir)?;
    let input = [
        fixture(0.08, 0.6, false),
        fixture(0.08, 0., false),
        fixture(0.08, 0.6, true),
        fixture(0.20, 0.6, false),
        fixture(0.35, 0.6, false),
        fixture(0.35, 0.3, false),
        fixture(0.20, 0.3, false),
        fixture(0.20, 0.15, false),
        fixture(0.08, 0.15, false),
    ];
    let mut b = ParallelGripperBatch::new(adapter, input.to_vec())?;
    let mut oracles = input.each_ref().map(|l| native::Native::new(l, 16));
    let mut trial = input.map(|_| Trial {
        min_held_clearance: [f64::INFINITY; 2],
        min_jaw_ns: [f64::INFINITY; 2],
        ..Default::default()
    });
    let mut hist = Vec::new();
    let mut visuals = vec![poses(0, &input[0])];
    let mut zero_visuals = vec![poses(0, &input[1])];
    let mut gpu_us = Vec::new();
    let mut previous = input.map(|l| l.velocity[1]);
    for tick in 1..=1000 {
        command(tick, &mut b)?;
        let start = Instant::now();
        b.step()?;
        gpu_us.push(start.elapsed().as_secs_f64() * 1e6);
        hist.push(b.lanes().to_vec());
        visuals.push(poses(tick, &b.lanes()[0]));
        zero_visuals.push(poses(tick, &b.lanes()[1]));
        for (i, l) in b.lanes().iter().enumerate() {
            let t = &mut trial[i];
            let native = &mut oracles[i];
            let now = Instant::now();
            let (native_n, native_support) = native.step(l);
            t.native_us.push(now.elapsed().as_secs_f64() * 1e6);
            let (q, ball) = native.state();
            for (engine, (n, z)) in [(l.normal_impulse, l.ball[1]), (native_n, ball[1])]
                .into_iter()
                .enumerate()
            {
                if n[0].min(n[1]) > 1e-7 && t.first[engine].is_none() {
                    t.first[engine] = Some(tick);
                    t.first_normal[engine] = [n[0], n[1]];
                }
                if n[0].max(n[1]) > 1e-7 {
                    t.last_contact[engine] = Some(tick);
                }
                if !n[..2].iter().any(|&v| v > 1e-7)
                    && t.max_clearance[engine] > 0.001
                    && n[2] > 1e-6
                {
                    t.landing[engine].get_or_insert(tick);
                }
                let clearance = z - l.radius - l.ground;
                t.max_clearance[engine] = t.max_clearance[engine].max(clearance);
                if (500..=640).contains(&tick) {
                    t.min_held_clearance[engine] = t.min_held_clearance[engine].min(clearance);
                }
                for (jaw, v) in n[..2].iter().enumerate() {
                    t.total_normal[engine][jaw] += v;
                }
            }
            t.max_error = t
                .max_error
                .max((l.ball[0] - ball[0]).hypot(l.ball[1] - ball[1]));
            t.max_jaw_error = t.max_jaw_error.max(
                l.position
                    .into_iter()
                    .zip(q)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0., f64::max),
            );
            let up = l.support_impulse.iter().sum::<f64>();
            t.max_balance = t.max_balance.max(
                (l.ball_mass * (l.velocity[1] - previous[i] + l.gravity * 0.001)
                    - up
                    - l.normal_impulse[2])
                    .abs(),
            );
            previous[i] = l.velocity[1];
            for j in 0..3 {
                t.max_friction_excess = t
                    .max_friction_excess
                    .max(l.tangent_impulse[j].abs() - l.friction[j] * l.normal_impulse[j]);
            }
            if (500..=640).contains(&tick) {
                for j in 0..2 {
                    t.min_jaw_ns[j] = t.min_jaw_ns[j].min(l.normal_impulse[j]);
                }
                for j in 0..2 {
                    t.average_jaw_n[j] += l.support_impulse[j] / 0.001;
                }
                t.average_native_up_n += native_support / 0.001;
                t.max_floor_hold[0] = t.max_floor_hold[0].max(l.normal_impulse[2]);
                t.max_floor_hold[1] = t.max_floor_hold[1].max(native_n[2]);
                t.support_ticks += 1;
            }
        }
    }
    b.reset();
    for (i, states) in hist.iter().enumerate() {
        command(i + 1, &mut b)?;
        b.step()?;
        assert_eq!(
            bytemuck::cast_slice::<_, u8>(states),
            bytemuck::cast_slice::<_, u8>(b.lanes())
        );
    }
    let mut matched = input.each_ref().map(|l| native::Native::new(l, 4));
    let mut native_us = Vec::new();
    for states in &hist {
        let now = Instant::now();
        for (world, lane) in matched.iter_mut().zip(states) {
            world.step(lane);
        }
        native_us.push(now.elapsed().as_secs_f64() * 1e6);
    }
    for i in [0, 3, 4] {
        let t = &trial[i];
        assert_eq!(t.first, [Some(29); 2]);
        assert_eq!(t.last_contact, [Some(650); 2]);
        assert_eq!(t.max_floor_hold, [0.; 2]);
        assert!(t.min_held_clearance.iter().all(|&z| z > 0.100));
        assert!(t.max_error < if i == 4 { 0.002 } else { 0.001 });
        assert!(t.landing[0].unwrap().abs_diff(t.landing[1].unwrap()) <= 1);
        let measured = t.average_jaw_n.iter().sum::<f64>() / t.support_ticks as f64;
        assert!((measured - input[i].ball_mass * input[i].gravity).abs() < 0.015);
        assert!((measured - t.average_native_up_n / t.support_ticks as f64).abs() < 0.003);
    }
    for i in [1, 2] {
        assert!(trial[i].max_clearance.iter().all(|&z| z < 1e-6));
        assert!(trial[i].landing.iter().all(Option::is_none));
    }
    assert!(
        trial
            .iter()
            .all(|t| t.max_balance < 1e-11 && t.max_friction_excess < 1e-12)
    );
    let mut sensitivity = Vec::new();
    for substeps in [4, 16, 32, 64, 128, 256] {
        let i = input.len() - 1;
        let mut reference = native::Native::new(&input[i], substeps);
        let mut held = f64::INFINITY;
        let mut max_error = 0.0_f64;
        for (step, states) in hist.iter().enumerate() {
            let lane = &states[i];
            reference.step(lane);
            let ball = reference.state().1;
            if (500..=640).contains(&(step + 1)) {
                held = held.min(ball[1] - lane.radius - lane.ground);
            }
            max_error = max_error.max((lane.ball[0] - ball[0]).hypot(lane.ball[1] - ball[1]));
        }
        sensitivity.push(json!({"native_dt_us":1000./substeps as f64,"native_minimum_hold_clearance_m":held,"max_device_native_path_error_m":max_error}));
    }
    let rows: Vec<_> = trial.iter().zip(&input).enumerate().map(|(i, (t, lane))| {
        let held = t.support_ticks as f64;
        json!({
            "mass_kg": lane.ball_mass,
            "mu": lane.friction[0],
            "starts_open": i == 2,
            "first_grip_ms": t.first,
            "first_normal_ns": t.first_normal,
            "fully_released_ms": t.last_contact.map(|last| last.filter(|&k| k < 1000).map(|k| k + 1)),
            "post_release_landing_ms": t.landing,
            "max_clearance_m": t.max_clearance,
            "minimum_hold_clearance_m": t.min_held_clearance,
            "max_object_path_error_m": t.max_error,
            "max_slider_error_m": t.max_jaw_error,
            "hold_minimum_jaw_normal_ns": t.min_jaw_ns,
            "hold_mean_jaw_vertical_force_n": t.average_jaw_n.map(|n| n / held),
            "native_mean_total_jaw_vertical_force_n": t.average_native_up_n / held,
            "hold_max_floor_ns": t.max_floor_hold,
            "sphere_balance_ns": t.max_balance,
            "cone_excess_ns": t.max_friction_excess,
            "total_normal_gpu_native_ns": t.total_normal,
            "native_reference_p95_us": p95(&t.native_us)
        })
    }).collect();
    let d = b.device_info();
    let report = json!({
        "adapter": d.name,
        "driver": d.driver_info,
        "precision": "f64",
        "ticks": 1000,
        "public_dt_ms": 1,
        "gpu_dt_us": 250,
        "native_reference_dt_us": 62.5,
        "near_slip_80g_mu015_native_resolution": sensitivity,
        "lane_count": input.len(),
        "gpu_readback_p95_us": p95(&gpu_us),
        "matched_native_serial_population_p95_us": p95(&native_us),
        "matched_native_dt_us": 250,
        "rows": rows
    });
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    fs::write(dir.join("source-poses.json"), serde_json::to_vec(&visuals)?)?;
    fs::write(
        dir.join("zero-poses.json"),
        serde_json::to_vec(&zero_visuals)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if let [adapter, target] = args.as_slice() {
        return run(adapter, Path::new(target));
    }
    Err("expected explicit Vulkan adapter name and receipt directory".into())
}
