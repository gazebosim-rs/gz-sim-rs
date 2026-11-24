// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a GPU horizontal transfer against an independent four-slider native world.
use gz_physics_gpu::{CarryGripperBatch, CarryGripperLane};
use serde_json::json;
use std::{error::Error, fs, path::Path, time::Instant};
#[path = "support/carry_gripper/eccentric.rs"]
mod eccentric;
#[path = "support/carry_gripper/fixture.rs"]
mod fixture;
#[path = "support/carry_gripper/native.rs"]
mod native;
#[derive(Default)]
struct Run {
    dwell: usize,
    open: Option<usize>,
    first: Option<usize>,
    shelf_first: Option<usize>,
    land: Option<usize>,
    loaded: usize,
    min_carry: f64,
    open_state: Option<[f64; 3]>,
    final_ball: [f64; 2],
    final_shelf: f64,
    floor_load_sum: f64,
    shelf_load_sum: f64,
}
impl Run {
    fn sample(
        &mut self,
        t: usize,
        l: &CarryGripperLane,
        (p, v): ([f64; 2], [f64; 2]),
        (normal, shelf_up): ([f64; 4], f64),
        forced: bool,
    ) {
        let top = l.shelf_center[1] + l.shelf_half[1];
        let weight = l.hand.ball_mass * l.hand.gravity;
        if normal[0].min(normal[1]) > 1e-7 {
            self.first.get_or_insert(t);
        }
        if shelf_up > 1e-7 {
            self.shelf_first.get_or_insert(t);
        }
        if (800..=1100).contains(&t) {
            self.min_carry = self.min_carry.min(p[1] - l.hand.radius - l.hand.ground);
        }
        if t >= 1201 && self.open.is_none() {
            let supported = (p[0] - l.shelf_center[0]).abs() < l.shelf_half[0] - l.hand.radius
                && (p[1] - l.hand.radius - top).abs() < 0.0008
                && v[1].abs() < 0.025
                && shelf_up <= 0.0014 * weight
                && shelf_up >= 0.00065 * weight
                && normal[2] < 1e-8;
            self.dwell = if supported { self.dwell + 1 } else { 0 };
            if self.dwell >= 24 || forced && t == 1201 {
                self.open = Some(t + 1);
                self.open_state = Some([p[0], p[1] - l.hand.radius - top, shelf_up / 0.001]);
            }
        }
        if self.open.is_some() && shelf_up > 1e-7 {
            self.land.get_or_insert(t);
        }
        if t >= 2101 {
            self.shelf_load_sum += shelf_up / 0.001;
            self.floor_load_sum += normal[2] / 0.001;
            self.loaded += 1;
        }
        self.final_ball = p;
        self.final_shelf = self.shelf_load_sum / self.loaded.max(1) as f64;
    }
}
fn poses(tick: usize, l: &CarryGripperLane) -> serde_json::Value {
    let pose = |x: f64, y: f64, z: f64, a: f64| {
        let (s, c) = (-a * 0.5).sin_cos();
        json!({"translation":[x,y,z],"rotation":[0.,s,0.,c]})
    };
    let h = l.hand;
    let [z, left, right] = h.position;
    let [x, bz] = h.ball;
    json!({"time_ns":tick*1_000_000,"poses":{
        "floor":pose(0.13,0.,-0.009,0.),
        "track_guide":pose(0.105,0.,0.263,0.),
        "shelf":pose(l.shelf_center[0],0.,l.shelf_center[1],0.),
        "cart":pose(l.track,0.,0.25,0.),
        "guide":pose(l.track,0.,0.177,0.),
        "carriage":pose(l.track,0.,z+0.08,0.),
        "left":pose(left,0.,z,0.),"left_stem":pose(left,0.,z+0.044,0.),
        "right":pose(right,0.,z,0.),"right_stem":pose(right,0.,z+0.044,0.),
        "sphere":pose(x,0.,bz,h.angle),
        "marker":pose(x+0.014*h.angle.cos(),0.015,bz+0.014*h.angle.sin(),h.angle)
    }})
}
fn run(adapter: &str, dir: &Path) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(dir)?;
    let init = [
        fixture::fixture(0.08, 0.6, 0.18),
        fixture::fixture(0.2, 0.6, 0.18),
        fixture::fixture(0.08, 0., 0.18),
        fixture::fixture(0.08, 0.6, 0.34),
        fixture::fixture(0.08, 0.6, 0.18),
        fixture::fixture(0.08, 0.6, 0.34),
    ];
    let mut b = CarryGripperBatch::new(adapter, init.to_vec())?;
    let mut native = init.each_ref().map(|l| native::Native::new(l, 16));
    let mut trials = init.map(|_| {
        [
            Run {
                min_carry: f64::INFINITY,
                ..Default::default()
            },
            Run {
                min_carry: f64::INFINITY,
                ..Default::default()
            },
        ]
    });
    let mut errors = [0.0_f64; 6];
    let mut device_time = Vec::new();
    let mut trajectory = Vec::new();
    let mut visual = vec![poses(0, &init[0])];
    let mut missed = vec![poses(0, &init[3])];
    let mut fine_cost = Vec::new();
    let mut recorded = Vec::new();
    for t in 1..=2300 {
        for (i, states) in trials.iter().enumerate() {
            b.set_targets(i, fixture::targets(t, states[0].open.is_some(), i == 5))?;
        }
        let now = Instant::now();
        b.step()?;
        device_time.push(now.elapsed().as_secs_f64() * 1e6);
        recorded.push(b.lanes().to_vec());
        visual.push(poses(t, &b.lanes()[0]));
        missed.push(poses(t, &b.lanes()[3]));
        let mut fine_wall = 0.;
        for (i, l) in b.lanes().iter().enumerate() {
            let q = fixture::targets(t, trials[i][1].open.is_some(), i == 5);
            let start = Instant::now();
            let (n, up) = native[i].step(l, q);
            fine_wall += start.elapsed().as_secs_f64() * 1e6;
            let (native_q, p, v) = native[i].state();
            let hand = l.hand;
            trials[i][0].sample(
                t,
                l,
                (hand.ball, hand.velocity),
                (
                    [
                        hand.normal_impulse[0],
                        hand.normal_impulse[1],
                        hand.normal_impulse[2],
                        l.shelf_normal,
                    ],
                    l.shelf_up,
                ),
                i == 4,
            );
            trials[i][1].sample(t, l, (p, v), (n, up), i == 4);
            errors[i] = errors[i].max((hand.ball[0] - p[0]).hypot(hand.ball[1] - p[1]));
            if i == 0 {
                trajectory.push(json!({
                    "tick":t, "track":l.track, "lift":hand.position[0],
                    "left":hand.position[1], "right":hand.position[2],
                    "ball":hand.ball, "angle":hand.angle,
                    "shelf_n":l.shelf_normal/0.001, "shelf_up":l.shelf_up/0.001,
                    "native_shelf_up":up/0.001, "native_ball":p, "native_q":native_q
                }));
            }
        }
        fine_cost.push(fine_wall);
    }
    b.reset();
    for (i, frame) in recorded.iter().enumerate() {
        for (j, l) in frame.iter().enumerate() {
            b.set_targets(
                j,
                [
                    l.track_target,
                    l.hand.target[0],
                    l.hand.target[1],
                    l.hand.target[2],
                ],
            )?;
        }
        b.step()?;
        assert_eq!(
            bytemuck::cast_slice::<_, u8>(frame),
            bytemuck::cast_slice::<_, u8>(b.lanes()),
            "reset tick {}",
            i + 1
        );
    }
    let mut matched = init.each_ref().map(|l| native::Native::new(l, 4));
    let mut cost = Vec::new();
    for (tick, frame) in recorded.iter().enumerate() {
        let start = Instant::now();
        for ((i, l), native) in frame.iter().enumerate().zip(&mut matched) {
            let released = trials[i][1].open.is_some_and(|t| tick + 1 >= t);
            native.step(l, fixture::targets(tick + 1, released, i == 5));
        }
        cost.push(start.elapsed().as_secs_f64() * 1e6);
    }
    cost.sort_by(f64::total_cmp);
    fine_cost.sort_by(f64::total_cmp);
    device_time.sort_by(f64::total_cmp);
    let rows: Vec<_> = trials.iter().enumerate().map(|(i,m)|{
        let result=|r:&Run|json!({
            "first_grip_ms":r.first, "first_shelf_ms":r.shelf_first,
            "open_ms":r.open, "open_state":r.open_state,
            "min_carry_clearance_m":r.min_carry, "final_ball_m":r.final_ball,
            "final_mean_shelf_n":r.final_shelf, "final_mean_floor_n":r.floor_load_sum/r.loaded as f64
        });
        json!({
            "mass_kg":init[i].hand.ball_mass, "mu":init[i].hand.friction[0],
            "shelf_x":init[i].shelf_center[0], "forced_open":i==4, "wrong_floor":i==5,
            "max_path_error_m":errors[i], "gpu":result(&m[0]), "native":result(&m[1])
        })
    }).collect();
    for i in [0, 1] {
        let weight = init[i].hand.ball_mass * init[i].hand.gravity;
        assert!(errors[i] < 0.002);
        for m in &trials[i] {
            let open = m.open_state.unwrap();
            assert!(
                m.dwell >= 24
                    && (0.65 * weight..=1.4 * weight).contains(&open[2])
                    && open[1].abs() < 0.0008
            );
            assert!(
                (m.final_shelf - weight).abs() < 1e-6
                    && m.min_carry > 0.09
                    && (m.final_ball[0] - 0.18).abs() < 0.003
            );
        }
    }
    for i in [2, 3, 5] {
        for m in &trials[i] {
            assert!(m.open.is_none() && m.final_shelf == 0.);
        }
    }
    for m in &trials[4] {
        let x = m.open_state.unwrap();
        assert!(x[1] > 0.08 && x[2] == 0. && m.land.unwrap() > m.open.unwrap() + 100);
    }
    for m in &trials[5] {
        assert!(m.floor_load_sum / m.loaded as f64 > 0.08 * 9.81);
    }
    let info = b.device_info();
    let d = json!({
        "adapter":info.name, "driver":info.driver_info,
        "device_p95_us":device_time[2184],
        "native_matched_population_p95_us":cost[2184],
        "native_fine_population_p95_us":fine_cost[2184],
        "rows":rows
    });
    fs::write(dir.join("summary.json"), serde_json::to_vec_pretty(&d)?)?;
    fs::write(
        dir.join("trajectory.json"),
        serde_json::to_vec(&trajectory)?,
    )?;
    fs::write(dir.join("source-poses.json"), serde_json::to_vec(&visual)?)?;
    fs::write(dir.join("miss-poses.json"), serde_json::to_vec(&missed)?)?;
    println!("{}", serde_json::to_string_pretty(&d)?);
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if let [adapter, dir, mode] = args.as_slice()
        && mode == "eccentric"
    {
        return eccentric::run(adapter, Path::new(dir));
    }
    if let [adapter, dir] = args.as_slice() {
        return run(adapter, Path::new(dir));
    }
    Err("expected named adapter and output directory".into())
}
