// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::{
    pad_rocking::{Patch, pad_impulses, sum_pad_impulses},
    *,
};
use bevy_ecs::prelude::{Res, ResMut, Resource};
use gz_sim_core::{Phase, UpdateInfo};
use serde::Deserialize;
use std::time::Instant;

const WORLD: &str = include_str!("../../../../examples/worlds/local_offset_gripper_carry.sdf");
const HELD: &str = include_str!("../../../../examples/replay/offset_carry_hold.json");
const SLIP: &str = include_str!("../../../../examples/replay/offset_carry_slip.json");
const ROCK: &str = include_str!("../../../../examples/replay/offset_carry_rock.json");
const DT: f64 = 0.0002;
const TAIL: u32 = 50; // The final 10 ms of a phase.
const OFFSET: f64 = 0.0045;
const LIFT_MASS: f64 = 0.56;
const LIFT_FORCE: f64 = 10.4956;

#[derive(Deserialize)]
struct Journal {
    commands: Vec<Command>,
}

#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
enum Command {
    SetJointForce {
        joint: String,
        force: f64,
    },
    Step {
        count: u32,
    },
    Wrench {
        link: String,
        force: [f64; 3],
        torque: [f64; 3],
    },
}

#[derive(Clone, Copy, Debug)]
enum Grid {
    Ordinary,
    Adaptive,
    Fine,
}

#[derive(Clone, Copy, Default, Resource)]
struct Tick {
    pads: [Patch; 2],
    // Left jaw peak and the end of its solve, relative to this public tick.
    peak_n: f64,
    peak_ns: u64,
}

impl Tick {
    fn sample(&mut self, pads: [Patch; 2], seconds: f64, end_ns: u64) {
        let force = pads[0].normal / seconds;
        if force > self.peak_n {
            self.peak_n = force;
            self.peak_ns = end_ns;
        }
        add(&mut self.pads, pads);
    }
}

fn add(total: &mut [Patch; 2], current: [Patch; 2]) {
    for (a, b) in total.iter_mut().zip(current) {
        a.normal += b.normal;
        a.linear += b.linear;
        a.angular += b.angular;
        a.tangent += b.tangent;
    }
}

// Only the native stepping system is replaced. The lift and fingers are
// commanded at the original public cadence, even in the continuous reference.
fn configure(rig: &mut Rig, grid: Grid, measure: bool) {
    match grid {
        Grid::Ordinary => gz_sim_scene::primary_contact::set_enabled(&mut rig.sim, false).unwrap(),
        Grid::Adaptive => {}
        Grid::Fine => {
            let parcel = rig.parcel;
            let pads = rig.pads;
            assert_eq!(
                rig.sim
                    .remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)
                    .unwrap(),
                1
            );
            rig.sim.world_mut().init_resource::<Tick>();
            rig.sim.add_systems(
                Phase::PhysicsStep,
                move |time: Res<UpdateInfo>,
                      mut physics: ResMut<PhysicsBackend>,
                      mut loads: ResMut<Tick>| {
                    let native = physics.downcast_mut::<MultiFidelityBackend>().unwrap();
                    *loads = Tick::default();
                    if measure {
                        native
                            .observe_contact_interval(time.dt, 200, 4, |sub| {
                                loads.sample(
                                    sum_pad_impulses(parcel, pads, sub.contact_patches()),
                                    sub.duration.as_secs_f64(),
                                    sub.elapsed.as_nanos() as u64,
                                );
                            })
                            .unwrap();
                    } else {
                        native.step_contact_interval(time.dt, 200, 4).unwrap();
                    }
                },
            );
        }
    }
}

fn tick_loads(rig: &Rig) -> (Tick, u32) {
    if let Some(fine) = rig.sim.world().get_resource::<Tick>() {
        return (*fine, 200);
    }
    if let Some(receipt) = gz_sim_scene::primary_contact::last_refinement(&rig.sim)
        .filter(|r| r.sim_time == rig.sim.elapsed())
    {
        let mut tick = Tick::default();
        for (side, id) in rig.pads.into_iter().enumerate() {
            let pad = receipt
                .pad_impacts
                .iter()
                .find(|p| p.foot == rig.parcel && p.support == id)
                .unwrap();
            if side == 0 {
                tick.peak_n = pad.peak_normal_force_n;
                tick.peak_ns = pad.peak_end.as_nanos() as u64;
            }
            let pair = receipt
                .interval
                .pairs
                .iter()
                .find(|p| (p.a, p.b) == (id.min(rig.parcel), id.max(rig.parcel)))
                .unwrap();
            let sign = if pair.a == rig.parcel { 1. } else { -1. };
            let tangent = sign * DVec3::from(pair.tangent_impulse_on_a.unwrap());
            tick.pads[side] = Patch {
                normal: pair.normal_impulse,
                tangent,
                linear: tangent + sign * DVec3::from(pair.normal_impulse_on_a),
                angular: pad.angular_impulse_on_foot.unwrap().into(),
                ..Default::default()
            };
        }
        return (tick, receipt.interval.native_steps);
    }
    let mut tick = Tick::default();
    tick.sample(pad_impulses(rig), DT, 200_000);
    (tick, 1)
}

#[derive(Clone, Copy)]
struct Observation {
    parcel_z: f64,
    wrist_z: f64,
    velocity_z: f64,
    pitch: f64,
    omega: DVec3,
    orientation: DQuat,
}

fn observe(rig: &Rig, wrist: BodyId, offset: f64) -> Observation {
    let body = rig.state();
    let orientation = DQuat::from_array(body.orientation);
    let com = DVec3::from(body.position) + orientation * (DVec3::X * offset);
    let pitch_axis = orientation * DVec3::X;
    Observation {
        parcel_z: com.z,
        wrist_z: rig
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .state(wrist)
            .unwrap()
            .position[2],
        velocity_z: body.velocity[2],
        pitch: (-pitch_axis.z).atan2(pitch_axis.x),
        omega: body.angular_velocity.into(),
        orientation,
    }
}

struct Window {
    start: Observation,
    samples: Vec<Observation>,
    impulses: [Patch; 2],
    tail: [Patch; 2],
    native_calls: u32,
    peaks: Vec<(f64, u64)>,
    offset: f64,
    applied: DVec3,
}

impl Window {
    fn end(&self) -> Observation {
        *self.samples.last().unwrap()
    }
    fn relative_drop(&self, s: Observation) -> f64 {
        (s.wrist_z - self.start.wrist_z) - (s.parcel_z - self.start.parcel_z)
    }
    fn slip_us(&self) -> Option<u32> {
        self.samples
            .iter()
            .position(|&s| self.relative_drop(s) >= 0.0001)
            .map(|i| (i as u32 + 1) * 200)
    }
}

fn run(
    world: &str,
    journal: &str,
    grid: Grid,
    offset: f64,
    measure: bool,
) -> (Vec<Window>, Vec<(u32, u128)>) {
    let mut rig = Rig::new(world, 200, -GRAVITY);
    let wrist = rig
        .sim
        .world()
        .get::<BodyHandle>(named(&rig.sim, "default::gripper::wrist"))
        .unwrap()
        .id();
    configure(&mut rig, grid, measure);
    let mut windows = Vec::new();
    let mut times = Vec::new();
    let mut applied = DVec3::ZERO;
    for command in serde_json::from_str::<Journal>(journal).unwrap().commands {
        match command {
            Command::SetJointForce { joint, force } => {
                let id = rig
                    .sim
                    .world()
                    .get::<EntityId>(named(&rig.sim, &joint))
                    .unwrap()
                    .get();
                set_joint_force(&mut rig.sim, id, force).unwrap();
            }
            Command::Wrench {
                link,
                force,
                torque,
            } => {
                let id = rig
                    .sim
                    .world()
                    .get::<EntityId>(named(&rig.sim, &link))
                    .unwrap()
                    .get();
                gz_sim_scene::queue_link_world_wrench(&mut rig.sim, id, force, torque).unwrap();
                applied += DVec3::from(torque) * DT;
            }
            Command::Step { count } => {
                let mut w = Window {
                    start: observe(&rig, wrist, offset),
                    samples: Vec::new(),
                    impulses: [Patch::default(); 2],
                    tail: [Patch::default(); 2],
                    native_calls: 0,
                    peaks: Vec::new(),
                    offset,
                    applied: std::mem::take(&mut applied),
                };
                for i in 0..count {
                    let then = Instant::now();
                    rig.sim.step_exact(1).unwrap();
                    let elapsed = then.elapsed().as_nanos();
                    if !measure {
                        let calls = match grid {
                            Grid::Fine => 200,
                            _ => gz_sim_scene::primary_contact::last_refinement(&rig.sim)
                                .filter(|r| r.sim_time == rig.sim.elapsed())
                                .map_or(1, |r| r.interval.native_steps),
                        };
                        times.push((calls, elapsed));
                        continue;
                    }
                    w.samples.push(observe(&rig, wrist, offset));
                    let (tick, calls) = tick_loads(&rig);
                    w.native_calls += calls;
                    w.peaks
                        .push((tick.peak_n, u64::from(i) * 200_000 + tick.peak_ns));
                    add(&mut w.impulses, tick.pads);
                    if i + TAIL >= count {
                        add(&mut w.tail, tick.pads);
                    }
                }
                windows.push(w);
            }
        }
    }
    (windows, times)
}

fn check_momentum(name: &str, grid: Grid, windows: &[Window]) {
    for (i, w) in windows.iter().enumerate() {
        let end = w.end();
        let dz = end.wrist_z - w.start.wrist_z;
        let duration = w.samples.len() as f64 * DT;
        let impulse_z: f64 = w.impulses.iter().map(|p| p.linear.z).sum();
        let momentum_residual =
            impulse_z - MASS * (end.velocity_z - w.start.velocity_z + GRAVITY * duration);
        let inertia = DVec3::new(0.00001626666666666667, 0.00002055, 0.00001761666666666667);
        let momentum =
            |s: Observation| s.orientation * (inertia * (s.orientation.inverse() * s.omega));
        let moment_error = (w.impulses[0].angular + w.impulses[1].angular + w.applied
            - (momentum(end) - momentum(w.start)))
        .length();
        let origin_arm = end.orientation * (DVec3::X * w.offset);
        let tail_s = f64::from(TAIL) * DT;
        let tail_normal = w.tail.map(|p| p.normal / tail_s);
        let tail_friction = w.tail.map(|p| p.tangent.z / tail_s);
        let tail_moment = w.tail.map(|p| p.angular.y / tail_s);
        let about_origin = tail_moment.iter().sum::<f64>()
            + origin_arm
                .cross((w.tail[0].linear + w.tail[1].linear) / tail_s)
                .y;
        println!(
            "carry name={name} grid={grid:?} phase={i} wrist_motion_m={dz:.12} relative_drop_m={:.12} slip_us={:?} pitch_deg={:.8} normal_n={tail_normal:?} friction_z_n={tail_friction:?} moment_com_ynm={tail_moment:?} moment_origin_ynm={about_origin} lin_res_ns={momentum_residual:e} ang_res_nms={moment_error:e} native_calls={}",
            w.relative_drop(end),
            w.slip_us(),
            end.pitch.to_degrees(),
            w.native_calls
        );
        assert!(
            momentum_residual.abs() < 1e-9 && moment_error < 1e-8,
            "momentum: {momentum_residual} {moment_error}"
        );
    }
}

fn return_peak(w: &Window) -> (f64, u64) {
    let turn = w
        .samples
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.pitch.total_cmp(&b.pitch))
        .unwrap()
        .0;
    *w.peaks[turn..]
        .iter()
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .unwrap()
}

fn test_run(world: &str, journal: &str, grid: Grid, offset: f64, name: &str) -> Vec<Window> {
    let (w, _) = run(world, journal, grid, offset, true);
    check_momentum(name, grid, &w);
    w
}

fn max_drop_discrepancy(a: &Window, b: &Window) -> f64 {
    assert_eq!(a.samples.len(), b.samples.len());
    a.samples
        .iter()
        .zip(&b.samples)
        .map(|(&p, &q)| (a.relative_drop(p) - b.relative_drop(q)).abs())
        .fold(0., f64::max)
}

#[test]
fn moving_offset_parcel_crosses_the_friction_bound() {
    let held_a = LIFT_FORCE / (MASS + LIFT_MASS) - GRAVITY;
    let held_load = MASS * (held_a + GRAVITY);
    let threshold = held_load / (2. * MU);
    // A feasible static solution uses equal vertical shears and offsets
    // the two normal pressure centers to balance their moment about the COM.
    let strong = 4.2;
    let weak = 2.3;
    assert!(weak < threshold && threshold < strong);
    assert!(weak > MASS * GRAVITY / (2. * MU));
    assert!(OFFSET * held_load / strong < 0.024); // vertical span of the parcel face
    let held = test_run(WORLD, HELD, Grid::Adaptive, OFFSET, "hold");
    let slipped = test_run(WORLD, SLIP, Grid::Adaptive, OFFSET, "slip");
    for (name, journal, actual) in [("hold", HELD, &held), ("slip", SLIP, &slipped)] {
        let fine = test_run(WORLD, journal, Grid::Fine, OFFSET, name);
        for (a, b) in actual.iter().zip(&fine) {
            assert_eq!(a.native_calls, a.samples.len() as u32); // no corner lifted in these two runs
            assert_eq!(b.native_calls, 200 * b.samples.len() as u32);
            let err = max_drop_discrepancy(a, b);
            println!("carry_path name={name} max_drop_error_m={err:e}");
            assert!(err < 2e-6);
            assert_eq!(a.slip_us(), b.slip_us());
            for (p, q) in a.tail.iter().zip(&b.tail) {
                near(p.tangent.z, q.tangent.z, 2e-5);
                near(p.normal, q.normal, 1e-6);
                near(p.angular.y, q.angular.y, 1e-7);
            }
        }
        for run in [actual, &fine] {
            let lift = &run[1];
            assert!(lift.slip_us().is_none());
            near(
                lift.end().wrist_z - lift.start.wrist_z,
                0.5 * held_a * 0.060_f64.powi(2),
                1e-5,
            );
            let end = &run[2];
            let tail_s = f64::from(TAIL) * DT;
            if name == "hold" {
                assert!(end.slip_us().is_none());
                near(
                    end.end().wrist_z - lift.start.wrist_z,
                    0.5 * held_a * 0.140_f64.powi(2),
                    2e-5,
                );
                near(
                    (end.tail[0].tangent.z + end.tail[1].tangent.z) / tail_s,
                    held_load,
                    0.0001,
                );
                for p in end.tail {
                    near(p.normal / tail_s, strong, 1e-5);
                }
            } else {
                let friction = 2. * MU * weak;
                let relative_a = (LIFT_FORCE - friction) / LIFT_MASS - friction / MASS;
                let t = end.samples.len() as f64 * DT;
                let predicted_drop = 0.5 * relative_a * t * t;
                let onset = (2. * 0.0001 / relative_a).sqrt();
                let measured_onset = f64::from(end.slip_us().unwrap()) * 1e-6;
                // Finite pad compression delays the fall slightly after the drive changes.
                near(measured_onset, onset, 0.0005);
                near(end.relative_drop(end.end()), predicted_drop, 0.0001);
                for p in end.tail {
                    near(p.normal / tail_s, weak, 0.0005);
                    near(p.tangent.z, MU * p.normal, 1e-7);
                }
            }
        }
    }
    // The asymmetric friction forces are a solution for these pads. Their
    // split is not uniquely determined by force balance on a finite face.
    let mirrored = WORLD.replace("0.0045 0 0 0 0 0", "-0.0045 0 0 0 0 0");
    assert_ne!(mirrored, WORLD);
    let reversed = test_run(&mirrored, HELD, Grid::Adaptive, -OFFSET, "mirror");
    let tail_s = f64::from(TAIL) * DT;
    assert!(held[2].tail[1].tangent.z - held[2].tail[0].tangent.z > 0.8 * tail_s);
    for (a, b) in held[2].tail.iter().zip(reversed[2].tail.iter().rev()) {
        near(a.tangent.z / tail_s, b.tangent.z / tail_s, 0.001);
    }
}

#[test]
fn twisted_carry_refines_the_return_with_vertical_friction() {
    let ordinary = test_run(WORLD, ROCK, Grid::Ordinary, OFFSET, "rock");
    let adaptive = test_run(WORLD, ROCK, Grid::Adaptive, OFFSET, "rock");
    let fine = test_run(WORLD, ROCK, Grid::Fine, OFFSET, "rock");
    let (a, b, c) = (&ordinary[2], &adaptive[2], &fine[2]);
    for w in [a, b, c] {
        assert!(
            w.samples
                .iter()
                .map(|s| s.pitch.to_degrees())
                .fold(0., f64::max)
                > 6.
        );
        assert!((0.002..0.003).contains(&w.relative_drop(w.end())));
        assert!(w.tail.iter().all(|p| p.normal > 0. && p.tangent.z > 0.));
        assert_eq!(w.slip_us(), c.slip_us());
    }
    assert!(b.native_calls > a.native_calls && b.native_calls < c.native_calls / 20);
    let err = max_drop_discrepancy(b, c);
    let ((pc, tc), (pa, ta), (pf, tf)) = (return_peak(a), return_peak(b), return_peak(c));
    println!(
        "carry_return coarse_n={pc} coarse_ns={tc} adaptive_n={pa} adaptive_ns={ta} fine_n={pf} fine_ns={tf} max_drop_error_m={err:e}"
    );
    assert!(pc < 0.5 * pf && (pa - pf).abs() < 0.01 * pf);
    assert!(ta.abs_diff(tf) < 20_000 && ta.abs_diff(tf) < tc.abs_diff(tf));
    assert!(err < 0.000025);
    for (p, q) in b.tail.iter().zip(c.tail) {
        near(p.angular.y, q.angular.y, 2e-7);
        near(p.tangent.z, q.tangent.z, 1e-7);
    }
}

#[test]
#[ignore = "run an optimized build pinned to one CPU for step timing"]
fn compare_carry_cost() {
    for batch in 0..3 {
        let mut order = [Grid::Ordinary, Grid::Adaptive, Grid::Fine];
        order.rotate_left(batch);
        for grid in order {
            for (name, journal) in [("hold", HELD), ("slip", SLIP), ("rock", ROCK)] {
                let (_, times) = run(WORLD, journal, grid, OFFSET, false);
                for calls in [0, 1, 8, 200] {
                    let mut ns: Vec<_> = times
                        .iter()
                        .filter(|(n, _)| calls == 0 || calls == *n)
                        .map(|(_, n)| *n)
                        .collect();
                    if ns.is_empty() {
                        continue;
                    }
                    ns.sort_unstable();
                    println!(
                        "carry_cost batch={batch} name={name} grid={grid:?} calls={calls} n={} p50_ns={} p95_ns={} total_ns={}",
                        ns.len(),
                        ns[(ns.len() - 1) / 2],
                        ns[(ns.len() * 95).div_ceil(100) - 1],
                        ns.iter().sum::<u128>()
                    );
                }
            }
        }
    }
}
