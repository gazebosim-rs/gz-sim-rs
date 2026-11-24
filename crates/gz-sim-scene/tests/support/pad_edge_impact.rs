// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::{
    Rig, named,
    pad_rocking::{pad_impulses, sum_pad_impulses},
};
use bevy_ecs::prelude::{Res, ResMut, Resource};
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Phase, UpdateInfo};
use std::time::{Duration, Instant};

const WORLD: &str = include_str!("../../../../examples/worlds/local_rocking_pad_grasp.sdf");
const REFINED: &str =
    include_str!("../../../../examples/worlds/local_refined_rocking_pad_grasp.sdf");
const CLOCK: Duration = Duration::from_micros(200);
const PRELOAD: u64 = 1000;
const POST_LAUNCH: u64 = 1250;
const LANDING_START_US: u64 = 38_000;
const LANDING_END_US: u64 = 40_000;

#[derive(Clone, Copy, Default, Debug)]
struct Load {
    impulse: f64,
    angular: DVec3,
    linear: DVec3,
    peak_time: Duration,
    peak: f64,
}

#[derive(Resource, Default)]
struct FineTick {
    loads: [Load; 2],
    calls: u32,
}

/// Replace just the native stepping system. Both fingers, the one-shot wrench
/// and all acquisition systems still execute once per 200 µs Bevy tick.
fn install_reference(rig: &mut Rig, measure_load: bool) {
    let pads = rig.pads;
    let parcel = rig.parcel;
    assert_eq!(
        rig.sim
            .remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)
            .unwrap(),
        1
    );
    rig.sim.world_mut().init_resource::<FineTick>();
    rig.sim.add_systems(
        Phase::PhysicsStep,
        move |time: Res<UpdateInfo>,
              mut backend: ResMut<PhysicsBackend>,
              mut sample: ResMut<FineTick>| {
            let native = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
            *sample = FineTick::default();
            assert_eq!(time.dt, CLOCK);
            // No observer during preload or outside the interval under test.
            if measure_load
                && (Duration::from_micros(200_000 + LANDING_START_US)
                    ..Duration::from_micros(200_000 + LANDING_END_US))
                    .contains(&time.start_time)
            {
                native
                    .observe_contact_interval(time.dt, 200, 4, |sub| {
                        assert_eq!(sub.duration, Duration::from_micros(1));
                        let impulses = sum_pad_impulses(parcel, pads, sub.contact_patches());
                        for (load, patch) in sample.loads.iter_mut().zip(impulses) {
                            load.impulse += patch.normal;
                            load.angular += patch.angular;
                            load.linear += patch.linear;
                            let force = patch.normal / sub.duration.as_secs_f64();
                            if force > load.peak {
                                load.peak = force;
                                load.peak_time = time.start_time + sub.elapsed;
                            }
                        }
                        sample.calls += 1;
                    })
                    .unwrap();
                assert_eq!(sample.calls, 200);
            } else {
                native.step_contact_interval(time.dt, 200, 4).unwrap();
                sample.calls = 200;
            }
        },
    );
}

#[derive(Clone, Copy)]
enum Integration {
    Coarse,
    Continuous,
    Matched(u64),
    Adaptive,
}

struct Trial {
    loads: [Load; 2],
    /// Poses sampled at the end of each public tick from launch.
    pitch: Vec<f64>,
    refined_at: Vec<u64>,
    incoming: Vec<BodyState>,
    calls: u64,
}

fn incoming(rig: &Rig) -> Vec<BodyState> {
    let backend = rig.sim.world().resource::<PhysicsBackend>();
    let mut ids = rig.pads.to_vec();
    ids.push(rig.parcel);
    let wrist = named(&rig.sim, "default::gripper::wrist");
    ids.push(
        rig.sim
            .world()
            .get::<gz_sim_scene::BodyHandle>(wrist)
            .unwrap()
            .id(),
    );
    ids.into_iter()
        .map(|id| backend.state(id).unwrap())
        .collect()
}

fn ordinary_loads(rig: &Rig) -> [Load; 2] {
    pad_impulses(rig).map(|p| Load {
        impulse: p.normal,
        angular: p.angular,
        linear: p.linear,
        peak_time: rig.sim.elapsed(),
        peak: p.normal / CLOCK.as_secs_f64(),
    })
}

fn adaptive_loads(rig: &Rig) -> Option<([Load; 2], u32)> {
    let receipt = gz_sim_scene::primary_contact::last_refinement(&rig.sim)?;
    if receipt.sim_time != rig.sim.elapsed() {
        return None;
    }
    let loads = rig.pads.map(|id| {
        let observed = receipt
            .pad_impacts
            .iter()
            .find(|p| (p.foot, p.support) == (rig.parcel, id))
            .unwrap();
        let pair = receipt
            .interval
            .pairs
            .iter()
            .find(|p| (p.a, p.b) == (id.min(rig.parcel), id.max(rig.parcel)))
            .unwrap();
        Load {
            impulse: pair.normal_impulse,
            angular: observed.angular_impulse_on_foot.unwrap().into(),
            linear: (if pair.a == rig.parcel { 1. } else { -1. })
                * (DVec3::from(pair.normal_impulse_on_a)
                    + DVec3::from(pair.tangent_impulse_on_a.unwrap())),
            peak_time: receipt.start_time + observed.peak_end,
            peak: observed.peak_normal_force_n,
        }
    });
    Some((loads, receipt.interval.native_steps))
}

fn preloaded(mode: Integration, measure_load: bool) -> Rig {
    let world = if matches!(mode, Integration::Adaptive) {
        REFINED
    } else {
        WORLD
    };
    let mut rig = Rig::new(world, 200, 0.);
    rig.drive([3.; 2]);
    if matches!(mode, Integration::Continuous) {
        install_reference(&mut rig, measure_load);
    }
    rig.sim.step_exact(PRELOAD).unwrap();
    assert_eq!(rig.sim.elapsed(), Duration::from_millis(200));
    assert!(gz_sim_scene::primary_contact::last_refinement(&rig.sim).is_none());
    let body = named(&rig.sim, "default::parcel::body");
    let id = rig.sim.world().get::<EntityId>(body).unwrap().get();
    // Submitted just once; the one public pulse lasts 200 µs in every run.
    gz_sim_scene::queue_link_world_wrench(&mut rig.sim, id, [0.; 3], [0., 6., 0.]).unwrap();
    rig
}

fn run(mode: Integration) -> Trial {
    let mut rig = preloaded(mode, true);
    let mut trial = Trial {
        loads: [Load::default(); 2],
        pitch: Vec::new(),
        refined_at: Vec::new(),
        incoming: Vec::new(),
        calls: 0,
    };
    for tick in 0..POST_LAUNCH {
        if matches!(mode,Integration::Matched(start) if start==tick) {
            trial.incoming = incoming(&rig);
            install_reference(&mut rig, true);
        }
        let pre = if matches!(mode, Integration::Adaptive) && trial.refined_at.is_empty() {
            incoming(&rig)
        } else {
            Vec::new()
        };
        rig.sim.step_exact(1).unwrap();
        let (loads, calls) = if let Some(fine) = rig.sim.world().get_resource::<FineTick>() {
            (fine.loads, fine.calls)
        } else if let Some((loads, calls)) = adaptive_loads(&rig) {
            if trial.refined_at.is_empty() {
                trial.incoming = pre;
            }
            trial.refined_at.push(tick);
            (loads, calls)
        } else {
            (ordinary_loads(&rig), 1)
        };
        trial.calls += u64::from(calls);
        let t = (tick + 1) * 200;
        if t > LANDING_START_US && t <= LANDING_END_US {
            for (total, load) in trial.loads.iter_mut().zip(loads) {
                total.impulse += load.impulse;
                total.angular += load.angular;
                total.linear += load.linear;
                if load.peak > total.peak {
                    total.peak = load.peak;
                    total.peak_time = load.peak_time;
                }
            }
        }
        let axis = DQuat::from_array(rig.state().orientation) * DVec3::X;
        trial.pitch.push((-axis.z).atan2(axis.x));
    }
    assert_eq!(rig.sim.elapsed(), Duration::from_millis(450));
    assert_eq!(rig.sim.committed_update_info().unwrap().iteration, 2250);
    trial
}

#[test]
fn refined_returning_edge_resolves_peak_at_the_original_actuator_clock() {
    let coarse = run(Integration::Coarse);
    let adaptive = run(Integration::Adaptive);
    let fine = run(Integration::Continuous);
    println!(
        "edge calls={} ticks={:?} coarse={:?} adaptive={:?} fine={:?}",
        adaptive.calls, adaptive.refined_at, coarse.loads, adaptive.loads, fine.loads
    );
    assert_eq!(adaptive.refined_at, [192, 193, 194]);
    assert_eq!(coarse.calls, POST_LAUNCH);
    assert_eq!(fine.calls, 200 * POST_LAUNCH);
    assert_eq!(adaptive.calls, POST_LAUNCH + 3 * 199);
    let matched = run(Integration::Matched(adaptive.refined_at[0]));
    println!("edge matched={:?}", matched.loads);
    assert_eq!(adaptive.incoming, matched.incoming);
    for side in 0..2 {
        let (c, a, f, m) = (
            coarse.loads[side],
            adaptive.loads[side],
            fine.loads[side],
            matched.loads[side],
        );
        // Independent prior continuous 1 µs launch with the same pulse impulse.
        assert!((f.peak - 193.91463).abs() < 0.01);
        assert!(c.peak < 0.4 * f.peak);
        assert!((a.peak - f.peak).abs() < 0.003 * f.peak);
        assert!(a.peak_time.abs_diff(f.peak_time) < CLOCK);
        assert!((a.peak - m.peak).abs() < 1e-8);
        assert_eq!(a.peak_time, m.peak_time);
        assert!((a.impulse - f.impulse).abs() < 0.004 * f.impulse);
        assert!(a.linear.distance(f.linear) < 0.004 * f.linear.length());
        assert!(a.angular.distance(f.angular) < 0.005 * f.angular.length());
        assert!(a.linear.distance(m.linear) < 0.00004 * m.linear.length());
        assert!(a.angular.distance(m.angular) < 0.0004 * m.angular.length());
    }
    let pitch_error = |trial: &Trial, reference: &Trial| {
        trial
            .pitch
            .iter()
            .zip(&reference.pitch)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max)
            .to_degrees()
    };
    let coarse_matched = pitch_error(&coarse, &matched);
    let adaptive_matched = pitch_error(&adaptive, &matched);
    let adaptive_continuous = pitch_error(&adaptive, &fine);
    println!(
        "pitch error (deg): coarse/matched={coarse_matched:.8}, adaptive/matched={adaptive_matched:.8}, adaptive/continuous={adaptive_continuous:.8}"
    );
    assert!(adaptive_matched < 0.0005 && adaptive_matched < 0.05 * coarse_matched);
    // Continuous fine integration diverges slightly during the coarse approach.
    assert!(adaptive_continuous < 0.22);
}

/// Time only the standard Bevy step. Per-pair test sampling is disabled; the
/// adaptive policy still produces its native telemetry on the refined ticks.
#[test]
#[ignore = "run an optimized build pinned to one CPU for step timing"]
fn compare_edge_refinement_step_cost() {
    for batch in 0..3 {
        let mut modes = [
            ("coarse", Integration::Coarse),
            ("adaptive", Integration::Adaptive),
            ("fine", Integration::Continuous),
        ];
        modes.rotate_left(batch);
        for (name, mode) in modes {
            let mut rig = preloaded(mode, false);
            let mut times = Vec::new();
            let mut selected = Vec::new();
            for _ in 0..POST_LAUNCH {
                let start = Instant::now();
                rig.sim.step_exact(1).unwrap();
                let ns = start.elapsed().as_nanos();
                times.push(ns);
                if gz_sim_scene::primary_contact::last_refinement(&rig.sim)
                    .is_some_and(|r| r.sim_time == rig.sim.elapsed())
                {
                    selected.push(ns);
                }
            }
            times.sort_unstable();
            println!(
                "edge_cost batch={batch} mode={name} p50_ns={} p95_ns={} p99_ns={} max_ns={} sum_ns={} selected_ns={selected:?}",
                times[624],
                times[1187],
                times[1237],
                times[1249],
                times.iter().sum::<u128>()
            );
        }
    }
}
