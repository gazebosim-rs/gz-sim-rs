// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Repeat native park/wake handoffs on the delivered serial arm in unloaded standby.
#![allow(deprecated)]
use bevy_tasks::{ComputeTaskPool, TaskPoolBuilder};
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::{ArticulationTier as Tier, MultiFidelityBackend, SubtreeMomentum};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, held_joint_controller_target, load_local_replay_world, model_fidelity,
    remove_model, reset_loaded_world, set_joint_position_drives_enabled, set_model_fidelity,
    subtree_momentum_about,
};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{error::Error, num::NonZeroUsize, path::PathBuf, time::Instant};

const XML: &str = include_str!("../../../examples/worlds/local_robot_serial_obstacle.sdf");
const MARKER: &str = "<model name=\"gripper\"><plugin filename='gz-sim-switchable-articulation-system' name='gz::sim::systems::SwitchableArticulation'/>";
const CYCLES: usize = 10_000;
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    output: PathBuf,
    #[arg(long, default_value = "1")]
    threads: NonZeroUsize,
}
fn standby_world() -> String {
    // Start after delivery, with the same arm, obstacle, 1 kg parcel inertia and receiver as the job.
    XML.replace("<model name=\"gripper\">", MARKER)
        .replace(
            "<pose>0 0 0.535 0 0 0</pose>",
            "<pose>0 0.46 0.615 0 0 0</pose>",
        )
        .replace(
            "<mass>0.2</mass><inertia><ixx>0.00016</ixx><iyy>0.00014</iyy><izz>0.00013</izz>",
            "<mass>1</mass><inertia><ixx>0.0008</ixx><iyy>0.0007</iyy><izz>0.00065</izz>",
        )
}
fn id(sim: &Simulation, name: &str) -> u64 {
    let full = format!("default::{name}");
    sim.world()
        .iter_entities()
        .find(|n| n.get::<Name>().is_some_and(|s| s.0 == full))
        .expect("named course entity")
        .get::<EntityId>()
        .unwrap()
        .get()
}
struct Rig {
    sim: Simulation,
    model: u64,
    travel: u64,
    drives: [u64; 2],
    links: [BodyId; 6],
    shelf: BodyId,
}
struct State {
    bodies: [BodyState; 6],
    momentum: SubtreeMomentum,
    support_ns: f64,
    targets: [f64; 2],
}
impl Rig {
    fn new() -> Result<Self> {
        let sim =
            load_local_replay_world(&standby_world(), "serial-standby.sdf", "default", [0; 32])?;
        let body = |name| {
            sim.world()
                .get::<BodyHandle>(gz_sim_core::entity_by_id(sim.world(), id(&sim, name)).unwrap())
                .unwrap()
                .id()
        };
        Ok(Self {
            model: id(&sim, "gripper"),
            travel: id(&sim, "gripper::travel"),
            drives: [id(&sim, "gripper::spindle"), id(&sim, "gripper::wrist")],
            links: [
                "gripper::carriage",
                "gripper::nut",
                "gripper::palm",
                "gripper::left",
                "gripper::right",
                "parcel::box",
            ]
            .map(body),
            shelf: body("receiver::shelf"),
            sim,
        })
    }
    fn settle(&mut self) -> Result<()> {
        let tray = id(&self.sim, "stand");
        remove_model(&mut self.sim, tray)?;
        set_joint_position_drives_enabled(&mut self.sim, &self.drives, false)?;
        self.sim.step_exact(4000)?;
        if model_fidelity(&self.sim, self.model)? != Tier::Dynamic {
            return Err("missing initial dynamic owner".into());
        }
        Ok(())
    }
    fn state(&self) -> Result<State> {
        let backend = self.sim.world().resource::<PhysicsBackend>();
        let b = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
        let support = b
            .normal_reaction_on(self.links[5], self.shelf)?
            .map_or(0., |r| r.magnitude);
        Ok(State {
            bodies: self.links.map(|id| backend.state(id).unwrap()),
            momentum: subtree_momentum_about(&self.sim, self.travel, [0.; 3])?,
            support_ns: support,
            targets: self
                .drives
                .map(|id| held_joint_controller_target(&self.sim, id).unwrap()),
        })
    }
}

#[derive(Default, Debug, PartialEq, serde::Serialize)]
struct Errors {
    max_position_m: f64,
    max_rotation_rad: f64,
    max_linear_momentum_kg_m_s: f64,
    max_angular_momentum_kg_m2_s: f64,
    max_kinetic_energy_j: f64,
    max_support_impulse_ns: f64,
}
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    DVec3::from(a).distance(b.into())
}
impl Errors {
    fn measure(&mut self, a: &State, b: &State) -> Result<()> {
        if a.targets != b.targets {
            return Err("controller targets changed during standby".into());
        }
        for (s, t) in a.bodies.iter().zip(&b.bodies) {
            self.max_position_m = self.max_position_m.max(distance(s.position, t.position));
            let dq = DQuat::from_array(s.orientation).inverse() * DQuat::from_array(t.orientation);
            self.max_rotation_rad = self
                .max_rotation_rad
                .max(2. * dq.xyz().length().atan2(dq.w.abs()));
        }
        self.max_linear_momentum_kg_m_s = self
            .max_linear_momentum_kg_m_s
            .max(distance(a.momentum.linear_kg_m_s, b.momentum.linear_kg_m_s));
        self.max_angular_momentum_kg_m2_s = self.max_angular_momentum_kg_m2_s.max(distance(
            a.momentum.angular_kg_m2_s,
            b.momentum.angular_kg_m2_s,
        ));
        self.max_kinetic_energy_j = self
            .max_kinetic_energy_j
            .max((a.momentum.kinetic_energy_j - b.momentum.kinetic_energy_j).abs());
        self.max_support_impulse_ns = self
            .max_support_impulse_ns
            .max((a.support_ns - b.support_ns).abs());
        Ok(())
    }
}
impl State {
    fn hash(&self, digest: &mut Sha256) {
        let mut scalar = |v: f64| {
            assert!(v.is_finite());
            digest.update(v.to_bits().to_le_bytes());
        };
        for body in self.bodies {
            for v in body
                .position
                .into_iter()
                .chain(body.orientation)
                .chain(body.velocity)
                .chain(body.angular_velocity)
            {
                scalar(v);
            }
        }
        for v in self
            .momentum
            .linear_kg_m_s
            .into_iter()
            .chain(self.momentum.angular_kg_m2_s)
            .chain([self.momentum.kinetic_energy_j, self.support_ns])
            .chain(self.targets)
        {
            scalar(v);
        }
    }
}
fn timing(mut samples: Vec<u64>) -> Value {
    samples.sort_unstable();
    let n = samples.len();
    let q = |q: usize| samples[(n * q).div_ceil(100usize) - 1];
    json!({"samples":n,"elapsed_ns":samples.iter().sum::<u64>(),"p95_ns":q(95),"p99_ns":q(99)})
}
fn episode(adaptive: &mut Rig, control: &mut Rig, cycles: usize) -> Result<(String, Value)> {
    adaptive.settle()?;
    control.settle()?;
    let mut boundary = Errors::default();
    let mut trajectory = Errors::default();
    let mut digest = Sha256::new();
    let mut before = adaptive.state()?;
    let mut first = None;
    let mut first_mount = None;
    let mut max_mount_error = 0_f64;
    let mut previous_reference = control.state()?;
    // Five body masses from the delivered arm. Once it is unloaded the only
    // external forces are world gravity and the world-mounted carriage constraint.
    let gravity_impulse = DVec3::NEG_Z
        * (1.2 + 0.7 + 0.4 + 0.08 + 0.08)
        * 9.81
        * adaptive.sim.step_duration().as_secs_f64();
    let mount = |previous: &State, next: &State| {
        DVec3::from(next.momentum.linear_kg_m_s)
            - DVec3::from(previous.momentum.linear_kg_m_s)
            - gravity_impulse
    };
    let mut min_support = f64::INFINITY;
    let mut switched = Vec::new();
    let mut always = Vec::new();
    let mut transitions = Vec::new();
    let elapsed = Instant::now();
    for k in 0..cycles {
        for tier in [Tier::Prescribed, Tier::Dynamic] {
            if model_fidelity(&adaptive.sim, adaptive.model)? == tier {
                return Err(format!("missing actual transition in cycle {k}, to {tier:?}").into());
            }
            let start = Instant::now();
            set_model_fidelity(&mut adaptive.sim, adaptive.model, tier)
                .map_err(|e| format!("cycle {k} to {tier:?}: {e}"))?;
            transitions.push(start.elapsed().as_nanos() as u64);
            let after = adaptive.state()?;
            boundary.measure(&before, &after)?;
            after.hash(&mut digest);
            // Alternate lane order each cycle. Timings exclude instrumentation.
            let step = |rig: &mut Rig, times: &mut Vec<u64>| -> Result<()> {
                let t = Instant::now();
                rig.sim.step_exact(1)?;
                times.push(t.elapsed().as_nanos() as u64);
                Ok(())
            };
            if k % 2 == 0 {
                step(adaptive, &mut switched)?;
                step(control, &mut always)?;
            } else {
                step(control, &mut always)?;
                step(adaptive, &mut switched)?;
            }
            if model_fidelity(&adaptive.sim, adaptive.model)? != tier
                || model_fidelity(&control.sim, control.model)? != Tier::Dynamic
            {
                return Err(format!(
                    "unexpected automatic transition in cycle {k}, requested {tier:?}"
                )
                .into());
            }
            before = adaptive.state()?;
            if tier == Tier::Prescribed {
                for (held, committed) in after.bodies[..5].iter().zip(&before.bodies[..5]) {
                    if committed.position != held.position
                        || committed.orientation != held.orientation
                        || committed.velocity != [0.; 3]
                        || committed.angular_velocity != [0.; 3]
                    {
                        return Err(format!(
                            "sleeping link moved or acquired velocity in cycle {k}"
                        )
                        .into());
                    }
                }
            }
            let reference = control.state()?;
            trajectory.measure(&before, &reference)?;
            before.hash(&mut digest);
            reference.hash(&mut digest);
            if tier == Tier::Dynamic {
                let switched_mount = mount(&after, &before);
                let continuous_mount = mount(&previous_reference, &reference);
                max_mount_error = max_mount_error.max(switched_mount.distance(continuous_mount));
                first_mount.get_or_insert([switched_mount.to_array(), continuous_mount.to_array()]);
                min_support = min_support.min(before.support_ns);
                first.get_or_insert([before.support_ns, reference.support_ns]);
            }
            previous_reference = reference;
        }
    }
    let wall = elapsed.elapsed().as_secs_f64();
    if boundary.max_position_m > 1e-12
        || boundary.max_rotation_rad > 1e-12
        || boundary.max_linear_momentum_kg_m_s > 0.001
        || boundary.max_angular_momentum_kg_m2_s > 0.001
        || trajectory.max_position_m > 0.0001
        || trajectory.max_rotation_rad > 0.001
        || trajectory.max_support_impulse_ns > 0.00001
        || max_mount_error > 0.00025
        || min_support < 0.0097
    {
        return Err(format!("standby bounds exceeded: boundary={boundary:?} trajectory={trajectory:?} min_support={min_support}").into());
    }
    let trace = format!("{:x}", digest.finalize());
    Ok((
        trace,
        json!({"cycles":cycles,"native_tier_changes":cycles*2,"steps_per_lane":cycles*2,
        "boundary":boundary,"matched_trajectory":trajectory,"first_resumed_shelf_impulse_ns_switched_dynamic":first,
        "first_resumed_mount_impulse_vectors_ns_switched_dynamic":first_mount,"max_resumed_mount_impulse_error_ns":max_mount_error,"minimum_resumed_shelf_impulse_ns":min_support,"bevy_step_switched":timing(switched),"bevy_step_dynamic":timing(always),
        "switch_only":timing(transitions),"paired_instrumented_elapsed_s":wall}),
    ))
}
fn main() -> Result<()> {
    let args = Args::parse();
    let pool = ComputeTaskPool::get_or_init(|| {
        TaskPoolBuilder::new()
            .num_threads(args.threads.get())
            .build()
    });
    if pool.thread_num() != args.threads.get() {
        return Err("unexpected Bevy pool size".into());
    }
    let mut adaptive = Rig::new()?;
    let mut control = Rig::new()?;
    let (trace, first) = episode(&mut adaptive, &mut control, CYCLES)?;
    reset_loaded_world(&mut adaptive.sim, |_| ())?;
    reset_loaded_world(&mut control.sim, |_| ())?;
    let (repeat, reset) = episode(&mut adaptive, &mut control, CYCLES)?;
    if trace != repeat {
        return Err("reset changed the ordered native handoffs or controls".into());
    }
    std::fs::write(
        args.output,
        serde_json::to_vec_pretty(
            &json!({"bevy_threads":pool.thread_num(),"per_episode_sha256":trace,"first":first,"reset":reset}),
        )?,
    )?;
    Ok(())
}
#[cfg(test)]
#[path = "serial_switch_soak/tests.rs"]
mod tests;

#[cfg(test)]
#[path = "serial_switch_soak/invalid_tests.rs"]
mod invalid_tests;
