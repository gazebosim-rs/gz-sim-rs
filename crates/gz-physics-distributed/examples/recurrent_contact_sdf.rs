// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Drive the finite-contact fixture through public SDF load, Bevy wrenches and reset.
#[path = "support/recurrent_activity.rs"]
mod activity;
#[path = "support/native_contact_control.rs"]
mod native;
use activity::{Cycle, GRAVITY, LIFT, PERIOD, WARMUP, Windows};
use bevy_ecs::entity::Entity;
use clap::Parser;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, BodyWrench, ContactImpulse, GroundSupport},
    pool::PoolFrame,
};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, Velocity, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    load_world_with_runtime, queue_link_world_wrench, reset_loaded_world,
};
use sdformat::NoResources;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{error::Error, fs, path::PathBuf, time::Duration};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/distributed_recurrent_contacts.sdf");
const NAMES: [&str; 4] = [
    "default::m1_stable_table::link",
    "default::m2_stable_sphere::link",
    "default::m3_driven_table::link",
    "default::m4_driven_sphere::link",
];

#[derive(Parser)]
pub(crate) struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(3..=8))]
    workers: u8,
    #[arg(long,default_value_t=100,value_parser=clap::value_parser!(u64).range(1..=100_000))]
    cycles: u64,
    #[arg(long)]
    canonical_reference: bool,
    /// Also require the independently run pool fixture's complete state/input hashes.
    #[arg(long)]
    pool_reference: Option<PathBuf>,
    /// Fresh JSON report path.
    output: PathBuf,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Link {
    raw: u64,
    public: u64,
    entity: Entity,
}
fn links(sim: &Simulation) -> Result<Vec<Link>> {
    NAMES
        .iter()
        .map(|name| {
            let e = sim
                .world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == *name))
                .ok_or_else(|| format!("missing public link {name}"))?;
            Ok(Link {
                raw: e
                    .get::<BodyHandle>()
                    .ok_or("missing body handle")?
                    .id()
                    .raw(),
                public: e.get::<EntityId>().ok_or("missing public ID")?.get(),
                entity: e.id(),
            })
        })
        .collect()
}
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
}
fn frame(sim: &Simulation) -> &PoolFrame {
    backend(sim).pool_frame().unwrap()
}
fn pulse(tick: u64) -> bool {
    tick > WARMUP && (tick - WARMUP - 1) % PERIOD < 10
}
fn commands(sim: &mut Simulation, link: Link, tick: u64) -> Result<()> {
    if pulse(tick) {
        queue_link_world_wrench(sim, link.public, [0., 0., LIFT], [0.; 3])?;
    }
    Ok(())
}
#[derive(serde::Serialize)]
struct Sample {
    bodies: Vec<Body>,
    pairs: Vec<ContactImpulse>,
    floors: Vec<GroundSupport>,
    owners: Vec<u8>,
    signed_zero_differences: usize,
}
impl Sample {
    fn read(sim: &Simulation, links: &[Link]) -> Result<Self> {
        let f = frame(sim);
        let mut bodies = vec![];
        let mut signed_zero_differences = 0;
        let mut owners = vec![];
        let mut pairs = vec![];
        let mut floors = vec![];
        let semantic = |id: u64| -> Result<u64> {
            Ok(links
                .iter()
                .position(|link| link.raw == id)
                .ok_or("unknown SDF body ID")? as u64
                + 1)
        };
        for (i, link) in links.iter().enumerate() {
            let (owner, body) = f.body(link.raw).ok_or("missing authoritative body")?;
            let pose = sim
                .world()
                .get::<WorldPose>(link.entity)
                .ok_or("missing public pose")?;
            let v = sim
                .world()
                .get::<Velocity>(link.entity)
                .ok_or("missing public velocity")?;
            if pose.translation != body.position
                || pose.rotation != body.orientation
                || v.linear != body.linear
                || v.angular != body.angular
            {
                return Err(format!(
                    "ECS link {} differs from worker at tick {}",
                    NAMES[i], f.tick
                )
                .into());
            }
            let ecs_values = pose
                .translation
                .iter()
                .chain(&pose.rotation)
                .chain(&v.linear)
                .chain(&v.angular);
            let native_values = body
                .position
                .iter()
                .chain(&body.orientation)
                .chain(&body.linear)
                .chain(&body.angular);
            // Equality above already excludes every other numerical difference.
            signed_zero_differences += ecs_values
                .zip(native_values)
                .filter(|(a, b)| a.to_bits() != b.to_bits())
                .count();
            let mut normalized = body.clone();
            normalized.id = i as u64 + 1;
            bodies.push(normalized);
            owners.push(owner);
        }
        for worker in &f.workers {
            for pair in &worker.snapshot.impulses {
                let a = semantic(pair.a)?;
                let b = semantic(pair.b)?;
                pairs.push(ContactImpulse {
                    a: a.min(b),
                    b: a.max(b),
                    normal_impulse: pair.normal_impulse,
                });
            }
            for floor in &worker.snapshot.supports {
                floors.push(GroundSupport {
                    body: semantic(floor.body)?,
                    normal_impulse: floor.normal_impulse,
                });
            }
        }
        pairs.sort_by_key(|p| (p.a, p.b));
        floors.sort_by_key(|p| p.body);
        Ok(Self {
            bodies,
            pairs,
            floors,
            owners,
            signed_zero_differences,
        })
    }
    fn vz(&self) -> [f64; 2] {
        [self.bodies[1].linear[2], self.bodies[3].linear[2]]
    }
    fn impulse(&self, id: u64) -> Option<f64> {
        self.pairs
            .iter()
            .find(|p| (p.a, p.b) == (id, id + 1))
            .map(|p| p.normal_impulse)
    }
}
#[cfg_attr(test, allow(dead_code))]
fn main() -> Result<()> {
    run(Args::parse())
}

pub(crate) fn run(args: Args) -> Result<()> {
    let mut sim = load_world_with_runtime(
        SDF,
        "distributed_recurrent_contacts.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedAdaptive(args.workers),
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(&args.worker),
        },
        WorldRuntimeOptions {
            distributed_native_64: args.canonical_reference,
            distributed_canonical_reference: args.canonical_reference,
            ..Default::default()
        },
    )?;
    let binding = links(&sim)?;
    let initial = frame(&sim).clone();
    let seed = Sample::read(&sim, &binding)?;
    for tick in 1..=WARMUP + 5 {
        commands(&mut sim, binding[3], tick)?;
        sim.step_exact(1)?;
    }
    if Sample::read(&sim, &binding)?.bodies[3].linear[2] <= 0. {
        return Err("reset preflight did not lift the sphere".into());
    }
    // Reset must also discard an input accepted for the next, unadvanced tick.
    queue_link_world_wrench(&mut sim, binding[3].public, [0., 0., LIFT], [0.; 3])?;
    reset_loaded_world(&mut sim, |_| ())?;
    let restored = links(&sim)?;
    if frame(&sim) != &initial
        || !sim.elapsed().is_zero()
        || !binding
            .iter()
            .zip(&restored)
            .all(|(old, new)| old.public == new.public && old.raw == new.raw)
    {
        return Err(
            "SDF reset changed the native frame, public clock or a public physics binding".into(),
        );
    }
    // reset_loaded_world rebuilds the Bevy entities backing the original public IDs.
    let recreated_entities = binding
        .iter()
        .zip(&restored)
        .filter(|(a, b)| a.entity != b.entity)
        .count();
    let binding = restored;
    let mut reference = native::Control::new(&seed.bodies, None, [0., 0., GRAVITY])?;
    let mut physical = Sha256::new();
    let mut inputs = Sha256::new();
    let mut ecs = Sha256::new();
    let mut before = seed.vz();
    let mut start = before;
    let mut cycle = Cycle::default();
    let mut windows = Windows::default();
    let mut faults = vec![];
    let mut receipts = 0usize;
    let mut raw_ecs = 0usize;
    let mut signed_zero_differences = 0usize;
    let mut owner_changes = 0;
    let mut owners = seed.owners;
    let initial_owners = owners.clone();
    let last = WARMUP + args.cycles * PERIOD;
    for tick in 1..=last {
        let driven = tick.saturating_sub(WARMUP);
        let epoch = driven.div_ceil(PERIOD);
        let phase = if driven == 0 {
            0
        } else {
            (driven - 1) % PERIOD + 1
        };
        let forced = pulse(tick);
        let input = forced.then_some(BodyWrench {
            id: 4,
            force: [0., 0., LIFT],
            torque: [0.; 3],
        });
        let old_pids = if tick == WARMUP + 46 {
            let owner = frame(&sim).body(binding[3].raw).unwrap().0;
            let mut resource = sim.world_mut().resource_mut::<PhysicsBackend>();
            let backend = resource.downcast_mut::<SceneBackend>().unwrap();
            backend.drop_next_acknowledgement(owner)?;
            Some((owner, backend.local_worker_pids().unwrap()))
        } else {
            None
        };
        commands(&mut sim, binding[3], tick)?;
        sim.step_exact(1)?;
        let actual = Sample::read(&sim, &binding)?;
        let rate = backend(&sim)
            .last_substeps()
            .ok_or("missing selected native rate")?;
        let (pairs, floors) = reference.advance(rate, input.as_slice(), &[])?;
        if frame(&sim).tick != tick
            || sim.elapsed() != Duration::from_millis(tick * 10)
            || actual.bodies != reference.bodies
            || actual.pairs != pairs
            || actual.floors != floors
        {
            fs::write(
                args.output.with_extension("mismatch.json"),
                serde_json::to_vec_pretty(&json!({
                    "tick":tick,"distributed":actual,"native_bodies":reference.bodies,"native_pairs":pairs,
                    "native_floors":floors,"rate":rate,"wrench":input
                }))?,
            )?;
            return Err(format!("public SDF state or native receipts differ at {tick}").into());
        }
        if let Some((owner, pids)) = old_pids {
            if backend(&sim).local_worker_pids().as_ref() != Some(&pids)
                || actual.impulse(3).is_none_or(|v| v < 0.5)
            {
                return Err(
                    "SDF retry at landing lost impulse or replaced a healthy worker".into(),
                );
            }
            faults.push(json!({"tick":tick,"owner":owner,"landing_impulse_n_s":actual.impulse(3),"pids":pids}));
        }
        if phase == 1 {
            start = before;
            cycle = Cycle {
                cycle: epoch,
                ..Default::default()
            };
        }
        if phase != 0 {
            cycle.observe(
                phase,
                actual.impulse(1),
                actual.impulse(3),
                if forced { LIFT } else { 0. },
                actual.bodies[3].position[2] - 1.1,
            );
        }
        if phase == PERIOD {
            cycle.complete(start, actual.vz())?;
            windows.observe(&cycle)?;
        }
        before = actual.vz();
        owner_changes += owners
            .iter()
            .zip(&actual.owners)
            .filter(|(a, b)| a != b)
            .count();
        owners = actual.owners;
        for link in &binding {
            let pose = sim.world().get::<WorldPose>(link.entity).unwrap();
            let vel = sim.world().get::<Velocity>(link.entity).unwrap();
            ecs.update(bincode::serialize(&(
                tick,
                link.public,
                pose.translation,
                pose.rotation,
                vel.linear,
                vel.angular,
            ))?);
            raw_ecs += 1;
        }
        signed_zero_differences += actual.signed_zero_differences;
        receipts += actual.pairs.len();
        if phase == PERIOD && epoch.is_multiple_of(1_000) {
            eprintln!("verified {tick} SDF ticks and {receipts} finite receipts");
        }
        physical.update(bincode::serialize(&(
            tick,
            &actual.bodies,
            &actual.pairs,
            &actual.floors,
        ))?);
        inputs.update(bincode::serialize(&(tick, rate, input.as_slice()))?);
    }
    let physical = format!("{:x}", physical.finalize());
    let inputs = format!("{:x}", inputs.finalize());
    if let Some(path) = &args.pool_reference {
        let other: Value = serde_json::from_slice(&fs::read(path)?)?;
        if other["trajectory_sha256"] != physical
            || other["input_sha256"] != inputs
            || other["counts"]["ticks"] != last
        {
            return Err("ordinary SDF run differs from pool reference inputs/physics".into());
        }
    }
    serde_json::to_writer_pretty(
        fs::File::create_new(args.output)?,
        &json!({
            "workers":args.workers,"canonical_reference":args.canonical_reference,"ticks":last,"cycles":args.cycles,
            "source":"examples/distributed_recurrent_contacts.sdf","reset_after_tick":WARMUP+5,"reset_with_pending_lift":true,"reset_bevy_entities_recreated":recreated_entities,
            "initial_owners":initial_owners,"final_owners":owners,"ownership_changes":owner_changes,
            "semantic_links":NAMES.iter().zip(&binding).enumerate().map(|(i,(name,id))|json!({"id":i+1,"name":name,"public_ecs":id.public,"body_handle":id.raw})).collect::<Vec<_>>(),
            "finite_receipts":receipts,"public_pose_velocity_checks":raw_ecs,"public_signed_zero_differences":signed_zero_differences,"native_control":"one uninterrupted world",
            "public_ecs_sha256":format!("{:x}",ecs.finalize()),"trajectory_sha256":physical,"input_sha256":inputs,
            "compared_pool_reference":args.pool_reference.is_some(),"landing_retry":faults,"last_cycle":cycle,"rolling_qualification":windows
        }),
    )?;
    Ok(())
}
