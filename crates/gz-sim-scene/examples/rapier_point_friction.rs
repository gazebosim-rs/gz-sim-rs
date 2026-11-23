// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Drive one ordinary SDF with both native friction policies, then repeat after loaded reset.
#[path = "support/rapier_friction_control.rs"]
mod control;
use gz_physics::{BodyId, DynamicBackend, PhysicsBackend};
use gz_physics_rapier::RapierBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    load_world_with_runtime,
};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/rapier_per_point_friction.sdf");
const SIZES: [[f64; 3]; 2] = [[8., 3., 0.4], [0.8, 0.4, 0.2]];

fn load(engine: PhysicsEngine, point: bool) -> Result<Simulation> {
    Ok(load_world_with_runtime(
        SDF,
        "rapier_per_point_friction.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_per_point_friction: point,
            ..Default::default()
        },
    )?)
}
fn bind(sim: &Simulation, name: &str) -> Result<(u64, BodyId)> {
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .ok_or("missing authored link")?;
    Ok((
        entity.get::<EntityId>().ok_or("missing public ID")?.get(),
        entity
            .get::<BodyHandle>()
            .ok_or("missing native handle")?
            .id(),
    ))
}
fn observe(
    sim: &Simulation,
    links: &[(u64, BodyId); 2],
    tick: u32,
    normal: f64,
    kick: bool,
) -> Result<Value> {
    let backend = sim.world().resource::<PhysicsBackend>();
    let mut bodies = Vec::new();
    for ((public, id), size) in links.iter().zip(SIZES) {
        let s = backend.state(*id).ok_or("missing native body")?;
        let e = entity_by_id(sim.world(), *public).ok_or("missing public body")?;
        let pose = sim
            .world()
            .get::<WorldPose>(e)
            .ok_or("missing Bevy world pose")?;
        assert_eq!(pose.translation, s.position);
        // Bevy composes parent transforms and may choose the other quaternion hemisphere.
        let agreement = s
            .orientation
            .iter()
            .zip(pose.rotation)
            .map(|(a, b)| a * b)
            .sum::<f64>()
            .abs();
        assert!((1. - agreement).abs() < 1e-12);
        bodies.push(
            json!({"id":public,"size":size,"position":s.position,"orientation":s.orientation,
            "velocity":s.velocity,"angular_velocity":s.angular_velocity}),
        );
    }
    let events = if kick {
        vec![
            json!({"body":links[1].0,"linear_impulse":[4.8,0,0],"angular_impulse":[0,0,0.96],"at_ns":300_000_000}),
        ]
    } else {
        vec![]
    };
    let time = sim.committed_update_info()?.sim_time;
    Ok(
        json!({"tick":tick,"time_ns":u64::try_from(time.as_nanos())?,"bodies":bodies,"normal_total":normal,"events":events}),
    )
}
fn run(sim: &mut Simulation, point: bool) -> Result<Vec<Value>> {
    let links = [
        bind(sim, "default::table::deck")?,
        bind(sim, "default::block::body")?,
    ];
    let solver = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<RapierBackend>()
        .ok_or("wrong backend")?;
    assert_eq!(solver.per_point_friction_enabled(), point);
    let mut control = control::Control::new(point);
    let mut frames = vec![observe(sim, &links, 0, 0., false)?];
    for tick in 1..=120 {
        let kick = tick == 31;
        if kick {
            control.kick();
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .apply_impulse(links[1].1, [4.8, 0., 0.], [0., 0., 0.96])?;
        }
        let mut normal = 0.;
        for _ in 0..10 {
            sim.step_exact(1)?;
            let backend = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<RapierBackend>()
                .unwrap();
            control.step();
            for (index, (_, id)) in links.iter().enumerate() {
                control.check(
                    index,
                    backend.state(*id).ok_or("missing body during control")?,
                );
            }
            normal += backend
                .rigid_contact_observations()
                .iter()
                .map(|c| c.normal_impulse)
                .sum::<f64>();
        }
        frames.push(observe(sim, &links, tick, normal, kick)?);
    }
    Ok(frames)
}
fn main() -> Result<()> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: rapier_point_friction OUTPUT_DIRECTORY")?,
    );
    qualify(&dir)
}

fn qualify(dir: &Path) -> Result<()> {
    fs::create_dir_all(dir)?;
    let mut summary = Vec::new();
    for (profile, point) in [("grouped", false), ("pointwise", true)] {
        let mut sim = load(PhysicsEngine::Rapier, point)?;
        let frames = run(&mut sim, point)?;
        sim.reset()?;
        assert_eq!(sim.epoch(), 1);
        assert_eq!(
            serde_json::to_vec(&frames)?,
            serde_json::to_vec(&run(&mut sim, point)?)?,
            "loaded reset changed the trajectory"
        );
        let mut out = BufWriter::new(fs::File::create(dir.join(format!("{profile}.jsonl")))?);
        for frame in &frames {
            serde_json::to_writer(&mut out, frame)?;
            writeln!(out)?;
        }
        out.flush()?;
        summary.push(json!({"profile":profile,"physics_steps":1200,"independent_native_steps":1200,"reset_epoch":sim.epoch(),"reset_exact_frames":frames.len(),"before_impulse":frames[30],"after_first_interval":frames[31],"final":frames[120]}));
    }
    assert_ne!(
        summary[0]["final"]["bodies"][1],
        summary[1]["final"]["bodies"][1]
    );
    for engine in [
        PhysicsEngine::Tpe,
        PhysicsEngine::MultiFidelity,
        PhysicsEngine::Distributed,
    ] {
        let error = load(engine, true)
            .err()
            .ok_or("foreign engine accepted Rapier-specific mode")?;
        assert!(error.to_string().contains("standalone Rapier"), "{error}");
    }
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    Ok(())
}

#[test]
fn ordinary_authored_friction_survives_loaded_reset_and_refuses_foreign_backends() -> Result<()> {
    let output = tempfile::tempdir()?;
    qualify(output.path())
}
