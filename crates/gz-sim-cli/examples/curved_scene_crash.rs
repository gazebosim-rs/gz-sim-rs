// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Kill and reopen whole SDF controllers with pending inputs, off-cadence IMUs and native loads.
#[path = "support/curved_world.rs"]
mod curved;
use curved::{SDF, body, imu, load, native};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, queue_link_world_velocity, queue_link_world_wrench, reset_loaded_world, set_imu_active,
    set_imu_rate,
    trajectory_workers::{self, checkpoint},
};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    io::{BufRead, Write},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn xml(seed: u64) -> String {
    curved::vary(SDF.replace("<collision name=\"box\">","<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>73</update_rate><imu/></sensor><collision name=\"box\">"),seed)
}
fn prepare(sim: &mut Simulation) -> Result<()> {
    let (_, t) = (sim.epoch(), sim.committed_update_info()?.iteration);
    let cargo = body(sim, "cargo")?.0;
    let deck = body(sim, "platform")?.0;
    let sensor = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::cargo::body::imu")
        })
        .and_then(|e| e.get::<EntityId>())
        .ok_or("missing IMU")?
        .get();
    if t % 23 == 4 || [9, 10, 40, 94, 190, 280, 440, 719].contains(&t) {
        queue_link_world_wrench(sim, cargo, [0.38, -0.25, 0.45], [0.001, 0.002, -0.003])?;
        queue_link_world_wrench(sim, cargo, [0.2, 0.08, -0.03], [0.0001, 0., 0.002])?;
    }
    if t == 40 {
        queue_link_world_velocity(sim, cargo, [0.12, -0.05, 0.08], [0.03, 0., 0.])?;
    }
    if t == 740 {
        queue_link_world_velocity(sim, deck, [-0.05, 0.03, 0.], [0.; 3])?;
    }
    if t == 58 {
        set_imu_rate(sim, sensor, 39.5)?;
    }
    if t == 94 || t == 112 {
        set_imu_active(sim, sensor, t == 112)?;
    }
    Ok(())
}
fn evidence(sim: &Simulation, sdf: &sdformat::Sdf) -> Result<Value> {
    let info = sim.committed_update_info()?;
    let state = native(sim).checkpoint_trajectory()?;
    let (scene, catalog) = gz_sim_cli::primitive_frame(sim, sdf, "default", info.iteration + 1)?;
    let canon = |s: &str| -> Result<String> {
        let v: Value = serde_json::from_str(s)?;
        Ok(format!("{:x}", Sha256::digest(serde_json::to_vec(&v)?)))
    };
    let digest = format!("{:x}", Sha256::digest(&state));
    Ok(
        json!({"epoch":info.epoch,"tick":info.iteration,"native":digest,"geometry":canon(&scene)?,"catalog":canon(&catalog)?,"imu":imu(sim)?}),
    )
}
fn actor(worker: &Path, dir: &Path, seed: u64, input: Option<&str>) -> Result<()> {
    let xml = xml(seed);
    let sdf = sdformat::load_str(&xml, "shuttle.sdf", &sdformat::NoResources)?;
    let mut sim = if let Some(f) = input {
        checkpoint::open(&dir.join(f), worker)?
    } else {
        load(&xml, Some(worker))?
    };
    let mut output = std::io::stdout().lock();
    writeln!(output, "{}", evidence(&sim, &sdf)?)?;
    output.flush()?;
    for line in std::io::stdin().lock().lines() {
        let command = line?;
        let result = match command.as_str() {
            query if query.starts_with("frame ") => {
                let seq = query[6..].parse::<u64>()?;
                serde_json::to_value(gz_sim_cli::primitive_frame(&sim, &sdf, "default", seq)?)?
            }
            "prepare" => {
                prepare(&mut sim)?;
                Value::Null
            }
            "step" => {
                sim.step_exact(1)?;
                evidence(&sim, &sdf)?
            }
            "reset" => {
                reset_loaded_world(&mut sim, |_| ())?;
                evidence(&sim, &sdf)?
            }
            "move" => {
                let i = sim.committed_update_info()?;
                let s = sim
                    .world()
                    .resource::<trajectory_workers::TrajectoryProcessPool>()
                    .owner();
                trajectory_workers::select(&mut sim, i.epoch, i.iteration, 1 - s)?;
                Value::Null
            }
            _ => {
                let name = command
                    .strip_prefix("save ")
                    .ok_or("unknown actor command")?;
                checkpoint::save(&sim, &dir.join(name))?;
                Value::Null
            }
        };
        writeln!(output, "{result}")?;
        output.flush()?;
    }
    Ok(())
}
#[path = "support/scene_actor.rs"]
mod scene_actor;
use scene_actor::Actor;
fn check(a: Value, b: &Value) -> Result<()> {
    if &a != b {
        return Err(format!("recovered state differs: {a} vs {b}").into());
    }
    Ok(())
}
fn run(worker: &Path, storage: &Path, seed: u64, ticks: u32, view: Option<&Path>) -> Result<()> {
    let dir = tempfile::Builder::new()
        .prefix("curve-crash-")
        .tempdir_in(storage)?;
    let xml = xml(seed);
    let sdf = sdformat::load_str(&xml, "shuttle.sdf", &sdformat::NoResources)?;
    let mut local = load(&xml, None)?;
    let mut child = Actor::new(worker, dir.path(), seed, None)?;
    check(child.read()?, &evidence(&local, &sdf)?)?;
    let mut cuts = 0;
    let mut kills = 0;
    let mut repeat = 0;
    let mut verified = 1;
    let mut lengths = Vec::new();
    let mut frames = Vec::new();
    let mut rows = Vec::new();
    let mut record = |child: &mut Actor,
                      local: &Simulation,
                      restored: bool,
                      discarded: bool|
     -> Result<()> {
        let seq = frames.len() as u64 + 1;
        let pair: (String, String) = serde_json::from_value(child.call(&format!("frame {seq}"))?)?;
        if pair != gz_sim_cli::primitive_frame(local, &sdf, "default", seq)? {
            return Err("displayed recovered scene differs from its uninterrupted control".into());
        }
        let solver = native(local);
        let (_, deck) = body(local, "platform")?;
        let (_, cargo) = body(local, "cargo")?;
        let p = local.world().resource::<gz_physics::PhysicsBackend>();
        let (d, b) = (p.state(deck).unwrap(), p.state(cargo).unwrap());
        let j = solver
            .translation_interval()
            .1
            .iter()
            .map(|h| h.normal_impulse)
            .sum::<f64>();
        rows.push(json!({"tick":local.committed_update_info()?.iteration,"time_ns":local.elapsed().as_nanos(),"deck":d.position,"cargo":b.position,"cargo_orientation":b.orientation,"normal_impulse":j,"reopened":restored,"discarded_successor":discarded}));
        frames.push(pair);
        Ok(())
    };
    for epoch in 0..2 {
        if epoch > 0 {
            reset_loaded_world(&mut local, |_| ())?;
            check(child.call("reset")?, &evidence(&local, &sdf)?)?;
        }
        for t in 0..ticks {
            child.call("prepare")?;
            prepare(&mut local)?;
            let fault = [0, 9, 10, 40, 58, 94, 112, 190, 280, 440, 719, 740].contains(&t)
                || t % 197 == 31 + (seed % 17) as u32;
            if fault {
                child.call("move")?;
                let file = format!("{epoch}-{t}.gzcurve");
                child.call(&format!("save {file}"))?;
                cuts += 1;
                lengths.push(fs::metadata(dir.path().join(&file))?.len());
                // Alternate cutting immediately after durable publication with killing a controller
                // that has already solved a private future. Reopen the same immutable public generation.
                let speculative = if cuts % 2 == 0 {
                    Some(child.call("step")?)
                } else {
                    None
                };
                child.abort()?;
                kills += 1;
                child = Actor::new(worker, dir.path(), seed, Some(&file))?;
                check(child.read()?, &evidence(&local, &sdf)?)?;
                verified += 1;
                if epoch == 0 && view.is_some() {
                    record(&mut child, &local, true, speculative.is_some())?;
                }
                let next = child.call("step")?;
                local.step_exact(1)?;
                check(next.clone(), &evidence(&local, &sdf)?)?;
                verified += 1;
                if let Some(old) = speculative {
                    check(old, &next)?;
                    repeat += 1;
                }
            } else {
                if epoch == 0 && view.is_some() && t % 5 == 0 {
                    record(&mut child, &local, false, false)?;
                }
                local.step_exact(1)?;
                check(child.call("step")?, &evidence(&local, &sdf)?)?;
                verified += 1;
            }
        }
    }
    if let Some(out) = view {
        fs::create_dir_all(out)?;
        gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
        fs::write(
            out.join("index.json"),
            serde_json::to_vec(&json!({"frames":rows,"seed":seed,"step_ns":5000000}))?,
        )?;
    }
    println!(
        "{}",
        json!({"seed":seed,"epochs":2,"ticks_per_epoch":ticks,"controller_sigkills":kills,"reopened_immutable_cuts":cuts,"verified_complete_native_and_public_states":verified,
        "repeated_discarded_successors":repeat,"cut_min_bytes":lengths.iter().min(),"cut_max_bytes":lengths.iter().max()})
    );
    Ok(())
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mode = args
        .next()
        .ok_or("curved_scene_crash run WORKER SCRATCH SEED TICKS")?;
    let worker = PathBuf::from(args.next().ok_or("worker")?);
    let dir = PathBuf::from(args.next().ok_or("scratch")?);
    let seed = args.next().ok_or("seed")?.parse()?;
    if mode == "actor" {
        actor(&worker, &dir, seed, args.next().as_deref())
    } else if mode == "run" {
        let ticks = args.next().ok_or("ticks")?.parse()?;
        if ticks < 120 {
            return Err("at least 120 ticks required".into());
        }
        run(
            &worker,
            &dir,
            seed,
            ticks,
            args.next().as_deref().map(Path::new),
        )
    } else {
        Err("unknown mode".into())
    }
}
