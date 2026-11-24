// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a shared rigid/XPBD worker against the independent local SDF over complete process loss.
#[path = "support/curved_material_batch.rs"]
mod batch;
#[path = "support/curved_material_deadline.rs"]
mod deadline;
#[path = "support/curved_material_field.rs"]
mod field;
#[path = "support/curved_material_lifetime.rs"]
mod lifetime;
#[path = "support/curved_material_pages.rs"]
mod pages;
#[path = "support/curved_material_partition_cuts.rs"]
mod partitions;
#[path = "support/curvature_pulse.rs"]
mod pulse;
#[path = "support/curved_material_rpc.rs"]
mod rpc;
#[path = "support/scene_actor.rs"]
mod scene_actor;

use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, ImuSample, LocalPose, Name, PhysicsEngine, WorldLoadOptions, WorldPose, deformable,
    load_world_configured, queue_link_world_velocity, reset_loaded_world,
    trajectory_workers::{self, TrajectoryProcessPool, checkpoint},
};
use scene_actor::Actor;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    io::{BufRead, Write},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const GUIDE: &str = include_str!("../../../examples/multifidelity_curved_cable_guide.sdf");
fn xml(seed: u64) -> String {
    let a = 0.003 + 0.00005 * (seed % 37) as f64;
    let b = 150_000_000 + 997 * (seed % 13001);
    let mut s=GUIDE.replace("<gravity>0 0 0","<gravity>0 0 -9.81").replace("<max_step_size>0.1","<max_step_size>0.002")
        .replace(" 0.65\" mass", &format!(" {:.17}\" mass",0.595+0.00007*(seed%23)as f64))
        .replace("<collision name=\"surface\">","<sensor name='guide_imu' type='imu'><always_on>true</always_on><update_rate>133</update_rate><imu/></sensor><collision name=\"surface\">");
    let i = s.find("<trajectory ").unwrap();
    let j = s[i..].find("</trajectory>").unwrap() + i + "</trajectory>".len();
    s.replace_range(i..j,&format!("<trajectory end='repeat'><waypoint time_ns='0' displacement='0 0 0' velocity='0 0 0'/><waypoint time_ns='{b}' displacement='0 0 {a:.17}' velocity='0 0 0'/><waypoint time_ns='400000000' displacement='0 0 0' velocity='0 0 0'/></trajectory>"));
    s.replace("<plugin name=\"gz::sim::systems::Deformables\"", "<model name='base'><static>true</static><pose>0 0 -0.1 0 0 0</pose><link name='body'><collision name='floor'><geometry><box><size>2 2 0.2</size></box></geometry></collision></link></model><plugin name=\"gz::sim::systems::Deformables\"")
      .replace("<cable name=\"line\"","<plane point='0 0 0' normal='0 0 1'/><cable name=\"line\"")
}
fn setup(s: &str, w: Option<&Path>) -> Result<Simulation> {
    Ok(load_world_configured(
        s,
        "curved_material.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            step: None,
            engine: PhysicsEngine::MultiFidelity,
            noise_seed: [0; 32],
            distributed_worker: w,
        },
    )?)
}
fn observation(s: &Simulation) -> Result<Value> {
    let b = s.world().resource::<PhysicsBackend>();
    let native = b
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .checkpoint_trajectory()?;
    let sample = deformable::bodies(s).next().unwrap().1;
    let mut h = Sha256::new();
    let mut published = Vec::new();
    let mut sensor = Value::Null;
    for e in s.world().iter_entities() {
        if let (Some(id), Some(p), Some(l)) = (
            e.get::<EntityId>(),
            e.get::<WorldPose>(),
            e.get::<LocalPose>(),
        ) {
            published.push((
                id.get(),
                p.translation,
                p.rotation,
                l.0.translation,
                l.0.rotation,
            ));
        }
        if let Some(imu) = e.get::<ImuSample>() {
            let r = imu.reading.as_ref().map_err(|e| format!("{e:?}"))?;
            sensor = json!([
                imu.epoch,
                imu.time.as_nanos(),
                r.orientation,
                r.specific_force,
                r.angular_velocity
            ]);
        }
    }
    published.sort_unstable_by_key(|r| r.0);
    for (id, p, q, l, r) in published {
        h.update(id.to_le_bytes());
        for v in p.into_iter().chain(q).chain(l).chain(r) {
            h.update(v.to_le_bytes());
        }
    }
    for p in sample.particles() {
        for x in p
            .position
            .into_iter()
            .chain(p.velocity)
            .chain([p.inverse_mass])
        {
            h.update(x.to_le_bytes());
        }
    }
    for r in sample.last_boundary_impulses() {
        h.update((r.boundary as u64).to_le_bytes());
        for x in r.linear.into_iter().chain(r.angular) {
            h.update(x.to_le_bytes());
        }
    }
    let info = s.committed_update_info()?;
    let loaded = sample
        .last_boundary_impulses()
        .first()
        .map_or(0., |r| -r.linear[2]);
    Ok(
        json!({"epoch":info.epoch,"tick":info.iteration,"time":info.sim_time.as_nanos(),"native":format!("{:x}",Sha256::digest(native)),"material_and_public":format!("{:x}",h.finalize()),"imu":sensor,
        "slices":sample.last_path_substeps(),"guide_impulse_ns":loaded,"particle_z":sample.particles()[0].position[2]}),
    )
}
#[cfg(unix)]
fn terminate_worker(pid: u32) -> Result<()> {
    use rustix::process::{Pid, Signal, kill_process};
    kill_process(
        Pid::from_raw(i32::try_from(pid)?).ok_or("invalid worker PID")?,
        Signal::KILL,
    )?;
    Ok(())
}
#[cfg(not(unix))]
fn terminate_worker(_: u32) -> Result<()> {
    Err("worker signal witness requires POSIX".into())
}

/// Independent local SDF observations for comparing the standard web operator's recovery stream.
fn reference(sdf: &Path, out: &Path, ticks: u32) -> Result<()> {
    let mut sim = setup(&fs::read_to_string(sdf)?, None)?;
    let guide = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::guide::roller")
        })
        .and_then(|e| e.get::<BodyHandle>())
        .ok_or("missing curved guide")?
        .id();
    let rollers = sim
        .world()
        .iter_entities()
        .filter_map(|e| {
            let name = &e.get::<Name>()?.0;
            let i = name
                .strip_prefix("default::roller_")?
                .strip_suffix("::shaft")?;
            Some((format!("band_{i}"), e.get::<BodyHandle>()?.id()))
        })
        .collect::<Vec<_>>();
    let mut rows = Vec::new();
    for t in 0..=ticks {
        if t > 0 {
            sim.step_exact(1)?;
        }
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(guide)
            .ok_or("missing guide body")?;
        let mut nodes = std::collections::BTreeMap::new();
        for (name, id) in &rollers {
            let pose = sim
                .world()
                .resource::<PhysicsBackend>()
                .state(*id)
                .ok_or("missing roller on public material guide")?;
            nodes.insert(name.clone(), pose.position);
        }
        let mut loads = Vec::new();
        for (name, line) in deformable::bodies(&sim) {
            let particles = line.particles();
            for j in 0..16 {
                let leaf = if name == "line" {
                    format!("node_{j}")
                } else {
                    format!("{name}_node_{j:02}")
                };
                nodes.insert(leaf, particles[j * (particles.len() - 1) / 15].position);
            }
            let native_loads = &line.last_boundary_impulses()
                [..line.last_boundary_impulses().len().min(1 + rollers.len())];
            loads.push(std::array::from_fn::<_, 3, _>(|axis| {
                native_loads.iter().map(|r| r.linear[axis]).sum::<f64>()
            }));
        }
        let total: [f64; 3] = std::array::from_fn(|i| loads.iter().map(|r| r[i]).sum());
        rows.push(json!({"tick":t,"time_ns":sim.committed_update_info()?.sim_time.as_nanos(),"guide":native.position,
           "visual_nodes":nodes,"reaction_ns":total,"individual_reactions_ns":loads}));
    }
    fs::write(out, serde_json::to_vec(&rows)?)?;
    Ok(())
}
fn prepare(s: &mut Simulation) -> Result<()> {
    let t = s.committed_update_info()?.iteration;
    if t == 410 {
        let e = s
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::guide::roller")
                    && e.contains::<BodyHandle>()
            })
            .unwrap();
        let id = e.get::<EntityId>().unwrap().get();
        queue_link_world_velocity(s, id, [0., 0., -0.025], [0.; 3])?;
    }
    Ok(())
}
fn actor(worker: &Path, dir: &Path, seed: u64, start: Option<&str>) -> Result<()> {
    let mut sim = if let Some(f) = start {
        checkpoint::open(&dir.join(f), worker)?
    } else {
        setup(&xml(seed), Some(worker))?
    };
    let mut out = std::io::stdout().lock();
    writeln!(out, "{}", observation(&sim)?)?;
    out.flush()?;
    for line in std::io::stdin().lock().lines() {
        let cmd = line?;
        let reply = match cmd.as_str() {
            "prepare" => {
                prepare(&mut sim)?;
                Value::Null
            }
            "move" => {
                let i = sim.committed_update_info()?;
                let slot = sim.world().resource::<TrajectoryProcessPool>().owner();
                trajectory_workers::select(&mut sim, i.epoch, i.iteration, 1 - slot)?;
                Value::Null
            }
            "kill" => {
                let p = sim.world().resource::<TrajectoryProcessPool>();
                let pid = p.pid(p.owner()).ok_or("owner is not active")?;
                terminate_worker(pid)?;
                Value::Null
            }
            "advance" => {
                sim.step_exact(1)?;
                let r = sim
                    .world()
                    .resource::<TrajectoryProcessPool>()
                    .last()
                    .unwrap();
                json!({"state":observation(&sim)?,"restarted":r.restarted,"out_bytes":r.successor_bytes,"wall_ns":r.wall.as_nanos()})
            }
            "reset" => {
                reset_loaded_world(&mut sim, |_| ())?;
                observation(&sim)?
            }
            _ => {
                checkpoint::save(
                    &sim,
                    &dir.join(cmd.strip_prefix("save ").ok_or("invalid actor command")?),
                )?;
                Value::Null
            }
        };
        writeln!(out, "{reply}")?;
        out.flush()?;
    }
    Ok(())
}
fn equal(actual: &Value, expected: &Value) -> Result<()> {
    if actual != expected {
        return Err(format!("coupled owner diverged: {actual} vs {expected}").into());
    }
    Ok(())
}
fn run(w: &Path, scratch: &Path, seed: u64, ticks: u32, world_out: Option<PathBuf>) -> Result<()> {
    let dir = tempfile::Builder::new()
        .prefix("gz-material-owner-")
        .tempdir_in(scratch)?;
    let xml = xml(seed);
    if let Some(path) = world_out {
        fs::write(path, &xml)?;
    }
    let mut local = setup(&xml, None)?;
    let mut proc = Actor::new(w, dir.path(), seed, None)?;
    equal(&proc.read()?, &observation(&local)?)?;
    let (mut kills, mut crashes, mut repeated, mut moves, mut loaded, mut comparisons, mut bytes) =
        (0, 0, 0, 0, 0, 1, 0u64);
    let mut dur = Vec::new();
    let mut max_load = 0f64;
    let mut seen = Vec::new();
    for epoch in 0..2 {
        if epoch > 0 {
            reset_loaded_world(&mut local, |_| ())?;
            equal(&proc.call("reset")?, &observation(&local)?)?;
        }
        let mut j = Vec::new();
        for t in 0..ticks {
            proc.call("prepare")?;
            prepare(&mut local)?;
            let cut = [0, 16, 22, 75, 193, 200, 410, 421].contains(&t)
                || t % 151 == (seed % 29) as u32 + 40;
            let kill = !cut
                && (![0, 1].contains(&t))
                && ([15, 19, 42, 76, 199, 201, 411].contains(&t) || t % 107 == 31);
            let flip = !kill && ([12, 14, 20, 73, 194, 213, 408].contains(&t) || t % 181 == 33);
            if flip {
                proc.call("move")?;
                moves += 1;
            }
            let mut old = None;
            if cut {
                let f = format!("cut-{epoch}-{t}");
                proc.call(&format!("save {f}"))?;
                if crashes % 2 == 1 {
                    old = Some(proc.call("advance")?["state"].clone());
                }
                proc.abort()?;
                crashes += 1;
                proc = Actor::new(w, dir.path(), seed, Some(&f))?;
                equal(&proc.read()?, &observation(&local)?)?;
                comparisons += 1;
            }
            if kill {
                proc.call("kill")?;
                kills += 1;
            }
            let r = proc.call("advance")?;
            if r["restarted"] != kill {
                return Err(format!("unexpected worker recovery {epoch}/{t}").into());
            }
            local.step_exact(1)?;
            let s = observation(&local)?;
            equal(&r["state"], &s)?;
            comparisons += 1;
            if let Some(prior) = old {
                equal(&s, &prior)?;
                repeated += 1;
            }
            let imp = s["guide_impulse_ns"].as_f64().unwrap();
            j.push(imp.to_bits());
            loaded += usize::from(imp > 0.);
            max_load = max_load.max(imp / 0.002);
            bytes += r["out_bytes"].as_u64().unwrap();
            dur.push(r["wall_ns"].as_u64().unwrap());
        }
        if epoch > 0 && seen != j {
            return Err("loaded reset changed reaction history".into());
        }
        seen = j;
    }
    dur.sort_unstable();
    let q = |v: usize| dur[(dur.len() * v).div_ceil(100) - 1];
    println!(
        "{}",
        json!({"seed":seed,"ticks_per_epoch":ticks,"epochs":2,"migrations":moves,"worker_sigkills":kills,"controller_sigkills":crashes,"discarded_successors_repeated":repeated,
        "exact_complete_cuts":comparisons,"loaded_intervals":loaded,"max_interval_load_n":max_load,"response_bytes":bytes,"p50_ns":q(50),"p95_ns":q(95)})
    );
    Ok(())
}
fn main() -> Result<()> {
    let mut a = std::env::args().skip(1);
    let mode = a
        .next()
        .ok_or("curved_material_worker run WORKER SCRATCH SEED TICKS [OUTPUT_SDF]")?;
    if mode == "pulse" {
        let worker = a.next().ok_or("worker path or local")?;
        let ns = a.next().ok_or("public step ns")?.parse()?;
        return pulse::run(
            (worker != "local").then_some(Path::new(&worker)),
            ns,
            a.next().as_deref().map(Path::new),
        );
    }
    if mode.starts_with("--managed-trajectory-material") {
        return pages::relay(&mode, a);
    }
    if mode == "pages" || mode == "page-owner" || mode == "measure" {
        let worker = a.next().ok_or("worker")?;
        let dir = a.next().ok_or("scratch directory")?;
        let particles = a.next().ok_or("population")?.parse()?;
        let ticks = a.next().ok_or("ticks")?.parse()?;
        if mode == "pages" {
            return pages::qualify(Path::new(&worker), Path::new(&dir), particles, ticks);
        }
        return pages::owner(
            Path::new(&worker),
            Path::new(&dir),
            particles,
            ticks,
            mode == "page-owner",
        );
    }
    if mode == "field-world" {
        let out = a.next().ok_or("SDF output")?;
        let sizes = batch::sizes(&a.next().ok_or("material sizes")?)?;
        return Ok(fs::write(
            out,
            field::fixture(
                &sizes,
                a.next().ok_or("guide count")?.parse()?,
                a.next().ok_or("workers")?.parse()?,
                a.next().ok_or("seed")?.parse()?,
            )?,
        )?);
    }
    if mode == "field" || mode == "field-faults" {
        return field::run(
            Path::new(&a.next().ok_or("SDF")?),
            Path::new(&a.next().ok_or("worker")?),
            a.next().ok_or("ticks")?.parse()?,
            mode == "field-faults",
        );
    }
    if mode == "inflight-controller" {
        let worker = PathBuf::from(a.next().ok_or("worker")?);
        let sdf = PathBuf::from(a.next().ok_or("packed two-group SDF")?);
        let scratch = PathBuf::from(a.next().ok_or("scratch")?);
        let cut = PathBuf::from(a.next().ok_or("new output checkpoint")?);
        return lifetime::inflight(&worker, &sdf, &scratch, &cut);
    }
    if mode == "stock-stopped-cut" {
        let exe = PathBuf::from(a.next().ok_or("stock server")?);
        let w = PathBuf::from(a.next().ok_or("worker")?);
        let sdf = PathBuf::from(a.next().ok_or("world SDF")?);
        let cut = PathBuf::from(a.next().ok_or("new immutable cut")?);
        let port = a.next().ok_or("port")?.parse()?;
        let reader = PathBuf::from(a.next().ok_or("operator reader")?);
        let reference = PathBuf::from(a.next().ok_or("reference")?);
        return lifetime::stock(&exe, &w, &sdf, &cut, port, &reader, &reference);
    }
    if mode == "stopped-controller" {
        let exe = PathBuf::from(a.next().ok_or("controller program")?);
        let w = PathBuf::from(a.next().ok_or("worker program")?);
        let sdf = PathBuf::from(a.next().ok_or("SDF")?);
        let dir = PathBuf::from(a.next().ok_or("scratch")?);
        let expected = match a.next().as_deref() {
            Some("guarded") => true,
            Some("legacy") => false,
            _ => return Err("guarded or legacy baseline".into()),
        };
        return lifetime::run(&exe, &w, &sdf, &dir, expected);
    }
    if mode == "warm-budget-world" {
        let input = a.next().ok_or("original SDF")?;
        let output = a.next().ok_or("output SDF")?;
        let ms: u64 = a.next().ok_or("floor in milliseconds")?.parse()?;
        if !(10..=6000).contains(&ms) {
            return Err("expected 10 to 6000 ms".into());
        }
        let sdf = fs::read_to_string(input)?.replace(
            "filename=\"gz-sim-deformables-system\"",
            &format!("filename=\"gz-sim-deformables-system\" worker_warm_budget_ms='{ms}'"),
        );
        fs::write(output, sdf)?;
        return Ok(());
    }
    if mode == "delayed-workers" {
        return deadline::qualify(
            Path::new(&a.next().ok_or("SDF")?),
            Path::new(&a.next().ok_or("worker")?),
            Path::new(&a.next().ok_or("scratch")?),
            a.next().ok_or("ticks")?.parse()?,
        );
    }
    if mode == "rpc-cuts" || mode == "rpc-stalls" {
        let worker = PathBuf::from(a.next().ok_or("worker")?);
        let sdf = PathBuf::from(a.next().ok_or("SDF")?);
        return rpc::run(
            &worker,
            &sdf,
            Path::new(&a.next().ok_or("scratch")?),
            a.next().ok_or("ticks")?.parse()?,
            mode == "rpc-stalls",
        );
    }
    if mode == "partition-disagreement" || mode == "partition-private-cut" {
        let sdf = PathBuf::from(a.next().ok_or("SDF")?);
        let worker = PathBuf::from(a.next().ok_or("worker")?);
        let dir = PathBuf::from(a.next().ok_or("scratch directory")?);
        return if mode == "partition-disagreement" {
            partitions::disagreement(&sdf, &worker, &dir)
        } else {
            partitions::refused(&sdf, &worker, &dir)
        };
    }
    if mode == "partition-child" || mode == "partition-cuts" {
        let sdf = PathBuf::from(a.next().ok_or("SDF path")?);
        let worker = PathBuf::from(a.next().ok_or("worker path")?);
        if mode == "partition-child" {
            return partitions::serve(&sdf, &worker, a.next().as_deref().map(Path::new));
        }
        return partitions::run(
            &sdf,
            &worker,
            Path::new(&a.next().ok_or("scratch directory")?),
            a.next().ok_or("ticks")?.parse()?,
        );
    }
    if mode == "archives" {
        let worker = a.next().ok_or("worker")?;
        let sdf = a.next().ok_or("SDF")?;
        let scratch = a.next().ok_or("scratch")?;
        return batch::archives(
            Path::new(&worker),
            Path::new(&sdf),
            Path::new(&scratch),
            a.next().ok_or("tick")?.parse()?,
            a.next().ok_or("repeats")?.parse()?,
            a.next().as_deref().map(Path::new),
        );
    }
    if mode == "bundle" || mode == "bundle-faults" {
        let worker = a.next().ok_or("worker")?;
        let scratch = a.next().ok_or("scratch directory")?;
        let sizes = batch::sizes(&a.next().ok_or("comma-separated sizes")?)?;
        return batch::run(
            Path::new(&worker),
            Path::new(&scratch),
            &sizes,
            batch::Run {
                ticks: a.next().ok_or("ticks")?.parse()?,
                faults: mode == "bundle-faults",
                seed: a.next().unwrap_or("31".into()).parse()?,
                threads: a.next().unwrap_or("1".into()).parse()?,
                processes: a.next().unwrap_or("1".into()).parse()?,
                transport: a.next().unwrap_or("pipe".into()),
            },
        );
    }
    if mode == "bundle-world" {
        let path = a.next().ok_or("output SDF")?;
        let sizes = batch::sizes(&a.next().ok_or("comma-separated sizes")?)?;
        return Ok(fs::write(
            path,
            rpc::authored(
                batch::fixture(
                    &sizes,
                    a.next().unwrap_or("31".to_owned()).parse()?,
                    a.next().unwrap_or("1".to_owned()).parse()?,
                    a.next().unwrap_or("1".to_owned()).parse()?,
                )?,
                &a.next().unwrap_or("pipe".into()),
            )?,
        )?);
    }
    if mode == "world" {
        let output = a.next().ok_or("output SDF")?;
        fs::write(
            output,
            pages::world(
                a.next().ok_or("population")?.parse()?,
                a.next().ok_or("seed")?.parse()?,
            )?,
        )?;
        return Ok(());
    }
    if mode == "reference" {
        return reference(
            Path::new(&a.next().ok_or("SDF")?),
            Path::new(&a.next().ok_or("output JSON")?),
            a.next().ok_or("ticks")?.parse()?,
        );
    }
    let w = PathBuf::from(a.next().ok_or("worker")?);
    let d = PathBuf::from(a.next().ok_or("scratch")?);
    let seed = a.next().ok_or("seed")?.parse()?;
    if mode == "actor" {
        actor(&w, &d, seed, a.next().as_deref())
    } else if mode == "run" {
        run(
            &w,
            &d,
            seed,
            a.next().ok_or("ticks")?.parse()?,
            a.next().map(PathBuf::from),
        )
    } else {
        Err("invalid mode".into())
    }
}
