// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record ordinary SDF visuals at committed CPU/GPU handover barriers.
#[path = "support/auto_placement_plot.rs"]
mod auto_placement_plot;
#[path = "../../gz-sim-scene/examples/support/native_pair_drive.rs"]
mod native_drive;
#[path = "support/native_pair_plot.rs"]
mod native_pair_plot;
#[path = "support/native_staging_plot.rs"]
mod native_staging_plot;
#[path = "support/placement_plot.rs"]
mod placement_plot;
#[path = "../../gz-sim-scene/examples/support/native_support_drive.rs"]
mod support_drive;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    PhysicsEngine, WorldLoadOptions,
    gpu_scene::{
        self, EnvironmentState,
        ExecutionPlacement::{self, Cpu, Gpu},
        GpuSceneObservations, batch_boundary,
    },
    load_world_on_adapter, reset_loaded_world,
};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{error::Error, io::Write, path::Path};

const SDF: &str = include_str!("../../../examples/gpu_mixed_world.sdf");
const CELLS: [(u64, f64); 3] = [(9, 1.78), (31, 0.), (80, -1.83)];
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn switch(sim: &mut Simulation, id: u64, target: ExecutionPlacement) -> Result<()> {
    let at = batch_boundary(sim);
    let before = sim
        .world()
        .resource::<GpuSceneObservations>()
        .environments()
        .to_vec();
    let changed = gpu_scene::set_environment_placement(sim, at, &[id], target)?;
    assert_eq!(changed.moved, 1);
    assert_eq!(gpu_scene::environment_placement(sim, id)?, target);
    assert_eq!(
        sim.world()
            .resource::<GpuSceneObservations>()
            .environments(),
        before
    );
    Ok(())
}

fn capture(
    sim: &Simulation,
    sdf: &sdformat::Sdf,
    sequence: u64,
    contacts: Option<[bool; 3]>,
) -> Result<(String, String)> {
    let (scene, catalog) = gz_sim_cli::primitive_frame(sim, sdf, "default", sequence)?;
    let mut scene: Value = serde_json::from_str(&scene)?;
    let mut catalog: Value = serde_json::from_str(&catalog)?;
    for (lane, (id, y)) in CELLS.into_iter().enumerate() {
        let cpu = gpu_scene::environment_placement(sim, id)? == Cpu;
        // Three diagnostic primitives, kept out of the physics world. The inactive
        // shape fits fully inside the opaque base. Geometry and material stay fixed.
        let model = 1000 + id * 10;
        let name = if contacts.is_some() {
            format!("default::solver_{id}")
        } else {
            format!("default::placement_{id}")
        };
        for (i, role, z, shape, color) in [
            (
                1,
                "base",
                0.08,
                json!({"kind":"box","size":[0.31,0.32,0.30]}),
                [0.24, 0.28, 0.32, 1.],
            ),
            (
                2,
                if contacts.is_some() {
                    "native_rapier"
                } else {
                    "cpu"
                },
                if cpu { 0.32 } else { 0.08 },
                json!({"kind":"sphere","radius":0.10}),
                [0.95, 0.16, 0.75, 1.],
            ),
            (
                3,
                if contacts.is_some() {
                    "specialized_gpu"
                } else {
                    "gpu"
                },
                if cpu { 0.08 } else { 0.32 },
                json!({"kind":"box","size":[0.18,0.18,0.18]}),
                [0.99, 0.77, 0.10, 1.],
            ),
        ] {
            scene["visuals"].as_array_mut().unwrap().push(json!({"id":model+i,"name":format!("{name}::{role}"),"translation":[0.82,y,z],"rotation":[0.,0.,0.,1.],"shape":shape,"color":color}));
        }
        let mut visual_ids = vec![model + 1, model + 2, model + 3];
        if let Some(active) = contacts {
            let z = if active[lane] { 0.48 } else { 0.08 };
            scene["visuals"].as_array_mut().unwrap().push(json!({"id":model+4,"name":format!("{name}::recent_contact"),"translation":[0.82,y,z],"rotation":[0.,0.,0.,1.],"shape":{"kind":"sphere","radius":0.049},"color":[1.,0.31,0.13,1.]}));
            visual_ids.push(model + 4);
        }
        catalog["models"].as_array_mut().unwrap().push(json!({"model_id":model,"name":name,"visual_ids":visual_ids,"can_remove":false,"can_set_state":false}));
    }
    Ok((scene.to_string(), catalog.to_string()))
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("provide ADAPTER ARCHIVE TRACE.csv")?;
    if adapter == "--support" {
        let adapter = args.next().ok_or("provide adapter")?;
        let dir = args.next().ok_or("provide a new recording directory")?;
        if args.next().is_some() {
            return Err("unexpected recording argument".into());
        }
        return support_archive(&adapter, Path::new(&dir));
    }
    if adapter == "--native" {
        let adapter = args.next().ok_or("provide adapter")?;
        let output = args
            .next()
            .ok_or("provide a new native recording directory")?;
        if args.next().is_some() {
            return Err("unexpected native recording argument".into());
        }
        return native_archive(&adapter, Path::new(&output));
    }
    if matches!(
        adapter.as_str(),
        "--plot-native" | "--plot-support" | "--plot-staging" | "--plot-overlap"
    ) {
        let dir = args.next().ok_or("provide native measurement directory")?;
        if args.next().is_some() {
            return Err("unexpected plotting argument".into());
        }
        return match adapter.as_str() {
            "--plot-support" => native_pair_plot::draw_support(Path::new(&dir)),
            "--plot-staging" => native_staging_plot::draw(Path::new(&dir)),
            "--plot-overlap" => native_staging_plot::draw_overlap(Path::new(&dir)),
            _ => native_pair_plot::draw(Path::new(&dir)),
        };
    }
    if adapter == "--plot-auto" {
        let dir = args
            .next()
            .ok_or("provide automatic measurement directory")?;
        if args.next().is_some() {
            return Err("unexpected plotting argument".into());
        }
        return auto_placement_plot::draw(Path::new(&dir));
    }
    if adapter == "--plot" {
        let cost = args.next().ok_or("provide cost CSV")?;
        let crossover = args.next().ok_or("provide crossover CSV")?;
        let dir = args.next().ok_or("provide plot directory")?;
        if args.next().is_some() {
            return Err("unexpected plotting argument".into());
        }
        return placement_plot::draw(Path::new(&cost), Path::new(&crossover), Path::new(&dir));
    }
    let automatic = adapter == "--auto";
    let adapter = if automatic {
        args.next().ok_or("supply physical adapter")?
    } else {
        adapter
    };
    let dir = args
        .next()
        .ok_or("provide a new native recording directory")?;
    let trace = args.next().ok_or("provide a new trace CSV")?;
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    let xml=if automatic {
        SDF.replace("<observe ", "<automatic_placement window_ticks='16' min_residency_ticks='64' reprobe_ticks='360' hysteresis_percent='8'/><observe ")
    } else {
        SDF.replace(" profile=", " execution=\"cpu\" profile=")
    }.replace("every=\"10\"","every=\"1\"");
    let sdf = sdformat::load_str(&xml, "placed.sdf", &NoResources)?;
    let mut sim = load_world_on_adapter(
        &xml,
        "placed.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(&adapter),
    )?;
    let mut out = std::fs::File::create_new(trace)?;
    writeln!(out, "tick,id,owner,normal_impulse_Ns,switched")?;
    let mut frames = vec![];
    let mut contacts = [None; 3];
    let mut previous = [if automatic { Gpu } else { Cpu }; 3];
    for tick in 0..=700 {
        if tick != 0 {
            sim.step_exact(1)?;
        }
        let mut changed = false;
        for (i, (id, _)) in CELLS.iter().enumerate() {
            let env = sim
                .world()
                .resource::<GpuSceneObservations>()
                .environments()[i];
            assert_eq!(env.id, *id);
            let impulse = match env.state {
                EnvironmentState::SpherePlane(p) => p.normal_impulse,
                EnvironmentState::SpherePair(p) => p.normal_impulse,
                EnvironmentState::SupportedSpherePair(p) => p.pair.normal_impulse,
            };
            let first = impulse > 0. && contacts[i].is_none();
            if first {
                contacts[i] = Some(tick);
            }
            let change = if automatic {
                None
            } else if tick == [35, 80, 130][i] {
                Some(Gpu)
            } else if first {
                Some(Cpu)
            } else if tick == 585 + 28 * i {
                Some(Gpu)
            } else {
                None
            };
            if let Some(target) = change {
                switch(&mut sim, *id, target)?;
            }
            let actual = gpu_scene::environment_placement(&sim, *id)?;
            let switched = previous[i] != actual;
            changed |= switched;
            previous[i] = actual;
            writeln!(
                out,
                "{tick},{id},{:?},{impulse:.15e},{}",
                gpu_scene::environment_placement(&sim, *id)?,
                switched
            )?;
        }
        if automatic
            && let Some(event) = sim
                .world()
                .resource::<gpu_scene::automatic::AutomaticPlacement>()
                .events()
                .back()
                .filter(|e| e.at.tick == tick as u64)
        {
            println!(
                "tick={tick} reason={:?} owner={:?} evidence={:?}",
                event.reason, event.owner, event.evidence
            );
        }
        if tick % 10 == 0 || changed {
            frames.push(capture(&sim, &sdf, (frames.len() + 1) as u64, None)?);
        }
    }
    assert!(contacts.iter().all(Option::is_some));
    if automatic {
        let policy = sim
            .world()
            .resource::<gpu_scene::automatic::AutomaticPlacement>();
        assert!(
            policy.probes() > 0
                && policy
                    .events()
                    .iter()
                    .any(|e| e.reason == gpu_scene::automatic::PlacementReason::Commit),
            "no measured handover for this recording"
        );
    }
    let count = frames.len();
    gz_sim_cli::export_archive(Path::new(&dir), Default::default(), frames)?;
    reset_loaded_world(&mut sim, |_| ())?;
    let authored = if automatic { (0, 3) } else { (3, 0) };
    assert_eq!(gpu_scene::execution_counts(&sim), Some(authored));
    println!(
        "frames={count} contacts_at_ms={contacts:?} measured IDs 9,31,80; reset restored authored owners {authored:?}"
    );
    Ok(())
}

fn native_archive(adapter: &str, output: &Path) -> Result<()> {
    let mut sim = native_drive::load(adapter)?;
    let sdf = sdformat::load_str(native_drive::SDF, "native_gpu_spheres.sdf", &NoResources)?;
    let mut drive = native_drive::Drive::new(&sim);
    let mut frames = vec![];
    let mut moves = 0;
    let mut rejections = 0;
    for tick in 0..=700u64 {
        if tick > 0 {
            sim.step_exact(1)?;
        }
        let events = drive.boundary(&mut sim)?;
        for &e in &events {
            println!("t={tick} {e:?}");
            match e {
                native_drive::Event::Moved(_, _) => moves += 1,
                native_drive::Event::ContactRefusal(_)
                | native_drive::Event::ClearanceRefusal(_) => rejections += 1,
                _ => {}
            }
        }
        if tick % 8 == 0 || tick == 700 || !events.is_empty() {
            let lamps = drive.impact.map(|t| t.is_some_and(|k| tick - k <= 40));
            frames.push(capture(&sim, &sdf, (frames.len() + 1) as u64, Some(lamps))?);
        }
    }
    assert_eq!(moves, 8);
    assert!(rejections >= 3);
    assert!(drive.impact.iter().all(Option::is_some));
    println!(
        "frames={} impacts={:?} unchanged exact-barrier moves={moves} refused={rejections}",
        frames.len(),
        drive.impact
    );
    gz_sim_cli::export_archive(output, Default::default(), frames)?;
    Ok(())
}

fn support_archive(adapter: &str, output: &Path) -> Result<()> {
    let mut sim = support_drive::load(adapter)?;
    let sdf = sdformat::load_str(support_drive::SDF, "native_gpu_support.sdf", &NoResources)?;
    let mut drive = support_drive::Drive::new(&sim);
    let mut frames = vec![];
    let mut moved = 0;
    let mut refused = 0;
    for t in 0..=support_drive::END {
        if t > 0 {
            sim.step_exact(1)?;
        }
        let events = drive.boundary(&mut sim)?;
        for &e in &events {
            println!("tick={t} {e:?}");
            match e {
                support_drive::Event::Moved(id, owner) => {
                    assert!(support_drive::IDS.contains(&id));
                    assert_eq!(gpu_scene::pair_solver(&sim, id)?, owner);
                    moved += 1;
                }
                support_drive::Event::Refused(id, reason) => {
                    assert!(support_drive::IDS.contains(&id));
                    assert!(matches!(
                        reason,
                        gpu_scene::NativeHandoffRefusal::ContactHistory
                            | gpu_scene::NativeHandoffRefusal::SweptClearance
                    ));
                    refused += 1;
                }
                support_drive::Event::Kick(id) => {
                    assert!(support_drive::IDS.contains(&id));
                }
            }
        }
        if t.is_multiple_of(10) || !events.is_empty() {
            let frame = sim
                .world()
                .resource::<GpuSceneObservations>()
                .environments();
            let lamps = std::array::from_fn(|i| match frame[i].state {
                EnvironmentState::SpherePlane(s) => s.normal_impulse > 1e-6,
                _ => false,
            });
            frames.push(capture(&sim, &sdf, (frames.len() + 1) as u64, Some(lamps))?);
        }
    }
    assert_eq!(moved, 6);
    assert!(refused >= 9);
    println!(
        "frames={} exact solver changes={moved} loaded refusals={refused}",
        frames.len()
    );
    gz_sim_cli::export_archive(output, Default::default(), frames)?;
    Ok(())
}
