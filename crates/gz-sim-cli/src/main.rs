// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use clap::Parser;
use gz_sim_scene::{EntityKind, Name, PhysicsEngine, WorldPose};
use std::{
    error::Error,
    io::{self, Write},
    num::NonZeroU64,
    path::PathBuf,
    process::ExitCode,
    time::Duration,
};

/// Run exact headless steps of a supported SDF world.
#[derive(Parser)]
#[command(name = "gz-sim-rs", version)]
struct Args {
    /// Local SDF world file or configured world directory.
    file: PathBuf,
    /// Additional resource roots, appended after environment roots.
    #[arg(long)]
    resource_path: Vec<PathBuf>,
    #[arg(long, default_value = "67108864")]
    max_xml_bytes: NonZeroU64,
    /// Fail loading if resource warnings exceed this budget.
    #[arg(long, default_value_t = 1024)]
    max_resource_warnings: usize,
    #[arg(long, default_value = "rapier", value_parser = parse_engine)]
    physics_engine: PhysicsEngine,
    #[arg(long, default_value = "default")]
    world: String,
    #[arg(long, default_value_t = 1000)]
    iterations: u64,
    #[arg(long, default_value = "1000000")]
    step_ns: NonZeroU64,
}

fn parse_engine(value: &str) -> Result<PhysicsEngine, String> {
    match value {
        "rapier" => Ok(PhysicsEngine::Rapier),
        "tpe" => Ok(PhysicsEngine::Tpe),
        _ => Err("expected rapier or tpe".into()),
    }
}

fn run(args: Args) -> Result<(), Box<dyn Error>> {
    let mut roots = Vec::new();
    for variable in ["SDF_PATH", "GZ_SIM_RESOURCE_PATH"] {
        if let Some(value) = std::env::var_os(variable) {
            for path in std::env::split_paths(&value).filter(|path| !path.as_os_str().is_empty()) {
                if !roots.contains(&path) {
                    roots.push(path);
                }
            }
        }
    }
    for path in args.resource_path {
        if !roots.contains(&path) {
            roots.push(path);
        }
    }
    let mut resolver =
        gz_assets::LocalResources::new(std::env::current_dir()?, roots, args.max_xml_bytes.get())?;
    resolver.set_warning_limit(args.max_resource_warnings);
    let loaded = (|| -> Result<_, Box<dyn Error>> {
        let resource = resolver.read_path(&args.file)?;
        Ok(gz_sim_scene::load_world_with_engine(
            &resource.xml,
            &resource.canonical_uri,
            &args.world,
            Duration::from_nanos(args.step_ns.get()),
            &resolver,
            args.physics_engine,
        )?)
    })();
    for warning in resolver.take_warnings()? {
        eprintln!("resource warning: {warning:?}");
    }
    let mut simulation = loaded?;
    simulation.step_exact(args.iterations)?;
    let elapsed = simulation.elapsed().as_nanos();
    let mut poses = simulation
        .world_mut()
        .query::<(&Name, &WorldPose, &EntityKind)>()
        .iter(simulation.world())
        .filter(|(_, _, kind)| **kind == EntityKind::Link)
        .map(|(name, pose, _)| (name.0.clone(), pose.translation))
        .collect::<Vec<_>>();
    poses.sort_by(|a, b| a.0.cmp(&b.0));
    let mut output = io::stdout().lock();
    writeln!(
        output,
        "simulation_ns={elapsed} iterations={}",
        args.iterations
    )?;
    for (name, [x, y, z]) in poses {
        writeln!(output, "{name} {x} {y} {z}")?;
    }
    Ok(())
}

fn main() -> ExitCode {
    match run(Args::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
