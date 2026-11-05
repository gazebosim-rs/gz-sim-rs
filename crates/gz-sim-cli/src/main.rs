// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
mod live;
mod web;

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

/// Run a supported SDF world headlessly, offline or as a transport server.
#[derive(Parser)]
#[command(name = "gz-sim-rs", version)]
struct Args {
    /// Local SDF world file/directory, or finalized recording directory with --replay.
    file: PathBuf,
    /// Additional resource roots, appended after environment roots.
    #[arg(long)]
    resource_path: Vec<PathBuf>,
    /// Offline Fuel cache root for model includes; misses never download.
    #[arg(long)]
    fuel_cache_path: Option<PathBuf>,
    #[arg(long, default_value = "67108864")]
    max_xml_bytes: NonZeroU64,
    /// Fail loading if resource warnings exceed this budget.
    #[arg(long, default_value_t = 1024)]
    max_resource_warnings: usize,
    #[arg(long, default_value = "rapier", value_parser = parse_engine)]
    physics_engine: PhysicsEngine,
    #[arg(long, default_value = "default")]
    world: String,
    /// Advancing steps: offline default 1000; server default 0 means unlimited.
    #[arg(long)]
    iterations: Option<u64>,
    /// Publish clock, poses and IMU, and accept remote world controls.
    #[arg(short = 's', long)]
    server: bool,
    /// Start the server running instead of paused.
    #[arg(short = 'r', long, requires = "server")]
    run: bool,
    /// Override the SDF physics step, in nanoseconds.
    #[arg(long)]
    step_ns: Option<NonZeroU64>,
    /// Serve one local gzweb-rs WebSocket client; viewer HTTP origin is port 8080 or 8081.
    #[arg(long, requires = "server", value_parser = clap::value_parser!(u16).range(1..))]
    web_viewer: Option<u16>,
    /// Enable bounded local template spawn/remove and entity-state editing.
    /// Camera-owned scene mutation requires a later renderer admission phase.
    #[arg(long, requires = "web_viewer", conflicts_with = "camera_worker")]
    web_lifecycle: bool,
    /// Record admitted scene/poses and committed browser controls into a new local directory.
    #[arg(long, requires = "web_lifecycle", conflicts_with_all = ["replay", "camera_worker"])]
    record_scene: Option<PathBuf>,
    /// Present a finalized local recording read-only; no native world is created.
    #[arg(long, requires = "web_viewer", conflicts_with_all = ["web_lifecycle", "camera_worker", "run", "iterations", "step_ns", "resource_path", "fuel_cache_path"])]
    replay: bool,
    /// Trusted absolute offscreen worker executable (requires the rgb-camera build feature).
    #[arg(long, requires = "server")]
    camera_worker: Option<PathBuf>,
}

/// Accepts short names and the upstream `gz sim --physics-engine` plugin names.
fn parse_engine(value: &str) -> Result<PhysicsEngine, String> {
    match value {
        "rapier" => Ok(PhysicsEngine::Rapier),
        "dart" | "gz-physics-dartsim-plugin" => Ok(PhysicsEngine::Dart),
        "mujoco" | "gz-physics-mujoco-plugin" => Ok(PhysicsEngine::Mujoco),
        "tpe" | "gz-physics-tpe-plugin" => Ok(PhysicsEngine::Tpe),
        _ => Err("expected rapier, dart, mujoco or tpe".into()),
    }
}

/// Reports the failed system's own diagnostic, such as a physics engine error,
/// instead of the generic terminal step error that wraps it.
fn with_fault(
    simulation: &gz_sim_core::Simulation,
    error: impl Into<Box<dyn Error>>,
) -> Box<dyn Error> {
    match simulation.fault() {
        Some(fault) => fault.to_string().into(),
        None => error.into(),
    }
}

fn run(args: Args) -> Result<(), Box<dyn Error>> {
    if args.replay {
        if args.world != "default" || args.physics_engine != PhysicsEngine::Rapier {
            return Err(
                "--replay accepts the recorded default world and no backend selection".into(),
            );
        }
        return live::replay(
            &args.file,
            args.web_viewer.expect("clap requires viewer for replay"),
        );
    }
    if args.record_scene.is_some() && args.world != "default" {
        return Err("--record-scene requires the admitted default viewer world".into());
    }
    if args.web_lifecycle && args.physics_engine != PhysicsEngine::Rapier {
        return Err("--web-lifecycle requires the Rapier physics engine".into());
    }
    #[cfg(not(feature = "rgb-camera"))]
    if args.camera_worker.is_some() {
        return Err("--camera-worker requires a build with the rgb-camera feature".into());
    }
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
    let mut fuel = gz_fuel_tools::ClientConfig::default();
    if let Some(path) = args.fuel_cache_path {
        fuel.cache_location = path;
    }
    resolver = resolver.with_cached_fuel(gz_fuel_tools::FuelClient::new(fuel));
    resolver.set_warning_limit(args.max_resource_warnings);
    let loaded = (|| -> Result<_, Box<dyn Error>> {
        let resource = resolver.read_path(&args.file)?;
        let simulation = if let Some(step) = args.step_ns {
            gz_sim_scene::load_world_with_engine(
                &resource.xml,
                &resource.canonical_uri,
                &args.world,
                Duration::from_nanos(step.get()),
                &resolver,
                args.physics_engine,
            )?
        } else {
            gz_sim_scene::load_world_with_sdf_step(
                &resource.xml,
                &resource.canonical_uri,
                &args.world,
                &resolver,
                args.physics_engine,
            )?
        };
        let lifecycle_sdf = if args.web_lifecycle {
            Some(sdformat::load_str(
                &resource.xml,
                &resource.canonical_uri,
                &resolver,
            )?)
        } else {
            None
        };
        Ok((simulation, lifecycle_sdf))
    })();
    for warning in resolver.take_warnings()? {
        eprintln!("resource warning: {warning:?}");
    }
    let (mut simulation, lifecycle_sdf) = loaded?;
    let iterations = if args.server {
        let result = live::run(
            simulation,
            &args.world,
            args.run,
            args.iterations.unwrap_or(0),
            args.camera_worker,
            live::ViewerOptions {
                port: args.web_viewer,
                lifecycle_sdf,
                recording_path: args.record_scene,
            },
        )?;
        simulation = result.0;
        result.1
    } else {
        let count = args.iterations.unwrap_or(1000);
        simulation
            .step_exact(count)
            .map_err(|error| with_fault(&simulation, error))?;
        count
    };
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
    writeln!(output, "simulation_ns={elapsed} iterations={}", iterations)?;
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

#[cfg(test)]
#[path = "../tests/private/recording_cli.rs"]
mod recording_cli_tests;
