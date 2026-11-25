// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_cli::{live, with_fault};

use clap::{Parser, ValueEnum};

#[derive(Clone, Copy, ValueEnum)]
enum RefinementScope {
    Island,
    Scene,
}
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
    #[arg(long, default_value = "rapier", value_parser = parse_engine, conflicts_with = "replay")]
    physics_engine: PhysicsEngine,
    /// Resolve friction at each contact point in local Rapier, local multi-fidelity or a three- to eight-worker pool.
    #[arg(long, conflicts_with = "replay")]
    rapier_per_point_friction: bool,
    /// Resolve cold isolated cylinder-cap impacts against fixed finite box faces in local Rapier.
    #[arg(long, conflicts_with = "replay")]
    rapier_axial_cap_impact: bool,
    /// Use a 120 Hz fixed-body contact response in local multi-fidelity physics (default 60 Hz).
    #[arg(long, conflicts_with_all = ["replay", "hinge_session", "returned_robot_session", "resume_pool_checkpoint", "save_pool_checkpoint", "live_pool_checkpoint"])]
    stiff_static_contacts: bool,
    /// Use SDFormat friction for omitted collision surfaces in the local CPU engine.
    #[arg(long, conflicts_with_all = ["replay", "hinge_session", "returned_robot_session", "resume_pool_checkpoint", "save_pool_checkpoint", "live_pool_checkpoint"])]
    sdf_default_friction: bool,
    /// Number of local authoritative processes in the distributed engine.
    #[arg(long,conflicts_with="replay",value_parser=clap::value_parser!(u8).range(2..=8))]
    physics_workers: Option<u8>,
    /// Refine contact islands within a 10 ms tick. Use =scene to refine the entire scene together.
    #[arg(long, value_enum, num_args = 0..=1, require_equals = true, default_missing_value = "island", conflicts_with = "replay")]
    adaptive_contacts: Option<RefinementScope>,
    /// Seed distributed workers by prospective contact group and estimated solver work.
    #[arg(long, conflicts_with_all = ["replay", "gpu_adapter"])]
    contact_placement: bool,
    /// Negotiate up to 64 bodies per native solver on three to eight distributed workers.
    #[arg(long, conflicts_with_all = ["replay", "gpu_adapter"])]
    native_64: bool,
    /// Seed one ID-ordered native reference solver on worker zero for uniform distributed stepping.
    #[arg(long, requires = "native_64", conflicts_with_all = ["replay", "gpu_adapter", "contact_placement"])]
    canonical_reference: bool,
    /// Sample local work periodically and move an island if predicted savings repay the transfer.
    #[cfg(feature = "distributed-physics")]
    #[arg(long,value_name="SAMPLE,PERIOD,HORIZON,HANDOFF_US[,MAX_IDLE_PERIODS]",conflicts_with_all=["replay","canonical_reference","gpu_adapter"])]
    balance_islands: Option<gz_sim_scene::PeriodicPlacement>,
    /// Cap the number of worker-local solver helpers. Zero keeps each native worker serial.
    #[arg(long, conflicts_with_all = ["replay", "gpu_adapter"])]
    physics_worker_helpers: Option<usize>,
    /// Use packed native handoffs on distributed workers that advertise support.
    #[arg(long, conflicts_with_all = ["replay", "gpu_adapter"])]
    packed_handoffs: bool,
    /// Explicit Vulkan hardware selector for the optional gpu-batch physics engine.
    #[arg(long, conflicts_with = "replay")]
    gpu_adapter: Option<String>,
    /// Create or resume a floating revolute tree, or a two-link hinge with supported contacts.
    #[arg(long, visible_alias="articulation-session", conflicts_with_all=["replay","adaptive_contacts","native_64","canonical_reference","contact_placement",
        "packed_handoffs","rapier_per_point_friction","rapier_axial_cap_impact","gpu_adapter","physics_worker_helpers","save_pool_checkpoint",
        "resume_pool_checkpoint","live_pool_checkpoint","web_lifecycle"])]
    #[cfg_attr(
        feature = "distributed-physics",
        arg(conflicts_with = "balance_islands")
    )]
    hinge_session: Option<PathBuf>,
    /// Reopen an ordinary local-owner humanoid cut using the original SDF and no worker.
    #[arg(long, conflicts_with_all = ["replay", "hinge_session", "physics_worker", "physics_workers",
        "adaptive_contacts", "contact_placement", "native_64", "canonical_reference",
        "physics_worker_helpers", "packed_handoffs", "rapier_per_point_friction", "rapier_axial_cap_impact", "gpu_adapter",
        "save_pool_checkpoint", "resume_pool_checkpoint", "live_pool_checkpoint", "camera_worker", "web_lifecycle"])]
    #[cfg_attr(
        feature = "distributed-physics",
        arg(conflicts_with = "balance_islands")
    )]
    returned_robot_session: Option<PathBuf>,
    /// Trusted local worker executable. Defaults to sibling physics_worker or gz_pair_worker for resident GPU processes.
    #[arg(long, conflicts_with = "replay")]
    physics_worker: Option<PathBuf>,
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
    /// Seal a new distributed or curved-worker SDF checkpoint at the selected warmup tick, before serving.
    #[arg(long, requires = "checkpoint_at", conflicts_with_all = ["resume_pool_checkpoint", "replay", "camera_worker", "record_scene"])]
    save_pool_checkpoint: Option<PathBuf>,
    /// Number of real Bevy ticks before sealing the requested pool checkpoint.
    #[arg(long, requires = "save_pool_checkpoint")]
    checkpoint_at: Option<u64>,
    /// Restore a passive or curved worker pool using this SDF and the original execution settings.
    #[arg(long, conflicts_with_all = ["replay", "camera_worker", "run"])]
    resume_pool_checkpoint: Option<PathBuf>,
    /// Seal the current paused physics pool to this new local file when the operator sends SIGUSR1 (Unix).
    #[arg(long, requires = "server", conflicts_with_all = ["replay", "camera_worker"])]
    live_pool_checkpoint: Option<PathBuf>,
    /// Trusted absolute offscreen worker executable (requires the rgb-camera build feature).
    #[arg(long, requires = "server")]
    camera_worker: Option<PathBuf>,
    /// Explicitly run one RGB and one depth camera with independent outputs.
    #[arg(long, requires = "camera_worker")]
    camera_mixed_pair: bool,
}

/// Accepts short names and the upstream `gz sim --physics-engine` plugin names.
fn parse_engine(value: &str) -> Result<PhysicsEngine, String> {
    match value {
        "rapier" => Ok(PhysicsEngine::Rapier),
        "tpe" | "gz-physics-tpe-plugin" => Ok(PhysicsEngine::Tpe),
        "multifidelity" => Ok(PhysicsEngine::MultiFidelity),
        "distributed" => Ok(PhysicsEngine::Distributed),
        "gpu-batch" => Ok(PhysicsEngine::GpuBatch),
        _ => Err("expected rapier, tpe, multifidelity, distributed, or gpu-batch".into()),
    }
}

fn run(args: Args) -> Result<(), Box<dyn Error>> {
    let engine = match (
        args.physics_engine,
        args.physics_workers,
        args.adaptive_contacts,
    ) {
        (PhysicsEngine::Distributed, width, Some(RefinementScope::Island)) => {
            PhysicsEngine::DistributedIslands(width.unwrap_or(2))
        }
        (PhysicsEngine::Distributed, width, Some(RefinementScope::Scene)) => {
            PhysicsEngine::DistributedAdaptive(width.unwrap_or(2))
        }
        (PhysicsEngine::Distributed, Some(n @ 3..=8), None) => PhysicsEngine::DistributedPool(n),
        (PhysicsEngine::Distributed, Some(2), None) => PhysicsEngine::Distributed,
        (_, _, Some(_)) => {
            return Err("--adaptive-contacts requires --physics-engine distributed".into());
        }
        (selected, None, None) => selected,
        _ => return Err("--physics-workers requires --physics-engine distributed".into()),
    };
    if args.hinge_session.is_some() && args.physics_engine != PhysicsEngine::Distributed {
        return Err("--hinge-session requires --physics-engine distributed".into());
    }
    if args.returned_robot_session.is_some()
        && (engine != PhysicsEngine::MultiFidelity
            || args.step_ns.is_some_and(|step| step.get() != 1_000_000))
    {
        return Err("--returned-robot-session requires --physics-engine multifidelity and its authored 1 ms step".into());
    }
    if args.rapier_per_point_friction && !engine.supports_per_point_friction() {
        return Err("--rapier-per-point-friction requires local Rapier, local multi-fidelity or three to eight distributed workers".into());
    }
    if args.replay {
        if args.world != "default" {
            return Err("--replay accepts the recorded default world".into());
        }
        return live::replay(
            &args.file,
            args.web_viewer.expect("clap requires viewer for replay"),
        );
    }
    if args.record_scene.is_some() && args.world != "default" {
        return Err("--record-scene requires the admitted default viewer world".into());
    }
    if args.web_lifecycle
        && !matches!(
            args.physics_engine,
            PhysicsEngine::Rapier
                | PhysicsEngine::Distributed
                | PhysicsEngine::MultiFidelity
                | PhysicsEngine::GpuBatch
        )
    {
        return Err(
            "--web-lifecycle requires the Rapier, multifidelity, distributed, or gpu-batch physics engine".into(),
        );
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
    let operator_checkpoint = args
        .live_pool_checkpoint
        .clone()
        .map(gz_sim_cli::OperatorCheckpoint::install)
        .transpose()?;
    let loaded = (|| -> Result<_, Box<dyn Error>> {
        let resource = resolver.read_path(&args.file)?;
        let options = gz_sim_scene::WorldLoadOptions {
            engine,
            step: args.step_ns.map(|step| Duration::from_nanos(step.get())),
            noise_seed: [0; 32],
            distributed_worker: args.physics_worker.as_deref(),
        };
        // Scene features may also be enabled by another workspace consumer.
        #[allow(clippy::needless_update)]
        let runtime = gz_sim_scene::WorldRuntimeOptions {
            rapier_per_point_friction: args.rapier_per_point_friction,
            rapier_axial_cap_impact: args.rapier_axial_cap_impact,
            stiff_static_contacts: args.stiff_static_contacts,
            sdf_default_friction: args.sdf_default_friction,
            distributed_packed_handoffs: args.packed_handoffs,
            gpu_adapter: args.gpu_adapter.as_deref(),
            distributed_contact_placement: args.contact_placement,
            distributed_native_64: args.native_64,
            distributed_canonical_reference: args.canonical_reference,
            distributed_local_helper_limit: args.physics_worker_helpers,
            #[cfg(feature = "distributed-physics")]
            distributed_periodic_placement: args.balance_islands,
            ..Default::default()
        };
        let simulation = if let Some(directory) = &args.returned_robot_session {
            #[cfg(not(feature = "distributed-physics"))]
            {
                let _ = directory;
                return Err(
                    "--returned-robot-session requires the distributed-physics Cargo feature"
                        .into(),
                );
            }
            #[cfg(feature = "distributed-physics")]
            gz_sim_scene::distributed_hinge::load_returned_grounded_robot_world(
                &resource.xml,
                &resource.canonical_uri,
                &args.world,
                directory,
            )?
        } else if let Some(directory) = &args.hinge_session {
            #[cfg(not(feature = "distributed-physics"))]
            {
                let _ = directory;
                return Err(
                    "--hinge-session requires the distributed-physics Cargo feature".into(),
                );
            }
            #[cfg(feature = "distributed-physics")]
            {
                let worker = match &args.physics_worker {
                    Some(worker) => worker.clone(),
                    None => std::env::current_exe()?.with_file_name("physics_worker"),
                };
                gz_sim_scene::distributed_hinge::load_distributed_articulation_world_with_step(
                    &resource.xml,
                    &resource.canonical_uri,
                    &args.world,
                    &worker,
                    directory,
                    args.physics_workers,
                    options.step,
                )?
            }
        } else if args.save_pool_checkpoint.is_some()
            || args.resume_pool_checkpoint.is_some()
            || operator_checkpoint.is_some()
        {
            #[cfg(not(feature = "distributed-physics"))]
            return Err("pool checkpoints require the distributed-physics Cargo feature".into());
            #[cfg(feature = "distributed-physics")]
            {
                if engine == PhysicsEngine::MultiFidelity {
                    use gz_sim_scene::trajectory_workers::checkpoint;
                    let exe = match options.distributed_worker {
                        Some(path) => path.to_owned(),
                        None => std::env::current_exe()?.with_file_name("physics_worker"),
                    };
                    let options = gz_sim_scene::WorldLoadOptions {
                        distributed_worker: Some(&exe),
                        ..options
                    };
                    let mut sim = if let Some(path) = &args.resume_pool_checkpoint {
                        checkpoint::resume(&resource.xml, &args.world, options, runtime, path)?
                    } else {
                        gz_sim_scene::load_world_with_runtime(
                            &resource.xml,
                            &resource.canonical_uri,
                            &args.world,
                            &resolver,
                            options,
                            runtime,
                        )?
                    };
                    if let Some(path) = args.save_pool_checkpoint.as_deref() {
                        sim.step_exact(args.checkpoint_at.expect("clap requires checkpoint tick"))
                            .map_err(|e| with_fault(&sim, e))?;
                        sim.set_paused(!args.run);
                        checkpoint::save(&sim, path)?;
                    }
                    sim
                } else {
                    use gz_sim_scene::passive_checkpoint::PassivePoolScene;
                    let mut scene = if let Some(path) = &args.resume_pool_checkpoint {
                        PassivePoolScene::resume(
                            &resource.xml,
                            &resource.canonical_uri,
                            &args.world,
                            options,
                            runtime,
                            path,
                        )?
                    } else {
                        PassivePoolScene::load(
                            &resource.xml,
                            &resource.canonical_uri,
                            &args.world,
                            options,
                            runtime,
                        )?
                    };
                    if let Some(path) = args.save_pool_checkpoint.as_deref() {
                        let count = args.checkpoint_at.expect("clap requires checkpoint tick");
                        scene
                            .step_exact(count)
                            .map_err(|error| with_fault(scene.simulation(), error))?;
                        scene.set_paused(!args.run);
                        scene.save(path)?;
                    }
                    if operator_checkpoint.is_some() {
                        scene.into_checkpointable_runtime()
                    } else {
                        scene.into_simulation()
                    }
                }
            }
        } else {
            gz_sim_scene::load_world_with_runtime(
                &resource.xml,
                &resource.canonical_uri,
                &args.world,
                &resolver,
                options,
                runtime,
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
    let running = if args.resume_pool_checkpoint.is_some() {
        !simulation.is_paused()
    } else {
        args.run
    };
    let iterations = if args.server {
        let result = live::run_with_operator(
            simulation,
            &args.world,
            running,
            args.iterations.unwrap_or(0),
            live::CameraOptions {
                worker: args.camera_worker,
                mixed_pair: args.camera_mixed_pair,
            },
            live::ViewerOptions {
                port: args.web_viewer,
                lifecycle_sdf,
                recording_path: args.record_scene,
            },
            operator_checkpoint,
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
    for (name, body) in gz_sim_scene::deformable::bodies(&simulation) {
        let particles = body.particles();
        let min_z = particles
            .iter()
            .map(|p| p.position[2])
            .fold(f64::INFINITY, f64::min);
        writeln!(
            output,
            "deformable {name} particles={} min_z={min_z}",
            particles.len()
        )?;
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
