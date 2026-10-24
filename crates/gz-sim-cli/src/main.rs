use clap::Parser;
use gz_sim_scene::{Name, WorldPose};
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
    /// Self-contained SDF file.
    file: PathBuf,
    #[arg(long, default_value = "default")]
    world: String,
    #[arg(long, default_value_t = 1000)]
    iterations: u64,
    #[arg(long, default_value = "1000000")]
    step_ns: NonZeroU64,
}

fn run(args: Args) -> Result<(), Box<dyn Error>> {
    let xml = std::fs::read_to_string(&args.file)?;
    let mut simulation = gz_sim_scene::load_world(
        &xml,
        &args.file.to_string_lossy(),
        &args.world,
        Duration::from_nanos(args.step_ns.get()),
        &sdformat::NoResources,
    )?;
    simulation.step_exact(args.iterations)?;
    let elapsed = simulation.elapsed().as_nanos();
    let mut poses = simulation
        .world_mut()
        .query::<(&Name, &WorldPose)>()
        .iter(simulation.world())
        .map(|(name, pose)| (name.0.clone(), pose.translation))
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
