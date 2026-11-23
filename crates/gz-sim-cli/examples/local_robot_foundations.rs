// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run the local rigid-robot qualification scenes and export their actual Bevy visual poses.
#[path = "local_robot_foundations/arm.rs"]
mod arm;
#[path = "local_robot_foundations/arm_cable.rs"]
mod arm_cable;
#[path = "local_robot_foundations/arm_recovery.rs"]
mod arm_recovery;
#[path = "local_robot_foundations/arm_sheet.rs"]
mod arm_sheet;
#[path = "local_robot_foundations/arm_switching.rs"]
mod arm_switching;
#[path = "local_robot_foundations/envelope.rs"]
mod envelope;
#[path = "local_robot_foundations/gripper.rs"]
mod gripper;
#[path = "local_robot_foundations/inertia.rs"]
mod inertia;
#[path = "local_robot_foundations/robot.rs"]
mod robot;
#[path = "local_robot_foundations/sheet_energy.rs"]
mod sheet_energy;
#[path = "local_robot_foundations/sheet_sweep.rs"]
mod sheet_sweep;
use clap::{Parser, Subcommand};
use glam::DVec3;
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    study: Study,
}
#[derive(Subcommand)]
enum Study {
    /// Lift a folded sheet and measure contact on each panel.
    ArmSheet { output: PathBuf },
    /// Compare nine folded-sheet mass and stiffness combinations on two material clocks.
    SheetSweep { output: PathBuf },
    /// Lift and release a flexible cable with the driven arm, and measure both jaw reactions.
    ArmCable { output: PathBuf },
    /// Compare a parked and always-dynamic arm across a complete loaded manipulation cycle.
    ArmSwitch { output: PathBuf },
    /// Rotate a rigid arm over an obstacle, pick up a free box and place it on a second stand.
    Arm { output: PathBuf },
    /// Pick up, carry and release a free parcel using friction between two moving jaws.
    Gripper { output: PathBuf },
    /// Disturb an airborne parcel in mid-swing and compare its recovery with fixed pressure.
    ArmMidSwing { output: PathBuf },
    /// Check tactile grasp recovery on the rotary arm against fixed jaw pressure.
    ArmRecovery { output: PathBuf },
    /// Measure the closed-loop load and slip envelope of the four-axis gripper.
    GraspEnvelope { output: PathBuf },
    /// Apply an off-center force to a single body and its two-link equivalent.
    Inertia { output: PathBuf },
    /// Drive a rail-mounted tool to push a parcel, then return the carriage to its start.
    Robot {
        output: PathBuf,
        /// Outbound carriage position in metres. The physical stops are at ±0.40 m.
        #[arg(long, default_value_t = 0.30)]
        travel: f64,
    },
    /// Record an unactuated SDF world named "default" at its authored physics step.
    Record {
        scene: PathBuf,
        ticks: usize,
        stride: usize,
        output: PathBuf,
    },
}

struct Recording {
    sim: Simulation,
    sdf: sdformat::Sdf,
    world: String,
    frames: Vec<(String, String)>,
    costs_ns: Vec<u64>,
}
impl Recording {
    fn new(xml: &str, filename: &str, world: &str) -> Result<Self> {
        let sdf = sdformat::load_str(xml, filename, &sdformat::NoResources)?;
        let sim = load_world_with_sdf_step(
            xml,
            filename,
            world,
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        Ok(Self {
            sim,
            sdf,
            world: world.to_string(),
            frames: Vec::new(),
            costs_ns: Vec::new(),
        })
    }
    fn capture(&mut self) -> Result<()> {
        self.frames.push(gz_sim_cli::primitive_frame(
            &self.sim,
            &self.sdf,
            &self.world,
            self.frames.len() as u64 + 1,
        )?);
        Ok(())
    }
    fn advance(&mut self) -> Result<()> {
        let started = Instant::now();
        self.sim.step_exact(1)?;
        self.costs_ns.push(started.elapsed().as_nanos() as u64);
        Ok(())
    }
    fn save(mut self, output: &Path) -> Result<()> {
        fs::create_dir_all(output)?;
        self.costs_ns.sort_unstable();
        let count = self.costs_ns.len();
        let p95 = self.costs_ns[(count * 95).div_ceil(100) - 1];
        let info = self.sim.committed_update_info()?;
        let metrics = json!({"steps":count, "frames":self.frames.len(), "step_ns":self.sim.step_duration().as_nanos(), "simulated_ns":info.sim_time.as_nanos(), "step_p95_ns":p95});
        let last = self.frames.len() - 1;
        write_json(
            &output.join("frames.json"),
            &json!({
                "kind":"authored-archive-v1", "camera":"right", "dt_ns":1,
                "captures":{"midpoint":last/2, "end":last},
            }),
        )?;
        gz_sim_cli::export_archive(&output.join("replay"), Default::default(), self.frames)?;
        write_json(&output.join("recording.json"), &metrics)
    }
}
fn write_json(path: &Path, value: &Value) -> Result<()> {
    fs::write(path, serde_json::to_vec_pretty(value)?)?;
    Ok(())
}
#[allow(deprecated)]
fn entity_id(sim: &Simulation, name: &str) -> Result<u64> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .and_then(|e| e.get::<EntityId>().map(EntityId::get))
        .ok_or_else(|| format!("missing {name}").into())
}
fn state(sim: &Simulation, id: u64) -> Result<BodyState> {
    let e = entity_by_id(sim.world(), id).ok_or("missing entity")?;
    let body = sim.world().get::<BodyHandle>(e).ok_or("missing body")?;
    sim.world()
        .resource::<PhysicsBackend>()
        .state(body.id())
        .ok_or_else(|| "missing native state".into())
}
fn record(scene: &Path, ticks: usize, stride: usize, output: &Path) -> Result<()> {
    if ticks == 0 || stride == 0 {
        return Err("ticks and stride must be positive".into());
    }
    if ticks.div_ceil(stride) >= 4096 {
        return Err("gz-web-rs recordings are limited to 4096 frames; increase the stride".into());
    }
    let mut run = Recording::new(
        &fs::read_to_string(scene)?,
        &scene.to_string_lossy(),
        "default",
    )?;
    run.capture()?;
    for tick in 1..=ticks {
        run.advance()?;
        if tick % stride == 0 || tick == ticks {
            run.capture()?;
        }
    }
    run.save(output)
}
fn write_new(output: &Path, generate: impl FnOnce(&Path) -> Result<()>) -> Result<()> {
    if output.try_exists()? {
        return Err(format!("choose a new recording directory: {}", output.display()).into());
    }
    let parent = output
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::tempdir_in(parent)?;
    generate(stage.path())?;
    fs::rename(stage.path(), output)?;
    Ok(())
}
fn main() -> Result<()> {
    match Args::parse().study {
        Study::ArmSheet { output } => write_new(&output, arm_sheet::run),
        Study::SheetSweep { output } => write_new(&output, sheet_sweep::run),
        Study::ArmCable { output } => write_new(&output, arm_cable::run),
        Study::ArmSwitch { output } => write_new(&output, arm_switching::run),
        Study::Arm { output } => write_new(&output, arm::run),
        Study::Gripper { output } => write_new(&output, gripper::run),
        Study::ArmMidSwing { output } => write_new(&output, arm_recovery::mid_swing::run),
        Study::ArmRecovery { output } => write_new(&output, arm_recovery::run),
        Study::GraspEnvelope { output } => write_new(&output, envelope::run),
        Study::Inertia { output } => write_new(&output, inertia::run),
        Study::Robot { output, travel } => write_new(&output, |p| robot::run(p, travel)),
        Study::Record {
            scene,
            ticks,
            stride,
            output,
        } => write_new(&output, |p| record(&scene, ticks, stride, p)),
    }
}
