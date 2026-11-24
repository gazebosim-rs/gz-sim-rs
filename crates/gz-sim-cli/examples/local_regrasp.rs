// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure recovery from one jaw losing contact during an accelerating vertical lift.
#[path = "closed_loop_carry/rig.rs"]
mod rig;

use bevy_tasks::{ComputeTaskPool, TaskPoolBuilder};
use clap::{Parser, ValueEnum};
use gz_sim_core::EntityId;
use gz_sim_scene::grasp_guard::{ContactRecovery, ContactRecoveryConfig, RecoveryState};
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step, queue_link_world_wrench};
pub(crate) use rig::Grid;
use rig::{BASE, DT, Rig};
use serde::Serialize;
use serde_json::json;
use std::{
    error::Error,
    fs,
    io::{BufWriter, Write},
    num::NonZeroUsize,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const ARM: u32 = 1050;

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum, Serialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum Case {
    Baseline,
    Left,
    Right,
    Dropout,
    Brief,
    Bounded,
    Reacquire,
    Settling,
    Repeat,
    Prolonged,
}

impl Case {
    /// Fault drives the selected jaw outward during these public intervals.
    fn fault(self, elapsed_ticks: u32) -> Option<(usize, f64)> {
        use Case::*;
        let (side, length, force) = match self {
            Left | Repeat => (0, 8, 12.2),
            Right => (1, 8, 12.2),
            Brief => (0, 2, 12.2),
            Bounded => (0, 23, 12.2),
            Reacquire => (1, 14, 12.2),
            Settling => (0, 12, 12.2),
            Prolonged => (0, 185, 12.2),
            Baseline | Dropout => return None,
        };
        if self == Settling && (49..53).contains(&elapsed_ticks) {
            return Some((1, 12.2));
        }
        if self == Repeat && (175..183).contains(&elapsed_ticks) {
            return Some((1, 12.2));
        }
        (elapsed_ticks < length).then_some((side, force))
    }
}

#[derive(Parser)]
#[command(about)]
struct Args {
    #[arg(value_enum)]
    case: Case,
    #[arg(value_enum)]
    grid: Grid,
    /// New directory for the tick measurements, summary and gz-web-rs archive.
    output: PathBuf,
    #[arg(long, default_value = "1")]
    threads: NonZeroUsize,
}

fn main() -> Result<()> {
    let args = Args::parse();
    ComputeTaskPool::get_or_init(|| {
        TaskPoolBuilder::new()
            .num_threads(args.threads.get())
            .build()
    });
    run(args.case, args.grid, &args.output)
}

pub(crate) fn run(case: Case, grid: Grid, output: &Path) -> Result<()> {
    let name = output
        .file_name()
        .ok_or("expected a new recording directory")?;
    let parent = output.parent().unwrap();
    fs::create_dir_all(parent)?;
    let directory = tempfile::Builder::new()
        .prefix(name)
        .rand_bytes(0)
        .tempdir_in(parent)?;
    let name = "local_offset_gripper_carry.sdf";
    let sdf = sdformat::load_str(BASE, name, &sdformat::NoResources)?;
    let link = &sdf.worlds[0]
        .models
        .iter()
        .find(|m| m.name == "parcel")
        .unwrap()
        .links[0];
    let mut sim = load_world_with_sdf_step(
        BASE,
        name,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    rig::configure(&mut sim, grid)?;
    let rig = Rig::new(&mut sim, link.inertial.pose.transform.translation);
    let mut links = sim.world_mut().query::<(&Name, &EntityId)>();
    let pad_ids = ["default::gripper::left", "default::gripper::right"].map(|pad| {
        links
            .iter(sim.world())
            .find(|(n, _)| n.0 == pad)
            .unwrap()
            .1
            .get()
    });
    let visuals = gz_sim_cli::ArchiveVisuals::new(&sim)?;
    let mut frames = vec![visuals.frame(&sim, &sdf, "default", 1)?];
    let mut ticks = BufWriter::new(fs::File::create(output.join("ticks.jsonl"))?);
    let settings = ContactRecoveryConfig::default();
    let mut controller = ContactRecovery::default();
    let mut state = RecoveryState::Held;
    let mut jaw = [4.2; 2];
    let mut lift = 0.;
    let mut external = [0.; 2];
    let mut baseline = None;
    let mut max_slip: f64 = 0.;
    let mut armed_load_ticks = [0; 4];
    let mut missed_packets = 0;
    let mut native_steps: u64 = 0;
    let mut times = Vec::new();
    let mut transitions = Vec::new();
    let mut last_label = String::new();
    rig.grip(&mut sim, jaw[0])?;
    for tick in 1..=1450 {
        let start = Instant::now();
        sim.step_exact(1)?;
        let us = start.elapsed().as_secs_f64() * 1e6;
        times.push(us);
        let o = rig.read(&sim);
        native_steps += u64::from(o.native_steps);
        let relative = o.parcel_z - o.pad_z;
        if tick == ARM {
            baseline = Some(relative);
        }
        let blind = case == Case::Dropout && (ARM + 1..=ARM + 25).contains(&tick);
        let speed = if tick >= ARM {
            rig.tactile_speed(&sim)
        } else {
            None
        };
        if tick > ARM {
            let loaded = o.normal.map(|n| n >= settings.min_load_n);
            let index = match loaded {
                [false, false] => 0,
                [true, false] => 1,
                [false, true] => 2,
                [true, true] => 3,
            };
            armed_load_ticks[index] += 1;
            missed_packets += u32::from(blind);
            state = controller.observe(
                DT,
                (!blind).then_some(o.normal),
                if blind { None } else { speed },
            );
        }
        let label = if tick < ARM {
            String::from("Preload")
        } else {
            format!("{state:?}")
        };
        let t = sim.elapsed().as_nanos() as u64;
        if label != last_label {
            transitions.push(json!({"time_ns":t,"state":label}));
            last_label.clone_from(&label);
        }
        let slip = baseline.map_or(0., |h| h - relative);
        max_slip = max_slip.max(slip);
        let next_jaw = [controller.squeeze_n(); 2];
        let mut next_external = [0.; 2];
        if tick >= ARM
            && let Some((side, force)) = case.fault(tick - ARM)
        {
            next_external[side] = force;
            let outward = if side == 0 { -force } else { force };
            queue_link_world_wrench(&mut sim, pad_ids[side], [outward, 0., 0.], [0.; 3])?;
        }
        writeln!(
            ticks,
            "{}",
            json!({
                "time_ns": t, "state": label, "slip_m": slip,
                "parcel_com_z_m": o.parcel_z, "pad_z_m": o.pad_z, "parcel_vz_m_s": o.parcel_vz,
                "jaw_force_n": jaw, "next_jaw_force_n": next_jaw, "external_opening_n": external,
                "normal_force_n": o.normal, "pad_vertical_force_n": o.vertical_total_n,
                "vertical_friction_n": o.friction, "lift_force_n": lift,
                "tactile_speed_m_s": speed, "tactile_travel_m": controller.tactile_travel_m(),
                "pressure_available": !blind, "requested_squeeze_n": controller.squeeze_n(),
                "native_steps": o.native_steps, "step_us": us
            })
        )?;
        if tick.is_multiple_of(5) {
            frames.push(visuals.frame(&sim, &sdf, "default", frames.len() as u64 + 1)?);
        }
        external = next_external;
        if jaw != next_jaw {
            rig.drive_jaws(&mut sim, next_jaw)?;
            jaw = next_jaw;
        }
        let next_lift = if tick >= 750 && state != RecoveryState::Lower {
            10.4956
        } else {
            0.
        };
        if next_lift != lift {
            rig.lift(&mut sim, next_lift)?;
            lift = next_lift;
        }
    }
    ticks.flush()?;
    gz_sim_cli::export_archive_at(
        &output.join("epoch-0"),
        visuals.files(),
        sim.epoch(),
        Duration::ZERO,
        frames,
    )?;
    times.sort_by(f64::total_cmp);
    let summary = json!({
        "case": case, "grid": grid, "parcel_mass_kg": link.inertial.mass,
        "public_dt_ns": DT.as_nanos() as u64, "native_steps": native_steps,
        "max_slip_m": max_slip, "terminal_state": format!("{state:?}"),
        "step_p95_us": times[(times.len()*95).div_ceil(100)-1],
        "state_transitions": transitions, "pressure_packets_missing": missed_packets,
        "armed_load_ticks": {
            "neither": armed_load_ticks[0], "left_only": armed_load_ticks[1],
            "right_only": armed_load_ticks[2], "bilateral": armed_load_ticks[3]
        }
    });
    fs::write(
        output.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    let _ = directory.keep();
    Ok(())
}
