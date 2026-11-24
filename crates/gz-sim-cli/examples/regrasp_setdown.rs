// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Regrasp failure, controlled descent and load-confirmed release onto a finite stand.
#[path = "closed_loop_carry/payload.rs"]
mod payload;
#[path = "regrasp_setdown/rig.rs"]
mod rig;
use bevy_tasks::{ComputeTaskPool, TaskPoolBuilder};
use clap::{Parser, ValueEnum};
use gz_sim_scene::{
    PhysicsEngine,
    grasp_guard::{
        ContactRecovery, PlacementPhase, PlacementReading, RecoveryPlacement, RecoveryState,
    },
    load_world_with_sdf_step,
};
pub(crate) use rig::Grid;
use rig::{DT, Rig};
use serde::Serialize;
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum, Serialize)]
#[serde(rename_all = "kebab-case")]
pub(crate) enum Case {
    Nominal,
    Brake,
    High,
    Low,
    Heavy,
    Mirror,
    Dropout,
    Wrong,
    Missing,
    Edge,
    /// The failed jaw continues applying outward force for the rest of the maneuver.
    Jammed,
}
impl Case {
    fn mass(self) -> f64 {
        if self == Self::Heavy { 0.26 } else { 0.2 }
    }
    fn deck_top(self) -> f64 {
        match self {
            Self::High => 0.286,
            Self::Low => 0.282,
            _ => 0.283,
        }
    }
    fn deck_x(self) -> f64 {
        if self == Self::Mirror { -0.003 } else { 0.003 }
    }
    fn deck_y(self) -> f64 {
        match self {
            Self::Missing | Self::Wrong => 0.15,
            Self::Edge => 0.0195,
            _ => 0.,
        }
    }
}
#[derive(Parser)]
struct Args {
    #[arg(value_enum)]
    case: Case,
    #[arg(value_enum)]
    grid: Grid,
    output: PathBuf,
    #[arg(long, default_value = "1")]
    threads: std::num::NonZeroUsize,
}
fn main() -> Result<()> {
    let args = Args::parse();
    ComputeTaskPool::get_or_init(|| {
        TaskPoolBuilder::new()
            .num_threads(args.threads.get())
            .build()
    });
    run(args.case, args.grid, &args.output).map(|_| ())
}
pub(crate) fn run(case: Case, grid: Grid, output: &Path) -> Result<Value> {
    let name = output.file_name().ok_or("expected new directory")?;
    let parent = output.parent().unwrap();
    fs::create_dir_all(parent)?;
    let directory = tempfile::Builder::new()
        .prefix(name)
        .rand_bytes(0)
        .tempdir_in(parent)?;
    let xml = rig::world(case)?;
    let file = "regrasp_setdown.sdf";
    let sdf = sdformat::load_str(&xml, file, &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        file,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    rig::configure(&mut sim, grid)?;
    let rig = Rig::new(&mut sim, case);
    let visuals = gz_sim_cli::ArchiveVisuals::new(&sim)?;
    let mut frames = vec![visuals.frame(&sim, &sdf, "default", 1)?];
    let mut log = BufWriter::new(fs::File::create(output.join("ticks.jsonl"))?);
    let mut recovery = ContactRecovery::default();
    let mut placement = RecoveryPlacement::new(
        case.mass(),
        0.56,
        0.30,
        if case == Case::Brake { 0.045 } else { 0.08 },
    )
    .unwrap();
    let mut handoff = None;
    let mut open = None;
    let mut clear = None;
    let mut arrival = None;
    let mut drop_start = None;
    let mut force = [4.2; 2];
    let mut lift = 0.;
    rig.drive(&mut sim, force, lift)?;
    let mass = case.mass();
    let mut step_cost = Vec::new();
    let mut final_info = Value::Null;
    let mut states = Vec::new();
    let mut last_state = String::new();
    for tick in 1_u32..=5500 {
        let start = Instant::now();
        sim.step_exact(1)?;
        step_cost.push(start.elapsed().as_secs_f64() * 1e6);
        let t = sim.elapsed().as_nanos() as u64;
        let obs = rig.read(&sim);
        let mut next = force;
        let mut next_lift = lift;
        let mut valid = true;
        let mut phase = if tick < 750 { "Preload" } else { "Lift" };
        if tick == 750 {
            next_lift = (0.56 + mass) * (9.81 + 4.);
        }
        if tick >= 1050 && handoff.is_none() {
            let state = recovery.observe(DT, Some(obs.jaw), rig.tactile(&sim));
            next = [recovery.squeeze_n(); 2];
            if tick < 1250 {
                let bad = usize::from(case == Case::Mirror);
                next[bad] = -6.;
            }
            if !matches!(state, RecoveryState::Held) {
                phase = "Regrasp";
            }
            if state == RecoveryState::Lower {
                handoff = Some(tick);
            }
        }
        let margin = (0.010 - (obs.parcel_com[0] - case.deck_x()).abs())
            .min(0.020 - (obs.parcel_com[1] - case.deck_y()).abs());
        if handoff.is_some() {
            if obs.receiver > 0.1 * mass * 9.81 {
                arrival.get_or_insert(tick);
            }
            if case == Case::Dropout && obs.receiver > 0.8 * mass * 9.81 {
                drop_start.get_or_insert(tick);
            }
            if let Some(s) = drop_start {
                valid = !(s + 30..s + 65).contains(&tick);
            }
            let drive = placement.advance(
                DT,
                PlacementReading {
                    pad_height_m: obs.pad_z,
                    pad_velocity_m_s: obs.pad_v,
                    parcel_velocity_m_s: obs.parcel_velocity[2],
                    receiver_impulse_ns: valid.then_some(obs.receiver * DT.as_secs_f64()),
                    obstacle_impulse_ns: obs.obstacle * DT.as_secs_f64(),
                    deck_margin_m: margin,
                    jaw_clearance_m: obs.clearance,
                    jaw_pressure_n: obs.jaw,
                },
            );
            next = [drive.jaw_force_n; 2];
            next_lift = drive.lift_force_n;
            phase = match drive.phase {
                PlacementPhase::Lower => "Lower",
                PlacementPhase::Hold => "Hold",
                PlacementPhase::Open => {
                    open.get_or_insert(tick);
                    "Open"
                }
                PlacementPhase::Retreat => {
                    clear.get_or_insert(tick);
                    "Retreat"
                }
                PlacementPhase::Placed => "Placed",
                PlacementPhase::SupportLost => "SupportLost",
                PlacementPhase::UnguidedLanding => "UnguidedLanding",
            };
        }
        if case == Case::Jammed && tick >= 1050 {
            next[0] = -6.;
        }
        if phase != last_state {
            states.push(json!({"state":phase,"time_ns":t}));
            last_state = phase.into();
        }
        let row = json!({"time_ns":t,"state":phase,"pad_z_m":obs.pad_z,"pad_vz_m_s":obs.pad_v,"parcel_com_z_m":obs.parcel_com[2],"parcel_com_m":obs.parcel_com,"parcel_vz_m_s":obs.parcel_velocity[2],"jaw_clearance_m":obs.clearance,"jaw_gap_m":obs.pad_gap,"normal_force_n":obs.jaw,"pad_vertical_force_n":obs.pad_vertical,"receiver_vertical_force_n":obs.receiver_vertical,"other_vertical_force_n":obs.other_vertical,"receiver_force_n":obs.receiver,"obstacle_force_n":obs.obstacle,"support_valid":valid,"deck_margin_m":margin,"jaw_force_n":force,"lift_force_n":lift,"native_steps":obs.native});
        writeln!(log, "{row}")?;
        if tick % 10 == 0 {
            frames.push(visuals.frame(&sim, &sdf, "default", frames.len() as u64 + 1)?);
        }
        if next != force || next_lift != lift {
            rig.drive(&mut sim, next, next_lift)?;
            force = next;
            lift = next_lift;
        }
        final_info = row;
    }
    log.flush()?;
    gz_sim_cli::export_archive_at(
        &output.join("epoch-0"),
        visuals.files(),
        sim.epoch(),
        std::time::Duration::ZERO,
        frames,
    )?;
    step_cost.sort_by(f64::total_cmp);
    let summary = json!({"case":case,"grid":grid,"dt_ns":200000,"parcel_mass_kg":mass,"deck_top_m":case.deck_top(),"handoff_tick":handoff,"arrival_tick":arrival,"opening_tick":open,"clear_tick":clear,"transitions":states,"final":final_info,"step_p95_us":step_cost[(95*step_cost.len()).div_ceil(100)-1]});
    fs::write(
        output.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    let _ = directory.keep();
    Ok(summary)
}
