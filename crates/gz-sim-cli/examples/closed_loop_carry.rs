// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Catch a slipping off-center parcel on a rising gripper with sampled feedback.
#[path = "closed_loop_carry/rig.rs"]
mod rig;

use bevy_tasks::{ComputeTaskPool, TaskPoolBuilder};
use clap::{Parser, ValueEnum};
use gz_sim_scene::grasp_guard::{SlipCatch, SlipCatchConfig, SlipCatchState, SlipLiftInterlock};
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use rig::{DT, Rig};
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
const STRONG: f64 = 4.2;
const WEAK: f64 = 2.3;
const LIFT: f64 = 10.4956;
const ARM_TICK: u32 = 1050;

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum, Serialize)]
#[serde(rename_all = "kebab-case")]
pub(crate) enum Case {
    /// Leave the weakened grip open loop.
    Open,
    /// Keep the original strong squeeze while monitoring for slip.
    Hold,
    /// Recover with the nominal sampled feedback.
    Catch,
    /// Catch the parcel using only the relative velocities measured at the two pads.
    Tactile,
    /// Read the sensor every 5 ms while driving the fingers every 200 µs.
    Delayed,
    /// Limit each finger to 30 N/s of additional squeeze.
    Slow,
    /// Cap each finger at 2.55 N while maintaining lift effort.
    Limited,
    /// Cap squeeze and reduce upward effort after sustained slip at the cap.
    Unload,
    /// Repeat the unloading trial with a 5 ms sensor and a 200 µs drive.
    DelayedUnload,
    /// Increase the mass and inertia of the parcel by 30%.
    Heavy,
    /// Lower the friction coefficient from 0.50 to 0.40.
    Slick,
    /// Reverse the parcel’s center-of-mass offset.
    Mirror,
    /// Increase lift effort after the first catch to provoke a second slip.
    Repeat,
    /// Open both jaws intentionally after catching the parcel.
    Release,
}
impl Case {
    fn config(self) -> SlipCatchConfig {
        let mut config = SlipCatchConfig::default();
        match self {
            Self::Limited | Self::Unload | Self::DelayedUnload => config.max_force_n = 2.55,
            Self::Repeat => config.max_force_n = 6.5,
            Self::Slow => config.force_rate_n_s = 30.,
            _ => {}
        }
        config
    }
    fn sample_period(self) -> u32 {
        if matches!(self, Self::Delayed | Self::DelayedUnload) {
            25
        } else {
            1
        }
    }
    fn ticks(self) -> u32 {
        if self == Self::Repeat { 1850 } else { 1450 }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum, Serialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum Grid {
    Adaptive,
    Fine,
}

#[derive(Parser)]
#[command(about)]
struct Args {
    #[arg(value_enum)]
    case: Case,
    #[arg(value_enum)]
    grid: Grid,
    /// New directory for the gz-web-rs replay, applied commands, measurements and summary.
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
    let parent = output.parent().expect("a filename has a parent");
    fs::create_dir_all(parent)?;
    // Claim this exact name exclusively. An I/O error drops the partial recording
    // and leaves the name available for a retry.
    let directory = tempfile::Builder::new()
        .prefix(name)
        .rand_bytes(0)
        .tempdir_in(parent)?;
    let xml = rig::world(case)?;
    let name = "local_offset_gripper_carry.sdf";
    let sdf = sdformat::load_str(&xml, name, &sdformat::NoResources)?;
    let parcel = sdf.worlds[0]
        .models
        .iter()
        .find(|m| m.name == "parcel")
        .unwrap();
    let link = &parcel.links[0];
    let center = sdf
        .resolve_pose(
            &link.inertial.pose,
            &parcel.scoped_name,
            &link.scoped_name,
            &link.scoped_name,
        )?
        .translation;
    let mut sim = load_world_with_sdf_step(
        &xml,
        name,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    assert_eq!(sim.step_duration(), DT);
    rig::configure(&mut sim, grid)?;
    let rig = Rig::new(&mut sim, center);
    let visuals = gz_sim_cli::ArchiveVisuals::new(&sim)?;
    let mut frames = vec![visuals.frame(&sim, &sdf, "default", 1)?];
    let mut ticks = BufWriter::new(fs::File::create(output.join("ticks.jsonl"))?);
    let mut costs = Vec::new();
    let mut transitions = Vec::new();
    let mut baseline = None;
    let mut guard: Option<SlipCatch> = None;
    let mut interlock = SlipLiftInterlock::new(Duration::from_millis(3));
    let mut prev_state = String::new();
    let mut force = STRONG;
    let mut lift = 0.;
    rig.grip(&mut sim, force)?;
    let mut max_slip: f64 = 0.;
    let mut final_slip = 0.;
    let mut native_steps: u64 = 0;
    // The tactile controller starts a fresh displacement integral at arming.
    // Ground-truth parcel COM above the pads is recorded independently.
    let mut tactile_height = 0.;
    let config = case.config();
    for tick in 1..=case.ticks() {
        let start = Instant::now();
        sim.step_exact(1)?;
        let step_us = start.elapsed().as_secs_f64() * 1e6;
        costs.push(step_us);
        let time_ns = sim.elapsed().as_nanos() as u64;
        let obs = rig.read(&sim);
        native_steps += u64::from(obs.native_steps);
        let relative_z = obs.parcel_z - obs.pad_z;
        let tactile_speed = if case == Case::Tactile && tick >= ARM_TICK {
            rig.tactile_speed(&sim)
        } else {
            None
        };
        let sensor_height = if case == Case::Tactile {
            match tactile_speed {
                Some(speed) => {
                    if tick > ARM_TICK {
                        tactile_height += speed * DT.as_secs_f64();
                    }
                    tactile_height
                }
                None => f64::NAN, // Feed missing contact into the controller's latched sensor fault.
            }
        } else {
            relative_z
        };
        let mut next_force = force;
        let mut next_lift = lift;
        let mut sampled = false;
        if tick == 750 {
            next_lift = LIFT;
        }
        if tick == ARM_TICK {
            baseline = Some(relative_z);
            if case != Case::Hold {
                next_force = WEAK;
            }
            if case != Case::Open {
                guard = Some(
                    SlipCatch::arm(config, next_force, sensor_height)
                        .ok_or("cannot arm without a valid grasp and catch configuration")?,
                );
            }
        }
        if tick > ARM_TICK
            && let Some(controller) = guard.as_mut()
        {
            if case == Case::Release && tick == 1325 {
                controller.release();
            } else if controller.state() != SlipCatchState::Released
                && (tick - ARM_TICK).is_multiple_of(case.sample_period())
            {
                controller.observe(DT * case.sample_period(), sensor_height);
                sampled = true;
            }
            // A released controller exerts zero squeeze. The negative joint
            // drive here is the task's explicit request to open both fingers.
            next_force = if controller.state() == SlipCatchState::Released {
                -WEAK
            } else {
                // A slow sensor can request several ticks of extra pressure at once.
                // Each actuator tick still obeys the physical drive slew.
                controller
                    .force_n()
                    .min(force + config.force_rate_n_s * DT.as_secs_f64())
            };
            if matches!(case, Case::Unload | Case::DelayedUnload) {
                next_lift = interlock.command(
                    controller.state(),
                    force >= config.max_force_n,
                    DT,
                    LIFT,
                    (0.56 + link.inertial.mass) * 9.81,
                );
            }
        }
        if case == Case::Repeat && tick == 1350 {
            next_lift = 18.5;
        }
        let state = guard.as_ref().map_or_else(
            || {
                if baseline.is_some() {
                    "OpenLoop"
                } else {
                    "Preload"
                }
                .to_string()
            },
            |controller| format!("{:?}", controller.state()),
        );
        if state != prev_state {
            transitions.push(json!({"time_ns":time_ns,"state":state}));
            prev_state.clone_from(&state);
        }
        final_slip = baseline.map_or(0., |h| h - relative_z);
        max_slip = max_slip.max(final_slip);
        let mut sample = json!({
                "time_ns":time_ns,"slip_m":final_slip,"parcel_com_z_m":obs.parcel_z,"pad_z_m":obs.pad_z,
                "force_n":force,"next_force_n":next_force,"requested_squeeze_n":guard.as_ref().map(SlipCatch::force_n),
                "lift_force_n":lift,"next_lift_force_n":next_lift,
                "sampled":sampled,"state":state,"normal_force_n":obs.normal,"vertical_friction_n":obs.friction,
                "native_steps":obs.native_steps,"step_us":step_us
        });
        if case == Case::Tactile {
            sample["tactile_speed_m_s"] = json!(tactile_speed);
            sample["tactile_height_m"] = json!(tactile_speed.map(|_| tactile_height));
        }
        writeln!(ticks, "{sample}")?;
        if tick.is_multiple_of(5) {
            frames.push(visuals.frame(&sim, &sdf, "default", frames.len() as u64 + 1)?);
        }
        if next_force != force {
            rig.grip(&mut sim, next_force)?;
            force = next_force;
        }
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
    costs.sort_by(f64::total_cmp);
    let profile = json!({
        "case":case,"grid":grid,"dt_ns":DT.as_nanos() as u64,"sensor_period_ns":(DT*case.sample_period()).as_nanos() as u64,
        "max_force_n":config.max_force_n,"force_rate_n_s":config.force_rate_n_s,"trigger_drop_m":config.trigger_drop_m,
        "parcel_mass_kg":link.inertial.mass,"parcel_com_offset_m":center,
        "friction_coefficient":if case==Case::Slick {0.4} else {0.5},
        "max_slip_m":max_slip,"final_slip_m":final_slip,"public_ticks":case.ticks(),"native_steps":native_steps,
        "step_p95_us":costs[(costs.len()*95).div_ceil(100)-1],"state_transitions":transitions
    });
    fs::write(
        output.join("summary.json"),
        serde_json::to_vec_pretty(&profile)?,
    )?;
    let _ = directory.keep();
    Ok(())
}
