// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{capture::Capture, native_tool::NativeTool, reference::turn, server};
use serde::Deserialize;
use serde_json::json;
use std::{error::Error, fs, num::NonZeroUsize, path::Path, time::Instant};

#[derive(Deserialize)]
struct Command {
    tick_ms: usize,
    command_rad: [f64; 2],
}

struct Trial {
    steps: usize,
    passes: usize,
    native: NativeTool,
    first: Option<(usize, f64)>,
    total: f64,
    last: usize,
    costs: Vec<u64>,
}

pub fn run(trace: &Path, out: &Path) -> Result<(), Box<dyn Error>> {
    if out.exists() {
        return Err("landing replay output already exists".into());
    }
    let commands: Vec<Command> = serde_json::from_slice(&fs::read(trace)?)?;
    if commands.len() != 600
        || commands
            .iter()
            .enumerate()
            .any(|(i, c)| c.tick_ms != i + 1 || !c.command_rad.iter().all(|v| v.is_finite()))
    {
        return Err("expected 600 consecutive finite command samples".into());
    }
    let mut fixture = server::reference_fixture();
    fixture.velocity_mode = [1.; 2];
    fixture.arm.damping = [6., 1.5];
    let limits = [Some([-0.36, 0.02]), Some([-0.15, 1.20])];
    let mut display = [fixture; 9];
    let mut runs = Vec::new();
    for steps in [8, 16, 32] {
        for passes in [1, 2, 4] {
            let mut native = NativeTool::new_with_limits(&fixture, limits);
            native.set_steps_per_ms(NonZeroUsize::new(steps).unwrap());
            native
                .arm
                .world
                .integration_parameters
                .num_internal_stabilization_iterations = passes;
            runs.push(Trial {
                steps,
                passes,
                native,
                first: None,
                total: 0.,
                last: 0,
                costs: Vec::with_capacity(commands.len()),
            });
        }
    }
    let mut capture = Capture::default();
    capture.sample(0, &display);
    for command in commands {
        for (trial, lane) in runs.iter_mut().zip(&mut display) {
            lane.velocity_target =
                server::drive_velocity(command.command_rad, trial.native.arm.angles());
            let start = Instant::now();
            let reaction = trial.native.step(lane);
            trial.costs.push(start.elapsed().as_nanos() as u64);
            lane.arm.angle = trial.native.arm.angles().map(turn);
            lane.arm.speed = trial.native.arm.rates();
            lane.normal_impulse = reaction.n;
            lane.tangent_impulse = reaction.t;
            lane.first_contact_offset = reaction.first.unwrap_or(-1.);
            let tip = lane.arm.points()[1];
            if !tip
                .into_iter()
                .chain(reaction.tip)
                .chain([reaction.n, reaction.t])
                .all(f64::is_finite)
                || (tip[0] - reaction.tip[0]).hypot(tip[1] - reaction.tip[1]) > 1e-10
            {
                return Err("invalid native tool coordinate or reaction".into());
            }
            if reaction.n > 0. {
                trial.first.get_or_insert((command.tick_ms, reaction.n));
                trial.last = command.tick_ms;
            }
            trial.total += reaction.n;
        }
        capture.sample(command.tick_ms as u64 * 1_000_000, &display);
    }
    let mut rows = Vec::new();
    for (lane, trial) in runs.into_iter().enumerate() {
        let Trial {
            steps,
            passes,
            first,
            total,
            last,
            mut costs,
            ..
        } = trial;
        let (tick, impulse) = first.ok_or("missing measured touchdown")?;
        if tick != 65 || !(301..320).contains(&last) {
            return Err("landing or release clock outside fixture bounds".into());
        }
        costs.sort_unstable();
        rows.push(json!({"lane_id":lane,"steps_per_ms":steps,"stabilization_passes":passes,"first_loaded_tick":tick,"first_tick_normal_ns":impulse,"total_normal_ns":total,"last_loaded_tick":last,"native_step_p95_ns":costs[(costs.len()*95).div_ceil(100)-1]}));
    }
    let parent = out
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::tempdir_in(parent)?;
    capture.write(stage.path())?;
    fs::write(
        stage.path().join("comparison.json"),
        serde_json::to_vec_pretty(&rows)?,
    )?;
    fs::rename(stage.path(), out)?;
    Ok(())
}
