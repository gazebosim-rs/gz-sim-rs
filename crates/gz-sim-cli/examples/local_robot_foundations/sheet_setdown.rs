// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run the existing gripper on a measured table-load gate, and export the real robot and cloth.
use super::*;
use crate::arm::Controller;
use gz_sim_scene::grasp_guard::{SetdownCommand, SupportedSetdown};
use serde::Serialize;

const WEIGHT: f64 = arm_sheet::COUNT as f64 * arm_sheet::NODE_MASS * 9.81;
const END: usize = 4300;

#[derive(clap::ValueEnum, Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub(crate) enum Mode {
    Supported,
    Timed,
    Missing,
    Finite,
}
#[derive(Debug, Serialize, PartialEq)]
struct Reading {
    tick: usize,
    material_state_sha256: [u8; 32],
    table_n: f64,
    jaws_up_n: f64,
    grip_n: [f64; 2],
    min_height_above_table_m: f64,
    lift_offset_m: f64,
    opening_ms: u16,
    search_exhausted: bool,
    pad_free_observations: usize,
    unsupported_pad_free_observations: usize,
    max_observed_table_impulse_ns: f64,
    deck_lateral_clearance_m: f64,
}
fn drive(
    c: &Controller,
    sim: &mut Simulation,
    t: f64,
    mode: Mode,
    cmd: SetdownCommand,
) -> Result<()> {
    if mode == Mode::Timed || (mode == Mode::Supported && t < 2.55) {
        return arm_cable::lift_with_jaws(c, sim, t, arm_sheet::JAW_STROKE);
    }
    let (approach, va) = gripper::ramp(t, 0., 0.25);
    let (close, vc) = gripper::ramp(t, 0.10, 0.55);
    let (raise, vu) = gripper::ramp(t, 0.85, 0.55);
    let (lower, vd) = gripper::ramp(t, 2., 0.55);
    let (open, vo) = gripper::ramp(f64::from(cmd.opening_ms) * 0.001, 0., 0.35);
    let jaw = (
        arm_sheet::JAW_STROKE * (close - open),
        arm_sheet::JAW_STROKE * (vc - if cmd.advancing_opening { vo } else { 0. }),
        0.,
    );
    c.actuate(
        sim,
        [
            (0., 0., 0.),
            (
                arm_cable::LIFT_TRAVEL * (raise - lower) - 0.010 * approach + cmd.lift_offset_m,
                arm_cable::LIFT_TRAVEL * (vu - vd) - 0.010 * va,
                0.96 * 9.81,
            ),
            jaw,
            jaw,
        ],
        true,
    )?;
    Ok(())
}

fn episode(
    r: &mut Recording,
    mode: Mode,
    frames: &mut Option<Vec<arm_sheet::Frame>>,
) -> Result<Vec<Reading>> {
    let native_per_command = usize::try_from(1_000_000 / r.sim.step_duration().as_nanos())?;
    let controller = Controller::with_jaw_damping(&r.sim, 1.5)?;
    let mut guard = SupportedSetdown::new(WEIGHT).unwrap();
    let bench = entity_id(&r.sim, "default::source::body")?;
    let deck_half_y = arm_cable::half_size(&r.sdf, "source", "body")?.y;
    let mut last_table_ns = 0.;
    let mut cmd = SetdownCommand::default();
    let mut rows = Vec::with_capacity(END);
    if let Some(f) = frames {
        arm_sheet::capture(r, f)?;
    }
    for tick in 1..=END {
        if mode == Mode::Missing && matches!(tick, 1501 | 1851) {
            let speed = if tick == 1501 { 0.75 } else { 0. };
            gz_sim_scene::queue_link_world_velocity(&mut r.sim, bench, [0., speed, 0.], [0.; 3])?;
        }
        if mode != Mode::Timed && tick > 2550 {
            cmd = guard
                .advance(last_table_ns)
                .ok_or("invalid table impulse")?;
        }
        drive(
            &controller,
            &mut r.sim,
            (tick - 1) as f64 * 0.001,
            mode,
            cmd,
        )?;
        let (mut js, mut grip) = ([0.; 3], [0.; 2]);
        let (mut pad_free, mut unsupported, mut peak) = (0, 0, 0_f64);
        for _ in 0..native_per_command {
            r.advance()?;
            let b = arm_sheet::cloth(&r.sim)?;
            let [l, r, t] = b.last_boundary_impulses() else {
                return Err("missing native jaw or table reading".into());
            };
            for (i, b) in [l, r, t].into_iter().enumerate() {
                js[i] -= b.linear[2];
            }
            for (i, b) in [l, r].into_iter().enumerate() {
                grip[i] += b.linear[0].hypot(b.linear[1]);
            }
            let no_pad = [l, r]
                .iter()
                .all(|j| DVec3::from(j.linear).length() < 1e-15);
            pad_free += usize::from(no_pad);
            unsupported += usize::from(no_pad && t.linear[2].abs() < 1e-15);
            peak = peak.max(-t.linear[2]);
        }
        let cloth = arm_sheet::cloth(&r.sim)?;
        use sha2::{Digest, Sha256};
        let mut material = Sha256::new();
        for particle in cloth.particles() {
            for value in particle.position.into_iter().chain(particle.velocity) {
                material.update(value.to_le_bytes());
            }
        }
        let low = |axis: usize| {
            cloth
                .particles()
                .iter()
                .map(|p| p.position[axis] - arm_sheet::RADIUS)
                .fold(f64::INFINITY, f64::min)
        };
        let far_y = cloth
            .particles()
            .iter()
            .map(|p| p.position[1] + arm_sheet::RADIUS)
            .fold(f64::NEG_INFINITY, f64::max);
        let deck_y = state(&r.sim, bench)?.position[1];
        last_table_ns = js[2];
        rows.push(Reading {
            tick,
            material_state_sha256: material.finalize().into(),
            table_n: js[2] / 0.001,
            jaws_up_n: (js[0] + js[1]) / 0.001,
            grip_n: grip.map(|v| v / 0.001),
            min_height_above_table_m: low(2) - arm_sheet::BENCH,
            lift_offset_m: cmd.lift_offset_m,
            opening_ms: if mode == Mode::Timed {
                // The command at the left endpoint of tick 2651 starts the 350 ms ramp.
                tick.saturating_sub(2651).min(350) as u16
            } else {
                cmd.opening_ms
            },
            search_exhausted: cmd.search_exhausted,
            pad_free_observations: pad_free,
            unsupported_pad_free_observations: unsupported,
            max_observed_table_impulse_ns: peak,
            deck_lateral_clearance_m: deck_y - deck_half_y - far_y,
        });
        if let Some(f) = frames
            && tick % 25 == 0
        {
            arm_sheet::capture(r, f)?;
        }
    }
    Ok(rows)
}

#[derive(Serialize)]
struct ObservationWindow {
    first_opening_tick: Option<usize>,
    fully_open_tick: Option<usize>,
    search_exhausted_tick: Option<usize>,
    preceding_20ms_table_force_range_n: Option<[f64; 2]>,
    first_pad_free_tick: Option<usize>,
    airborne_after_opening_s: f64,
    max_loose_gap_m: Option<f64>,
    peak_loose_table_interval_ns: Option<f64>,
    final_table_n: f64,
    final_jaw_up_n: f64,
    final_grip_n: f64,
    final_deck_lateral_clearance_m: f64,
    max_downward_adjustment_m: f64,
    min_raised_clearance_m: f64,
}
impl ObservationWindow {
    fn new(mode: Mode, rows: &[Reading], observed_s: f64) -> Result<Self> {
        let opening = rows.iter().find(|r| r.opening_ms > 0).map(|r| r.tick);
        let complete = rows.iter().find(|r| r.opening_ms == 350).map(|r| r.tick);
        let late = &rows[END - 200..];
        let mean = |f: fn(&Reading) -> f64| late.iter().map(f).sum::<f64>() / late.len() as f64;
        let preceding = opening.map(|tick| {
            let iter = rows[tick - 21..tick - 1].iter().map(|r| r.table_n);
            [
                iter.clone().fold(f64::INFINITY, f64::min),
                iter.fold(f64::NEG_INFINITY, f64::max),
            ]
        });
        let released = opening.map_or(&[][..], |t| &rows[t - 1..]);
        let first = released
            .iter()
            .find(|r| r.pad_free_observations > 0)
            .map(|r| r.tick);
        let free = first.map(|t| &rows[t - 1..]);
        let m = Self {
            first_opening_tick: opening,
            fully_open_tick: complete,
            search_exhausted_tick: rows.iter().find(|r| r.search_exhausted).map(|r| r.tick),
            preceding_20ms_table_force_range_n: preceding,
            first_pad_free_tick: first,
            airborne_after_opening_s: released
                .iter()
                .map(|r| r.unsupported_pad_free_observations)
                .sum::<usize>() as f64
                * observed_s,
            max_loose_gap_m: free.map(|r| {
                r.iter()
                    .map(|r| r.min_height_above_table_m)
                    .fold(0., f64::max)
            }),
            peak_loose_table_interval_ns: free.map(|r| {
                r.iter()
                    .map(|r| r.max_observed_table_impulse_ns)
                    .fold(0., f64::max)
            }),
            final_table_n: mean(|r| r.table_n),
            final_jaw_up_n: mean(|r| r.jaws_up_n),
            final_grip_n: mean(|r| r.grip_n.iter().sum()),
            final_deck_lateral_clearance_m: rows[END - 1].deck_lateral_clearance_m,
            max_downward_adjustment_m: -rows.iter().map(|r| r.lift_offset_m).fold(0., f64::min),
            min_raised_clearance_m: rows[1600..1900]
                .iter()
                .map(|r| r.min_height_above_table_m)
                .fold(f64::INFINITY, f64::min),
        };
        let supported = m.fully_open_tick.is_some()
            && m.first_pad_free_tick.is_some()
            && (m.final_table_n - WEIGHT).abs() < 0.01 * WEIGHT
            && m.final_grip_n < 0.001;
        let passed = match mode {
            Mode::Missing => {
                m.search_exhausted_tick.is_some()
                    && m.first_opening_tick.is_none()
                    && m.final_table_n.abs() < 1e-8
                    && m.final_grip_n > 0.5
                    && (m.final_jaw_up_n - WEIGHT).abs() < 0.03 * WEIGHT
                    && m.final_deck_lateral_clearance_m > 0.03
            }
            Mode::Timed => supported && m.airborne_after_opening_s > 0.005,
            Mode::Supported | Mode::Finite => {
                supported
                    && preceding.is_some_and(|[lo, hi]| lo >= 0.8 * WEIGHT && hi <= 1.5 * WEIGHT)
                    && m.airborne_after_opening_s == 0.
                    && m.max_loose_gap_m.is_some_and(|gap| gap < 1e-5)
            }
        };
        if !passed || m.min_raised_clearance_m < 0.10 || m.max_downward_adjustment_m > 0.004 {
            return Err(format!(
                "setdown failed its physical checks: {}",
                serde_json::to_string(&m)?
            )
            .into());
        }
        Ok(m)
    }
}

pub(super) fn run(
    out: &Path,
    mode: Mode,
    exchanges: usize,
    public: bool,
    verify_reset: bool,
) -> Result<()> {
    if exchanges == 0 || !1_000_000usize.is_multiple_of(exchanges) || public && exchanges > 256 {
        return Err(
            "exchange count must divide 1 ms exactly and fit the public exchange clock".into(),
        );
    }
    let deck = matches!(mode, Mode::Missing | Mode::Finite);
    if deck && public && exchanges > 1 {
        return Err(
            "prescribed finite bench requires --native-steps for multiple exchanges".into(),
        );
    }
    let mut xml = sheet_jaws::setdown_world(exchanges, deck, public)?;
    if deck {
        const STATIC: &str = r#"<model name="source">
      <static>true</static>"#;
        const DECK: &str = r#"<model name="source">
      <plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system">
        <linear_velocity>0 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity>
      </plugin>"#;
        xml = xml.replace(STATIC, DECK);
    }
    let mut r = Recording::new(&xml, "sheet_setdown.sdf", "default")?;
    let mut frames = Some(Vec::new());
    let rows = episode(&mut r, mode, &mut frames)?;
    if verify_reset {
        gz_sim_scene::reset_loaded_world(&mut r.sim, |_| ())?;
        let repeat = episode(&mut r, mode, &mut None)?;
        if rows != repeat {
            return Err("reset changed a measured controller tick".into());
        }
        r.costs_ns
            .truncate(END * usize::try_from(1_000_000 / r.sim.step_duration().as_nanos())?);
    }
    let window = ObservationWindow::new(mode, &rows, r.sim.step_duration().as_secs_f64())?;
    let receipt = json!({"mode":mode,"actuator_ms":1,"exchanges_per_ms":exchanges,"public_ms":r.sim.step_duration().as_secs_f64()*1000.,"reset_checked":verify_reset,"weight_n":WEIGHT,"measurement":window});
    fs::write(out.join("world.sdf"), xml)?;
    write_json(&out.join("ticks.json"), &serde_json::to_value(rows)?)?;
    write_json(&out.join("comparison.json"), &receipt)?;
    let (camera, captures) = if mode == Mode::Missing {
        (
            "perspective",
            json!({"grasp":40,"deck_moving":64,"deck_clear":76,"search":112,"retained":172}),
        )
    } else {
        let release = window
            .first_opening_tick
            .expect("qualified physical opening");
        (
            "right",
            json!({"grasp":40,"raised":72,"search":104,"opening":release.div_ceil(25),"settled":172}),
        )
    };
    arm_sheet::save_archive_with_camera(r, frames.unwrap(), out, captures, camera)?;
    println!("{receipt}");
    Ok(())
}

#[cfg(test)]
#[path = "sheet_setdown/tests.rs"]
mod tests;
