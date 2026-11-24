// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure layer contact while the articulated jaw motors respond to the cloth.
use super::*;
use crate::{arm_sheet, fold_hold, fold_mesh};
use glam::DQuat;
use gz_sim_scene::deformable::set_material_resolution;
use serde::Serialize;

pub(super) const APPROACH: usize = 1650;
const DT: f64 = 0.001;
const PLATEAU: usize = 100;
const WINDOW: usize = 20;
const FORCE: f64 = 0.0001;
pub(super) const START: f64 = arm_sheet::JAW_STROKE - 0.00015;
const INCREMENT: f64 = 0.000025;
const FINE_INCREMENT: f64 = 0.0000025;
const FINE_GAP: f64 = 0.6e-6;
const MAX_STAGES: usize = 100;
const REFERENCE: (u32, u32) = (64, 48);
const CLOCK: (u32, u32) = (32, 24);
const STRIDE: usize = 25;

pub(super) struct Jaws {
    pub(super) ids: [u64; 2],
    pub(super) half: [DVec3; 2],
}
impl Jaws {
    pub(super) fn new(r: &Recording) -> Result<Self> {
        Ok(Self {
            ids: [
                entity_id(&r.sim, "default::arm::left")?,
                entity_id(&r.sim, "default::arm::right")?,
            ],
            half: [
                arm_cable::half_size(&r.sdf, "arm", "left")?,
                arm_cable::half_size(&r.sdf, "arm", "right")?,
            ],
        })
    }

    fn aperture(&self, sim: &Simulation) -> Result<f64> {
        let [a, b] = [state(sim, self.ids[0])?, state(sim, self.ids[1])?];
        let [left, right] = [a, b].map(|jaw| DQuat::from_array(jaw.orientation) * DVec3::X);
        let axis = (left + right).try_normalize().ok_or("opposed jaw axes")?;
        Ok((DVec3::from(b.position)
            - DVec3::from(a.position)
            - left * self.half[0].x
            - right * self.half[1].x)
            .dot(axis))
    }
}

#[derive(Serialize)]
pub(super) struct Sample {
    time_ns: u128,
    stage: usize,
    command_stroke_m: f64,
    pub(super) aperture_m: f64,
    pub(super) cross_panel_n: f64,
    pub(super) panel_gap_m: f64,
    pub(super) jaw_up_n: [f64; 2],
    pub(super) jaw_squeeze_n: [f64; 2],
    pub(super) bench_up_n: f64,
    pub(super) min_bench_clearance_m: f64,
    step_ns: u64,
}
pub(super) fn sample(
    r: &Recording,
    jaws: &Jaws,
    columns: usize,
    stage: usize,
    stroke: f64,
) -> Result<Sample> {
    let sheet = arm_sheet::cloth(&r.sim)?;
    let [left, right, bench, other_benches @ ..] = sheet.last_boundary_impulses() else {
        return Err("closure needs two jaws and the bench".into());
    };
    let mut squeeze = [0.; 2];
    for load in sheet.last_pad_loads() {
        squeeze[load.boundary] += DVec3::from(load.normal).length() / DT;
    }
    let mut cross = 0.;
    for load in sheet.last_self_contact_loads() {
        let (a, b) = load.particles.split_at(load.first_side_len);
        let (wa, wb) = load.particle_normal_ns.split_at(load.first_side_len);
        if matches!(
            (
                fold_hold::side_panel(a, wa, columns),
                fold_hold::side_panel(b, wb, columns)
            ),
            (Some(0), Some(1)) | (Some(1), Some(0))
        ) {
            cross += load.normal_ns;
        }
    }
    Ok(Sample {
        time_ns: r.sim.elapsed().as_nanos(),
        stage,
        command_stroke_m: stroke,
        aperture_m: jaws.aperture(&r.sim)?,
        cross_panel_n: cross / DT,
        panel_gap_m: fold_mesh::panel_gap(sheet.particles(), columns)?,
        jaw_up_n: [-left.linear[2] / DT, -right.linear[2] / DT],
        jaw_squeeze_n: squeeze,
        bench_up_n: -(bench.linear[2] + other_benches.iter().map(|j| j.linear[2]).sum::<f64>())
            / DT,
        min_bench_clearance_m: sheet
            .particles()
            .iter()
            .map(|p| p.position[2] - arm_sheet::RADIUS - arm_sheet::BENCH)
            .fold(f64::INFINITY, f64::min),
        step_ns: *r.costs_ns.last().unwrap(),
    })
}

#[derive(Serialize)]
struct Plateau {
    stage: usize,
    command_stroke_m: f64,
    mean_aperture_m: f64,
    min_aperture_m: f64,
    max_aperture_m: f64,
    mean_cross_panel_n: f64,
    min_cross_panel_n: f64,
    max_cross_panel_n: f64,
    mean_panel_gap_m: f64,
    mean_jaw_up_n: [f64; 2],
    mean_jaw_squeeze_n: [f64; 2],
    mean_bench_up_n: f64,
    min_bench_clearance_m: f64,
    step_p95_ns: u64,
    sustained: bool,
}
fn plateau(samples: &[Sample]) -> Plateau {
    let tail = &samples[samples.len() - WINDOW..];
    let mean = |f: fn(&Sample) -> f64| tail.iter().map(f).sum::<f64>() / WINDOW as f64;
    let min = |f: fn(&Sample) -> f64| tail.iter().map(f).fold(f64::INFINITY, f64::min);
    let max = |f: fn(&Sample) -> f64| tail.iter().map(f).fold(f64::NEG_INFINITY, f64::max);
    let mut costs: Vec<_> = samples.iter().map(|s| s.step_ns).collect();
    costs.sort_unstable();
    Plateau {
        stage: tail[0].stage,
        command_stroke_m: tail[0].command_stroke_m,
        mean_aperture_m: mean(|s| s.aperture_m),
        min_aperture_m: min(|s| s.aperture_m),
        max_aperture_m: max(|s| s.aperture_m),
        mean_cross_panel_n: mean(|s| s.cross_panel_n),
        min_cross_panel_n: min(|s| s.cross_panel_n),
        max_cross_panel_n: max(|s| s.cross_panel_n),
        mean_panel_gap_m: mean(|s| s.panel_gap_m),
        mean_jaw_up_n: [mean(|s| s.jaw_up_n[0]), mean(|s| s.jaw_up_n[1])],
        mean_jaw_squeeze_n: [mean(|s| s.jaw_squeeze_n[0]), mean(|s| s.jaw_squeeze_n[1])],
        mean_bench_up_n: mean(|s| s.bench_up_n),
        min_bench_clearance_m: min(|s| s.min_bench_clearance_m),
        step_p95_ns: costs[(costs.len() * 95).div_ceil(100) - 1],
        sustained: tail.iter().all(|s| s.cross_panel_n >= FORCE),
    }
}

#[derive(Serialize)]
struct Bracket {
    open_stage: usize,
    loaded_stage: usize,
    open_aperture_m: f64,
    loaded_aperture_m: f64,
}

fn bracket(stages: &[Plateau]) -> Option<Bracket> {
    let i = stages
        .windows(2)
        .position(|pair| pair.iter().all(|p| p.sustained))?;
    let open = stages.get(i.checked_sub(1)?)?;
    let loaded = &stages[i];
    if loaded.command_stroke_m == open.command_stroke_m {
        return None;
    }
    Some(Bracket {
        open_stage: open.stage,
        loaded_stage: loaded.stage,
        open_aperture_m: open.mean_aperture_m,
        loaded_aperture_m: loaded.mean_aperture_m,
    })
}
struct Outcome {
    report: Value,
    anchor: Value,
    preceding: Option<Value>,
    lower: Option<usize>,
    loaded_at_fixed_target: bool,
}

fn trial(
    columns: usize,
    out: &Path,
    clock: (u32, u32),
    hold_after: Option<usize>,
) -> Result<Outcome> {
    let scene = closing_sheet_scene::scene(columns)?;
    let mut r = Recording::new(&scene, "fold_closure.sdf", "default")?;
    let ctrl = arm::Controller::new(&r.sim)?;
    let jaws = Jaws::new(&r)?;
    let mut frames = vec![];
    arm_sheet::capture(&mut r, &mut frames)?;
    for tick in 1..=APPROACH {
        arm_cable::lift_with_jaws(&ctrl, &mut r.sim, (tick - 1) as f64 * DT, START)?;
        r.advance()
            .map_err(|e| format!("{columns}-column approach tick {tick}: {e}"))?;
        if tick % STRIDE == 0 {
            arm_sheet::capture(&mut r, &mut frames)?;
        }
    }
    let anchor = fold_hold::snapshot(&r.sim)?;
    set_material_resolution(&mut r.sim, "sheet", clock.0, clock.1)?;
    let weight: f64 = arm_sheet::cloth(&r.sim)?
        .particles()
        .iter()
        .map(|p| 9.81 / p.inverse_mass)
        .sum();
    let mut stages = vec![];
    let mut trace = vec![];
    let mut stroke = START;
    let mut fine = false;
    let mut preceding = None;
    for stage in 0..MAX_STAGES {
        if stage > 1 && hold_after.is_none_or(|i| stage <= i) {
            stroke += if fine { FINE_INCREMENT } else { INCREMENT };
        }
        for tick in 1..=PLATEAU {
            // Holding time fixed keeps the arm aloft. Only the jaw targets change.
            arm_cable::lift_with_jaws(&ctrl, &mut r.sim, APPROACH as f64 * DT, stroke)?;
            r.advance()
                .map_err(|e| format!("{columns}-column stage {stage} tick {tick}: {e}"))?;
            trace.push(sample(&r, &jaws, columns, stage, stroke)?);
            if tick % STRIDE == 0 {
                arm_sheet::capture(&mut r, &mut frames)?;
            }
        }
        let p = plateau(&trace[trace.len() - PLATEAU..]);
        fine |= p.mean_panel_gap_m <= FINE_GAP;
        eprintln!(
            "mesh {columns} stage {stage}: aperture {:.9} m, cross {:.6} N, gap {:.4} um, upward {:?} N",
            p.mean_aperture_m,
            p.mean_cross_panel_n,
            p.mean_panel_gap_m * 1e6,
            p.mean_jaw_up_n
        );
        check_support(&trace[trace.len() - WINDOW..], weight, columns)?;
        if hold_after.map_or(!p.sustained, |i| i == stage) {
            preceding = Some(fold_hold::snapshot(&r.sim)?);
        }
        stages.push(p);
        let confirmed = stages.len() >= 2 && stages[stages.len() - 2..].iter().all(|s| s.sustained);
        if hold_after.map_or(confirmed, |i| stage == i + 2) {
            break;
        }
    }
    let threshold = bracket(&stages);
    let last = r.frames.len() - 1;
    let captures = json!({"approach":0,"held":APPROACH/STRIDE,"closure":last});
    let suffix = if hold_after.is_some() {
        "-fixed"
    } else if clock == REFERENCE {
        "-fine"
    } else {
        ""
    };
    let dir = out.join(format!("mesh-{columns}{suffix}"));
    r.save(&dir)?;
    fs::write(dir.join("world.sdf"), scene)?;
    write_json(
        &dir.join("particle-frames.json"),
        &serde_json::to_value(frames)?,
    )?;
    write_json(
        &dir.join("frames.json"),
        &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"source_public_dt_ns":1_000_000,"capture_stride_ticks":STRIDE,"captures":captures,"particle_visuals":(0..4*columns).map(arm_sheet::name).collect::<Vec<_>>(),"jaw_visuals":["default::arm::left::left","default::arm::right::right"],"cloth_grid":{"rows":4,"columns":columns,"skin_radius_m":arm_sheet::RADIUS,"last_row_left":1,"first_row_right":2}}),
    )?;
    write_json(&dir.join("trace.json"), &serde_json::to_value(trace)?)?;
    let status = onset_status(&stages, threshold.is_some(), hold_after.is_some());
    let loaded_at_fixed_target =
        hold_after.is_some_and(|i| stages.iter().skip(i + 1).any(|p| p.sustained));
    let report = json!({"columns":columns,"material_substeps":clock.0,"solver_iterations":clock.1,"weight_n":weight,"stages":stages,"bracket":threshold,
        "fixed_from_stage":hold_after,"status":status });
    Ok(Outcome {
        report,
        anchor,
        preceding,
        lower: threshold.map(|p| p.open_stage),
        loaded_at_fixed_target,
    })
}

fn supported(s: &Sample, weight: f64) -> bool {
    s.min_bench_clearance_m > 0.01
        && s.bench_up_n.abs() < 1e-6
        && s.jaw_up_n.iter().all(|v| *v > 0.)
        && (s.jaw_up_n.iter().sum::<f64>() / weight - 1.).abs() < 0.025
}

fn check_support(window: &[Sample], weight: f64, columns: usize) -> Result<()> {
    if let Some(s) = window.iter().find(|s| !supported(s, weight)) {
        return Err(format!("mesh {columns} lost bilateral support at plateau {}, time {} ns (jaws {:?} N, bench {} N, clearance {} m)",
            s.stage,s.time_ns,s.jaw_up_n,s.bench_up_n,s.min_bench_clearance_m).into());
    }
    Ok(())
}

fn onset_status(stages: &[Plateau], bracketed: bool, fixed: bool) -> &'static str {
    if fixed {
        "fixed_target"
    } else if bracketed {
        "bracketed"
    } else if matches!(stages, [a,b,..] if a.sustained && b.sustained) {
        "already_loaded"
    } else {
        "unbracketed"
    }
}

type Simulate = fn(usize, &Path, (u32, u32), Option<usize>) -> Result<Outcome>;

fn primary_and_control(
    columns: usize,
    out: &Path,
    simulate: Simulate,
) -> Result<(Outcome, Option<Value>)> {
    let mut primary = simulate(columns, out, CLOCK, None)?;
    let fixed = if let Some(i) = primary.lower {
        let held = simulate(columns, out, CLOCK, Some(i))?;
        if primary.preceding.is_none() || primary.preceding != held.preceding {
            return Err("fixed-target control did not reproduce the preceding live state".into());
        }
        if held.loaded_at_fixed_target {
            primary.report["status"] = "also_loaded_at_fixed_target".into();
        }
        Some(held.report)
    } else {
        None
    };
    Ok((primary, fixed))
}

pub(super) fn run(out: &Path) -> Result<()> {
    run_comparison(out, trial)
}

fn completed<T>(
    worker: std::thread::ScopedJoinHandle<'_, std::result::Result<T, String>>,
) -> std::result::Result<T, String> {
    worker.join().map_err(|_| "closure trial panicked")?
}

fn run_comparison(out: &Path, simulate: Simulate) -> Result<()> {
    let jobs = std::thread::scope(|scope| {
        let jobs = [5, 7].map(|columns| {
            (
                scope.spawn(move || {
                    primary_and_control(columns, out, simulate).map_err(|e| e.to_string())
                }),
                scope.spawn(move || {
                    simulate(columns, out, REFERENCE, None).map_err(|e| e.to_string())
                }),
            )
        });
        // Join every worker even if more than one clock refuses or panics.
        jobs.map(|(primary, reference)| (completed(primary), completed(reference)))
    });
    let mut trials = vec![];
    let mut references = vec![];
    let mut controls = vec![];
    for (primary, reference) in jobs {
        let (primary, control) = primary?;
        let reference = reference?;
        if primary.anchor != reference.anchor {
            return Err("clock comparison started with different robot or material states".into());
        }
        trials.push(primary.report);
        references.push(reference.report);
        controls.extend(control);
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"trials":trials,"reference_trials":references,"fixed_target_controls":controls,
        "matched_native_and_material_prefix":true,"dt_ns":1_000_000,"plateau_ticks":PLATEAU,
        "window_ticks":WINDOW,"force_floor_n":FORCE,"confirming_plateaus":2,"increment_per_jaw_m":INCREMENT,"fine_increment_per_jaw_m":FINE_INCREMENT,
        "refine_below_gap_m":FINE_GAP,"initial_command_stroke_m":START,"approach_ticks":APPROACH,"hold_substeps":CLOCK.0,"iterations":CLOCK.1,
        "reference_substeps":REFERENCE.0,"reference_iterations":REFERENCE.1}),
    )
}

#[cfg(test)]
#[path = "fold_closure_tests.rs"]
mod tests;
