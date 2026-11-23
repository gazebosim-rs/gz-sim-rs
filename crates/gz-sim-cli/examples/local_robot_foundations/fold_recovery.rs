// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Test the live tactile recovery policy against identical unguarded pulls.
use super::*;
#[path = "fold_landing.rs"]
mod landing;
use gz_sim_scene::grasp_guard::{GraspRetention, PadReading, RetentionCommand};
pub(crate) use landing::End;
use std::time::Duration;

const HORIZON: usize = 1200;
// Limit the commanded squeeze to the measured near-closure stroke range.
const EXTRA_STROKE: f64 = 0.00015;
const TIGHTEN_TICKS: usize = 30;
const RELEASE_TICKS: usize = 300;

#[derive(Default)]
struct Recovery {
    guard: GraspRetention,
    tighten: Option<usize>,
    lower: Option<usize>,
    open: Option<usize>,
    lost_aloft: Option<usize>,
    initial_stroke: f64,
    escaped: Option<usize>,
    clear_ticks: usize,
}

impl Recovery {
    fn observe(
        &mut self,
        tick: usize,
        prev: &Tick,
        now: &Tick,
        stroke: f64,
        weight: f64,
        enabled: bool,
    ) {
        let dt = Duration::from_millis(1);
        let pads = std::array::from_fn(|i| PadReading {
            normal_force_n: now.jaw_squeeze_n[i],
            slip_speed_m_s: (now.pad_frame_y_m[i] - prev.pad_frame_y_m[i]) / DT,
        });
        if self.lost_aloft.is_none()
            && now.min_bench_clearance_m > 0.01
            && pads.iter().all(|p| p.normal_force_n < 0.05)
        {
            self.lost_aloft = Some(tick);
        }
        if !enabled {
            return;
        }
        match self.guard.observe(dt, pads, now.bench_up_n * DT, weight) {
            RetentionCommand::Hold => {}
            RetentionCommand::Tighten if self.tighten.is_none() => {
                self.tighten = Some(tick);
                self.initial_stroke = stroke;
            }
            RetentionCommand::Lower => {
                self.lower.get_or_insert(tick);
            }
            RetentionCommand::Open => {
                self.open.get_or_insert(tick);
            }
            _ => {}
        }
    }

    fn confirm_escape(&mut self, tick: usize, sample: &Tick, clearances: [f64; 2]) {
        if sample.min_bench_clearance_m > 0.01
            && sample.jaw_squeeze_n.iter().all(|n| *n < 1e-9)
            && clearances.iter().all(|clearance| *clearance > 0.)
        {
            self.clear_ticks += 1;
            if self.clear_ticks >= 4 {
                self.escaped.get_or_insert(tick);
            }
        } else {
            self.clear_ticks = 0;
        }
    }

    // Commands only depend on the previous committed sensors and action clock.
    fn drive(&self, tick: usize, grasp: &mut Grasp, phase: &'static str, force: f64) -> Result<()> {
        if let Some(start) = self.tighten {
            let ramp = ((tick - start) as f64 / TIGHTEN_TICKS as f64).min(1.);
            let release = self
                .open
                .map_or(0., |k| ((tick - k) as f64 / RELEASE_TICKS as f64).min(1.));
            grasp.stroke = (self.initial_stroke + EXTRA_STROKE * ramp).min(0.080) * (1. - release);
        }
        let lift_time = self.lower.map_or(APPROACH as f64 * DT, |k| {
            (2.0 + (tick - k) as f64 * DT).min(2.55)
        });
        grasp.step_at(phase, force, lift_time)
    }
}

fn jaw_clearances(grasp: &Grasp) -> Result<[f64; 2]> {
    let sheet = arm_sheet::cloth(&grasp.recording.sim)?;
    let mut margins = [0.; 2];
    for ((margin, id), half) in margins.iter_mut().zip(grasp.jaws.ids).zip(grasp.jaws.half) {
        let jaw = state(&grasp.recording.sim, id)?;
        let axis = DQuat::from_array(jaw.orientation) * DVec3::Y;
        let center = DVec3::from(jaw.position);
        let (lo, hi) = sheet
            .particles()
            .iter()
            .map(|p| (DVec3::from(p.position) - center).dot(axis))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), pos| {
                (lo.min(pos), hi.max(pos))
            });
        *margin = lo.max(-hi) - half.y - arm_sheet::RADIUS;
    }
    Ok(margins)
}

struct Branch {
    outcome: Outcome,
    actions: Recovery,
}

fn branch(columns: usize, peak_mn: u32, enabled: bool, out: &Path, end: End) -> Result<Branch> {
    let scene = closing_sheet_scene::scene(columns)?;
    let mut grasp = Grasp::pickup(&scene, columns)?;
    grasp.acquire(MAX_BLOCKS)?;
    let baseline = grasp.trace.len() - 1;
    let frame = grasp.frames.len() - 1;
    let anchor = fold_hold::snapshot(&grasp.recording.sim)?;
    let mut actions = Recovery::default();
    let mut clearances = Vec::with_capacity(HORIZON);
    let mut landing = None;
    for k in 1..=HORIZON {
        let (phase, force) = pulse(k, f64::from(peak_mn) * 0.001);
        let incoming = if end != End::Escape && actions.escaped.is_some() && landing.is_none() {
            Some(landing::particles(&grasp)?)
        } else {
            None
        };
        actions.drive(k, &mut grasp, phase, force).map_err(|err| {
            format!("{columns}-point {peak_mn} mN guard={enabled} tick {k} stroke={} lower={:?} open={:?} last={:?}: {err} ({:?})", grasp.stroke, actions.lower, actions.open, grasp.trace.last(), grasp.recording.sim.fault())
        })?;
        let last = baseline + k;
        actions.observe(
            k,
            &grasp.trace[last - 1],
            &grasp.trace[last],
            grasp.stroke,
            grasp.weight_n,
            enabled,
        );
        let margins = jaw_clearances(&grasp)?;
        actions.confirm_escape(k, &grasp.trace[last], margins);
        clearances.push(margins);
        if let Some(incoming) = incoming
            && grasp.trace[last].bench_up_n > 1e-9
        {
            landing = Some(landing::impact(
                k,
                &incoming,
                &landing::particles(&grasp)?,
                &grasp.trace[last - 1],
                &grasp.trace[last],
                grasp.weight_n,
            ));
        }
        if (actions.escaped.is_some() && end == End::Escape)
            || (landing.is_some() && end == End::FirstLanding)
        {
            if !k.is_multiple_of(STRIDE) {
                arm_sheet::capture(&mut grasp.recording, &mut grasp.frames)?;
            }
            break;
        }
        if actions.tighten.is_none() && k.is_multiple_of(PERIOD) {
            grasp.feedback();
        }
    }
    let dir = out.join(format!(
        "mesh-{columns}-{peak_mn}-{}",
        if enabled { "guard" } else { "control" }
    ));
    save_case(
        grasp.recording,
        grasp.frames,
        &grasp.trace,
        &scene,
        &dir,
        frame,
        columns,
    )?;
    let observed = grasp.trace.len() - 1 - baseline;
    write_json(
        &dir.join("escape-clearance.json"),
        &serde_json::to_value(clearances)?,
    )?;
    if let Some(impact) = &landing {
        write_json(&dir.join("landing.json"), impact)?;
    }
    let path = dir.join("frames.json");
    let mut manifest: Value = serde_json::from_slice(&fs::read(&path)?)?;
    let captures = &mut manifest["captures"];
    *captures = json!({"approach":0,"held":APPROACH/STRIDE,"baseline":frame,"end":frame + observed.div_ceil(STRIDE)});
    for (name, event) in [
        ("peak", Some(RAMP + HOLD)),
        ("released", Some(2 * RAMP + HOLD)),
        ("recovered", (observed == HORIZON).then_some(HORIZON)),
        ("tighten", actions.tighten),
        ("lower", actions.lower),
        ("open", actions.open),
        ("lost_aloft", actions.lost_aloft),
        ("escaped", actions.escaped),
        (
            "first_landing",
            landing
                .as_ref()
                .and_then(|v| v["tick"].as_u64())
                .map(|v| v as usize),
        ),
    ] {
        if let Some(tick) = event.filter(|tick| *tick <= observed) {
            captures[name] = json!(frame + tick.div_ceil(STRIDE));
        }
    }
    write_json(&path, &manifest)?;
    Ok(Branch {
        actions,
        outcome: Outcome {
            anchor,
            baseline,
            trace: grasp.trace,
            weight: grasp.weight_n,
        },
    })
}

fn metrics(b: &Branch) -> Value {
    let o = &b.outcome;
    let base = &o.trace[o.baseline];
    let rows = &o.trace[o.baseline + 1..];
    let last = &rows[rows.len() - WINDOW..];
    let shifts =
        |t: &Tick| std::array::from_fn::<_, 2, _>(|i| t.pad_frame_y_m[i] - base.pad_frame_y_m[i]);
    let mean = |data: &[Tick]| {
        std::array::from_fn::<_, 2, _>(|i| {
            data.iter().map(|r| shifts(r)[i]).sum::<f64>() / data.len() as f64
        })
    };
    let max: [f64; 2] =
        std::array::from_fn(|i| rows.iter().map(|r| shifts(r)[i].abs()).fold(0., f64::max));
    let bilateral = last.iter().filter(|r| supported(r, o.weight)).count();
    let bench = last
        .iter()
        .filter(|r| r.bench_up_n >= 0.8 * o.weight)
        .count();
    let result = if b.actions.escaped.is_some() {
        "lost_aloft"
    } else if b.actions.open.is_some() && bench == WINDOW {
        "placed_on_bench"
    } else if bilateral == WINDOW {
        "retained"
    } else {
        "unsettled"
    };
    json!({"outcome":result,"observed_postgrip_ticks":rows.len(),"escaped_after_tick":b.actions.escaped,"tighten_after_tick":b.actions.tighten,"lower_after_tick":b.actions.lower,"release_after_tick":b.actions.open,"lost_aloft_tick":b.actions.lost_aloft,
        "first_250um_tick":rows.iter().position(|r|shifts(r).iter().any(|x|x.abs()>0.00025)).map(|k|k+1),
        "peak_patch_motion_m":max,"patch_motion_at_400ms_m":rows.get(END-WINDOW..END).map(mean),"final_patch_motion_m":mean(last),
        "final_bilateral_ticks":bilateral,"final_bench_supported_ticks":bench,
        "final_stroke_m":last.last().unwrap().command_stroke_m})
}

fn pair(columns: usize, peak_mn: u32, out: &Path, end: End) -> Result<Value> {
    let guard = branch(columns, peak_mn, true, out, end)?;
    let control = branch(columns, peak_mn, false, out, end)?;
    compare(columns, peak_mn, &guard, &control)
}

fn compare(columns: usize, peak_mn: u32, guard: &Branch, control: &Branch) -> Result<Value> {
    let i = guard.outcome.baseline;
    if i != control.outcome.baseline
        || guard.outcome.anchor != control.outcome.anchor
        || guard.outcome.trace[..=i] != control.outcome.trace[..=i]
    {
        return Err("guard and control started with different tactile or physical states".into());
    }
    let same = guard
        .outcome
        .trace
        .iter()
        .zip(&control.outcome.trace)
        .take_while(|(a, b)| a == b)
        .count();
    Ok(
        json!({"columns":columns,"peak_mn":peak_mn,"baseline_ns":guard.outcome.trace[i].time_ns,"matching_postgrip_ticks":same-i-1,"guard":metrics(guard),"control":metrics(control)}),
    )
}

#[cfg(test)]
#[path = "fold_recovery_tests.rs"]
mod tests;

pub(crate) fn run(out: &Path, trial: Option<(usize, u32)>, end: End) -> Result<()> {
    let mut cases = Vec::new();
    let sets = match trial {
        Some((columns @ (5 | 7), force)) => vec![vec![(columns, force)]],
        Some(_) => {
            return Err("the calibrated recovery study accepts five or seven columns".into());
        }
        None => vec![
            vec![(5, 300), (7, 300)],
            vec![(5, 1100), (7, 1160)],
            vec![(5, 1250), (7, 1300)],
        ],
    };
    for trials in sets {
        std::thread::scope(|scope| -> Result<()> {
            let workers = trials
                .into_iter()
                .map(|(c, n)| scope.spawn(move || pair(c, n, out, end).map_err(|e| e.to_string())))
                .collect::<Vec<_>>();
            for w in workers {
                cases.push(w.join().map_err(|_| "recovery trial panicked")??);
            }
            Ok(())
        })?;
    }
    let mut comparison = json!({"dt_ns":1000000,"horizon_ticks":HORIZON,"extra_stroke_m":EXTRA_STROKE,"cases":cases});
    if end != End::Escape {
        comparison["follow_dropped"] = json!(true);
        comparison["stop_at_first_landing"] = json!(end == End::FirstLanding);
    }
    write_json(&out.join("comparison.json"), &comparison)
}
