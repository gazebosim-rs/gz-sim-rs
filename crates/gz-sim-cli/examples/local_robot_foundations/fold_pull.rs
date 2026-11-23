// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Force-controlled grasp of a folded sheet under a transverse body load.
use super::*;
use crate::fold_closure::{APPROACH, Jaws, START};
use glam::DQuat;
use gz_physics_xpbd::{PadSample, Particle};
use gz_sim_scene::deformable::{set_material_force, set_material_resolution};
use serde::Serialize;

#[path = "fold_breakaway.rs"]
pub(super) mod breakaway;

const DT: f64 = 0.001;
const STRIDE: usize = 25;
const PERIOD: usize = 50;
const WINDOW: usize = 20;
const TARGET: f64 = 0.002;
const BAND: f64 = 0.00025;
const PULL: f64 = 0.3;
const RAMP: usize = 50;
const HOLD: usize = 100;
const RECOVER: usize = 200;
const END: usize = 2 * RAMP + HOLD + RECOVER;
const MAX_BLOCKS: usize = 100;

#[derive(Debug, Clone, Serialize, PartialEq)]
struct Tick {
    time_ns: u128,
    phase: &'static str,
    force_y_n: f64,
    mass_kg: f64,
    momentum_y_kg_m_s: f64,
    bench_side_reaction_n: f64,
    command_stroke_m: f64,
    cross_panel_n: f64,
    panel_gap_m: f64,
    aperture_m: f64,
    jaw_up_n: [f64; 2],
    jaw_side_reaction_n: [f64; 2],
    jaw_squeeze_n: [f64; 2],
    bench_up_n: f64,
    min_bench_clearance_m: f64,
    jaw_frame_y_m: f64,
    pad_frame_y_m: [f64; 2],
}

fn lateral_positions(
    particles: &[Particle],
    pads: &[PadSample],
    jaws: &[BodyState; 2],
) -> (f64, [f64; 2]) {
    let positions = jaws.each_ref().map(|j| DVec3::from(j.position));
    let axes = jaws
        .each_ref()
        .map(|j| DQuat::from_array(j.orientation) * DVec3::Y);
    let mut mass = 0.;
    let mut moment = DVec3::ZERO;
    for p in particles {
        let m = p.inverse_mass.recip();
        mass += m;
        moment += m * DVec3::from(p.position);
    }
    let relative = (moment / mass - (positions[0] + positions[1]) * 0.5).dot(axes[0]);
    // The original material patch on each panel has eight equal-area quadrature sites.
    // Retain their material identities even if contact with the jaw is lost.
    let patches = std::array::from_fn(|j| {
        let n = pads.len() / 2;
        let local_y: f64 = pads[j * n..(j + 1) * n]
            .iter()
            .map(|site| {
                let location: DVec3 = site
                    .particles
                    .iter()
                    .zip(site.weights)
                    .map(|(&i, w)| DVec3::from(particles[i].position) * w)
                    .sum();
                (location - positions[j]).dot(axes[j])
            })
            .sum();
        local_y / n as f64
    });
    (relative, patches)
}

struct Grasp {
    recording: Recording,
    controller: arm::Controller,
    jaws: Jaws,
    pads: Vec<PadSample>,
    columns: usize,
    weight_n: f64,
    frames: Vec<arm_sheet::Frame>,
    trace: Vec<Tick>,
    stroke: f64,
}

impl Grasp {
    fn pickup(scene: &str, columns: usize) -> Result<Self> {
        let mut recording = Recording::new(scene, "fold_pull.sdf", "default")?;
        let controller = arm::Controller::new(&recording.sim)?;
        let jaws = Jaws::new(&recording)?;
        let pads = fold_patch::material_samples(jaws.half[0].y, columns);
        let mut frames = vec![];
        arm_sheet::capture(&mut recording, &mut frames)?;
        for tick in 1..=APPROACH {
            arm_cable::lift_with_jaws(
                &controller,
                &mut recording.sim,
                (tick - 1) as f64 * DT,
                START,
            )?;
            recording.advance()?;
            if tick % STRIDE == 0 {
                arm_sheet::capture(&mut recording, &mut frames)?;
            }
        }
        set_material_resolution(&mut recording.sim, "sheet", 32, 24)?;
        let weight_n = arm_sheet::cloth(&recording.sim)?
            .particles()
            .iter()
            .map(|p| 9.81 / p.inverse_mass)
            .sum();
        Ok(Self {
            recording,
            controller,
            jaws,
            pads,
            columns,
            weight_n,
            frames,
            trace: vec![],
            stroke: START,
        })
    }

    fn step(&mut self, phase: &'static str, force_y_n: f64) -> Result<()> {
        let r = &mut self.recording;
        arm_cable::lift_with_jaws(
            &self.controller,
            &mut r.sim,
            APPROACH as f64 * DT,
            self.stroke,
        )?;
        set_material_force(&mut r.sim, "sheet", [0., force_y_n, 0.])?;
        r.advance()?;
        let s = fold_closure::sample(r, &self.jaws, self.columns, 0, self.stroke)?;
        let sheet = arm_sheet::cloth(&r.sim)?;
        let jaw_states = [
            state(&r.sim, self.jaws.ids[0])?,
            state(&r.sim, self.jaws.ids[1])?,
        ];
        let (jaw_frame_y_m, pad_frame_y_m) =
            lateral_positions(sheet.particles(), &self.pads, &jaw_states);
        let jaw_side_reaction_n =
            std::array::from_fn(|j| sheet.last_boundary_impulses()[j].linear[1] / DT);
        self.trace.push(Tick {
            time_ns: r.sim.elapsed().as_nanos(),
            phase,
            force_y_n,
            mass_kg: self.weight_n / 9.81,
            momentum_y_kg_m_s: sheet
                .particles()
                .iter()
                .map(|p| p.velocity[1] / p.inverse_mass)
                .sum(),
            bench_side_reaction_n: sheet.last_boundary_impulses()[2].linear[1] / DT,
            command_stroke_m: self.stroke,
            cross_panel_n: s.cross_panel_n,
            panel_gap_m: s.panel_gap_m,
            aperture_m: s.aperture_m,
            jaw_up_n: s.jaw_up_n,
            jaw_side_reaction_n,
            jaw_squeeze_n: s.jaw_squeeze_n,
            bench_up_n: s.bench_up_n,
            min_bench_clearance_m: s.min_bench_clearance_m,
            jaw_frame_y_m,
            pad_frame_y_m,
        });
        if self.trace.len().is_multiple_of(STRIDE) {
            arm_sheet::capture(r, &mut self.frames)?;
        }
        Ok(())
    }

    fn feedback(&mut self) -> bool {
        let tail = &self.trace[self.trace.len() - WINDOW..];
        let mean = tail.iter().map(|s| s.cross_panel_n).sum::<f64>() / WINDOW as f64;
        let gap = tail.iter().map(|s| s.panel_gap_m).sum::<f64>() / WINDOW as f64;
        self.stroke = command(self.stroke, mean, gap);
        (mean - TARGET).abs() <= BAND
            && tail
                .iter()
                .all(|s| s.cross_panel_n >= 0.0001 && supported(s, self.weight_n))
    }

    fn acquire(&mut self, max_blocks: usize) -> Result<()> {
        let mut previous = false;
        for block in 0..max_blocks {
            for _ in 0..PERIOD {
                self.step("acquire", 0.)?;
            }
            let ready = self.feedback();
            if ready && previous {
                return Ok(());
            }
            previous = ready;
            if block % 10 == 0 {
                eprintln!(
                    "{} points, acquisition block {}, command {:.8} m, last layer force {:.6} N",
                    self.columns,
                    block,
                    self.stroke,
                    self.trace.last().unwrap().cross_panel_n
                );
            }
        }
        Err(format!(
            "{}-point sheet did not settle at the layer-force target",
            self.columns
        )
        .into())
    }
}
fn command(stroke: f64, force: f64, gap: f64) -> f64 {
    let error = TARGET - force;
    let correction = if error.abs() <= BAND {
        0.
    } else if force == 0. && gap > 0.6e-6 {
        25e-6
    } else {
        (0.003 * error).clamp(-25e-6, 25e-6)
    };
    (stroke + correction).clamp(0., 0.080)
}

fn supported(s: &Tick, weight: f64) -> bool {
    s.jaw_up_n.iter().all(|v| *v > 0.)
        && (s.jaw_up_n.iter().sum::<f64>() / weight - 1.).abs() < 0.025
        && s.bench_up_n.abs() < 1e-6
        && s.min_bench_clearance_m > 0.01
}

fn pulse(tick: usize, peak: f64) -> (&'static str, f64) {
    if tick <= RAMP {
        ("ramp", peak * tick as f64 / RAMP as f64)
    } else if tick <= RAMP + HOLD {
        ("pull", peak)
    } else if tick <= 2 * RAMP + HOLD {
        (
            "unload",
            peak * (2 * RAMP + HOLD - tick) as f64 / RAMP as f64,
        )
    } else {
        ("recovery", 0.)
    }
}

struct Outcome {
    anchor: Value,
    baseline: usize,
    trace: Vec<Tick>,
    weight: f64,
}

fn trial(columns: usize, loaded: bool, out: &Path) -> Result<Outcome> {
    let name = format!("mesh-{columns}-{}", if loaded { "pull" } else { "sham" });
    trial_at(
        columns,
        if loaded { PULL } else { 0. },
        32,
        24,
        &out.join(name),
    )
}

fn trial_at(
    columns: usize,
    load_n: f64,
    substeps: u32,
    iterations: u32,
    dir: &Path,
) -> Result<Outcome> {
    let scene = closing_sheet_scene::scene(columns)?;
    let mut grasp = Grasp::pickup(&scene, columns)?;
    grasp.acquire(MAX_BLOCKS)?;
    let anchor = fold_hold::snapshot(&grasp.recording.sim)?;
    set_material_resolution(&mut grasp.recording.sim, "sheet", substeps, iterations)?;
    let baseline = grasp.trace.len() - 1;
    let frame = grasp.frames.len() - 1;
    for tick in 1..=END {
        let (phase, load) = pulse(tick, load_n);
        grasp.step(phase, load)?;
        if tick.is_multiple_of(PERIOD) {
            grasp.feedback();
        }
    }
    save_case(
        grasp.recording,
        grasp.frames,
        &grasp.trace,
        &scene,
        dir,
        frame,
        columns,
    )?;
    Ok(Outcome {
        anchor,
        baseline,
        trace: grasp.trace,
        weight: grasp.weight_n,
    })
}

fn save_case(
    recording: Recording,
    frames: Vec<arm_sheet::Frame>,
    trace: &[Tick],
    scene: &str,
    dir: &Path,
    frame: usize,
    columns: usize,
) -> Result<()> {
    recording.save(dir)?;
    fs::write(dir.join("world.sdf"), scene)?;
    write_json(&dir.join("trace.json"), &serde_json::to_value(trace)?)?;
    write_json(
        &dir.join("particle-frames.json"),
        &serde_json::to_value(frames)?,
    )?;
    let captures = json!({"approach":0,"held":APPROACH/STRIDE,"baseline":frame,"peak":frame+(RAMP+HOLD)/STRIDE,"released":frame+(2*RAMP+HOLD)/STRIDE,"recovered":frame+END/STRIDE});
    write_json(
        &dir.join("frames.json"),
        &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"source_public_dt_ns":1_000_000,"capture_stride_ticks":STRIDE,"captures":captures,"particle_visuals":(0..4*columns).map(arm_sheet::name).collect::<Vec<_>>(),"jaw_visuals":["default::arm::left::left","default::arm::right::right"],"cloth_grid":{"rows":4,"columns":columns,"skin_radius_m":arm_sheet::RADIUS,"last_row_left":1,"first_row_right":2}}),
    )?;
    Ok(())
}

#[derive(Serialize)]
struct Window {
    mean_cross_panel_n: f64,
    mean_jaw_up_n: [f64; 2],
    mean_jaw_side_reaction_n: [f64; 2],
    mean_aperture_m: f64,
    mean_centroid_change_m: f64,
    mean_pad_change_m: [f64; 2],
}

impl Outcome {
    fn window(&self, offset: usize) -> Window {
        let end = self.baseline + offset + 1;
        let rows = &self.trace[end - WINDOW..end];
        let mean = |pick: fn(&Tick) -> f64| rows.iter().map(pick).sum::<f64>() / WINDOW as f64;
        let ref_pose = &self.trace[self.baseline];
        Window {
            mean_cross_panel_n: mean(|s| s.cross_panel_n),
            mean_jaw_up_n: [mean(|s| s.jaw_up_n[0]), mean(|s| s.jaw_up_n[1])],
            mean_jaw_side_reaction_n: [
                mean(|s| s.jaw_side_reaction_n[0]),
                mean(|s| s.jaw_side_reaction_n[1]),
            ],
            mean_aperture_m: mean(|s| s.aperture_m),
            mean_centroid_change_m: mean(|s| s.jaw_frame_y_m) - ref_pose.jaw_frame_y_m,
            mean_pad_change_m: [
                mean(|s| s.pad_frame_y_m[0]) - ref_pose.pad_frame_y_m[0],
                mean(|s| s.pad_frame_y_m[1]) - ref_pose.pad_frame_y_m[1],
            ],
        }
    }

    fn stages(&self) -> Value {
        let disturbance = &self.trace[self.baseline + 1..];
        json!({
            "baseline": self.window(0),
            "pull": self.window(RAMP+HOLD),
            "recovered": self.window(END),
            "supported_ticks": disturbance.iter().filter(|s|supported(s,self.weight)).count(),
            "min_bench_clearance_m": disturbance.iter().map(|s|s.min_bench_clearance_m).fold(f64::INFINITY,f64::min),
            "loaded_ticks_in_band": disturbance.iter().filter(|s|s.phase=="pull" && (s.cross_panel_n - TARGET).abs() <= BAND).count()
        })
    }
}

fn report(columns: usize, pull: Outcome, sham: Outcome) -> Result<Value> {
    let i = pull.baseline;
    if i != sham.baseline || pull.anchor != sham.anchor || pull.trace[..=i] != sham.trace[..=i] {
        return Err(
            "pulled sheet and sham did not start with identical live states, readings and commands"
                .into(),
        );
    }
    let delta = |i: usize| {
        [
            pull.trace[i].jaw_frame_y_m - sham.trace[i].jaw_frame_y_m,
            pull.trace[i].pad_frame_y_m[0] - sham.trace[i].pad_frame_y_m[0],
            pull.trace[i].pad_frame_y_m[1] - sham.trace[i].pad_frame_y_m[1],
        ]
    };
    let peak: [f64; 3] = std::array::from_fn(|j| {
        (i + 1..=i + 2 * RAMP + HOLD)
            .map(|k| delta(k)[j])
            .max_by(|a, b| a.abs().total_cmp(&b.abs()))
            .unwrap()
    });
    let residual: [f64; 3] = std::array::from_fn(|j| {
        (i + END + 1 - WINDOW..=i + END)
            .map(|k| delta(k)[j])
            .sum::<f64>()
            / WINDOW as f64
    });
    Ok(
        json!({"columns":columns,"weight_n":pull.weight,"baseline_time_ns":pull.trace[i].time_ns,"matched_prefix_ticks":i+1,"pull":pull.stages(),"sham":sham.stages(),"peak_signed_displacement_vs_sham_m":peak,"residual_displacement_vs_sham_m":residual}),
    )
}

pub(super) fn run(out: &Path) -> Result<()> {
    run_cases(out, trial)
}

fn run_cases(out: &Path, simulate: fn(usize, bool, &Path) -> Result<Outcome>) -> Result<()> {
    let cases = std::thread::scope(|scope| {
        let jobs = [5, 7].map(|c| {
            [true, false]
                .map(|load| scope.spawn(move || simulate(c, load, out).map_err(|e| e.to_string())))
        });
        jobs.map(|pair| {
            pair.map(|job| {
                job.join()
                    .map_err(|_| "pull trial panicked".to_string())
                    .and_then(|result| result)
            })
        })
    });
    let mut trials = vec![];
    for (c, [pulled, sham]) in [5, 7].into_iter().zip(cases) {
        trials.push(report(c, pulled?, sham?)?);
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"target_n":TARGET,"force_band_n":BAND,"pull_n":PULL,"dt_ns":1_000_000,"clock_substeps":32,"clock_iterations":24,"feedback_period_ticks":PERIOD,"window_ticks":WINDOW,"ramp_ticks":RAMP,"hold_ticks":HOLD,"recovery_ticks":RECOVER,"trials":trials}),
    )
}

#[cfg(test)]
#[path = "fold_pull_tests.rs"]
mod tests;
