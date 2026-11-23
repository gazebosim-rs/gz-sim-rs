// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Lift one free, folded triangular sheet with both jaws of the articulated arm.
use super::*;
use crate::{arm::Controller, sheet_energy::EnergyAudit};
use gz_sim_scene::deformable::{DeformableSample, bodies};
use serde::Serialize;
use std::fmt::Write;

const COLUMNS: usize = 3;
const ROWS: usize = 4;
pub(super) const COUNT: usize = COLUMNS * ROWS;
pub(super) const RADIUS: f64 = 0.011;
pub(super) const NODE_MASS: f64 = 0.0007;
pub(super) const BENCH: f64 = 0.120;
pub(super) const PAD_COMPLIANCE: f64 = 0.001;
const ITERATIONS: usize = 24;
const DT: f64 = 0.001;
const HOLD: std::ops::RangeInclusive<usize> = 1651..=1950;
const END: usize = 3400;
const RELEASED_TICKS: usize = 200;
pub(super) const JAW_STROKE: f64 = 0.07725;
const STRIDE: usize = 25;
const NOMINAL_FLARE: f64 = 0.00015;
pub(super) const STRETCH: f64 = 0.00001;
pub(super) const SHEAR: f64 = 0.00002;
pub(super) const BEND: f64 = 0.02;

#[derive(Clone, Copy, Debug, Serialize)]
pub(super) struct Case {
    pub substeps: usize,
    pub mass_scale: f64,
    pub stiffness_scale: f64,
    pub closed: bool,
}
#[derive(Debug)]
pub(super) struct SheetStepFault {
    pub tick: usize,
    pub diagnostic: String,
}
impl std::fmt::Display for SheetStepFault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "sheet failed at tick {}: {}", self.tick, self.diagnostic)
    }
}
impl std::error::Error for SheetStepFault {}

impl Default for Case {
    fn default() -> Self {
        Self {
            substeps: 2,
            mass_scale: 1.,
            stiffness_scale: 1.,
            closed: true,
        }
    }
}
// Two vertical panels joined along the crown by one row of triangulated cloth.
// The nominal fold opens slightly at the crown and free side edges.
pub(super) fn seed(i: usize, flare: f64) -> DVec3 {
    let row = i / COLUMNS;
    let col = i % COLUMNS;
    let top = usize::from(row == 1 || row == 2) as f64;
    let side = usize::from(col != 1) as f64;
    let offset = RADIUS + 0.00001 + flare * (top + side);
    let sign = if panel(i) == 0 { -1. } else { 1. };
    DVec3::new(
        0.32 + sign * offset,
        (col as f64 - 1.) * 0.053,
        BENCH + RADIUS + 0.043 * top,
    )
}
fn panel(i: usize) -> usize {
    usize::from(i / COLUMNS >= ROWS / 2)
}
pub(super) fn name(i: usize) -> String {
    format!("default::source::body::sheet_{i}")
}
fn scene(robot: &str, c: Case, flare: f64) -> Result<String> {
    scene_with_loads(robot, c, flare, false)
}

pub(super) fn scene_with_loads(
    robot: &str,
    c: Case,
    flare: f64,
    self_contact_loads: bool,
) -> Result<String> {
    scene_with_marker_grid(robot, c, flare, self_contact_loads, COLUMNS)
}

// Extra visuals are passive. Material bindings still identify the twelve original
// particles at their shared locations, independent of the presentation grid.
pub(super) fn scene_with_marker_grid(
    robot: &str,
    c: Case,
    flare: f64,
    self_contact_loads: bool,
    columns: usize,
) -> Result<String> {
    let mut particles = String::new();
    let mut bindings = String::new();
    for i in 0..COUNT {
        let p = seed(i, flare);
        writeln!(
            particles,
            "<particle position=\"{} {} {}\" mass=\"{}\"/>",
            p.x,
            p.y,
            p.z,
            NODE_MASS * c.mass_scale
        )
        .unwrap();

        writeln!(
            bindings,
            "<particle_visual material=\"sheet\" particle=\"{i}\" visual=\"{}\"/>",
            name((i / COLUMNS) * columns + (i % COLUMNS) * ((columns - 1) / (COLUMNS - 1)))
        )
        .unwrap();
    }
    let witness_attr = if self_contact_loads {
        " self_contact_loads=\"true\""
    } else {
        ""
    };
    let material = format!(
        "<cloth name=\"sheet\" pad_loads=\"true\"{witness_attr} columns=\"{COLUMNS}\" substeps=\"{}\" iterations=\"{ITERATIONS}\" radius=\"{RADIUS}\" self_contact=\"true\" self_friction=\"0.3\" friction=\"0.7\" stretch=\"{}\" shear=\"{}\" bend=\"{}\">{particles}</cloth>",
        c.substeps,
        STRETCH / c.stiffness_scale,
        SHEAR / c.stiffness_scale,
        BEND / c.stiffness_scale
    );
    scene_with_material(robot, columns, flare, &material, &bindings)
}

pub(super) fn scene_with_material(
    robot: &str,
    columns: usize,
    flare: f64,
    material: &str,
    bindings: &str,
) -> Result<String> {
    let (arm, rest) = robot
        .split_once("<model name=\"parcel\">")
        .ok_or("missing arm parcel")?;
    let (_, rest) = rest
        .split_once("<model name=\"source\">")
        .ok_or("missing bench")?;
    let (source, rest) = rest.split_once("</link>").ok_or("missing bench link")?;
    let (world, _) = rest.split_once("</world>").ok_or("missing arm world")?;
    let mut xml = format!("{arm}<model name=\"source\">{source}");
    for i in 0..ROWS * columns {
        let row = i / columns;
        let u = (i % columns) as f64 * (COLUMNS - 1) as f64 / (columns - 1) as f64;
        let a = row * COLUMNS + u.floor() as usize;
        let b = (a + 1).min((row + 1) * COLUMNS - 1);
        let p = seed(a, flare).lerp(seed(b, flare), u.fract());
        let rgb = if row < ROWS / 2 {
            "1 0.49 0.06"
        } else {
            "0.05 0.80 0.90"
        };
        writeln!(xml,"<visual name=\"sheet_{i}\"><pose>{} {} {} 0 0 0</pose><geometry><sphere><radius>{RADIUS}</radius></sphere></geometry><material><diffuse>{rgb} 1</diffuse></material></visual>",p.x-0.32,p.y,p.z-0.06).unwrap();
    }
    write!(xml,"</link>{world}<plugin name=\"gz::sim::systems::Deformables\" filename=\"gz-sim-deformables-system\">
        <box link=\"default::arm::left\" coupling=\"articulated_wrench\" normal_compliance=\"{PAD_COMPLIANCE}\"/>
        <box link=\"default::arm::right\" coupling=\"articulated_wrench\" normal_compliance=\"{PAD_COMPLIANCE}\"/>
        <plane point=\"0 0 {BENCH}\" normal=\"0 0 1\"/>
        {material}{bindings}</plugin></world></sdf>").unwrap();
    Ok(xml)
}
#[derive(Default, Debug, Serialize)]
pub(super) struct Metrics {
    pub case: Case,
    pub energy: EnergyAudit,
    pub panel_flare_m: f64,
    pub weight_n: f64,
    pub mean_hold_lift_m: [f64; 2],
    pub mean_hold_panel_relative_height_m: [f64; 2],
    pub max_hold_panel_drift_m: [f64; 2],
    pub mean_hold_jaw_up_n: [f64; 2],
    pub mean_hold_jaw_squeeze_n: [f64; 2],
    pub mean_hold_bench_up_n: f64,
    pub max_strain: f64,
    pub max_compression_m: f64,
    pub max_linear_impulse_residual_ns: f64,
    pub step_p95_ns: u64,
    pub refined_ticks: usize,
    pub max_refinement_doublings: u32,
    pub max_material_substeps: usize,
    pub accepted_material_substeps: usize,
    pub min_hold_bench_gap_m: f64,
    pub min_hold_panel_squeeze_n: [f64; 2],
    pub mean_released_bench_up_n: f64,
    pub mean_released_jaw_force_n: f64,
    pub min_bench_margin_m: f64,
    /// First index is jaw (left/right), second is material panel (left/right).
    pub mean_hold_panel_up_by_jaw_n: [[f64; 2]; 2],
    pub mean_hold_panel_normal_by_jaw_n: [[f64; 2]; 2],
    pub max_pad_attribution_residual_ns: f64,
    pub sum_sheet_self_contact_ns: f64,
    pub mean_hold_self_contact_n: f64,
}
impl Metrics {
    pub(super) fn qualify(&self) -> Result<()> {
        let supported: f64 = self.mean_hold_jaw_up_n.iter().sum();
        let ok = if self.case.closed {
            // Each pad must really contact its own panel for the full hold.
            self.min_hold_panel_squeeze_n.iter().all(|n| *n > 0.01)
                && self.mean_hold_panel_normal_by_jaw_n[0][1] < 1e-7
                && self.mean_hold_panel_normal_by_jaw_n[1][0] < 1e-7
                && self.mean_hold_lift_m.iter().all(|z| *z > 0.08)
                && self.max_hold_panel_drift_m.iter().all(|d| *d < 0.002)
                && self.min_hold_bench_gap_m > 0.03
                && (supported - self.weight_n).abs() < 0.1 * self.weight_n
                && self.mean_hold_bench_up_n.abs() < 0.001
        } else {
            self.mean_hold_lift_m.iter().all(|z| z.abs() < 0.005)
                && self.mean_hold_jaw_up_n == [0.; 2]
                && (self.mean_hold_bench_up_n - self.weight_n).abs() < 0.1 * self.weight_n
        };
        if !ok
            || self.max_pad_attribution_residual_ns > 1e-10
            || self.max_linear_impulse_residual_ns > 1e-7
            || self.max_compression_m >= RADIUS
            || self.max_strain > 0.05
            || (self.mean_released_bench_up_n - self.weight_n).abs() > 0.1 * self.weight_n
            || self.mean_released_jaw_force_n > 0.001
            || self.min_bench_margin_m < 0.
        {
            return Err(format!("folded sheet failed qualification: {self:?}").into());
        }
        Ok(())
    }
}
// In the baseline trials the fold itself must carry a load, and each jaw
// must support its own panel against gravity. The broader envelope reports
// those individual measurements even if one panel remains unloaded.
fn qualify_loaded_fold(m: &Metrics) -> Result<()> {
    m.qualify()?;
    if m.case.closed
        && (m.mean_hold_self_contact_n <= 0.01 * m.weight_n
            || (0..2).any(|jaw| m.mean_hold_panel_up_by_jaw_n[jaw][jaw] <= 0.))
    {
        return Err("fold pressure or upward support on an individual panel was lost".into());
    }
    Ok(())
}
pub(super) fn cloth(sim: &Simulation) -> Result<&DeformableSample> {
    bodies(sim)
        .find(|(n, _)| *n == "sheet")
        .map(|(_, b)| b)
        .ok_or_else(|| "missing sheet".into())
}
#[derive(Serialize)]
pub(super) struct Frame {
    time_ns: u128,
    positions: Vec<[f64; 3]>,
}
pub(super) fn capture(rec: &mut Recording, frames: &mut Vec<Frame>) -> Result<()> {
    let b = cloth(&rec.sim)?;
    frames.push(Frame {
        time_ns: b.sim_time().as_nanos(),
        positions: b.particles().iter().map(|p| p.position).collect(),
    });
    rec.capture()
}
pub(super) fn run_case(case: Case, out: Option<&Path>) -> Result<Metrics> {
    cycle(case, NOMINAL_FLARE, out)
}
fn cycle(case: Case, flare: f64, out: Option<&Path>) -> Result<Metrics> {
    simulate(&scene(arm::SCENE, case, flare)?, case, flare, out)
}
fn simulate(xml: &str, case: Case, flare: f64, out: Option<&Path>) -> Result<Metrics> {
    let mut r = Recording::new(xml, "arm_sheet.sdf", "default")?;
    let controller = Controller::new(&r.sim)?;
    let jaws = [
        entity_id(&r.sim, "default::arm::left")?,
        entity_id(&r.sim, "default::arm::right")?,
    ];
    let pads = [
        arm_cable::half_size(&r.sdf, "arm", "left")?,
        arm_cable::half_size(&r.sdf, "arm", "right")?,
    ];
    let bench_box = arm_cable::half_size(&r.sdf, "source", "body")?;
    let bench_center = state(&r.sim, entity_id(&r.sim, "default::source::body")?)?.position;
    let hold_ticks = HOLD.clone().count() as f64;
    let mass = NODE_MASS * case.mass_scale;
    let mut m = Metrics {
        case,
        panel_flare_m: flare,
        energy: EnergyAudit::new(&r.sim, "sheet", jaws, BENCH)?,
        weight_n: COUNT as f64 * mass * 9.81,
        min_hold_bench_gap_m: f64::INFINITY,
        min_hold_panel_squeeze_n: [f64::INFINITY; 2],
        min_bench_margin_m: f64::INFINITY,
        ..Default::default()
    };
    let mut frames = vec![];
    let mut momentum = DVec3::ZERO;
    let mut relative_at_hold_start = [0.; 2];
    let edges: Vec<_> = (0..COUNT)
        .flat_map(|i| {
            [
                (i % COLUMNS + 1 < COLUMNS).then_some(i + 1),
                (i + COLUMNS < COUNT).then_some(i + COLUMNS),
            ]
            .into_iter()
            .flatten()
            .map(move |j| (i, j, seed(i, flare).distance(seed(j, flare))))
        })
        .collect();
    if out.is_some() {
        capture(&mut r, &mut frames)?
    }
    for tick in 1..=END {
        arm_cable::lift_with_jaws(
            &controller,
            &mut r.sim,
            (tick - 1) as f64 * DT,
            if case.closed { JAW_STROKE } else { 0. },
        )?;
        r.advance().map_err(|e| SheetStepFault {
            tick,
            diagnostic: format!("{e} {:?}", r.sim.fault()),
        })?;
        let b = cloth(&r.sim)?;
        m.energy.observe(&r.sim, b)?;
        let refinements = b.last_sweep_refinements();
        let accepted = case.substeps << refinements;
        m.refined_ticks += usize::from(refinements > 0);
        m.max_refinement_doublings = m.max_refinement_doublings.max(refinements);
        m.max_material_substeps = m.max_material_substeps.max(accepted);
        m.accepted_material_substeps += accepted;
        let points = b.particles();
        let self_load = b.last_self_contact_impulse().normal;
        m.sum_sheet_self_contact_ns += self_load;
        if HOLD.contains(&tick) {
            m.mean_hold_self_contact_n += self_load / (hold_ticks * DT);
        }
        let [left, right, bench] = b.last_boundary_impulses() else {
            return Err("missing sheet boundary loads".into());
        };
        let js = [left, right, bench].map(|j| DVec3::from(j.linear));
        let mut measured = [DVec3::ZERO; 2];
        let mut same_panel_normal = [0.; 2];
        for load in b.last_pad_loads() {
            let j = DVec3::from(load.normal) + DVec3::from(load.tangent);
            measured[load.boundary] += j;
            if HOLD.contains(&tick) {
                let up = -j.z / (hold_ticks * DT);
                let layer = panel(load.particle);
                m.mean_hold_panel_up_by_jaw_n[load.boundary][layer] += up;
                if layer == load.boundary {
                    same_panel_normal[layer] += DVec3::from(load.normal).length() / DT;
                }
                m.mean_hold_panel_normal_by_jaw_n[load.boundary][layer] +=
                    DVec3::from(load.normal).length() / (hold_ticks * DT);
            }
        }
        for (observed, total) in measured.into_iter().zip(js) {
            m.max_pad_attribution_residual_ns = m
                .max_pad_attribution_residual_ns
                .max(observed.distance(total));
        }
        let now = points
            .iter()
            .map(|p| mass * DVec3::from(p.velocity))
            .sum::<DVec3>();
        m.max_linear_impulse_residual_ns = m
            .max_linear_impulse_residual_ns
            .max((now - momentum + DT * m.weight_n * DVec3::Z + js.iter().sum::<DVec3>()).length());
        momentum = now;
        for &(i, j, rest) in &edges {
            m.max_strain = m.max_strain.max(
                (DVec3::from(points[i].position).distance(points[j].position.into()) / rest - 1.)
                    .abs(),
            );
        }
        let mut jaw_z = 0.;
        for (index, id) in jaws.into_iter().enumerate() {
            let jaw = state(&r.sim, id)?;
            jaw_z += jaw.position[2] * 0.5;
            let inv = glam::DQuat::from_array(jaw.orientation).conjugate();
            if HOLD.contains(&tick) {
                m.mean_hold_jaw_squeeze_n[index] +=
                    (inv * js[index]).x * if index == 0 { -1. } else { 1. } / (hold_ticks * DT);
            }
            for p in points {
                let q = (inv * (DVec3::from(p.position) - DVec3::from(jaw.position))).abs()
                    - pads[index];
                m.max_compression_m = m
                    .max_compression_m
                    .max(RADIUS - q.max(DVec3::ZERO).length() - q.max_element().min(0.));
            }
        }
        let mut heights = [0.; 2];
        for (i, p) in points.iter().enumerate() {
            let [x, y, z] = p.position;
            m.min_bench_margin_m = m.min_bench_margin_m.min(
                (bench_box.x - (x - bench_center[0]).abs())
                    .min(bench_box.y - (y - bench_center[1]).abs())
                    - RADIUS,
            );
            if HOLD.contains(&tick) {
                heights[panel(i)] += (z - jaw_z) / (COUNT / 2) as f64;
                m.min_hold_bench_gap_m = m.min_hold_bench_gap_m.min(z - BENCH - RADIUS);
                m.mean_hold_lift_m[panel(i)] +=
                    (z - seed(i, flare).z) / (hold_ticks * (COUNT / 2) as f64);
            }
        }
        if HOLD.contains(&tick) {
            if tick == *HOLD.start() {
                relative_at_hold_start = heights;
            }
            for i in 0..2 {
                m.mean_hold_panel_relative_height_m[i] += heights[i] / hold_ticks;
                m.max_hold_panel_drift_m[i] =
                    m.max_hold_panel_drift_m[i].max((heights[i] - relative_at_hold_start[i]).abs());
                m.min_hold_panel_squeeze_n[i] =
                    m.min_hold_panel_squeeze_n[i].min(same_panel_normal[i]);
            }
            for (up, j) in m.mean_hold_jaw_up_n.iter_mut().zip(js) {
                *up -= j.z / (hold_ticks * DT);
            }
            m.mean_hold_bench_up_n -= js[2].z / (hold_ticks * DT);
        }
        if tick > END - RELEASED_TICKS {
            m.mean_released_bench_up_n -= js[2].z / (RELEASED_TICKS as f64 * DT);
            m.mean_released_jaw_force_n +=
                (js[0].length() + js[1].length()) / (RELEASED_TICKS as f64 * DT);
        }
        if out.is_some() && tick % STRIDE == 0 {
            capture(&mut r, &mut frames)?;
        }
    }
    r.costs_ns.sort_unstable();
    m.step_p95_ns = r.costs_ns[(END * 95).div_ceil(100) - 1];
    if let Some(out) = out {
        save_archive(
            r,
            frames,
            out,
            json!({"pickup":26,"rising":45,"hold":72,"release":112,"settled":136}),
        )?;
    }
    Ok(m)
}

pub(super) fn save_archive(
    r: Recording,
    frames: Vec<Frame>,
    out: &Path,
    captures: Value,
) -> Result<()> {
    r.save(out)?;
    write_json(
        &out.join("particle-frames.json"),
        &serde_json::to_value(frames)?,
    )?;
    write_json(
        &out.join("frames.json"),
        &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"cloth_grid":{"rows":ROWS,"columns":COLUMNS,"skin_radius_m":RADIUS,"last_row_left":1,"first_row_right":ROWS-2},"particle_visuals":(0..COUNT).map(name).collect::<Vec<_>>(),"captures":captures}),
    )
}
pub(super) fn run(out: &Path) -> Result<()> {
    let mut rows = vec![];
    for substeps in [1, 2, 4] {
        for closed in [true, false] {
            let path = (substeps == 2).then(|| out.join(if closed { "held" } else { "open" }));
            let m = run_case(
                Case {
                    substeps,
                    closed,
                    ..Default::default()
                },
                path.as_deref(),
            )?;

            qualify_loaded_fold(&m)?;
            rows.push(m);
        }
    }
    // The nearly parallel, uniformly spaced layers are a separate contact
    // challenge. Repeat on finer clocks to expose load resolution sensitivity.
    let mut parallel = Vec::new();
    for substeps in [8, 12, 16] {
        let output = (substeps == 8).then(|| out.join("pressed"));
        let m = cycle(
            Case {
                substeps,
                ..Default::default()
            },
            0.,
            output.as_deref(),
        )?;
        qualify_loaded_fold(&m)?;
        parallel.push(m);
    }
    let mut coarse_parallel = Vec::new();
    for substeps in [1, 2, 4] {
        let c = Case {
            substeps,
            ..Default::default()
        };
        coarse_parallel.push(crate::sheet_sweep::Trial::from_result(
            c,
            cycle(c, 0., None),
        )?);
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"cloth_rows":ROWS,"cloth_columns":COLUMNS,"total_mass_kg":NODE_MASS * COUNT as f64,"radius_m":RADIUS,"iterations":ITERATIONS,"substeps":[1,2,4],"stretch_compliance_m_per_n":STRETCH,"shear_compliance_m_per_n":SHEAR,"bend_compliance_rad_per_nm":BEND,"pad_normal_compliance_m_per_n":PAD_COMPLIANCE,"self_friction":0.3,"pad_friction":0.7,"commanded_jaw_stroke_m":JAW_STROKE,"trials":rows,"nominal_panel_flare_m":NOMINAL_FLARE,"closest_initial_skin_gap_m":0.00002,"parallel_layer_references":parallel,"parallel_coarse_attempts":coarse_parallel}),
    )
}

#[cfg(test)]
#[path = "arm_sheet_tests.rs"]
mod tests;
