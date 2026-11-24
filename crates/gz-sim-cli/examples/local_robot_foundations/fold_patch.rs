// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Prescribe one material area on each outer panel and replay the same measured jaw motion.
use super::*;
use crate::arm_sheet::{self, PAD_COMPLIANCE, RADIUS};
use crate::fold_mesh::{self, Anchor, Loads};
use glam::DQuat;
use gz_physics_xpbd::{PadSample, Particle, SoftBody};
use gz_sim_scene::{WorldPose, deformable::set_material_resolution};
use serde::Serialize;
use std::time::Duration;

const MATCH: usize = 1650;
const HOLD: usize = 300;
const STRIDE: usize = 25;
const DT: f64 = 0.001;
const ACROSS: usize = 4;
const HIGH: usize = 2;

#[derive(Clone, Copy, Serialize)]
struct Site {
    panel: usize,
    width_m: f64,
    height_m: f64,
    stiffness_weight: f64,
}
fn sites(half_width: f64) -> Vec<Site> {
    let height = arm_sheet::seed(3, 0.).z - arm_sheet::seed(0, 0.).z;
    let mut sites = vec![];
    for panel in 0..2 {
        for z in 0..HIGH {
            for y in 0..ACROSS {
                sites.push(Site {
                    panel,
                    width_m: half_width * ((2 * y + 1) as f64 / ACROSS as f64 - 1.),
                    height_m: height * (z as f64 + 0.5) / HIGH as f64,
                    stiffness_weight: 2. / (ACROSS * HIGH) as f64,
                });
            }
        }
    }
    sites
}

impl Site {
    fn stencil(self, columns: usize) -> PadSample {
        let lo = arm_sheet::seed(0, 0.);
        let hi = arm_sheet::seed(5, 0.);
        let u = (self.width_m - lo.y) / (hi.y - lo.y) * (columns - 1) as f64;
        let height_fraction = self.height_m / (hi.z - lo.z);
        // The right panel is numbered from the crown down towards its free hem.
        let v = if self.panel == 0 {
            height_fraction
        } else {
            1. - height_fraction
        };
        let a = 2 * self.panel * columns + u.floor() as usize;
        let u = u.fract();
        let d = a + columns + 1;
        let (particles, weights) = if u >= v {
            ([a, a + 1, d], [1. - u, u - v, v])
        } else {
            ([a, d, a + columns], [1. - v, u, v - u])
        };
        PadSample {
            particles,
            weights,
            stiffness_weight: self.stiffness_weight,
        }
    }
}

pub(super) fn material_samples(half_width: f64, columns: usize) -> Vec<PadSample> {
    sites(half_width)
        .into_iter()
        .map(|s| s.stencil(columns))
        .collect()
}

// All cloth vertices lie on the interior (gap) side of each flat inner jaw
// plane. Their minimum signed distance is a conservative core clearance for
// whole triangles, including the unsampled crown and the outer side edges.
fn core_clearance(p: &[Particle], jaws: [BodyState; 2], half: [DVec3; 2]) -> f64 {
    let mut gap = f64::INFINITY;
    for j in 0..2 {
        let to_local = DQuat::from_array(jaws[j].orientation).conjugate();
        let inward = if j == 0 { 1. } else { -1. };
        for node in p {
            let relative = DVec3::from(node.position) - DVec3::from(jaws[j].position);
            gap = gap.min(inward * (to_local * relative).x - half[j].x);
        }
    }
    gap
}

#[derive(Serialize)]
struct Observation {
    time_ns: u128,
    positions: Vec<[f64; 3]>,
}
struct Branch {
    body: SoftBody,
    model: &'static str,
    columns: usize,
    compliance: [f64; 2],
    initial_gap: f64,
    min_core: f64,
    minimum_gap: f64,
    pad_panel_ns: [[f64; 2]; 2],
    max_pad_residual_ns: f64,
    loads: Loads,
    cross_forces: Vec<f64>,
    times: Vec<u64>,
    refusal: Option<String>,
    frames: Vec<(String, String)>,
    observations: Vec<Observation>,
}
impl Branch {
    fn new(anchor: &Anchor, columns: usize, jaws: [BodyState; 2], patch: bool) -> Result<Self> {
        let mut body = fold_mesh::mesh(anchor, columns)?;
        let coarse = fold_mesh::face_nodes(&anchor.loaded, 3, &jaws, anchor.half);
        let fine = fold_mesh::face_nodes(body.particles(), columns, &jaws, anchor.half);
        if coarse != [2; 2] || fine.contains(&0) || anchor.half[0].y != anchor.half[1].y {
            return Err("expected two occupied coarse nodes per jaw and equal width pads".into());
        }
        let compliance = if patch {
            let stencils = material_samples(anchor.half[0].y, columns);
            body = body.with_pad_samples(stencils)?;
            [PAD_COMPLIANCE; 2]
        } else {
            std::array::from_fn(|j| PAD_COMPLIANCE * fine[j] as f64 / coarse[j] as f64)
        };
        let initial_gap = fold_mesh::panel_gap(body.particles(), columns)?;
        let min_core = core_clearance(body.particles(), jaws, anchor.half);
        Ok(Self {
            body,
            model: if patch { "patch" } else { "particle" },
            columns,
            compliance,
            initial_gap,
            min_core,
            minimum_gap: f64::INFINITY,
            pad_panel_ns: [[0.; 2]; 2],
            max_pad_residual_ns: 0.,
            loads: Loads::default(),
            cross_forces: vec![],
            times: vec![],
            refusal: None,
            frames: vec![],
            observations: vec![],
        })
    }
    fn advance(&mut self, half: [DVec3; 2], jaws: &[[BodyState; 2]; 2]) -> Result<()> {
        if self.refusal.is_some() {
            return Ok(());
        }
        let start = Instant::now();
        let status = fold_mesh::advance(&mut self.body, half, self.compliance, jaws);
        let elapsed_ns = start.elapsed().as_nanos() as u64;
        if let Err(e) = status {
            self.refusal = Some(e.to_string());
            return Ok(());
        }
        self.times.push(elapsed_ns);
        let points = self.body.particles();
        self.min_core = self.min_core.min(core_clearance(points, jaws[1], half));
        let gap = fold_mesh::panel_gap(points, self.columns)?;
        self.minimum_gap = self.minimum_gap.min(gap);
        self.loads.mean_closest_panel_skin_gap_m += gap;
        self.cross_forces
            .push(self.loads.add(&self.body, self.columns, &jaws[1]) / DT);
        let mut wrench = [DVec3::ZERO; 2];
        for pad in self.body.last_pad_loads() {
            let j = pad.boundary;
            self.pad_panel_ns[j][pad.particle / (2 * self.columns)] +=
                DVec3::from(pad.normal).length();
            wrench[j] += DVec3::from(pad.normal) + DVec3::from(pad.tangent);
        }
        for (sum, reaction) in wrench.into_iter().zip(self.body.last_boundary_impulses()) {
            self.max_pad_residual_ns = self
                .max_pad_residual_ns
                .max(sum.distance(reaction.linear.into()));
        }
        Ok(())
    }
    fn capture(&mut self, r: &mut Recording, marker_ids: &[u64]) -> Result<()> {
        for (&id, particle) in marker_ids.iter().zip(self.body.particles()) {
            let e = entity_by_id(r.sim.world(), id).ok_or("missing sheet marker")?;
            r.sim
                .world_mut()
                .get_mut::<WorldPose>(e)
                .ok_or("missing sheet visual pose")?
                .translation = particle.position;
        }
        self.observations.push(Observation {
            time_ns: r.sim.elapsed().as_nanos(),
            positions: self.body.particles().iter().map(|p| p.position).collect(),
        });
        self.frames.push(gz_sim_cli::primitive_frame(
            &r.sim,
            &r.sdf,
            &r.world,
            self.frames.len() as u64 + 1,
        )?);
        Ok(())
    }
    fn save(mut self, out: &Path) -> Result<Value> {
        let dir = out.join(format!("{}-{}", self.model, self.columns));
        fs::create_dir_all(&dir)?;
        let last = self.frames.len() - 1;
        let captured_ns = self.observations[last].time_ns;
        let committed_ns = (MATCH + self.times.len()) as u128 * 1_000_000;
        gz_sim_cli::export_archive_at(
            &dir.join("replay"),
            Default::default(),
            0,
            Duration::from_millis(MATCH as u64),
            self.frames,
        )?;
        write_json(
            &dir.join("particle-frames.json"),
            &serde_json::to_value(self.observations)?,
        )?;
        write_json(
            &dir.join("frames.json"),
            &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"source_public_dt_ns":1_000_000,"capture_stride_ticks":STRIDE,
            "last_committed_ns":committed_ns,"last_captured_ns":captured_ns,
            "particle_visuals":(0..4*self.columns).map(arm_sheet::name).collect::<Vec<_>>(),
            "jaw_visuals":["default::arm::left::left","default::arm::right::right"],
            "cloth_grid":{"rows":4,"columns":self.columns,"skin_radius_m":RADIUS,"last_row_left":1,"first_row_right":2},
            "captures":{"matched":0,"hold":last/2,"end":last}}),
        )?;
        write_json(
            &dir.join("cross-panel-force.json"),
            &json!({"matched_ms":MATCH,"dt_s":DT,
            "cross_panel_n":self.cross_forces}),
        )?;
        self.times.sort_unstable();
        let ticks = self.times.len();
        let mut row = json!({"model":self.model,"columns":self.columns,"completed_ticks":ticks,
            "status":if self.refusal.is_some() {"refused"} else {"completed"},"refusal":self.refusal,
            "initial_panel_skin_gap_m":self.initial_gap,"min_mid_surface_jaw_core_clearance_m":self.min_core,
            "min_held_panel_skin_gap_m":(ticks>0).then_some(self.minimum_gap),
            "first_cross_panel_n":self.cross_forces.first(),
            "ticks_with_cross_panel_impulse":self.cross_forces.iter().filter(|&&n|n>0.).count(),
            "last_cross_panel_tick":self.cross_forces.iter().rposition(|&n|n>0.).map(|i|i+1),
            "peak_cross_panel_n":self.cross_forces.iter().copied().reduce(f64::max),"max_pad_attribution_residual_ns":self.max_pad_residual_ns,"pad_compliance_m_per_n":self.compliance});
        if ticks > 0 {
            let tail = &self.cross_forces[ticks.saturating_sub(100)..];
            row["trailing_window_ticks"] = tail.len().into();
            row["trailing_mean_cross_panel_n"] =
                (tail.iter().sum::<f64>() / tail.len() as f64).into();
            self.loads.mean_closest_panel_skin_gap_m /= ticks as f64;
            self.loads.average(ticks as f64 * DT);
            row.as_object_mut().unwrap().extend(
                serde_json::to_value(self.loads)?
                    .as_object()
                    .unwrap()
                    .clone(),
            );
            row["mean_pad_normal_by_jaw_panel_n"] = json!(
                self.pad_panel_ns
                    .map(|p| p.map(|v| v / (ticks as f64 * DT)))
            );
            row["step_p95_ns"] = self.times[(ticks * 95).div_ceil(100) - 1].into();
        }
        Ok(row)
    }
}

#[derive(PartialEq)]
struct Drive {
    loaded: Vec<Particle>,
    path: Vec<[BodyState; 2]>,
}
fn pair(columns: usize, ticks: usize, out: &Path) -> Result<(Drive, [Value; 2], f64)> {
    let scene = arm_sheet::scene_with_marker_grid(
        arm::SCENE,
        arm_sheet::Case {
            substeps: 8,
            ..Default::default()
        },
        0.,
        true,
        columns,
    )?;
    let mut r = Recording::new(&scene, "fold_patch.sdf", "default")?;
    let controller = arm::Controller::new(&r.sim)?;
    let jaws = [
        entity_id(&r.sim, "default::arm::left")?,
        entity_id(&r.sim, "default::arm::right")?,
    ];
    let jaw_state =
        |s: &Simulation| -> Result<[BodyState; 2]> { Ok([state(s, jaws[0])?, state(s, jaws[1])?]) };
    let marker_ids: Vec<_> = (0..4 * columns)
        .map(|i| entity_id(&r.sim, &arm_sheet::name(i)))
        .collect::<Result<_>>()?;
    for tick in 1..=MATCH {
        fold_hold::drive(&mut r, &controller, tick)?;
    }
    let anchor = Anchor::capture(&r.sim)?;
    set_material_resolution(
        &mut r.sim,
        "sheet",
        fold_mesh::SUBSTEPS,
        fold_mesh::ITERATIONS,
    )?;
    let mut path = vec![jaw_state(&r.sim)?];
    let mut branches = [
        Branch::new(&anchor, columns, path[0], false)?,
        Branch::new(&anchor, columns, path[0], true)?,
    ];
    for branch in &mut branches {
        branch.capture(&mut r, &marker_ids)?;
    }
    for tick in 1..=ticks {
        fold_hold::drive(&mut r, &controller, MATCH + tick)?;
        path.push(jaw_state(&r.sim)?);
        for branch in &mut branches {
            branch.advance(anchor.half, &[path[tick - 1], path[tick]])?;
            if (tick % STRIDE == 0 || tick == ticks) && branch.times.len() == tick {
                branch.capture(&mut r, &marker_ids)?;
            }
        }
    }
    let [particle, patch] = branches;
    Ok((
        Drive {
            loaded: anchor.loaded,
            path,
        },
        [particle.save(out)?, patch.save(out)?],
        anchor.half[0].y,
    ))
}

pub(super) fn run(out: &Path) -> Result<()> {
    run_hold(out, HOLD)
}

fn run_hold(out: &Path, ticks: usize) -> Result<()> {
    let mut trials = vec![];
    let mut drive = None;
    let mut half_width = 0.;
    for columns in [3, 5, 7] {
        let (reference, pair, width) = pair(columns, ticks, out)?;
        if let Some(expected) = &drive {
            if *expected != reference {
                return Err("loaded particles or recorded jaw states differ between meshes".into());
            }
        } else {
            drive = Some(reference);
        }
        trials.extend(pair);
        half_width = width;
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"start_ms":MATCH,"ticks":ticks,"dt_s":DT,
        "material_substeps":fold_mesh::SUBSTEPS,"solver_iterations":fold_mesh::ITERATIONS,"weight_n":0.0084*9.81,
        "drive":"Identical loaded particle states and every native jaw endpoint state checked across the three presentation grids. Both contact laws start with cold material histories and use the same prescribed jaw poses.",
        "quadrature":"Material-attached midpoint cells on the two vertical panels over the flat jaw width and full panel height. Each panel has total stiffness weight two. Both cuboids test every point. The crown and rounded shoulders are outside the prescribed patch.",
        "patch_width_m":2.*half_width,"patch_height_m":arm_sheet::seed(3,0.).z-arm_sheet::seed(0,0.).z,
        "sites":sites(half_width),"trials":trials}),
    )
}

#[cfg(test)]
#[path = "fold_patch_tests.rs"]
mod tests;
