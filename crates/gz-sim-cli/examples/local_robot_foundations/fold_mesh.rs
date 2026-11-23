// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cold-start both meshes from the loaded cloth and drive them with the same measured jaw path.
use super::*;
use crate::arm_sheet::{self, BENCH, BEND, PAD_COMPLIANCE, RADIUS, SHEAR, STRETCH};
use glam::DQuat;
use gz_physics_xpbd::{
    Boundary, Particle, RigidMobility, SoftBody, SolverConfig, step_coupled_rigid,
};
use parry3d_f64::{
    math::{Pose, Vector},
    query::distance,
    shape::Triangle,
};
use serde::Serialize;
use std::time::Duration;

const ROWS: usize = 4;
const COARSE: usize = 3;
const DT: f64 = 0.001;
pub(super) const SUBSTEPS: u32 = 64;
pub(super) const ITERATIONS: u32 = 48;

pub(super) struct Anchor {
    pub(super) loaded: Vec<Particle>,
    pub(super) half: [DVec3; 2],
}
impl Anchor {
    pub(super) fn capture(sim: &Simulation) -> Result<Self> {
        let loaded = arm_sheet::cloth(sim)?.particles().to_vec();
        let sdf = sdformat::load_str(arm::SCENE, "arm.sdf", &sdformat::NoResources)?;
        Ok(Self {
            loaded,
            half: [
                arm_cable::half_size(&sdf, "arm", "left")?,
                arm_cable::half_size(&sdf, "arm", "right")?,
            ],
        })
    }
}

// The transpose of linear interpolation sends these masses back to exactly
// the original masses at all three coarse columns, including the free edges.
fn width_mass(column: usize, columns: usize) -> f64 {
    let stride = (columns - 1) as f64 / (COARSE - 1) as f64;
    if column == 0 || column + 1 == columns {
        (stride + 1.) / (2. * stride)
    } else {
        1. / stride
    }
}

fn resample(coarse: &[Particle], columns: usize) -> Vec<Particle> {
    let stride = (columns - 1) / (COARSE - 1);
    (0..ROWS * columns)
        .map(|i| {
            let col = i % columns;
            let left = (i / columns) * COARSE + col / stride;
            let a = coarse[left];
            let mut p = a;
            if !col.is_multiple_of(stride) {
                let b = coarse[left + 1];
                let fraction = (col % stride) as f64 / stride as f64;
                p.position = DVec3::from(a.position)
                    .lerp(b.position.into(), fraction)
                    .to_array();
                p.velocity = DVec3::from(a.velocity)
                    .lerp(b.velocity.into(), fraction)
                    .to_array();
            }
            p.inverse_mass /= width_mass(col, columns);
            p
        })
        .collect()
}

fn rest_grid(loaded: &[Particle], columns: usize) -> Vec<Particle> {
    let coarse: Vec<_> = loaded
        .iter()
        .enumerate()
        .map(|(i, p)| Particle {
            position: arm_sheet::seed(i, 0.).to_array(),
            velocity: [0.; 3],
            ..*p
        })
        .collect();
    resample(&coarse, columns)
}

fn edge_law(a: usize, b: usize, columns: usize) -> f64 {
    let ratio = (columns - 1) as f64 / (COARSE - 1) as f64;
    if a / columns == b / columns {
        return STRETCH / ratio;
    } // springs in series
    if a % columns == b % columns {
        return STRETCH / width_mass(a % columns, columns);
    } // parallel strips
    // Calibrate the diagonal springs to the same energy under infinitesimal
    // simple shear. The change of triangle aspect ratio changes their angle.
    let row = a / columns;
    let across =
        arm_sheet::seed(row * COARSE, 0.).distance(arm_sheet::seed((row + 1) * COARSE, 0.));
    let width = arm_sheet::seed(0, 0.).distance(arm_sheet::seed(1, 0.));
    let h = width / ratio;
    SHEAR * (width * width + across * across) / (ratio * (h * h + across * across))
}

fn hinge_law(edge: [usize; 4], columns: usize) -> f64 {
    // For a local plate hinge, angular compliance scales with the dual width
    // divided by the shared edge length (area of the two faces / length²).
    let [a, b, _, _] = edge;
    let ratio = (columns - 1) as f64 / (COARSE - 1) as f64;
    let width = arm_sheet::seed(0, 0.).distance(arm_sheet::seed(1, 0.));
    let row = (a / columns).min(b / columns);
    let along = if a / columns == b / columns {
        0.
    } else {
        arm_sheet::seed(row * COARSE, 0.).distance(arm_sheet::seed((row + 1) * COARSE, 0.))
    };
    let cross = if a % columns == b % columns {
        0.
    } else {
        width
    };
    BEND / ratio * (cross.powi(2) + along.powi(2)) / ((cross / ratio).powi(2) + along.powi(2))
}

pub(super) fn mesh(anchor: &Anchor, columns: usize) -> Result<SoftBody> {
    let loaded = resample(&anchor.loaded, columns);
    let rest = rest_grid(&anchor.loaded, columns);
    let cfg = SolverConfig {
        gravity: [0., 0., -9.81],
        substeps: SUBSTEPS,
        iterations: ITERATIONS,
        particle_radius: RADIUS,
        self_contact: true,
        friction: 0.7,
    };
    Ok(SoftBody::cloth_from_rest(
        loaded,
        &rest,
        columns,
        cfg,
        |a, b| edge_law(a, b, columns),
        |v| hinge_law(v, columns),
    )?
    .with_self_contact_friction(0.3)?
    .with_self_contact_loads()?
    .with_pad_loads())
}

pub(super) fn face_nodes(
    particles: &[Particle],
    columns: usize,
    jaws: &[BodyState; 2],
    half: [DVec3; 2],
) -> [usize; 2] {
    std::array::from_fn(|j| {
        let to_pad = DQuat::from_array(jaws[j].orientation).conjugate();
        particles
            .iter()
            .enumerate()
            .filter(|(i, p)| {
                if (*i / columns) / (ROWS / 2) != j {
                    return false;
                }
                let local = to_pad * (DVec3::from(p.position) - DVec3::from(jaws[j].position));
                local.y.abs() <= half[j].y && local.z.abs() <= half[j].z
            })
            .count()
    })
}
pub(super) fn advance(
    body: &mut SoftBody,
    half: [DVec3; 2],
    compliance: [f64; 2],
    jaws: &[[BodyState; 2]],
) -> std::result::Result<(), gz_physics_xpbd::XpbdError> {
    let mut boundaries = [Boundary::Plane {
        point: [0., 0., BENCH],
        normal: [0., 0., 1.],
        velocity: [0.; 3],
    }; 3];
    let mut mobility = [None; 3];
    for i in 0..2 {
        let from = jaws[0][i];
        let to = jaws[1][i];
        boundaries[i] = Boundary::Cuboid {
            center: from.position,
            half_extents: half[i].to_array(),
            velocity: ((DVec3::from(to.position) - DVec3::from(from.position)) / DT).to_array(),
            angular_velocity: to.angular_velocity,
        };
        mobility[i] = Some(RigidMobility::PrescribedCuboid {
            normal_compliance: compliance[i],
            start_orientation: from.orientation,
            end_orientation: to.orientation,
            start_angular_velocity: from.angular_velocity,
        });
    }
    step_coupled_rigid(
        &mut [body],
        Duration::from_millis(1),
        &boundaries,
        &mobility,
    )?;
    Ok(())
}

// Closest surface-to-surface clearance between the two panel meshes,
// excluding the two rows' shared crown strip. Triangles span each complete
// panel, including its free edges. This is sampled at 1 ms endpoints.
pub(super) fn panel_gap(points: &[Particle], columns: usize) -> Result<f64> {
    let mut minimum = f64::INFINITY;
    let panel = |row: usize| {
        let mut faces = Vec::with_capacity(2 * (columns - 1));
        for col in 1..columns {
            let br = (row + 1) * columns + col;
            let tl = br - columns - 1;
            for indices in [[tl, br - columns, br], [tl, br, br - 1]] {
                let [a, b, c] = indices.map(|i| Vector::from_array(points[i].position));
                faces.push(Triangle::new(a, b, c));
            }
        }
        faces
    };
    let right = panel(2);
    for a in panel(0) {
        for b in &right {
            minimum = minimum.min(distance(&Pose::IDENTITY, &a, &Pose::IDENTITY, b)?.distance);
        }
    }
    Ok(minimum - 2. * RADIUS)
}
#[derive(Serialize, Default)]
pub(super) struct Loads {
    pub(super) mean_self_contact_n: f64,
    pub(super) mean_closest_panel_skin_gap_m: f64,
    pub(super) mean_cross_panel_n: f64,
    cross_panel_width_n: [f64; 3],
    pub(super) mean_jaw_squeeze_n: [f64; 2],
    pub(super) mean_jaw_up_n: [f64; 2],
    pub(super) mean_bench_up_n: f64,
}
impl Loads {
    pub(super) fn add(
        &mut self,
        body: &SoftBody,
        columns: usize,
        orientations: &[BodyState; 2],
    ) -> f64 {
        let mut cross_impulse = 0.;
        self.mean_self_contact_n += body.last_self_contact_impulse().normal;
        for load in body.last_self_contact_loads() {
            let (side_a, side_b) = load.particles.split_at(load.first_side_len);
            let (weight_a, weight_b) = load.particle_normal_ns.split_at(load.first_side_len);
            let panel_a = super::fold_hold::side_panel(side_a, weight_a, columns);
            let panel_b = super::fold_hold::side_panel(side_b, weight_b, columns);
            if !matches!((panel_a, panel_b), (Some(0), Some(1)) | (Some(1), Some(0))) {
                continue;
            }
            self.mean_cross_panel_n += load.normal_ns;
            cross_impulse += load.normal_ns;
            for (&i, &n) in load.particles.iter().zip(&load.particle_normal_ns) {
                // Both incident sides carry the same contact once. Distribute
                // the nodal shares onto the three original material columns.
                let u = (i % columns) as f64 * 2. / (columns - 1) as f64;
                let k = u.floor() as usize;
                let t = u.fract();
                self.cross_panel_width_n[k] += 0.5 * n * (1. - t);
                self.cross_panel_width_n[(k + 1).min(2)] += 0.5 * n * t;
            }
        }
        let reactions = body.last_boundary_impulses();
        for j in 0..2 {
            let impulse = DVec3::from(reactions[j].linear);
            let local = DQuat::from_array(orientations[j].orientation).conjugate() * impulse;
            self.mean_jaw_squeeze_n[j] += if j == 0 { -local.x } else { local.x };
            self.mean_jaw_up_n[j] -= impulse.z;
        }
        self.mean_bench_up_n -= reactions[2].linear[2];
        cross_impulse
    }
    pub(super) fn average(&mut self, seconds: f64) {
        self.mean_self_contact_n /= seconds;
        self.mean_cross_panel_n /= seconds;
        self.mean_bench_up_n /= seconds;
        for f in self
            .cross_panel_width_n
            .iter_mut()
            .chain(&mut self.mean_jaw_squeeze_n)
            .chain(&mut self.mean_jaw_up_n)
        {
            *f /= seconds;
        }
    }
}
#[derive(Serialize)]
struct Trial {
    columns: usize,
    spacing_m: f64,
    face_nodes: [usize; 2],
    pad_compliance_m_per_n: [f64; 2],
    status: &'static str,
    completed_ticks: usize,
    refusal: Option<String>,
    mass_error_kg: f64,
    center_of_mass_error_m: f64,
    linear_momentum_error_ns: f64,
    angular_momentum_error_nms: f64,
    max_matched_node_position_error_m: f64,
    max_matched_node_velocity_error_m_s: f64,
    #[serde(flatten)]
    loads: Option<Loads>,
    initial_panel_skin_gap_m: f64,
    min_held_panel_skin_gap_m: Option<f64>,
}
// Angular momentum here is about the common inertial world origin.
fn moments(particles: &[Particle]) -> (f64, DVec3, DVec3, DVec3) {
    let mut m = 0.;
    let mut q = DVec3::ZERO;
    let mut p = DVec3::ZERO;
    let mut l = DVec3::ZERO;
    for node in particles {
        let mass = 1. / node.inverse_mass;
        let x = DVec3::from(node.position);
        let mv = mass * DVec3::from(node.velocity);
        m += mass;
        q += mass * x;
        p += mv;
        l += x.cross(mv);
    }
    (m, q / m, p, l)
}
fn trial(anchor: &Anchor, columns: usize, path: &[[BodyState; 2]]) -> Result<Trial> {
    let mut body = mesh(anchor, columns)?;
    let coarse = face_nodes(&anchor.loaded, COARSE, &path[0], anchor.half);
    let fine = face_nodes(body.particles(), columns, &path[0], anchor.half);
    if coarse.contains(&0) || fine.contains(&0) {
        return Err("jaw footprint contains no material points".into());
    }
    let compliance = std::array::from_fn(|j| PAD_COMPLIANCE * fine[j] as f64 / coarse[j] as f64);
    let orig = moments(&anchor.loaded);
    let posed = moments(body.particles());
    let mut position_error: f64 = 0.;
    let mut velocity_error: f64 = 0.;
    for (i, p) in anchor.loaded.iter().enumerate() {
        let refined = body.particles()
            [(i / COARSE) * columns + (i % COARSE) * ((columns - 1) / (COARSE - 1))];
        position_error =
            position_error.max(DVec3::from(p.position).distance(refined.position.into()));
        velocity_error =
            velocity_error.max(DVec3::from(p.velocity).distance(refined.velocity.into()));
    }
    let initial_panel_skin_gap_m = panel_gap(body.particles(), columns)?;
    let mut min_gap = f64::INFINITY;
    let mut sums = Loads::default();
    let mut refusal = None;
    let mut ticks = 0;
    for pair in path.windows(2) {
        if let Err(e) = advance(&mut body, anchor.half, compliance, pair) {
            refusal = Some(e.to_string());
            break;
        }
        let gap = panel_gap(body.particles(), columns)?;
        min_gap = min_gap.min(gap);
        sums.mean_closest_panel_skin_gap_m += gap;
        sums.add(&body, columns, &pair[1]);
        ticks += 1;
    }
    let loads = (ticks > 0).then(|| {
        sums.mean_closest_panel_skin_gap_m /= ticks as f64;
        sums.average(ticks as f64 * DT);
        sums
    });
    Ok(Trial {
        columns,
        spacing_m: arm_sheet::seed(0, 0.).distance(arm_sheet::seed(1, 0.)) * 2.
            / (columns - 1) as f64,
        face_nodes: fine,
        pad_compliance_m_per_n: compliance,
        status: if refusal.is_some() {
            "refused"
        } else {
            "completed"
        },
        completed_ticks: ticks,
        refusal,
        mass_error_kg: (orig.0 - posed.0).abs(),
        center_of_mass_error_m: orig.1.distance(posed.1),
        linear_momentum_error_ns: orig.2.distance(posed.2),
        angular_momentum_error_nms: orig.3.distance(posed.3),
        max_matched_node_position_error_m: position_error,
        max_matched_node_velocity_error_m_s: velocity_error,
        loads,
        initial_panel_skin_gap_m,
        min_held_panel_skin_gap_m: (ticks > 0).then_some(min_gap),
    })
}
pub(super) fn run(anchor: Anchor, path: &[[BodyState; 2]], out: &Path) -> Result<()> {
    if path.len() < 2
        || anchor.loaded.len() != COARSE * ROWS
        || anchor
            .loaded
            .iter()
            .any(|p| p.inverse_mass != anchor.loaded[0].inverse_mass || p.inverse_mass <= 0.)
    {
        return Err(
            "expected a sampled jaw path and twelve equally weighted free sheet particles".into(),
        );
    }
    let trials = [
        trial(&anchor, 3, path)?,
        trial(&anchor, 5, path)?,
        trial(&anchor, 7, path)?,
    ];
    fs::create_dir_all(out)?;
    write_json(
        &out.join("comparison.json"),
        &json!({"start_ms":1650,"dt_s":DT,
        "material_substeps":SUBSTEPS,"solver_iterations":ITERATIONS,"ticks":path.len()-1,
        "drive":"Identical recorded jaw endpoint trajectory on all meshes; cold material contact histories.",
        "calibration":"Total flat-face pad stiffness for equal indentation, axial stretch, simple-shear energy and hinge dual-width/edge-length scaling.",
        "trials":trials}),
    )
}
#[cfg(test)]
#[path = "fold_mesh_tests.rs"]
mod tests;
