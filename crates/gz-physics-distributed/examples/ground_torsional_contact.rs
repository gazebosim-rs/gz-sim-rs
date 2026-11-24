// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Yaw braking against a rough shared halfspace, measured on actual native worker images.
#[allow(dead_code)]
#[path = "support/native_contact_control.rs"]
mod native;
use clap::Parser;
use gz_physics::DynamicBackend;
use gz_physics_distributed::{
    physics::{Body, GroundContactPatch, GroundPlane, Primitive, SimulationStep, Tier},
    pool::{PoolNativeRecovery, WorkerPool},
};
use gz_physics_multifidelity::GroundPatchPoint;
use rapier3d_f64::prelude::{Rotation, Vector};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const MASS: f64 = 2.;
const KICK: f64 = 0.96;
const ID: u64 = 27;
const WIDTH: usize = 4;
#[derive(Clone, Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    tilted: bool,
    #[arg(long)]
    rectangle: bool,
    #[arg(long)]
    reverse: bool,
    #[arg(long)]
    frictionless: bool,
    #[arg(long)]
    coarse: bool,
    #[arg(long)]
    packed: bool,
    #[arg(long)]
    frames: bool,
    /// Qualify both plane orientations, box shapes, spin signs, friction regimes,
    /// solve policies and handoff encodings; report every matched trajectory.
    #[arg(long, conflicts_with_all = ["tilted","rectangle","reverse","frictionless","coarse","packed"])]
    matrix: bool,
    out: PathBuf,
}
struct Case {
    body: Body,
    plane: GroundPlane,
    normal: Vector,
    gravity: [f64; 3],
    radius: f64,
    mu: f64,
}
impl Case {
    fn new(a: &Args) -> Result<Self> {
        let n = if a.tilted {
            Vector::new(0., -0.6, 0.8)
        } else {
            Vector::Z
        };
        let rot = Rotation::from_rotation_x(if a.tilted { 0.6_f64.atan2(0.8) } else { 0. });
        let origin = Vector::new(2.25, -1.75, 0.85);
        let center = origin + rot * Vector::new(0.65, -0.4, 0.15);
        let size: [f64; 3] = if a.rectangle {
            [0.8, 0.4, 0.2]
        } else {
            [0.6, 0.6, 0.2]
        };
        let mu = if a.frictionless { 0. } else { 0.35 };
        let body = Body {
            id: ID,
            primitive: Primitive::Box,
            tier: Tier::Dynamic,
            size,
            mass: MASS,
            friction: mu,
            position: center.to_array(),
            orientation: [rot.x, rot.y, rot.z, rot.w],
            linear: [0.; 3],
            angular: [0.; 3],
        };
        let plane = GroundPlane::oriented(origin.to_array(), n.to_array(), 0.65)?;
        Ok(Self {
            body,
            plane,
            normal: n,
            gravity: (-9.81 * n).to_array(),
            radius: 0.5 * size[0].hypot(size[1]),
            mu,
        })
    }
    fn spin(&self, b: &Body) -> f64 {
        let [x, y, z] = b.size;
        let [qx, qy, qz, qw] = b.orientation;
        let q = Rotation::from_xyzw(qx, qy, qz, qw);
        let moments = Vector::new(y * y + z * z, x * x + z * z, x * x + y * y) * (b.mass / 12.);
        self.normal
            .dot(q * (moments * (q.inverse() * Vector::from(b.angular))))
    }
    fn omega(&self, b: &Body) -> f64 {
        self.normal.dot(Vector::from(b.angular))
    }
    fn tangent(&self, x: Vector) -> Vector {
        x - self.normal * self.normal.dot(x)
    }
}
#[derive(Default)]
struct Geometry {
    max_normal: f64,
    max_plane: f64,
    max_midpoint_tangent: f64,
    max_radius: f64,
    max_negative_load: f64,
    four_point_ticks: usize,
    points: usize,
}
impl Geometry {
    fn observe(&mut self, case: &Case, patch: &GroundContactPatch) -> (usize, f64, f64) {
        let n = Vector::from(patch.normal_on_body);
        self.max_normal = self.max_normal.max((n - case.normal).length());
        let count = patch.points.len();
        if count == 0 {
            return (0, 0., 0.);
        }
        let center = patch
            .points
            .iter()
            .map(|p| Vector::from(p.plane_witness))
            .sum::<Vector>()
            / (count as f64);
        let mut normal = 0.;
        let mut twist_cap = 0.;
        for point in &patch.points {
            let witness = Vector::from(point.plane_witness);
            let solver = Vector::from(point.solver_point);
            self.max_plane = self.max_plane.max(
                (witness - Vector::from(case.plane.point()))
                    .dot(case.normal)
                    .abs(),
            );
            self.max_midpoint_tangent = self
                .max_midpoint_tangent
                .max(case.tangent(solver - witness).length());
            let radius = case.tangent(witness - center).length();
            if count == 4 {
                self.max_radius = self.max_radius.max((radius - case.radius).abs());
            }
            self.max_negative_load = self.max_negative_load.max(-point.normal_impulse);
            normal += point.normal_impulse;
            // The normal exerts no axial moment. The Coulomb bound about the
            // body's center of mass uses the actual world-space solver lever.
            let lever = case.tangent(Vector::from(point.body_lever)).length();
            twist_cap += case.mu * lever * point.normal_impulse;
        }
        self.four_point_ticks += usize::from(count == 4);
        self.points += count;
        (count, normal, twist_cap)
    }
}
fn snapshot(
    p: &mut WorkerPool,
    path: &Path,
    expected: &[GroundContactPatch],
) -> Result<Vec<GroundContactPatch>> {
    p.save_native_recovery(path)?;
    let image = PoolNativeRecovery::open(path)?;
    let rows = image.ground_contact_patches()?;
    let owner = usize::from(p.frame().body(ID).ok_or("missing moving ID")?.0);
    if rows.len() != WIDTH
        || rows
            .iter()
            .enumerate()
            .any(|(i, r)| i != owner && !r.is_empty())
        || bincode::serialize(&rows[owner])? != bincode::serialize(expected)?
        || !image.finite_contact_patches()?.iter().all(Vec::is_empty)
    {
        return Err(format!(
            "ground patch or owner differs at {} {}",
            p.frame().tick,
            path.display()
        )
        .into());
    }
    Ok(rows.into_iter().nth(owner).unwrap())
}
fn loaded(points: &[GroundPatchPoint]) -> usize {
    points.iter().filter(|p| p.normal_impulse > 0.).count()
}
fn run(a: &Args) -> Result<()> {
    fs::create_dir(&a.out)?;
    let case = Case::new(a)?;
    let mut control = native::Control::new(
        std::slice::from_ref(&case.body),
        Some(case.plane),
        case.gravity,
    )?;
    let mut p = WorkerPool::spawn(
        &a.worker,
        0x47726f75,
        vec![vec![case.body.clone()], vec![], vec![], vec![]],
        SimulationStep::Ms10,
        Some(case.plane),
        128,
    )?;
    let configure = |p: &mut WorkerPool| {
        p.set_packed_observations(a.packed);
        p.set_packed_handoffs(a.packed);
    };
    configure(&mut p);
    let scratch = tempfile::tempdir_in(&a.out)?;
    if a.frames {
        let cold_patch = snapshot(&mut p, &scratch.path().join("cold.gzpool"), &[])?;
        let (owner, body) = p.frame().body(ID).ok_or("missing seed")?;
        if body != &case.body {
            return Err("native bootstrap differs from authored geometry".into());
        }
        let initial = json!({"tick":0,"dt_s":0.01,"body":body,"owner":owner,"ground":case.plane,"patches":cold_patch,
            "public_normal_ns":0.,"last_native_normal_ns":0.,"angular_speed_rad_s":case.omega(body),
            "events":[]});
        serde_json::to_writer_pretty(File::create_new(a.out.join("initial.json"))?, &initial)?;
    }
    let mut trace = a
        .frames
        .then(|| File::create_new(a.out.join("frames.jsonl")))
        .transpose()?
        .map(BufWriter::new);
    let sign = if a.reverse { -1. } else { 1. };
    let mut geom = Geometry::default();
    let mut rate_hist = BTreeMap::new();
    let mut count_hist = BTreeMap::new();
    let mut positive_events = Vec::new();
    let mut errors = vec![];
    let mut digest = Sha256::new();
    let mut patches_digest = Sha256::new();
    let mut max_sum_error = 0f64;
    let mut max_support_error = 0f64;
    let mut max_steady_tangent = 0f64;
    let mut max_reaction_drift = 0f64;
    let mut max_lever_com_error = 0f64;
    let mut settled = 0;
    let mut stop = None;
    let mut initial = 0.;
    let mut terminal = 0.;
    let mut total_reaction = 0.;
    let mut inspected = 0;
    for t in 1..=120u64 {
        if t == 31 {
            let impulse = (case.normal * (sign * KICK)).to_array();
            let h0 = case.spin(p.frame().body(ID).unwrap().1);
            p.apply_impulse_in_field(ID, [0.; 3], impulse, case.gravity, &[])?;
            control
                .solver
                .apply_impulse(control.handles[&ID], [0.; 3], impulse)?;
            let b = p.frame().body(ID).unwrap().1;
            if (case.spin(b) - h0 - sign * KICK).abs() > 1e-9 {
                return Err("angular impulse admission failed inertia balance".into());
            }
            initial = case.omega(b);
        }
        let prev = p.frame().body(ID).unwrap().1.clone();
        let advance = |p: &mut WorkerPool| -> Result<()> {
            if a.coarse {
                p.advance_in_gravity(case.gravity)?;
            } else {
                p.advance_adaptive_in_field(case.gravity, &[])?;
            }
            Ok(())
        };
        let mut events = vec![];
        if [35, 94].contains(&t) {
            let owner = p.frame().body(ID).unwrap().0;
            let pid = p.worker_pid(owner);
            let old = p.frame().clone();
            p.drop_next_acknowledgement(owner)?;
            if advance(&mut p).is_ok() || p.frame() != &old {
                return Err("ambiguous solve advanced".into());
            }
            advance(&mut p)?;
            if p.worker_pid(owner) != pid {
                return Err("cached step replaced owner".into());
            }
            events.push(json!({"kind":"cached_final_ack","worker":owner,"pid":pid}));
        } else {
            advance(&mut p)?;
        }
        let rate = p.last_substeps().ok_or("missing solve receipt")?;
        *rate_hist.entry(rate).or_insert(0) += 1;
        let mut final_native = vec![];
        let mut point_sum = 0.;
        let mut cap = 0.;
        let mut every_four = true;
        let mut previous_com = Vector::from(prev.position);
        let control_id = control.handles[&ID];
        let (pairs, floors) = control.advance_observing(rate, &[], &[], |engine| {
            final_native.clear();
            let mut count = 0;
            for m in engine.partition_ground_patches() {
                assert_eq!(m.body, control_id);
                let patch = GroundContactPatch {
                    body: ID,
                    normal_on_body: m.normal_on_body,
                    points: m.points,
                };
                let (n, j, twist) = geom.observe(&case, &patch);
                point_sum += j;
                cap += twist;
                count += n;
                for pt in &patch.points {
                    let com = Vector::from(pt.solver_point) - Vector::from(pt.body_lever);
                    max_lever_com_error = max_lever_com_error.max((com - previous_com).length());
                }
                final_native.push(patch);
            }
            previous_com = Vector::from(engine.state(control_id).unwrap().position);
            every_four &= count == 4;
            *count_hist.entry(count).or_insert(0) += 1;
        })?;
        let (owner, body) = p.frame().body(ID).unwrap();
        let b = body.clone();
        if b != control.bodies[0]
            || p.frame().workers[usize::from(owner)].snapshot.supports != floors
            || !pairs.is_empty()
        {
            return Err(format!("native physics diverged at {t}").into());
        }
        let public_j = floors.iter().map(|r| r.normal_impulse).sum::<f64>();
        max_sum_error = max_sum_error.max((public_j - point_sum).abs());
        if (21..=30).contains(&t) || (100..=120).contains(&t) {
            max_support_error = max_support_error.max((public_j / (MASS * 9.81 * 0.01) - 1.).abs());
            max_steady_tangent =
                max_steady_tangent.max(case.tangent(Vector::from(b.linear)).length());
        }
        let omega = case.omega(&b);
        let reaction = case.spin(&b) - case.spin(&prev);
        if t >= 31 {
            total_reaction += reaction;
            if every_four && cap > 1e-9 && case.omega(&prev).abs() > 0.2 && omega.abs() > 0.2 {
                errors.push((reaction + sign * cap).abs() / cap);
            }
            if a.frictionless {
                max_reaction_drift = max_reaction_drift.max((case.spin(&b) - sign * KICK).abs());
            }
            if omega.abs() < 0.01 {
                settled += 1;
                if settled == 5 {
                    stop.get_or_insert((t - 4 - 30) as f64 * 0.01);
                }
            } else {
                settled = 0;
            }
        }
        terminal = omega;
        digest.update(bincode::serialize(&(t, &b, &floors))?);
        let path = scratch.path().join(format!("{t}.gzpool"));
        let actual = snapshot(&mut p, &path, &final_native)?;
        inspected += 1;
        patches_digest.update(bincode::serialize(&(t, &actual))?);
        let last_j = actual
            .iter()
            .flat_map(|p| &p.points)
            .map(|p| p.normal_impulse)
            .sum::<f64>();
        if [36, 37, 38, 95, 96, 97].contains(&t) {
            let count = actual.iter().map(|p| loaded(&p.points)).sum::<usize>();
            if count != 4 || last_j <= 0. {
                return Err(format!("operation did not carry a loaded manifold at {t}").into());
            }
            let old = p.frame().clone();
            let event = if t == 36 || t == 95 {
                let dest = (owner + 1) % 4;
                p.transfer_component(ID, dest)?;
                if p.frame().body(ID) != Some((dest, &b))
                    || p.frame().workers[usize::from(dest)].snapshot.supports != floors
                    || p.frame().workers[usize::from(dest)].generations[&ID]
                        != old.workers[usize::from(owner)].generations[&ID] + 1
                {
                    return Err("bad ground handoff".into());
                }
                json!({"kind":"loaded_move","from":owner,"to":dest})
            } else if t == 37 || t == 96 {
                let pid = p.worker_pid(owner);
                p.terminate_worker(owner)?;
                p.recover(owner)?;
                if p.frame() != &old || p.worker_pid(owner) == pid {
                    return Err("bad owner replacement".into());
                }
                json!({"kind":"owner_death","old":pid,"new":p.worker_pid(owner)})
            } else {
                let cut = a.out.join(format!("cut-{t}.gzpool"));
                fs::copy(&path, &cut)?;
                let old_pids: Vec<_> = (0..4).map(|w| p.worker_pid(w)).collect();
                drop(p);
                p = PoolNativeRecovery::open(&cut)?.resume(&a.worker)?;
                configure(&mut p);
                let new_pids: Vec<_> = (0..4).map(|w| p.worker_pid(w)).collect();
                if p.frame() != &old || old_pids.iter().zip(&new_pids).any(|(a, b)| a == b) {
                    return Err("bad fresh pool".into());
                }
                json!({"kind":"disk_restore","old":old_pids,"new":new_pids})
            };
            let after = snapshot(
                &mut p,
                &scratch.path().join(format!("{t}-post.gzpool")),
                &final_native,
            )?;
            inspected += 1;
            if actual != after {
                return Err("authoritative geometry changed at same tick".into());
            }
            let e = json!({"tick":t,"loaded_points":count,"final_native_normal_ns":last_j,"public_normal_ns":public_j,"event":event});
            positive_events.push(e.clone());
            events.push(e);
        }
        if let Some(log) = &mut trace {
            serde_json::to_writer(
                &mut *log,
                &json!({"tick":t,"dt_s":0.01,"substeps":rate,"body":b,"owner":p.frame().body(ID).unwrap().0,"ground":case.plane,"patches":actual,
            "public_normal_ns":public_j,"last_native_normal_ns":last_j,"angular_speed_rad_s":omega,"axial_reaction_nms":reaction,"bounded_coulomb_nms":cap,"events":events}),
            )?;
            writeln!(log)?;
        }
    }
    errors.sort_by(f64::total_cmp);
    let median = (!errors.is_empty()).then(|| errors[errors.len() / 2]);
    let torque = case.mu * MASS * 9.81 * case.radius;
    let ideal = (!a.frictionless).then(|| KICK / torque);
    let expected_initial =
        sign * KICK / (MASS * (case.body.size[0].powi(2) + case.body.size[1].powi(2)) / 12.);
    let ok_geom = geom.max_normal < 1e-12
        && geom.max_plane < 1e-11
        && geom.max_midpoint_tangent < 1e-10
        && geom.max_radius < 1e-3
        && geom.max_negative_load == 0.
        && max_lever_com_error < 1e-10
        && geom.four_point_ticks > 60;
    let ok_analytic = if a.frictionless {
        stop.is_none() && max_reaction_drift < 0.002 && (terminal - initial).abs() < 0.02
    } else {
        stop.zip(ideal).is_some_and(|(a, b)| (a - b).abs() <= 0.02)
            && terminal.abs() < 0.01
            && median.is_some_and(|e| e < 0.01)
            && errors.len() >= 10
            && (total_reaction + sign * KICK).abs() < 0.002
    };
    let ok_support = max_sum_error < 1e-10
        && max_support_error < 0.01
        && (initial - expected_initial).abs() < 0.01;
    let geometry = json!({"max_plane_witness_distance_m":geom.max_plane,"max_midpoint_tangent_offset_m":geom.max_midpoint_tangent,"max_normal_vector_error":geom.max_normal,"max_four_corner_radius_error_m":geom.max_radius,
        "max_frozen_lever_vs_previous_com_m":max_lever_com_error,"max_negative_normal_ns":geom.max_negative_load,"max_point_sum_vs_public_ns":max_sum_error,
        "max_steady_support_relative_error":max_support_error,"max_steady_com_tangent_m_s":max_steady_tangent,"max_frictionless_angular_drift_nms":max_reaction_drift});
    let summary = json!({"ticks":120,"saved_image_comparisons":inspected,"tilted":a.tilted,"rectangle":a.rectangle,"reverse":a.reverse,"frictionless":a.frictionless,"coarse":a.coarse,"packed":a.packed,
        "plane":case.plane,"gravity":case.gravity,"seed":case.body,"mu":case.mu,"radius_m":case.radius,"torque_nm":torque,"signed_kick_nms":sign*KICK,
        "initial_omega_rad_s":initial,"ideal_initial_omega_rad_s":expected_initial,"final_omega_rad_s":terminal,"axial_reaction_sum_nms":total_reaction,
        "ideal_stop_s":ideal,"measured_stop_s":stop,"saturated_intervals":errors.len(),"median_fractional_torque_error":median,"peak_fractional_torque_error":errors.last(),
        "geometry":geometry,
        "points":geom.points,"four_point_native_steps":geom.four_point_ticks,"counts":count_hist,"rates":rate_hist,"events":positive_events,"trajectory_sha256":format!("{:x}",digest.finalize()),
        "patch_sha256":format!("{:x}",patches_digest.finalize()),"passed":{"geometry":ok_geom,"analytic":ok_analytic,"support":ok_support}});
    serde_json::to_writer_pretty(File::create_new(a.out.join("summary.json"))?, &summary)?;
    if !(ok_geom && ok_analytic && ok_support) {
        return Err("ground patch acceptance failed; see summary".into());
    }
    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();
    if !args.matrix {
        return run(&args);
    }
    fs::create_dir(&args.out)?;
    let mut rows = Vec::new();
    for mask in 0u8..32 {
        let mut baseline: Option<serde_json::Value> = None;
        for packed in [false, true] {
            let arm = Args {
                tilted: mask & 1 != 0,
                rectangle: mask & 2 != 0,
                reverse: mask & 4 != 0,
                frictionless: mask & 8 != 0,
                coarse: mask & 16 != 0,
                packed,
                out: args.out.join(format!(
                    "arm-{mask:02}-{}",
                    if packed { "packed" } else { "json" }
                )),
                ..args.clone()
            };
            run(&arm)?;
            let row: serde_json::Value =
                serde_json::from_reader(File::open(arm.out.join("summary.json"))?)?;
            if let Some(old) = &baseline {
                for key in [
                    "trajectory_sha256",
                    "patch_sha256",
                    "rates",
                    "counts",
                    "passed",
                ] {
                    if old[key] != row[key] {
                        return Err(format!("warm encoding changed {key} in case {mask}").into());
                    }
                }
            } else {
                baseline = Some(row.clone());
            }
            rows.push(json!({"case":mask,"dir":arm.out.file_name().unwrap().to_string_lossy(), "tilted":arm.tilted,"rectangle":arm.rectangle,
                "reverse":arm.reverse,"frictionless":arm.frictionless,"coarse":arm.coarse,"packed":packed,"ticks":row["ticks"],
                "saved_images":row["saved_image_comparisons"],"points":row["points"],"positive_events":row["events"].as_array().unwrap().len(),
                "stop_s":row["measured_stop_s"],"ideal_stop_s":row["ideal_stop_s"],"median_torque_fractional_error":row["median_fractional_torque_error"],
                "steady_support_relative_error":row["geometry"]["max_steady_support_relative_error"],"plane_error_m":row["geometry"]["max_plane_witness_distance_m"],
                "point_public_error_ns":row["geometry"]["max_point_sum_vs_public_ns"],"trajectory_sha256":row["trajectory_sha256"],"patch_sha256":row["patch_sha256"]}));
        }
    }
    serde_json::to_writer_pretty(File::create_new(args.out.join("matrix.json"))?, &rows)?;
    Ok(())
}
