// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure the contact wrench of a sliding, turning cuboid against a pointwise Coulomb support bound.
#[path = "support/coupled_box_pair.rs"]
mod box_pair;
#[path = "support/coupled_convergence.rs"]
mod convergence;
#[path = "support/distributed_friction_profiles.rs"]
mod distributed_profiles;
#[path = "support/coupled_feasibility.rs"]
mod feasibility;
#[allow(dead_code)]
#[path = "support/native_contact_control.rs"]
mod native;
#[path = "support/coupled_point_control.rs"]
mod point;
#[path = "support/coupled_timing.rs"]
mod timing;
use clap::Parser;
use gz_physics::{BodyState, DynamicBackend};
use gz_physics_distributed::{
    physics::{Body, SimulationStep},
    pool::{PoolNativeRecovery, WorkerPool},
};
use rapier3d_f64::math::{Rotation, Vector};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const G: [f64; 3] = [0., 0., -9.81];
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: Option<PathBuf>,
    /// Qualify actual distributed friction profiles or time matched placements.
    #[arg(long,requires="worker",conflicts_with_all=["timing_frames","refinement_frames","wrench_frames"])]
    pool_profiles: Option<distributed_profiles::Mode>,
    /// Time the direct grouped and per-corner controls against an existing physical frame stream.
    #[arg(long, conflicts_with = "worker")]
    timing_frames: Option<PathBuf>,
    /// Refine both direct solvers from the measured loaded cut in this frame stream.
    #[arg(long,conflicts_with_all=["worker","timing_frames"])]
    refinement_frames: Option<PathBuf>,
    /// Bound the distance of planar generalized impulses to four independent Coulomb disks.
    #[arg(long,conflicts_with_all=["worker","timing_frames","refinement_frames"])]
    wrench_frames: Option<PathBuf>,
    #[arg(long, default_value_t = 64)]
    rounds: usize,
    directory: PathBuf,
}
#[derive(Clone, Copy)]
struct Case {
    square: bool,
    moving: bool,
    smooth: bool,
    coarse: bool,
    drive: Drive,
}
#[derive(Debug, Clone, Copy, PartialEq)]
enum Drive {
    Slide,
    Spin,
    Mixed,
    Reverse,
}
impl Drive {
    fn impulses(self) -> (f64, f64) {
        match self {
            Self::Slide => (4.8, 0.),
            Self::Spin => (0., 0.96),
            Self::Mixed => (4.8, 0.96),
            Self::Reverse => (-4.8, -0.96),
        }
    }
}
impl Case {
    fn seeds(self) -> [Body; 2] {
        box_pair::seeds(self.square, self.moving, self.smooth)
    }
    fn representative(self) -> bool {
        !self.square && self.moving && !self.smooth && !self.coarse && self.drive == Drive::Mixed
    }
}
struct Inertia {
    diag: Vector,
    mass: f64,
    table_v: Vector,
    radius: f64,
    mu: f64,
}
impl Inertia {
    fn new(body: &Body, table: &Body) -> Self {
        let [x, y, z] = body.size;
        Self {
            diag: Vector::new(y * y + z * z, x * x + z * z, x * x + y * y) * (body.mass / 12.),
            mass: body.mass,
            table_v: table.linear.into(),
            radius: x.hypot(y) * 0.5,
            mu: body.friction,
        }
    }
    fn momentum(&self, s: BodyState) -> Vector {
        let rot = Rotation::from_array(s.orientation);
        rot * (self.diag * (rot.inverse() * Vector::from(s.angular_velocity)))
    }
    fn energy(&self, s: BodyState) -> f64 {
        0.5 * self.mass * (Vector::from(s.velocity) - self.table_v).length_squared()
            + 0.5 * Vector::from(s.angular_velocity).dot(self.momentum(s))
    }
}
#[derive(Default)]
struct Trial {
    eligible: usize,
    violations: usize,
    exceeding: Option<Vec<Value>>,
    max_ratio: f64,
    sum_excess: f64,
    max_energy_residual: f64,
    max_cumulative_residual: f64,
    cumulative_residual: f64,
    normal_error: f64,
    max_force_ratio: f64,
    max_torque_ratio: f64,
    max_horizontal_smooth: f64,
    max_yaw_smooth: f64,
    peak: Value,
    cone_excess: f64,
    impulse_base_error: f64,
    angular_base_error: f64,
}
#[derive(Default)]
struct Interval {
    work: f64,
    bound: f64,
    normals: f64,
    tangent_impulse: Vector,
    tangent_moment: Vector,
    energy_delta: f64,
    max_ratio: f64,
    corners: usize,
    bad: usize,
    samples: Vec<Value>,
}
use point::ContactSample;
struct Observer<'a> {
    i: &'a Inertia,
    trial: &'a mut Trial,
    interval: &'a mut Interval,
    keep: bool,
    tick: u64,
    dt: f64,
}
impl Observer<'_> {
    fn observe(
        &mut self,
        pre: BodyState,
        post: BodyState,
        points: impl IntoIterator<Item = ContactSample>,
        sub: u64,
    ) {
        let i = self.i;
        let v = (Vector::from(pre.velocity) + Vector::from(post.velocity)) * 0.5 - i.table_v;
        let w = (Vector::from(pre.angular_velocity) + Vector::from(post.angular_velocity)) * 0.5;
        let h0 = i.momentum(pre);
        let h1 = i.momentum(post);
        let grav = Vector::from(G) * (i.mass * self.dt);
        let mut pn = Vector::ZERO;
        let mut ln = Vector::ZERO;
        let mut b = 0.;
        let mut nsum = 0.;
        let mut radius_weight = 0.;
        let mut npoint = 0;
        let mut point_pt = Vector::ZERO;
        let mut point_lt = Vector::ZERO;
        let mut known = false;
        for point in points {
            let n = point.normal;
            let r = point.lever;
            let j = n * point.impulse;
            let slip = v + w.cross(r);
            pn += j;
            ln += r.cross(j);
            b += i.mu * point.impulse * (slip - n * slip.dot(n)).length();
            nsum += point.impulse;
            radius_weight += r.x.hypot(r.y) * point.impulse;
            npoint += 1;
            if let Some(t) = point.tangent {
                self.trial.cone_excess = self
                    .trial
                    .cone_excess
                    .max(t.length() - i.mu * point.impulse);
                point_pt += t;
                point_lt += r.cross(t);
                known = true;
            }
        }
        let pt = (Vector::from(post.velocity) - Vector::from(pre.velocity)) * i.mass - grav - pn;
        let lt = h1 - h0 - ln;
        if known {
            self.trial.impulse_base_error =
                self.trial.impulse_base_error.max((pt - point_pt).length());
            self.trial.angular_base_error =
                self.trial.angular_base_error.max((lt - point_lt).length());
        }
        let work = -pt.dot(v) - lt.dot(w);
        let delta = i.energy(post) - i.energy(pre);
        // Exact rotating-inertia endpoint identity for K_rot = omega·H/2.
        let inertia_term = 0.5
            * (Vector::from(post.angular_velocity).dot(h0)
                - Vector::from(pre.angular_velocity).dot(h1));
        let residual = delta - ((grav + pn).dot(v) + ln.dot(w) - work + inertia_term);
        self.trial.max_energy_residual = self.trial.max_energy_residual.max(residual.abs());
        self.trial.cumulative_residual += residual;
        self.trial.max_cumulative_residual = self
            .trial
            .max_cumulative_residual
            .max(self.trial.cumulative_residual.abs());
        let ratio = if b > 1e-8 { work / b } else { 0. };
        let eligible = self.tick >= 31
            && npoint == 4
            && b > 1e-8
            && v.x.hypot(v.y) + i.radius * w.z.abs() > 0.15;
        if eligible {
            self.trial.eligible += 1;
            self.interval.max_ratio = self.interval.max_ratio.max(ratio);
            self.trial.max_force_ratio = self
                .trial
                .max_force_ratio
                .max(pt.x.hypot(pt.y) / (i.mu * nsum));
            self.trial.max_torque_ratio = self
                .trial
                .max_torque_ratio
                .max(lt.z.abs() / (i.mu * radius_weight));
            if ratio > self.trial.max_ratio {
                self.trial.max_ratio = ratio;
                self.trial.peak = json!({
                    "tick": self.tick,
                    "microstep": sub,
                    "work_j": work,
                    "bound_j": b,
                    "ratio": ratio,
                    "velocity": v.to_array(),
                    "omega": w.to_array(),
                    "tangent_impulse_ns": pt.to_array(),
                    "tangent_moment_nms": lt.to_array(),
                    "normal_ns": nsum
                });
            }
            if ratio > 1.01 {
                self.trial.violations += 1;
                if let Some(steps) = &mut self.trial.exceeding {
                    steps.push(json!({
                    "tick":self.tick, "sub":sub, "work_j":work, "bounded_j":b, "ratio":ratio,
                    "linear_tangent_ns":pt.to_array(), "angular_tangent_nms":lt.to_array(),
                    "normal_ns":nsum, "average_com_m_s":v.to_array(), "average_angular_rad_s":w.to_array()
                }));
                }
                self.interval.bad += 1;
                self.trial.sum_excess += work - b;
            }
        }
        if i.mu == 0. && self.tick >= 31 {
            self.trial.max_horizontal_smooth =
                self.trial.max_horizontal_smooth.max(pt.x.hypot(pt.y));
            self.trial.max_yaw_smooth = self.trial.max_yaw_smooth.max(lt.z.abs());
        }
        self.interval.tangent_impulse += pt;
        self.interval.tangent_moment += lt;
        self.interval.work += work;
        self.interval.bound += b;
        self.interval.normals += nsum;
        self.interval.energy_delta += delta;
        self.interval.corners = npoint;
        if self.keep {
            self.interval.samples.push(json!({
                "substep": sub,
                "points": npoint,
                "normal_ns": nsum,
                "work_j": work,
                "point_bound_j": b,
                "eligible": eligible,
                "ratio": ratio,
                "energy_error_j": residual
            }));
        }
    }
}
fn run(case: Case, worker: &Path, out: &Path, ledger: &mut impl Write) -> Result<Value> {
    let seeds = case.seeds();
    let inertia = Inertia::new(&seeds[1], &seeds[0]);
    let mut direct = point::Direct::new(&seeds, false);
    let mut point = point::Direct::new(&seeds, true);
    let mut point_trial = Trial::default();
    let mut max_position_gap = 0.0_f64;
    let mut control = native::Control::new(&seeds, None, G)?;
    let mut pool = WorkerPool::spawn(
        worker,
        0x434f5550,
        vec![seeds.to_vec(), vec![], vec![], vec![]],
        SimulationStep::Ms10,
        None,
        128,
    )?;
    pool.set_packed_handoffs(true);
    let mut log = case
        .representative()
        .then(|| File::create_new(out.join("frames.jsonl")))
        .transpose()?
        .map(BufWriter::new);
    let scratch = tempfile::tempdir_in(out)?;
    let mut trial = Trial {
        exceeding: Some(vec![]),
        ..Default::default()
    };
    let mut hash = Sha256::new();
    let mut terminal = 0.;
    let mut event_inventory = vec![];
    for tick in 1..=120 {
        let handle = control.handles[&2];
        let mut events = vec![];
        if tick == 31 {
            let (fx, tz) = case.drive.impulses();
            let (linear, angular) = ([fx, 0., 0.], [0., 0., tz]);
            pool.apply_impulse_in_field(2, linear, angular, G, &[])?;
            control.solver.apply_impulse(handle, linear, angular)?;
            direct.kick(linear, angular);
            point.kick(linear, angular);
            events.push(json!({"kind":"drive","com_ns":fx,"yaw_nms":tz}));
        }
        let advance = |p: &mut WorkerPool| -> Result<()> {
            if case.coarse {
                p.advance_in_gravity(G)?;
            } else {
                p.advance_adaptive_in_field(G, &[])?;
            }
            Ok(())
        };
        let owner = pool.frame().body(2).unwrap().0;
        if tick == 35 {
            let prev = pool.frame().clone();
            let pid = pool.worker_pid(owner);
            pool.drop_next_acknowledgement(owner)?;
            if advance(&mut pool).is_ok() || pool.frame() != &prev {
                return Err("partial step published".into());
            }
            advance(&mut pool)?;
            if pool.worker_pid(owner) != pid {
                return Err("cached owner replaced".into());
            }
            events.push(json!({"kind":"lost_ack","pid":pid}));
        } else {
            advance(&mut pool)?;
        }
        let substeps = pool.last_substeps().ok_or("missing rate")?;
        let mut prev = control.solver.state(handle).ok_or("missing control")?;
        let mut interval = Interval::default();
        let mut sub = 0;
        let mut point_interval = Interval::default();
        let dt = 0.01 / f64::from(substeps);
        let mut point_obs = Observer {
            i: &inertia,
            trial: &mut point_trial,
            interval: &mut point_interval,
            keep: log.is_some(),
            tick,
            dt,
        };
        let mut observer = Observer {
            i: &inertia,
            trial: &mut trial,
            interval: &mut interval,
            keep: log.is_some(),
            tick,
            dt: 0.01 / f64::from(substeps),
        };
        let (pairs, floors) = control.advance_observing(substeps, &[], &[], |engine| {
            let next = engine.state(handle).unwrap();
            direct.step(dt);
            assert_eq!(next, direct.state(1), "separate grouped direct control");
            let points = engine
                .rigid_contact_patches()
                .into_iter()
                .flat_map(|patch| {
                    assert_eq!((patch.a.raw(), patch.b.raw()), (0, 1));
                    let normal = -Vector::from(patch.normal_on_a);
                    patch.points.into_iter().map(move |p| ContactSample {
                        normal,
                        lever: p.lever_b.into(),
                        impulse: p.normal_impulse,
                        tangent: None,
                    })
                });
            observer.observe(prev, next, points, sub);
            let first = point.state(1);
            point.step(dt);
            let last = point.state(1);
            point_obs.observe(first, last, point.points(), sub);
            max_position_gap = max_position_gap
                .max((Vector::from(next.position) - Vector::from(last.position)).length());
            prev = next;
            sub += 1;
        })?;
        let physical = [
            pool.frame().body(1).unwrap().1.clone(),
            pool.frame().body(2).unwrap().1.clone(),
        ];
        if physical.as_slice() != control.bodies
            || pairs != pool.frame().workers[owner as usize].snapshot.impulses
            || !floors.is_empty()
            || pool
                .frame()
                .workers
                .iter()
                .any(|w| !w.snapshot.supports.is_empty())
        {
            return Err(format!("native oracle at tick {tick}").into());
        }
        let normal = pairs.iter().map(|p| p.normal_impulse).sum::<f64>();
        trial.normal_error = trial.normal_error.max((interval.normals - normal).abs());
        terminal = inertia.energy(prev);
        hash.update(bincode::serialize(&(tick, &physical, &pairs))?);
        let old = pool.frame().clone();
        match tick {
            36 => {
                if normal <= 0. {
                    return Err("unloaded transfer".into());
                }
                pool.transfer_component(2, 1)?;
                for b in &physical {
                    let (owner, new) = pool.frame().body(b.id).unwrap();
                    if owner != 1 || b != new || pool.frame().workers[1].generations[&b.id] != 1 {
                        return Err("transfer state".into());
                    }
                }
                if pool.frame().workers[1].snapshot.impulses != pairs {
                    return Err("contact lost".into());
                }
                events.push(json!({"kind":"warm_move","members":[1,2],"from":0,"to":1}));
            }
            37 => {
                let pid = pool.worker_pid(owner);
                pool.terminate_worker(owner)?;
                pool.recover(owner)?;
                if pool.frame() != &old || pool.worker_pid(owner) == pid {
                    return Err("replacement changed state".into());
                }
                events.push(
                    json!({"kind":"owner_death","old_pid":pid,"new_pid":pool.worker_pid(owner)}),
                );
            }
            38 => {
                let file = scratch.path().join("cut");
                pool.save_native_recovery(&file)?;
                let pids = (0..4).map(|s| pool.worker_pid(s)).collect::<Vec<_>>();
                drop(pool);
                pool = PoolNativeRecovery::open(file)?.resume(worker)?;
                pool.set_packed_handoffs(true);
                let fresh = (0..4).map(|s| pool.worker_pid(s)).collect::<Vec<_>>();
                if pool.frame() != &old || fresh.iter().any(|id| pids.contains(id)) {
                    return Err("fresh process restore".into());
                }
                events.push(json!({"kind":"disk_restore","before":pids,"after":fresh}));
            }
            _ => (),
        }
        event_inventory.extend(events.iter().map(|e| json!({"tick":tick,"event":e})));
        if let Some(w) = &mut log {
            let mut alt = seeds.clone();
            for (id, b) in alt.iter_mut().enumerate() {
                let s = point.state(id);
                b.position = s.position;
                b.orientation = s.orientation;
                b.linear = s.velocity;
                b.angular = s.angular_velocity;
            }

            writeln!(
                w,
                "{}",
                json!({
                    "tick": tick,
                    "bodies": physical,
                    "reference_bodies": control.bodies,
                    "point_control": {
                        "bodies": alt,
                        "work_j": point_interval.work,
                        "point_bound_j": point_interval.bound,
                        "eligible_ratio": point_interval.max_ratio,
                        "violations": point_interval.bad,
                        "energy_j": inertia.energy(point.state(1)),
                        "microsteps": point_interval.samples
                    },
                    "pairs": pairs,
                    "owner": pool.frame().body(2).unwrap().0,
                    "events": events,
                    "substeps": substeps,
                    "energy_j": terminal,
                    "energy_delta_j": interval.energy_delta,
                    "work_j": interval.work,
                    "point_bound_j": interval.bound,
                    "max_eligible_ratio": interval.max_ratio,
                    "violations": interval.bad,
                    "microsteps": interval.samples,
                    "tangent_impulse_ns": interval.tangent_impulse.to_array(),
                    "tangent_moment_nms": interval.tangent_moment.to_array()
                })
            )?;
        }
    }
    if trial.normal_error > 1e-9 || trial.max_cumulative_residual > 1e-9 {
        return Err("normal or kinetic energy closure failed".into());
    }
    for step in trial.exceeding.into_iter().flatten() {
        writeln!(
            ledger,
            "{}",
            json!({
                "square":case.square,"moving":case.moving,"smooth":case.smooth,"coarse":case.coarse,
                "drive": format!("{:?}",case.drive),"sample":step
            })
        )?;
    }
    Ok(json!({
        "square": case.square,
        "moving": case.moving,
        "smooth": case.smooth,
        "coarse": case.coarse,
        "drive": format!("{:?}",case.drive),
        "eligible_microsteps": trial.eligible,
        "violations": trial.violations,
        "maximum_ratio": trial.max_ratio,
        "summed_violating_excess_j": trial.sum_excess,
        "peak": trial.peak,
        "maximum_linear_marginal_ratio": trial.max_force_ratio,
        "maximum_yaw_marginal_ratio": trial.max_torque_ratio,
        "max_normal_error_ns": trial.normal_error,
        "max_energy_error_j": trial.max_energy_residual,
        "point_control": {
            "eligible_microsteps": point_trial.eligible,
            "violations": point_trial.violations,
            "maximum_ratio": point_trial.max_ratio,
            "maximum_corner_cone_excess_ns": point_trial.cone_excess,
            "maximum_tangent_linear_residual_ns": point_trial.impulse_base_error,
            "maximum_tangent_angular_residual_nms": point_trial.angular_base_error,
            "peak": point_trial.peak,
            "maximum_position_difference_m": max_position_gap,
            "final_energy_j": inertia.energy(point.state(1))
        },
        "max_cumulative_energy_error_j": trial.max_cumulative_residual,
        "smooth_max_horizontal_residual_ns": trial.max_horizontal_smooth,
        "smooth_max_yaw_residual_nms": trial.max_yaw_smooth,
        "final_energy_j": terminal,
        "trajectory_sha256": format!("{:x}",hash.finalize()),
        "events": event_inventory
    }))
}
fn main() -> Result<()> {
    let a = Args::parse();
    fs::create_dir(&a.directory)?;
    if let Some(source) = &a.wrench_frames {
        return feasibility::run(source, &a.directory);
    }
    if let Some(source) = &a.refinement_frames {
        return convergence::run(source, &a.directory);
    }
    if let Some(source) = &a.timing_frames {
        return timing::run(source, a.rounds, &a.directory);
    }
    let worker = a.worker.ok_or(
        "give --worker for the physical study or --timing-frames for the paired benchmark",
    )?;
    if let Some(mode) = a.pool_profiles {
        return distributed_profiles::study(&worker, &a.directory, mode, a.rounds);
    }
    let mut w = BufWriter::new(File::create_new(a.directory.join("cases.jsonl"))?);
    let mut rows = vec![];
    let mut ledger = BufWriter::new(File::create_new(a.directory.join("violating-steps.jsonl"))?);
    for square in [false, true] {
        for moving in [true, false] {
            for smooth in [false, true] {
                for coarse in [false, true] {
                    for drive in [Drive::Mixed, Drive::Slide, Drive::Spin, Drive::Reverse] {
                        let row = run(
                            Case {
                                square,
                                moving,
                                smooth,
                                coarse,
                                drive,
                            },
                            &worker,
                            &a.directory,
                            &mut ledger,
                        )?;
                        writeln!(w, "{row}")?;
                        w.flush()?;
                        rows.push(row);
                    }
                }
            }
        }
    }
    ledger.flush()?;
    fs::write(
        a.directory.join("summary.json"),
        serde_json::to_vec_pretty(&json!({"ticks_per_case":120,"cases":rows}))?,
    )?;
    Ok(())
}
