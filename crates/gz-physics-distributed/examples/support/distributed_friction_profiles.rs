// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the two negotiated distributed friction profiles on active finite contacts.
use super::{Body, Case, Drive, G, Inertia, Interval, Observer, Result, Trial, point};
use clap::ValueEnum;
use gz_physics::BodyState;
use gz_physics_distributed::{
    physics::{ContactImpulse, NativeFrictionModel, NativeSolverProfile, SimulationStep},
    pool::{PoolNativeRecovery, WorkerPool},
};
use rapier3d_f64::math::Vector;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    fs::{self, File},
    io::{BufWriter, Write},
    path::Path,
    time::Instant,
};

#[derive(Clone, Copy, PartialEq, ValueEnum)]
pub(super) enum Mode {
    Smoke,
    Matrix,
    Bench,
}
#[derive(Debug, Clone, Copy)]
struct Config {
    pointwise: bool,
    extended: bool,
    split: bool,
    packed: bool,
    width: u8,
    drive: Drive,
}
impl Config {
    fn profile(self) -> NativeSolverProfile {
        let base = if self.extended {
            NativeSolverProfile::Extended64
        } else {
            NativeSolverProfile::Standard32
        };
        if self.pointwise {
            base.with_friction(NativeFrictionModel::PerPoint)
        } else {
            base
        }
    }
    fn source(self) -> [Vec<Body>; 2] {
        let case = Case {
            square: false,
            moving: true,
            smooth: false,
            coarse: false,
            drive: self.drive,
        };
        let left = case.seeds();
        let mut right = left.clone();
        for b in &mut right {
            b.id += 10;
            b.position[1] += 20.;
        }
        [left.to_vec(), right.to_vec()]
    }
    fn representative(self) -> bool {
        !self.extended && self.split && self.packed && self.width == 4 && self.drive == Drive::Mixed
    }
}
fn configure(pool: &mut WorkerPool, c: Config) -> Result<()> {
    pool.set_local_helper_limit(Some(0));
    pool.set_packed_observations(c.packed);
    let n = pool.set_packed_handoffs(c.packed);
    if n != if c.packed { usize::from(c.width) } else { 0 }
        || pool.native_solver_profile() != c.profile()
    {
        return Err("profile or handoff negotiation mismatch".into());
    }
    Ok(())
}
fn stamp(pool: &WorkerPool) -> Vec<Option<u32>> {
    (0..pool.frame().workers.len())
        .map(|i| pool.worker_pid(i as u8))
        .collect()
}
fn arch_check(
    pool: &mut WorkerPool,
    oracles: &[point::Direct; 2],
    c: Config,
    path: &Path,
    sink: &mut Option<BufWriter<File>>,
) -> Result<usize> {
    pool.save_native_recovery(path)?;
    let image = PoolNativeRecovery::open(path)?;
    if image.native_solver_profile() != c.profile() {
        return Err("checkpoint profile mismatch".into());
    }
    let all = image.finite_contact_patches()?;
    let mut checked = 0;
    for (site, oracle) in oracles.iter().enumerate() {
        let id = 1 + 10 * site as u64;
        let owner = usize::from(pool.frame().body(id).ok_or("missing site")?.0);
        let mut patches = all
            .iter()
            .enumerate()
            .flat_map(|(w, patches)| patches.iter().map(move |p| (w, p)))
            .filter(|(_, p)| p.a == id && p.b == id + 1);
        let (w, p) = patches.next().ok_or("missing native manifold")?;
        if w != owner || patches.next().is_some() {
            return Err("duplicate native patch or wrong owner".into());
        }
        let expected = oracle.points();
        if p.points.len() != 4 || expected.len() != 4 {
            return Err("need four actual finite corners at qualified cut".into());
        }
        let mut found = Vec::new();
        for actual in &p.points {
            let j = expected
                .iter()
                .position(|e| {
                    e.lever.to_array() == actual.lever_b
                        && e.impulse.to_bits() == actual.normal_impulse.to_bits()
                        && e.normal.to_array() == p.normal_on_a.map(|n| -n)
                })
                .ok_or("native COM lever, normal or corner impulse differs")?;
            if found.contains(&j) {
                return Err("duplicate corner".into());
            }
            let direct = &expected[j];
            let force = actual.tangent_impulse_on_a;
            let direct_on_block = if c.pointwise {
                let observed = Vector::from(force.ok_or("missing actual per-corner tangent")?);
                let expected_on_block = direct.tangent.ok_or("missing direct native tangent")?;
                if observed != -expected_on_block
                    || observed.length() > 0.35 * actual.normal_impulse + 1e-12
                    || observed.dot(direct.normal).abs() > 1e-12
                {
                    return Err("archived signed world tangent, its direction, or the individual Coulomb cone differs".into());
                }
                Some(expected_on_block.to_array())
            } else {
                if force.is_some() {
                    return Err("grouped checkpoint advertised a per-corner tangent force".into());
                }
                None
            };
            if let Some(w) = sink.as_mut() {
                writeln!(
                    w,
                    "{}",
                    json!({
                        "stage":path.file_name().unwrap().to_string_lossy(),"tick":pool.frame().tick,"owner":owner,
                        "pair":[p.a,p.b],"pointwise":c.pointwise,"normal_on_table":p.normal_on_a,"normal_ns":actual.normal_impulse,
                        "lever_table_m":actual.lever_a,"lever_block_m":actual.lever_b,"archived_table_tangent_ns":force,
                        "direct_block_tangent_ns":direct_on_block
                    })
                )?;
            }
            found.push(j);
            checked += 1;
        }
    }
    Ok(checked)
}
fn body_state(b: &Body) -> BodyState {
    BodyState {
        position: b.position,
        orientation: b.orientation,
        velocity: b.linear,
        angular_velocity: b.angular,
    }
}
#[derive(Default)]
struct CornerSums {
    linear: Vector,
    angular: Vector,
    normal: Vector,
    normal_moment: Vector,
}
impl CornerSums {
    fn accumulate(&mut self, points: &[super::ContactSample]) {
        for p in points {
            let normal = p.impulse * p.normal;
            self.normal += normal;
            self.normal_moment += p.lever.cross(normal);
            if let Some(t) = p.tangent {
                self.linear += t;
                self.angular += p.lever.cross(t);
            }
        }
    }
}
fn actual(seeds: &[Vec<Body>; 2], pool: &WorkerPool) -> Vec<Body> {
    seeds
        .iter()
        .flatten()
        .map(|b| pool.frame().body(b.id).unwrap().1.clone())
        .collect()
}
fn run(
    worker: &Path,
    out: &Path,
    c: Config,
    qualified: bool,
    record: bool,
    round: usize,
) -> Result<Value> {
    let seeds = c.source();
    let mut slots = vec![Vec::new(); usize::from(c.width)];
    slots[0].extend_from_slice(&seeds[0]);
    slots[usize::from(c.split)].extend_from_slice(&seeds[1]);
    let mut pool = WorkerPool::spawn_with_profile(
        worker,
        0x465243,
        slots,
        SimulationStep::Ms10,
        None,
        180,
        c.profile(),
    )?;
    configure(&mut pool, c)?;
    let mut native = seeds
        .each_ref()
        .map(|v| point::Direct::new(&[v[0].clone(), v[1].clone()], c.pointwise));
    let inertias = seeds.each_ref().map(|v| Inertia::new(&v[1], &v[0]));
    let mut trials: [Trial; 2] = Default::default();
    let mut hash = Sha256::new();
    let mut measured = vec![];
    let mut history = vec![];
    let mut inspected = 0;
    let mut loaded = 0;
    let mut micros = 0;
    let probe =
        qualified && !c.extended && c.split && c.packed && c.width == 4 && c.drive == Drive::Spin;
    let label = if c.pointwise { "pointwise" } else { "grouped" };
    let mut cut_log = if record || probe {
        Some(BufWriter::new(File::create_new(out.join(format!(
            "{}-{label}-corners.jsonl",
            if probe { "spin" } else { "mixed" }
        )))?))
    } else {
        None
    };
    let mut extra_points = 0;
    let mut micro_witness = Value::Null;
    let mut max_barrier_force_error = 0.0_f64;
    let mut max_barrier_moment_error = 0.0_f64;
    let scratch = tempfile::tempdir_in(out)?;
    let archive_dir = if record || probe {
        let dir = out.join(format!(
            "native-cuts-{}-{label}",
            if probe { "spin" } else { "mixed" }
        ));
        fs::create_dir(&dir)?;
        dir
    } else {
        scratch.path().to_owned()
    };
    let mut log = if record {
        Some(BufWriter::new(File::create_new(out.join(
            if c.pointwise {
                "pointwise.jsonl"
            } else {
                "grouped.jsonl"
            },
        ))?))
    } else {
        None
    };
    for tick in 1..=120u64 {
        let mut events = vec![];
        if tick == 31 {
            let (fx, tz) = c.drive.impulses();
            for (site, n) in native.iter_mut().enumerate() {
                let sign = if site == 0 { 1. } else { -1. };
                let lin = [sign * fx, 0., 0.];
                let ang = [0., 0., sign * tz];
                pool.apply_impulse_in_field(2 + 10 * site as u64, lin, ang, G, &[])?;
                n.kick(lin, ang);
            }
            events.push(json!({"kind":"opposite_drives","linear_x":[fx,-fx],"yaw":[tz,-tz]}));
        }
        let before_motion = [2, 12].map(|id| body_state(pool.frame().body(id).unwrap().1));
        if qualified && tick == 35 {
            let owner = pool.frame().body(2).unwrap().0;
            let old = pool.frame().clone();
            let pids = stamp(&pool);
            pool.drop_next_acknowledgement(owner)?;
            if pool.advance_adaptive_in_field(G, &[]).is_ok() || pool.frame() != &old {
                return Err("published lost-reply advance".into());
            }
            pool.advance_adaptive_in_field(G, &[])?;
            if stamp(&pool) != pids {
                return Err("living owner unexpectedly replaced".into());
            }
            events.push(json!({"kind":"lost_final_ack","owner":owner,"pids":pids}));
        } else {
            let start = Instant::now();
            pool.advance_adaptive_in_field(G, &[])?;
            measured.push(start.elapsed().as_nanos() as u64);
        }
        let rate = pool.last_substeps().ok_or("rate")?;
        let dt = 0.01 / f64::from(rate);
        let mut metrics = vec![];
        let mut reference = vec![];
        let mut receipts = vec![];
        for (site, n) in native.iter_mut().enumerate() {
            let mut interval = Interval::default();
            let mut norm = None;
            let mut corners = CornerSums::default();
            for sub in 0..rate {
                let pre = n.state(1);
                n.step(dt);
                let post = n.state(1);
                let mut points = n.points();
                if probe && site == 0 && tick == 31 && sub == 1 {
                    let mut sums = CornerSums::default();
                    sums.accumulate(&points);
                    let i = &inertias[site];
                    let jt = (Vector::from(post.velocity)
                        - Vector::from(pre.velocity)
                        - Vector::from(G) * dt)
                        * i.mass
                        - sums.normal;
                    let lt = i.momentum(post) - i.momentum(pre) - sums.normal_moment;
                    let required = Vector::new(jt.x, jt.y, lt.z / i.radius);
                    micro_witness =
                        super::feasibility::certificate(&points, required, i.radius, i.mu)?;
                    let fraction = micro_witness["lower_ns"].as_f64().unwrap()
                        / micro_witness["mu_normal_ns"].as_f64().unwrap();
                    if (!c.pointwise && (fraction - 0.01157740333).abs() > 1.0e-7)
                        || (c.pointwise && fraction > 1e-6)
                    {
                        return Err("prior pure-spin substep certificate failed".into());
                    }
                }
                if c.pointwise {
                    corners.accumulate(&points);
                }
                if !c.pointwise {
                    for p in &mut points {
                        p.tangent = None;
                    }
                }
                Observer {
                    i: &inertias[site],
                    trial: &mut trials[site],
                    interval: &mut interval,
                    keep: record,
                    tick,
                    dt,
                }
                .observe(pre, post, points, u64::from(sub));
                if let Some(j) = n.normal_receipt() {
                    *norm.get_or_insert(0.) += j;
                }
                micros += 1;
            }
            let actual_after = body_state(pool.frame().body(2 + 10 * site as u64).unwrap().1);
            let before = before_motion[site];
            let inv = &inertias[site];
            let barrier_tangent = (Vector::from(actual_after.velocity)
                - Vector::from(before.velocity)
                - Vector::from(G) * 0.01)
                * inv.mass
                - corners.normal;
            let barrier_torsion =
                inv.momentum(actual_after) - inv.momentum(before) - corners.normal_moment;
            if c.pointwise {
                max_barrier_force_error =
                    max_barrier_force_error.max((barrier_tangent - corners.linear).length());
                max_barrier_moment_error =
                    max_barrier_moment_error.max((barrier_torsion - corners.angular).length());
            }
            if let Some(normal_impulse) = norm {
                let a = 1 + 10 * site as u64;
                receipts.push(ContactImpulse {
                    a,
                    b: a + 1,
                    normal_impulse,
                });
                loaded += usize::from(normal_impulse > 0.);
            }
            for (i, b) in seeds[site].iter().enumerate() {
                let s = n.state(i);
                let mut complete = b.clone();
                complete.position = s.position;
                complete.orientation = s.orientation;
                complete.linear = s.velocity;
                complete.angular = s.angular_velocity;
                reference.push(complete);
            }
            if record {
                metrics.push(json!({"site":site,"work_j":interval.work,"point_bound_j":interval.bound,"normal_ns":interval.normals,"max_eligible_ratio":interval.max_ratio,
                    "distributed_tangent_ns":c.pointwise.then(||barrier_tangent.to_array()),"native_corner_tangent_ns":c.pointwise.then(||corners.linear.to_array()),
                    "distributed_tangent_moment_nms":c.pointwise.then(||barrier_torsion.to_array()),"native_corner_tangent_moment_nms":c.pointwise.then(||corners.angular.to_array()),"violations":interval.bad,"microsteps":interval.samples,"tangent_impulse_ns":interval.tangent_impulse.to_array(),"tangent_moment_nms":interval.tangent_moment.to_array()}));
            }
        }
        let state = actual(&seeds, &pool);
        let frame = pool.frame();
        let mut got = frame
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.clone())
            .collect::<Vec<_>>();
        got.sort_by_key(|v| (v.a, v.b));
        if state != reference
            || got != receipts
            || frame
                .workers
                .iter()
                .any(|s| !s.snapshot.supports.is_empty())
        {
            return Err(format!("independent native mismatch at {tick} ({c:?}), states={} receipts={} actual={got:?} expected={receipts:?}",state==reference,got==receipts).into());
        }
        hash.update(bincode::serialize(&(tick, &state, &got))?);
        if probe && tick == 31 {
            extra_points = arch_check(
                &mut pool,
                &native,
                c,
                &archive_dir.join("spin-end-31"),
                &mut cut_log,
            )?;
            let b = &state[1];
            fs::write(
                out.join(format!("spin-31-{label}.json")),
                serde_json::to_vec_pretty(&json!({
                    "profile":label,"site":0,
                    "direct_native_sample":{"physical_time_ns":302_000_000,"public_tick":31,"substep_zero_based":1,"certificate":micro_witness},
                    "actual_worker_barrier":{"physical_time_ns":310_000_000,"public_tick":31,"owner":pool.frame().body(2).unwrap().0,
                        "block":b,"normal_impulse_10ms_ns":got.iter().find(|p|p.a==1&&p.b==2).unwrap().normal_impulse,
                        "last_microstep_corner_records":format!("spin-{label}-corners.jsonl"),"archive_stage":"spin-end-31",
                        "matches_direct_state_and_normal_receipt":true}
                }))?,
            )?;
        }
        if qualified && [36, 37, 38].contains(&tick) {
            let before = pool.frame().clone();
            let previous = stamp(&pool);
            let path = archive_dir.join(format!("pre-{tick}"));
            inspected += arch_check(&mut pool, &native, c, &path, &mut cut_log)?;
            let owner = pool.frame().body(2).unwrap().0;
            match tick {
                36 => {
                    pool.transfer_component(2, 1)?;
                    for b in &state[0..2] {
                        let (w, body) = pool.frame().body(b.id).unwrap();
                        if w != 1 || body != b || pool.frame().workers[1].generations[&b.id] != 1 {
                            return Err("warm move lost body or generation".into());
                        }
                    }
                    let expected_other = if c.split { 1 } else { 0 };
                    if pool.frame().body(12).unwrap().0 != expected_other
                        || stamp(&pool) != previous
                    {
                        return Err("second site unexpectedly moved".into());
                    }
                    events.push(json!({"kind":"loaded_handoff","from":owner,"to":1,"members":[1,2],"other_owner":expected_other}));
                }
                37 => {
                    pool.terminate_worker(owner)?;
                    pool.recover(owner)?;
                    let after = stamp(&pool);
                    if before != *pool.frame()
                        || previous
                            .iter()
                            .zip(&after)
                            .enumerate()
                            .any(|(i, (x, y))| (x != y) != (i == usize::from(owner)))
                    {
                        return Err("target replacement lost state or healthy process".into());
                    }
                    events.push(json!({"kind":"owner_replacement","owner":owner,"before":previous,"after":after}));
                }
                38 => {
                    drop(pool);
                    pool = PoolNativeRecovery::open(&path)?.resume(worker)?;
                    configure(&mut pool, c)?;
                    if before != *pool.frame() || stamp(&pool).iter().any(|p| previous.contains(p))
                    {
                        return Err("fresh restart lost committed frame".into());
                    }
                    events
                        .push(json!({"kind":"disk_resume","before":previous,"after":stamp(&pool)}));
                }
                _ => unreachable!(),
            }
            inspected += arch_check(
                &mut pool,
                &native,
                c,
                &archive_dir.join(format!("post-{tick}")),
                &mut cut_log,
            )?;
        }
        if let Some(output) = &mut log {
            let owners = state
                .iter()
                .map(|b| pool.frame().body(b.id).unwrap().0)
                .collect::<Vec<_>>();
            writeln!(
                output,
                "{}",
                json!({
                    "tick": tick,
                    "time_ns": tick*10_000_000,
                    "pointwise": c.pointwise,
                    "bodies": state,
                    "reference": reference,
                    "pairs": receipts,
                    "owners": owners,
                    "metrics": metrics,
                    "substeps": rate,
                    "events": events
                })
            )?;
        }
        history.extend(events.into_iter().map(|e| json!({"tick":tick,"event":e})));
    }
    if max_barrier_force_error > 1e-10 {
        return Err("public COM impulse disagrees with summed native corner tangents".into());
    }
    if c.pointwise
        && trials
            .iter()
            .any(|t| t.violations != 0 || t.cone_excess > 1e-10 || t.impulse_base_error > 1e-10)
    {
        return Err("per-corner Coulomb, tangential impulse or work qualification".into());
    }
    let sum: u64 = measured.iter().sum();
    measured.sort_unstable();
    Ok(json!({
        "pointwise": c.pointwise,
        "extended": c.extended,
        "split": c.split,
        "packed": c.packed,
        "width": c.width,
        "drive": format!("{:?}",c.drive),
        "round": round,
        "ticks": 120,
        "native_site_microsteps": micros,
        "loaded_pair_receipts": loaded,
        "exact_patch_points": inspected,
        "additional_spin_barrier_points": extra_points,
        "actual_signed_tangent_checks": if c.pointwise{inspected+extra_points}else{0},
        "eligible": trials.iter().map(|t|t.eligible).sum::<usize>(),
        "violations": trials.iter().map(|t|t.violations).sum::<usize>(),
        "maximum_ratio": trials.iter().map(|t|t.max_ratio).fold(0.,f64::max),
        "worst_cone_excess_ns": trials.iter().map(|t|t.cone_excess).fold(0.,f64::max),
        "max_tangent_linear_residual_ns": trials.iter().map(|t|t.impulse_base_error).fold(0.,f64::max),
        "max_tangent_angular_residual_nms": trials.iter().map(|t|t.angular_base_error).fold(0.,f64::max),
        "max_barrier_tangent_residual_ns": c.pointwise.then_some(max_barrier_force_error),
        "max_barrier_tangent_moment_residual_nms": c.pointwise.then_some(max_barrier_moment_error),
        "elapsed_step_ns": sum,
        "p95_step_ns": measured[(measured.len()*95).div_ceil(100)-1],
        "trajectory_sha256": format!("{:x}",hash.finalize()),
        "events": history
    }))
}
pub(super) fn study(worker: &Path, out: &Path, mode: Mode, rounds: usize) -> Result<()> {
    let mut file = BufWriter::new(File::create_new(out.join("trials.jsonl"))?);
    let mut rows = vec![];
    let mut add = |c: Config, qualification, record, round| -> Result<()> {
        let row = run(worker, out, c, qualification, record, round)?;
        writeln!(file, "{row}")?;
        file.flush()?;
        rows.push(row);
        Ok(())
    };
    let base = Config {
        pointwise: false,
        extended: false,
        split: true,
        packed: true,
        width: 4,
        drive: Drive::Mixed,
    };
    match mode {
        Mode::Smoke => {
            for pointwise in [false, true] {
                add(Config { pointwise, ..base }, true, true, 0)?;
            }
        }
        Mode::Matrix => {
            for width in [3, 4, 8] {
                for extended in [false, true] {
                    for split in [false, true] {
                        for packed in [false, true] {
                            for drive in [Drive::Slide, Drive::Spin, Drive::Mixed, Drive::Reverse] {
                                for pointwise in [false, true] {
                                    let c = Config {
                                        width,
                                        extended,
                                        split,
                                        packed,
                                        drive,
                                        pointwise,
                                    };
                                    add(c, true, c.representative(), 0)?;
                                }
                            }
                        }
                    }
                }
            }
        }
        Mode::Bench => {
            if rounds < 2 || !rounds.is_multiple_of(2) {
                return Err("benchmark requires an even pair count".into());
            }
            for round in 0..rounds {
                for layout in 0..2 {
                    let split = (layout + round) % 2 == 1;
                    for order in 0..2 {
                        let pointwise = (order + round + layout) % 2 == 1;
                        add(
                            Config {
                                split,
                                pointwise,
                                ..base
                            },
                            false,
                            false,
                            round,
                        )?;
                    }
                }
            }
        }
    }
    fs::write(out.join("summary.json"), serde_json::to_vec_pretty(&rows)?)?;
    Ok(())
}
