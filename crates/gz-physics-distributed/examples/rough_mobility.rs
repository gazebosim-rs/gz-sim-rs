// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A free solid sphere slides, spins up, then rolls on a finite moving support.
#[path = "support/rough_contact_native.rs"]
mod native;
use clap::Parser;
use gz_physics_distributed::{
    coordinator::Error as PoolError,
    physics::{Body, ContactImpulse, GroundSupport, Primitive, SimulationStep, Tier},
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use serde::Serialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Duration,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const MASS: f64 = 2.;
const RADIUS: f64 = 0.3;
const INERTIA: f64 = 0.4 * MASS * RADIUS * RADIUS;
const G: f64 = 9.81;
const FIELD: [f64; 3] = [0., 0., -G];
const MU: f64 = 0.35;
const KICK: f64 = 7.;
const DRIVE_MS: u64 = 200;
const END_MS: u64 = 1200;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// Public interval, in milliseconds. The qualification supports 1, 2, 5 and 10.
    #[arg(long, default_value_t = 2)]
    step_ms: u64,
    /// Exercise packed observations and warm handoffs.
    #[arg(long)]
    packed: bool,
    /// Retain every measured state and native contact point.
    #[arg(long)]
    frames: bool,
    /// Fresh directory for the stationary and moving frame reports.
    out: PathBuf,
}
fn seeds(speed: f64) -> [Body; 2] {
    let slab = Body {
        id: 1,
        primitive: Primitive::Box,
        tier: Tier::Prescribed,
        size: [10., 2., 0.4],
        mass: 10.,
        friction: 0.65,
        position: [0., 0., -0.2],
        orientation: [0., 0., 0., 1.],
        linear: [speed, 0., 0.],
        angular: [0.; 3],
    };
    let sphere = Body {
        id: 2,
        primitive: Primitive::Sphere,
        tier: Tier::Dynamic,
        size: [2. * RADIUS; 3],
        mass: MASS,
        friction: MU,
        position: [0., 0., 0.35],
        ..slab.clone()
    };
    [slab, sphere]
}
#[derive(Debug, Serialize, PartialEq)]
struct Physical {
    bodies: [Body; 2],
    pairs: Vec<ContactImpulse>,
    floors: Vec<GroundSupport>,
}
impl Physical {
    fn read(f: &PoolFrame) -> Self {
        Self {
            bodies: [f.body(1).unwrap().1.clone(), f.body(2).unwrap().1.clone()],
            pairs: f
                .workers
                .iter()
                .flat_map(|w| w.snapshot.impulses.clone())
                .collect(),
            floors: f
                .workers
                .iter()
                .flat_map(|w| w.snapshot.supports.clone())
                .collect(),
        }
    }
}
fn slip(b: &Body, speed: f64) -> f64 {
    b.linear[0] - speed - RADIUS * b.angular[1]
}
fn energy(b: &Body, speed: f64) -> f64 {
    0.5 * MASS * (b.linear[0] - speed).powi(2) + 0.5 * INERTIA * b.angular[1].powi(2)
}
fn require_close(name: &str, actual: f64, expected: f64, tolerance: f64, tick: u64) -> Result<()> {
    if !actual.is_finite() || (actual - expected).abs() > tolerance {
        return Err(format!("{name} at tick {tick}: observed {actual:.16e}, prediction {expected:.16e}, tolerance {tolerance:e}").into());
    }
    Ok(())
}
#[derive(Default, Serialize)]
struct Measures {
    point_model_different_body_frames: usize,
    point_model_different_normal_frames: usize,
    first_point_model_difference_tick: Option<u64>,
    max_point_model_position_difference_m: f64,
    max_point_model_linear_difference_m_s: f64,
    max_point_model_angular_difference_rad_s: f64,
    max_point_model_quaternion_component_difference: f64,
    max_horizontal_balance_n_s: f64,
    max_vertical_balance_n_s: f64,
    max_coulomb_cone_excess_n_s: f64,
    max_sliding_saturation_error_n_s: f64,
    saturated_observations: usize,
    active_finite_contacts: usize,
    max_abs_angular_balance_n_m_s: f64,
    tangent_n_s: f64,
    normal_n_s: f64,
    table_work_j: f64,
    rolling_time_after_drive_s: f64,
    relative_speed_m_s: f64,
    angular_speed_rad_s: f64,
    final_slip_m_s: f64,
    relative_displacement_m: f64,
    dissipated_relative_energy_j: f64,
    actuator_work_balance_error_j: f64,
}
fn set_wire(pool: &mut WorkerPool, packed: bool) {
    pool.set_packed_handoffs(packed);
    pool.set_packed_observations(packed);
}
fn configured_pool(
    worker: &Path,
    seeds: &[Body; 2],
    step: SimulationStep,
    packed: bool,
) -> Result<WorkerPool> {
    let mut pool = WorkerPool::spawn(
        worker,
        0x524d,
        vec![seeds.to_vec(), vec![], vec![], vec![]],
        step,
        None,
        4096,
    )?;
    set_wire(&mut pool, packed);
    Ok(pool)
}
struct Faults<'a> {
    worker: &'a Path,
    scratch: &'a Path,
    packed: bool,
    events: Vec<Value>,
}
impl Faults<'_> {
    fn event(&mut self, tick: u64, kind: &str, details: Value) {
        self.events
            .push(json!({"tick":tick,"operation":kind,"details":details}));
    }
    fn advance(&mut self, pool: &mut WorkerPool, tick: u64, ms: u64) -> Result<()> {
        if matches!(ms, 260 | 660) {
            let owner = pool.frame().body(2).unwrap().0;
            let old = pool.frame().clone();
            let pid = pool.worker_pid(owner);
            pool.drop_next_acknowledgement(owner)?;
            if !matches!(pool.advance_in_gravity(FIELD), Err(PoolError::Transport(_)))
                || pool.frame() != &old
            {
                return Err("lost acknowledgement advanced a published barrier".into());
            }
            pool.advance_in_gravity(FIELD)?;
            if pool.worker_pid(owner) != pid {
                return Err("a healthy owner restarted after its lost reply".into());
            }
            self.event(
                tick,
                "lost acknowledgment and cached retry",
                json!({"owner":owner,"pid":pid}),
            );
        } else {
            pool.advance_in_gravity(FIELD)?;
        }
        Ok(())
    }
    fn after(&mut self, mut pool: WorkerPool, tick: u64, ms: u64) -> Result<WorkerPool> {
        let owner = pool.frame().body(2).unwrap().0;
        if matches!(ms, 240 | 640) {
            let old = pool.frame().clone();
            let before = Physical::read(&old);
            let members = &old.workers[owner as usize]
                .components
                .iter()
                .find(|c| c.ids.contains(&2))
                .unwrap()
                .ids;
            if members.as_slice() != [1, 2]
                || before.pairs.len() != 1
                || before.pairs[0].normal_impulse <= 0.
            {
                return Err("attempted transfer without a loaded table and sphere".into());
            }
            let to = (owner + 1) % 4;
            pool.transfer_component(2, to)?;
            if Physical::read(pool.frame()) != before {
                return Err("transfer changed physical data".into());
            }
            for id in members {
                if pool.frame().body(*id).unwrap().0 != to
                    || pool.frame().workers[to as usize].generations[id]
                        != old.workers[owner as usize].generations[id] + 1
                {
                    return Err("transfer changed incorrect membership or generation".into());
                }
            }
            self.event(
                tick,
                "loaded whole-component migration",
                json!({"source":owner,"destination":to,"ids":members}),
            );
        }
        if matches!(ms, 300 | 700) {
            let old = pool.frame().clone();
            let pid = pool.worker_pid(owner);
            pool.terminate_worker(owner)?;
            pool.recover(owner)?;
            if pool.frame() != &old || pool.worker_pid(owner) == pid {
                return Err("process replacement changed the public barrier".into());
            }
            self.event(
                tick,
                "terminated and recovered loaded owner",
                json!({"owner":owner,"before_pid":pid,"after_pid":pool.worker_pid(owner)}),
            );
        }
        if matches!(ms, 350 | 750) {
            let old = pool.frame().clone();
            let path = self.scratch.join(format!("warm-{ms}.gzpool"));
            pool.save_native_recovery(&path)?;
            drop(pool);
            pool = PoolNativeRecovery::open(&path)?.resume(self.worker)?;
            set_wire(&mut pool, self.packed);
            if pool.frame() != &old {
                return Err("disk recovery changed the public barrier".into());
            }
            self.event(
                tick,
                "full native pool reloaded",
                json!({"workers":4,"archive_bytes":fs::metadata(path)?.len()}),
            );
        }
        Ok(pool)
    }
}
#[derive(Serialize)]
struct MechanicsFrame<'a> {
    native_reaction: &'a native::Reaction,
    distributed_horizontal_contact_impulse_n_s: f64,
    slip_m_s: f64,
    com_impulse_n_s: f64,
    relative_energy_j: f64,
    dissipated_relative_energy_j: f64,
    table_work_j: f64,
}
fn record(
    out: &mut impl Write,
    pool: &WorkerPool,
    physical: &Physical,
    state: &[u8],
    ms: u64,
    mechanics: MechanicsFrame<'_>,
) -> Result<()> {
    let f = pool.frame();
    let owners = [f.body(1).unwrap().0, f.body(2).unwrap().0];
    let generations = owners
        .iter()
        .enumerate()
        .map(|(i, o)| f.workers[*o as usize].generations[&(i as u64 + 1)])
        .collect::<Vec<_>>();
    let mut row = json!({
        "tick":f.tick, "time_ns":ms*1_000_000, "bodies":physical.bodies, "pairs":physical.pairs, "floors":physical.floors,
        "owners":owners, "generations":generations,"components":f.workers.iter().map(|w|&w.components).collect::<Vec<_>>(),
        "trajectory_sha256_sample":format!("{:x}",Sha256::digest(state))
    });
    row.as_object_mut()
        .unwrap()
        .append(serde_json::to_value(mechanics)?.as_object_mut().unwrap());
    serde_json::to_writer(&mut *out, &row)?;
    writeln!(out)?;
    Ok(())
}
fn max_difference<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(u, v)| (u - v).abs())
        .fold(0., f64::max)
}
fn run(
    worker: &Path,
    step: SimulationStep,
    speed: f64,
    packed: bool,
    save_frames: bool,
    out: &Path,
) -> Result<Measures> {
    fs::create_dir(out)?;
    let scratch = tempfile::tempdir_in(out)?;
    let mut faults = Faults {
        worker,
        scratch: scratch.path(),
        packed,
        events: vec![],
    };
    let dt = step.duration().as_secs_f64();
    let h = step.duration().as_millis() as u64;
    let mut seeds = seeds(speed);
    let mut control = native::Native::new(&seeds, dt, false);
    let mut point_bodies = seeds.clone();
    let mut point_control = native::Native::new(&point_bodies, dt, true);
    let mut pool = configured_pool(worker, &seeds, step, packed)?;
    let mut frames = if save_frames {
        Some(BufWriter::new(File::create_new(out.join("frames.jsonl"))?))
    } else {
        None
    };
    let mut digest = Sha256::new();
    let mut previous = seeds[1].clone();
    let mut m = Measures::default();
    let mut window = 0u64;
    let mut first_rolling = None;
    let mut kick_state = previous.clone();
    let mut kick_position = 0.;
    if let Some(out) = &mut frames {
        let initial = Physical::read(pool.frame());
        let state = bincode::serialize(&(0u64, &initial.bodies, &initial.pairs, &initial.floors))?;
        record(
            out,
            &pool,
            &initial,
            &state,
            0,
            MechanicsFrame {
                native_reaction: &native::Reaction::default(),
                distributed_horizontal_contact_impulse_n_s: 0.,
                slip_m_s: slip(&previous, speed),
                com_impulse_n_s: 0.,
                relative_energy_j: energy(&previous, speed),
                dissipated_relative_energy_j: 0.,
                table_work_j: 0.,
            },
        )?;
    }
    for tick in 1..=END_MS / h {
        let ms = tick * h;
        let input = if ms == DRIVE_MS + h { KICK } else { 0. };
        if input != 0. {
            kick_state = previous.clone();
            kick_state.linear[0] += input / MASS;
            kick_position = kick_state.position[0];
            pool.queue_impulse_in_gravity(2, [input, 0., 0.], FIELD)?;
            control.kick([input, 0., 0.]);
            point_control.kick([input, 0., 0.]);
            faults.event(
                tick,
                "sphere center impulse",
                json!({"body":2,"impulse_n_s":[input,0.,0.]}),
            );
        }
        faults.advance(&mut pool, tick, ms)?;
        pool = faults.after(pool, tick, ms)?;
        let physical = Physical::read(pool.frame());
        let default_reaction = control.step(&mut seeds);
        let reaction = point_control.step(&mut point_bodies);
        let different_body = seeds != point_bodies;
        let different_normal = default_reaction.active != reaction.active
            || default_reaction.normal_n_s.to_bits() != reaction.normal_n_s.to_bits();
        m.point_model_different_body_frames += usize::from(different_body);
        m.point_model_different_normal_frames += usize::from(different_normal);
        if different_body || different_normal {
            m.first_point_model_difference_tick.get_or_insert(tick);
        }
        for (a, b) in seeds.iter().zip(&point_bodies) {
            m.max_point_model_position_difference_m = m
                .max_point_model_position_difference_m
                .max(max_difference(a.position, b.position));
            m.max_point_model_linear_difference_m_s = m
                .max_point_model_linear_difference_m_s
                .max(max_difference(a.linear, b.linear));
            m.max_point_model_angular_difference_rad_s = m
                .max_point_model_angular_difference_rad_s
                .max(max_difference(a.angular, b.angular));
            m.max_point_model_quaternion_component_difference = m
                .max_point_model_quaternion_component_difference
                .max(max_difference(a.orientation, b.orientation));
        }
        if reaction.points.len() > 1 || default_reaction.points.len() > 1 {
            return Err(
                "more than one finite sphere-plane point prevents unique tangential attribution"
                    .into(),
            );
        }
        let expected = default_reaction.active.then_some(ContactImpulse {
            a: 1,
            b: 2,
            normal_impulse: default_reaction.normal_n_s,
        });
        if physical.bodies != seeds
            || physical.pairs.as_slice() != expected.as_slice()
            || !physical.floors.is_empty()
        {
            fs::write(
                out.join("native-mismatch.json"),
                serde_json::to_vec_pretty(
                    &json!({"tick":tick,"physical":physical,"direct_native_bodies":seeds,"direct_native_reaction":reaction}),
                )?,
            )?;
            return Err(format!("independent native/public mismatch at {tick}").into());
        }
        let state =
            bincode::serialize(&(tick, &physical.bodies, &physical.pairs, &physical.floors))?;
        digest.update(&state);
        let sphere = &physical.bodies[1];
        let jx = reaction.tangent_on_sphere[0];
        if tick == DRIVE_MS / h + 1 {
            fs::write(
                out.join("first-slide.json"),
                serde_json::to_vec_pretty(
                    &json!({"tick":tick,"distributed":physical,"native_default":default_reaction,"direct_coulomb":reaction,"coulomb_bodies":point_bodies}),
                )?,
            )?;
        }
        let momentum_x = MASS * (sphere.linear[0] - previous.linear[0]) - input;
        let rz = MASS * (sphere.linear[2] - previous.linear[2] + G * dt)
            - reaction.normal_on_sphere[2]
            - reaction.tangent_on_sphere[2];
        let rx = momentum_x - jx - reaction.normal_on_sphere[0];
        m.max_horizontal_balance_n_s = m.max_horizontal_balance_n_s.max(rx.abs());
        m.max_vertical_balance_n_s = m.max_vertical_balance_n_s.max(rz.abs());
        require_close("signed horizontal reaction balance", rx, 0., 1e-9, tick)?;
        require_close("vertical reaction balance", rz, 0., 1e-9, tick)?;
        let cone = reaction
            .tangent_on_sphere
            .iter()
            .map(|x| x * x)
            .sum::<f64>()
            .sqrt()
            - MU * reaction.normal_n_s;
        m.max_coulomb_cone_excess_n_s = m.max_coulomb_cone_excess_n_s.max(cone);
        require_close("Coulomb cone excess", cone.max(0.), 0., 1e-8, tick)?;
        let before_slip = slip(&previous, speed) + input / MASS;
        let after_slip = slip(sphere, speed);
        if reaction.active {
            m.active_finite_contacts += 1;
        }
        if ms > DRIVE_MS {
            m.normal_n_s += reaction.normal_n_s;
            m.tangent_n_s += jx;
            m.table_work_j += jx * speed;
            let angular_residual =
                INERTIA * (sphere.angular[1] - kick_state.angular[1]) + RADIUS * m.tangent_n_s;
            m.max_abs_angular_balance_n_m_s =
                m.max_abs_angular_balance_n_m_s.max(angular_residual.abs());
            require_close(
                "cumulative angular reaction balance",
                angular_residual,
                0.,
                0.003,
                tick,
            )?;
            if before_slip > 0.08 && after_slip > 0.04 {
                let saturated = jx + MU * reaction.normal_n_s;
                m.saturated_observations += 1;
                m.max_sliding_saturation_error_n_s =
                    m.max_sliding_saturation_error_n_s.max(saturated.abs());
                require_close("sliding Coulomb saturation", saturated, 0., 1e-6, tick)?;
            }
            if !reaction.active || reaction.normal_n_s <= 0. {
                return Err(format!("lost finite load after spin-up began at {tick}").into());
            }
            if after_slip.abs() <= 0.01 {
                window += h;
                if window >= 20 && first_rolling.is_none() {
                    let acquisition = (ms + h - window) / h;
                    first_rolling = Some(acquisition);
                    faults.event(
                        acquisition,
                        "rolling acquired",
                        json!({"confirmed_at_tick":tick,"slip_threshold_m_s":0.01}),
                    );
                }
            } else {
                if first_rolling.is_some() {
                    return Err("rolling slip exceeded threshold after acquisition".into());
                }
                window = 0;
            }
            m.dissipated_relative_energy_j = energy(&kick_state, speed) - energy(sphere, speed);
        }
        if let Some(out) = &mut frames {
            record(
                out,
                &pool,
                &physical,
                &state,
                ms,
                MechanicsFrame {
                    native_reaction: &reaction,
                    distributed_horizontal_contact_impulse_n_s: momentum_x,
                    slip_m_s: after_slip,
                    com_impulse_n_s: input,
                    relative_energy_j: energy(sphere, speed),
                    dissipated_relative_energy_j: m.dissipated_relative_energy_j,
                    table_work_j: m.table_work_j,
                },
            )?;
        }
        previous = sphere.clone();
    }
    if let Some(mut out) = frames {
        out.flush()?;
    }
    if m.point_model_different_body_frames != 0 || m.point_model_different_normal_frames != 0 {
        fs::write(
            out.join("point-model-difference.json"),
            serde_json::to_vec_pretty(&m)?,
        )?;
        return Err("per-point Coulomb control diverged from the default native trajectory".into());
    }
    let tick = END_MS / h;
    let transition = first_rolling.ok_or("slip never reached rolling")?;
    m.rolling_time_after_drive_s = (transition * h - DRIVE_MS) as f64 / 1000.;
    m.relative_speed_m_s = previous.linear[0] - speed;
    m.angular_speed_rad_s = previous.angular[1];
    m.final_slip_m_s = slip(&previous, speed);
    m.relative_displacement_m =
        previous.position[0] - kick_position - speed * ((END_MS - DRIVE_MS) as f64 / 1000.);
    m.actuator_work_balance_error_j = energy(&previous, 0.) - energy(&kick_state, 0.)
        + m.dissipated_relative_energy_j
        - m.table_work_j;
    require_close(
        "sliding-to-rolling time",
        m.rolling_time_after_drive_s,
        1. / (MU * G),
        2. * dt + 0.003,
        tick,
    )?;
    require_close(
        "rolling relative speed",
        m.relative_speed_m_s,
        2.5,
        0.01,
        tick,
    )?;
    require_close(
        "rolling angular speed",
        m.angular_speed_rad_s,
        2.5 / RADIUS,
        0.05,
        tick,
    )?;
    require_close(
        "signed total tangent impulse",
        m.tangent_n_s,
        -2.,
        0.02,
        tick,
    )?;
    require_close(
        "dissipated relative kinetic energy",
        m.dissipated_relative_energy_j,
        3.5,
        0.05,
        tick,
    )?;
    require_close(
        "moving table work",
        m.actuator_work_balance_error_j,
        0.,
        1e-8,
        tick,
    )?;
    if m.saturated_observations < 20 || faults.events.len() != 10 {
        return Err("missing sliding observations or transactional operations".into());
    }
    faults.events.sort_by_key(|e| e["tick"].as_u64());
    fs::write(
        out.join("summary.json"),
        serde_json::to_vec_pretty(
            &json!({"step_ms":h,"ticks":tick,"time_s":1.2,"drive_time_s":0.2,"table_speed_m_s":speed,"mass_kg":MASS,"radius_m":RADIUS,"inertia_kg_m2":INERTIA,"mu":MU,"packed":packed,"analytic":{"slip_mobility_inverse_kg":1./MASS+RADIUS*RADIUS/INERTIA,"relative_speed_m_s":2.5,"spin_rad_s":2.5/RADIUS,"tangent_n_s":-2.,"relative_dissipation_j":3.5,"slip_duration_s":1./(MU*G)},"observed":m,"events":faults.events,"trajectory_sha256":format!("{:x}",digest.finalize())}),
        )?,
    )?;
    Ok(m)
}
pub(crate) fn qualify(
    worker: &Path,
    step_ms: u64,
    packed: bool,
    frames: bool,
    out: &Path,
) -> Result<()> {
    let step = SimulationStep::from_duration(Duration::from_millis(step_ms))
        .ok_or("choose 1, 2, 5 or 10 ms")?;
    fs::create_dir(out)?;
    let rest = run(worker, step, 0., packed, frames, &out.join("stationary"))?;
    let moving = run(worker, step, 1., packed, frames, &out.join("moving"))?;
    let mut delta = vec![];
    for (name, a, b) in [
        (
            "co-moving sphere displacement",
            rest.relative_displacement_m,
            moving.relative_displacement_m,
        ),
        (
            "co-moving speed",
            rest.relative_speed_m_s,
            moving.relative_speed_m_s,
        ),
        ("spin", rest.angular_speed_rad_s, moving.angular_speed_rad_s),
    ] {
        require_close(name, a, b, 1e-6, END_MS / step_ms)?;
        delta.push(json!({"measurement":name,"absolute_difference":(a-b).abs()}));
    }
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"step_ms":step_ms,"packed":packed,"galilean_differences":delta,"stationary":rest,"moving":moving}),
        )?,
    )?;
    Ok(())
}
#[cfg_attr(test, allow(dead_code))]
fn main() -> Result<()> {
    let args = Args::parse();
    qualify(
        &args.worker,
        args.step_ms,
        args.packed,
        args.frames,
        &args.out,
    )
}
