// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Head-on and glancing impacts, an untouched airborne island, and recovery at positive contact.
#[path = "support/momentum_audit.rs"]
mod momentum;
#[path = "support/native_contact_control.rs"]
mod native;
#[path = "support/planar_impact.rs"]
mod planar;
use clap::Parser;
use gz_physics_distributed::{
    physics::{Body, BodyWrench, GroundPlane, PendingImpulse, Primitive, SimulationStep, fixture},
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use serde::Serialize;
use std::io::{BufWriter, Write};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const G: [f64; 3] = [0., 0., -9.81];
const END: u64 = 150;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long,default_value_t=4,value_parser=clap::value_parser!(u8).range(3..=8))]
    width: u8,
    /// Offset the centers sideways by 0.20 m and add a known spin torque.
    #[arg(long)]
    glancing: bool,
    /// Run 64 glancing collisions at eight offsets and eight sub-tick arrival phases.
    #[arg(long, conflicts_with = "glancing")]
    sweep: bool,
    out: PathBuf,
}
#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "snake_case")]
enum Rate {
    Coarse,
    Scene,
    Island,
}
#[derive(Clone, Copy)]
struct Geometry {
    offset_y: f64,
    shift_x: f64,
}
impl Geometry {
    fn glancing(self) -> bool {
        self.offset_y != 0.
    }
}
fn make(worker: &Path, width: u8, geometry: Geometry) -> Result<WorkerPool> {
    let seeds = vec![
        (0, 1, -1., 0.25, 1.5, 2.),
        (1, 2, 1., 0.25, -0.75, 1.),
        (2, 3, 100., 50., 0., 1.),
    ];
    let mut bodies = vec![vec![]; width.into()];
    for (w, id, x, z, v, m) in seeds {
        bodies[w].push(Body {
            id,
            primitive: Primitive::Sphere,
            size: [0.5; 3],
            mass: m,
            position: [
                if id == 2 { x + geometry.shift_x } else { x },
                if id == 2 { geometry.offset_y } else { 0. },
                z,
            ],
            linear: [v, 0., 0.],
            angular: [
                0.,
                0.,
                if geometry.glancing() && id == 1 {
                    1.25
                } else {
                    0.
                },
            ],
            friction: 0.,
            ..fixture()[1].clone()
        });
    }
    let mut p = WorkerPool::spawn(
        worker,
        19002,
        bodies,
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        48,
    )?;
    p.set_handoff_confirmation(true);
    Ok(p)
}
fn step(p: &mut WorkerPool, rate: Rate, loads: &[BodyWrench]) -> Result<()> {
    match rate {
        Rate::Coarse => p.advance_in_field(G, loads)?,
        Rate::Scene => p.advance_adaptive_in_field(G, loads)?,
        Rate::Island => p.advance_islands_in_field(G, loads)?,
    };
    Ok(())
}
fn inputs(
    p: &mut WorkerPool,
    t: u64,
    glancing: bool,
) -> Result<(Vec<BodyWrench>, Vec<PendingImpulse>)> {
    let queued = match t {
        12 => vec![PendingImpulse {
            id: 1,
            impulse: [0.25, -0., 0.],
        }],
        16 => vec![PendingImpulse {
            id: 2,
            impulse: [0.15, 0., 0.],
        }],
        _ => vec![],
    };
    for q in &queued {
        p.queue_impulse(q.id, q.impulse)?;
    }
    let loads = if t == 20 {
        vec![BodyWrench {
            id: 1,
            force: [6., 0., 0.],
            torque: [0., 0., if glancing { 0.4 } else { 0. }],
        }]
    } else {
        vec![]
    };
    Ok((loads, queued))
}
fn active_pair(frame: &PoolFrame) -> f64 {
    frame
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.impulses)
        .filter(|c| [c.a, c.b] == [1, 2] || [c.a, c.b] == [2, 1])
        .map(|p| p.normal_impulse)
        .sum()
}
fn record(out: &mut impl Write, frame: &PoolFrame, event: &str) -> Result<()> {
    let members: Vec<_> = [1, 2, 3]
        .map(|id| {
            let (owner, body) = frame.body(id).unwrap();
            serde_json::json!({"owner":owner,"body":body})
        })
        .into_iter()
        .collect();
    let pairs: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.impulses)
        .collect();
    let floors: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.supports)
        .collect();
    serde_json::to_writer(
        &mut *out,
        &serde_json::json!({"tick":frame.tick,"event":event,"members":members,
        "pairs":pairs,"floors":floors,"planar":planar::Budget::read(frame)}),
    )?;
    writeln!(out)?;
    Ok(())
}
fn trial(args: &Args, rate: Rate, geometry: Geometry, out: &Path) -> Result<serde_json::Value> {
    let mut control = make(&args.worker, args.width, geometry)?;
    let mut expected = vec![control.frame().clone()];
    let mut routed = None;
    let mut touch = None;
    let mut pair_impulse = 0.;
    let mut free_steps = 0;
    let mut pre_handoff = None;
    let mut destination = None;
    let initial_pair = [1, 2].map(|id| control.frame().body(id).unwrap().1.clone());
    let prediction = planar::collision(&initial_pair);
    let ground = Some(GroundPlane::new(0.)?);
    let mut direct = native::Control::new(&initial_pair, ground, G)?;
    let mut direct_air =
        native::Control::new(&[control.frame().body(3).unwrap().1.clone()], ground, G)?;
    let mut native_steps = 0;
    let mut before_impact = None;
    for t in 1..=END {
        let (loads, pending) = inputs(&mut control, t, geometry.glancing())?;
        step(&mut control, rate, &loads)?;
        let substeps = control.last_substeps().unwrap();
        let (pairs, floors) = direct.advance(substeps, &loads, &pending)?;
        let air_steps = if matches!(rate, Rate::Scene) {
            substeps
        } else {
            1
        };
        let (_, air_floor) = direct_air.advance(air_steps, &[], &[])?;
        if !air_floor.is_empty()
            || direct
                .bodies
                .iter()
                .chain(&direct_air.bodies)
                .any(|b| control.frame().body(b.id).unwrap().1 != b)
            || pairs
                .iter()
                .map(|p| p.normal_impulse)
                .sum::<f64>()
                .to_bits()
                != active_pair(control.frame()).to_bits()
            || floors.iter().any(|floor| {
                !control
                    .frame()
                    .workers
                    .iter()
                    .flat_map(|w| &w.snapshot.supports)
                    .any(|r| r == floor)
            })
        {
            return Err(
                format!("direct native state or impulse diverged at {t} in {rate:?}").into(),
            );
        }
        native_steps += 1;
        if t == 25 {
            before_impact = Some(planar::Budget::read(control.frame()));
        }
        let frame = control.frame();
        if routed.is_none() && frame.body(1).unwrap().0 == frame.body(2).unwrap().0 {
            routed = Some(t);
        }
        let j = active_pair(frame);
        pair_impulse += j;
        if j > 0. && touch.is_none() {
            touch = Some(t);
        }
        if matches!(rate, Rate::Island)
            && !control.last_island_refined_bodies().unwrap().contains(&3)
        {
            free_steps += 1;
        }
        if Some(t) == touch {
            pre_handoff = Some(frame.clone());
            let dest = (frame.body(1).unwrap().0 + 1) % args.width;
            destination = Some(dest);
            control.transfer_component(1, dest)?;
            if control.frame().body(2).unwrap().0 != dest
                || active_pair(control.frame()).to_bits() != j.to_bits()
            {
                return Err("positive impact contact did not travel with both bodies".into());
            }
        }
        expected.push(control.frame().clone());
    }
    let routed = routed.ok_or("no automatic migration")?;
    let touch = touch.ok_or("no positive impact")?;
    if routed > touch {
        return Err("impact preceded colocation".into());
    }
    drop(control);
    let prefix = format!("{rate:?}").to_lowercase();
    let mut p = make(&args.worker, args.width, geometry)?;
    let mut audit =
        momentum::Audit::new(p.frame(), Some(&out.join(format!("{prefix}-ledger.jsonl"))))?;
    let mut samples = BufWriter::new(fs::File::create_new(
        out.join(format!("{prefix}-frames.jsonl")),
    )?);
    record(&mut samples, p.frame(), "initial")?;
    let before_impact = before_impact.unwrap();
    let mut angular_expected = planar::Budget::read(p.frame()).angular_z_nms;
    let mut peak_angular_error: f64 = 0.;
    let mut fault_owner = None;
    let mut killed_receiver = None;
    let mut edited = false;
    let mut synthetic_floor = false;
    for t in 1..=END {
        let (loads, queued) = inputs(&mut p, t, geometry.glancing())?;
        angular_expected += planar::external(p.frame(), &loads, &queued);
        // Independent negative controls omit a genuine queued impulse or remove
        // a positive published ground reaction. Both ledgers must detect an error.
        let mut omission = (t == 12 || t == 30)
            .then(|| momentum::Audit::new(p.frame(), None))
            .transpose()?;
        if t == routed {
            let prior = p.frame().clone();
            let source = [1, 2]
                .into_iter()
                .find(|&id| prior.body(id).unwrap().0 != expected[t as usize].body(id).unwrap().0)
                .ok_or("missing routed source")?;
            let owner = prior.body(source).unwrap().0;
            let pids: Vec<_> = (0..args.width).map(|w| p.worker_pid(w)).collect();
            // Worker 2 owns only the distant sphere and receives the final solve.
            p.drop_next_acknowledgement(2)?;
            if step(&mut p, rate, &loads).is_ok() || p.frame() != &prior {
                return Err("speculative solve published before confirmation".into());
            }
            p.terminate_worker(owner)?;
            step(&mut p, rate, &loads)?;
            for w in 0..args.width {
                if (p.worker_pid(w) != pids[w as usize]) != (w == owner) {
                    return Err("unexpected routed-step restart scope".into());
                }
            }
            fault_owner = Some(owner);
        } else {
            step(&mut p, rate, &loads)?;
        }
        let before_move = if t == touch {
            pre_handoff.as_ref().unwrap()
        } else {
            &expected[t as usize]
        };
        if p.frame() != before_move {
            return Err(format!("faulted {rate:?} physics changed at {t}").into());
        }
        audit.advance(p.frame(), G, &loads, &queued)?;
        record(&mut samples, p.frame(), "step")?;
        peak_angular_error = peak_angular_error
            .max((planar::Budget::read(p.frame()).angular_z_nms - angular_expected).abs());
        if t == 12 {
            edited = omission
                .take()
                .unwrap()
                .advance(p.frame(), G, &loads, &[])
                .is_err();
        }
        if t == 30 {
            let mut wrong = p.frame().clone();
            let supports = wrong
                .workers
                .iter_mut()
                .find(|w| w.snapshot.supports.iter().any(|r| r.normal_impulse > 0.))
                .ok_or("no positive floor receipt")?;
            supports.snapshot.supports.clear();
            synthetic_floor = omission
                .take()
                .unwrap()
                .advance(&wrong, G, &loads, &queued)
                .is_err();
        }
        if t == touch {
            let dest = destination.unwrap();
            let ids: Vec<_> = (0..args.width).map(|w| p.worker_pid(w)).collect();
            p.terminate_after_command(dest, 0)?;
            p.transfer_component(1, dest)?;
            if p.frame() != &expected[t as usize]
                || (0..args.width).any(|w| (ids[w as usize] != p.worker_pid(w)) != (w == dest))
            {
                return Err(
                    "warm impact handoff changed state or restarted another endpoint".into(),
                );
            }
            killed_receiver = Some(dest);
            audit.same_tick(p.frame())?;
            record(&mut samples, p.frame(), "warm_handoff_after_prepare_death")?;
            let path = out.join(format!("{prefix}.gzpool"));
            p.save_native_recovery(&path)?;
            drop(p);
            p = PoolNativeRecovery::open(&path)?.resume(&args.worker)?;
            if p.frame() != &expected[t as usize] {
                return Err("changed contact on durable restore".into());
            }
            audit.same_tick(p.frame())?;
            record(&mut samples, p.frame(), "file_resume")?;
        }
    }
    samples.flush()?;
    let audit = audit.finish()?;
    if !edited || !synthetic_floor || audit.peak_worker_flux_ns < 0.5 || audit.moved_bodies == 0 {
        return Err("missing physical negative control or transport impulse".into());
    }
    let final_budget = planar::Budget::read(p.frame());
    let energy_loss = before_impact.kinetic_horizontal_j - final_budget.kinetic_horizontal_j;
    let final_xy = [1, 2].map(|id| {
        let v = p.frame().body(id).unwrap().1.linear;
        [v[0], v[1]]
    });
    let velocity_error = final_xy
        .iter()
        .flatten()
        .zip(prediction.horizontal_m_s.iter().flatten())
        .map(|(a, b)| (a - b).abs())
        .fold(0., f64::max);
    let impulse_error = (pair_impulse - prediction.impact_ns).abs();
    let energy_error = (energy_loss - prediction.dissipated_j).abs();
    if ![
        energy_loss,
        velocity_error,
        impulse_error,
        energy_error,
        peak_angular_error,
    ]
    .into_iter()
    .all(f64::is_finite)
        || energy_loss <= 0.
        || audit.same_tick_checks != 2
        || audit.moved_bodies < 3
        || (!geometry.glancing()
            && [
                velocity_error,
                impulse_error,
                energy_error,
                peak_angular_error,
            ]
            .into_iter()
            .any(|e| e > 1e-10))
        || (geometry.glancing() && !(final_xy[0][1] < 0. && final_xy[1][1] > 0.))
    {
        return Err("inelastic collision or same-tick conservation test failed".into());
    }
    Ok(
        serde_json::json!({"rate":rate,"width":args.width,"glancing":geometry.glancing(),"offset_y_m":geometry.offset_y,"shift_x_m":geometry.shift_x,"direct_native_steps":native_steps,"routed_tick":routed,"first_positive_impact_tick":touch,
        "failed_routing_source":fault_owner,"failed_warm_receiver":killed_receiver,"ticks_verified":END,"unrefined_airborne_ticks":free_steps,
        "impact_pair_impulse_ns":pair_impulse,"continuum":prediction,"final_xy_m_s":final_xy,
        "measured_loss_j":energy_loss,"abs_impulse_error_ns":impulse_error,"abs_energy_error_j":energy_error,
        "max_velocity_error_m_s":velocity_error,"peak_vertical_axis_angular_error_nms":peak_angular_error,
        "missing_queue_detected":edited,"missing_floor_detected":synthetic_floor,"momentum":audit}),
    )
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.out)?;
    let run = |geometry, out: &Path| -> Result<Vec<_>> {
        [Rate::Coarse, Rate::Scene, Rate::Island]
            .into_iter()
            .map(|r| trial(&args, r, geometry, out))
            .collect()
    };
    if args.sweep {
        let mut output = BufWriter::new(fs::File::create_new(args.out.join("cases.jsonl"))?);
        let mut count = 0;
        for offset_y in [0.025, 0.075, 0.15, 0.225, 0.30, 0.375, 0.425, 0.465] {
            for phase in 0..8 {
                let geometry = Geometry {
                    offset_y,
                    shift_x: f64::from(phase) * 0.02255 / 8.,
                };
                let scratch = tempfile::tempdir_in(&args.out)?;
                let rows = run(geometry, scratch.path())?;
                serde_json::to_writer(&mut output, &rows)?;
                writeln!(output)?;
                output.flush()?;
                count += rows.len();
            }
        }
        fs::write(
            args.out.join("count.json"),
            serde_json::to_vec(
                &serde_json::json!({"runs":count,"steps_per_run":END,"offsets":8,"phases":8}),
            )?,
        )?;
    } else {
        let geometry = Geometry {
            offset_y: if args.glancing { 0.20 } else { 0. },
            shift_x: 0.,
        };
        let reports = run(geometry, &args.out)?;
        fs::write(
            args.out.join("summary.json"),
            serde_json::to_vec_pretty(&reports)?,
        )?;
    }
    Ok(())
}
