// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Braking and arrest of pure yaw through a measured finite contact patch.
#[allow(dead_code)]
#[path = "support/native_contact_control.rs"]
mod native;
use clap::Parser;
use gz_physics::DynamicBackend;
use gz_physics_distributed::{
    physics::{Body, FiniteContactPatch, Primitive, SimulationStep, Tier},
    pool::{PoolNativeRecovery, WorkerPool},
};
use rapier3d_f64::prelude::{Rotation, Vector};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const G: [f64; 3] = [0., 0., -9.81];
const MASS: f64 = 2.;
const KICK: f64 = 0.96;
#[derive(Parser)]
pub(crate) struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    rectangular: bool,
    #[arg(long)]
    reverse: bool,
    #[arg(long)]
    moving: bool,
    #[arg(long)]
    frictionless: bool,
    #[arg(long)]
    coarse: bool,
    #[arg(long)]
    packed: bool,
    #[arg(long)]
    frames: bool,
    out: PathBuf,
}
fn bodies(a: &Args) -> [Body; 2] {
    let speed = f64::from(u8::from(a.moving));
    let table = Body {
        id: 1,
        primitive: Primitive::Box,
        tier: Tier::Prescribed,
        size: [6., 3., 0.4],
        mass: 10.,
        friction: 0.65,
        position: [0., 0., -0.2],
        orientation: [0., 0., 0., 1.],
        linear: [speed, 0., 0.],
        angular: [0.; 3],
    };
    let block = Body {
        id: 2,
        tier: Tier::Dynamic,
        size: if a.rectangular {
            [0.8, 0.4, 0.2]
        } else {
            [0.6, 0.6, 0.2]
        },
        mass: MASS,
        friction: if a.frictionless { 0. } else { 0.35 },
        position: [0., 0., 0.15],
        ..table.clone()
    };
    [table, block]
}
/// Actual spatial angular momentum around the moving body's center, in world coordinates.
fn spin(body: &Body) -> Vector {
    let [x, y, z] = body.size;
    let [qx, qy, qz, qw] = body.orientation;
    let r = Rotation::from_xyzw(qx, qy, qz, qw);
    let inert = Vector::new(y * y + z * z, x * x + z * z, x * x + y * y) * (body.mass / 12.);
    r * (inert * (r.inverse() * Vector::from(body.angular)))
}
#[cfg_attr(test, allow(dead_code))]
fn main() -> Result<()> {
    run(Args::parse())
}

pub(crate) fn run(a: Args) -> Result<()> {
    fs::create_dir(&a.out)?;
    let seeds = bodies(&a);
    let mut control = native::Control::new(&seeds, None, G)?;
    let worker = &a.worker;
    let mut p = WorkerPool::spawn(
        worker,
        0x545743,
        vec![seeds.to_vec(), vec![], vec![], vec![]],
        SimulationStep::Ms10,
        None,
        128,
    )?;
    let configure = |p: &mut WorkerPool| {
        p.set_packed_handoffs(a.packed);
        p.set_packed_observations(a.packed);
    };
    configure(&mut p);
    let mut log = a
        .frames
        .then(|| File::create_new(a.out.join("frames.jsonl")))
        .transpose()?
        .map(BufWriter::new);
    let mut digest = Sha256::new();
    let [x, y, _] = seeds[1].size;
    let radius = 0.5 * x.hypot(y);
    let mu = seeds[1].friction;
    let sign = if a.reverse { -1. } else { 1. };
    let ideal_torque = mu * MASS * 9.81 * radius;
    let ideal_stop = (!a.frictionless).then(|| KICK / ideal_torque);
    let mut peak_patch_error = 0f64;
    let mut saturation = vec![];
    let mut stops = 0u64;
    let mut first_stop = None;
    let mut max_com_error = 0f64;
    let mut max_point_normal_error = 0f64;
    let mut total_reaction = 0.;
    let mut point_hist = std::collections::BTreeMap::new();
    let mut all_events = vec![];
    let mut inspected_images = 0;
    let scratch = tempfile::tempdir_in(&a.out)?;
    let mut rates = std::collections::BTreeMap::new();
    let mut initial_speed = 0.;
    let mut last_speed = 0.;
    for tick in 1u64..=120 {
        let mut events = vec![];
        let mut omega_before = p.frame().body(2).unwrap().1.angular[2];
        if tick == 31 {
            let before = spin(p.frame().body(2).unwrap().1);
            p.apply_impulse_in_field(2, [0.; 3], [0., 0., sign * KICK], G, &[])?;
            control
                .solver
                .apply_impulse(control.handles[&2], [0.; 3], [0., 0., sign * KICK])?;
            let after = spin(p.frame().body(2).unwrap().1);
            if (after - before - Vector::new(0., 0., sign * KICK)).length() > 1e-9 {
                return Err("angular impulse failed the independent inertia balance".into());
            }
            omega_before = p.frame().body(2).unwrap().1.angular[2];
            initial_speed = omega_before;
            events.push(json!({"kind":"angular impulse","angular_nms":[0.,0.,sign*KICK]}));
        }
        let h0 = spin(p.frame().body(2).unwrap().1);
        let owner = p.frame().body(2).unwrap().0;
        let advance = |pool: &mut WorkerPool| -> Result<()> {
            if a.coarse {
                pool.advance_in_gravity(G)?;
            } else {
                pool.advance_adaptive_in_field(G, &[])?;
            }
            Ok(())
        };
        if [35, 94].contains(&tick) {
            let old = p.frame().clone();
            let pid = p.worker_pid(owner);
            p.drop_next_acknowledgement(owner)?;
            if advance(&mut p).is_ok() || p.frame() != &old {
                return Err("ambiguous step was published".into());
            }
            advance(&mut p)?;
            if p.worker_pid(owner) != pid {
                return Err("lost receipt replaced a living owner".into());
            }
            events.push(json!({"kind":"cached final step","owner":owner,"pid":pid}));
        } else {
            advance(&mut p)?;
        }
        let rate = p.last_substeps().ok_or("missing native rate")?;
        *rates.entry(rate).or_insert(0) += 1;
        let mut cap = 0.;
        let mut point_normal = 0.;
        let mut count = 0;
        let mut every_four = true;
        let mut points = vec![];
        let mut microstep = 0;
        let inspect = [36, 37, 38, 95, 96, 97].contains(&tick);
        let mut reference_patch = vec![];
        let (contacts,floor)=control.advance_observing(rate,&[],&[],|engine|{
            let mut n=0;
            for patch in engine.rigid_contact_patches(){
                if inspect && microstep+1==rate {
                    reference_patch.push(FiniteContactPatch{a:seeds[patch.a.raw() as usize].id,b:seeds[patch.b.raw() as usize].id,normal_on_a:patch.normal_on_a,points:patch.points.clone()});
                }
                let count=patch.points.len();
                let center=patch.points.iter().map(|r|Vector::from(r.lever_a)).sum::<Vector>()/count as f64;
                for r in patch.points{
                    let radius=(Vector::from(r.lever_a)-center).length();
                    point_normal+=r.normal_impulse;
                    cap+=mu*radius*r.normal_impulse;
                    if a.frames {points.push(json!({"microstep":microstep,"point_on_block_lever_m":r.lever_b,"normal_on_table":patch.normal_on_a,"normal_n_s":r.normal_impulse,"radius_m":radius}));}
                }
                n+=count;
            }
            count=n;every_four&=n==4;
            *point_hist.entry(n).or_insert(0)+=1;
            microstep+=1;
        })?;
        let frame = p.frame();
        let (owner, block) = frame.body(2).unwrap();
        let physical: Vec<_> = seeds
            .iter()
            .map(|b| frame.body(b.id).unwrap().1.clone())
            .collect();
        let pair = &frame.workers[owner as usize].snapshot.impulses;
        if physical != control.bodies
            || *pair != contacts
            || !floor.is_empty()
            || frame
                .workers
                .iter()
                .any(|w| !w.snapshot.supports.is_empty())
        {
            return Err(format!("independent native control differs at {tick}").into());
        }
        let normal = pair.iter().map(|r| r.normal_impulse).sum::<f64>();
        max_point_normal_error = max_point_normal_error.max((point_normal - normal).abs());
        if (point_normal - normal).abs() > 1e-9 {
            return Err("point loads disagree with public barrier".into());
        }
        let reaction = (spin(block) - h0).z;
        if tick >= 31 {
            total_reaction += reaction;
            max_com_error =
                max_com_error.max((block.linear[0] - seeds[0].linear[0]).hypot(block.linear[1]));
            if omega_before.abs() > 0.2 && block.angular[2].abs() > 0.2 && every_four && cap > 1e-9
            {
                let err = (reaction + sign * cap).abs() / cap;
                saturation.push(err);
                peak_patch_error = peak_patch_error.max(err);
            }
            if block.angular[2].abs() < 0.01 {
                stops += 1;
                if stops == 5 && first_stop.is_none() {
                    first_stop = Some((tick - 4 - 30) as f64 * 0.01);
                }
            } else {
                stops = 0;
            }
        }
        last_speed = block.angular[2];
        digest.update(bincode::serialize(&(tick, &physical, pair))?);
        let compare_image = |pool: &mut WorkerPool, label: &str| -> Result<()> {
            let path = scratch.path().join(format!("{tick}-{label}"));
            pool.save_native_recovery(&path)?;
            let image = PoolNativeRecovery::open(path)?;
            let rows = image.finite_contact_patches()?;
            let expected = usize::from(pool.frame().body(2).unwrap().0);
            let expected_bytes = bincode::serialize(&reference_patch)?;
            if reference_patch.is_empty()
                || bincode::serialize(&rows[expected])? != expected_bytes
                || rows
                    .iter()
                    .enumerate()
                    .any(|(w, r)| w != expected && !r.is_empty())
            {
                return Err(format!(
                    "saved manifold geometry, normal, lever or impulse differs at {tick} {label}"
                )
                .into());
            }
            Ok(())
        };
        if inspect {
            compare_image(&mut p, "before")?;
            inspected_images += 1;
        }
        let old = p.frame().clone();
        if [36, 95].contains(&tick) {
            if normal <= 0. {
                return Err("unloaded transfer".into());
            }
            let dest = (owner + 1) % 4;
            p.transfer_component(2, dest)?;
            for b in &physical {
                let (w, new) = p.frame().body(b.id).unwrap();
                if w != dest
                    || new != b
                    || p.frame().workers[w as usize].generations[&b.id]
                        != old.workers[owner as usize].generations[&b.id] + 1
                {
                    return Err("invalid transferred membership".into());
                }
            }
            if p.frame().workers[dest as usize].snapshot.impulses != contacts {
                return Err("changed pair impulse on transfer".into());
            }
            events.push(json!({"kind":"warm transfer","source":owner,"dest":dest,"members":[1,2]}));
        } else if [37, 96].contains(&tick) {
            let pid = p.worker_pid(owner);
            p.terminate_worker(owner)?;
            p.recover(owner)?;
            if p.frame() != &old || p.worker_pid(owner) == pid {
                return Err("invalid process recovery".into());
            }
            events.push(json!({"kind":"real owner replacement","owner":owner,"old_pid":pid,"new_pid":p.worker_pid(owner)}));
        } else if [38, 97].contains(&tick) {
            let path = a.out.join(format!("cut-{tick}.gzpool"));
            p.save_native_recovery(&path)?;
            drop(p);
            p = PoolNativeRecovery::open(&path)?.resume(worker)?;
            configure(&mut p);
            if p.frame() != &old {
                return Err("full disk resume changed native frame".into());
            }
            events.push(json!({"kind":"full disk restore"}));
        }
        if inspect {
            compare_image(&mut p, "after")?;
            inspected_images += 1;
        }
        all_events.extend(events.iter().map(|e| json!({"tick":tick,"event":e})));
        if let Some(log) = &mut log {
            let (now, _) = p.frame().body(2).unwrap();
            writeln!(
                log,
                "{}",
                json!({"tick":tick,"bodies":physical,"reference_bodies":control.bodies,"owner":now,"generations":p.frame().workers[now as usize].generations,"pairs":contacts,"substeps":rate,
                 "omega_before":omega_before,"omega_after":last_speed,"angular_reaction_z_nms":reaction,"predicted_twist_limit_nms":cap,
                 "last_native_point_count":count,"native_points":points,"events":events})
            )?;
        }
    }
    saturation.sort_by(f64::total_cmp);
    let median = (!saturation.is_empty())
        .then(|| (saturation[saturation.len() / 2] + saturation[(saturation.len() - 1) / 2]) * 0.5);
    let stop_ok = match (ideal_stop, first_stop) {
        (Some(ideal), Some(t)) => (ideal - t).abs() <= 0.02,
        (None, None) => true,
        _ => false,
    };
    let speed_ok = if a.frictionless {
        (initial_speed - last_speed).abs() < 0.01
    } else {
        last_speed.abs() < 0.01
    };
    let saturation_ok = if a.frictionless {
        saturation.is_empty()
    } else {
        median.is_some_and(|e| e < 0.01)
    };
    let summary = json!({"ticks":120,"saved_native_patch_comparisons":inspected_images,"fine":!a.coarse,"rates":rates,"box":seeds[1],"table":seeds[0],"analytic_torque_nm":ideal_torque,"angular_impulse_nms":sign*KICK,"effective_corner_radius_m":radius,
        "ideal_stopping_s":ideal_stop,"measured_stopping_s":first_stop,"initial_omega":initial_speed,"terminal_omega":last_speed,"total_reaction_nms":total_reaction,
        "native_points_per_substep":point_hist,"saturated_intervals":saturation.len(),"median_relative_saturation_error":median,"max_relative_saturation_error":peak_patch_error,"max_horizontal_com_speed_difference":max_com_error,
        "max_point_normal_disagreement_ns":max_point_normal_error,"passed":{"stopping":stop_ok,"terminal_speed":speed_ok,"twist_limit":saturation_ok},"events":all_events,"trajectory_sha256":format!("{:x}",digest.finalize())});
    fs::write(
        a.out.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    if !stop_ok || !speed_ok || !saturation_ok {
        return Err("physical threshold exceeded; see retained summary".into());
    }
    Ok(())
}
