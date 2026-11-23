// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Branch a loaded parallel fold at one actuator tick, then vary only the material solver.
use super::*;
use crate::{
    arm::Controller,
    arm_sheet::{self, COUNT},
    fold_mesh,
};
use gz_sim_scene::{
    JointSample,
    deformable::{DeformableSample, set_material_resolution},
};
use serde::Serialize;

const MATCH: usize = 1650;
const HOLD: usize = 300;
const DT: f64 = 0.001;
const STRIDE: usize = 25;

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
struct Resolution {
    substeps: u32,
    iterations: u32,
}

const fn grid(substeps: u32, iterations: u32) -> Resolution {
    Resolution {
        substeps,
        iterations,
    }
}
const LOADED: Resolution = grid(8, 24);
const REFERENCE: Resolution = grid(64, 48);
const GRIDS: [Resolution; 7] = [
    LOADED,
    grid(16, 24),
    grid(32, 24),
    grid(32, 12),
    grid(32, 48),
    grid(64, 24),
    REFERENCE,
];

/// Direct equality covers the native links, joint samples, material state, and last reaction
/// passed through the coupling. Each branch replays the same commands from construction so
/// the native solver's private history and the pending pad wrench remain in the live world.
#[allow(deprecated)]
fn snapshot(sim: &Simulation) -> Result<Value> {
    let mut rigid = std::collections::BTreeMap::new();
    let mut joints = std::collections::BTreeMap::new();
    for e in sim.world().iter_entities() {
        if let (Some(handle), Some(id)) = (e.get::<BodyHandle>(), e.get::<EntityId>()) {
            let b = sim
                .world()
                .resource::<PhysicsBackend>()
                .state(handle.id())
                .ok_or("missing loaded native state")?;
            rigid.insert(
                id.get(),
                json!([b.position, b.orientation, b.velocity, b.angular_velocity]),
            );
        }
        if let (Some(joint), Some(id)) = (e.get::<JointSample>(), e.get::<EntityId>()) {
            joints.insert(
                id.get(),
                json!([joint.state.position, joint.state.velocity]),
            );
        }
    }
    let b = arm_sheet::cloth(sim)?;
    let internal = b.last_self_contact_impulse();
    Ok(
        json!({"time_ns":b.sim_time().as_nanos(),"epoch":b.epoch(),"rigid":rigid,"joints":joints,
        "particles":b.particles().iter().map(|p|json!([p.position,p.velocity,p.inverse_mass])).collect::<Vec<_>>(),
        "boundary":b.last_boundary_impulses().iter().map(|l|json!([l.boundary,l.linear,l.angular])).collect::<Vec<_>>(),
        "pad":b.last_pad_loads().iter().map(|l|json!([l.boundary,l.particle,l.normal,l.tangent])).collect::<Vec<_>>(),
        "self_contact":[internal.normal,internal.tangent,internal.friction_work],
        "self_loads":b.last_self_contact_loads().iter().map(|l|json!([l.particles,l.first_side_len,l.normal_ns,l.particle_normal_ns,l.particle_impulse_ns])).collect::<Vec<_>>() }),
    )
}

#[derive(Debug, Serialize)]
struct Sample {
    time_ns: u128,
    self_contact_n: f64,
    cross_panel_n: f64,
    same_panel_n: f64,
    mixed_n: f64,
    jaw_up_n: [f64; 2],
    jaw_normal_n: [f64; 2],
    bench_up_n: f64,
}
#[derive(Default)]
struct Distribution {
    cross: [f64; COUNT],
    pad: [f64; COUNT],
}
/// Zero barycentric weights do not carry an impulse at that feature.
pub(super) fn side_panel(ids: &[usize], weights: &[f64], columns: usize) -> Option<usize> {
    let mut panels = ids
        .iter()
        .zip(weights)
        .filter(|(_, w)| **w > 0.)
        .map(|(&i, _)| usize::from(i / columns >= 2));
    let first = panels.next()?;
    panels.all(|p| p == first).then_some(first)
}

fn measure(b: &DeformableSample, dist: &mut Distribution) -> Result<Sample> {
    let [left, right, bench] = b.last_boundary_impulses() else {
        return Err("fold requires two jaws and bench".into());
    };
    let mut s = Sample {
        time_ns: b.sim_time().as_nanos(),
        self_contact_n: b.last_self_contact_impulse().normal / DT,
        cross_panel_n: 0.,
        same_panel_n: 0.,
        mixed_n: 0.,
        jaw_up_n: [-left.linear[2] / DT, -right.linear[2] / DT],
        jaw_normal_n: [0.; 2],
        bench_up_n: -bench.linear[2] / DT,
    };
    for l in b.last_pad_loads() {
        let n = DVec3::from(l.normal).length() / DT;
        s.jaw_normal_n[l.boundary] += n;
        dist.pad[l.particle] += n / HOLD as f64;
    }
    for l in b.last_self_contact_loads() {
        let (a, b) = l.particles.split_at(l.first_side_len);
        let (wa, wb) = l.particle_normal_ns.split_at(l.first_side_len);
        let n = l.normal_ns / DT;
        match (side_panel(a, wa, 3), side_panel(b, wb, 3)) {
            (Some(left), Some(right)) if left != right => {
                s.cross_panel_n += n;
                for (&id, w) in l.particles.iter().zip(l.particle_normal_ns) {
                    dist.cross[id] += w / (DT * HOLD as f64);
                }
            }
            (Some(_), Some(_)) => s.same_panel_n += n,
            _ => s.mixed_n += n,
        }
    }
    if (s.cross_panel_n + s.same_panel_n + s.mixed_n - s.self_contact_n).abs()
        > 1e-8 * s.self_contact_n.max(1.)
    {
        return Err("sheet stencil impulses do not reproduce total self-contact".into());
    }
    Ok(s)
}

#[derive(Serialize)]
struct Trial {
    #[serde(flatten)]
    resolution: Resolution,
    mean_self_contact_n: f64,
    mean_cross_panel_n: f64,
    mean_mixed_n: f64,
    mean_same_panel_n: f64,
    mean_cross_panel_node_normal_n: [f64; COUNT],
    mean_pad_normal_by_node_n: [f64; COUNT],
    mean_jaw_up_n: [f64; 2],
    mean_jaw_normal_n: [f64; 2],
    mean_bench_up_n: f64,
    first_self_contact_n: f64,
    last_100ms_self_contact_n: f64,
    max_particle_displacement_from_anchor_m: f64,
    min_bench_clearance_m: f64,
    refined_ticks: usize,
    hold_step_p95_ns: u64,
    samples: Vec<Sample>,
}

fn drive(r: &mut Recording, c: &Controller, tick: usize) -> Result<()> {
    arm_cable::lift_with_jaws(c, &mut r.sim, (tick - 1) as f64 * DT, arm_sheet::JAW_STROKE)?;
    r.advance()
}
fn trial(
    xml: &str,
    resolution: Resolution,
    expected: &mut Option<Value>,
    out: &Path,
) -> Result<Trial> {
    let mut r = Recording::new(xml, "fold_hold.sdf", "default")?;
    let ctrl = Controller::new(&r.sim)?;
    let recording = if resolution == LOADED {
        Some("coarse")
    } else if resolution == REFERENCE {
        Some("fine")
    } else {
        None
    };
    let mut frames = vec![];
    if recording.is_some() {
        arm_sheet::capture(&mut r, &mut frames)?
    }
    for tick in 1..=MATCH {
        drive(&mut r, &ctrl, tick)?;
        if recording.is_some() && tick % STRIDE == 0 {
            arm_sheet::capture(&mut r, &mut frames)?;
        }
    }
    let loaded = snapshot(&r.sim)?;
    match expected {
        Some(e) if *e != loaded => return Err("loaded fold differs before branching".into()),
        None => *expected = Some(loaded.clone()),
        _ => {}
    }
    let b = arm_sheet::cloth(&r.sim)?;
    let matched: Vec<DVec3> = b
        .particles()
        .iter()
        .map(|p| DVec3::from(p.position))
        .collect();
    let weight = b
        .particles()
        .iter()
        .map(|p| 9.81 / p.inverse_mass)
        .sum::<f64>();
    let jaws = [
        entity_id(&r.sim, "default::arm::left")?,
        entity_id(&r.sim, "default::arm::right")?,
    ];
    let jaw_states = |sim: &Simulation| -> Result<[BodyState; 2]> {
        Ok([state(sim, jaws[0])?, state(sim, jaws[1])?])
    };
    let mesh = if resolution == REFERENCE {
        Some(fold_mesh::Anchor::capture(&r.sim)?)
    } else {
        None
    };
    let mut path = vec![jaw_states(&r.sim)?];
    if resolution != LOADED {
        set_material_resolution(
            &mut r.sim,
            "sheet",
            resolution.substeps,
            resolution.iterations,
        )?;
    }
    if snapshot(&r.sim)? != loaded {
        return Err("retuning changed the committed loaded state".into());
    }
    let mut dist = Distribution::default();
    let mut samples = Vec::with_capacity(HOLD);
    let mut refined = 0;
    let mut clearance = f64::INFINITY;
    let mut drift = 0f64;
    for tick in MATCH + 1..=MATCH + HOLD {
        drive(&mut r, &ctrl, tick)?;
        let b = arm_sheet::cloth(&r.sim)?;
        samples.push(measure(b, &mut dist)?);
        refined += usize::from(b.last_sweep_refinements() > 0);
        for (p, initial) in b.particles().iter().zip(&matched) {
            clearance = clearance.min(p.position[2] - arm_sheet::BENCH - arm_sheet::RADIUS);
            drift = drift.max(DVec3::from(p.position).distance(*initial));
        }
        if mesh.is_some() {
            path.push(jaw_states(&r.sim)?);
        }
        if recording.is_some() && tick % STRIDE == 0 {
            arm_sheet::capture(&mut r, &mut frames)?;
        }
    }
    let mean = |f: fn(&Sample) -> f64| samples.iter().map(f).sum::<f64>() / HOLD as f64;
    let mean_jaws = |f: fn(&Sample) -> [f64; 2]| {
        [0, 1].map(|i| samples.iter().map(|s| f(s)[i]).sum::<f64>() / HOLD as f64)
    };
    let up = mean_jaws(|s| s.jaw_up_n);
    let bench = mean(|s| s.bench_up_n);
    if clearance < 0.03
        || drift > 0.002
        || bench.abs() > 0.001
        || (up.iter().sum::<f64>() / weight - 1.).abs() > 0.1
    {
        return Err(format!("fold lost support or moved during hold: {resolution:?}, drift {drift}, clearance {clearance}, bench {bench}, jaws {up:?}").into());
    }
    let cost = &mut r.costs_ns[MATCH..];
    cost.sort_unstable();
    let p95 = cost[(HOLD * 95).div_ceil(100) - 1];
    if let Some(dir) = recording {
        arm_sheet::save_archive(
            r,
            frames,
            &out.join(dir),
            json!({"pickup":26,"rising":45,"matched":MATCH/STRIDE,"hold":72,"end":(MATCH+HOLD)/STRIDE}),
        )?;
    }
    if let Some(anchor) = mesh {
        fold_mesh::run(anchor, &path, &out.join("mesh"))?;
    }
    Ok(Trial {
        resolution,
        mean_self_contact_n: mean(|s| s.self_contact_n),
        mean_cross_panel_n: mean(|s| s.cross_panel_n),
        mean_mixed_n: mean(|s| s.mixed_n),
        mean_same_panel_n: mean(|s| s.same_panel_n),
        mean_cross_panel_node_normal_n: dist.cross,
        mean_pad_normal_by_node_n: dist.pad,
        mean_jaw_up_n: up,
        mean_jaw_normal_n: mean_jaws(|s| s.jaw_normal_n),
        mean_bench_up_n: bench,
        first_self_contact_n: samples[0].self_contact_n,
        last_100ms_self_contact_n: samples[HOLD - 100..]
            .iter()
            .map(|s| s.self_contact_n)
            .sum::<f64>()
            / 100.,
        max_particle_displacement_from_anchor_m: drift,
        min_bench_clearance_m: clearance,
        refined_ticks: refined,
        hold_step_p95_ns: p95,
        samples,
    })
}

fn comparison(t: &Trial, r: &Trial) -> Value {
    let cross_max_node = t
        .mean_cross_panel_node_normal_n
        .iter()
        .zip(r.mean_cross_panel_node_normal_n)
        .map(|(s, v)| (s - v).abs())
        .fold(0., f64::max);
    json!({"substeps":t.resolution.substeps,"iterations":t.resolution.iterations,
        "mean_self_contact_error_n":(t.mean_self_contact_n-r.mean_self_contact_n).abs(),
        "mean_cross_panel_error_n":(t.mean_cross_panel_n-r.mean_cross_panel_n).abs(),
        "max_mean_cross_panel_node_error_n":cross_max_node,
        "max_tick_self_contact_error_n":t.samples.iter().zip(&r.samples).map(|(a,b)|(a.self_contact_n-b.self_contact_n).abs()).fold(0.,f64::max)})
}
pub(super) fn run(out: &Path) -> Result<()> {
    let xml = arm_sheet::scene_with_loads(
        arm::SCENE,
        arm_sheet::Case {
            substeps: LOADED.substeps as usize,
            ..Default::default()
        },
        0.,
        true,
    )?;
    let mut anchor = None;
    let mut trials = vec![];
    for resolution in GRIDS {
        trials.push(trial(&xml, resolution, &mut anchor, out)?);
    }
    let reference = trials.last().unwrap();
    let comparisons = trials
        .iter()
        .map(|t| comparison(t, reference))
        .collect::<Vec<_>>();
    write_json(
        &out.join("comparison.json"),
        &json!({"matched_time_ns":MATCH*1_000_000,"dt_ns":1_000_000,"hold_ticks":HOLD,
        "approach":LOADED,"reference":REFERENCE,"matched_state":anchor,"trials":trials,"comparisons":comparisons}),
    )
}

#[cfg(test)]
#[path = "fold_hold_tests.rs"]
mod tests;
