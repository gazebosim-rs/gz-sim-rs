// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Actual authored finite-pad scene with no initial dynamic velocity.
use super::*;
use gz_sim_scene::primary_contact::{last_box_timing, last_refinement, set_refined_observer};
use std::sync::{Arc, Mutex};
const BASE: &str = include_str!("../../../../examples/multifidelity_local_prospective_timing.sdf");
const ROLL: f64 = 0.00019;
const PITCH: f64 = 0.008;
const YAW: f64 = 0.17;
const PAD_V: DVec3 = DVec3::new(-0.13, 0.09, 1.4);
fn sequence() -> Result<Sequence> {
    let arrival_ns = 2_292_725;
    let orientation = DQuat::from_euler(glam::EulerRot::ZYX, YAW, PITCH, ROLL);
    let low = corners()
        .map(|p| (orientation * (p - ARM)).z)
        .into_iter()
        .fold(f64::INFINITY, f64::min);
    let case = Case {
        id: N_CASES,
        kind: Feature::NearCorner,
        arrival_ns,
        surface: DQuat::IDENTITY,
        orientation,
        incoming_spin: DVec3::ZERO,
        pad_v: PAD_V,
        incoming_v: DVec3::ZERO,
        at_com: PAD_V * (arrival_ns as f64 * 1e-9) - DVec3::Z * low,
    };
    let s = Sequence::solve(Reference::solve(case)?)?;
    if s.reason != "certified" || s.impacts.len() < 2 {
        return Err(format!(
            "authored strike sequence: {}, {} events",
            s.reason,
            s.impacts.len()
        )
        .into());
    }
    Ok(s)
}
fn authored(prospective: bool) -> String {
    if prospective {
        BASE.to_owned()
    } else {
        BASE.replace("<prospective_timing>true</prospective_timing>", "")
    }
}
pub(super) fn unrefined(native: &MultiFidelityBackend, ids: Bodies) -> ([f64; 4], DVec3) {
    let q = DQuat::from_array(native.state(ids.foot).unwrap().orientation);
    let mut corners = [0.; 4];
    let mut vector = DVec3::ZERO;
    for p in native.rigid_contact_patches() {
        let direct = p.a == ids.foot && p.b == ids.pad;
        if !direct && (p.b, p.a) != (ids.foot, ids.pad) {
            continue;
        }
        for v in p.points {
            let r = q.conjugate() * DVec3::from(if direct { v.lever_a } else { v.lever_b });
            corners[2 * usize::from(r.x > 0.) + usize::from(r.y > 0.)] += v.normal_impulse;
            vector +=
                DVec3::from(p.normal_on_a) * (if direct { 1. } else { -1. }) * v.normal_impulse;
        }
    }
    (corners, vector)
}
fn generate(seq: &Sequence, xml: &str) -> Result<(Value, Vec<Snapshot>, State)> {
    let (mut sim, ids) = sdf_features::load_input(xml)?;
    let begin = state(backend(&sim), ids)?;
    for (native, expected) in [
        (begin.foot, seq.state(0.).foot),
        (begin.upper, seq.state(0.).upper),
        (begin.pad, seq.state(0.).pad),
    ] {
        if DVec3::from(native.position).distance(expected.position.into()) > 1e-12
            || DQuat::from_array(native.orientation)
                .dot(DQuat::from_array(expected.orientation))
                .abs()
                < 1. - 1e-13
            || native.velocity != expected.velocity
            || native.angular_velocity != expected.angular_velocity
        {
            return Err("authored initial state differs from independent reference".into());
        }
    }
    let data = Arc::new(Mutex::new(Observed::default()));
    let shared = data.clone();
    set_refined_observer(&mut sim, move |_, step| {
        shared.lock().unwrap().record(step, ids)
    })?;
    let mut frames = vec![frame(0, begin, 0, false, [0.; 4])];
    let mut ticks = vec![];
    let mut j = DVec3::ZERO;
    let mut calls = 0;
    let mut first = None;
    for tick in 1..=8 {
        sim.step_exact(1)?;
        let t = sim.elapsed();
        let req = last_refinement(&sim).filter(|r| r.sim_time == t);
        let live = last_box_timing(&sim).filter(|r| r.sim_time == t);
        let r = live
            .map(|r| &r.interval)
            .or_else(|| req.map(|r| &r.interval));
        let measured = std::mem::take(&mut *data.lock().unwrap());
        let (corners, vector) = if let Some(r) = r {
            if measured.elapsed.len() != r.native_steps as usize
                || measured.elapsed.last() != Some(&500000)
            {
                return Err("authored selected observer lost a substep".into());
            }
            let pair = r
                .pairs
                .iter()
                .find(|c| (c.a, c.b) == (ids.foot, ids.pad) || (c.b, c.a) == (ids.foot, ids.pad));
            if let Some(p) = pair
                && (p.normal_impulse - measured.corners.iter().sum::<f64>()).abs() > 1e-10
            {
                return Err("authored integrated load mismatch".into());
            }
            (
                measured.corners,
                pair.map_or(DVec3::ZERO, |p| {
                    DVec3::from(p.normal_impulse_on_a) * if p.a == ids.foot { 1. } else { -1. }
                }),
            )
        } else {
            if !measured.elapsed.is_empty() {
                return Err("ordinary step unexpectedly invoked observer".into());
            }
            unrefined(backend(&sim), ids)
        };
        if corners.iter().sum::<f64>() > 0. {
            first.get_or_insert((tick - 1) * 500000 + measured.first_load.unwrap_or(500000));
        }
        let count = r.map_or(1, |r| r.native_steps);
        calls += count;
        j += vector;
        frames.push(frame(
            tick,
            state(backend(&sim), ids)?,
            count,
            live.is_some(),
            corners,
        ));
        ticks.push(json!({"tick":tick,"native_steps":count,"outer_primary":req.is_some(),"retimed":live.map_or(0,|v|v.timing.retimed),"decline":live.and_then(|v|v.timing.decline).map(|v|format!("{v:?}")),"end_ns":measured.elapsed,"corner_ns":corners,"impulse_ns":vector.to_array()}));
    }
    let end = frames.last().unwrap().state;
    Ok((
        json!({"ticks":ticks,"calls":calls,"metrics":score(seq,end,j),"first_positive_end_ns":first}),
        frames,
        end,
    ))
}
pub(super) fn write(path: &Path) -> Result<()> {
    let dir = path.join("authored");
    fs::create_dir_all(dir.join("traces"))?;
    let seq = sequence()?;
    let mut rows = vec![];
    for (enabled, name) in [(false, "uniform"), (true, "prospective")] {
        let xml = authored(enabled);
        let (r, frames, _) = generate(&seq, &xml)?;
        fs::write(dir.join(format!("{name}.sdf")), xml)?;
        trace(&dir, name, &frames)?;
        rows.push(json!({"profile":name,"run":r}));
    }
    trace(&dir, "analytic", &native::trace_reference(&seq))?;
    write_json(
        &dir.join("authored.json"),
        &json!({"cases":rows,"reference":seq.impacts}),
    )?;
    write_json(
        &dir.join("comparison.json"),
        &json!({"title":"Successive corner impacts from an authored moving-pad world","lanes":[
        {"label":"uniform","name":"Authored uniform primary","file":"traces/uniform.json"},
        {"label":"prospective","name":"Authored live contact timing","file":"traces/prospective.json"},
        {"label":"analytic","name":"Certified event sequence","file":"traces/analytic.json","analytical":true}]}),
    )
}
pub(super) fn cost(path: &Path) -> Result<()> {
    let seq = sequence()?;
    let inputs = [authored(false), authored(true)];
    let mut expected = vec![];
    for xml in &inputs {
        expected.push(generate(&seq, xml)?.2);
    }
    let mut rows: [Vec<[u64; 8]>; 2] = Default::default();
    for rep in 0..257 {
        for turn in 0..2 {
            let i = (rep + turn) % 2;
            let (mut sim, ids) = sdf_features::load_input(&inputs[i])?;
            let mut timing = [0; 8];
            for n in &mut timing {
                let start = Instant::now();
                sim.step_exact(1)?;
                *n = start.elapsed().as_nanos() as u64;
            }
            if state(backend(&sim), ids)? != expected[i] {
                return Err("timed authored trajectory changed".into());
            }
            if rep > 0 {
                rows[i].push(timing);
            }
        }
    }
    fs::create_dir_all(path.join("authored"))?;
    write_json(
        &path.join("authored/cost.json"),
        &json!({"uniform":rows[0],"prospective":rows[1],"primary_tick":seq.r.case.arrival_ns.div_ceil(500000)}),
    )
}
