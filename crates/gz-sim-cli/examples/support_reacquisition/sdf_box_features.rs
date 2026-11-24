// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! End-to-end authored SDF. The body starts at rest and the support supplies the incoming relative speed.
use super::*;
use gz_sim_scene::{
    load_world_with_sdf_step,
    primary_contact::{last_box_feature_refresh, last_refinement},
};
use std::time::Instant;
const INPUT: &str = include_str!("../../../../examples/multifidelity_local_box_features.sdf");
const BOOST: DVec3 = DVec3::new(-0.13, 0.09, 1.10);
fn exact(ideal: Ideal, t: u64) -> State {
    let mut s = ideal.state(t);
    let dt = t as f64 * 1e-9;
    for b in [&mut s.foot, &mut s.upper, &mut s.pad] {
        b.position = (DVec3::from(b.position) + dt * BOOST).into();
        b.velocity = (DVec3::from(b.velocity) + BOOST).into();
    }
    s
}
fn load(enabled: bool) -> Result<(Simulation, Bodies)> {
    let sdf = if enabled {
        INPUT.to_owned()
    } else {
        INPUT.replace("<refresh_box_features>true</refresh_box_features>", "")
    };
    load_input(&sdf)
}
pub(super) fn load_input(sdf: &str) -> Result<(Simulation, Bodies)> {
    load_finite_box(sdf, true)
}
pub(super) fn load_finite_box(sdf: &str, has_upper: bool) -> Result<(Simulation, Bodies)> {
    let sim = load_world_with_sdf_step(
        sdf,
        "local_box_features.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let ids = finite_ids(&sim, has_upper)?;
    Ok((sim, ids))
}
pub(super) fn finite_ids(sim: &Simulation, has_upper: bool) -> Result<Bodies> {
    let body = |label: &str| -> Result<BodyId> {
        #[allow(deprecated)]
        sim.world()
            .iter_entities()
            .find_map(|e| {
                (e.get::<Name>()?.0 == label)
                    .then(|| e.get::<BodyHandle>().map(|h| h.id()))
                    .flatten()
            })
            .ok_or_else(|| format!("missing {label}").into())
    };
    let ids = Bodies {
        foot: body("default::rocker::foot")?,
        upper: has_upper
            .then(|| body("default::rocker::upper"))
            .transpose()?,
        pad: body("default::lift::pad")?,
    };
    Ok(ids)
}
fn solver(sim: &Simulation) -> &MultiFidelityBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .expect("SDF selected local multifidelity")
}
fn classify(point: &gz_physics_multifidelity::RigidPatchPoint, foot_is_a: bool, q: DQuat) -> usize {
    let lever = DVec3::from(if foot_is_a {
        point.lever_a
    } else {
        point.lever_b
    });
    let r = q.conjugate() * lever;
    usize::from(r.x > 0.) * 2 + usize::from(r.y > 0.)
}
fn generate(enabled: bool) -> Result<(Vec<Snapshot>, Value)> {
    let ideal = Ideal::new(FIXTURES[3]);
    let (mut sim, ids) = load(enabled)?;
    let initial = state(solver(&sim), ids)?;
    let expected = exact(ideal, 0);
    for (a, b) in [
        (initial.foot, expected.foot),
        (initial.upper, expected.upper),
        (initial.pad, expected.pad),
    ] {
        if DVec3::from(a.position).distance(b.position.into()) > 1e-10
            || DVec3::from(a.velocity).distance(b.velocity.into()) > 1e-10
            || DQuat::from_array(a.orientation)
                .dot(DQuat::from_array(b.orientation))
                .abs()
                < 1. - 1e-12
        {
            return Err("SDF initial state differs from analytical reference".into());
        }
    }
    let mut frames = vec![];
    let mut receipts = vec![];
    let mut j = 0.;
    let mut primary = 0.;
    let mut minimum = f64::INFINITY;
    let mut scopes = 0;
    let mut rebuilt = 0;
    for tick in 0..=8 {
        if tick > 0 {
            sim.step_exact(1)?;
        }
        let t = tick * 500_000;
        let s = state(solver(&sim), ids)?;
        let q = DQuat::from_array(s.foot.orientation);
        let mut observed = [0.; 4];
        for patch in solver(&sim).rigid_contact_patches() {
            if (patch.a, patch.b) != (ids.foot, ids.pad)
                && (patch.b, patch.a) != (ids.foot, ids.pad)
            {
                continue;
            }
            for point in patch.points {
                let k = classify(&point, patch.a == ids.foot, q);
                observed[k] += point.normal_impulse;
            }
        }
        let (g, _, _) = geometry(s, ideal.corner);
        minimum = minimum.min(g);
        let mut normal = observed.iter().sum::<f64>();
        let request = last_refinement(&sim).filter(|v| v.sim_time == sim.elapsed());
        if let Some(scope) = request {
            if scope.requested_pairs.len() != 1 {
                return Err("unexpected count of selected primary pairs".into());
            }
            primary = scope.requested_pairs[0].normal_impulse;
            normal = primary;
        }
        j += normal;
        let work = last_box_feature_refresh(&sim).filter(|v| v.sim_time == sim.elapsed());
        if let Some(w) = work {
            scopes += 1;
            rebuilt += w.rebuilt_manifolds;
        }
        let native_steps = if tick == 0 {
            0
        } else {
            request.map_or(1, |x| x.interval.native_steps)
        };
        receipts.push(json!({"tick":tick,"time_ns":t,"native_steps":native_steps,"last_native_observed_corner_ns":observed,"total_normal_ns":normal,
            "feature_requests":work.map_or(0,|w|w.pairs.len()),"rebuilt_manifolds":work.map_or(0,|w|w.rebuilt_manifolds)}));
        frames.push(Snapshot {
            tick,
            time_us: t / 1000,
            state: s,
            native_steps,
            selected: work.is_some(),
            secondary_start: false,
            normal_impulse_ns: normal,
            corner_ns: observed,
        });
    }
    if minimum <= 0.03 || primary <= 0. || enabled && (scopes == 0 || rebuilt == 0) {
        return Err(format!("invalid SDF trial: primary {primary}, scopes {scopes}, rebuilt {rebuilt}, clearance {minimum}").into());
    }
    let end = frames.last().unwrap().state;
    Ok((
        frames,
        json!({"enabled":enabled,"total_normal_ns":j,"total_error_ns":j-ideal.impulse,"primary_normal_ns":primary,"normal_after_primary_ns":j-primary,"min_sampled_other_clearance_m":minimum,"feature_ticks":scopes,"rebuilt_manifolds":rebuilt,
      "spin_error_rad_s":DVec3::from(end.foot.angular_velocity).distance(ideal.spin),"com_error_m":end.com().distance(exact(ideal,4_000_000).com()),"ticks":receipts}),
    ))
}
fn measure(enabled: bool, expected: State) -> Result<[u64; 8]> {
    let (mut sim, ids) = load(enabled)?;
    let mut times = [0; 8];
    for t in &mut times {
        let start = Instant::now();
        sim.step_exact(1)?;
        *t = start.elapsed().as_nanos() as u64;
    }
    if state(solver(&sim), ids)? != expected {
        return Err("SDF timing endpoint differs".into());
    }
    Ok(times)
}
// The public-tick receipt measures a pair-wide total. Show the same observable
// in all three lanes without attributing the integrated load to points.
fn pair_trace(path: &Path, name: &str, frames: &[Snapshot]) -> Result<()> {
    let frames=frames.iter().map(|f|json!({"tick":f.tick,"time_us":f.time_us,"foot":f.state.foot,"upper":f.state.upper,
        "pad":f.state.pad,"native_steps":f.native_steps,"selected":f.selected,"secondary_start":false,"normal_impulse_ns":f.normal_impulse_ns})).collect::<Vec<_>>();
    write_json(
        &path.join("traces").join(format!("{name}.json")),
        &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":UPPER_HALF,"pad_half_extents":PAD_HALF},"frames":frames}),
    )
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut cases = vec![];
    let mut states = vec![];
    for enabled in [false, true] {
        let (frames, data) = generate(enabled)?;
        states.push(frames.last().unwrap().state);
        pair_trace(path, if enabled { "fresh" } else { "default" }, &frames)?;
        cases.push(data);
    }
    let ideal = Ideal::new(FIXTURES[3]);
    let mut reference = vec![];
    for tick in 0..=8 {
        let t = tick * 500_000;
        let hit = tick == 5;
        let mut corner = [0.; 4];
        if hit {
            corner[ideal.corner.unwrap()] = ideal.impulse;
        }
        reference.push(Snapshot {
            tick,
            time_us: t / 1000,
            state: exact(ideal, t),
            native_steps: 0,
            selected: false,
            secondary_start: false,
            normal_impulse_ns: corner.iter().sum(),
            corner_ns: corner,
        });
    }
    pair_trace(path, "analytic", &reference)?;
    write_json(
        &path.join("comparison.json"),
        &json!({"title":"Authored SDF box feature refresh","pair_totals_only":true,"lanes":[{"label":"default","name":"SDF primary refinement","file":"traces/default.json"},
        {"label":"fresh","name":"SDF primary plus selected box features","file":"traces/fresh.json"},{"label":"analytic","name":"Analytic corner impact","file":"traces/analytic.json","analytical":true}]}),
    )?;
    write_json(
        &path.join("sdf.json"),
        &json!({"ideal_normal_ns":ideal.impulse,"cases":cases}),
    )?;
    let mut times: [Vec<[u64; 8]>; 2] = Default::default();
    for e in [false, true] {
        measure(e, states[usize::from(e)])?;
    }
    for i in 0usize..80 {
        for k in 0..2 {
            let idx = if i.is_multiple_of(2) { k } else { 1 - k };
            times[idx].push(measure(idx != 0, states[idx])?);
        }
    }
    let timing=times.into_iter().enumerate().map(|(index,values)|{
        let mut primary:Vec<_>=values.iter().map(|v|v[4]).collect();
        let mut follow:Vec<_>=values.iter().flat_map(|v|v[5..].iter().copied()).collect();
        json!({"enabled":index!=0,"primary_p95_ns":cost::rank(&mut primary,95),"following_ticks_p95_ns":cost::rank(&mut follow,95),"tick_ns":values})
    }).collect::<Vec<_>>();
    write_json(
        &path.join("cost.json"),
        &json!({"repeats":80,"timing":timing}),
    )
}
