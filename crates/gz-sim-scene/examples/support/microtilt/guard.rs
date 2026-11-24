// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay exactly the planner's prefix through the independently certified single-corner cutoff.
use super::*;
use gz_physics_multifidelity::{BoxContactActions, ContactSubstep};

struct End {
    state: PoseState,
    reaction: DVec3,
    moments: DVec3,
    corners: [f64; 4],
    duration: Duration,
}
fn end(s: ContactSubstep<'_>, ids: &Reading, prior: PoseState, rigid: bool) -> Result<End> {
    let state = Seed::read((
        s.state(ids.foot).unwrap(),
        ids.upper.and_then(|u| s.state(u)),
        s.state(ids.pad).unwrap(),
    ))
    .state;
    let (reaction, moments, corners) = moments(&s.contact_patches(), ids, prior.q, rigid)?;
    Ok(End {
        state,
        reaction,
        moments,
        corners,
        duration: s.duration,
    })
}
fn trial(seed: Seed, c: Case, rigid: bool, guard: bool, oracle: &oracle::Oracle) -> Result<Value> {
    let full = Duration::from_micros(500);
    let cut = Duration::from_nanos(oracle.stop_ns);
    let budget = ContactSolverIterations {
        temporal: 4,
        pgs: Some(1),
        relaxation: Some(8),
    };
    let source = seed.scene(c.mu, rigid, "ordinary");
    let mut s = load(&source, true)?;
    let id = ids(&s)?;
    let pair = [(id.foot, id.pad)];
    let active = if guard { &pair[..] } else { &[] };
    let mut p = s.world_mut().resource_mut::<PhysicsBackend>();
    let n = p.downcast_mut::<MultiFidelityBackend>().unwrap();
    let mut previous = Seed::read(departure::ingest(n, &id)).state;
    let mut full_ends = vec![];
    let mut error = None;
    let outcome = n.advance_box_contacts(
        full,
        8,
        budget,
        BoxContactActions {
            fresh_features: &pair,
            prospective_timing: &pair,
            guard_separated_flight: active,
            ..Default::default()
        },
        Some(&mut |s| match end(s, &id, previous, rigid) {
            Ok(v) => {
                previous = v.state;
                full_ends.push(v)
            }
            Err(e) => error = Some(e),
        }),
    )?;
    if let Some(e) = error {
        return Err(e);
    }
    let timing = outcome.timing.ok_or("no full timing receipt")?;
    let full_guards = outcome
        .flight
        .map_or_else(|| vec![false; full_ends.len()], |f| f.guarded);
    if full_ends
        .iter()
        .map(|e| e.duration)
        .ne(timing.durations.iter().copied())
        || full_guards.len() != 8
    {
        return Err("planner and observer duration mismatch".into());
    }
    let mut shadow = load(&source, true)?;
    let id = ids(&shadow)?;
    let pair = [(id.foot, id.pad)];
    let active = if guard { &pair[..] } else { &[] };
    let mut p = shadow.world_mut().resource_mut::<PhysicsBackend>();
    let n = p.downcast_mut::<MultiFidelityBackend>().unwrap();
    let initial = Seed::read(departure::ingest(n, &id)).state;
    let mut before = initial;
    let mut elapsed = Duration::ZERO;
    let mut observed = vec![];
    let mut j = DVec3::ZERO;
    let mut h = DVec3::ZERO;
    let mut quadrants = [0.; 4];
    let mut prefix_delta = 0f64;
    let mut guard_differences = 0;
    let mut first = None;
    for (i, real) in full_ends.iter().enumerate() {
        let dt = real.duration.min(cut - elapsed);
        let mut saved = None;
        let o = n.advance_box_contacts(
            dt,
            1,
            budget,
            BoxContactActions {
                fresh_features: &pair,
                guard_separated_flight: active,
                ..Default::default()
            },
            Some(&mut |s| saved = Some(end(s, &id, before, rigid))),
        )?;
        let sub = saved.ok_or("missing guarded prefix call")??;
        let flag = o.flight.as_ref().is_some_and(|f| f.guarded == [true]);
        elapsed += dt;
        if flag && sub.reaction != DVec3::ZERO {
            return Err("guarded native pair delivered reaction".into());
        }
        if dt == real.duration {
            guard_differences += usize::from(flag != full_guards[i]);
            prefix_delta = prefix_delta
                .max(near(sub.state, real.state))
                .max(sub.reaction.distance(real.reaction));
        }
        observed.push(json!({"end_ns":elapsed.as_nanos() as u64,"duration_ns":dt.as_nanos() as u64,"guarded":flag,"full_guarded":full_guards[i],"full_duration_ns":real.duration.as_nanos() as u64,"normal_ns":sub.reaction.z}));
        before = sub.state;
        j += sub.reaction;
        h += sub.moments;
        for (q, load) in quadrants.iter_mut().zip(sub.corners) {
            *q += load;
        }
        if sub.reaction.z > 0. {
            first.get_or_insert(elapsed.as_nanos() as u64);
        }
        if elapsed == cut {
            break;
        }
    }
    if guard_differences != 0 || prefix_delta > 1e-10 {
        return Err(format!("fixed prefix disagrees with full native run in {} rigid={rigid} guarded={guard}: {guard_differences} decisions and {prefix_delta:e} state/impulse norm",c.label).into());
    }
    let all=full_ends.iter().zip(full_guards).map(|(e,g)|json!({"duration_ns":e.duration.as_nanos() as u64,"guarded":g,"impulse_ns":e.reaction.to_array(),"quadrants_ns":e.corners})).collect::<Vec<_>>();
    Ok(
        json!({"single_rigid":rigid,"guarded_policy":guard,"start_error":near(initial,seed.state),"retimed":timing.retimed,"full":all,"prefix":observed,
       "prefix_state_and_impulse_difference":prefix_delta,"prefix_guard_differences":guard_differences,"first_positive_end_ns":first,
       "impulse_ns":j.to_array(),"quadrants_ns":quadrants,"moment_nms":h.to_array(),"delta_h_nms":((before.w-initial.w)*0.05).to_array(),"state":before.wire(),
       "impulse_error_ns":j.distance(oracle.impulse),"wrong_corner_ns":quadrants.iter().enumerate().filter(|(i,_)|*i!=oracle.corner).map(|(_,v)|v).sum::<f64>()}),
    )
}
pub(in super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    let mut public = vec![];
    for (material, mu) in [(None, 0.5), (Some(0.2), 0.2), (Some(0.8), 0.8)] {
        let (early, incoming) = donor(material)?;
        for (label, xs, ys, gap) in [
            ("measured", 1., 1., 0.),
            ("reflect-x", -1., 1., 0.),
            ("reflect-y", 1., -1., 0.),
            ("reflect-xy", -1., -1., 0.),
            ("earlier", -1., 1., -0.00011),
            ("later", 1., -1., 0.00005),
        ] {
            let c = Case {
                label,
                mu,
                material,
                xs,
                ys,
                gap,
            };
            let seed = incoming.transform(xs, ys, gap);
            let o = oracle::solve(seed.state, mu)?;
            let mut cases = vec![];
            for rigid in [false, true] {
                for guard in [false, true] {
                    cases.push(trial(seed, c, rigid, guard, &o)?);
                }
                for policy in ["relax8", "prospective", "guarded"] {
                    let write = material.is_none() && label == "measured" && rigid;
                    public.push(json!({"case":label,"mu":mu,"result":public_case(out,early.transform(xs,ys,gap),c,rigid,policy,write)?}));
                }
            }
            rows.push(json!({"case":label,"mu":mu,"physical":o.wire,"samples":cases}));
        }
    }
    fs::write(out.join("first.json"), serde_json::to_vec(&rows)?)?;
    fs::write(out.join("public.json"), serde_json::to_vec(&public)?)?;
    let lanes = [
        ("relax8", "Uniform eight"),
        ("prospective", "Timed eight"),
        ("guarded", "Timed eight with flight guard"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"Rigid microtilt · guarding certified separated flight","single_body":true,"pair_totals_only":true,"sampled_ticks":true,"capture_ticks":{"first_load":1531,"brake_start":1613,"demotion_gate":2120},"lanes":lanes}),
        )?,
    )?;
    Ok(())
}
