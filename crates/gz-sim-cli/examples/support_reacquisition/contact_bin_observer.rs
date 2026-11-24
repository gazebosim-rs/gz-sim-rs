// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconcile native point reactions and explicitly integrated contact loads at
//! eight nested, actual observation times. Never interpolate contact events.
use super::*;
use gz_physics_multifidelity::{BoxContactActions, ContactIntervalReceipt, ContactSubstep};
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer::propagation) fn run(
    native: &mut MultiFidelityBackend,
    segment: gz_physics_multifidelity::ContactSegment,
    ids: [BodyId; 3],
    initial: &Value,
    actions: Option<BoxContactActions<'_>>,
    rows: &mut Vec<Value>,
) -> Result<(ContactIntervalReceipt, DVec3)> {
    let gz_physics_multifidelity::ContactSegment {
        duration: dt,
        native_steps: n,
        iterations: params,
    } = segment;
    if !n.is_multiple_of(8) {
        return Err("non-nested native observation grid".into());
    }
    let [foot, upper, pad] = ids;
    let origin = initial["time_ns"].as_u64().ok_or("missing cut timestamp")?;
    let mut first = initial.clone();
    first["tick"] = json!(0);
    rows.push(first);
    let variable = actions.is_some_and(|a| !a.compact_point_tails.is_empty());
    let per = if variable { 1 } else { n / 8 };
    let mut calls = 0;
    let mut available = true;
    let mut bins = vec![];
    let mut group = 0.;
    let mut impulse = DVec3::ZERO;
    let mut observer = |s: ContactSubstep<'_>| {
        calls += 1;
        for p in s
            .contact_patches()
            .into_iter()
            .filter(|p| (p.a, p.b) == (foot.min(pad), foot.max(pad)))
        {
            for site in p.points {
                if site.normal_impulse == 0. {
                    continue;
                }
                let Some(t) = site.tangent_impulse_on_a else {
                    available = false;
                    continue;
                };
                let sign = if p.a == foot { 1. } else { -1. };
                group += site.normal_impulse;
                impulse +=
                    sign * (DVec3::from(t) + site.normal_impulse * DVec3::from(p.normal_on_a));
            }
        }
        for hit in s.finite_point_impacts {
            let h = hit.resultant;
            if (h.foot, h.support) == (foot, pad) {
                let j = DVec3::from(h.impulse_on_foot);
                group += j.dot(h.normal.into());
                impulse += j;
            }
        }
        if let Some(span) = s.point_span {
            let (pair, normal, j) = span.reaction();
            if pair == (foot, pad) {
                group += normal;
                impulse += DVec3::from(j);
            }
        }
        if calls % per == 0 {
            let (Some(a), Some(u), Some(d)) = (s.state(foot), s.state(upper), s.state(pad)) else {
                available = false;
                return;
            };
            let ns = origin + s.elapsed.as_nanos() as u64;
            rows.push(json!({"tick":calls/per,"time_us":ns/1000,"time_ns":ns,"foot":Body::from(a),"upper":Body::from(u),"pad":Body::from(d),"native_steps":per,"solver_iterations":params.temporal}));
            bins.push((s.elapsed, group, impulse));
            group = 0.;
            impulse = DVec3::ZERO;
        }
    };
    let r = match actions {
        Some(actions) => {
            native
                .advance_box_contacts(dt, n, params, actions, Some(&mut observer))?
                .interval
        }
        None => native.observe_contact_interval(dt, n, params, observer)?,
    };
    if !available
        || calls != r.native_steps
        || bins.last().is_none_or(|b| b.0 != dt)
        || (!variable && (calls != n || rows.len() != 9))
        || rows.len() != bins.len() + 1
    {
        return Err("missing contact-bin observation".into());
    }
    let mut sum = DVec3::ZERO;
    for (row, (_, normal, j)) in rows[1..].iter_mut().zip(bins) {
        row["normal_impulse_ns"] = json!(normal);
        row["world_impulse_ns"] = json!(j.to_array());
        sum += j;
    }
    if let Some(pair) = r
        .pairs
        .iter()
        .find(|p| (p.a, p.b) == (foot.min(pad), foot.max(pad)))
    {
        let sign = if foot == pair.a { 1. } else { -1. };
        let public = sign
            * (DVec3::from(pair.normal_impulse_on_a)
                + DVec3::from(pair.tangent_impulse_on_a.ok_or("missing signed friction")?));
        if public.distance(sum) > 2e-10 * (1. + sum.length()) {
            return Err("live point observations disagree with published pair load".into());
        }
    } else if sum != DVec3::ZERO {
        return Err("live point load on absent pair".into());
    }
    Ok((r, sum))
}
