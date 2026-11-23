// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time proposals for a measured direction change of an articulated cloth contact.
use super::{ArticulatedSupportInterval, RESIDUAL, SupportApplication};
use glam::{DQuat, DVec3};
use gz_physics_xpbd::articulated_impact::MaterialSupportLoad;
use std::time::Duration;
pub(super) struct Proposal {
    pub(super) next: u64,
    pub(super) horizon: u64,
}
fn matching<'a>(
    a: &'a SupportApplication,
    source: &MaterialSupportLoad,
) -> Option<&'a MaterialSupportLoad> {
    a.material
        .iter()
        .find(|l| l.site.sphere == source.site.sphere && l.site.feature == source.site.feature)
}
fn transported_slip(from: &MaterialSupportLoad, to: &MaterialSupportLoad) -> Option<DVec3> {
    let f = from.friction?;
    let (n0, n1) = (DVec3::from(from.site.normal), DVec3::from(to.site.normal));
    if n0.dot(n1) < 0.5
        || n0.dot(DVec3::from(from.on_rigid)) >= 0.
        || n1.dot(DVec3::from(to.on_rigid)) >= 0.
    {
        return None;
    }
    Some(DQuat::from_rotation_arc(n0, n1) * DVec3::from(f.initial_slip))
}
/// True endpoint velocities on the same loaded material region, expressed
/// in a common tangent plane. A chord remaining well clear of zero provides
/// no evidence of a kinetic arrest.
fn candidate(before: &MaterialSupportLoad, after: &MaterialSupportLoad, dt: f64) -> Option<DVec3> {
    let s = transported_slip(before, after)?;
    let e = DVec3::from(after.friction?.final_slip);
    if s.length() * dt <= RESIDUAL || (s.dot(e) > 0. && e.length() * dt > RESIDUAL) {
        return None;
    }
    let change = e - s;
    if change.length_squared() == 0. {
        return None;
    }
    let t = (-s.dot(change) / change.length_squared()).clamp(0., 1.);
    ((s + t * change).length_squared() <= 0.01 * s.length_squared().max(e.length_squared()))
        .then_some(DVec3::from(before.friction?.initial_slip))
}
/// Extrapolate the previous *accepted* skin speed to suggest a cut near a
/// possible zero. This does not authorize any advance; the ordinary coupled
/// integrator evaluates every proposed interval from the actual current cut.
fn predict(
    previous: &ArticulatedSupportInterval,
    before: &MaterialSupportLoad,
    s: DVec3,
    h: f64,
) -> Option<f64> {
    let tail = matching(previous.end.as_ref()?, before)?;
    // A previous published cut can survive a user edit between public ticks.
    // Reuse only its measured local velocity trend when the end site and
    // actual tangent velocity still agree at the current physical source.
    if tail.site != before.site
        || DVec3::from(tail.friction?.final_slip).distance(s) > 1e-11 * (1. + s.length())
    {
        return None;
    }
    let p = matching(previous.beginning.as_ref()?, before)?;
    let v0 = transported_slip(p, before)?;
    let decay = (s - v0) / previous.duration.as_secs_f64();
    if decay.length_squared() == 0. {
        return None;
    }
    let t = -s.dot(decay) / decay.length_squared();
    if !(t > 0. && t < h) || (s + t * decay).length_squared() > 0.04 * s.length_squared() {
        return None;
    }
    Some(t)
}
pub(super) fn proposal(
    start: &SupportApplication,
    end: &SupportApplication,
    prior: Option<&ArticulatedSupportInterval>,
    dt: Duration,
    cap: Duration,
    minimum: u64,
) -> Option<Proposal> {
    let h = dt.as_nanos() as u64;
    let mut first = f64::INFINITY;
    let mut last = 0f64;
    let mut agreeing = 0;

    let mut seen = false;
    for before in &start.material {
        let Some(after) = matching(end, before) else {
            continue;
        };
        let Some(s) = candidate(before, after, dt.as_secs_f64()) else {
            continue;
        };
        seen = true;
        if let Some(t) = prior.and_then(|p| predict(p, before, s, dt.as_secs_f64())) {
            first = first.min(t);
            last = last.max(t);
            agreeing += 1;
        }
    }
    if !seen {
        return None;
    }
    // Contacts sharing material or a reduced native tree can turn at different
    // times. Extrapolate a common turning point only when all physical rows
    // support it independently within the requested time resolution.
    if agreeing != start.material.len()
        || agreeing != end.material.len()
        || last - first > 0.5 * cap.as_secs_f64()
    {
        return Some(Proposal {
            next: h / 2,
            horizon: h,
        });
    }
    let cap = cap.as_nanos() as u64;
    let near = (first * 1e9).round() as u64;
    let margin = (cap / 4).max(minimum);
    let next = near.saturating_sub(margin).clamp(margin, h - margin);
    let horizon = near.saturating_add(margin).min(h).max(next + minimum);
    Some(Proposal { next, horizon })
}
