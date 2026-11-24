// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Closed-form plastic impacts against a translating plane, with finite-pad and free-flight certificates.
use super::*;
#[path = "impact_certification_cost.rs"]
pub(crate) mod cost;
#[path = "impact_sequence.rs"]
pub(crate) mod sequence;
const N_CASES: usize = 200;
const LEN_NS: u64 = 4_000_000;
const DT_NS: u64 = 62_500;
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Feature {
    Face,
    PitchEdge,
    RollEdge,
    Corner,
    NearCorner,
}
#[derive(Clone, Copy)]
struct Case {
    id: usize,
    kind: Feature,
    arrival_ns: u64,
    surface: DQuat,
    orientation: DQuat,
    incoming_spin: DVec3,
    pad_v: DVec3,
    incoming_v: DVec3,
    at_com: DVec3,
}
#[derive(Clone, Copy)]
struct Reference {
    case: Case,
    active: [bool; 4],
    normal: DVec3,
    j: [f64; 4],
    outgoing_v: DVec3,
    outgoing_spin: DVec3,
}
fn radical(mut i: usize, base: usize) -> f64 {
    let mut denominator = 1.;
    let mut x = 0.;
    while i > 0 {
        denominator *= base as f64;
        x += (i % base) as f64 / denominator;
        i /= base;
    }
    x
}
fn build(id: usize) -> Case {
    // Independent low-discrepancy inputs. The six original calibration fixtures are excluded.
    let index = 501 + id;
    let h = |p| radical(index, p);
    let signed = |p| 2. * h(p) - 1.;
    let kind = [
        Feature::Face,
        Feature::PitchEdge,
        Feature::RollEdge,
        Feature::Corner,
        Feature::NearCorner,
    ][id % 5];
    let sign = |v: f64| if v < 0.5 { -1. } else { 1. };
    let pitch = match kind {
        Feature::Face | Feature::RollEdge => 0.,
        Feature::NearCorner => sign(h(11)) * (0.002 + 0.018 * h(13)),
        _ => sign(h(11)) * (0.12 + 0.20 * h(13)),
    };
    let roll = match kind {
        Feature::Face | Feature::PitchEdge => 0.,
        Feature::NearCorner => sign(h(17)) * (0.000015 + 0.00045 * h(19)),
        _ => sign(h(17)) * (0.13 + 0.19 * h(19)),
    };
    let surface = DQuat::from_euler(
        glam::EulerRot::ZYX,
        0.37 * signed(3),
        0.20 * signed(5),
        0.19 * signed(7),
    );
    let q = surface * DQuat::from_euler(glam::EulerRot::ZYX, 0.31 * signed(23), pitch, roll);
    let n = surface * DVec3::Z;
    let arrival_ns = 1_750_000 + (h(2) * 950_000.).round() as u64;
    let pad_v = surface * DVec3::new(0.35 * signed(29), 0.2 * signed(31), 0.58 * signed(37));
    let incoming_spin =
        surface * DVec3::new(1.10 * signed(41), 1.30 * signed(43), 1.8 * signed(47));
    let incoming_v =
        pad_v + surface * DVec3::new(0.28 * signed(53), 0.22 * signed(59), -(0.55 + 1.45 * h(61)));
    let low = corners()
        .map(|p| (q * (p - ARM)).dot(n))
        .into_iter()
        .fold(f64::INFINITY, f64::min);
    let at_com = pad_v * (arrival_ns as f64 * 1e-9) - n * low;
    Case {
        id,
        kind,
        arrival_ns,
        surface,
        orientation: q,
        incoming_spin,
        pad_v,
        incoming_v,
        at_com,
    }
}
impl Reference {
    fn solve(c: Case) -> Result<Self> {
        let n = c.surface * DVec3::Z;
        let r = corners().map(|p| c.orientation * (p - ARM));
        let lowest = r.iter().map(|p| p.dot(n)).fold(f64::INFINITY, f64::min);
        let active = r.map(|p| (p.dot(n) - lowest).abs() < 1e-12);
        let ks: Vec<_> = (0..4).filter(|&k| active[k]).collect();
        let speed = r.map(|p| (c.incoming_v - c.pad_v + c.incoming_spin.cross(p)).dot(n));
        let col = r.map(|p| p.cross(n));
        let w = |i: usize, j: usize| 0.5 + col[i].dot(col[j]) / I;
        let mut j = [0.; 4];
        match ks[..] {
            [k] => j[k] = -speed[k] / w(k, k),
            [a, b] => {
                let (aa, bb, ab) = (w(a, a), w(b, b), w(a, b));
                let det = aa * bb - ab * ab;
                j[a] = (-speed[a] * bb + ab * speed[b]) / det;
                j[b] = (-speed[b] * aa + ab * speed[a]) / det;
            }
            [_, _, _, _] => {
                let total = M * (c.pad_v - c.incoming_v).dot(n);
                let torque = -I * (c.incoming_spin - n * c.incoming_spin.dot(n));
                let center_world = n.cross(torque) / total;
                let center = c.orientation.conjugate() * center_world + ARM;
                for (k, p) in corners().iter().enumerate() {
                    j[k] = total
                        * 0.25
                        * (1. + p.x.signum() * center.x / FOOT_HALF[0])
                        * (1. + p.y.signum() * center.y / FOOT_HALF[1]);
                }
            }
            _ => return Err("unsupported analytic contact set".into()),
        }
        let impulse: f64 = j.iter().sum();
        let v = c.incoming_v + n * impulse / M;
        let spin = c.incoming_spin + (0..4).map(|k| col[k] * j[k]).sum::<DVec3>() / I;
        Ok(Self {
            case: c,
            active,
            normal: n,
            j,
            outgoing_v: v,
            outgoing_spin: spin,
        })
    }
    fn state(self, ns: u64) -> State {
        let c = self.case;
        let t = ns as f64 * 1e-9;
        let offset = (ns as i64 - c.arrival_ns as i64) as f64 * 1e-9;
        let (v, w) = if ns < c.arrival_ns {
            (c.incoming_v, c.incoming_spin)
        } else {
            (self.outgoing_v, self.outgoing_spin)
        };
        let com = c.at_com + v * offset;
        let q = DQuat::from_scaled_axis(w * offset) * c.orientation;
        let link = |r: DVec3| Body {
            position: (com + r).to_array(),
            orientation: q.to_array(),
            velocity: (v + w.cross(r)).to_array(),
            angular_velocity: w.to_array(),
        };
        State {
            foot: link(-(q * ARM)),
            upper: link(q * ARM),
            pad: Body {
                position: (c.pad_v * t - self.normal * PAD_HALF[2]).to_array(),
                orientation: c.surface.to_array(),
                velocity: c.pad_v.to_array(),
                angular_velocity: [0.; 3],
            },
        }
    }
    fn certification(self) -> (bool, &'static str) {
        let c = self.case;
        let t0 = c.arrival_ns as f64 * 1e-9;
        let t1 = (LEN_NS - c.arrival_ns) as f64 * 1e-9;
        if self.j.iter().zip(self.active).any(|(j, a)| a && *j <= 0.) {
            return (false, "inadmissible unilateral load");
        }
        for (p, on) in corners().into_iter().zip(self.active) {
            let r = c.orientation * (p - ARM);
            let n = self.normal;
            let gap = (c.at_com + r - c.pad_v * t0).dot(n);
            let pre_v = (c.incoming_v - c.pad_v + c.incoming_spin.cross(r)).dot(n);
            let a_pre = c.incoming_spin.length_squared() * r.length();
            if pre_v + a_pre * t0 >= 0. {
                return (false, "prior contact cannot be excluded");
            }
            let w = self.outgoing_spin;
            let post_v = (self.outgoing_v - c.pad_v + w.cross(r)).dot(n);
            if on {
                if post_v.abs() > 1e-10 {
                    return (false, "post-impact normal velocity");
                }
                let alpha = w.cross(w.cross(r)).dot(n);
                // g''(t) is Lipschitz with bound |w|^3 |r|. Pure rotation about n
                // leaves all normal coordinates invariant.
                if w.cross(n).length() > 1e-10
                    && alpha - w.length().powi(3) * r.length() * t1 < -1e-12
                {
                    return (false, "active point may require sustained contact");
                }
            } else if gap + post_v.min(0.) * t1 - 0.5 * w.length_squared() * r.length() * t1 * t1
                <= 0.
            {
                return (false, "secondary corner may reach plane");
            }
            let reach =
                ((c.incoming_v - c.pad_v).length() + c.incoming_spin.length() * r.length()) * t0;
            let after = ((self.outgoing_v - c.pad_v).length() + w.length() * r.length()) * t1;
            let xy = c.surface.conjugate() * (c.at_com + r - c.pad_v * t0);
            if xy.x.abs() + reach.max(after) >= PAD_HALF[0]
                || xy.y.abs() + reach.max(after) >= PAD_HALF[1]
            {
                return (false, "finite top-face margin");
            }
        }
        // Upper support value along n, inflated by the maximum swept distance.
        let extent = (c.orientation.conjugate() * self.normal)
            .abs()
            .dot(UPPER_HALF.into());
        let radius = ARM.length() + DVec3::from(UPPER_HALF).length();
        let h = (c.at_com + c.orientation * ARM - c.pad_v * t0).dot(self.normal) - extent;
        if h <= (((c.incoming_v - c.pad_v).length() + radius * c.incoming_spin.length()) * t0)
            .max(((self.outgoing_v - c.pad_v).length() + radius * self.outgoing_spin.length()) * t1)
        {
            return (false, "upper link may reach plane");
        }
        (true, "certified")
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Policy {
    Default,
    Relaxation,
    Refresh,
    Combined,
    Scoped,
}
fn schedule(s: State, policy: Policy) -> (bool, usize) {
    match policy {
        Policy::Default => (false, 1),
        Policy::Relaxation => (false, 16),
        Policy::Refresh => (true, 1),
        Policy::Combined => (true, 16),
        Policy::Scoped => {
            let n = DQuat::from_array(s.pad.orientation) * DVec3::Z;
            let dt = DT_NS as f64 * 1e-9;
            let q = DQuat::from_array(s.foot.orientation);
            let w = DVec3::from(s.foot.angular_velocity);
            let base = DVec3::from(s.foot.position) - DVec3::from(s.pad.position);
            let relative = DVec3::from(s.foot.velocity) - DVec3::from(s.pad.velocity);
            let mut count = 0;
            for arm in corners().map(|r| q * r) {
                let gap = (base + arm).dot(n) - PAD_HALF[2];
                let speed = (relative + w.cross(arm)).dot(n);
                let uncertainty = 0.5 * w.length_squared() * arm.length() * dt * dt + 1e-8;
                if gap + speed.min(0.) * dt <= uncertainty {
                    count += 1
                }
            }
            (count > 0, if count > 1 { 16 } else { 1 })
        }
    }
}
fn relative_state_error(a: State, b: State) -> (f64, f64, f64) {
    let mut p = 0.0_f64;
    let mut v = 0.0_f64;
    let mut w = 0.0_f64;
    for (l, r) in [(a.foot, b.foot), (a.upper, b.upper), (a.pad, b.pad)] {
        p = p.max(DVec3::from(l.position).distance(r.position.into()));
        v = v.max(DVec3::from(l.velocity).distance(r.velocity.into()));
        w = w.max(DVec3::from(l.angular_velocity).distance(r.angular_velocity.into()));
    }
    (p, v, w)
}
fn advance(
    native: &mut MultiFidelityBackend,
    ids: Bodies,
    refresh: bool,
    relax: usize,
) -> std::result::Result<(), PhysicsError> {
    let dt = Duration::from_nanos(DT_NS);
    if refresh {
        native.step_landing_with_fresh_box_pairs(dt, relax, &[(ids.foot, ids.pad)])
    } else if relax == 1 {
        native.step(dt)
    } else {
        native.step_landing_with_relaxation(dt, relax)
    }
}
fn evaluate(
    r: Reference,
    tree: bool,
    pad_first: bool,
    policy: Policy,
    visual: bool,
) -> Result<(Value, Vec<Snapshot>)> {
    let initial = r.state(0);
    let (mut engine, ids) = fresh_in_order(initial, tree, 1, pad_first)?;
    let init_err = relative_state_error(state(&engine, ids)?, initial);
    if init_err.0 > 1e-12 || init_err.1 > 1e-12 || init_err.2 > 1e-12 {
        return Err(format!("unmatched incoming {:?} #{}", init_err, r.case.id).into());
    }
    let mut frames = vec![];
    let mut corner = [0.; 4];
    let mut outer = [0.; 4];
    let mut total_vector = DVec3::ZERO;
    let mut event_vector = DVec3::ZERO;
    let mut peak = 0.;
    let mut peak_time = 0;
    let mut first = 0;
    let mut max_loaded_normal = 0.0_f64;
    let mut before = initial;
    let mut before_spin = 0.;
    let mut before_v = 0.;
    let mut before_p = 0.;
    let mut refreshed = 0;
    let mut relaxed = 0;
    let snapshot = |tick, state, native_steps, corner_ns: [f64; 4], selected| Snapshot {
        tick,
        time_us: tick * 500,
        state,
        native_steps,
        selected,
        secondary_start: false,
        normal_impulse_ns: corner_ns.iter().sum(),
        corner_ns,
    };
    if visual {
        frames.push(snapshot(0, initial, 0, outer, false));
    }
    let mut window = false;
    for i in 1..=LEN_NS / DT_NS {
        let t = i * DT_NS;
        let (refresh, relax) = schedule(before, policy);
        advance(&mut engine, ids, refresh, relax)?;
        refreshed += usize::from(refresh);
        relaxed += usize::from(relax > 1);
        window |= refresh || relax > 1;
        let post = state(&engine, ids)?;
        let inverse = DQuat::from_array(before.foot.orientation).conjugate();
        let mut vector = DVec3::ZERO;
        for p in engine.rigid_contact_patches() {
            let direct = (p.a, p.b) == (ids.foot, ids.pad);
            if !direct && (p.b, p.a) != (ids.foot, ids.pad) {
                if p.points.iter().any(|p| p.normal_impulse > 1e-10) {
                    return Err("unexpected native upper-contact impulse in certification".into());
                }
                continue;
            }
            let normal = DVec3::from(p.normal_on_a) * if direct { 1. } else { -1. };
            for point in p.points {
                let j = point.normal_impulse;
                let local = inverse
                    * DVec3::from(if direct { point.lever_a } else { point.lever_b })
                    + if tree { DVec3::ZERO } else { ARM };
                let k = 2 * usize::from(local.x > 0.) + usize::from(local.y > 0.);
                corner[k] += j;
                outer[k] += j;
                vector += normal * j;
                if j > 1e-9 {
                    max_loaded_normal = max_loaded_normal.max(normal.distance(r.normal));
                }
            }
        }
        if vector.length() > 1e-8 && first == 0 {
            first = t;
            let pt = r.state(t - DT_NS);
            // Pre-impact native trajectory is compared with the incoming branch even
            // when the first significant receipt occurs after geometric arrival.
            if t - DT_NS < r.case.arrival_ns {
                (before_p, before_v, before_spin) = relative_state_error(before, pt);
            } else {
                let q = r.case;
                let tau = (t - DT_NS - q.arrival_ns) as f64 * 1e-9;
                let predicted = q.at_com + q.incoming_v * tau;
                before_p = before.com().distance(predicted);
                before_v = before.velocity().distance(q.incoming_v);
                before_spin = DVec3::from(before.foot.angular_velocity).distance(q.incoming_spin);
            }
        }
        if vector.length() > peak {
            peak = vector.length();
            peak_time = t;
            event_vector = vector;
        }
        total_vector += vector;
        before = post;
        if i % 8 == 0 {
            if visual {
                frames.push(snapshot(i / 8, post, 8, outer, window));
            }
            outer = [0.; 4];
            window = false;
        }
    }
    let exact = r.state(LEN_NS);
    let j = r.j.iter().sum::<f64>();
    let end = before;
    Ok((
        json!({"representation":if tree{"locked_pair"}else{"single_rigid"},"pad_first":pad_first,"policy":policy,
        "first_load_ns":first,"dominant_ns":peak_time,"dominant_vector_ns":event_vector.to_array(),"reaction_vector_ns":total_vector.to_array(),"corner_ns":corner,
        "signed_normal_error_ns":total_vector.dot(r.normal)-j,"impulse_vector_error_ns":total_vector.distance(r.normal*j),
        "angular_error_rad_s":DVec3::from(end.foot.angular_velocity).distance(r.outgoing_spin),"com_error_m":end.com().distance(exact.com()),
        "com_velocity_error_m_s":end.velocity().distance(r.outgoing_v),"max_normal_deviation":max_loaded_normal,"refreshed_calls":refreshed,"relaxed_calls":relaxed,
        "preload_position_error_m":before_p,"preload_velocity_error_m_s":before_v,"preload_spin_error_rad_s":before_spin}),
        frames,
    ))
}
fn analytic_trace(r: Reference) -> Vec<Snapshot> {
    (0..=8)
        .map(|i| {
            let t = i * 500_000;
            let corner = if i > 0 && r.case.arrival_ns > (i - 1) * 500_000 && r.case.arrival_ns <= t
            {
                r.j
            } else {
                [0.; 4]
            };
            Snapshot {
                state: r.state(t),
                tick: i,
                time_us: t / 1000,
                native_steps: 0,
                selected: false,
                secondary_start: false,
                normal_impulse_ns: corner.iter().sum(),
                corner_ns: corner,
            }
        })
        .collect()
}
fn write_view(path: &Path, r: Reference) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    for (label, policy) in [("default", Policy::Default), ("scoped", Policy::Scoped)] {
        let (_, frames) = evaluate(r, true, false, policy, true)?;
        trace(path, label, &frames)?;
    }
    trace(path, "analytic", &analytic_trace(r))?;
    write_json(
        &path.join("comparison.json"),
        &json!({"title":format!("Certified {:?} impact on a moving inclined pad",r.case.kind),"lanes":[
        {"label":"default","name":"Default locked foot","file":"traces/default.json"},{"label":"scoped","name":"Scoped feature refresh and relaxation","file":"traces/scoped.json"},
        {"label":"analytic","name":"Analytic plastic impact","file":"traces/analytic.json","analytical":true}]}),
    )
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut cases = vec![];
    let mut chose_edge = false;
    let mut chose_corner = false;
    for id in 0..N_CASES {
        let r = Reference::solve(build(id))?;
        let (certified, reason) = r.certification();
        let mut runs = vec![];
        for pad_first in [false, true] {
            for policy in [
                Policy::Default,
                Policy::Relaxation,
                Policy::Refresh,
                Policy::Combined,
                Policy::Scoped,
            ] {
                runs.push(evaluate(r, true, pad_first, policy, false)?.0);
            }
            runs.push(evaluate(r, false, pad_first, Policy::Default, false)?.0);
        }
        if certified && r.case.kind == Feature::PitchEdge && !chose_edge {
            write_view(path, r)?;
            chose_edge = true;
        }
        if certified && r.case.kind == Feature::Corner && !chose_corner {
            write_view(&path.join("corner"), r)?;
            chose_corner = true;
        }
        cases.push(json!({"case":r.case.id,"feature":r.case.kind,"certified":certified,"qualification":reason,
            "arrival_ns":r.case.arrival_ns,"initial":r.state(0),"at_impact":r.state(r.case.arrival_ns),"target":r.state(LEN_NS),
            "active":r.active,"impulses_ns":r.j,"normal":r.normal.to_array(),"runs":runs}));
    }
    if !chose_edge || !chose_corner {
        return Err("missing certified replay fixture".into());
    }
    write_json(
        &path.join("certification.json"),
        &json!({"fixtures":N_CASES,"native_dt_ns":DT_NS,"actuator_dt_ns":500000,"duration_ns":LEN_NS,"cases":cases}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn incoming_and_outgoing_contact_set_satisfy_plastic_impact_conditions() {
        let mut certified = 0;
        for id in 0..N_CASES {
            let r = Reference::solve(build(id)).unwrap();
            if !r.certification().0 {
                continue;
            }
            certified += 1;
            let hit = r.state(r.case.arrival_ns);
            for (k, p) in corners().iter().enumerate() {
                if !r.active[k] {
                    continue;
                }
                let arm = r.case.orientation * (*p - ARM);
                assert!(
                    (hit.velocity() - r.case.pad_v + r.outgoing_spin.cross(arm))
                        .dot(r.normal)
                        .abs()
                        < 1e-10
                );
                assert!(
                    (r.case.at_com + arm - r.case.pad_v * (r.case.arrival_ns as f64 * 1e-9))
                        .dot(r.normal)
                        .abs()
                        < 1e-12
                );
                assert!(r.j[k] > 0.);
            }
        }
        assert!(certified >= 100);
    }
    #[test]
    fn selected_box_refresh_rejects_an_invalid_second_pair_before_any_step() {
        let r = Reference::solve(build(3)).unwrap();
        let (mut native, ids) = fresh_in_order(r.state(0), true, 1, false).unwrap();
        let before = state(&native, ids).unwrap();
        assert!(
            native
                .step_landing_with_fresh_box_pairs(
                    Duration::from_nanos(DT_NS),
                    4,
                    &[(ids.foot, ids.pad), (ids.pad, ids.foot)]
                )
                .is_err()
        );
        assert!(state(&native, ids).unwrap() == before);
    }
    #[test]
    fn unrelated_pair_and_two_simultaneous_pairs_keep_independent_impulses() {
        let r = Reference::solve(build(3)).unwrap();
        let initial = r.state(0);
        let step = |selected: usize| {
            let (mut native, first) = fresh_in_order(initial, true, 1, false).unwrap();
            let shift = |mut b: Body| {
                b.position[0] += 2.;
                b
            };
            let [foot, upper] =
                spawn_locked_pair(&mut native, shift(initial.foot), shift(initial.upper)).unwrap();
            let pad = native
                .spawn_ideal_landing_surface(
                    cube(PAD_HALF, shift(initial.pad), 1.),
                    orientation(initial.pad),
                )
                .unwrap();
            let second = Bodies {
                foot,
                upper: Some(upper),
                pad,
            };
            let pairs = [(first.foot, first.pad), (second.foot, second.pad)];
            let mut sum = [DVec3::ZERO; 2];
            for _ in 0..64 {
                if selected == 0 {
                    native.step(Duration::from_nanos(DT_NS)).unwrap();
                } else {
                    native
                        .step_landing_with_fresh_box_pairs(
                            Duration::from_nanos(DT_NS),
                            1,
                            &pairs[..selected],
                        )
                        .unwrap();
                }
                for patch in native.rigid_contact_patches() {
                    for (i, (a, b)) in pairs.into_iter().enumerate() {
                        let sign = if (patch.a, patch.b) == (a, b) {
                            1.
                        } else if (patch.b, patch.a) == (a, b) {
                            -1.
                        } else {
                            continue;
                        };
                        sum[i] += DVec3::from(patch.normal_on_a)
                            * sign
                            * patch.points.iter().map(|p| p.normal_impulse).sum::<f64>();
                    }
                }
            }
            sum
        };
        let default = step(0);
        let single = step(1);
        let both = step(2);
        assert!(single[1].distance(default[1]) < 1e-12);
        assert!(both[0].distance(single[0]) < 1e-12);
        assert!(both[1].distance(both[0]) < 1e-8);
    }
    #[test]
    fn side_and_separating_faces_receive_only_native_side_reactions() {
        let side = |inward: bool, refresh: bool| -> Result<DVec3> {
            let mut engine = MultiFidelityBackend::new([0.; 3])?;
            let orientation = DQuat::from_euler(glam::EulerRot::ZYX, 0.31, -0.17, 0.26);
            let n = orientation * DVec3::X;
            let body = |p: DVec3, v: DVec3| Body {
                position: p.to_array(),
                orientation: orientation.to_array(),
                velocity: v.to_array(),
                angular_velocity: [0.; 3],
            };
            let drift = DVec3::new(-0.17, 0.12, 0.3);
            let h = [0.018, 0.018, 0.018];
            let foot = body(
                n * (PAD_HALF[0] + h[0] + 0.00011),
                drift + n * if inward { -0.95 } else { 0.95 },
            );
            let pad = body(DVec3::ZERO, drift);
            let id = engine.spawn_link(
                cube(h, foot, 1.),
                super::orientation(foot),
                inertia([0.; 3], [0.001; 3]),
            )?;
            let deck = engine
                .spawn_ideal_landing_surface(cube(PAD_HALF, pad, 1.), super::orientation(pad))?;
            let mut j = DVec3::ZERO;
            for _ in 0..24 {
                if refresh {
                    engine.step_landing_with_fresh_box_pairs(
                        Duration::from_nanos(DT_NS),
                        16,
                        &[(id, deck)],
                    )?;
                } else {
                    engine.step_landing_with_relaxation(Duration::from_nanos(DT_NS), 16)?;
                }
                for p in engine.rigid_contact_patches() {
                    let sign = if p.a == id { 1. } else { -1. };
                    j += sign
                        * DVec3::from(p.normal_on_a)
                        * p.points.iter().map(|x| x.normal_impulse).sum::<f64>();
                }
            }
            if inward {
                assert!(j.distance(n * 0.95) < 1e-6, "side impulse {j:?}");
            } else {
                assert_eq!(j, DVec3::ZERO);
            }
            Ok(j)
        };
        for inward in [false, true] {
            assert!(
                side(inward, true)
                    .unwrap()
                    .distance(side(inward, false).unwrap())
                    < 1e-6
            );
        }
    }
}
