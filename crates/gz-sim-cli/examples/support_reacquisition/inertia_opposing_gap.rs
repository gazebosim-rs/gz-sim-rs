// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resolve small positive heights from a known material tangency without subtracting world poses.
use super::*;
#[path = "inertia_gap_compact.rs"]
pub(crate) mod compact;
#[path = "inertia_gap_view.rs"]
mod display;
#[path = "inertia_gap_limit.rs"]
mod limit;
#[path = "inertia_gap_terminal.rs"]
pub(crate) mod terminal;
#[path = "inertia_gap_uncertainty.rs"]
mod uncertainty;
#[path="inertia_native_normal_series.rs"]
pub(crate) mod native_normal;
#[derive(Clone, Copy)]
struct Site {
    side: usize,
    k: usize,
}
impl Site {
    fn of(h: Crossing) -> Self {
        Self {
            side: h.surface,
            k: h.corner,
        }
    }
}
#[derive(Clone, Copy)]
struct Interval {
    lo: f64,
    mid: f64,
    hi: f64,
}
impl Interval {
    fn data(self) -> Value {
        json!([self.lo, self.mid, self.hi])
    }
    fn positive(self) -> bool {
        self.lo.is_finite()
            && 0. < self.lo
            && self.lo <= self.mid
            && self.mid <= self.hi
            && self.hi.is_finite()
    }
}
#[derive(Clone, Copy)]
struct Jet {
    v: f64,
    a: f64,
    jerk: f64,
    third: f64,
    snap: f64,
    jeps: f64,
    veps: f64,
    aeps: f64,
}
fn jet(r: &Physical, x: Flight, site: Site, plane: Face) -> Jet {
    let n = plane.frame * DVec3::Z;
    let b = Bounds::new(r.body, x);
    let w = r.body.omega(x.q, x.l);
    let arm = x.q * vertices()[site.k];
    let radial = r.body.accel(x.q, w).cross(arm) + w.cross(w.cross(arm));
    let u = x.v - plane.v + w.cross(arm);
    let wb = x.q.conjugate() * w;
    let tensor = r.body.tensor;
    let ab = r.body.inverse * ((tensor * wb).cross(wb));
    let alpha = x.q * ab;
    let beta =
        x.q * (wb.cross(ab) + r.body.inverse * ((tensor * ab).cross(wb) + (tensor * wb).cross(ab)));
    let wr = w.cross(arm);
    let third =
        beta.cross(arm) + 2. * alpha.cross(wr) + w.cross(alpha.cross(arm)) + w.cross(w.cross(wr));
    let ratio = (r.body.upper - r.body.lower) / r.body.lower;
    let gamma = (1. + ratio).powi(2) * b.w * b.w * b.alpha + ratio * b.alpha * b.alpha;
    let global_j = (b.beta + 3. * b.w * b.alpha + b.w.powi(3)) * arm.length();
    let snap = (gamma
        + 4. * b.w * b.beta
        + 3. * b.alpha * b.alpha
        + 6. * b.w * b.w * b.alpha
        + b.w.powi(4))
        * arm.length();
    Jet {
        v: u.dot(n),
        a: (r.gravity + radial).dot(n),
        jerk: global_j,
        third: third.dot(n),
        snap: snap * (1. + 4096. * f64::EPSILON),
        jeps: 4096. * f64::EPSILON * (1. + global_j),
        veps: 1024. * f64::EPSILON * (1. + x.v.length() + plane.v.length() + b.w * arm.length()),
        aeps: 2048.
            * f64::EPSILON
            * (1. + r.gravity.length() + (b.alpha + b.w.powi(2)) * arm.length()),
    }
}
/// On a known perfectly inelastic, releasing vertex, g(0)=g'(0)=0.
/// Bound g(t)=integral((t-s)a(s) ds) with directional derivatives and a separate attitude error bound.
fn curvature(
    r: &Physical,
    x: Flight,
    site: Site,
    plane: Face,
    t: f64,
    slices: usize,
) -> Option<(Interval, f64)> {
    if !t.is_finite() || t <= 0. {
        return None;
    }
    let initial = jet(r, x, site, plane);
    // Signed cubic Taylor model from the matched cut. A global bound on its
    // fourth derivative encloses the actual normal displacement. No subtraction
    // of translated world coordinates occurs in the small released height.
    let center = t * t * (initial.a / 2. + initial.third * t / 6.);
    let err = t * t * (initial.aeps / 2. + t * (initial.jeps / 6. + t * initial.snap / 24.))
        + 16. * f64::EPSILON * center.abs();
    let amin = initial.a - initial.aeps + (initial.third - initial.jeps).min(0.) * t
        - 0.5 * initial.snap * t * t;
    let dt = t / slices as f64;
    let mut state = x;
    let mut integral = 0.;
    let mut piecewise = 0.;
    let mut uncertainty = 0.;
    let mut qerr = 0.;
    let mut least = f64::INFINITY;
    let b = Bounds::new(r.body, x);
    let wcap = x.l.length() / r.body.lower;
    let ratio = (r.body.upper - r.body.lower) / r.body.lower;
    let acap = 0.5 * ratio * wcap * wcap;
    let length = vertices()[site.k].length();
    let acceler_lipschitz = 2. * length * (acap + wcap * wcap * (3. + ratio));
    let lip_q = 2.5 * wcap;
    let growth = (lip_q * dt).exp();
    let q3 = 0.5 * b.beta + 0.75 * b.w * b.alpha + 0.125 * b.w.powi(3);
    for k in 0..slices {
        let left = jet(r, state, site, plane);
        let mut next = state;
        next.advance(r, dt);
        let w = r.body.omega(state.q, state.l);
        let alpha = r.body.accel(state.q, w);
        let dq = (pure(w) * state.q) * 0.5;
        let ddq = (pure(alpha) * state.q + pure(w) * dq) * 0.5;
        let taylor = state.q + dq * dt + ddq * (0.5 * dt * dt);
        let defect = (next.q - taylor).length() + q3 * dt.powi(3) / 6. + 64. * f64::EPSILON;
        let interval_err = qerr * growth;
        qerr = interval_err + defect;
        let err_a = acceler_lipschitz * interval_err
            + left.aeps
            + left.jeps * dt
            + 0.5 * left.snap * dt * dt;
        least = least.min(left.a + left.third.min(0.) * dt - err_a);
        // Integral of (t-s)*(A_left+J_left*s), translated to this subinterval.
        let remain = (slices - k) as f64;
        let weight = (remain - 0.5) * dt * dt;
        piecewise += left.a * weight + left.third * (0.5 * remain - 1. / 3.) * dt * dt * dt;
        uncertainty += (err_a + 32. * f64::EPSILON * left.a.abs()) * weight;
        for node in [0.21132486540518713, 0.7886751345948129] {
            let mut p = state;
            p.advance(r, dt * node);
            integral += jet(r, p, site, plane).a * (remain - node) * 0.5 * dt * dt;
        }
        state = next;
    }
    let sum_eps = 8. * slices as f64 * f64::EPSILON * (piecewise.abs() + uncertainty);
    uncertainty += sum_eps;
    let lo = (center - err).max(piecewise - uncertainty);
    let hi = (center + err).min(piecewise + uncertainty);
    if !integral.is_finite()
        || !lo.is_finite()
        || !hi.is_finite()
        || lo > hi
        || integral < lo - sum_eps
        || integral > hi + sum_eps
    {
        return None;
    }
    let range = Interval {
        lo,
        mid: integral.clamp(lo, hi),
        hi,
    };
    let amin = amin.max(least);
    Some((range, amin))
}
// An inexpensive cubic enclosure for short, regular intervals. A global
// angular snap bound covers the Taylor remainder. The full signed-curvature
// integrator remains available when the cubic enclosure is insufficient.
fn cubic(r: &Physical, x: Flight, site: Site, plane: Face, t: f64) -> Option<(Interval, f64)> {
    if !t.is_finite() || t < 1e-153 {
        return None;
    }
    let j = jet(r, x, site, plane);
    let mid = t * t * (j.a / 2. + j.third * t / 6.);
    let err = t * t * (j.aeps / 2. + t * (j.jeps / 6. + t * j.snap / 24.))
        + 32. * f64::EPSILON * mid.abs();
    let amin = j.a - j.aeps + (j.third - j.jeps).min(0.) * t - 0.5 * j.snap * t * t;
    Some((
        Interval {
            lo: mid - err,
            mid,
            hi: mid + err,
        },
        amin,
    ))
}
fn bend(
    r: &Physical,
    x: Flight,
    site: Site,
    plane: Face,
    t: f64,
    slices: usize,
    quick: bool,
) -> Option<(Interval, f64)> {
    if quick
        && let Some(mut v) = cubic(r, x, site, plane, t)
        && v.0.hi - v.0.lo <= 0.01 * v.0.mid.abs().max(1e-100)
    {
        // The enclosure uses the cubic remainder. For longer flights use
        // Gauss quadrature only for the nominal trajectory inside that enclosure.
        if t > 1e-7 {
            let mut sum = 0.;
            for node in [0.21132486540518713, 0.7886751345948129] {
                let mut y = x;
                y.advance(r, t * node);
                sum += (1. - node) * 0.5 * jet(r, y, site, plane).a;
            }
            v.0.mid = (t * (t * sum)).clamp(v.0.lo, v.0.hi);
        }
        return Some(v);
    }
    curvature(r, x, site, plane, t, slices)
}
fn released(
    r: &Physical,
    x: Flight,
    site: Site,
    plane: Face,
    t: f64,
    slices: usize,
) -> Option<(Interval, f64)> {
    let (v, a) = curvature(r, x, site, plane, t, slices)?;
    (v.positive() && a > 0.).then_some((v, a))
}
fn crossing(g: Interval, j: Jet) -> std::result::Result<Interval, &'static str> {
    if !g.positive() || !j.v.is_finite() || j.v >= -1e-7 {
        return Err("no resolved closing velocity");
    }
    let h = 3. * g.hi / (-j.v - j.veps);
    if !h.is_finite() || h <= 0. || h > 0.001 {
        return Err("outside local flight horizon");
    }
    let speed = j.v + j.veps + (j.a.abs() + j.aeps) * h + 0.5 * j.jerk * h * h;
    if speed >= 0. {
        return Err("non-monotone arrival");
    }
    let residual = |t: f64, sign: f64| {
        let start = if sign == 0. {
            g.mid
        } else if sign < 0. {
            g.lo
        } else {
            g.hi
        };
        start
            + t * (j.v
                + sign * j.veps
                + t * (0.5 * (j.a + sign * j.aeps)
                    + t * ((j.third + sign * j.jeps) / 6. + sign * t * j.snap / 24.)))
    };
    if residual(h, 1.) >= 0. {
        return Err("unbounded arrival");
    }
    let solve = |sign| {
        let (mut lo, mut hi) = (0., h);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if residual(mid, sign) > 0. {
                lo = mid
            } else {
                hi = mid
            }
        }
        (lo, hi)
    };
    let range = Interval {
        lo: solve(-1.).0,
        mid: {
            let (a, b) = solve(0.);
            0.5 * (a + b)
        },
        hi: solve(1.).1,
    };
    if !range.positive() {
        return Err("time resolution exhausted");
    }

    Ok(range)
}
fn protect(
    r: &Physical,
    x: Flight,
    planes: [Face; 2],
    sites: [Site; 2],
    global: f64,
    h: f64,
) -> Option<(f64, f64)> {
    let b = Bounds::new(r.body, x);
    let mut min = f64::INFINITY;
    let mut margin = f64::INFINITY;
    for (side, f) in planes.iter().enumerate() {
        for (k, local) in vertices().iter().enumerate() {
            let p = x.com + x.q * (*local);
            let coords = f.coordinates(p, global);
            let travel =
                ((x.v - f.v).length() + b.w * local.length() + 0.5 * r.gravity.length() * h) * h;
            margin = margin.min((f.half - coords.truncate().abs()).min_element() - travel);
            if !sites.iter().any(|s| s.side == side && s.k == k) {
                min = min.min(coords.z - travel);
            }
        }
    }
    (min > 1e-11 && margin > 1e-10).then_some((min, margin))
}
fn advance(r: &Physical, mut x: Flight, t: f64) -> Flight {
    let n = (t / (0.2e-6)).ceil().max(1.) as u32;
    for _ in 0..n {
        x.advance(r, t / f64::from(n));
    }
    x
}
/// Instantaneous frictionless two-normal control at the measured pose. Allow an
/// alternating impact sequence only for strictly positive diagonal response and
/// a contracting negative off-diagonal coupling.
fn fixed_normals(r: &Physical, x: Flight, p: [Face; 2], sites: [Site; 2]) -> Option<Value> {
    let n = p.map(|f| f.frame * DVec3::Z);
    let arm = sites.map(|s| x.q * vertices()[s.k]);
    let vel = [0, 1].map(|k| (x.v - p[k].v + r.body.omega(x.q, x.l).cross(arm[k])).dot(n[k]));
    let response = |row: usize, col: usize| {
        n[row].dot(n[col] / M + r.body.omega(x.q, arm[col].cross(n[col])).cross(arm[row]))
    };
    let (a, b, c) = (response(0, 0), response(0, 1), response(1, 1));
    let det = a * c - b * b;
    if det <= 1e-12 || b >= 0. || vel[1] >= 0. {
        return None;
    }
    let target = [
        (-c * vel[0] + b * vel[1]) / det,
        (b * vel[0] - a * vel[1]) / det,
    ];
    if target.iter().any(|v| *v <= 0.) {
        return None;
    }
    let mut u = vel;
    let mut sums = [0., 0.];
    let mut iterations = 0;
    for k in (0..16384).map(|k| 1 - k % 2) {
        let j = (-u[k] / if k == 1 { c } else { a }).max(0.);
        if j <= 1e-13 * (1. + target[0] + target[1]) {
            break;
        }
        sums[k] += j;
        for (i, v) in u.iter_mut().enumerate() {
            *v += response(i, k) * j;
        }
        iterations += 1;
    }
    Some(
        json!({"normal_matrix":[a,b,b,c],"velocity":vel,"rho":b*b/(a*c),"simultaneous_ns":target,"alternating_ns":sums,"remaining_velocity":u,"iterations":iterations}),
    )
}
struct Sequence {
    finite: Vec<Crossing>,
    records: Vec<Value>,
    reason: &'static str,
    third: Option<(Crossing, Interval)>,
    first: Value,
    limit: Value,
}
fn sequence(i: &Impact, g: &Gate, upper: Crossing, lower: Crossing, mu: [f64; 2]) -> Sequence {
    evaluate(i, g, upper, lower, mu, None)
}
fn evaluate(
    i: &Impact,
    g: &Gate,
    upper: Crossing,
    lower: Crossing,
    mu: [f64; 2],
    goal: Option<limit::Goal>,
) -> Sequence {
    let quick = goal.is_some();
    let r = &i.r;
    let planes = faces(r, g);
    let sites = [Site::of(lower), Site::of(upper)];
    let prior = upper.after();
    let flight = lower.at - upper.at;
    let fail = |reason| Sequence {
        records: vec![],
        finite: vec![],
        reason,
        third: None,
        first: Value::Null,
        limit: Value::Null,
    };
    let initial = lower.after();
    let (mut gap, seed) = if quick {
        let first = bend(r, prior, sites[1], planes[1], flight, 32, true);
        let first = if first.is_some_and(|(g, _)| g.hi - g.lo > 1e-8 * g.mid.abs()) {
            curvature(r, prior, sites[1], planes[1], flight, 32)
        } else {
            first
        };
        let Some((g, a)) = first else {
            return fail("initial coarse release");
        };
        if !g.positive() || a <= 0. {
            return fail("initial coarse release");
        }
        (g, Value::Null)
    } else {
        let Some((g, _)) = released(r, prior, sites[1], planes[1], flight, 512) else {
            return fail("initial upper release not certified");
        };
        let Some((other, _)) = released(r, prior, sites[1], planes[1], flight, 256) else {
            return fail("coarse release not certified");
        };
        assert!(g.lo <= other.hi && other.lo <= g.hi);
        let absg = planes[1]
            .coordinates(initial.com + initial.q * vertices()[upper.corner], lower.at)
            .z;
        (
            g,
            json!({"gap_m":g.data(),"coarse_gap_m":other.data(),"absolute_pose_gap_m":absg,"fixed_normal_control":fixed_normals(r,initial,planes,sites)}),
        )
    };
    let mut result = Sequence {
        records: vec![],
        finite: vec![],
        reason: "32-event limit",
        third: None,
        first: seed,
        limit: Value::Null,
    };
    let mut x = initial;
    let mut offset = 0.;
    let mut correction = 0.;
    let mut fired = 0;
    let mut propagated = uncertainty::Error::default();
    for k in 0..32 {
        if let Some(goal) = goal
            && let Some(p) = limit::early(
                r,
                (planes, sites, mu),
                x,
                fired,
                gap,
                lower.at + offset + correction,
                (goal, propagated),
            )
        {
            result.limit = p;
            result.reason = "bounded early closure";
            return result;
        }
        let target = 1 - fired;
        let mut incoming = jet(r, x, sites[target], planes[target]);
        if quick {
            let rr = vertices()[sites[target].k].length();
            incoming.veps += propagated.velocity(r, x, rr);
            if incoming.v + incoming.veps >= 0. {
                result.reason = "approach outside state enclosure";
                break;
            }
            let horizon = 3. * gap.hi / (-incoming.v - incoming.veps);
            incoming.aeps += propagated.accel(r, x, rr, horizon);
        }
        let mut time = match crossing(gap, incoming) {
            Ok(v) => v,
            Err(e) => {
                result.reason = e;
                break;
            }
        };
        // A coarse global remainder can overlap the first crossing. Re-bracket
        // from the translated curvature integrals while retaining monotonicity.
        if quick && time.hi - time.lo > time.mid * 1e-6 {
            // An enclosure of the signed height at one proposed root and a
            // certified lower bound on closing speed imply an interval for
            // time by the mean value theorem. Keep the existing root bracket.
            let h = time.hi;
            let slope = -(incoming.v
                + incoming.veps
                + (incoming.a.abs() + incoming.aeps) * h
                + 0.5 * incoming.jerk * h * h);
            if slope <= 0. {
                result.reason = "uncertain local speed";
                break;
            }
            for _ in 0..3 {
                let t = time.mid;
                let Some((b, _)) = propagated.bend(r, x, sites[target], planes[target], t, 4)
                else {
                    break;
                };
                let lower = gap.lo + (incoming.v - incoming.veps) * t + b.lo;
                let upper = gap.hi + (incoming.v + incoming.veps) * t + b.hi;
                let lo = time.lo.max(t + lower.min(0.) / slope);
                let hi = time.hi.min(t + upper.max(0.) / slope);
                if lo > hi {
                    break;
                }
                let next = (t - (gap.mid + incoming.v * t + b.mid) / (incoming.v + incoming.a * t))
                    .clamp(lo, hi);
                time = Interval { lo, mid: next, hi };
                if (t - next).abs() < 1e-12 * t {
                    break;
                }
            }
        } else if !quick && time.hi - time.lo > time.mid * 1e-6 {
            let initial = time;
            let root = |upper: bool| -> Option<f64> {
                let (mut lo, mut hi) = (initial.lo, initial.hi);
                for _ in 0..38 {
                    let t = (lo + hi) * 0.5;
                    let (bend, _) = bend(r, x, sites[target], planes[target], t, 32, quick)?;
                    let residual = if upper {
                        gap.hi + (incoming.v + incoming.veps) * t + bend.hi
                    } else {
                        gap.lo + (incoming.v - incoming.veps) * t + bend.lo
                    };
                    if residual > 0. {
                        lo = t
                    } else {
                        hi = t;
                    }
                }
                Some(if upper { hi } else { lo })
            };
            if let (Some(lo), Some(hi)) = (root(false), root(true))
                && lo <= hi
            {
                time.lo = lo;
                time.hi = hi;
                time.mid = initial.mid.clamp(lo, hi);
            }
        }
        if time.hi - time.lo > time.mid * 0.05 {
            result.reason = "uncertain event ordering";
            break;
        }
        for _ in 0..3 {
            if let Some((bend, _)) = bend(r, x, sites[target], planes[target], time.mid, 16, quick)
            {
                let residual = gap.mid + incoming.v * time.mid + bend.mid;
                let slope = incoming.v + time.mid * (incoming.a + 0.5 * incoming.third * time.mid);
                let t = time.mid - residual / slope;
                if t >= time.lo && t <= time.hi {
                    time.mid = t;
                }
            }
        }
        let Some((other, margin)) = protect(r, x, planes, sites, lower.at + offset, time.hi) else {
            result.reason = "other finite feature";
            break;
        };
        // Below this duration a 16-slice quadratic displacement can enter
        // the subnormal range. Hand the still-normal initial gap to the
        // contraction analysis before the next quadratic evaluation.
        if time.lo < 1e-153 {
            result.reason = "quadratic displacement precision floor";
            break;
        }
        if quick && other.min(margin) <= propagated.covered(r, x, time.hi, planes) {
            result.reason = "finite neighbor outside state enclosure";
            break;
        }
        let released_range = |t| {
            let (g, a) = if quick {
                propagated.bend(r, x, sites[fired], planes[fired], t, 16)?
            } else {
                curvature(r, x, sites[fired], planes[fired], t, 16)?
            };
            (g.positive() && a > 0.).then_some((g, a))
        };
        let (Some((lo, alo)), Some((mid, amid)), Some((hi, ahi))) = (
            released_range(time.lo),
            released_range(time.mid),
            released_range(time.hi),
        ) else {
            result.reason = if jet(r, x, sites[fired], planes[fired]).a < 0. {
                "compressive contact at the preceding vertex"
            } else {
                "unbounded release"
            };
            break;
        };
        let outgoing_gap = Interval {
            lo: lo.lo,
            mid: mid.mid,
            hi: hi.hi,
        };
        if !outgoing_gap.positive() {
            result.reason = "released height resolution exhausted";
            break;
        }
        let pre = if quick {
            let (y, err) = propagated.flight(r, x, time);
            propagated = err;
            y
        } else {
            advance(r, x, time.mid)
        };
        let arm = pre.q * vertices()[sites[target].k];
        let law = match coulomb(r.body, pre, arm, planes[target], mu[target]) {
            Ok(l) => l,
            Err(e) => {
                result.reason = e;
                break;
            }
        };
        if quick {
            let Some(err) = propagated.impact(r, pre, arm, law, mu[target], target) else {
                result.reason = "unbounded point-response error";
                break;
            };
            propagated = err;
            if goal.is_some_and(|g| err.impulses.iter().any(|x| *x > g.vector_radius)) {
                result.reason = "finite impulse uncertainty exceeds goal";
                break;
            }
        }
        let next = offset + time.mid;
        correction += (offset - next) + time.mid;
        offset = next;
        let hit = Crossing {
            at: lower.at + (offset + correction),
            surface: target,
            corner: sites[target].k,
            before: pre,
            arm,
            plane: planes[target],
            mu: mu[target],
            law,
            other_gap: outgoing_gap.lo.min(other),
            finite_margin: margin,
        };
        let after = hit.after();
        if k == 0 {
            result.third = Some((hit, time));
        }
        let abs = planes[fired]
            .coordinates(pre.com + pre.q * vertices()[sites[fired].k], hit.at)
            .z;
        result.finite.push(hit);
        if quick {
            result.records.push(json!({"offset_s":offset,"offset_correction_s":correction,"time_s":time.data(),"released_gap_m":outgoing_gap.data()}));
            fired = target;
            gap = outgoing_gap;
            x = after;
            continue;
        }
        let physical = hit.data();
        let post = jet(r, after, sites[target], planes[target]);
        let other_post = jet(r, after, sites[fired], planes[fired]);
        result.records.push(json!({"ordinal":k+3,"incoming_gap_m":gap.data(),"relative_interval_s":time.data(),"offset_s":offset,"offset_correction_s":correction,"released_height_m":outgoing_gap.data(),"released_height_from_world_pose_m":abs,"released_accel_floor":alo.min(amid).min(ahi),"other_gap_m":other,"finite_margin_m":margin,"event":physical,"post_active_accel":post.a,"post_other_speed":other_post.v,"post_other_accel":other_post.a}));
        fired = target;
        gap = outgoing_gap;
        x = after;
    }
    if quick {
        return result;
    }
    result.limit = limit::classify(
        &i.r,
        (planes, sites, mu),
        x,
        fired,
        gap,
        lower.at + offset + correction,
    );
    result
}
fn open_after(
    i: &Impact,
    planes: [Face; 2],
    sites: [Site; 2],
    hit: Crossing,
    other_gap: Interval,
    h: f64,
) -> Option<Value> {
    let x = hit.after();
    let r = &i.r;
    let prior = 1 - hit.surface;
    let (_, margin) = protect(r, x, planes, sites, hit.at, h)?;
    let (height, active_a) = released(r, x, sites[hit.surface], planes[hit.surface], h, 64)?;
    let j = jet(r, x, sites[prior], planes[prior]);
    let (_, amin) = curvature(r, x, sites[prior], planes[prior], h, 64)?;
    let vmin = j.v - j.veps + amin.min(0.) * h;
    let clearance = other_gap.lo + vmin.min(0.) * h;
    if clearance <= 0. {
        return None;
    }
    Some(
        json!({"h_s":h,"other_height_floor_m":clearance,"other_speed_floor":vmin,"active_height_end_m":height.data(),"active_accel_floor":active_a,"finite_margin_m":margin}),
    )
}
pub(crate) fn write(path: &Path) -> crate::Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for i in inputs() {
        let mut cases = vec![];
        for delay in [20000, 750000, 1100000] {
            let g = gate(&i, delay as f64 * 1e-9);
            for gate_mu in [0., 0.35, 0.8] {
                let mu = [i.mu, i.mu.min(gate_mu)];
                let Ok(upper) = upper(&i, &g, mu[1]) else {
                    continue;
                };
                let prior = follow(&i, faces(&i.r, &g), mu, upper, WINDOW);
                let Some(lower) = prior.next.filter(|s| s.surface == 0) else {
                    continue;
                };
                let older = follow(&i, faces(&i.r, &g), mu, lower, WINDOW);
                let s = sequence(&i, &g, upper, lower, mu);
                let mut native = vec![];
                let mut post = vec![];
                if let Some((hit, when)) = s.third {
                    let sites = [Site::of(lower), Site::of(upper)];
                    let v = &s.records[0]["released_height_m"];
                    let gap = Interval {
                        lo: v[0].as_f64().unwrap(),
                        mid: v[1].as_f64().unwrap(),
                        hi: v[2].as_f64().unwrap(),
                    };
                    for h in [1e-9, 8e-9, 32e-9, 500e-9, 8e-6] {
                        if let Some(x) = open_after(&i, faces(&i.r, &g), sites, hit, gap, h) {
                            post.push(x);
                        }
                    }
                    let width = (when.lo * 1e9).floor().min(64.) as u64;
                    if width >= 1 {
                        let t = hit.at - width as f64 * 1e-9;
                        let x = advance(&i.r, lower.after(), t - lower.at);
                        native = native::third_controls(
                            &i,
                            &g,
                            &native::source(&i, &g, gate_mu),
                            t,
                            x,
                            width,
                        )?;
                    }
                    if i.r.id == 1368
                        && MODES[i.mode] == "stick"
                        && delay == 20000
                        && gate_mu == 0.35
                    {
                        display::write(path, &i, &g, gate_mu, lower, hit, gap)?;
                    }
                }
                cases.push(json!({"delay_ns":delay,"gate_mu":gate_mu,"gate_center0":g.center.to_array(),"gate_velocity":g.velocity.to_array(),"upper":upper.data(),"lower":lower.data(),"original_reason":older.reason,"original_third":older.next.map(Crossing::data),"original_through_ns":older.through*1e9,"native":native,"post_third_free":post,"seed":s.first,"limit":s.limit,"sequence":s.records,"stop":s.reason}));
            }
        }
        if !cases.is_empty() {
            rows.push(json!({"id":i.r.id,"mode":MODES[i.mode],"primary":law(&i),"tensor":i.r.body.tensor.to_cols_array(),"pad_velocity":i.r.pad_v.to_array(),"cases":cases}));
        }
    }
    write_json(&path.join("classified.json"), &json!({"donors":rows}))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn relative_height_preserves_a_release_that_world_positions_cannot_resolve() {
        let mut r = Physical::with_body(1, ReferenceBody::new(DVec3::splat(0.03), DQuat::IDENTITY));
        r.gravity = DVec3::Z * 8.;
        let x = Flight {
            com: DVec3::new(0., 0., 1.),
            v: DVec3::ZERO,
            q: DQuat::IDENTITY,
            l: DVec3::ZERO,
        };
        let f = Face {
            frame: DQuat::IDENTITY,
            origin: DVec3::ZERO,
            v: DVec3::ZERO,
            half: DVec2::ONE,
        };
        let (g, a) = released(&r, x, Site { side: 0, k: 0 }, f, 1e-12, 32).unwrap();
        let truth = 4e-24;
        assert!(a > 7.9 && g.lo < truth && truth < g.hi);
        assert_eq!(1. + truth, 1.);
        let j = Jet {
            v: -2.,
            a: 0.,
            jerk: 0.,
            third: 0.,
            snap: 0.,
            jeps: 1e-12,
            veps: 1e-13,
            aeps: 1e-12,
        };
        let t = crossing(g, j).unwrap();
        assert!(t.lo <= truth / 2. && truth / 2. <= t.hi);
    }
}
