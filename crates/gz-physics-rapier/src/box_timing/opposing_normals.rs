// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Close an isolated, frictionless two-site sequence after a measured lower impact.
use super::*;
use gz_physics::AlternatingNormalInput;
const RADIUS: f64 = 1e-4;
/// A request which fell through to the ordinary solver. The last completed
/// finite impulse and the terminal estimate use independent qualification tests.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OpposingNormalDecline {
    pub start: Duration,
    pub duration: Duration,
    pub finite_returns: u8,
    pub stage: OpposingNormalStage,
    pub limit: Option<OpposingLimitStage>,
}
impl OpposingNormalDecline {
    pub(crate) fn new(start: Duration, duration: Duration) -> Self {
        Self {
            start,
            duration,
            finite_returns: 0,
            stage: OpposingNormalStage::World,
            limit: None,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpposingNormalStage {
    World,
    Material,
    LowerBracket,
    LowerIsolation,
    PriorCeiling,
    IncomingEnclosure,
    InitialHeight,
    ApproachSpeed,
    RootHorizon,
    PositiveOutwardAcceleration,
    RootEnclosure,
    FiniteGeometry,
    FiniteMotion,
    FiniteImpulse,
    MaxReturns,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpposingLimitStage {
    InitialGap,
    TimeBudget,
    PositiveOutwardAcceleration,
    Contraction,
    EffectiveMass,
    ImpulseRadius,
    Geometry,
    FinalMotion,
}
/// Finite impulse followed by a certified two-site limiting response. Site
/// zero is the lower box and site one is the ceiling. Each reported scalar is
/// a positive impulse on the foot along the corresponding world normal.
#[derive(Clone, Debug, PartialEq)]
pub struct OpposingNormalReceipt {
    pub first_lower: OpposingCornerReceipt,
    pub supports: [BodyId; 2],
    pub corners: [u8; 2],
    pub normals: [[f64; 3]; 2],
    pub finite: Vec<OpposingNormalEvent>,
    pub normal_impulse: [f64; 2],
    pub impulse_radius: [f64; 2],
    pub terminal_normal: [f64; 2],
    /// World levers about the foot COM used for the bounded terminal normal sums.
    pub terminal_levers: [[f64; 3]; 2],
    /// Bounds on the accumulation time from the start of the public step.
    pub terminal_time_s: [f64; 2],
    pub end_time: Duration,
    pub separation_acceleration: [f64; 2],
    pub min_other_gap_m: f64,
    pub min_face_margin_m: f64,
    pub com_error_m: f64,
    pub spin_error_rad_s: f64,
    /// Signed work on the foot by each translating support in the inertial frame.
    pub support_work_j: [f64; 2],
    pub gravity_work_j: f64,
    pub kinetic_energy_change_j: f64,
    /// Mechanical energy dissipated by perfectly inelastic normal impacts.
    pub dissipated_energy_j: f64,
    /// Energy error from both impulse intervals, flight, momentum and mechanical roundoff.
    pub energy_error_j: f64,
    pub end: BodyState,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpposingNormalEvent {
    pub site: u8,
    pub time_s: f64,
    pub time_error_s: f64,
    pub impulse: f64,
    pub dissipated_energy_j: f64,
    pub world_point: [f64; 3],
    pub lever_from_com: [f64; 3],
}
#[derive(Clone, Copy, Default)]
struct Uncertainty {
    t: f64,
    p: f64,
    q: f64,
    v: f64,
    l: f64,
    j: [f64; 2],
}
#[derive(Clone, Copy)]
struct Contact {
    arm: Vector,
    n: Vector,
    v: Vector,
}
#[derive(Clone, Copy)]
struct Motion {
    rotor: Rotor,
    mass: f64,
    max_inv: f64,
    contrast: f64,
    contacts: [Contact; 2],
    rr: [f64; 2],
}
impl Motion {
    fn vel(self, x: Cut, k: usize) -> f64 {
        let p = self.contacts[k];
        (x.v - p.v + x.spin.cross(x.q * p.arm)).dot(p.n)
    }
    fn accel(self, x: Cut, k: usize) -> f64 {
        let p = self.contacts[k];
        let arm = x.q * p.arm;
        let alpha = self
            .rotor
            .inverse_at(x.q, self.rotor.angular_momentum(x).cross(x.spin));
        (self.rotor.acc + alpha.cross(arm) + x.spin.cross(x.spin.cross(arm))).dot(p.n)
    }
    fn matrix(self, x: Cut) -> [[f64; 2]; 2] {
        [0, 1].map(|i| {
            [0, 1].map(|j| {
                let a = self.contacts[i];
                let b = self.contacts[j];
                let z = (x.q * b.arm).cross(b.n);
                a.n.dot(b.n / self.mass + self.rotor.inverse_at(x.q, z).cross(x.q * a.arm))
            })
        })
    }
    fn rates(self, x: Cut, e: Uncertainty, extra: f64) -> (f64, f64, f64) {
        let w =
            (self.rotor.angular_momentum(x).length() + e.l + self.rr[0].max(self.rr[1]) * extra)
                * self.max_inv;
        let alpha = 0.5 * self.contrast * w * w;
        let jerk = (1. + self.contrast) * w * alpha;
        (w, alpha, jerk)
    }
    fn verr(self, x: Cut, e: Uncertainty, k: usize) -> f64 {
        let (w, _, _) = self.rates(x, e, 0.);
        e.v + self.rr[k] * (e.l * self.max_inv + 3. * w * e.q)
            + 256.
                * f64::EPSILON
                * (1. + x.v.length() + self.contacts[k].v.length() + self.rr[k] * w)
    }
    fn accelerations(self, x: Cut, e: Uncertainty, k: usize, t: f64) -> [f64; 2] {
        let (w, a, b) = self.rates(x, e, 0.);
        let theta = (2.5 * w * t).exp() * (e.q + e.l * self.max_inv * t);
        let delta = self.rr[k]
            * ((self.contrast + 2.) * w * (e.l * self.max_inv + w * theta)
                + (a + w * w) * theta
                + (b + 3. * w * a + w.powi(3)) * t)
            + 512. * f64::EPSILON * (1. + self.rotor.acc.length() + (a + w * w) * self.rr[k]);
        let center = self.accel(x, k);
        [center - delta, center + delta]
    }
    fn flight(self, x: Cut, e: &mut Uncertainty, t: f64, et: f64) -> Option<Cut> {
        let (w, a, b) = self.rates(x, *e, 0.);
        let l = self.rotor.angular_momentum(x);
        let steps = (w * t / 0.002).ceil().max(1.) as u32;
        if steps > 128 {
            return None;
        }
        let h = t / f64::from(steps);
        let mut y = x;
        let eps = 64. * f64::EPSILON;
        for _ in 0..steps {
            let next = self.rotor.at(y, h).ok()?;
            let pure = |v: Vector| Rotation::from_xyzw(v.x, v.y, v.z, 0.);
            let dq = (pure(y.spin) * y.q) * 0.5;
            let alpha = self.rotor.inverse_at(y.q, l.cross(y.spin));
            let ddq = (pure(alpha) * y.q + pure(y.spin) * dq) * 0.5;
            let taylor = y.q + dq * h + ddq * (0.5 * h * h);
            let q3 = 0.5 * b + 0.75 * w * a + 0.125 * w.powi(3);
            e.q = (2.5 * w * h).exp() * (e.q + h * e.l * self.max_inv)
                + 3. * ((if next.q.dot(taylor) < 0. {
                    -next.q
                } else {
                    next.q
                } - taylor)
                    .length()
                    + q3 * h.powi(3) / 6.
                    + eps);
            e.p += h * e.v + eps * (1. + y.com.length() + h * y.v.length());
            e.v += eps * (1. + y.v.length() + self.rotor.acc.length() * h);
            e.l += self
                .rotor
                .angular_momentum(next)
                .distance(self.rotor.angular_momentum(y))
                + eps * (1. + l.length());
            y = next;
        }
        self.widen_time(y, e, et);
        Some(y)
    }
    fn widen_time(self, y: Cut, e: &mut Uncertainty, et: f64) {
        let (w, _, _) = self.rates(y, *e, 0.);
        e.t += et;
        e.q += w * et;
        e.p += (y.v.length() + e.v + self.rotor.acc.length() * et) * et;
        e.v += self.rotor.acc.length() * et;
    }
    fn impulse(self, mut x: Cut, e: &mut Uncertainty, k: usize) -> Option<(Cut, f64)> {
        let n = self.contacts[k].n;
        let rr = self.rr[k];
        let kn = self.matrix(x)[k][k];
        let u = self.vel(x, k);
        let j = -u / kn;
        let dk = 4. * rr * rr * e.q * self.max_inv
            + 128. * f64::EPSILON * (1. / self.mass + rr * rr * self.max_inv);
        let dv = self.verr(x, *e, k);
        if u + dv >= 0. || kn <= dk || !j.is_finite() {
            return None;
        }
        let error = (dv + dk * j) / (kn - dk) + 64. * f64::EPSILON * j;
        e.j[k] += error;
        e.v += error / self.mass;
        e.l += rr * (error + e.q * (j + error));
        let angular = self.rotor.angular_momentum(x) + (x.q * self.contacts[k].arm).cross(n * j);
        x.v += n * (j / self.mass);
        x.spin = self.rotor.inverse_at(x.q, angular);
        e.l += self.rotor.angular_momentum(x).distance(angular)
            + 64. * f64::EPSILON * (1. + angular.length());
        e.v += 64. * f64::EPSILON * (1. + x.v.length() + j / self.mass);
        Some((x, j))
    }
}
struct Series<'a> {
    m: Motion,
    initial: &'a Incoming,
    sites: [usize; 2],
    x: Cut,
    e: Uncertainty,
    gap: [f64; 3],
    fired: usize,
    t: f64,
    sums: [f64; 2],
    events: Vec<OpposingNormalEvent>,
    min_gap: f64,
    min_margin: f64,
}
impl Series<'_> {
    fn protect(&self, span: f64, extra: f64) -> Option<(f64, f64)> {
        let e = self.e;
        let (w, _, _) = self.m.rates(self.x, e, extra);
        let mut gaps = f64::INFINITY;
        let mut margins = f64::INFINITY;
        for side in 0..2 {
            let d = self.initial.decks[side].at(self.t);
            for (k, &arm) in self.initial.arms.iter().enumerate() {
                let pos = self.x.com + self.x.q * arm;
                let local = d.pose.inverse_transform_point(pos);
                let travel = e.p
                    + arm.length() * e.q
                    + d.v.length() * e.t
                    + ((self.x.v - d.v).length()
                        + e.v
                        + extra / self.m.mass
                        + w * arm.length()
                        + self.m.rotor.acc.length() * span)
                        * span
                    + self.initial.tol;
                margins =
                    margins.min((d.half.x - local.x.abs()).min(d.half.y - local.y.abs()) - travel);
                if k != self.sites[side] {
                    gaps = gaps.min((pos - d.top).dot(d.n) - travel);
                }
            }
        }
        (gaps > 0. && margins > 0.).then_some((gaps, margins))
    }
    fn finish(&self, remaining: f64, reason: &mut Option<OpposingLimitStage>) -> Option<Limit> {
        use OpposingLimitStage::*;
        *reason = Some(InitialGap);
        let m = self.m;
        let x = self.x;
        let e = self.e;
        let next = 1 - self.fired;
        let c = -m.vel(x, next);
        let drift = m.verr(x, e, next);
        if c <= drift || self.gap[0] <= 0. {
            return None;
        }
        let h = 2. * self.gap[2] / (c - drift);
        *reason = Some(TimeBudget);
        if 2. * h + e.t >= remaining || !h.is_finite() {
            return None;
        }
        let base = m.matrix(x);
        let largest = m.rr[0].max(m.rr[1]);
        let g = m.rotor.acc.length();
        for power in 0..36 {
            let budget = m.mass * c * 0.125 * 1.5f64.powi(power);
            let (w, alpha, _) = m.rates(x, e, budget);
            let theta = e.q + w * (remaining + e.t) + 256. * f64::EPSILON;
            let dw = (budget * largest + e.l) * m.max_inv + w * theta;
            let high = g + largest * (alpha + w * w);
            let acc = [0, 1].map(|k| {
                m.accel(x, k)
                    - m.rr[k] * ((2. + m.contrast) * w * dw + (alpha + w * w) * theta)
                    - 256. * f64::EPSILON * (1. + high)
            });
            *reason = Some(PositiveOutwardAcceleration);
            if acc[0] <= 0. || acc[1] <= 0. {
                return None;
            }
            // Impulse-weighted compliance changes only while the series is
            // active. Outward acceleration above still spans the entire step.
            let contact_theta = e.q + w * (2. * h + e.t) + 256. * f64::EPSILON;
            let coeff = [0, 1].map(|i| {
                [0, 1].map(|j| {
                    let dr = 4. * m.rr[i] * m.rr[j] * m.max_inv * contact_theta
                        + 256. * f64::EPSILON * (1. + base[i][j].abs());
                    [base[i][j] - dr, base[i][j] + dr]
                })
            });
            let input = AlternatingNormalInput {
                next,
                clearance_upper: self.gap[2],
                closing_speed: [c - drift, c + drift],
                response: coeff,
                outward_min: acc,
                acceleration_max: high,
                world_per_normal: [1.; 2],
                impulse_budget: budget,
                time_budget: remaining - e.t,
                normal_roundoff: 512. * f64::EPSILON * (1. + x.v.length() + w * largest),
            };
            *reason = Some(Contraction);
            let Some(p) = input.bound() else {
                continue;
            };
            let (det, b) = (
                base[0][0] * base[1][1] - base[0][1] * base[1][0],
                base[0][1],
            );
            *reason = Some(EffectiveMass);
            if det <= 0. {
                return None;
            }
            let rhs = [0, 1].map(|k| -m.vel(x, k));
            let j = [
                (base[1][1] * rhs[0] - b * rhs[1]) / det,
                (base[0][0] * rhs[1] - base[1][0] * rhs[0]) / det,
            ];
            let r = [0, 1].map(|k| {
                e.j[k]
                    + (p.normal_impulse_bounds[k][0] - j[k])
                        .abs()
                        .max((p.normal_impulse_bounds[k][1] - j[k]).abs())
            });
            *reason = Some(ImpulseRadius);
            if j.iter().any(|v| *v < 0. || !v.is_finite())
                || r.iter().any(|v| *v > RADIUS || !v.is_finite())
            {
                return None;
            }
            *reason = Some(Geometry);
            let (gaps, margins) = self.protect(remaining + e.t, budget)?;
            let mut end = x;
            let mut l = m.rotor.angular_momentum(x);
            let mut ee = e;
            let lag = p.time_cap;
            for k in 0..2 {
                let v = m.contacts[k].n * j[k];
                end.v += v / m.mass;
                l += (x.q * m.contacts[k].arm).cross(v);
                ee.l += m.rr[k] * (r[k] + (e.q + w * lag) * j[k]);
                ee.v += r[k] / m.mass;
            }
            ee.p += 2. * (x.v.length() + e.v + budget / m.mass + g * remaining) * lag;
            ee.q += 2. * w * lag;
            end.spin = m.rotor.inverse_at(x.q, l);
            *reason = Some(FinalMotion);
            end = m.flight(end, &mut ee, remaining, e.t)?;
            return Some(Limit {
                end,
                normal: j,
                radius: r,
                span: lag,
                acc,
                min_gap: self.min_gap.min(gaps),
                min_margin: self.min_margin.min(margins),
                com_error: ee.p,
                spin_error: ee.l * m.max_inv + 2. * w * ee.q,
                v_error: ee.v,
                momentum_error: ee.l,
            });
        }
        None
    }
    fn next(
        &mut self,
        remaining: f64,
        elapsed: f64,
        stage: &mut OpposingNormalStage,
    ) -> Option<()> {
        use OpposingNormalStage::*;
        *stage = ApproachSpeed;
        let m = self.m;
        let target = 1 - self.fired;
        let x = self.x;
        let mut e = self.e;
        let v = m.vel(x, target);
        let dv = m.verr(x, e, target);
        if v + dv >= 0. {
            return None;
        }
        // The quadratic bounds below must actually bracket a root. Never demand
        // additional simulated time beyond that bracket just to test it.
        let h = (3. * self.gap[2] / (-v - dv)).min(remaining - e.t);
        *stage = RootHorizon;
        if !h.is_finite() || h <= 1e-150 {
            return None;
        }
        let a = m.accelerations(x, e, target, h);
        let released = m.accelerations(x, e, self.fired, h);
        *stage = PositiveOutwardAcceleration;
        if released[0] <= 0. || v + dv + a[1].max(0.) * h >= 0. {
            return None;
        }
        let root = |gap: f64, speed: f64, acc: f64| -> Option<f64> {
            let disc = speed * speed - 2. * acc * gap;
            if disc <= 0. {
                return None;
            }
            let t = 2. * gap / (-speed + disc.sqrt());
            (t > 0. && t <= h).then_some(t)
        };
        *stage = RootEnclosure;
        let lo = root(self.gap[0], v - dv, a[0])? * (1. - 128. * f64::EPSILON);
        let hi = root(self.gap[2], v + dv, a[1])? * (1. + 128. * f64::EPSILON);
        if lo > hi || !lo.is_finite() || !hi.is_finite() {
            return None;
        }
        let mid = root(self.gap[1], v, m.accel(x, target))?.clamp(lo, hi);
        // The complete root interval propagates into pose, momentum and the
        // next signed gap. The final impulse enclosure limits its usefulness.
        *stage = FiniteGeometry;
        let (gaps, margin) = self.protect(hi, 0.)?;
        self.min_gap = self.min_gap.min(gaps);
        self.min_margin = self.min_margin.min(margin);
        *stage = FiniteMotion;
        let pre = m.flight(x, &mut e, mid, (hi - mid).max(mid - lo))?;
        *stage = FiniteImpulse;
        let (after, j) = m.impulse(pre, &mut e, target)?;
        if e.j[target] > RADIUS {
            return None;
        }
        self.gap = [
            0.5 * lo * (released[0] * lo),
            0.5 * mid * (m.accel(x, self.fired) * mid),
            0.5 * hi * (released[1] * hi),
        ];
        self.t += mid;
        self.fired = target;
        self.x = after;
        self.e = e;
        self.sums[target] += j;
        self.events.push(OpposingNormalEvent {
            site: target as u8,
            time_s: elapsed + self.t,
            time_error_s: e.t,
            impulse: j,
            dissipated_energy_j: -0.5 * j * m.vel(pre, target),
            world_point: (pre.com + pre.q * m.contacts[target].arm).to_array(),
            lever_from_com: (pre.q * m.contacts[target].arm).to_array(),
        });
        Some(())
    }
}
struct Limit {
    end: Cut,
    normal: [f64; 2],
    radius: [f64; 2],
    span: f64,
    acc: [f64; 2],
    min_gap: f64,
    min_margin: f64,
    com_error: f64,
    spin_error: f64,
    v_error: f64,
    momentum_error: f64,
}
impl UpperPair {
    pub(crate) fn normal_series(
        &self,
        e: &RapierBackend,
        elapsed: Duration,
        dt: Duration,
        decline: &mut OpposingNormalDecline,
    ) -> Option<UpperStep> {
        use OpposingNormalStage::*;
        if dt.is_zero()
            || dt > Duration::from_micros(250)
            || e.world.colliders.len() != 3
            || e.world.bodies.len() != 3
        {
            return None;
        }
        let i = self.incoming(e)?;
        decline.stage = Material;
        if i.friction != 0. {
            return None;
        }
        let lower = self.for_lower(e, i, i.state, 0.)?;
        if lower.friction != 0. {
            return None;
        }
        decline.stage = LowerBracket;
        let first = self.first_upper_hint(lower, dt.as_secs_f64())?;
        let horizon = Duration::from_nanos((first * 1e9).ceil() as u64 + 1);
        if horizon >= dt {
            return None;
        }
        let roof = i.decks[1];
        let top = (4..8).min_by(|&a, &b| {
            (i.state.q * i.arms[a])
                .dot(roof.n)
                .total_cmp(&(i.state.q * i.arms[b]).dot(roof.n))
        })?;
        // Establish the true order from the live state. A previously touching
        // roof is ineligible here. The named high vertex must stay separated
        // up to the first actual lower impact even with fractional-time error.
        decline.stage = LowerIsolation;
        let step = self.branch_to_measured_impact(
            e,
            lower,
            Branch {
                elapsed,
                dt: horizon,
                separated: false,
                lower_after: Some((top - 4) as u8),
            },
        )?;
        let mut first_lower = step.receipt?;
        let out = first_lower.outgoing;
        let local = e.world.bodies[self.c.foot]
            .mass_properties()
            .local_mprops
            .local_com;
        let live = |b: BodyState| {
            let q = Rotation::from_array(b.orientation);
            Cut {
                com: Vector::from_array(b.position) + q * local,
                q,
                spin: b.angular_velocity.into(),
                v: b.velocity.into(),
            }
        };
        let x = live(out);
        let sites = [usize::from(first_lower.point.corner), top];
        let contacts = [0, 1].map(|k| Contact {
            arm: i.arms[sites[k]],
            n: i.decks[k].n,
            v: i.decks[k].v,
        });
        let mass = e.world.bodies[self.c.foot].mass();
        let max_inv = i.rotor.inv.max_element();
        let m = Motion {
            rotor: i.rotor,
            mass,
            max_inv,
            contrast: max_inv / i.rotor.inv.min_element() - 1.,
            contacts,
            rr: contacts.map(|p| p.arm.length()),
        };
        let root = first_lower.point.after_cut_s;
        let total = dt.as_secs_f64();

        let initial = i.state;
        let round = 128. * f64::EPSILON;
        let mut err = Uncertainty {
            p: round * (1. + initial.com.length()),
            q: round,
            v: round * (1. + initial.v.length()),
            l: round * (1. + m.rotor.angular_momentum(initial).length()),
            ..Default::default()
        };
        let fold = |a: Cut, b: Cut, e: &mut Uncertainty| {
            e.p += a.com.distance(b.com);
            e.v += a.v.distance(b.v);
            e.q += 3. * (a.q - b.q).length().min((a.q + b.q).length());
            e.l += m
                .rotor
                .angular_momentum(a)
                .distance(m.rotor.angular_momentum(b));
        };
        decline.stage = IncomingEnclosure;
        let before = m.flight(initial, &mut err, root, 0.)?;
        let measured = live(first_lower.incoming);
        fold(before, measured, &mut err);
        // Bound the *physical* root error, not only the residual of the
        // numerical free-flight state used by the native point solver.
        let d = i.decks[0];
        let full = horizon.as_secs_f64();
        let r = initial.q * contacts[0].arm;
        let start_gap = (initial.com + r - d.top).dot(d.n);
        let speed = self
            .jet(initial, full)
            .height(
                r,
                d.n,
                initial.spin,
                (initial.v - d.v, m.rotor.acc),
                start_gap,
                initial.com.length() + d.top.length(),
            )
            .closing_speed(full)?;
        let timing = first_lower
            .timing_error_s
            .max((first_lower.point.impact.gap.abs() + i.tol + err.p + err.q * m.rr[0]) / speed);
        if !timing.is_finite() || timing > 0.1e-9 || root + timing > full {
            return None;
        }
        first_lower.timing_error_s = timing;
        decline.stage = PriorCeiling;
        let (gap, margin) =
            self.isolated_fractional_prefix(&lower, initial, root + timing, 4 + sites[0])?;
        first_lower.minimum_other_gap_m = gap;
        first_lower.minimum_footprint_margin_m = margin;
        m.widen_time(measured, &mut err, timing);
        decline.stage = IncomingEnclosure;
        let (after, j) = m.impulse(measured, &mut err, 0)?;
        let j0 = Vector::from_array(first_lower.point.impact.impulse_on_foot).dot(i.decks[0].n);
        err.j[0] += (j0 - j).abs();
        fold(after, x, &mut err);
        decline.stage = InitialHeight;
        let upper_gap = (x.com + x.q * contacts[1].arm - roof.at(root).top).dot(roof.n);
        let dz = i.tol + err.p + err.q * m.rr[1] + roof.v.length() * timing;
        if upper_gap <= dz || err.j[0] > RADIUS {
            return None;
        }
        let mut s = Series {
            m,
            initial: &i,
            sites,
            x,
            e: err,
            gap: [upper_gap - dz, upper_gap, upper_gap + dz],
            fired: 0,
            t: root,
            sums: [j0, 0.],
            events: vec![],
            min_gap: first_lower.minimum_other_gap_m,
            min_margin: first_lower.minimum_footprint_margin_m,
        };
        for count in 0..=8 {
            decline.finite_returns = count;
            let remaining = total - s.t;
            if let Some(p) = s.finish(remaining, &mut decline.limit) {
                let start = elapsed.as_secs_f64() + s.t;
                let totals = [s.sums[0] + p.normal[0], s.sums[1] + p.normal[1]];
                let support_work = [0, 1].map(|k| totals[k] * contacts[k].n.dot(contacts[k].v));
                let gravity_work = m.mass * m.rotor.acc.dot(p.end.com - initial.com);
                let kinetic = |b: Cut| {
                    0.5 * (m.mass * b.v.length_squared() + b.spin.dot(m.rotor.angular_momentum(b)))
                };
                let change = kinetic(p.end) - kinetic(initial);
                let dissipated = support_work.iter().sum::<f64>() + gravity_work - change;
                // Dot-product perturbations enclose K=mv²/2+ω·L/2. The same
                // independently bounded endpoint accounts for gravity work.
                let momentum = m.rotor.angular_momentum(p.end);
                let energy_error = contacts
                    .iter()
                    .zip(p.radius)
                    .map(|(c, r)| c.v.dot(c.n).abs() * r)
                    .sum::<f64>()
                    + m.mass * m.rotor.acc.length() * p.com_error
                    + m.mass * (p.end.v.length() + 0.5 * p.v_error) * p.v_error
                    + 0.5
                        * (p.end.spin.length() * p.momentum_error
                            + (momentum.length() + p.momentum_error) * p.spin_error)
                    + 512.
                        * f64::EPSILON
                        * (1.
                            + kinetic(initial)
                            + kinetic(p.end)
                            + gravity_work.abs()
                            + support_work[0].abs()
                            + support_work[1].abs());
                if !energy_error.is_finite()
                    || !dissipated.is_finite()
                    || dissipated + energy_error < 0.
                {
                    return None;
                }
                let r = OpposingNormalReceipt {
                    first_lower,
                    supports: [self.ids.1, self.ids.2],
                    corners: [sites[0] as u8, (top - 4) as u8],
                    normals: contacts.map(|c| c.n.to_array()),
                    finite: s.events,
                    normal_impulse: totals,
                    impulse_radius: p.radius,
                    terminal_normal: p.normal,
                    terminal_levers: contacts.map(|c| (s.x.q * c.arm).to_array()),
                    terminal_time_s: [start - s.e.t, start + s.e.t + p.span],
                    end_time: elapsed + dt,
                    separation_acceleration: p.acc,
                    min_other_gap_m: p.min_gap,
                    min_face_margin_m: p.min_margin,
                    com_error_m: p.com_error,
                    spin_error_rad_s: p.spin_error,
                    support_work_j: support_work,
                    gravity_work_j: gravity_work,
                    kinetic_energy_change_j: change,
                    dissipated_energy_j: dissipated,
                    energy_error_j: energy_error,
                    end: self.state_at(e, p.end),
                };
                return Some(UpperStep {
                    pair: step.pair,
                    ceiling: step.ceiling,
                    receipt: None,
                    returned: None,
                    normals: Some(Box::new(r)),
                    body: step.body,
                    end: p.end,
                });
            }
            if count == 8 {
                decline.stage = MaxReturns;
                return None;
            }
            s.next(remaining, elapsed.as_secs_f64(), &mut decline.stage)?;
        }
        None
    }
}
