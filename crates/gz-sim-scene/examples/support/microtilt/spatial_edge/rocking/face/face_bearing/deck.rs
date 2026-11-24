// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cartesian Newton–Euler reference for a fixed pad edge against a moving sole.
#[path = "deck/live.rs"]
mod live;
use super::*;
use gz_physics::{BoundDeckEdge, DeckEdgeSample};
pub(super) use live::compare as registered;
#[derive(Clone, Copy)]
struct Deck {
    shape: Shape,
    origin: DVec3,
    line: DVec3,
    half: f64,
}
#[derive(Clone, Copy)]
struct Deriv {
    c: DVec3,
    v: DVec3,
    q: DQuat,
    w: DVec3,
    force: f64,
    j: DVec3,
    torque: DVec3,
}
impl Deck {
    fn from(f: Fixture, bit: usize) -> Self {
        let out = [-DVec3::X, DVec3::X, -DVec3::Y, DVec3::Y][bit];
        Self {
            shape: f.shape,
            origin: out * f.shape.pad_half[bit / 2],
            line: DVec3::Z.cross(out),
            half: f.shape.pad_half[1 - bit / 2],
        }
    }
    fn derivative(self, s: State) -> Deriv {
        let q = s.motion.q;
        let w = s.motion.w;
        let n = q * DVec3::Z;
        let v = s.v;
        let iw = DMat3::from_quat(q) * self.shape.inertia * DMat3::from_quat(q).transpose();
        let inv = iw.inverse();
        let gyro = inv * (-w.cross(iw * w));
        // Two fixed deck sites define the active line. Their material counterparts
        // slide on the sole; differentiating its plane constraint introduces both
        // the normal's time derivative and the motion of the COM-relative lever.
        let r = [-self.half, self.half].map(|l| self.origin + self.line * l - s.c);
        let torque = r.map(|x| x.cross(n));
        let speed = r.map(|x| {
            n.dot(self.shape.gravity + gyro.cross(x) - w.cross(v)) + w.cross(n).dot(v + w.cross(x))
        });
        let wmat = torque.map(|u| torque.map(|v| 1. / self.shape.mass + u.dot(inv * v)));
        let d = wmat[0][0] * wmat[1][1] - wmat[0][1] * wmat[1][0];
        let j = [
            (wmat[0][1] * speed[1] - wmat[1][1] * speed[0]) / d,
            (wmat[1][0] * speed[0] - wmat[0][0] * speed[1]) / d,
        ];
        let total = j[0] + j[1];
        let f = total * n;
        let t = torque[0] * j[0] + torque[1] * j[1];
        Deriv {
            c: v,
            v: self.shape.gravity + f / self.shape.mass,
            q: DQuat::from_xyzw(w.x, w.y, w.z, 0.) * q * 0.5,
            w: gyro + inv * t,
            force: total,
            j: f,
            torque: t,
        }
    }
    fn at(s: State, d: Deriv, dt: f64) -> State {
        State {
            c: s.c + dt * d.c,
            v: s.v + dt * d.v,
            motion: Motion {
                q: (s.motion.q + d.q * dt).normalize(),
                w: s.motion.w + dt * d.w,
            },
        }
    }
    fn advance(self, s: State, h: f64) -> (State, f64, DVec3, DVec3) {
        let a = self.derivative(s);
        let b = self.derivative(Self::at(s, a, 0.5 * h));
        let c = self.derivative(Self::at(s, b, 0.5 * h));
        let d = self.derivative(Self::at(s, c, h));
        let w = h / 6.;
        let sum = |a, b, c, d| w * (a + 2. * b + 2. * c + d);
        let state = State {
            c: s.c + sum(a.c, b.c, c.c, d.c),
            v: s.v + sum(a.v, b.v, c.v, d.v),
            motion: Motion {
                q: (s.motion.q + (a.q + b.q * 2. + c.q * 2. + d.q) * w).normalize(),
                w: s.motion.w + sum(a.w, b.w, c.w, d.w),
            },
        };
        (
            state,
            w * (a.force + 2. * b.force + 2. * c.force + d.force),
            sum(a.j, b.j, c.j, d.j),
            sum(a.torque, b.torque, c.torque, d.torque),
        )
    }
}
#[derive(Clone, Copy)]
pub(super) struct Case {
    pub(super) f: Fixture,
    pub(super) edge: usize,
    pub(super) clock: f64,
}
impl Case {
    fn start(self) -> State {
        self.f.state(self.clock)
    }
    fn bound(self) -> Result<BoundDeckEdge> {
        self.f
            .model()
            .bind_deck_edge(Fixture::coordinates(self.start()), self.edge)?
            .ok_or_else(|| "missing positive co-linear deck edge at face loss".into())
    }
    fn status(self, x: DeckEdgeSample) -> &'static str {
        if x.normal_force <= 0. {
            "tension"
        } else if x.signed_tilt < -1e-11 {
            "return to face"
        } else if x.signed_tilt >= std::f64::consts::FRAC_PI_2 {
            "vertical"
        } else if x.center_along_edge <= x.segment[0] || x.center_along_edge >= x.segment[1] {
            "line endpoint"
        } else {
            "bearing"
        }
    }
    pub(super) fn compare(self) -> Result<Value> {
        let arc = self.bound()?;
        let mut trajectory = arc.trajectory();
        let spatial = Deck::from(self.f, self.edge);
        let mut state = self.start();
        let mut normal = 0.;
        let mut vector = DVec3::ZERO;
        let mut angular = DVec3::ZERO;
        let mut errs = [0f64; 7];
        let mut end = "horizon";
        let mut end_t = 0.;
        let mut max_tilt = 0f64;
        let mut lowest = f64::INFINITY;
        let mut last = None;
        let dt = 0.000125;
        for k in 1..=1200 {
            let prev = trajectory;
            let p = trajectory.advance(dt, dt)?;
            let status = self.status(p);
            if status != "bearing" {
                let (mut l, mut h) = (0., dt);
                for _ in 0..42 {
                    let mid = (l + h) * 0.5;
                    let mut bracket = prev;
                    let y = bracket.advance(mid, dt)?;
                    if self.status(y) == "bearing" {
                        l = mid;
                    } else {
                        h = mid;
                    }
                }
                end_t = (k - 1) as f64 * dt + 0.5 * (l + h);
                end = status;
                break;
            }
            let (v, j, dj, da) = spatial.advance(state, dt);
            state = v;
            normal += j;
            vector += dj;
            angular += da;
            let pair = spatial.derivative(v);
            let center = v.c;
            let n = v.motion.q * DVec3::Z;
            let off = (spatial.origin - center).cross(pair.j);
            let s = (pair.torque - off).dot(spatial.line.cross(n)) / pair.force;
            let cp = spatial.origin + spatial.line * s;
            errs[0] = errs[0].max(center.distance(p.state.center));
            errs[1] = errs[1].max(v.v.distance(p.state.velocity));
            errs[2] = errs[2].max(v.motion.w.distance(p.state.angular_velocity));
            errs[3] = errs[3].max((normal - p.normal_impulse).abs());
            errs[4] = errs[4].max(vector.distance(p.integrated_force));
            errs[5] = errs[5].max(angular.distance(p.angular_impulse_about_com));
            errs[6] = errs[6].max(cp.distance(p.center_of_pressure));
            lowest = lowest.min(p.equivalent_endpoint_force[0].min(p.equivalent_endpoint_force[1]));
            max_tilt = max_tilt.max(p.signed_tilt);
            end_t = k as f64 * dt;
            last = Some(p);
        }
        if errs.iter().any(|v| !v.is_finite() || *v > 1e-8) {
            return Err(format!("independent tipping edge mismatched {errs:?}").into());
        }
        let last = last.ok_or("immediate deck-edge loss")?;
        Ok(
            json!({"support_edge":self.edge,"event_s":self.clock,"duration_s":end_t,"end":end,"max_tilt_rad":max_tilt,"min_equivalent_force_n":lowest,"normal_ns":normal,"horizontal_impulse_ns":last.integrated_force.truncate().length(),"errors_com_velocity_spin_area_vector_angular_cp":errs}),
        )
    }
}
pub(super) fn case(f: Fixture, pressure: &Value) -> Option<Case> {
    let bits = pressure["pad_edges"].as_u64()?;
    if bits.count_ones() != 1 {
        return None;
    }
    Some(Case {
        f,
        edge: bits.trailing_zeros() as usize,
        clock: pressure["exit_s"].as_f64()?,
    })
}
pub(super) fn replay(out: &Path, i: Input, c: Case, study: &Value) -> Result<()> {
    let x = c.start();
    let f = Fixture { start: x, ..c.f };
    let end = study["duration_s"].as_f64().unwrap();
    let ticks = (end / DT).floor().min(240.) as u64;
    let native = super::actual(out, i, f, ticks)?;
    let mut cursor = c.bound()?.trajectory();
    let mut frames = vec![];
    let mut prev = 0.;
    for tick in 0..=ticks {
        let p = cursor.advance(if tick == 0 { 0. } else { DT }, 0.000125)?;
        let reaction = p.normal_impulse - prev;
        prev = p.normal_impulse;
        let x = p.state;
        let state = State {
            c: x.center,
            v: x.velocity,
            motion: Motion {
                q: x.orientation,
                w: x.angular_velocity,
            },
        };
        frames.push(row(
            tick,
            0,
            reaction,
            super::super::super::replay::bodies(i, tick as f64 * DT, state),
        ));
    }
    write_trace(out, "deck", frames)?;
    fs::write(
        out.join("public.json"),
        serde_json::to_vec(&json!({"native":native,"edge":study,"ticks":ticks}))?,
    )?;
    let lanes=[("ordinary","One native solve"),("refined","Eight native solves"),("deck","Independent sliding deck-edge constraint")].map(|(l,n)|json!({"label":l,"name":n,"file":format!("traces/{l}.json"),"analytical":l=="deck"}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"The articulated sole tips over the pad edge as the pressure leaves its face","pair_totals_only":true,"capture_ticks":{"tip_over_edge":ticks},"lanes":lanes}),
        )?,
    )?;
    Ok(())
}
