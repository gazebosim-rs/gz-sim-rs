// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Frictionless rigid-body dynamics and event detection on a finite flat deck.
use glam::{DMat2, DMat3, DQuat, DVec2, DVec3};
type Result<T> = std::result::Result<T, String>;

#[derive(Clone, Copy)]
pub(super) struct Shape {
    pub mass: f64,
    pub inertia: DMat3,
    /// Eight moving sole vertices followed by eight vertices of the upper link.
    pub vertices: [DVec3; 16],
    pub gravity: DVec3,
    pub pad_half: [f64; 2],
}
#[derive(Clone, Copy, Debug)]
pub(super) struct Motion {
    pub q: DQuat,
    pub w: DVec3,
}
#[derive(Clone, Copy, Debug)]
pub(super) struct State {
    pub motion: Motion,
    pub c: DVec3,
    pub v: DVec3,
}
#[derive(Clone, Copy, Debug)]
pub(super) enum Sites {
    One(usize),
    Two([usize; 2]),
}
impl Sites {
    pub fn first(self) -> usize {
        match self {
            Self::One(a) => a,
            Self::Two([a, _]) => a,
        }
    }
    fn has(self, i: usize) -> bool {
        match self {
            Self::One(a) => i == a,
            Self::Two(ab) => ab.contains(&i),
        }
    }
}
#[derive(Clone, Copy)]
pub(super) struct Arc {
    pub shape: Shape,
    pub start: State,
    pub sites: Sites,
}
#[derive(Clone, Copy)]
struct Rate {
    q: DQuat,
    w: DVec3,
    loads: [f64; 2],
}
#[derive(Clone, Copy, Debug)]
pub(super) enum End {
    Contact(usize),
    Loss,
    Boundary,
    Horizon,
    Ambiguous,
}
#[derive(Clone, Copy)]
pub(super) struct Arrival {
    pub time: f64,
    pub state: State,
    pub end: End,
    pub min_load: f64,
    pub max_energy: f64,
    pub max_constraint: f64,
}
#[derive(Clone, Copy)]
pub(super) struct Jump {
    pub normal: [f64; 2],
    pub old: usize,
    pub new: usize,
    pub after: State,
    pub loss: f64,
    pub new_sites: Sites,
}
impl Shape {
    fn inv(self, q: DQuat, t: DVec3) -> DVec3 {
        q * (self.inertia.inverse() * (q.conjugate() * t))
    }
    pub fn energy(self, s: State) -> f64 {
        let wb = s.motion.q.conjugate() * s.motion.w;
        0.5 * (self.mass * s.v.length_squared() + wb.dot(self.inertia * wb))
            - self.mass * self.gravity.dot(s.c)
    }
    pub fn point(self, k: usize, s: State) -> (DVec3, DVec3) {
        let r = s.motion.q * self.vertices[k];
        (s.c + r, s.v + s.motion.w.cross(r))
    }
    fn normal_mobility(self, q: DQuat, a: DVec3, b: DVec3) -> f64 {
        1. / self.mass + a.cross(DVec3::Z).dot(self.inv(q, b.cross(DVec3::Z)))
    }
    /// Plastic frictionless two-normal LCP at a new vertex and an existing bearing point.
    pub fn recollide(self, s: State, old: usize, new: usize) -> Result<Jump> {
        let q = s.motion.q;
        let ro = q * self.vertices[old];
        let rn = q * self.vertices[new];
        let wo = self.normal_mobility(q, ro, ro);
        let wn = self.normal_mobility(q, rn, rn);
        let cross = self.normal_mobility(q, ro, rn);
        let before = DVec2::new(self.point(old, s).1.z, self.point(new, s).1.z);
        if before.x.abs() > 2e-9 || before.y >= -1e-9 {
            return Err("recollision requires an existing bearing and a closing new site".into());
        }
        let a = -before.y / wn;
        let vo = before.x + cross * a;
        let (j, sites) = if vo > 1e-8 {
            (DVec2::new(0., a), Sites::One(new))
        } else if vo < -1e-8 {
            let j = -(DMat2::from_cols(DVec2::new(wo, cross), DVec2::new(cross, wn)).inverse()
                * before);
            if j.min_element() <= 0. {
                return Err("unresolved two-point normal LCP".into());
            }
            (j, Sites::Two([old, new]))
        } else {
            return Err("neutral old bearing at recollision".into());
        };
        let v = s.v + DVec3::Z * ((j.x + j.y) / self.mass);
        let w = s.motion.w + self.inv(q, ro.cross(j.x * DVec3::Z) + rn.cross(j.y * DVec3::Z));
        let after = State {
            motion: Motion { q, w },
            c: s.c,
            v,
        };
        let loss = -0.5 * before.dot(j);
        if (self.energy(s) - self.energy(after) - loss).abs() > 1e-10
            || self.point(new, after).1.z.abs() > 2e-9
            || self.point(old, after).1.z < -2e-9
        {
            return Err("recollision violated work or complementarity".into());
        }
        Ok(Jump {
            normal: j.to_array(),
            old,
            new,
            after,
            loss,
            new_sites: sites,
        })
    }
}
impl Arc {
    fn rates(self, s: Motion) -> Result<Rate> {
        let q = s.q.normalize();
        let w = s.w;
        let a = self.sites.first();
        let r = q * self.shape.vertices[a];
        let inertia_w = q * (self.shape.inertia * (q.conjugate() * w));
        let gyro = self.shape.inv(q, -w.cross(inertia_w));
        let free = |r: DVec3| self.shape.gravity.z + (gyro.cross(r) + w.cross(w.cross(r))).z;
        let w0 = self.shape.normal_mobility(q, r, r);
        let (loads, moment) = match self.sites {
            Sites::One(_) => {
                let n = -free(r) / w0;
                ([n, 0.], r.cross(n * DVec3::Z))
            }
            Sites::Two([_, b]) => {
                let rb = q * self.shape.vertices[b];
                let cross = self.shape.normal_mobility(q, r, rb);
                let w1 = self.shape.normal_mobility(q, rb, rb);
                let m = DMat2::from_cols(DVec2::new(w0, cross), DVec2::new(cross, w1));
                if m.determinant() <= 1e-12 * w0 * w1 {
                    return Err("degenerate two-normal sustained constraint".into());
                }
                let n = -m.inverse() * DVec2::new(free(r), free(rb));
                (
                    n.to_array(),
                    r.cross(n.x * DVec3::Z) + rb.cross(n.y * DVec3::Z),
                )
            }
        };
        let alpha = gyro + self.shape.inv(q, moment);
        let dq = (DQuat::from_xyzw(w.x, w.y, w.z, 0.) * q) * 0.5;
        Ok(Rate {
            q: dq,
            w: alpha,
            loads,
        })
    }
    pub fn load(self, s: Motion) -> Result<[f64; 2]> {
        Ok(self.rates(s)?.loads)
    }
    fn advance(self, s: Motion, dt: f64) -> Result<Motion> {
        Ok(self.advance_loaded(s, dt)?.0)
    }
    pub fn advance_loaded(self, s: Motion, dt: f64) -> Result<(Motion, [f64; 2])> {
        let step = |k: Rate, h: f64| Motion {
            q: s.q + k.q * h,
            w: s.w + h * k.w,
        };
        let a = self.rates(s)?;
        let b = self.rates(step(a, 0.5 * dt))?;
        let c = self.rates(step(b, 0.5 * dt))?;
        let d = self.rates(step(c, dt))?;
        let loads = std::array::from_fn(|i| {
            dt / 6. * (a.loads[i] + 2. * (b.loads[i] + c.loads[i]) + d.loads[i])
        });
        Ok((
            Motion {
                q: (s.q + (a.q + (b.q + c.q) * 2. + d.q) * (dt / 6.)).normalize(),
                w: s.w + (dt / 6.) * (a.w + 2. * (b.w + c.w) + d.w),
            },
            loads,
        ))
    }
    pub fn state(self, t: f64, motion: Motion) -> State {
        let r = motion.q * self.shape.vertices[self.sites.first()];
        let mut c = self.start.c + t * self.start.v + 0.5 * t * t * self.shape.gravity;
        let mut v = self.start.v + t * self.shape.gravity;
        c.z = -r.z;
        v.z = -motion.w.cross(r).z;
        State { motion, c, v }
    }
    pub fn at(self, t: f64, max_step: f64) -> Result<State> {
        let mut m = self.start.motion;
        let steps = (t.abs() / max_step).ceil() as u32;
        if steps > 0 {
            let dt = t / f64::from(steps);
            for _ in 0..steps {
                m = self.advance(m, dt)?;
            }
        }
        Ok(self.state(t, m))
    }
    fn min_load(self, m: Motion) -> Result<f64> {
        let n = self.load(m)?;
        Ok(match self.sites {
            Sites::One(_) => n[0],
            Sites::Two(_) => n[0].min(n[1]),
        })
    }
    fn lateral_clearance(self, s: State) -> f64 {
        self.shape
            .vertices
            .iter()
            .map(|r| {
                let p = s.c + s.motion.q * *r;
                (self.shape.pad_half[0] - p.x.abs()).min(self.shape.pad_half[1] - p.y.abs())
            })
            .fold(f64::INFINITY, f64::min)
    }
    fn angular_bounds(self, m: Motion, h: f64) -> Option<(f64, f64, f64)> {
        let Sites::One(id) = self.sites else {
            return None;
        };
        let frobenius = |a: DMat3| {
            (a.x_axis.length_squared() + a.y_axis.length_squared() + a.z_axis.length_squared())
                .sqrt()
        };
        let inv = frobenius(self.shape.inertia.inverse());
        let condition = inv * frobenius(self.shape.inertia);
        let radius = self.shape.vertices[id].length();
        let omega = m.w.length() + 1.;
        let gyro = condition * omega * omega;
        let normal =
            self.shape.mass * (self.shape.gravity.z.abs() + radius * (gyro + omega * omega));
        let alpha = gyro + inv * radius * normal;
        if h * alpha >= 1. {
            return Some((f64::INFINITY, f64::INFINITY, f64::INFINITY));
        }
        // Bounds for the time derivative of the free normal acceleration
        // and the second derivative of another vertex's relative gap.
        let normal_derivative = radius
            * ((2. * condition + 1.) * omega.powi(3) + (2. * condition + 2.) * omega * alpha);
        Some((omega, alpha + omega * omega, normal_derivative))
    }
    pub fn next(self, max_step: f64, horizon: f64) -> Result<Arrival> {
        let mut m = self.start.motion;
        let mut t = 0.;
        let k0 = self.shape.energy(self.start);
        let mut min_load = self.min_load(m)?;
        let mut energy = 0f64;
        let mut constraint = 0f64;
        let finish =
            |time: f64, m: Motion, end: End, min_load: f64, energy: f64, constraint: f64| Arrival {
                time,
                state: self.state(time, m),
                end,
                min_load,
                max_energy: energy,
                max_constraint: constraint,
            };
        if min_load <= 0. {
            return Ok(finish(t, m, End::Loss, min_load, energy, constraint));
        }
        if self.lateral_clearance(self.start) <= 1e-8 {
            return Ok(finish(t, m, End::Boundary, min_load, energy, constraint));
        }
        let mut step = max_step;
        while t < horizon {
            let h = (horizon - t).min(step);
            if h < 1e-16 {
                break;
            }
            let b = self.advance(m, h)?;
            let a = self.state(t, m);
            let next = self.state(t + h, b);
            let load = self.min_load(b)?;
            min_load = min_load.min(load);
            energy = energy.max((self.shape.energy(next) - k0).abs());
            if let Sites::Two([_, k]) = self.sites {
                let (p, v) = self.shape.point(k, next);
                constraint = constraint.max(p.z.abs()).max(v.z.abs());
            }
            let bounds = self.angular_bounds(m, h);
            let mut subdivide = false;
            let mut candidates = vec![];
            for k in 0..self.shape.vertices.len() {
                if self.sites.has(k) {
                    continue;
                }
                let (pa, va) = self.shape.point(k, a);
                let (pb, vb) = self.shape.point(k, next);
                if pa.z < -2e-11 {
                    return Err("existing penetration outside the declared active set".into());
                }
                if let Some((_, accel, _)) = bounds {
                    let span =
                        (self.shape.vertices[k] - self.shape.vertices[self.sites.first()]).length();
                    let acc = accel * span;
                    // A monotone gap or its bounded linear-interpolation
                    // remainder rules out an unseen crossing between samples.
                    let monotone = va.z.abs() > acc * h || vb.z.abs() > acc * h;
                    if !monotone && pa.z.min(pb.z) <= acc * h * h / 8. + 5e-15 {
                        subdivide = true;
                    }
                }
                if pb.z <= 0. && vb.z < 0. {
                    let mut lo = 0.;
                    let mut hi = h;
                    for _ in 0..38 {
                        let mid = 0.5 * (lo + hi);
                        let state = self.state(t + mid, self.advance(m, mid)?);
                        if self.shape.point(k, state).0.z > 0. {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    candidates.push((0.5 * (lo + hi), End::Contact(k)));
                } else if pa.z < 1e-12 && va.z < -1e-8 {
                    return Err("undeclared closing contact at arc start".into());
                }
            }
            if let Some((omega, _, jerk)) = bounds {
                let r_a = m.q * self.shape.vertices[self.sites.first()];
                let r_b = b.q * self.shape.vertices[self.sites.first()];
                let free_a = self.min_load(m)? * self.shape.normal_mobility(m.q, r_a, r_a);
                let free_b = load * self.shape.normal_mobility(b.q, r_b, r_b);
                if free_a > 0. && free_b > 0. && free_a.min(free_b) <= 0.5 * h * jerk {
                    subdivide = true;
                }
                let edge_speed = a.v.truncate().length()
                    + h * self.shape.gravity.truncate().length()
                    + omega
                        * self
                            .shape
                            .vertices
                            .iter()
                            .map(|p| p.length())
                            .fold(0., f64::max);
                let aa = self.lateral_clearance(a) - 1e-8;
                let bb = self.lateral_clearance(next) - 1e-8;
                if aa > 0. && bb > 0. && aa.min(bb) <= 0.5 * h * edge_speed {
                    subdivide = true;
                }
            }
            if subdivide {
                if h < 1e-10 {
                    return Ok(finish(t, m, End::Ambiguous, min_load, energy, constraint));
                }
                step = 0.5 * h;
                continue;
            }
            if load <= 0. || self.lateral_clearance(next) <= 1e-8 {
                let contact_loss = load <= 0.;
                let mut lo = 0.;
                let mut hi = h;
                for _ in 0..38 {
                    let mid = 0.5 * (lo + hi);
                    let at = self.advance(m, mid)?;
                    let positive = if contact_loss {
                        self.min_load(at)? > 0.
                    } else {
                        self.lateral_clearance(self.state(t + mid, at)) > 1e-8
                    };
                    if positive {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                candidates.push((
                    0.5 * (lo + hi),
                    if contact_loss {
                        End::Loss
                    } else {
                        End::Boundary
                    },
                ));
            }
            if !candidates.is_empty() {
                candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
                let (at, mut end) = candidates[0];
                if candidates.get(1).is_some_and(|b| (at - b.0).abs() < 1e-9) {
                    end = End::Ambiguous;
                }
                return Ok(finish(
                    t + at,
                    self.advance(m, at)?,
                    end,
                    min_load,
                    energy,
                    constraint,
                ));
            }
            t += h;
            m = b;
            step = (2. * h).min(max_step);
        }
        Ok(finish(t, m, End::Horizon, min_load, energy, constraint))
    }
}
