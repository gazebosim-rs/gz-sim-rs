// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent upper-strike Coulomb law and finite-vertex free-flight ordering.
use super::*;
use glam::{DMat2, DVec2};
#[path = "inertia_upper_native.rs"]
mod native;
pub(crate) use native::projected;
#[path = "inertia_opposing_gap.rs"]
pub(crate) mod gaps;
#[path = "inertia_upper_view.rs"]
mod views;
#[derive(Clone, Copy)]
struct Face {
    frame: DQuat,
    origin: DVec3,
    v: DVec3,
    half: DVec2,
}
impl Face {
    fn axes(self) -> [DVec3; 3] {
        [
            self.frame * DVec3::X,
            self.frame * DVec3::Y,
            self.frame * DVec3::Z,
        ]
    }
    fn coordinates(self, p: DVec3, t: f64) -> DVec3 {
        self.frame.conjugate() * (p - self.origin - self.v * t)
    }
}
fn faces(r: &Physical, g: &Gate) -> [Face; 2] {
    [
        Face {
            frame: r.pad_q,
            origin: DVec3::ZERO,
            v: r.pad_v,
            half: DVec2::new(PAD_HALF[0], PAD_HALF[1]),
        },
        Face {
            frame: r.pad_q * DQuat::from_xyzw(1., 0., 0., 0.),
            origin: g.center - r.n * HALF_GATE[2],
            v: g.velocity,
            half: DVec2::new(HALF_GATE[0], HALF_GATE[1]),
        },
    ]
}
fn vertices() -> [DVec3; 8] {
    let bottom = corners();
    std::array::from_fn(|i| {
        let mut p = bottom[i % 4];
        if i >= 4 {
            p.z = -p.z;
        }
        p - ARM
    })
}
fn from_primary(r: &Physical, t: f64) -> Flight {
    let s = r.state(t);
    Flight {
        com: s.com(),
        v: s.velocity(),
        q: DQuat::from_array(s.foot.orientation),
        l: if t < r.arrival { r.pre.l } else { r.post.l },
    }
}
#[derive(Clone, Copy)]
struct Law {
    impulse: DVec3,
    before: DVec3,
    after: DVec3,
    energy_loss: f64,
    monotone: f64,
    stick: bool,
    effective: DMat3,
}
fn coulomb(
    body: ReferenceBody,
    x: Flight,
    arm: DVec3,
    plane: Face,
    mu: f64,
) -> std::result::Result<Law, &'static str> {
    let axes = plane.axes();
    let before = plane.frame.conjugate() * (x.v - plane.v + body.omega(x.q, x.l).cross(arm));
    if before.z >= -1e-7 {
        return Err("nonclosing point");
    }
    let [a, b, c] = axes.map(|axis| {
        plane.frame.conjugate() * (axis / M + body.omega(x.q, arm.cross(axis)).cross(arm))
    });
    let k = DMat3::from_cols(a, b, c);
    let cross = DVec2::new(c.x, c.y);
    let schur = DMat2::from_cols(
        a.truncate() - cross * (a.z / c.z),
        b.truncate() - cross * (b.z / c.z),
    );
    let trace = schur.x_axis.x + schur.y_axis.y;
    let diff = (schur.x_axis.x - schur.y_axis.y).hypot(2. * schur.x_axis.y);
    let (lo, hi) = ((trace - diff) / 2., (trace + diff) / 2.);
    if c.z <= 0. || lo <= 0. {
        return Err("singular effective mass");
    }
    // The positive tangential multiplier is slip speed / tangential impulse.
    // Eliminating the normal component leaves a strictly decreasing scalar
    // cone residual if every rotated principal direction satisfies this bound.
    let monotone = 2. * (lo * hi).sqrt() / (lo + hi) - mu * cross.length() / c.z;
    if monotone < 1e-9 {
        return Err("Coulomb uniqueness not certified");
    }
    let rhs = -before.truncate() + cross * (before.z / c.z);
    let response = |lambda: f64| {
        let jt = (schur + lambda * DMat2::IDENTITY).inverse() * rhs;
        let jn = (-before.z - cross.dot(jt)) / c.z;
        (jt.extend(jn), jt.length() - mu * jn)
    };
    let (trial, residual) = response(0.);
    let (local, stick) = if mu == 0. {
        (DVec3::new(0., 0., -before.z / c.z), false)
    } else if residual <= 0. {
        (trial, true)
    } else {
        let (mut left, mut right) = (0., hi.max(1.));
        for _ in 0..96 {
            if response(right).1 < 0. {
                break;
            }
            right *= 2.;
        }
        if response(right).1 >= 0. {
            return Err("sliding multiplier outside bracket");
        }
        for _ in 0..72 {
            let mid = (left + right) / 2.;
            if response(mid).1 > 0. {
                left = mid;
            } else {
                right = mid;
            }
        }
        (response((left + right) / 2.).0, false)
    };
    let after = before + k * local;
    let loss = -0.5 * local.dot(before + after);
    if local.z <= 0.
        || loss < 0.
        || after.z.abs() > 1e-9
        || (stick && after.length() > 1e-9)
        || (!stick
            && mu > 0.
            && (after.truncate().normalize() + local.truncate().normalize()).length() > 1e-8)
    {
        return Err("impulse residual");
    }
    Ok(Law {
        impulse: plane.frame * local,
        before: plane.frame * before,
        after: plane.frame * after,
        energy_loss: loss,
        monotone,
        stick,
        effective: k,
    })
}
#[derive(Clone, Copy)]
struct Crossing {
    at: f64,
    surface: usize,
    corner: usize,
    before: Flight,
    arm: DVec3,
    plane: Face,
    mu: f64,
    law: Law,
    other_gap: f64,
    finite_margin: f64,
}
impl Crossing {
    fn after(self) -> Flight {
        Flight {
            v: self.before.v + self.law.impulse / M,
            l: self.before.l + self.arm.cross(self.law.impulse),
            ..self.before
        }
    }
    fn data(self) -> Value {
        let x = self;
        json!({"at_ns":x.at*1e9,"face":x.surface,"corner":x.corner,"point":(x.before.com+x.arm).to_array(),"arm":x.arm.to_array(),"mu":x.mu,"normal":(x.plane.frame*DVec3::Z).to_array(),"active_prior_velocity":x.law.before.to_array(),"after_relative_velocity":x.law.after.to_array(),"impulse":x.law.impulse.to_array(),"normal_ns":x.law.impulse.dot(x.plane.frame*DVec3::Z),"loss_j":x.law.energy_loss,"stick":x.law.stick,"schur_margin":x.law.monotone,"delassus_columns":x.law.effective.to_cols_array(),"other_gap_m":x.other_gap,"finite_margin_m":x.finite_margin,"orientation":x.before.q.to_array(),"com":x.before.com.to_array(),"velocity":x.before.v.to_array(),"angular_momentum":x.before.l.to_array(),"after_velocity":x.after().v.to_array(),"after_angular_momentum":x.after().l.to_array()})
    }
}
fn event(
    i: &Impact,
    planes: [Face; 2],
    mu: [f64; 2],
    x: Flight,
    t: f64,
    face: usize,
    k: usize,
) -> std::result::Result<Crossing, &'static str> {
    let pts = vertices();
    let mut other_gap = f64::INFINITY;
    let mut margin = f64::INFINITY;
    for (j, p) in pts.iter().enumerate() {
        let world = x.com + x.q * (*p);
        for (s, f) in planes.iter().enumerate() {
            let r = f.coordinates(world, t);
            // Qualify finite-face events only while all vertices are interior
            // to both footprints. Side/edge arrivals are separate contact laws.
            margin = margin.min((f.half - r.truncate().abs()).min_element());
            if (s, j) == (face, k) {
                if r.z.abs() > 2e-10 {
                    return Err("unmatched event cut");
                }
            } else {
                other_gap = other_gap.min(r.z);
            }
        }
    }
    if margin <= 1e-9 {
        return Err("finite footprint");
    }
    if other_gap <= 1e-10 {
        return Err("simultaneous second contact");
    }
    let arm = x.q * pts[k];
    let law = coulomb(i.r.body, x, arm, planes[face], mu[face])?;
    Ok(Crossing {
        at: t,
        corner: k,
        surface: face,
        before: x,
        arm,
        plane: planes[face],
        mu: mu[face],
        law,
        other_gap,
        finite_margin: margin,
    })
}
fn upper(i: &Impact, g: &Gate, mu: f64) -> std::result::Result<Crossing, &'static str> {
    if g.at < i.r.arrival {
        return Err("upper arrives before primary");
    }
    let plane = faces(&i.r, g);
    let x = from_primary(&i.r, g.at);
    let corner = vertices()
        .into_iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            plane[1]
                .coordinates(x.com + x.q * (*a), g.at)
                .z
                .total_cmp(&plane[1].coordinates(x.com + x.q * (*b), g.at).z)
        })
        .unwrap()
        .0;
    event(i, plane, [i.mu, mu], x, g.at, 1, corner)
}

#[derive(Clone, Copy)]
struct Bounds {
    w: f64,
    alpha: f64,
    beta: f64,
}
impl Bounds {
    fn new(body: ReferenceBody, x: Flight) -> Self {
        let w = (x.l.dot(body.omega(x.q, x.l)) / body.lower)
            .sqrt()
            .min(x.l.length() / body.lower);
        let ratio = (body.upper - body.lower) / body.lower;
        let alpha = 0.5 * ratio * w * w;
        Self {
            w,
            alpha,
            beta: (1. + ratio) * w * alpha,
        }
    }
}
struct Continued {
    through: f64,
    next: Option<Crossing>,
    reason: &'static str,
    integration_error: f64,
}
fn follow(i: &Impact, planes: [Face; 2], mu: [f64; 2], first: Crossing, end: f64) -> Continued {
    let r = &i.r;
    let mut x = first.after();
    let (mut q2, mut w2) = (x.q, x.q.conjugate() * r.body.omega(x.q, x.l));
    let bounds = Bounds::new(r.body, x);
    let pts = vertices();
    let mut t = first.at;
    let mut error = 0f64;
    let mut h = RK_H.min(end - t);
    while end - t > 1e-14 {
        h = h.min(end - t);
        let mut later = x;
        later.advance(r, h);
        let w = r.body.omega(x.q, x.l);
        let alpha = r.body.accel(x.q, w);
        let mut candidate = None;
        let mut reason = None;
        for (s, face) in planes.iter().copied().enumerate() {
            let n = face.frame * DVec3::Z;
            for (k, &local) in pts.iter().enumerate() {
                let arm = x.q * local;
                let geometry = face.coordinates(x.com + arm, t);
                let travel = ((x.v - face.v).length()
                    + bounds.w * arm.length()
                    + 0.5 * r.gravity.length() * h)
                    * h;
                if (face.half - geometry.truncate().abs()).min_element() <= travel {
                    reason = Some("finite footprint");
                    break;
                }
                let v = (x.v - face.v + w.cross(arm)).dot(n);
                let a = (r.gravity + alpha.cross(arm) + w.cross(w.cross(arm))).dot(n);
                let abound =
                    r.gravity.dot(n).abs() + (bounds.alpha + bounds.w.powi(2)) * arm.length();
                let jerk =
                    (bounds.beta + 3. * bounds.w * bounds.alpha + bounds.w.powi(3)) * arm.length();
                let fired = (s, k) == (first.surface, first.corner) && t == first.at;
                if fired && a < -1e-8 {
                    return Continued {
                        through: t,
                        next: None,
                        reason: "compressive acceleration at struck point",
                        integration_error: error,
                    };
                }
                if (geometry.z >= -2e-13
                    && (v - abound * h > 0. || (v >= 0. || fired) && a - jerk * h > 0.))
                    || geometry.z + v.min(0.) * h - 0.5 * abound * h * h > 2e-13
                {
                    continue;
                }
                let gend = face.coordinates(later.com + later.q * local, t + h).z;
                if geometry.z > 0. && v + abound * h < -1e-6 {
                    if gend > 2e-13 {
                        continue;
                    }
                    if gend <= 0. && candidate.is_none() {
                        candidate = Some((s, k));
                        continue;
                    }
                }
                reason = Some("tied or unbounded flight");
                break;
            }
            if reason.is_some() {
                break;
            }
        }
        if let Some(why) = reason {
            h *= 0.5;
            if h < 2e-11 {
                return Continued {
                    through: t,
                    next: None,
                    reason: why,
                    integration_error: error,
                };
            }
            continue;
        }
        if let Some((s, k)) = candidate {
            let (mut a, mut b) = (0., h);
            for _ in 0..48 {
                let m = 0.5 * (a + b);
                let mut st = x;
                st.advance(r, m);
                if planes[s].coordinates(st.com + st.q * pts[k], t + m).z > 0. {
                    a = m;
                } else {
                    b = m;
                }
            }
            let dt = 0.5 * (a + b);
            x.advance(r, dt);
            let (qt, _) = advance_euler(r.body, q2, w2, dt);
            error = error.max(angular_distance(x.q, qt));
            assert!(error < 2e-10);
            let at = t + dt;
            return match event(i, planes, mu, x, at, s, k) {
                Ok(next) => Continued {
                    through: at,
                    next: Some(next),
                    reason: "next finite point",
                    integration_error: error,
                },
                Err(why) => Continued {
                    through: at,
                    next: None,
                    reason: why,
                    integration_error: error,
                },
            };
        }
        x = later;
        for _ in 0..2 {
            (q2, w2) = advance_euler(r.body, q2, w2, 0.5 * h);
        }
        error = error.max(angular_distance(x.q, q2));
        assert!(error < 2e-10);
        t += h;
        h = RK_H;
    }
    Continued {
        through: end,
        next: None,
        reason: "window complete",
        integration_error: error,
    }
}

fn material(g: &Gate, m: f64) -> String {
    g.xml.replace("</collision>",&format!("<surface><friction><ode><mu>{m}</mu><mu2>{m}</mu2></ode></friction></surface></collision>"))
}
pub(crate) fn write(path: &Path) -> crate::Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    let mut featured = false;
    let mut counts = std::collections::BTreeMap::new();
    for i in inputs() {
        let mut cases = vec![];
        for delay in [0, 8, 20000, 750000, 1100000] {
            let g = gate(&i, delay as f64 * 1e-9);
            for gate_mu in [0., 0.35, 0.8] {
                let mu = i.mu.min(gate_mu);
                let data = match upper(&i, &g, mu) {
                    Err(why) => {
                        *counts.entry(why).or_insert(0) += 1;
                        json!({"reason":why})
                    }
                    Ok(hit) => {
                        let planes = faces(&i.r, &g);
                        let after_upper = follow(&i, planes, [i.mu, mu], hit, WINDOW);
                        *counts.entry(after_upper.reason).or_insert(0) += 1;
                        if !featured
                            && i.r.id >= 1100
                            && mu > 0.
                            && !hit.law.stick
                            && delay >= 750000
                            && let Some(next) = after_upper.next
                            && next.surface == 0
                            && next.at - hit.at > 0.000012
                        {
                            views::write(path, &i, &g, gate_mu, hit, next)?;
                            featured = true;
                        }
                        let raw = native::sample(&i, &g, gate_mu, hit, &after_upper)?;
                        let xml = native::source(&i, &g, gate_mu);
                        let mut subsequent = vec![];
                        let mut event = after_upper.next;
                        for step in 0..8 {
                            let Some(current) = event else {
                                break;
                            };
                            let after = follow(&i, planes, [i.mu, mu], current, WINDOW);
                            let control = if step == 0 {
                                let (h, cold) = native::response(&i, &g, &xml, current, &after)?;
                                Some(json!({"span_ns":h,"cold":cold}))
                            } else {
                                None
                            };
                            subsequent.push(json!({"event":current.data(),"through_ns":after.through*1e9,"follow":after.reason,
                                "next":after.next.map(Crossing::data),"attitude_comparison_rad":after.integration_error,"matched":control}));
                            event = after.next;
                        }
                        json!({"upper":hit.data(),"through_ns":after_upper.through*1e9,"follow":after_upper.reason,"next":after_upper.next.map(Crossing::data),"attitude_comparison_rad":after_upper.integration_error,"native":raw,"subsequent":subsequent,"sequence_budget_reached":event.is_some()})
                    }
                };
                cases.push(
                    json!({"delay_ns":delay,"upper_mu":gate_mu,"effective_mu":mu,"data":data}),
                );
            }
        }
        rows.push(json!({"id":i.r.id,"mode":MODES[i.mode],"primary":law(&i),"cases":cases}));
    }
    write_json(
        &path.join("mechanics.json"),
        &json!({"cases":rows,"outcomes":counts}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn planar_com_and_manufactured_spatial_sliding_agree_with_the_independent_multiplier() {
        let i = manufactured(1160, 3);
        let r = &i.r;
        let plane = Face {
            frame: DQuat::IDENTITY,
            origin: DVec3::ZERO,
            v: DVec3::ZERO,
            half: DVec2::splat(2.),
        };
        for (v, mu, expected, stick) in [
            (
                DVec3::new(0.1, -0.2, -1.),
                0.5,
                DVec3::new(-0.1, 0.2, 1.) * M,
                true,
            ),
            (
                DVec3::new(2., 0., -1.),
                0.5,
                DVec3::new(-0.5, 0., 1.) * M,
                false,
            ),
            (DVec3::new(2., 1., -1.), 0., DVec3::Z * M, false),
        ] {
            let x = Flight {
                q: r.q,
                v,
                com: DVec3::ZERO,
                l: DVec3::ZERO,
            };
            let law = coulomb(r.body, x, DVec3::ZERO, plane, mu).unwrap();
            assert_eq!(law.stick, stick);
            assert!(law.impulse.distance(expected) < 1e-12);
        }
        for id in 0..32 {
            let i = manufactured(id, 3);
            let r = &i.r;
            let fac = Face {
                frame: r.pad_q,
                origin: DVec3::ZERO,
                v: r.pad_v,
                half: DVec2::splat(2.),
            };
            let x = Flight {
                q: r.q,
                v: r.pre.v,
                com: r.at,
                l: r.pre.l,
            };
            let arm = r.q * (corners()[r.corner] - ARM);
            if let Ok(sol) = coulomb(r.body, x, arm, fac, i.mu) {
                assert!(sol.impulse.distance(i.impulse) < 2e-11);
                assert!((sol.energy_loss - i.loss).abs() < 1e-10);
            }
        }
    }
    #[test]
    fn free_return_after_a_zero_torque_ceiling_strike_matches_the_quadratic_root() {
        let mut i = manufactured(1160, 3);
        let pts = vertices();
        let k = 3;
        let radial = pts[k].normalize();
        let low = DQuat::from_rotation_arc(DVec3::Z, radial);
        let high = low * DQuat::from_xyzw(1., 0., 0., 0.);
        i.r.gravity = -3. * radial;
        let x = Flight {
            com: DVec3::ZERO,
            q: DQuat::IDENTITY,
            v: radial,
            l: DVec3::ZERO,
        };
        let gap = 0.00013;
        let lowest = pts
            .iter()
            .map(|p| p.dot(radial))
            .fold(f64::INFINITY, f64::min);
        let planes = [
            Face {
                frame: low,
                origin: (lowest - gap) * radial,
                v: DVec3::ZERO,
                half: DVec2::splat(3.),
            },
            Face {
                frame: high,
                origin: pts[k],
                v: -radial,
                half: DVec2::splat(3.),
            },
        ];
        let hit = event(&i, planes, [0.; 2], x, 0., 1, k).unwrap();
        assert!(hit.after().v.distance(-radial) < 1e-12);
        assert!(hit.after().l.length() < 1e-14);
        let exact = 2. * gap / (1. + (1. + 6. * gap).sqrt());
        let result = follow(&i, planes, [0.; 2], hit, 2. * exact);
        let next = result
            .next
            .unwrap_or_else(|| panic!("{} at {}", result.reason, result.through));
        assert_eq!(next.surface, 0);
        assert!((next.at - exact).abs() < 1e-14);
        assert!((next.law.before.dot(radial) + 1. + 3. * exact).abs() < 1e-12);
    }
}
