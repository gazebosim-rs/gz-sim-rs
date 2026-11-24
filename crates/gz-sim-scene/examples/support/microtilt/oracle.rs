// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! First plastic frictional impact under gravity and the brief following single-point phase.
use super::*;
use gz_physics::PointImpact;

const G: DVec3 = DVec3::new(0., 0., -9.81);
const MASS: f64 = 2.;
const INERTIA: f64 = 0.05;
const ARMS: [DVec3; 4] = [
    DVec3::new(-0.18, -0.1, -0.13),
    DVec3::new(-0.18, 0.1, -0.13),
    DVec3::new(0.18, -0.1, -0.13),
    DVec3::new(0.18, 0.1, -0.13),
];
#[derive(Clone, Copy)]
pub(super) struct PoseState {
    pub c: DVec3,
    pub q: DQuat,
    pub v: DVec3,
    pub w: DVec3,
}
impl PoseState {
    pub fn wire(self) -> Value {
        json!({"com_m":self.c.to_array(),"orientation":self.q.to_array(),"velocity_m_s":self.v.to_array(),"spin_rad_s":self.w.to_array()})
    }
    pub fn airborne(self, t: f64) -> Self {
        Self {
            c: self.c + t * self.v + 0.5 * t * t * G,
            q: (DQuat::from_scaled_axis(t * self.w) * self.q).normalize(),
            v: self.v + t * G,
            ..self
        }
    }
    fn point(self, k: usize) -> DVec3 {
        self.c + self.q * ARMS[k]
    }
    fn point_v(self, k: usize) -> DVec3 {
        self.v + self.w.cross(self.q * ARMS[k])
    }
    fn energy(self) -> f64 {
        0.5 * (MASS * self.v.length_squared() + INERTIA * self.w.length_squared())
    }
}
/// Delassus matrix in world axes for the spherical aggregate inertia and one contact point.
fn inverse_mass(r: DVec3) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            (if i == j {
                1. / MASS + r.length_squared() / INERTIA
            } else {
                0.
            }) - r[i] * r[j] / INERTIA
        })
    })
}
#[derive(Clone, Copy, Debug)]
enum Mode {
    Opening,
    Sticking,
    Sliding,
}
#[derive(Clone, Copy)]
struct Derivative {
    dc: DVec3,
    dq: DQuat,
    dv: DVec3,
    dw: DVec3,
    force: DVec3,
}
fn derivative(s: PoseState, k: usize, mu: f64, mode: Mode) -> Result<Derivative> {
    let r = s.q * ARMS[k];
    let w = s.w;
    let free = G + w.cross(w.cross(r));
    let k_eff = inverse_mass(r);
    let force = match mode {
        Mode::Opening => {
            if free.z < 0. {
                return Err("opening point reversed acceleration".into());
            }
            DVec3::ZERO
        }
        Mode::Sticking => {
            // Analytic inverse of a I - r rᵀ/I. Eigenvalues 1/m along r,
            // and 1/m + |r|²/I on the perpendicular space.
            let parallel = r * (r.dot(free) / r.length_squared());
            let f =
                -MASS * parallel - (free - parallel) / (1. / MASS + r.length_squared() / INERTIA);
            if f.z <= 0. || f.truncate().length() > mu * f.z + 1e-12 {
                return Err("sticking point left its friction cone".into());
            }
            f
        }
        Mode::Sliding => {
            let vt = s.point_v(k) * DVec3::new(1., 1., 0.);
            if vt.length() < 1e-8 {
                return Err("sliding point approached a friction transition".into());
            }
            let direction = DVec3::Z - mu * vt.normalize();
            let normal = -free.z / DVec3::from(k_eff[2]).dot(direction);
            if normal <= 0. {
                return Err("sliding contact opened".into());
            }
            direction * normal
        }
    };
    Ok(Derivative {
        dc: s.v,
        dq: (DQuat::from_xyzw(w.x, w.y, w.z, 0.) * s.q) * 0.5,
        dv: G + force / MASS,
        dw: r.cross(force) / INERTIA,
        force,
    })
}
fn increment(s: PoseState, h: f64, d: Derivative) -> PoseState {
    PoseState {
        c: s.c + h * d.dc,
        q: (s.q + d.dq * h).normalize(),
        v: s.v + h * d.dv,
        w: s.w + h * d.dw,
    }
}
fn advance(
    mut s: PoseState,
    dt: f64,
    k: usize,
    mu: f64,
    mode: Mode,
    steps: usize,
) -> Result<(PoseState, DVec3)> {
    if matches!(mode, Mode::Opening) {
        return Ok((s.airborne(dt), DVec3::ZERO));
    }
    let h = dt / steps as f64;
    let mut impulse = DVec3::ZERO;
    for _ in 0..steps {
        let a = derivative(s, k, mu, mode)?;
        let b = derivative(increment(s, 0.5 * h, a), k, mu, mode)?;
        let c = derivative(increment(s, 0.5 * h, b), k, mu, mode)?;
        let d = derivative(increment(s, h, c), k, mu, mode)?;
        let blend = Derivative {
            dc: (a.dc + 2. * b.dc + 2. * c.dc + d.dc) / 6.,
            dq: (a.dq + b.dq * 2. + c.dq * 2. + d.dq) / 6.,
            dv: (a.dv + 2. * b.dv + 2. * c.dv + d.dv) / 6.,
            dw: (a.dw + 2. * b.dw + 2. * c.dw + d.dw) / 6.,
            force: (a.force + 2. * b.force + 2. * c.force + d.force) / 6.,
        };
        s = increment(s, h, blend);
        impulse += h * blend.force;
        if s.point(k).z.abs() > 1e-11 || s.point_v(k).z.abs() > 1e-9 {
            return Err("persistent point constraint drift".into());
        }
    }
    Ok((s, impulse))
}
pub(super) struct Oracle {
    pub time: f64,
    pub stop_ns: u64,
    pub incoming: PoseState,
    pub endpoint: PoseState,
    pub impulse: DVec3,
    pub corner: usize,
    pub wire: Value,
}
pub(super) fn solve(start: PoseState, mu: f64) -> Result<Oracle> {
    let horizon = DT;
    let radius = ARMS[0].length();
    if start.v.z + start.w.length() * radius >= 0. {
        return Err("strictly closing incoming corners required".into());
    }
    let mut roots = [0.; 4];
    for (k, root) in roots.iter_mut().enumerate() {
        let p = start.point(k);
        if p.z <= 0. || start.airborne(horizon).point(k).z >= 0. {
            return Err("first roots not bracketed by the original public tick".into());
        }
        let mut lo = 0.;
        let mut hi = horizon;
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if start.airborne(mid).point(k).z > 0. {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        *root = 0.5 * (lo + hi);
    }
    let corner = (0..4)
        .min_by(|a, b| roots[*a].total_cmp(&roots[*b]))
        .unwrap();
    let time = roots[corner];
    let incoming = start.airborne(time);
    // Each corner is strictly on the finite support face. Exclude all four
    // side faces and the upper link by positive clearance over the short arc.
    let bound = horizon * (start.v.length() + start.w.length() * radius + 9.81 * horizon);
    let lower_upper = (start.c + start.q * ARM).z
        - (start.q.conjugate() * DVec3::Z)
            .abs()
            .dot(DVec3::new(0.05, 0.06, 0.06));
    if (0..4).any(|k| {
        let p = start.point(k);
        p.x.abs() + bound >= 0.33 || p.y.abs() + bound >= 0.18
    }) || lower_upper <= bound
        || (start.q * DVec3::Z).z <= start.w.length() * horizon
    {
        return Err("unqualified finite supported footprint".into());
    }
    let gap = (0..4)
        .filter(|k| *k != corner)
        .map(|k| incoming.point(k).z)
        .fold(f64::INFINITY, f64::min);
    if gap < 1e-8 {
        return Err("first contact not isolated".into());
    }
    let r = incoming.q * ARMS[corner];
    let relative = incoming.point_v(corner);
    let result = PointImpact {
        inverse_mass: inverse_mass(r),
        relative_velocity: relative.to_array(),
        friction: mu,
    }
    .solve()?;
    let jump = DVec3::from(result.impulse);
    let outgoing = PoseState {
        v: incoming.v + jump / MASS,
        w: incoming.w + r.cross(jump) / INERTIA,
        ..incoming
    };
    // Passive contact on a plane at rest cannot increase kinetic energy.
    // Gravity can raise sqrt(2E/m) no faster than |g|. This bounds the
    // distance any other sole corner can travel, including if contact persists.
    let speed = (2. * outgoing.energy() / MASS).sqrt() + 9.81 * horizon;
    let point_speed_bound = speed * (1. + radius * (MASS / INERTIA).sqrt());
    let post = 0.45 * gap / point_speed_bound;
    let stop_ns = ((time + post) * 1e9).floor() as u64;
    let dt = stop_ns as f64 * 1e-9 - time;
    if dt <= 1e-9 || dt * point_speed_bound >= gap || dt + time > horizon {
        return Err("no common positive one-corner interval on the nanosecond clock".into());
    }
    let accel = G + outgoing.w.cross(outgoing.w.cross(r));
    let mode = if accel.z > 0. {
        Mode::Opening
    } else if result.sticking {
        Mode::Sticking
    } else {
        Mode::Sliding
    };
    if matches!(mode, Mode::Opening) && accel.z - outgoing.w.length().powi(3) * radius * dt <= 0. {
        return Err("uncertain point release within comparison interval".into());
    }
    let (endpoint, reaction) = advance(outgoing, dt, corner, mu, mode, 32)?;
    let (coarse, coarse_reaction) = advance(outgoing, dt, corner, mu, mode, 8)?;
    let repeat = (endpoint.v - coarse.v)
        .length()
        .max((endpoint.w - coarse.w).length())
        .max((reaction - coarse_reaction).length());
    if repeat > 1e-10 {
        return Err("single-contact continuation did not converge".into());
    }
    let impulse = jump + reaction;
    if (endpoint.v - incoming.v - G * dt - impulse / MASS).length() > 1e-12 {
        return Err("physical reference momentum imbalance".into());
    }
    let wire = json!({"time_s":time,"common_cut_ns":stop_ns,"incoming":incoming.wire(),"outgoing":outgoing.wire(),"endpoint":endpoint.wire(),
        "first_corner":corner,"first_normal_tangent_impulse_ns":jump.to_array(),"continuous_support_impulse_ns":reaction.to_array(),"impulse_ns":impulse.to_array(),
        "incoming_point_velocity_m_s":relative.to_array(),"post_jump_point_velocity_m_s":outgoing.point_v(corner).to_array(),"sticking_at_impact":result.sticking,
        "continuation":format!("{mode:?}"),"other_corner_gap_at_impact_m":gap,"other_corner_clearance_bound_m":gap-point_speed_bound*dt,
        "one_corner_window_s":dt,"corner_speed_bound_m_s":point_speed_bound,"rk4_repeat":repeat,"energy_loss_j":result.dissipated_energy});
    Ok(Oracle {
        time,
        stop_ns,
        incoming,
        endpoint,
        impulse,
        corner,
        wire,
    })
}
