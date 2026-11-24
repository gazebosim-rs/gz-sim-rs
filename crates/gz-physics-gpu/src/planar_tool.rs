// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! An inelastic round tool on a planar articulated arm, touching a horizontal floor.
use crate::arithmetic::{Condition, Scalar};
use crate::planar_arm::{ArmInertia, rotate_by, tick_with_velocity};
use crate::{GpuError, PlanarArmLane, PlanarJointStops, ResidentBatch, STEP, kernel, planar_stops};
use bytemuck::{Pod, Zeroable};

#[derive(Clone, Copy, Debug, Pod, Zeroable)]
#[repr(C)]
pub struct PlanarToolLane {
    pub arm: PlanarArmLane,
    /// Plane altitude relative to the fixed shoulder, m.
    pub ground_z: f64,
    pub radius: f64,
    pub friction: f64,
    /// Sum of impulses on the tool in the committed public tick, along +Z and +X (N s).
    pub normal_impulse: f64,
    pub tangent_impulse: f64,
    /// Estimated first geometric contact time in the public tick, s; -1 if unloaded.
    pub first_contact_offset: f64,
    /// Impulse at the first loaded internal step in that public tick, N s.
    pub first_normal_impulse: f64,
    pub first_tangent_impulse: f64,
    /// Signed complete turns about each initial principal-angle branch, initially zero.
    pub turns: [f64; 2],
    /// Per-axis velocity motor selection (0 or 1) and demand in rad/s.
    pub velocity_mode: [f64; 2],
    pub velocity_target: [f64; 2],
    pub stops: [PlanarJointStops; 2],
    /// Signed angular impulse on each hinge from its travel limit during this tick (N m s).
    pub stop_impulse: [f64; 2],
}
const WIDTH: usize = 62;
const _: () = assert!(size_of::<PlanarToolLane>() == 8 * WIDTH);
pub type PlanarToolBatch = ResidentBatch<PlanarToolLane>;
impl PlanarToolLane {
    /// Continuous shoulder and relative elbow angles, including full revolutions.
    pub fn joint_angles(&self) -> [f64; 2] {
        std::array::from_fn(|i| {
            let [c, s] = self.arm.angle[i];
            // Both signed zeros use +pi at the negative X branch cut.
            let s = if s == 0. { 0. } else { s };
            s.atan2(c) + std::f64::consts::TAU * self.turns[i]
        })
    }
    pub fn gap(&self) -> f64 {
        self.arm.points()[1][1] - self.radius - self.ground_z
    }
    fn valid(self) -> bool {
        self.arm.valid()
            && self.radius.is_finite()
            && self.radius > 0.
            && self.ground_z.is_finite()
            && self.gap().is_finite()
            && self.gap() >= 0.
            && self.friction.is_finite()
            && self.friction >= 0.
            && self.normal_impulse == 0.
            && self.tangent_impulse == 0.
            && self.first_contact_offset == -1.
            && self.first_normal_impulse == 0.
            && self.first_tangent_impulse == 0.
            && self.stop_impulse == [0.; 2]
            && self
                .stops
                .iter()
                .zip(self.arm.angle)
                .all(|(limit, angle)| limit.admits(angle))
            && self.turns == [0.; 2]
            && self.velocity_mode.iter().all(|v| matches!(*v, 0. | 1.))
            && self.velocity_target.iter().all(|v| v.is_finite())
    }
}
impl PlanarToolBatch {
    pub fn new(adapter_name: &str, initial: Vec<PlanarToolLane>) -> Result<Self, GpuError> {
        let shader = if initial.iter().any(|l| l.stops.iter().any(|s| s.enabled())) {
            module::<true>
        } else {
            module::<false>
        };
        Self::new_profile(adapter_name, initial, PlanarToolLane::valid, shader, None)
    }
    /// Change the demand of an enabled velocity motor at a public boundary.
    pub fn set_motor_speed(
        &mut self,
        lane: usize,
        axis: usize,
        speed: f64,
    ) -> Result<(), GpuError> {
        let row = self
            .committed
            .get_mut(lane)
            .ok_or(GpuError::InvalidArmTarget(lane))?;
        if !speed.is_finite() || row.velocity_mode.get(axis) != Some(&1.) {
            return Err(GpuError::InvalidArmTarget(lane));
        }
        if row.velocity_target[axis] != speed {
            row.velocity_target[axis] = speed;
            self.upload_needed = true;
        }
        Ok(())
    }
    pub fn set_target_angles(&mut self, lane: usize, radians: [f64; 2]) -> Result<(), GpuError> {
        crate::planar_arm::set_target(
            lane,
            self.committed.get_mut(lane).map(|l| &mut l.arm),
            radians,
        )?;
        self.upload_needed = true;
        Ok(())
    }
}
fn module<const BOUNDED: bool>() -> naga::Module {
    kernel::build::<WIDTH, 17>(&[STEP.as_secs_f64() / 2., 0., 1., 0.5], |v, c| {
        solve::<_, BOUNDED>(v, c)
    })
}
fn dot<T: Scalar>(a: [T; 2], b: [T; 2]) -> T {
    a[0] * b[0] + a[1] * b[1]
}
/// Project velocities in contact coordinates. Try sticking, then solve on the sliding cone edge.
fn impact<T: Scalar>([vn, vt]: [T; 2], [nn, nt, tt]: [T; 3], mu: T, z: T, one: T) -> [T; 2] {
    let determinant = nn * tt - nt * nt;
    let regular = determinant.greater(z);
    let denom = regular.select(determinant, one);
    let n = (nt * vt - tt * vn) / denom;
    let t = (nt * vn - nn * vt) / denom;
    let stick = regular
        .and(n.at_least(z))
        .and(t.at_least(z - mu * n))
        .and(t.at_most(mu * n));
    let residual = vt * nn - nt * vn;
    let direction = residual
        .greater(z)
        .select(z - mu, residual.less(z).select(mu, z));
    let effective = nn + nt * direction;
    let slide = (z - vn) / effective.greater(z).select(effective, one);
    let loaded = vn.less(z).and(nn.greater(z));
    [
        loaded.select(stick.select(n, slide), z),
        loaded.select(stick.select(t, direction * slide), z),
    ]
}
/// Every Cayley rotation is strictly less than half a turn. Count branch
/// crossings after each free rotation and each independent position correction.
pub(super) fn crossing<T: Scalar>([c, s]: [T; 2], [x, y]: [T; 2], z: T, one: T) -> T {
    let cross = c * y - s * x;
    s.at_least(z).and(y.less(z)).and(cross.greater(z)).select(
        one,
        s.less(z)
            .and(y.at_least(z))
            .and(cross.less(z))
            .select(z - one, z),
    )
}
/// Find a free-to-loaded boundary inside the public step. The prediction uses
/// exactly the same midpoint position update as the subsequent free advance.
/// The upper bound includes contact, leaving the rest of the tick for the
/// constrained motion. Settled contact and clear flight keep their two half steps.
fn impact_boundary<T: Scalar>(
    v: &[T; WIDTH],
    mass: &ArmInertia<T>,
    half_step: T,
    z: T,
    one: T,
    half: T,
) -> T {
    let start = [[v[0], v[1]], [v[2], v[3]]];
    let speed = [v[4], v[5]];
    let motor = Some(([v[38], v[39]], [v[40], v[41]]));
    let [a, b, _, _] = mass.dynamics(v, start, speed, motor, z);
    let gap = |[[c, s], [d, t]]: [[T; 2]; 2]| v[6] * s + v[7] * (s * d + c * t) - v[29] - v[28];
    let reaches = |dt: T| {
        let ht = half * dt;
        gap([
            rotate_by(v[0], v[1], ht * (v[4] + ht * a), one),
            rotate_by(v[2], v[3], ht * (v[5] + ht * b), one),
        ])
        .at_most(z)
    };
    let end = half_step + half_step;
    let middle_loaded = reaches(half_step);
    let crossed = v[31]
        .at_most(z)
        .and(gap(start).greater(z))
        .and(middle_loaded.or(reaches(end)));
    let mut lo = middle_loaded.select(z, half_step);
    let mut hi = middle_loaded.select(half_step, end);
    for _ in 0..7 {
        let mid = (lo + hi) * half;
        let hit = reaches(mid);
        hi = hit.select(mid, hi);
        lo = hit.select(lo, mid);
    }
    crossed.and(hi.less(end)).select(hi, half_step)
}

fn solve<T: Scalar, const BOUNDED: bool>(mut v: [T; WIDTH], constants: &[T]) -> [(usize, T); 17] {
    let [dt, z, one, half] = *constants else {
        unreachable!()
    };
    let mass = ArmInertia::from_lane(&v);
    let (floor, radius, mu) = (v[28], v[29], v[30]);
    let mut normal = z;
    let mut tangent = z;
    let mut stop_impulse = [z; 2];
    let locked = [v[50], v[59]];
    let mut first = z - one;
    let mut first_n = z;
    let mut first_t = z;
    let geometry = |v: &[T; WIDTH]| {
        let (c, s, d, t) = (v[0], v[1], v[2], v[3]);
        let x2 = v[7] * (c * d - s * t);
        let z2 = v[7] * (s * d + c * t);
        let x = v[6] * c + x2;
        let h = v[6] * s + z2;
        // A sphere rigidly welded to link two also rolls with omega1+omega2.
        (h - radius - floor, [x, x2], [radius - h, radius - z2])
    };
    let boundary = impact_boundary(&v, &mass, dt, z, one, half);
    for (offset, dt) in [(z, boundary), (boundary, dt + dt - boundary)] {
        let (start, _, _) = geometry(&v);
        let before = [[v[0], v[1]], [v[2], v[3]]];
        for (i, value) in tick_with_velocity(
            &v,
            &[dt, z, one, half],
            Some(([v[38], v[39]], [v[40], v[41]])),
        ) {
            v[i] = value;
        }
        for (i, from) in before.into_iter().enumerate() {
            v[36 + i] = v[36 + i] + crossing(from, [v[2 * i], v[2 * i + 1]], z, one);
        }
        let side = if BOUNDED {
            planar_stops::project(&mut v, &mass, z, one, half)
        } else {
            [z; 2]
        };
        let (gap, jn, jt) = geometry(&v);
        let in_contact = gap.at_most(z);
        let an = mass.inverse(v[2], v[3], jn);
        let at = mass.inverse(v[2], v[3], jt);
        let (nn, nt, tt) = (dot(jn, an), dot(jn, at), dot(jt, at));
        let w = [v[4], v[5]];
        let [n, t] = impact([dot(jn, w), dot(jt, w)], [nn, nt, tt], mu, z, one);
        let n = in_contact.select(n, z);
        let t = in_contact.select(t, z);
        let (next_w, [n, t], stop) = if BOUNDED {
            planar_stops::solve(
                &mass,
                v[2],
                v[3],
                w,
                side,
                locked,
                in_contact,
                jn,
                jt,
                mu,
                [n, t],
                z,
                one,
            )
        } else {
            (
                std::array::from_fn(|i| w[i] + an[i] * n + at[i] * t),
                [n, t],
                [z; 2],
            )
        };
        for i in 0..2 {
            v[4 + i] = next_w[i];
            stop_impulse[i] = stop_impulse[i] + stop[i];
        }
        let travel = start - gap;
        let fraction = (start / travel.greater(z).select(travel, one))
            .maximum(z)
            .minimum(one);
        let onset = first.less(z).and(n.greater(z));
        first = onset.select(offset + dt * fraction, first);
        first_n = onset.select(n, first_n);
        first_t = onset.select(t, first_t);
        normal = normal + n;
        tangent = tangent + t;
        // Split position correction in the inverse-mass normal direction. It adds no velocity.
        let correction = in_contact
            .and(nn.greater(z))
            .select((z - gap) / nn.greater(z).select(nn, one), z);
        let delta = an.map(|a| correction * a);
        let delta = if BOUNDED {
            planar_stops::floor_correction(delta, jn, (z - gap).maximum(z), side, locked, z, one)
        } else {
            delta
        };
        for (i, a) in delta.into_iter().enumerate() {
            let h = half * a;
            let co = (one - h * h) / (one + h * h);
            let si = (h + h) / (one + h * h);
            let (c, s) = (v[2 * i], v[2 * i + 1]);
            v[2 * i] = c * co - s * si;
            v[2 * i + 1] = s * co + c * si;
            v[36 + i] = v[36 + i] + crossing([c, s], [v[2 * i], v[2 * i + 1]], z, one);
        }
        if BOUNDED {
            planar_stops::snap(&mut v, z, one);
        }
    }
    [
        (0, v[0]),
        (1, v[1]),
        (2, v[2]),
        (3, v[3]),
        (4, v[4]),
        (5, v[5]),
        (26, v[26]),
        (27, v[27]),
        (31, normal),
        (32, tangent),
        (33, first),
        (34, first_n),
        (35, first_t),
        (36, v[36]),
        (37, v[37]),
        (60, stop_impulse[0]),
        (61, stop_impulse[1]),
    ]
}

#[cfg(test)]
#[path = "../tests/private/planar_tool.rs"]
mod tests;
