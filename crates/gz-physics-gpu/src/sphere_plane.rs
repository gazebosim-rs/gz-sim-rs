// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Homogeneous, free solid spheres against independent stationary infinite planes.
use crate::arithmetic::{Condition, Scalar};
use crate::spatial::{V3, cross, dot, rotate};
use crate::{STEP, kernel::build};
use bytemuck::{Pod, Zeroable};
use naga::Module;

/// World-frame SI state for one sphere. The outward unit plane normal and
/// offset define the half-space dot(normal,x) >= plane_offset. Orientation is xyzw.
/// Normal impact is perfectly inelastic. Both impulses are on the sphere.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct SpherePlaneLane {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub orientation: [f64; 4],
    pub gravity: [f64; 3],
    pub plane_normal: [f64; 3],
    pub plane_offset: f64,
    pub radius: f64,
    pub inverse_mass: f64,
    pub friction: f64,
    pub normal_impulse: f64,
    pub tangential_impulse: [f64; 3],
}
const WIDTH: usize = 27;
const _: () = assert!(std::mem::size_of::<SpherePlaneLane>() == WIDTH * 8);

impl SpherePlaneLane {
    pub(crate) fn initial_valid(self) -> bool {
        let norm = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>();
        let distance = self
            .position
            .iter()
            .zip(self.plane_normal)
            .map(|(p, n)| p * n)
            .sum::<f64>()
            - self.plane_offset;
        let inv_inertia = 2.5 * self.inverse_mass / (self.radius * self.radius);
        // The kernel squares the contact slip and angular increment. Reject
        // initial states whose free-motion or surface-speed bound would overflow.
        let h = STEP.as_secs_f64();
        let free: [f64; 3] = std::array::from_fn(|i| self.velocity[i] + self.gravity[i] * h);
        let spin = norm(&self.angular_velocity).sqrt();
        let surface_speed = norm(&free).sqrt() + self.radius * spin;
        let eps = 16. * f64::EPSILON * (1. + self.plane_offset.abs() + self.radius);
        bytemuck::cast_ref::<Self, [f64; WIDTH]>(&self)
            .iter()
            .all(|v| v.is_finite())
            && (norm(&self.plane_normal) - 1.).abs() <= 2e-12
            && (norm(&self.orientation) - 1.).abs() <= 2e-12
            && self.radius > 0.
            && self.inverse_mass > 0.
            && self.inverse_mass.recip().is_finite()
            && inv_inertia.is_finite()
            && inv_inertia > 0.
            && (3.5 * self.inverse_mass).is_finite()
            && surface_speed.powi(2).is_finite()
            && self.friction >= 0.
            && distance.is_finite()
            && distance + eps >= self.radius
            && self.normal_impulse == 0.
            && self.tangential_impulse == [0.; 3]
    }
}
pub(super) const PARAMETERS: &[f64] = &[STEP.as_secs_f64(), 0., 1., 0.5, 2.5, 3.5];
pub(crate) fn module() -> Module {
    build::<WIDTH, 17>(PARAMETERS, |lane, c| tick(&lane, c))
}

/// The same motion and impulse graph runs for dense and resident observation.
pub(super) fn tick<T: Scalar>(l: &[T], c: &[T]) -> [(usize, T); 17] {
    let [h, zero, one, half, inertia_factor, tangent_factor] = *c else {
        unreachable!()
    };
    let v3 = |i: usize| [l[i], l[i + 1], l[i + 2]];
    let position = v3(0);
    let old = v3(3);
    let spin = v3(6);
    let q = [l[9], l[10], l[11], l[12]];
    let gravity = v3(13);
    let n = v3(16);
    let offset = l[19];
    let radius = l[20];
    let inv_m = l[21];
    let mu = l[22];
    let free: V3<T> = std::array::from_fn(|i| old[i] + h * gravity[i]);
    let vn = dot(free, n);
    let gap = dot(position, n) - offset - radius;
    let closing = vn.less(zero);
    let reaches = (gap + h * vn).at_most(zero);
    let hit = closing.and(reaches);
    let jn = hit.select((zero - vn) / inv_m, zero);
    let arm = n.map(|v| (zero - radius) * v);
    let rot_v = cross(spin, arm);
    let surface = std::array::from_fn(|i| free[i] + rot_v[i]);
    let normal_speed = dot(surface, n);
    let slip: V3<T> = std::array::from_fn(|i| surface[i] - normal_speed * n[i]);
    let slip_speed = dot(slip, slip).root();
    let safe_speed = slip_speed.greater(zero).select(slip_speed, one);
    // For I=2mr²/5 the tangential effective inverse mass is (7/2)/m.
    let scale = (one / (tangent_factor * inv_m)).minimum(mu * jn / safe_speed);
    let jt = slip.map(|v| (zero - v) * scale);
    let velocity: V3<T> = std::array::from_fn(|i| free[i] + inv_m * (jt[i] + n[i] * jn));
    let angular_impulse = cross(arm, jt);
    let inv_i = inertia_factor * inv_m / (radius * radius);
    let omega: V3<T> = std::array::from_fn(|i| spin[i] + inv_i * angular_impulse[i]);
    let safe_vn = closing.select(zero - vn, one);
    let until = hit.select((gap / safe_vn).maximum(zero).minimum(h), h);
    let remainder = h - until;
    let drift: V3<T> =
        std::array::from_fn(|i| position[i] + free[i] * until + velocity[i] * remainder);
    let depth = (radius + offset - dot(drift, n)).maximum(zero);
    let pos: V3<T> = std::array::from_fn(|i| drift[i] + hit.select(depth, zero) * n[i]);
    let orientation = rotate(rotate(q, spin, until, half), omega, remainder, half);
    [
        (0, pos[0]),
        (1, pos[1]),
        (2, pos[2]),
        (3, velocity[0]),
        (4, velocity[1]),
        (5, velocity[2]),
        (6, omega[0]),
        (7, omega[1]),
        (8, omega[2]),
        (9, orientation[0]),
        (10, orientation[1]),
        (11, orientation[2]),
        (12, orientation[3]),
        (23, jn),
        (24, jt[0]),
        (25, jt[1]),
        (26, jt[2]),
    ]
}
