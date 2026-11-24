// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional angular integration for the finite-sphere profile. The canonical
//! mixed solver and its checkpoint kernel fingerprint retain their original ABI.
use crate::{
    GpuError, SpherePairBatch, SpherePairLane,
    arithmetic::{Condition, Scalar},
    kernel::build,
    spatial::{V3, cross, dot},
    sphere_pair::{self, PARAMETERS},
};
use naga::Module;
/// Quaternion integration for a spinning sphere over each free-flight interval.
/// Both methods use the same continuous impact and angular impulse. Accurate
/// phase requires a physical timestep that resolves the angular speed.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SphereRotation {
    #[default]
    Midpoint,
    FourthOrder,
}

impl SpherePairBatch {
    /// Choose the attitude integrator when constructing this dense device batch.
    pub fn with_rotation(
        adapter_name: &str,
        initial: Vec<SpherePairLane>,
        rotation: SphereRotation,
    ) -> Result<Self, GpuError> {
        match rotation {
            SphereRotation::Midpoint => Self::new(adapter_name, initial),
            SphereRotation::FourthOrder => Self::new_profile(
                adapter_name,
                initial,
                SpherePairLane::initial_valid,
                fourth_module,
                None,
            ),
        }
    }
}
fn fourth_module() -> Module {
    const WIDTH: usize = std::mem::size_of::<SpherePairLane>() / 8;
    build::<WIDTH, 34>(PARAMETERS, |state, c| {
        tick_with_rotation(&state, c, SphereRotation::FourthOrder)
    })
}
pub(super) fn tick_with_rotation<T: Scalar>(
    state: &[T],
    c: &[T],
    rotation: SphereRotation,
) -> [(usize, T); 34] {
    let mut result = sphere_pair::tick(state, c);
    if rotation == SphereRotation::FourthOrder {
        let [h, zero, one, half, ..] = *c else {
            unreachable!()
        };
        let impact = result[33].1;
        let t = impact.at_least(zero).select(impact, h);
        for (input, output) in [(0, 0), (15, 13)] {
            let spin = std::array::from_fn(|i| state[input + 6 + i]);
            let next_spin = std::array::from_fn(|i| result[output + 6 + i].1);
            let q = std::array::from_fn(|i| state[input + 9 + i]);
            let at = rotate_fourth(q, spin, t, half, one);
            let next = rotate_fourth(at, next_spin, h - t, half, one);
            for (i, v) in next.into_iter().enumerate() {
                result[output + 9 + i].1 = v;
            }
        }
    }
    result
}

/// Fourth-order unit quaternion increment for constant world-frame angular
/// velocity. The angle of [r, 1 - |r|²/3] is 2 atan2(|r|, 1 - |r|²/3),
/// where r = dt omega / 2. Its cubic phase error vanishes. Rescale before
/// multiplying so finite high spin does not overflow the norm unnecessarily.
fn rotate_fourth<T: Scalar>(q: [T; 4], w: V3<T>, dt: T, half: T, one: T) -> [T; 4] {
    let r = w.map(|x| x * (dt * half));
    let third = one / (one + one + one);
    let z = dot(r, r) * third;
    let scale = one / z.maximum(one);
    let scalar = (one - z) * scale;
    let r = r.map(|v| v * scale);
    let p = [q[0], q[1], q[2]];
    let v = cross(r, p);
    let next = [
        scalar * q[0] + r[0] * q[3] + v[0],
        scalar * q[1] + r[1] * q[3] + v[1],
        scalar * q[2] + r[2] * q[3] + v[2],
        scalar * q[3] - dot(r, p),
    ];
    let length =
        (next[0] * next[0] + next[1] * next[1] + next[2] * next[2] + next[3] * next[3]).root();
    next.map(|a| a / length)
}
