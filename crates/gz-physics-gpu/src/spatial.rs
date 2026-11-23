// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Shared f64 vector and orientation operations for the spatial kernels.
use crate::arithmetic::Scalar;
pub(super) type V3<T> = [T; 3];
pub(super) fn dot<T: Scalar>(a: V3<T>, b: V3<T>) -> T {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub(super) fn cross<T: Scalar>(a: V3<T>, b: V3<T>) -> V3<T> {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub(super) fn rotate<T: Scalar>(q: [T; 4], w: V3<T>, dt: T, half: T) -> [T; 4] {
    // Normalized midpoint increment in world axes: [dt*omega/2,1] * q.
    let r = w.map(|x| x * (dt * half));
    let p = [q[0], q[1], q[2]];
    let product = cross(r, p);
    let mut next = [
        q[0] + r[0] * q[3] + product[0],
        q[1] + r[1] * q[3] + product[1],
        q[2] + r[2] * q[3] + product[2],
        q[3] - dot(r, p),
    ];
    let norm =
        (next[0] * next[0] + next[1] * next[1] + next[2] * next[2] + next[3] * next[3]).root();
    for v in &mut next {
        *v = *v / norm;
    }
    next
}
