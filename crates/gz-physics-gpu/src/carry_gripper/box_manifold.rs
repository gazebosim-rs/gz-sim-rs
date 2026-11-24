// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Planar cuboid contacts. Clip an incident edge against a separating-axis reference face.
use crate::arithmetic::{Condition, Scalar};
type V<T> = [T; 2];
fn dot<T: Scalar>(a: V<T>, b: V<T>) -> T {
    a[0] * b[0] + a[1] * b[1]
}
fn add<T: Scalar>(a: V<T>, b: V<T>) -> V<T> {
    [a[0] + b[0], a[1] + b[1]]
}
fn sub<T: Scalar>(a: V<T>, b: V<T>) -> V<T> {
    [a[0] - b[0], a[1] - b[1]]
}
fn mul<T: Scalar>(a: V<T>, s: T) -> V<T> {
    [a[0] * s, a[1] * s]
}
fn choose<T: Scalar>(c: T::Condition, a: V<T>, b: V<T>) -> V<T> {
    [c.select(a[0], b[0]), c.select(a[1], b[1])]
}
fn abs<T: Scalar>(x: T, z: T) -> T {
    x.maximum(z - x)
}
#[derive(Clone, Copy)]
pub(super) struct Rectangle<T> {
    pub(super) center: V<T>,
    pub(super) x_axis: V<T>,
    pub(super) half: V<T>,
}
impl<T: Scalar> Rectangle<T> {
    fn axes(&self, z: T) -> [V<T>; 2] {
        [self.x_axis, [z - self.x_axis[1], self.x_axis[0]]]
    }
    fn radius(&self, n: V<T>, z: T) -> T {
        let [x, h] = self.axes(z);
        self.half[0] * abs(dot(x, n), z) + self.half[1] * abs(dot(h, n), z)
    }
    fn face(&self, n: V<T>, z: T, one: T) -> (V<T>, V<T>) {
        let axes = self.axes(z);
        let projections = axes.map(|a| dot(a, n));
        let along_x = abs(projections[0], z).at_least(abs(projections[1], z));
        let outward = choose(along_x, axes[0], axes[1]);
        let extent = along_x.select(self.half[0], self.half[1]);
        let direction = dot(outward, n).at_least(z).select(one, z - one);
        let tangent = choose(
            along_x,
            mul(axes[1], self.half[1]),
            mul(axes[0], self.half[0]),
        );
        (add(self.center, mul(outward, extent * direction)), tangent)
    }
}
/// A signed gap and a world-space contact witness on the payload.
#[derive(Clone, Copy)]
pub(super) struct Witness<T> {
    pub(super) gap: T,
    pub(super) normal: V<T>,
    pub(super) point: V<T>,
}
pub(super) fn floor<T: Scalar>(b: Rectangle<T>, height: T, z: T, one: T) -> [Witness<T>; 2] {
    let (center, tangent) = b.face([z, z - one], z, one);
    [sub(center, tangent), add(center, tangent)].map(|point| Witness {
        gap: point[1] - height,
        normal: [z, one],
        point,
    })
}
pub(super) fn rectangles<T: Scalar>(
    a: Rectangle<T>,
    b: Rectangle<T>,
    z: T,
    one: T,
) -> [Witness<T>; 2] {
    let delta = sub(b.center, a.center);
    let separation = |axis| abs(dot(delta, axis), z) - a.radius(axis, z) - b.radius(axis, z);
    let [ax, az] = a.axes(z);
    let [bx, bz] = b.axes(z);
    let mut axis = ax;
    let mut gap = separation(ax);
    let mut reference_b = z;
    for (candidate, owner) in [(az, z), (bx, one), (bz, one)] {
        let sep = separation(candidate);
        let wins = sep.greater(gap);
        reference_b = wins.select(owner, reference_b);
        axis = choose(wins, candidate, axis);
        gap = gap.maximum(sep);
    }
    let n = mul(axis, dot(delta, axis).at_least(z).select(one, z - one));
    let b_ref = reference_b.greater(z);
    let reference_normal = mul(n, b_ref.select(z - one, one));
    let (ac, at) = a.face(n, z, one);
    let (bc, bt) = b.face(mul(n, z - one), z, one);
    let reference = choose(b_ref, bc, ac);
    let reference_tangent = choose(b_ref, bt, at);
    let width = dot(reference_tangent, reference_tangent).root();
    let tangent = mul(reference_tangent, one / width);
    let incident = choose(b_ref, ac, bc);
    let edge = choose(b_ref, at, bt);
    let edge = mul(edge, dot(edge, tangent).at_least(z).select(one, z - one));
    let span = dot(edge, tangent);
    let middle = dot(sub(incident, reference), tangent);
    let lo = (middle - span).maximum(z - width);
    let hi = (middle + span).minimum(width);
    std::array::from_fn(|i| {
        let t = [lo, hi][i];
        let point = add(incident, mul(edge, (t - middle) / span));
        let depth = dot(sub(point, reference), reference_normal);
        let accepted = if i == 0 {
            hi.at_least(lo)
        } else {
            hi.greater(lo)
        };
        let gap = accepted.select(gap.maximum(depth), one);
        Witness {
            gap,
            normal: n,
            point: choose(b_ref, sub(point, mul(reference_normal, depth)), point),
        }
    })
}

#[cfg(test)]
#[path = "../../tests/private/box_manifold.rs"]
mod tests;
