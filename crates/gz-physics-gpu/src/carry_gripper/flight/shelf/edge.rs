// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two unilateral Newton contact rows acting on the same freely rotating body.
use super::*;
use crate::arithmetic::Condition;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ShelfWitness {
    /// World position on the shelf (m).
    pub point_m: [f64; 3],
    /// Non-attractive normal momentum delivered to the body at this witness (N s).
    pub normal_impulse: f64,
}

pub(super) fn well_conditioned(
    state: FreeBoxState,
    arms: [DVec3; 2],
    inertia: DVec3,
    mass: f64,
) -> bool {
    let q = DQuat::from_array(state.orientation);
    let rows = arms.map(|r| q.inverse() * ((q * r).cross(DVec3::Z)));
    let a = 1. / mass + rows[0].dot(rows[0] / inertia);
    let b = 1. / mass + rows[1].dot(rows[1] / inertia);
    let c = 1. / mass + rows[0].dot(rows[1] / inertia);
    let product = a * b;
    product.is_finite() && product - c * c > 1e-10 * product
}
pub(in crate::carry_gripper::flight) fn pair_impulses<T: Scalar>(
    diagonal: [T; 2],
    coupling: T,
    desired: [T; 2],
    zero: T,
    one: T,
) -> [T; 2] {
    let [a, b] = diagonal;
    let [u, v] = desired;
    let det = a * b - coupling * coupling;
    // The solo route supplies a coincident dummy row. Keep its unused solution finite.
    let safe = det.greater(zero).select(det, one);
    let x = (u * b - v * coupling) / safe;
    let y = (v * a - u * coupling) / safe;
    let both = x.at_least(zero).and(y.at_least(zero));
    let solo = (u / a).maximum(zero);
    let first = (coupling * solo).at_least(v);
    [
        both.select(x, first.select(solo, zero)),
        both.select(y, first.select(zero, (v / b).maximum(zero))),
    ]
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub(super) struct EdgeLane {
    point: EventLane,
    second_arm: [f64; 3],
    paired: f64,
}
pub(super) enum Solver {
    Point(ResidentBatch<EventLane>),
    Edge(ResidentBatch<EdgeLane>),
}
impl Solver {
    pub(super) fn new(f: &BoxFlight, initial: EventLane, edges: bool) -> Result<Self, GpuError> {
        if edges {
            let gpu = f.device.gpu.companion::<EdgeLane>(1)?;
            let lane = EdgeLane {
                point: initial,
                second_arm: [0.; 3],
                paired: 0.,
            };
            Ok(Self::Edge(ResidentBatch::on_device(
                gpu,
                vec![lane],
                module::<{ WORDS + 9 }, true>,
                None,
            )?))
        } else {
            let gpu = f.device.gpu.companion::<EventLane>(1)?;
            Ok(Self::Point(ResidentBatch::on_device(
                gpu,
                vec![initial],
                module::<{ WORDS + 5 }, false>,
                None,
            )?))
        }
    }
    pub(super) fn allocated(&self) -> u64 {
        match self {
            Self::Point(d) => d.device_info(),
            Self::Edge(d) => d.device_info(),
        }
        .allocated_buffer_bytes
    }
    pub(super) fn reset(&mut self) {
        match self {
            Self::Point(d) => d.reset(),
            Self::Edge(d) => d.reset(),
        }
    }
    pub(super) fn step(
        &mut self,
        tick: u64,
        point: EventLane,
        second: Option<DVec3>,
        check: impl FnOnce(Lane) -> Result<(), GpuError>,
    ) -> Result<(crate::StepMetrics, Lane), GpuError> {
        fn accept<T: Pod + Send + Sync>(
            device: &mut ResidentBatch<T>,
            tick: u64,
            input: T,
            extract: impl Fn(&T) -> Lane,
            check: impl FnOnce(Lane) -> Result<(), GpuError>,
        ) -> Result<(crate::StepMetrics, Lane), GpuError> {
            device.committed[0] = input;
            device.tick = tick;
            device.upload_needed = true;
            let metrics = device.advance_checked(1, |rows| check(extract(&rows[0])))?;
            Ok((metrics, extract(&device.committed[0])))
        }
        match self {
            Self::Point(d) => accept(d, tick, point, |l| l.free, check),
            Self::Edge(d) => accept(
                d,
                tick,
                EdgeLane {
                    point,
                    second_arm: second.unwrap_or_else(|| point.arm.into()).to_array(),
                    paired: if second.is_some() { 1. } else { 0. },
                },
                |l| l.point.free,
                check,
            ),
        }
    }
}
