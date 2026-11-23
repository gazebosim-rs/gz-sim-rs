// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded piecewise-linear sweeps for prescribed one-way boundary translations.
use super::{Boundary, PreparedBoundary, XpbdError};
use glam::DVec3;
use std::{collections::HashSet, time::Duration};

/// Offsets are relative to the start of the current public physics interval.
pub trait BoundaryPath {
    /// Absolute world center of the collision geometry.
    fn center(&self, offset: Duration) -> [f64; 3];
    /// Mean world velocity on one smooth segment.
    fn mean_velocity(&self, offset: Duration, dt: Duration) -> [f64; 3];
    /// Upper bound on the norm of acceleration up to the next knot, in m/s².
    fn acceleration_bound(&self, offset: Duration) -> f64;
    /// Bound on the requested smooth prefix. Existing path implementations may use their
    /// whole-polynomial bound. The caller clips the span at every participating knot first.
    fn acceleration_bound_over(&self, offset: Duration, _span: Duration) -> f64 {
        self.acceleration_bound(offset)
    }
    /// Positive duration to the next change of polynomial, or None on an unbroken tail.
    fn until_knot(&self, offset: Duration) -> Option<Duration>;
}
pub const DEFAULT_PATH_TOLERANCE: f64 = 1e-6;
const MAX_SLICES: usize = 16_384;
const MAX_SEGMENTS: usize = 262_144;

struct Segment {
    start: DVec3,
    end: DVec3,
    velocity: DVec3,
}
pub(super) struct Schedule {
    ends: Vec<u64>,
    indices: Vec<usize>,
    segments: Vec<Segment>,
}
impl Schedule {
    pub fn build(
        dt: Duration,
        structural: u32,
        boundaries: &[Boundary],
        paths: &[(usize, &dyn BoundaryPath)],
        tolerance: f64,
    ) -> Result<Self, XpbdError> {
        if !tolerance.is_finite()
            || tolerance <= 0.
            || structural == 0
            || paths.is_empty()
            || paths.len() > MAX_SEGMENTS
        {
            return Err(XpbdError::InvalidBoundary);
        }
        let ns = u64::try_from(dt.as_nanos()).map_err(|_| XpbdError::InvalidStep)?;
        let mut unique = HashSet::new();
        for &(i, path) in paths {
            let Some(Boundary::Sphere { center, .. } | Boundary::Capsule { center, .. }) =
                boundaries.get(i)
            else {
                return Err(XpbdError::InvalidBoundary);
            };
            let x = DVec3::from(path.center(Duration::ZERO));
            if !unique.insert(i) || !x.is_finite() || x != DVec3::from(*center) {
                return Err(XpbdError::InvalidBoundary);
            }
        }
        let count = u64::from(structural);
        if ns != 0 && (ns < count || count as usize > MAX_SLICES) {
            return Err(XpbdError::InvalidStep);
        }
        let mut schedule = Self {
            ends: Vec::new(),
            indices: paths.iter().map(|r| r.0).collect(),
            segments: Vec::new(),
        };
        if ns == 0 {
            return Ok(schedule);
        }
        let mut start = 0;
        for i in 1..=count {
            // Match the standard XPBD structural grid, including its integer remainder order.
            let deadline = i * (ns / count) + i.min(ns % count);
            while start < deadline {
                if schedule.ends.len() >= MAX_SLICES
                    || schedule.segments.len() > MAX_SEGMENTS - paths.len()
                {
                    return Err(XpbdError::InvalidStep);
                }
                let at = Duration::from_nanos(start);
                let mut span = deadline - start;
                for &(_, path) in paths {
                    if let Some(knot) = path.until_knot(at) {
                        span = span.min(u64::try_from(knot.as_nanos()).unwrap_or(u64::MAX));
                    }
                }
                let smooth = Duration::from_nanos(span);
                for &(_, path) in paths {
                    let a = path.acceleration_bound_over(at, smooth);
                    if !a.is_finite() || a < 0. {
                        return Err(XpbdError::InvalidBoundary);
                    }
                    if a > 0. {
                        // The deviation from the endpoint chord is bounded by ||a|| h² / 8.
                        span = span.min((1e9 * (8. * tolerance / a).sqrt()).floor() as u64);
                    }
                }
                if span == 0 {
                    return Err(XpbdError::InvalidStep);
                }
                let end = start + span;
                for &(_, path) in paths {
                    let x = DVec3::from(path.center(at));
                    let y = DVec3::from(path.center(Duration::from_nanos(end)));
                    let v = DVec3::from(path.mean_velocity(at, Duration::from_nanos(span)));
                    if !x.is_finite() || !y.is_finite() || !v.is_finite() {
                        return Err(XpbdError::InvalidBoundary);
                    }
                    schedule.segments.push(Segment {
                        start: x,
                        end: y,
                        velocity: v,
                    });
                }
                schedule.ends.push(end);
                start = end;
            }
        }
        Ok(schedule)
    }
    pub fn len(&self) -> usize {
        self.ends.len()
    }
    pub fn apply(&self, index: usize, prepared: &mut [PreparedBoundary]) -> u128 {
        let width = self.indices.len();
        for (&boundary, part) in self
            .indices
            .iter()
            .zip(&self.segments[index * width..(index + 1) * width])
        {
            let b = &mut prepared[boundary];
            b.path_endpoints = Some((part.start, part.end));
            b.velocity = part.velocity;
        }
        u128::from(self.ends[index])
    }
}
