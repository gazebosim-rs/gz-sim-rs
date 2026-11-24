// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cubic Hermite translation in world axes, with integer knot times and analytical derivatives.
use std::{sync::Arc, time::Duration};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    pub time: Duration,
    /// World-axis displacement from the authored link position, metres.
    pub displacement: [f64; 3],
    /// World-axis tangent velocity, metres/second.
    pub velocity: [f64; 3],
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathEnd {
    Hold,
    Repeat,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TranslationSample {
    pub displacement: [f64; 3],
    pub velocity: [f64; 3],
    pub acceleration: [f64; 3],
}
#[derive(Clone, Debug)]
struct Segment {
    start: Waypoint,
    end: Waypoint,
    /// Quadratic Bernstein velocity control between the two endpoint tangents.
    middle: [f64; 3],
    seconds: f64,
}
fn mix(a: f64, b: f64, u: f64) -> f64 {
    (1. - u) * a + u * b
}
impl Segment {
    fn velocity(&self, u: f64) -> [f64; 3] {
        std::array::from_fn(|i| {
            mix(
                mix(self.start.velocity[i], self.middle[i], u),
                mix(self.middle[i], self.end.velocity[i], u),
                u,
            )
        })
    }
}
fn length(v: [f64; 3]) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}
/// Immutable C1 translation. Acceleration at an interior knot is the right-hand derivative.
/// Hold starts at zero final velocity. Repeat requires matching endpoint displacements and tangents.
#[derive(Clone, Debug)]
pub struct TranslationSpline {
    segments: Arc<[Segment]>,
    end: PathEnd,
    speed_bound: f64,
    acceleration_bound: f64,
    extent: [f64; 3],
}
impl TranslationSpline {
    pub fn new(points: &[Waypoint], end: PathEnd) -> Result<Self, &'static str> {
        let Some((first, last)) = points.first().zip(points.last()) else {
            return Err("translation needs at least two waypoints");
        };
        if points.len() < 2 || first.time != Duration::ZERO || first.displacement != [0.; 3] {
            return Err(
                "translation must start at zero time and displacement and have a successor",
            );
        }
        if points.iter().any(|p| {
            !p.displacement
                .into_iter()
                .chain(p.velocity)
                .all(f64::is_finite)
        }) {
            return Err("nonfinite translation waypoint");
        }
        match end {
            PathEnd::Hold if last.velocity != [0.; 3] => {
                return Err("held translation must end at zero velocity");
            }
            PathEnd::Repeat
                if first.displacement != last.displacement || first.velocity != last.velocity =>
            {
                return Err("repeating translation must close in position and velocity");
            }
            _ => (),
        }
        let mut segments = Vec::with_capacity(points.len() - 1);
        let (mut speed_bound, mut acceleration_bound) = (0f64, 0f64);
        let mut extent = [0f64; 3];
        for p in points.windows(2) {
            let (a, b) = (p[0], p[1]);
            if b.time <= a.time {
                return Err("translation waypoint times must increase");
            }
            let h = (b.time - a.time).as_secs_f64();
            let mut middle = [0.; 3];
            for i in 0..3 {
                let delta = b.displacement[i] - a.displacement[i];
                middle[i] = 3. * (delta / h) - a.velocity[i] - b.velocity[i];
                let bounds = [
                    a.displacement[i],
                    a.displacement[i] + (h / 3.) * a.velocity[i],
                    b.displacement[i] - (h / 3.) * b.velocity[i],
                    b.displacement[i],
                ];
                if !bounds.into_iter().all(f64::is_finite) || !middle[i].is_finite() {
                    return Err("unrepresentable translation span");
                }
                extent[i] = bounds.into_iter().map(f64::abs).fold(extent[i], f64::max);
            }
            let from = std::array::from_fn(|i| 2. * (middle[i] - a.velocity[i]) / h);
            let to = std::array::from_fn(|i| 2. * (b.velocity[i] - middle[i]) / h);
            for v in [a.velocity, middle, b.velocity] {
                speed_bound = speed_bound.max(length(v));
            }
            for v in [from, to] {
                if !v.into_iter().all(f64::is_finite) {
                    return Err("unrepresentable translation derivative");
                }
                acceleration_bound = acceleration_bound.max(length(v));
            }
            segments.push(Segment {
                start: a,
                end: b,
                middle,
                seconds: h,
            });
        }
        if !speed_bound.is_finite() || !acceleration_bound.is_finite() {
            return Err("unrepresentable translation bound");
        }
        Ok(Self {
            segments: segments.into(),
            end,
            speed_bound,
            acceleration_bound,
            extent,
        })
    }
    /// All immutable construction knots in increasing physical time.
    pub fn waypoints(&self) -> impl ExactSizeIterator<Item = Waypoint> + '_ {
        (0..self.segments.len() + 1).map(|i| {
            if i == self.segments.len() {
                self.segments[i - 1].end
            } else {
                self.segments[i].start
            }
        })
    }
    pub fn end_policy(&self) -> PathEnd {
        self.end
    }
    pub fn duration(&self) -> Duration {
        self.segments.last().expect("nonempty spline").end.time
    }
    /// Conservative Euclidean speed and acceleration bounds from the polynomial control hull.
    pub fn speed_bound(&self) -> f64 {
        self.speed_bound
    }
    pub fn acceleration_bound(&self) -> f64 {
        self.acceleration_bound
    }
    /// Bound acceleration over one smooth polynomial interval. At a knot on the
    /// right endpoint this uses the arriving polynomial's left-hand acceleration.
    /// A held tail has zero acceleration. An interval crossing a knot returns None.
    pub fn acceleration_bound_over(&self, at: Duration, dt: Duration) -> Option<f64> {
        let t = self.local(at);
        let Some(segment) = self
            .segments
            .get(self.segments.partition_point(|s| s.end.time <= t))
        else {
            return Some(0.);
        };
        if dt > segment.end.time - t {
            return None;
        }
        let elapsed = t - segment.start.time;
        let acceleration = |u: f64| {
            std::array::from_fn(|i| {
                let start = 2. * (segment.middle[i] - segment.start.velocity[i]) / segment.seconds;
                let end = 2. * (segment.end.velocity[i] - segment.middle[i]) / segment.seconds;
                mix(start, end, u)
            })
        };
        // Acceleration is affine. The norm is convex and is bounded by its two endpoints.
        let a = length(acceleration(elapsed.as_secs_f64() / segment.seconds));
        let b = length(acceleration((elapsed + dt).as_secs_f64() / segment.seconds));
        Some(a.max(b))
    }
    /// Per-axis maximum absolute displacement, enclosing the entire curve.
    pub fn extent(&self) -> [f64; 3] {
        self.extent
    }
    fn local(&self, at: Duration) -> Duration {
        let period = self.duration();
        if at < period {
            return at;
        }
        match self.end {
            PathEnd::Hold => period,
            PathEnd::Repeat => {
                let n = at.as_nanos() % period.as_nanos();
                Duration::new((n / 1_000_000_000) as u64, (n % 1_000_000_000) as u32)
            }
        }
    }
    /// Positive duration until the next polynomial boundary, including a repeat seam.
    pub fn until_knot(&self, at: Duration) -> Option<Duration> {
        let t = self.local(at);
        self.segments
            .get(self.segments.partition_point(|s| s.end.time <= t))
            .map(|s| s.end.time - t)
    }
    /// Average of the exact derivative on one polynomial span. Integrating the quadratic
    /// derivative avoids subtracting two nearly equal positions over a very short substep.
    pub fn mean_velocity(&self, at: Duration, dt: Duration) -> Option<[f64; 3]> {
        if dt.is_zero() {
            return None;
        }
        let t = self.local(at);
        let Some(s) = self
            .segments
            .get(self.segments.partition_point(|s| s.end.time <= t))
        else {
            return Some([0.; 3]);
        };
        if dt > s.end.time - t {
            return None;
        }
        let lo = (t - s.start.time).as_secs_f64() / s.seconds;
        let hi = ((t - s.start.time) + dt).as_secs_f64() / s.seconds;
        let (a, m, b) = (s.velocity(lo), s.velocity((lo + hi) * 0.5), s.velocity(hi));
        Some(std::array::from_fn(|i| {
            a[i] / 6. + m[i] * (2. / 3.) + b[i] / 6.
        }))
    }
    pub fn sample(&self, at: Duration) -> TranslationSample {
        let t = self.local(at);
        let Some(s) = self
            .segments
            .get(self.segments.partition_point(|s| s.end.time <= t))
        else {
            return TranslationSample {
                displacement: self.segments.last().unwrap().end.displacement,
                velocity: [0.; 3],
                acceleration: [0.; 3],
            };
        };
        let u = (t - s.start.time).as_secs_f64() / s.seconds;
        let mut out = TranslationSample {
            displacement: [0.; 3],
            velocity: s.velocity(u),
            acceleration: [0.; 3],
        };
        for i in 0..3 {
            let a = s.start.velocity[i];
            let m = s.middle[i];
            let b = s.end.velocity[i];
            out.acceleration[i] = mix(2. * (m - a) / s.seconds, 2. * (b - m) / s.seconds, u);
            let p = s.start.displacement[i];
            let q = s.end.displacement[i];
            let c = p + (s.seconds / 3.) * a;
            let d = q - (s.seconds / 3.) * b;
            let base = mix(c, d, u);
            out.displacement[i] = mix(mix(mix(p, c, u), base, u), mix(base, mix(d, q, u), u), u);
        }
        out
    }
}
