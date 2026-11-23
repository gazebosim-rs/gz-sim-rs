// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure how long a bound spatial point remains the only finite contact.
//! The event enclosure is numerical. Compare independently refined integrations
//! before using its time to judge a native result.
use crate::{
    FeatureLimit, FiniteBoxFlight, FiniteFeatureSample, FiniteFeatureTrack, FiniteLoadMode,
    FinitePointLoad, FinitePointSite, FlightBox, PhysicsError, SphericalPointMaterial,
};
use glam::{DQuat, DVec3, DVec4};
#[path = "finite_sliding_edge.rs"]
mod edge_slide;
#[path = "finite_event_search.rs"]
mod event_search;
#[path = "finite_slip_growth.rs"]
mod slip_growth;
#[path = "finite_slip_launch.rs"]
mod slip_launch;
#[path = "finite_vertex_span.rs"]
mod vertex;
pub use slip_growth::SlipGrowth;
pub use vertex::SlidingVertexSpan;
#[path = "finite_sticking_span.rs"]
mod sticking_span;
pub use sticking_span::StickingPointSpan;
#[path = "finite_point_transition.rs"]
mod transition;
pub use transition::{BoundPointImpact, BoundPointPath, BoundPointPiece};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FiniteArcEnd {
    Horizon,
    Feature(FeatureLimit),
    LoadMode,
    Unresolved,
}
#[derive(Debug, Clone, Copy)]
pub struct FiniteArcResult {
    pub end: FiniteArcEnd,
    /// Last admitted and first crossed integration time in seconds.
    pub bracket: [f64; 2],
    pub moving: FlightBox,
    pub center: DVec3,
    pub velocity: DVec3,
    pub angular_velocity: DVec3,
    pub site: FiniteFeatureSample,
    pub impulse: DVec3,
    pub angular_impulse: DVec3,
    /// Integral of the nonnegative normal force, and of that force times its moving normal.
    pub normal_impulse: f64,
    pub normal_impulse_vector: DVec3,
    pub friction_work: f64,
    pub energy_defect: f64,
    pub normal_velocity_defect: f64,
    /// Initial adjustment in the direction of the measured contact normal.
    pub initial_translation: f64,
}
#[derive(Clone, Copy)]
struct State {
    c: DVec3,
    q: DVec4,
    v: DVec3,
    w: DVec3,
    j: DVec3,
    normal: f64,
    normal_vector: DVec3,
    l: DVec3,
    work: f64,
}
impl State {
    fn add(self, d: Self, h: f64) -> Self {
        Self {
            c: self.c + h * d.c,
            q: self.q + h * d.q,
            v: self.v + h * d.v,
            w: self.w + h * d.w,
            j: self.j + h * d.j,
            normal: self.normal + h * d.normal,
            normal_vector: self.normal_vector + h * d.normal_vector,
            l: self.l + h * d.l,
            work: self.work + h * d.work,
        }
    }
}
#[derive(Debug, Clone, Copy)]
pub struct FiniteFeatureArc {
    flight: FiniteBoxFlight,
    track: FiniteFeatureTrack,
    material: SphericalPointMaterial,
    locked: bool,
    initial_slip_force: Option<FinitePointLoad>,
    sticking_impact_launch: bool,
    require_restart_precision: bool,
    implicit_slip: bool,
    scalar_slip: bool,
    local_center: DVec3,
    initial_shift: f64,
}
impl FiniteFeatureArc {
    /// Place the measured, isolated point exactly on the support plane.
    /// Caller provides the maximum displacement justified by its arrival bracket.
    pub fn new(
        f: FiniteBoxFlight,
        site: FinitePointSite,
        material: SphericalPointMaterial,
        stuck: bool,
        allowance: f64,
    ) -> Result<Option<Self>, PhysicsError> {
        if allowance < 0. || !allowance.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        let track = FiniteFeatureTrack::new(f, site)?;
        Self::with_track(f, site, track, material, stuck, allowance, false)
    }
    /// Start on the right-hand side of an already resolved line contact.
    /// Geometric admission verifies a positive opening rate at every zero margin.
    pub fn after_line_endpoint(
        f: FiniteBoxFlight,
        line: crate::FiniteContactSegment,
        loaded: usize,
        material: SphericalPointMaterial,
        stuck: bool,
        allowance: f64,
    ) -> Result<Option<Self>, PhysicsError> {
        if !allowance.is_finite() || allowance < 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let Some((track, s)) = FiniteFeatureTrack::outgoing_endpoint(f, line, loaded) else {
            return Ok(None);
        };
        Self::with_track(f, s.site, track, material, stuck, allowance, true)
    }
    fn with_track(
        f: FiniteBoxFlight,
        site: FinitePointSite,
        track: FiniteFeatureTrack,
        material: SphericalPointMaterial,
        stuck: bool,
        allowance: f64,
        right_hand: bool,
    ) -> Result<Option<Self>, PhysicsError> {
        let Some(s) = track.sample(
            f.moving,
            f.center_of_mass,
            f.relative_velocity,
            f.angular_velocity,
        ) else {
            return Ok(None);
        };
        if site.point.distance(s.site.point) > allowance
            || s.gap.abs() > allowance
            || (!right_hand && s.margins().iter().any(|m| m.distance <= 0.))
        {
            return Ok(None);
        }
        let Some(load) = f.first_point_load(s.site, material, stuck)? else {
            return Ok(None);
        };
        if !matches!(
            load.mode,
            FiniteLoadMode::Sliding | FiniteLoadMode::Sticking | FiniteLoadMode::IncipientSlip
        ) {
            return Ok(None);
        }
        let shift = -s.gap * s.site.normal;
        let flight = FiniteBoxFlight {
            moving: FlightBox {
                center: f.moving.center + shift,
                ..f.moving
            },
            center_of_mass: f.center_of_mass + shift,
            ..f
        };
        Ok(Some(Self {
            flight,
            track,
            material,
            locked: load.mode == FiniteLoadMode::Sticking,
            // The zero-slip acceleration law fixes the one-sided Coulomb
            // direction. Every later RK stage uses its evolving material slip.
            scalar_slip: false,
            implicit_slip: load.mode == FiniteLoadMode::IncipientSlip,
            initial_slip_force: (load.mode == FiniteLoadMode::IncipientSlip).then_some(load),
            sticking_impact_launch: right_hand
                && stuck
                && load.mode == FiniteLoadMode::IncipientSlip,
            require_restart_precision: false,
            local_center: f.moving.orientation.inverse() * (f.moving.center - f.center_of_mass),
            initial_shift: -s.gap,
        }))
    }
    fn start(self) -> State {
        let f = self.flight;
        State {
            c: f.center_of_mass,
            q: DVec4::from_array(f.moving.orientation.to_array()),
            v: f.relative_velocity,
            w: f.angular_velocity,
            j: DVec3::ZERO,
            normal: 0.,
            normal_vector: DVec3::ZERO,
            l: DVec3::ZERO,
            work: 0.,
        }
    }
    fn frame(self, y: State) -> Option<(FlightBox, FiniteFeatureSample)> {
        let q = DQuat::from_array(y.q.normalize().to_array());
        let moving = FlightBox {
            center: y.c + q * self.local_center,
            orientation: q,
            ..self.flight.moving
        };
        Some((moving, self.track.sample(moving, y.c, y.v, y.w)?))
    }
    fn force(self, y: State, moving: FlightBox, s: FiniteFeatureSample) -> Option<FinitePointLoad> {
        if y.j == DVec3::ZERO
            && let Some(load) = self.initial_slip_force
        {
            return Some(load);
        }
        let port = y.v + y.w.cross(s.site.point - y.c);
        // Remove off-manifold normal speed at intermediate RK stages. In
        // static contact the material port is instantaneously at rest. The
        // actual integrated speed and signed gap are reported independently.
        let defect = if self.locked {
            port
        } else {
            s.site.normal * port.dot(s.site.normal)
        };
        let state = FiniteBoxFlight {
            moving,
            center_of_mass: y.c,
            relative_velocity: y.v - defect,
            angular_velocity: y.w,
            ..self.flight
        };
        state
            .first_point_load(s.site, self.material, self.locked)
            .ok()
            .flatten()
    }
    fn derivative(self, y: State) -> Option<State> {
        let (moving, s) = self.frame(y)?;
        let load = self.force(y, moving, s)?;
        let f = load.force;
        let torque = (s.site.point - y.c).cross(f);
        let spin = DQuat::from_xyzw(y.w.x, y.w.y, y.w.z, 0.);
        let qdot = DVec4::from_array((spin * moving.orientation).to_array()) * 0.5;
        Some(State {
            c: y.v,
            q: qdot,
            v: self.flight.acceleration + f / self.material.mass,
            w: torque / self.material.inertia,
            j: f,
            normal: f.dot(s.site.normal),
            normal_vector: s.site.normal * f.dot(s.site.normal),
            l: torque,
            work: f.dot(load.relative_point_velocity),
        })
    }
    fn step(self, y: State, h: f64) -> Option<State> {
        if self.implicit_slip {
            return self.slip_launch_step(y, h);
        }
        let a = self.derivative(y)?;
        let b = self.derivative(y.add(a, h / 2.))?;
        let c = self.derivative(y.add(b, h / 2.))?;
        let d = self.derivative(y.add(c, h))?;
        let mut out = y
            .add(a, h / 6.)
            .add(b, h / 3.)
            .add(c, h / 3.)
            .add(d, h / 6.);
        out.q = out.q.normalize();
        Some(out)
    }
    fn boundary(self, y: State) -> FiniteArcEnd {
        let Some((m, s)) = self.frame(y) else {
            return FiniteArcEnd::Unresolved;
        };
        if let Some(limit) = s
            .margins()
            .iter()
            .filter(|v| v.distance <= 0.)
            .min_by(|a, b| a.distance.total_cmp(&b.distance))
        {
            return FiniteArcEnd::Feature(limit.limit);
        }
        let Some(load) = self.force(y, m, s) else {
            return FiniteArcEnd::Unresolved;
        };
        if load.mode
            != if self.locked {
                FiniteLoadMode::Sticking
            } else {
                FiniteLoadMode::Sliding
            }
        {
            FiniteArcEnd::LoadMode
        } else {
            FiniteArcEnd::Horizon
        }
    }
    fn finish(
        self,
        y: State,
        end: FiniteArcEnd,
        lo: f64,
        hi: f64,
    ) -> Result<FiniteArcResult, PhysicsError> {
        let (moving, s) = self.frame(y).ok_or(PhysicsError::InvalidValue)?;
        let a = self.start();
        let m = self.material;
        let ke =
            |p: State| 0.5 * (m.mass * p.v.length_squared() + m.inertia * p.w.length_squared());
        let balance = ke(y) - ke(a) - m.mass * self.flight.acceleration.dot(y.c - a.c) - y.work;
        let port = y.v + y.w.cross(s.site.point - y.c);
        Ok(FiniteArcResult {
            end,
            bracket: [lo, hi],
            moving,
            center: y.c,
            velocity: y.v,
            angular_velocity: y.w,
            site: s,
            impulse: y.j,
            angular_impulse: y.l,
            normal_impulse: y.normal,
            normal_impulse_vector: y.normal_vector,
            friction_work: y.work,
            energy_defect: balance,
            normal_velocity_defect: port.dot(s.site.normal),
            initial_translation: self.initial_shift,
        })
    }
    /// Integrate the bound arc on a fixed mesh and complete
    /// the first detected feature or force-mode transition by bisection. A
    /// finer mesh checks for missed events between samples. Smooth initially
    /// sliding or sticking arcs use RK4. A launch from zero slip uses implicit midpoint.
    pub fn measure(
        self,
        duration: f64,
        subdivisions: u32,
    ) -> Result<FiniteArcResult, PhysicsError> {
        self.measure_to(duration, subdivisions, 1e-13)
    }
    /// Choose the stopping width for local event bisection. This controls the
    /// time bracket only. Integration and floating-point geometry retain their
    /// own errors, which callers can check by refining the spatial trajectory.
    pub fn measure_to(
        self,
        duration: f64,
        subdivisions: u32,
        bracket_width: f64,
    ) -> Result<FiniteArcResult, PhysicsError> {
        self.measure_candidate(duration, subdivisions, bracket_width, None)
    }
    fn measure_candidate(
        self,
        duration: f64,
        subdivisions: u32,
        bracket_width: f64,
        limit: Option<FeatureLimit>,
    ) -> Result<FiniteArcResult, PhysicsError> {
        if !duration.is_finite()
            || duration < 0.
            || subdivisions == 0
            || !bracket_width.is_finite()
            || bracket_width <= 0.
        {
            return Err(PhysicsError::InvalidValue);
        }
        let mut y = self.start();
        let mut t = 0.;
        for i in 1..=subdivisions {
            let next = duration * f64::from(i) / f64::from(subdivisions);
            let h = next - t;
            let trial = self.step(y, h);
            let reason = trial.map_or(FiniteArcEnd::Unresolved, |z| self.boundary(z));
            if reason != FiniteArcEnd::Horizon {
                if let Some(limit) = limit
                    && reason == FiniteArcEnd::Feature(limit)
                    && let Some(last) = trial
                    && let Some((lo, hi, at)) =
                        self.monotone_bracket(y, h, bracket_width, limit, last)
                {
                    return self.finish(at, reason, t + lo, t + hi);
                }
                let (mut lo, mut hi) = (0., h);
                let mut at = y;
                let mut end = reason;
                for _ in 0..56 {
                    if hi - lo < bracket_width {
                        break;
                    }
                    let mid = (hi + lo) * 0.5;
                    let z = self.step(y, mid);
                    let r = z.map_or(FiniteArcEnd::Unresolved, |z| self.boundary(z));
                    if r == FiniteArcEnd::Horizon {
                        lo = mid;
                        at = z.unwrap()
                    } else {
                        hi = mid;
                        end = r
                    }
                }
                return self.finish(at, end, t + lo, t + hi);
            }
            y = trial.unwrap();
            t = next;
        }
        self.finish(y, FiniteArcEnd::Horizon, duration, duration)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SeparatingFeature;
    fn pivot(initial_pitch: f64) -> FiniteFeatureArc {
        let q = DQuat::from_rotation_y(initial_pitch) * DQuat::from_rotation_x(0.21);
        let h = DVec3::new(0.18, 0.10, 0.06);
        let local = h * (q.inverse() * -DVec3::Z).signum();
        let center = 0.17 * DVec3::Z;
        let w = DVec3::Y * 1.4;
        let moving = FlightBox {
            center: -(q * local),
            orientation: q,
            half_extent: h,
        };
        let support = FlightBox {
            center: -DVec3::Z,
            orientation: DQuat::IDENTITY,
            half_extent: DVec3::ONE,
        };
        let f = FiniteBoxFlight {
            moving,
            support,
            center_of_mass: center,
            relative_velocity: w.cross(center),
            angular_velocity: w,
            acceleration: -9.81 * DVec3::Z,
        };
        let site = FinitePointSite {
            feature: SeparatingFeature::SupportFace(2),
            point: DVec3::ZERO,
            normal: DVec3::Z,
        };
        FiniteFeatureArc::new(
            f,
            site,
            SphericalPointMaterial {
                mass: 2.,
                inertia: 0.05,
                friction: 1.2,
            },
            true,
            1e-12,
        )
        .unwrap()
        .unwrap()
    }
    // Integrate dθ / θ' from conservation of energy about a fixed material pivot.
    fn time(theta: f64) -> f64 {
        let n = 128;
        let omega2 = |a: f64| {
            1.4_f64.powi(2)
                + 2. * 2. * 9.81 * 0.17 * (1. - a.cos()) / (0.05 + 2. * 0.17_f64.powi(2))
        };
        let z = theta / f64::from(n);
        z / 3.
            * (0..=n)
                .map(|i| {
                    (if i == 0 || i == n {
                        1.
                    } else if i % 2 == 1 {
                        4.
                    } else {
                        2.
                    }) / omega2(z * f64::from(i)).sqrt()
                })
                .sum::<f64>()
    }
    #[test]
    fn incipient_slip_follows_the_one_sided_coulomb_direction_from_rest() {
        // A single vertex at the aggregate COM eliminates rotational coupling.
        // Oblique gravity above the static limit gives a constant sliding force.
        for heading in [-0.86, 0.41, 1.73] {
            let q = DQuat::from_rotation_y(0.32) * DQuat::from_rotation_x(-0.17);
            let half = DVec3::new(0.15, 0.07, 0.06);
            let arm = q * (half * (q.inverse() * -DVec3::Z).signum());
            let dir = DQuat::from_rotation_z(heading) * DVec3::X;
            let f = FiniteBoxFlight {
                moving: FlightBox {
                    center: -arm,
                    orientation: q,
                    half_extent: half,
                },
                support: FlightBox {
                    center: -0.5 * DVec3::Z,
                    orientation: DQuat::IDENTITY,
                    half_extent: DVec3::new(2., 2., 0.5),
                },
                center_of_mass: DVec3::ZERO,
                relative_velocity: DVec3::ZERO,
                angular_velocity: DVec3::ZERO,
                acceleration: 8. * dir - 9.81 * DVec3::Z,
            };
            let m = SphericalPointMaterial {
                mass: 2.,
                inertia: 0.04,
                friction: 0.3,
            };
            let site = FinitePointSite {
                feature: SeparatingFeature::SupportFace(2),
                normal: DVec3::Z,
                point: DVec3::ZERO,
            };
            assert_eq!(
                f.first_point_load(site, m, true).unwrap().unwrap().mode,
                FiniteLoadMode::IncipientSlip
            );
            let arc = FiniteFeatureArc::new(f, site, m, true, 1e-13)
                .unwrap()
                .unwrap();
            let t = 0.01;
            for n in [1, 4, 32] {
                let out = arc.measure(t, n).unwrap();
                assert_eq!(out.end, FiniteArcEnd::Horizon);
                let acc = (8. - 0.3 * 9.81) * dir;
                assert!(out.velocity.distance(t * acc) < 1e-13);
                assert!(out.center.distance(0.5 * t * t * acc) < 1e-14);
                assert!(
                    out.impulse
                        .distance(9.81 * m.mass * t * (DVec3::Z - 0.3 * dir))
                        < 1e-13
                );
                assert!(out.energy_defect.abs() < 1e-13 && out.friction_work < 0.);
            }
        }
    }
    #[test]
    fn spatial_vertex_follows_pendulum_until_the_sole_edge_becomes_incident() {
        let angle = 0.024;
        for s in [8, 32, 128] {
            let out = pivot(-angle).measure(0.025, s).unwrap();
            assert_eq!(
                out.end,
                FiniteArcEnd::Feature(FeatureLimit::MovingNeighbor(0))
            );
            assert!((out.bracket[0] - time(angle)).abs() < 2e-9);
            assert!(out.energy_defect.abs() < 2e-9);
            assert!(out.site.gap.abs() < 2e-11 && out.normal_velocity_defect.abs() < 2e-9);
            assert!(out.friction_work.abs() < 1e-12);
        }
    }
    #[test]
    fn static_port_work_and_world_momenta_close_before_the_second_contact() {
        let arc = pivot(-0.15);
        let t = time(0.0175);
        let out = arc.measure(t, 24).unwrap();
        assert_eq!(out.end, FiniteArcEnd::Horizon);
        let start = arc.start();
        let m = arc.material;
        let expected = DQuat::from_rotation_y(0.0175) * start.c;
        assert!(out.center.distance(expected) < 2e-12);
        assert!(
            (m.mass * (out.velocity - start.v - t * arc.flight.acceleration)).distance(out.impulse)
                < 2e-12
        );
        assert!(
            (m.inertia * (out.angular_velocity - start.w)).distance(out.angular_impulse) < 2e-13
        );
        assert!(out.energy_defect.abs() < 5e-12);
    }
}
