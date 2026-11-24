// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two positive line bearings meeting through a bounded, subnanosecond face
//! event on one native clock. The interval is committed only as a whole.
mod face;
mod deck;
mod shared;
pub use shared::{SharedFaceReceipt,SharedSupportLoad,SharedSupportRequest};
pub use deck::DeckEdgeBearingReceipt;
pub use face::FaceBearingReceipt;
mod rocking;
use super::*;
use gz_physics::{FrictionlessSoleArc, SoleBearingSpan};
pub use rocking::RockingLimitReceipt;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FractionalBearingSpan {
    pub duration_seconds: f64,
    pub sole_vertices: [u8; 2],
    pub start_points: [[f64; 3]; 2],
    pub end_points: [[f64; 3]; 2],
    pub normal_impulses: [f64; 2],
    pub normal_force_bounds: [[f64; 2]; 2],
    pub angular_impulse_about_com: [f64; 3],
    pub integration_steps: u32,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoleTransferReceipt {
    pub foot: BodyId,
    pub support: BodyId,
    pub start: Duration,
    pub duration: Duration,
    /// Numerical geometric bracket in seconds from the start of this public tick.
    pub event_bracket_seconds: [f64; 2],
    pub before: FractionalBearingSpan,
    pub pressure: FacePressureReceipt,
    pub after: FractionalBearingSpan,
}
impl SoleTransferReceipt {
    pub fn integrated_normal_impulse(&self) -> f64 {
        self.before
            .normal_impulses
            .iter()
            .chain(&self.after.normal_impulses)
            .sum::<f64>()
            + self.pressure.normal_impulse
    }
}
fn vertex_bits(k: usize) -> u8 {
    (2 * (k & 1) + ((k >> 1) & 1) + 4 * (k >> 2)) as u8
}
#[derive(Clone, Copy)]
pub(super) struct Frame {
    pub boxes: RegisteredBoxes,
    pub owner: MultibodyIndex,
    pub root: RigidBodyHandle,
    pub old: Pose,
    pub com: Vector,
    pub deck_velocity: Vector,
    pub start: EdgeSupportState,
    pub mass: f64,
    pub inertia: rapier3d_f64::parry::math::Matrix,
    pub gravity: Vector,
    pub normal: Vector,
    pub top: Vector,
    pub tolerance: f64,
    pub foot: BodyId,
    pub pad: BodyId,
}
impl Frame {
    fn momentum(self, s: EdgeSupportState) -> Vector {
        s.orientation * (self.inertia * (s.orientation.inverse() * s.angular_velocity))
    }
    fn point(self, s: EdgeSupportState, r: Vector, seconds: f64) -> Vector {
        self.top + self.deck_velocity * seconds + s.center + s.orientation * r
    }
    fn span(self, p: SoleBearingSpan, r: [Vector; 4], seconds: f64) -> FractionalBearingSpan {
        FractionalBearingSpan {
            duration_seconds: p.duration_seconds,
            sole_vertices: p.vertices.map(vertex_bits),
            start_points: p
                .vertices
                .map(|k| self.point(p.start, r[k], seconds).to_array()),
            end_points: p.vertices.map(|k| {
                self.point(p.end.state, r[k], seconds + p.duration_seconds)
                    .to_array()
            }),
            normal_impulses: p.end.signed_impulse,
            normal_force_bounds: p.envelope.force_bounds,
            angular_impulse_about_com: (self.momentum(p.end.state) - self.momentum(p.start))
                .to_array(),
            integration_steps: p.integration_steps,
        }
    }
    fn bearing_vertices(self, r: [Vector; 4]) -> Option<[usize; 2]> {
        let mut ids = [0; 2];
        let mut n = 0;
        let s = self.start;
        for (k, r) in r.into_iter().enumerate() {
            let arm = s.orientation * r;
            let gap = self.normal.dot(s.center + arm);
            let normal = self.normal.dot(s.velocity + s.angular_velocity.cross(arm));
            if gap < -self.tolerance {
                return None;
            }
            if gap.abs() <= self.tolerance
                && normal.abs()
                    <= 512.
                        * f64::EPSILON
                        * (1. + s.velocity.length() + s.angular_velocity.length() * arm.length())
            {
                if n == 2 {
                    return None;
                }
                ids[n] = k;
                n += 1;
            }
        }
        (n == 2 && (ids[0] ^ ids[1]).count_ones() == 1).then_some(ids)
    }
    fn pad_clear(self, p: SoleBearingSpan, vertices: [Vector; 8], seconds: f64) -> bool {
        let b = self.boxes;
        let state = p.start;
        for (k, r) in vertices.into_iter().enumerate() {
            // The pure event solver checked the bottom rectangle continuously,
            // including the two points that exchange their contact status.
            if k >= 4 && p.envelope.directed_height(r)[0] <= self.tolerance {
                return false;
            }
            let point = self.point(state, r, seconds) - self.deck_velocity * seconds;
            let at = b.pad_pose.inverse_transform_point(point);
            let reach = self.tolerance
                + p.duration_seconds
                    * (p.envelope.center_speed_bound + p.envelope.angular_speed_bound * r.length());
            if at.x.abs() + reach >= b.pad_half.x || at.y.abs() + reach >= b.pad_half.y {
                return false;
            }
        }
        true
    }
    fn motion(self, target: EdgeSupportState, seconds: f64) -> RigidTreeMotion {
        let center = self.top + self.deck_velocity * seconds + target.center;
        let dq = target.orientation * self.start.orientation.inverse();
        RigidTreeMotion {
            owner: self.owner,
            root: self.root,
            root_pose: Pose::from_parts(
                center + dq * (self.old.translation - self.com),
                target.orientation,
            ),
            center,
            center_velocity: target.velocity + self.deck_velocity,
            angular: target.angular_velocity,
        }
    }
    pub(super) fn prepare(
        self,
        backend: &RapierBackend,
        t0: Duration,
        dt: Duration,
        modes: super::BearingModes<'_>,
    ) -> Result<Option<PreparedBearing>, PhysicsError> {
        let b = self.boxes;
        let n = self.normal;
        let q = self.start.orientation;
        let vertices = std::array::from_fn::<_, 8, _>(|i| {
            let h = b.sole_half;
            let sign = |bit: usize| if i & (1 << bit) == 0 { -1. } else { 1. };
            let p = b.sole_pose * Vector::new(sign(1) * h.x, sign(0) * h.y, sign(2) * h.z);
            q.inverse() * (p - self.com)
        });
        let corners = std::array::from_fn(|i| vertices[i]);
        if let Some(req)=modes.shared {return self.shared(backend,t0,dt,corners,req,modes.batch)}
        if modes.face
            && let Some(p) = self.face(backend, t0, dt, corners)?
        {
            return Ok(Some(p));
        }
        if modes.deck && let Some(p)=self.deck(backend,t0,dt,corners)? {return Ok(Some(p));}
        if !modes.edge {
            return Ok(None);
        }
        if modes.rocking
            && let Some(p) = self.rocking(backend, t0, dt, vertices)?
        {
            return Ok(Some(p));
        }
        let Some(ids) = self.bearing_vertices(corners) else {
            return Ok(None);
        };
        let law = FrictionlessSoleArc {
            mass: self.mass,
            inertia_body: self.inertia,
            corners,
            normal: n,
            plane_offset: 0.,
            gravity: self.gravity,
        };
        let seconds = dt.as_secs_f64();
        let Some(result) = law.advance(self.start, ids, seconds)? else {
            return Ok(None);
        };
        let first = result.first;
        if !self.pad_clear(first, vertices, 0.) {
            return Ok(None);
        }
        let mut center_bound = first.envelope.center_speed_bound;
        let mut spin_bound = first.envelope.angular_speed_bound;
        let pre = self.span(first, corners, 0.);
        let (receipt, sum) = if let Some((event, second)) = result.transfer {
            if !self.pad_clear(second, vertices, event.seconds) {
                return Ok(None);
            }
            center_bound = center_bound.max(second.envelope.center_speed_bound);
            spin_bound = spin_bound.max(second.envelope.angular_speed_bound);
            let after = self.span(second, corners, event.seconds);
            let points = corners.map(|r| self.point(event.before, r, event.seconds));
            let [x, y] = event
                .pressure
                .normalized_center
                .expect("positive transfer resultant");
            let cp = points[0]
                + (0.5 + 0.5 * x) * (points[2] - points[0])
                + (0.5 + 0.5 * y) * (points[1] - points[0]);
            let time = t0
                + Duration::from_nanos(
                    (event.seconds * 1e9).ceil().clamp(0., dt.as_nanos() as f64) as u64,
                );
            let ek = |s: EdgeSupportState| {
                0.5 * (self.mass * (s.velocity + self.deck_velocity).length_squared()
                    + s.angular_velocity.dot(self.momentum(s)))
            };
            let hit = FacePressureReceipt {
                foot: self.foot,
                support: self.pad,
                elapsed: time,
                physical_elapsed_seconds: t0.as_secs_f64() + event.seconds,
                normal: n.to_array(),
                points: points.map(|p| p.to_array()),
                gaps: points.map(|p| n.dot(p - self.top - self.deck_velocity * event.seconds)),
                normal_impulse: event.pressure.resultant,
                center_of_pressure: cp.to_array(),
                normalized_center: [x, y],
                equivalent_corner_loads: event.pressure.equivalents,
                before_normal: event.before_normal,
                after_normal: event.pressure.normal,
                kinetic_energy_change: ek(event.after) - ek(event.before),
                dissipated_energy: event.dissipated_energy,
            };
            let transfer = SoleTransferReceipt {
                foot: self.foot,
                support: self.pad,
                start: t0,
                duration: dt,
                event_bracket_seconds: event.numerical_bracket.map(|t| t + t0.as_secs_f64()),
                before: pre,
                pressure: hit,
                after,
            };
            let sum = transfer.integrated_normal_impulse();
            (BearingResult::Transfer(Box::new(transfer)), sum)
        } else {
            let sum = pre.normal_impulses.iter().sum();
            (
                BearingResult::Smooth(TwoBearingReceipt {
                    foot: self.foot,
                    support: self.pad,
                    start: t0,
                    duration: dt,
                    normal: n.to_array(),
                    sole_vertices: pre.sole_vertices,
                    start_points: pre.start_points,
                    end_points: pre.end_points,
                    normal_impulses: pre.normal_impulses,
                    normal_force_bounds: pre.normal_force_bounds,
                    angular_impulse_about_com: pre.angular_impulse_about_com,
                    integration_steps: pre.integration_steps,
                }),
                sum,
            )
        };
        let target = result.end();
        let expect =
            self.mass * n.dot(target.velocity - self.start.velocity - seconds * self.gravity);
        if (expect - sum).abs() > 1e-10 * (1. + sum.abs()) {
            return Ok(None);
        }
        let tree = backend
            .world
            .multibody_joints
            .get_multibody(self.owner)
            .ok_or(PhysicsError::InvalidJoint)?;
        if !backend.swept_locked_neighborhood(
            tree,
            self.owner,
            b,
            self.com,
            self.gravity,
            self.tolerance,
            center_bound,
            spin_bound,
            dt,
        )? {
            return Ok(None);
        }
        let motion = self.motion(target, seconds);
        Ok(Some(PreparedBearing {
            colliders: b.colliders,
            motion,
            receipt,
        }))
    }
}
