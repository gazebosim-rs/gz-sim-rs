// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Certified interior corner arrivals of a uniformly translating passive tree.
use super::*;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ProjectionGeometry {
    Corner,
    Planar,
    SlidingEdge,
    MixedEdge,
    Edge,
    Finite,
    FiniteSegment,
    FrictionlessSole,
}
/// The sole and all its passive links share one free-flight trajectory
/// until this exact top-face crossing. No second body can intervene in flight.
#[derive(Clone, Copy)]
pub(crate) struct CornerFlight {
    pub foot: BodyId,
    pub support: BodyId,
    pub at: Duration,
    pub arrives: bool,
    pub geometry: ProjectionGeometry,
    pub allowance: f64,
    pub finite_witness: Option<gz_physics::FiniteContactWitness>,
    pub colliders: (ColliderHandle, ColliderHandle),
    pub(super) ballistic: Option<Ballistic>,
    pub(super) rigid: Option<super::finite::RigidFlight>,
}
#[derive(Clone, Copy)]
pub(super) struct Ballistic {
    owner: MultibodyIndex,
    root: RigidBodyHandle,
    pose: Pose,
    velocity: Vector,
    acceleration: Vector,
}
impl CornerFlight {
    /// Materialize the certified ballistic arc at every observable native
    /// boundary. Other bodies have already taken the original native solve.
    pub(crate) fn finish_free_step(self, backend: &mut RapierBackend, elapsed: Duration) {
        if let Some(r) = self.rigid {
            r.at(elapsed).commit(backend);
            return;
        }
        let Some(b) = self.ballistic else { return };
        let w = &mut backend.world;
        let tree = w
            .multibody_joints
            .get_multibody_mut(b.owner)
            .expect("certified free tree");
        let t = elapsed.as_secs_f64();
        let mut pose = b.pose;
        pose.translation += (b.velocity + 0.5 * t * b.acceleration) * t;
        let v = b.velocity + t * b.acceleration;
        w.bodies[b.root].set_position(pose, false);
        tree.forward_kinematics(&w.bodies, true);
        // Only the floating root translates. The relative coordinates of a
        // passive uniformly accelerated tree start at rest and stay at rest.
        let mut qd = tree.generalized_velocity_mut();
        qd.fill(0.);
        qd.rows_mut(0, 3).copy_from_slice(&v.to_array());
        for l in tree.links() {
            let rb = &mut w.bodies[l.rigid_body_handle()];
            rb.set_position(*l.local_to_world(), false);
            rb.set_linvel(v, false);
            rb.set_angvel(Vector::ZERO, false);
        }
        w.bodies
            .propagate_modified_body_positions_to_colliders(&mut w.colliders);
    }
}
impl RapierBackend {
    /// First certified approach to an interior corner during this public tick.
    pub fn corner_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        Ok(self
            .corner_flight(foot, pad, dt)?
            .filter(|f| f.arrives)
            .map(|f| f.at))
    }
    pub(crate) fn corner_flight(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<CornerFlight>, PhysicsError> {
        self.registered_flight(foot, pad, dt, ProjectionGeometry::Corner)
    }
    pub fn planar_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        Ok(self
            .registered_flight(foot, pad, dt, ProjectionGeometry::Planar)?
            .filter(|f| f.arrives)
            .map(|f| f.at))
    }
    /// First qualified pair of co-level cuboid vertices during shared free translation.
    pub fn sliding_edge_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        Ok(self
            .registered_flight(foot, pad, dt, ProjectionGeometry::SlidingEdge)?
            .filter(|f| f.arrives)
            .map(|f| f.at))
    }
    /// First qualified arrival with at least one sticking site on a finite box edge.
    pub fn mixed_edge_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        Ok(self
            .registered_flight(foot, pad, dt, ProjectionGeometry::MixedEdge)?
            .filter(|f| f.arrives)
            .map(|f| f.at))
    }
    pub fn frictionless_sole_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        Ok(self
            .registered_flight(foot, pad, dt, ProjectionGeometry::FrictionlessSole)?
            .filter(|f| f.arrives)
            .map(|f| f.at))
    }
    pub fn edge_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        Ok(self
            .registered_flight(foot, pad, dt, ProjectionGeometry::Edge)?
            .filter(|f| f.arrives)
            .map(|f| f.at))
    }
    pub(crate) fn registered_flight(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
        geometry: ProjectionGeometry,
    ) -> Result<Option<CornerFlight>, PhysicsError> {
        if geometry == ProjectionGeometry::FiniteSegment {
            return Ok(None);
        }
        if geometry == ProjectionGeometry::Finite {
            return self.finite_flight(foot, pad, dt, false, false);
        }
        if dt.is_zero() {
            return Ok(None);
        }
        let fh = self.bodies.get(&foot).ok_or(PhysicsError::MissingBody)?.0;
        let ph = self.bodies.get(&pad).ok_or(PhysicsError::MissingBody)?.0;
        let f = &self.world.bodies[fh];
        let d = &self.world.bodies[ph];
        if !f.is_dynamic()
            || !(d.is_fixed() || d.is_kinematic())
            || f.angvel() != Vector::ZERO
            || d.angvel() != Vector::ZERO
        {
            return Ok(None);
        }
        let ([a], [b]) = (f.colliders(), d.colliders()) else {
            return Ok(None);
        };
        let (ca, cb) = (&self.world.colliders[*a], &self.world.colliders[*b]);
        let (Some(sole), Some(deck)) = (ca.shape().as_cuboid(), cb.shape().as_cuboid()) else {
            return Ok(None);
        };
        let fp = *f.position() * ca.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?;
        let dp = *d.position() * cb.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?;
        let normal = dp.rotation * Vector::Z;
        let rel = f.linvel() - d.linvel();
        let closing = -rel.dot(normal);
        let acceleration = self.world.gravity * f.gravity_scale();
        let thickness = (fp.rotation.inverse() * normal)
            .abs()
            .dot(sole.half_extents);
        let top = dp.translation + deck.half_extents.z * normal;
        let gap = (fp.translation - top).dot(normal) - thickness;
        let approach = gz_physics::NormalApproach {
            gap,
            closing_speed: closing,
            closing_acceleration: -acceleration.dot(normal),
        };
        let (time, arrives) = match approach.touchdown_seconds(dt)? {
            Some(time) if time > 0. && closing + approach.closing_acceleration * time > 0. => {
                (time, true)
            }
            // With positive gap throughout, the entire native interval is
            // eligible for analytic translation. Selection also requires the
            // transverse footprint and a free passive tree below.
            None if gap > 0. && acceleration != Vector::ZERO => (dt.as_secs_f64(), false),
            _ => return Ok(None),
        };
        let mut at = if arrives {
            Duration::try_from_secs_f64(time).map_err(|_| PhysicsError::InvalidValue)?
        } else {
            dt
        };
        if arrives && at.as_secs_f64() > time {
            at = at.saturating_sub(Duration::from_nanos(1));
        }
        // A positive sub-nanosecond flight reaches the first representable
        // nonzero boundary. The signed gap is bounded on both sides.
        at = at.max(Duration::from_nanos(1));
        if at > dt {
            return Ok(None);
        }
        let t = at.as_secs_f64();
        let allowance = (closing.abs() + approach.closing_acceleration.abs() * time) * 1.000001e-9;
        let displacement = (rel + 0.5 * t * acceleration) * t;
        let projected = if arrives {
            displacement
        } else {
            displacement - normal * (gap + displacement.dot(normal))
        };
        let pair = match geometry {
            ProjectionGeometry::Finite | ProjectionGeometry::FiniteSegment => {
                unreachable!("handled by the finite box sweep")
            }
            ProjectionGeometry::Corner => self
                .corner_candidate(foot, pad, projected, allowance)?
                .map(|c| c.colliders),
            ProjectionGeometry::Planar => {
                self.patch_colliders(foot, pad, projected, acceleration * t, allowance)?
            }
            ProjectionGeometry::FrictionlessSole=>self.qualify_normal_sole(foot,pad,projected,acceleration*t,allowance)?,
            ProjectionGeometry::Edge => self.qualified_edge(
                foot,
                pad,
                projected,
                acceleration * t,
                allowance,
                edge::EdgeMode::Any,
            )?,
            ProjectionGeometry::MixedEdge => self.qualified_edge(
                foot,
                pad,
                projected,
                acceleration * t,
                allowance,
                edge::EdgeMode::Mixed,
            )?,
            ProjectionGeometry::SlidingEdge => self.qualified_edge(
                foot,
                pad,
                projected,
                acceleration * t,
                allowance,
                edge::EdgeMode::Sliding,
            )?,
        };
        let Some(colliders) = pair else {
            return Ok(None);
        };
        let translated = match self.sweep_with_support(&[foot], dt, Some(colliders)) {
            Ok(ArticulatedSweep::Translation { velocity }) => Vector::from_array(velocity),
            Ok(_) | Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(e) => return Err(e),
        };
        if translated != f.linvel() {
            return Ok(None);
        }
        let Some(link) = self.world.multibody_joints.rigid_body_link(fh) else {
            return Ok(None);
        };
        let tree = self
            .world
            .multibody_joints
            .get_multibody(link.multibody)
            .ok_or(PhysicsError::InvalidJoint)?;
        if !tree
            .links()
            .all(|l| self.world.bodies[l.rigid_body_handle()].is_dynamic())
        {
            return Ok(None);
        }
        let ballistic = if acceleration != Vector::ZERO {
            if tree.links().any(|l| {
                let rb = &self.world.bodies[l.rigid_body_handle()];
                rb.gravity_scale() != f.gravity_scale() || rb.linear_damping() != 0.
            }) || (!self.joints.construction_open()
                && tree.damping().iter().take(3).any(|r| *r != 0.))
                || tree.armature().iter().take(6).any(|r| *r != 0.)
            {
                return Ok(None);
            }
            let root = tree.root().rigid_body_handle();
            Some(Ballistic {
                owner: link.multibody,
                root,
                pose: *self.world.bodies[root].position(),
                velocity: translated,
                acceleration,
            })
        } else {
            if tree.links().any(|l| {
                self.world.gravity * self.world.bodies[l.rigid_body_handle()].gravity_scale()
                    != Vector::ZERO
            }) {
                return Ok(None);
            }
            None
        };
        // The remainder of the tree must remain above the supporting plane
        // throughout approach, including native speculative contact distance.
        let margin = self.world.integration_parameters.prediction_distance() + allowance;
        for member in tree.links() {
            let h = member.rigid_body_handle();
            let rb = &self.world.bodies[h];
            if rb.angvel() != Vector::ZERO {
                return Ok(None);
            }
            for cid in rb.colliders() {
                let c = &self.world.colliders[*cid];
                let pose =
                    *rb.position() * c.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?;
                if *cid != *a && crate::collision::can_exchange_impulses(c, cb, d.body_type()) {
                    let lowest =
                        c.shape().compute_aabb(&(dp.inverse() * pose)).mins.z - deck.half_extents.z;
                    if lowest
                        - closing.max(0.) * t
                        - 0.5 * approach.closing_acceleration.max(0.) * t * t
                        <= margin
                    {
                        return Ok(None);
                    }
                }
                let sweep = crate::approach_clearance::ApproachSweep {
                    pose,
                    velocity: translated,
                    dt: at,
                    margin: margin + acceleration.length() * t * t,
                };
                if !self.approach_neighbors_clear(c, sweep, |h| {
                    h == ph
                        || self
                            .world
                            .multibody_joints
                            .rigid_body_link(h)
                            .is_some_and(|l| l.multibody == link.multibody)
                })? {
                    return Ok(None);
                }
            }
        }
        Ok(Some(CornerFlight {
            finite_witness:None,
            foot,
            support: pad,
            at,
            arrives,
            allowance,
            colliders,
            geometry,
            ballistic,
            rigid: None,
        }))
    }
}
