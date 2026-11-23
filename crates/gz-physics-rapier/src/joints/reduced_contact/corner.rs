// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Instantaneous impact at a uniquely identified cold cuboid corner.
mod normal_sole;
pub use normal_sole::FacePressureReceipt;
mod edge;
pub(crate) use edge::{BearingModes, BearingResult};
pub use edge::{
    DeckEdgeBearingReceipt, EdgeImpactReceipt, FaceBearingReceipt, FractionalBearingSpan,
    FrictionlessRecontactReceipt, MixedEdgeImpactReceipt, OpeningEdgeImpactReceipt,
    RockingLimitReceipt, SharedFaceReceipt, SharedSupportLoad, SharedSupportRequest,
    SlidingEdgeImpactReceipt, SoleTransferReceipt, TwoBearingReceipt,
};
mod finite;
pub use finite::FiniteImpactReceipt;
mod finite_segment;
pub use finite_segment::FiniteEndpointCertificate;
mod finite_path;
mod finite_vertex;
mod flight;
mod fractional_path;
mod patch;
mod sustained;
pub use finite_path::{BoundPointInterval, BoundTransferReceipt, InternalPointImpact};
pub use finite_vertex::PointSpanLoad;
pub(crate) use finite_vertex::{MaterialEndpoint, PointFollowModes};
pub use finite_vertex::{SlidingPointReceipt, SlidingVertexReceipt, StickingPointReceipt};
pub(crate) use fractional_path::PreparedFractional;
pub use fractional_path::{FractionalPointEvent, FractionalPointReceipt};
pub use sustained::SustainedEdgeReceipt;

use super::*;
pub(crate) use flight::{CornerFlight, ProjectionGeometry};
use gz_physics::PointImpact;
pub use patch::PlanarImpactReceipt;

/// One instantaneous reaction on an isolated articulated link. All vectors
/// are world-space, and work is evaluated in the inertial world frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CornerImpactReceipt {
    /// Elapsed actuator time at which the impulse was delivered.
    pub elapsed: Duration,
    /// Signed normal gap in metres at the application boundary.
    pub gap: f64,
    pub foot: BodyId,
    pub support: BodyId,
    pub point: [f64; 3],
    pub normal: [f64; 3],
    pub impulse_on_foot: [f64; 3],
    pub before_relative_velocity: [f64; 3],
    pub after_relative_velocity: [f64; 3],
    pub friction: f64,
    pub sticking: bool,
    pub kinetic_energy_change: f64,
    pub dissipated_energy: f64,
}
#[derive(Clone, Copy)]
struct RegisteredBoxes {
    foot: RigidBodyHandle,
    pad: RigidBodyHandle,
    colliders: (ColliderHandle, ColliderHandle),
    sole_pose: Pose,
    pad_pose: Pose,
    sole_half: Vector,
    pad_half: Vector,
    friction: f64,
}
struct Candidate {
    site: ArticulatedContactSite,
    colliders: (ColliderHandle, ColliderHandle),
    world_point: Vector,
    basis: [Vector; 3],
    deck_velocity: Vector,
    friction: f64,
    gap: f64,
}
impl RapierBackend {
    fn registered_boxes(
        &self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<Option<RegisteredBoxes>, PhysicsError> {
        let (f, d) = (
            self.bodies.get(&foot).ok_or(PhysicsError::MissingBody)?.0,
            self.bodies
                .get(&support)
                .ok_or(PhysicsError::MissingBody)?
                .0,
        );
        let shoe = &self.world.bodies[f];
        let deck = &self.world.bodies[d];
        if !shoe.is_dynamic()
            || !shoe.is_enabled()
            || !deck.is_enabled()
            || !(deck.is_fixed() || deck.is_kinematic())
            || self.world.multibody_joints.rigid_body_link(f).is_none()
            || self.world.multibody_joints.rigid_body_link(d).is_some()
        {
            return Ok(None);
        }
        let ([fh], [dh]) = (shoe.colliders(), deck.colliders()) else {
            return Ok(None);
        };
        let (a, b) = (&self.world.colliders[*fh], &self.world.colliders[*dh]);
        if !crate::collision::can_exchange_impulses(a, b, deck.body_type())
            || a.contact_skin() != 0.
            || b.contact_skin() != 0.
            || CoefficientCombineRule::combine(
                a.restitution(),
                b.restitution(),
                a.restitution_combine_rule(),
                b.restitution_combine_rule(),
            ) != 0.
        {
            return Ok(None);
        }
        let (Some(shape), Some(pad)) = (a.shape().as_cuboid(), b.shape().as_cuboid()) else {
            return Ok(None);
        };
        let h = shape.half_extents;
        let p = pad.half_extents;
        let fpose = *shoe.position() * a.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?;
        let dpose = *deck.position() * b.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?;
        Ok(Some(RegisteredBoxes {
            foot: f,
            pad: d,
            colliders: (*fh, *dh),
            sole_pose: fpose,
            pad_pose: dpose,
            sole_half: h,
            pad_half: p,
            friction: CoefficientCombineRule::combine(
                a.friction(),
                b.friction(),
                a.friction_combine_rule(),
                b.friction_combine_rule(),
            ),
        }))
    }
    fn corner_candidate(
        &self,
        foot: BodyId,
        support: BodyId,
        displacement: Vector,
        gap_allowance: f64,
    ) -> Result<Option<Candidate>, PhysicsError> {
        let Some(boxes) = self.registered_boxes(foot, support)? else {
            return Ok(None);
        };
        let (fpose, dpose, h, p) = (
            boxes.sole_pose,
            boxes.pad_pose,
            boxes.sole_half,
            boxes.pad_half,
        );
        let shoe = &self.world.bodies[boxes.foot];
        let deck = &self.world.bodies[boxes.pad];
        // Pad-frame clearances certify a unique vertex on the interior of the
        // rectangular top face. Reject grazing its edges or numerically
        // indistinguishable vertices before constructing a mobility matrix.
        let geometry_scale = 1. + h.max_element() + p.max_element();
        let tol = 64.
            * f64::EPSILON
            * (geometry_scale
                + fpose.translation.abs().max_element()
                + dpose.translation.abs().max_element());
        let tol = tol + gap_allowance;
        let separation = (1e-8 * geometry_scale).max(32. * tol);
        let mut point = None;
        for x in [-h.x, h.x] {
            for y in [-h.y, h.y] {
                for z in [-h.z, h.z] {
                    let world = fpose * Vector::new(x, y, z);
                    let q = dpose.inverse_transform_point(world + displacement);
                    let gap = q.z - p.z;
                    if gap > separation {
                        continue;
                    }
                    if gap.abs() > tol
                        || q.x.abs() + separation >= p.x
                        || q.y.abs() + separation >= p.y
                        || point.replace((world, gap)).is_some()
                    {
                        return Ok(None);
                    }
                }
            }
        }
        let Some((world_point, gap)) = point else {
            return Ok(None);
        };
        let basis = [Vector::X, Vector::Y, Vector::Z].map(|v| dpose.rotation * v);
        let origin =
            Vector::from_array(self.state(foot).ok_or(PhysicsError::MissingBody)?.position);
        let site = ArticulatedContactSite {
            body: foot,
            local_point: (shoe.rotation().inverse() * (world_point - origin)).to_array(),
        };
        let ds = self.state(support).ok_or(PhysicsError::MissingBody)?;
        let deck_velocity = Vector::from_array(ds.velocity)
            + Vector::from_array(ds.angular_velocity).cross(world_point - deck.center_of_mass());
        Ok(Some(Candidate {
            gap,
            site,
            colliders: boxes.colliders,
            world_point,
            basis,
            deck_velocity,
            friction: boxes.friction,
        }))
    }
    /// Cheap geometry filter for the exact, isolated corner projection. A
    /// later solve still checks the contact frame, incoming twist and all constraints.
    pub fn corner_at_support_face(
        &self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<bool, PhysicsError> {
        let Some(c) = self.corner_candidate(foot, support, Vector::ZERO, 0.)? else {
            return Ok(false);
        };
        let h = self.bodies[&foot].0;
        let s = self.state(foot).ok_or(PhysicsError::MissingBody)?;
        let relative = Vector::from_array(s.velocity)
            + Vector::from_array(s.angular_velocity)
                .cross(c.world_point - self.world.bodies[h].center_of_mass())
            - c.deck_velocity;
        Ok(relative.dot(c.basis[2]) < -128. * f64::EPSILON * (1. + relative.length()))
    }
    /// Project an isolated, closing, zero-restitution point impact on a
    /// registered pad. Require exact tangency and a cold passive articulation.
    /// Any other touching collider, active native contact, motor, or engaged
    /// limit leaves the original state for an ordinary simultaneous solve.
    pub fn project_corner_impact(
        &mut self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<Option<CornerImpactReceipt>, PhysicsError> {
        if foot == support {
            return Err(PhysicsError::InvalidValue);
        }
        if !self.bodies.contains_key(&foot) || !self.bodies.contains_key(&support) {
            return Err(PhysicsError::MissingBody);
        }
        match self.validate_contact_interval(Duration::from_nanos(1), 1, 1usize) {
            Ok(()) => self.apply_corner_at(foot, support, Duration::ZERO, 0.),
            Err(PhysicsError::Unsupported { .. }) => Ok(None),
            Err(e) => Err(e),
        }
    }

    pub(crate) fn apply_corner_at(
        &mut self,
        foot: BodyId,
        support: BodyId,
        elapsed: Duration,
        gap_allowance: f64,
    ) -> Result<Option<CornerImpactReceipt>, PhysicsError> {
        if foot == support {
            return Err(PhysicsError::InvalidValue);
        }
        let Some(candidate) = self.corner_candidate(foot, support, Vector::ZERO, gap_allowance)?
        else {
            return Ok(None);
        };
        self.apply_point_candidate(candidate, foot, support, elapsed)
    }
    fn apply_point_candidate(
        &mut self,
        candidate: Candidate,
        foot: BodyId,
        support: BodyId,
        elapsed: Duration,
    ) -> Result<Option<CornerImpactReceipt>, PhysicsError> {
        let tx = match self.prepare_contact_sites(&[candidate.site], Some(candidate.colliders)) {
            Ok(tx) => tx,
            Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(e) => return Err(e),
        };
        let b = candidate.basis;
        let m = tx
            .sample()
            .mobility(0, 0)
            .ok_or(PhysicsError::InvalidValue)?
            .map(Vector::from_array);
        let inv = std::array::from_fn(|i| {
            std::array::from_fn(|j| b[i].dot(Vector::from_array(m.map(|row| row.dot(b[j])))))
        });
        let incoming = Vector::from_array(tx.sample().velocities[0]) - candidate.deck_velocity;
        if incoming.dot(b[2]) >= -128. * f64::EPSILON * (1. + incoming.length()) {
            return Ok(None);
        }
        let law = PointImpact {
            inverse_mass: inv,
            relative_velocity: b.map(|axis| axis.dot(incoming)),
            friction: candidate.friction,
        };
        let solution = match law.solve() {
            Ok(v) => v,
            Err(PhysicsError::Unsupported { .. } | PhysicsError::InvalidValue) => return Ok(None),
            Err(e) => return Err(e),
        };
        let [x, y, z] = solution.impulse;
        let impulse = x * b[0] + y * b[1] + z * b[2];
        let applied = tx.commit(&[impulse.to_array()])?;
        let after = Vector::from_array(applied.after_velocities[0]) - candidate.deck_velocity;
        Ok(Some(CornerImpactReceipt {
            elapsed,
            gap: candidate.gap,
            foot,
            support,
            point: candidate.world_point.to_array(),
            normal: b[2].to_array(),
            impulse_on_foot: impulse.to_array(),
            before_relative_velocity: incoming.to_array(),
            after_relative_velocity: after.to_array(),
            friction: candidate.friction,
            sticking: solution.sticking,
            kinetic_energy_change: applied.kinetic_energy_change,
            dissipated_energy: solution.dissipated_energy,
        }))
    }
}
