// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resultant and center of pressure at a cold, frictionless sole face. Native
//! generalized velocities are invariant to the diagonal corner-load null mode.
use super::*;
use gz_physics::{NormalCornerEquivalents, RectangularNormalContact, RectangularNormalResponse};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FacePressureReceipt {
    pub elapsed: Duration,
    /// Physical event time in seconds within the public interval. For a resolved
    /// internal transfer, elapsed is the first integer nanosecond at or after it.
    pub physical_elapsed_seconds: f64,
    pub foot: BodyId,
    pub support: BodyId,
    pub normal: [f64; 3],
    /// Ordered (−x,−y),(−x,+y),(+x,−y),(+x,+y) in the sole collider frame.
    pub points: [[f64; 3]; 4],
    pub gaps: [f64; 4],
    pub normal_impulse: f64,
    pub center_of_pressure: [f64; 3],
    pub normalized_center: [f64; 2],
    pub equivalent_corner_loads: NormalCornerEquivalents,
    pub before_normal: [f64; 4],
    pub after_normal: [f64; 4],
    pub kinetic_energy_change: f64,
    pub dissipated_energy: f64,
}
struct Face {
    boxes: RegisteredBoxes,
    points: [Vector; 4],
    gaps: [f64; 4],
    sites: [ArticulatedContactSite; 4],
    deck_velocity: [Vector; 4],
    normal: Vector,
}
impl Face {
    fn prepare(
        &self,
        s: &ArticulatedContactSample,
        dv: Vector,
    ) -> Result<Option<RectangularNormalResponse>, PhysicsError> {
        let n = self.normal;
        let mut matrix = [[0.; 4]; 4];
        for (i, row) in matrix.iter_mut().enumerate() {
            for (k, w) in row.iter_mut().enumerate() {
                let block = s.mobility(i, k).ok_or(PhysicsError::InvalidValue)?;
                *w = n.dot(Vector::from_array(
                    block.map(|r| Vector::from_array(r).dot(n)),
                ));
            }
        }
        let velocities = std::array::from_fn::<_, 4, _>(|i| {
            Vector::from(s.velocities[i]) - self.deck_velocity[i] + dv
        });
        let u = velocities.map(|v| v.dot(n));
        if !(0..4).any(|i| u[i] < -512. * f64::EPSILON * (1. + velocities[i].length())) {
            return Ok(None);
        }
        match (RectangularNormalContact {
            inverse_mass: matrix,
            free_normal: u,
        })
        .solve()
        {
            Ok(r) => Ok((r.resultant > 0. && r.normalized_center.is_some()).then_some(r)),
            Err(PhysicsError::Unsupported { .. } | PhysicsError::InvalidValue) => Ok(None),
            Err(e) => Err(e),
        }
    }
}
impl RapierBackend {
    fn normal_sole_face(
        &self,
        foot: BodyId,
        support: BodyId,
        displacement: Vector,
        allowance: f64,
    ) -> Result<Option<Face>, PhysicsError> {
        let Some(b) = self.registered_boxes(foot, support)? else {
            return Ok(None);
        };
        if b.friction != 0. {
            return Ok(None);
        }
        let n = b.pad_pose.rotation * Vector::Z;
        if (b.sole_pose.rotation * Vector::Z - n).length() > 512. * f64::EPSILON {
            return Ok(None);
        }
        let h = b.sole_half;
        let points = [(-h.x, -h.y), (-h.x, h.y), (h.x, -h.y), (h.x, h.y)]
            .map(|(x, y)| b.sole_pose * Vector::new(x, y, -h.z));
        let local = points.map(|p| b.pad_pose.inverse_transform_point(p + displacement));
        let gaps = local.map(|p| p.z - b.pad_half.z);
        let tol = 128.
            * f64::EPSILON
            * (1.
                + h.length()
                + b.pad_half.length()
                + b.sole_pose.translation.length()
                + b.pad_pose.translation.length());
        let clear = (1e-8 * (1. + h.length() + b.pad_half.length())).max(32. * (tol + allowance));
        if gaps
            .iter()
            .any(|g| g.abs() > tol + allowance || (*g - gaps[0]).abs() > tol)
            || local
                .iter()
                .any(|p| p.x.abs() + clear >= b.pad_half.x || p.y.abs() + clear >= b.pad_half.y)
            || 2. * h.z <= clear
        {
            return Ok(None);
        }
        let base = self.state(foot).ok_or(PhysicsError::MissingBody)?;
        let shoe = &self.world.bodies[b.foot];
        let deck = &self.world.bodies[b.pad];
        let prescribed = self.state(support).ok_or(PhysicsError::MissingBody)?;
        let sites = points.map(|p| ArticulatedContactSite {
            body: foot,
            local_point: (shoe.rotation().inverse() * (p - Vector::from(base.position))).to_array(),
        });
        let deck_velocity = points.map(|p| {
            Vector::from(prescribed.velocity)
                + Vector::from(prescribed.angular_velocity).cross(p - deck.center_of_mass())
        });
        Ok(Some(Face {
            boxes: b,
            points,
            gaps,
            sites,
            deck_velocity,
            normal: n,
        }))
    }
    pub(super) fn qualify_normal_sole(
        &self,
        foot: BodyId,
        pad: BodyId,
        displacement: Vector,
        dv: Vector,
        allowance: f64,
    ) -> Result<Option<(ColliderHandle, ColliderHandle)>, PhysicsError> {
        let Some(b) = self.registered_boxes(foot, pad)? else {
            return Ok(None);
        };
        if b.friction != 0. {
            return Ok(None);
        }
        let Some(face) = self.normal_sole_face(foot, pad, displacement, allowance)? else {
            return self.qualified_edge(
                foot,
                pad,
                displacement,
                dv,
                allowance,
                edge::EdgeMode::Any,
            );
        };
        let op = match self.frozen_contact_operator(
            &face.sites,
            Some(b.colliders),
            displacement != Vector::ZERO,
        ) {
            Ok(o) => o,
            Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(e) => return Err(e),
        };
        Ok(face.prepare(&op.sample, dv)?.map(|_| b.colliders))
    }
    pub fn frictionless_sole_closing(
        &self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<bool, PhysicsError> {
        Ok(self
            .qualify_normal_sole(foot, pad, Vector::ZERO, Vector::ZERO, 0.)?
            .is_some())
    }
    pub fn project_normal_face_impact(
        &mut self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<Option<FacePressureReceipt>, PhysicsError> {
        self.apply_normal_face_at(foot, pad, Duration::ZERO, 0.)
    }
    pub(crate) fn apply_normal_face_at(
        &mut self,
        foot: BodyId,
        pad: BodyId,
        elapsed: Duration,
        allowance: f64,
    ) -> Result<Option<FacePressureReceipt>, PhysicsError> {
        if foot == pad {
            return Err(PhysicsError::InvalidValue);
        }
        let Some(face) = self.normal_sole_face(foot, pad, Vector::ZERO, allowance)? else {
            return Ok(None);
        };
        let tx = match self.prepare_contact_sites(&face.sites, Some(face.boxes.colliders)) {
            Ok(t) => t,
            Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(e) => return Err(e),
        };
        let Some(r) = face.prepare(tx.sample(), Vector::ZERO)? else {
            return Ok(None);
        };
        let before = std::array::from_fn::<_, 4, _>(|k| {
            face.normal
                .dot(Vector::from(tx.sample().velocities[k]) - face.deck_velocity[k])
        });
        let [x, y] = r.normalized_center.expect("admitted nonzero pressure");
        let h = face.boxes.sole_half;
        let center = face.boxes.sole_pose * Vector::new(h.x * x, h.y * y, -h.z);
        let allocation = r.equivalents.base;
        let report = tx.commit(&allocation.map(|j| (j * face.normal).to_array()))?;
        let after = std::array::from_fn(|k| {
            face.normal
                .dot(Vector::from(report.after_velocities[k]) - face.deck_velocity[k])
        });
        let loss = -(0..4)
            .map(|k| 0.5 * allocation[k] * (before[k] + after[k]))
            .sum::<f64>();
        Ok(Some(FacePressureReceipt {
            elapsed,
            physical_elapsed_seconds: elapsed.as_secs_f64(),
            foot,
            support: pad,
            normal: face.normal.to_array(),
            points: face.points.map(|p| p.to_array()),
            gaps: face.gaps,
            normal_impulse: r.resultant,
            center_of_pressure: center.to_array(),
            normalized_center: [x, y],
            equivalent_corner_loads: r.equivalents,
            before_normal: before,
            after_normal: after,
            kinetic_energy_change: report.kinetic_energy_change,
            dissipated_energy: loss,
        }))
    }
    pub(crate) fn frictionless_registered(
        &self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<bool, PhysicsError> {
        Ok(self
            .registered_boxes(foot, pad)?
            .is_some_and(|b| b.friction == 0.))
    }
}
