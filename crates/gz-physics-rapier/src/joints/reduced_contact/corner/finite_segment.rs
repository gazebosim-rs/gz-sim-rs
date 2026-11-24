// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cold finite edge/face contact. Only a certified single pressure-bearing
//! endpoint is committed, its opposite endpoint must be opening afterwards.
use super::*;
use gz_physics::{
    EdgeFaceFeature, FiniteContactSegment, SeparatingFeature, generalized_segment_impact,
    generalized_segment_work_impact,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteEndpointCertificate {
    pub feature: EdgeFaceFeature,
    pub loaded_end: usize,
    /// Outward-rounded pressure boxes used to rule out both ends staying
    /// closed. Zero means the direct opening-velocity bound was sufficient.
    pub work_boxes: u32,
    pub points: [[f64; 3]; 2],
    pub opposite_before: [f64; 3],
    pub opposite_after: [f64; 3],
    pub opening_lower_bound: f64,
}
impl RapierBackend {
    fn cold_segment(
        &self,
        foot: BodyId,
        pad: BodyId,
    ) -> Result<Option<(RegisteredBoxes, EdgeFaceFeature, FiniteContactSegment)>, PhysicsError>
    {
        let Some(boxes) = self.registered_boxes(foot, pad)? else {
            return Ok(None);
        };
        if self.world.bodies[boxes.pad].angvel() != Vector::ZERO {
            return Ok(None);
        }
        // Zero skin, zero restitution, and strictly incident rather than a
        // positive-gap speculative pair. The global query also clips both endpoints.
        let Some((feature, segment)) = boxes.flight_geometry().incident_segment_now(0.) else {
            return Ok(None);
        };
        Ok(Some((boxes, feature, segment)))
    }
    pub fn finite_segment_closing(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        let Some((b, _, s)) = self.cold_segment(foot, pad)? else {
            return Ok(false);
        };
        let Some((v, w)) = self.finite_incoming_twist(b.foot) else {
            return Ok(false);
        };
        let v = v - self.world.bodies[b.pad].linvel();
        let center = self.world.bodies[b.foot].center_of_mass();
        Ok(s.moving_points.iter().any(|p| {
            (v + w.cross(*p - center)).dot(s.normal) < -128. * f64::EPSILON * (1. + v.length())
        }))
    }
    pub(crate) fn apply_finite_segment(
        &mut self,
        foot: BodyId,
        pad: BodyId,
        elapsed: Duration,
        work: bool,
    ) -> Result<Option<FiniteImpactReceipt>, PhysicsError> {
        let Some((b, feature, segment)) = self.cold_segment(foot, pad)? else {
            return Ok(None);
        };
        let shoe = &self.world.bodies[b.foot];
        let origin =
            Vector::from_array(self.state(foot).ok_or(PhysicsError::MissingBody)?.position);
        let points = segment.moving_points;
        let sites = points.map(|p| ArticulatedContactSite {
            body: foot,
            local_point: (shoe.rotation().inverse() * (p - origin)).to_array(),
        });
        let deck = self.world.bodies[b.pad].linvel();
        let tx = match self.prepare_contact_sites(&sites, Some(b.colliders)) {
            Ok(tx) => tx,
            Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(e) => return Err(e),
        };
        let sample = tx.sample();
        let before = [0, 1].map(|i| Vector::from_array(sample.velocities[i]) - deck);
        let mobility = std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                sample
                    .mobility(i, j)
                    .expect("two prepared segment endpoints")
            })
        });
        let law = if work {
            generalized_segment_work_impact(
                mobility,
                before,
                segment.normal,
                points[1] - points[0],
                b.friction,
                128,
            )
        } else {
            generalized_segment_impact(mobility, before, segment.normal, b.friction)
                .map(|h| h.map(|x| (x, 0)))
        };
        let (hit, work_boxes) = match law {
            Ok(Some(s)) => s,
            Ok(None) | Err(PhysicsError::InvalidValue | PhysicsError::Unsupported { .. }) => {
                return Ok(None);
            }
            Err(e) => return Err(e),
        };
        let k = hit.loaded_end;
        let other = 1 - k;
        let mut impulses = [[0.; 3]; 2];
        impulses[k] = hit.impulse.to_array();
        let applied = tx.commit(&impulses)?;
        let after = [0, 1].map(|i| Vector::from_array(applied.after_velocities[i]) - deck);
        let face = match feature {
            EdgeFaceFeature::MovingEdge { support_face, .. } => {
                SeparatingFeature::SupportFace(support_face)
            }
            EdgeFaceFeature::SupportEdge { moving_face, .. } => {
                SeparatingFeature::MovingFace(moving_face)
            }
        };
        Ok(Some(FiniteImpactReceipt {
            feature: face,
            next_feature_boundary: None,
            segment: Some(FiniteEndpointCertificate {
                feature,
                loaded_end: k,
                work_boxes,
                points: points.map(|p| p.to_array()),
                opposite_before: before[other].to_array(),
                opposite_after: after[other].to_array(),
                opening_lower_bound: hit.opening_lower_bound,
            }),
            resultant: CornerImpactReceipt {
                foot,
                support: pad,
                elapsed,
                gap: segment.normal.dot(points[k] - segment.support_points[k]),
                point: points[k].to_array(),
                normal: segment.normal.to_array(),
                impulse_on_foot: impulses[k],
                before_relative_velocity: before[k].to_array(),
                after_relative_velocity: after[k].to_array(),
                friction: b.friction,
                sticking: hit.sticking,
                kinetic_energy_change: applied.kinetic_energy_change,
                dissipated_energy: hit.lost_energy,
            },
        }))
    }
}
