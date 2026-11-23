// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BoundDeckEdge, DeckEdgeSample, FrictionlessFaceSupport};
/// A sole tips about a finite pad-owned edge. Scalar normal impulse and signed
/// vector impulse differ because the force direction follows the rotating sole.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeckEdgeBearingReceipt {
    pub foot: BodyId,
    pub support: BodyId,
    pub start: Duration,
    pub duration: Duration,
    /// Pad side order is −x, +x, −y, +y.
    pub pad_edge: u8,
    /// Seconds relative to the start of this native span.
    pub face_exit_bracket_seconds: Option<[f64; 2]>,
    pub face_seconds: f64,
    pub edge_seconds: f64,
    pub face_normal_impulse: f64,
    pub edge_normal_impulse: f64,
    pub impulse_on_foot: [f64; 3],
    pub angular_impulse_about_com: [f64; 3],
    pub start_edge_pressure_world: [f64; 3],
    pub end_edge_pressure_world: [f64; 3],
    pub end_normal: [f64; 3],
    pub end_normal_force: f64,
    /// Equivalent reactions at the moving intersection segment's endpoints.
    pub end_equivalent_force: [f64; 2],
    pub normal_force_bounds: [f64; 2],
    pub integrated_substeps: u32,
    pub certified_time_domains: u32,
}
impl DeckEdgeBearingReceipt {
    pub fn normal_impulse(&self) -> f64 {
        self.face_normal_impulse + self.edge_normal_impulse
    }
}
fn integrate(line: BoundDeckEdge, h: f64) -> Result<Option<(DeckEdgeSample, u32)>, PhysicsError> {
    if h == 0. {
        return Ok(Some((line.sample(0., 1.)?, 0)));
    }
    let mut count = (h / 0.00025).ceil().max(1.) as u32;
    if count > 32 {
        return Ok(None);
    }
    let mut coarse = line.sample(h, h / f64::from(count))?;
    while count < 64 {
        count *= 2;
        let fine = line.sample(h, h / f64::from(count))?;
        let near = |x: Vector, y: Vector| x.distance(y) <= 2e-12 * (1. + x.length() + y.length());
        let a = coarse.state;
        let b = fine.state;
        if near(a.center, b.center)
            && near(a.velocity, b.velocity)
            && near(a.angular_velocity, b.angular_velocity)
            && a.orientation
                .to_array()
                .into_iter()
                .zip(b.orientation.to_array())
                .all(|(x, y)| (x - y).abs() < 2e-12)
            && (coarse.normal_impulse - fine.normal_impulse).abs()
                < 2e-12 * (1. + fine.normal_impulse.abs())
        {
            return Ok(Some((fine, count)));
        }
        coarse = fine;
    }
    Ok(None)
}
impl Frame {
    pub(super) fn deck(
        self,
        backend: &RapierBackend,
        t0: Duration,
        dt: Duration,
        corners: [Vector; 4],
    ) -> Result<Option<PreparedBearing>, PhysicsError> {
        let seconds = dt.as_secs_f64();
        let axes = [Vector::X, Vector::Y].map(|a| self.boxes.pad_pose.rotation * a);
        let model = FrictionlessFaceSupport {
            mass: self.mass,
            inertia_body: self.inertia,
            corners,
            pad_axes: axes,
            pad_half: self.boxes.pad_half.truncate(),
            gravity: self.gravity,
        };
        let mut face_seconds = 0.;
        let mut bracket = None;
        let mut face_normal_impulse = 0.;
        let mut angular = Vector::ZERO;
        let (edge, line, checked) = if let Some(face) = model.bind(self.start)? {
            let Some((change, line)) = face.departing_deck_edge(seconds)? else {
                return Ok(None);
            };
            let mid = face.sample(change.seconds)?;
            face_seconds = change.seconds;
            bracket = Some(change.numerical_bracket_seconds);
            face_normal_impulse = mid.normal_impulse;
            angular = mid.angular_impulse_about_com;
            (change.pad_edges.trailing_zeros() as usize, line, None)
        } else {
            // A continuing tilted face must contain the complete physical line
            // of the pad, carry positive force, and tip towards its exterior.
            // Near co-level tangency several pad lines can be geometrically
            // coincident. Only a unique certified unilateral mode is admissible.
            let mut active = None;
            for id in 0..4 {
                if let Some(line) = model.bind_deck_edge(self.start, id)?
                    && let Some(checked) = line.enclose(seconds)?
                {
                    if active.is_some() {
                        return Ok(None);
                    }
                    active = Some((id, line, Some(checked)));
                }
            }
            let Some(v) = active else { return Ok(None) };
            v
        };
        let edge_seconds = (seconds - face_seconds).max(0.);
        let envelope = match checked {
            Some(v) => v,
            None => {
                let Some(v) = line.enclose(edge_seconds)? else {
                    return Ok(None);
                };
                v
            }
        };
        let Some((end, steps)) = integrate(line, edge_seconds)? else {
            return Ok(None);
        };
        let begin = line.sample(0., 1.)?;
        let tree = backend
            .world
            .multibody_joints
            .get_multibody(self.owner)
            .ok_or(PhysicsError::InvalidJoint)?;
        let speed = envelope
            .center_speed_bound
            .max(self.start.velocity.length() + self.gravity.length() * face_seconds);
        let spin = envelope
            .angular_speed_bound
            .max(self.start.angular_velocity.length());
        if !backend.swept_locked_neighborhood(
            tree,
            self.owner,
            self.boxes,
            self.com,
            self.gravity,
            self.tolerance,
            speed,
            spin,
            dt,
        )? {
            return Ok(None);
        }
        let impulse = self.normal * face_normal_impulse + end.integrated_force;
        let receipt = DeckEdgeBearingReceipt {
            foot: self.foot,
            support: self.pad,
            start: t0,
            duration: dt,
            pad_edge: edge as u8,
            face_exit_bracket_seconds: bracket,
            face_seconds,
            edge_seconds,
            face_normal_impulse,
            edge_normal_impulse: end.normal_impulse,
            impulse_on_foot: impulse.to_array(),
            angular_impulse_about_com: (angular + end.angular_impulse_about_com).to_array(),
            start_edge_pressure_world: (self.top
                + self.deck_velocity * face_seconds
                + begin.center_of_pressure)
                .to_array(),
            end_edge_pressure_world: (self.top
                + self.deck_velocity * seconds
                + end.center_of_pressure)
                .to_array(),
            end_normal: end.sole_normal.to_array(),
            end_normal_force: end.normal_force,
            end_equivalent_force: end.equivalent_endpoint_force,
            normal_force_bounds: envelope.normal_force,
            integrated_substeps: steps,
            certified_time_domains: envelope.domains,
        };
        Ok(Some(PreparedBearing {
            colliders: self.boxes.colliders,
            motion: self.motion(end.state, seconds),
            receipt: BearingResult::Deck(Box::new(receipt)),
        }))
    }
}
