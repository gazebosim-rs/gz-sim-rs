// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Signed native receipts for substep-internal changes of the load-bearing point.
use super::*;
use gz_physics::{BoundPointPath, SeparatingFeature};
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InternalPointImpact {
    /// Enclosure of the event time in seconds from this native step's start.
    pub local_bracket: [f64; 2],
    pub point: [f64; 3],
    pub normal: [f64; 3],
    pub impulse_on_foot: [f64; 3],
    pub angular_impulse_about_com: [f64; 3],
    pub opposite_opening_speed: f64,
    pub sticking: bool,
    pub lost_energy: f64,
    pub work_boxes: u32,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundPointInterval {
    pub offset: f64,
    pub slip_growth:Option<gz_physics::SlipGrowth>,
    pub end_material_velocity:[f64;3],
    pub duration: f64,
    pub feature: SeparatingFeature,
    pub sticking: bool,
    pub normal_impulse: f64,
    pub normal_impulse_on_foot: [f64; 3],
    pub impulse_on_foot: [f64; 3],
    pub angular_impulse_about_com: [f64; 3],
    pub friction_work: f64,
    pub impact: Option<InternalPointImpact>,
}
#[derive(Clone, Debug, PartialEq)]
pub struct BoundTransferReceipt {
    pub foot: BodyId,
    pub support: BodyId,
    pub start: Duration,
    pub duration: Duration,
    pub intervals: Vec<BoundPointInterval>,
    /// Includes sustained reactions and each instantaneous impact.
    pub normal_impulse: f64,
    pub normal_impulse_on_foot: [f64; 3],
    pub impulse_on_foot: [f64; 3],
}
impl BoundTransferReceipt {
    pub(super) fn from_path(
        foot: BodyId,
        support: BodyId,
        start: Duration,
        duration: Duration,
        path: BoundPointPath,
        deck: Vector,
    ) -> Self {
        let mut intervals = Vec::with_capacity(path.pieces.len());
        let mut normal = 0.;
        let mut normal_vector = Vector::ZERO;
        let mut impulse = Vector::ZERO;
        for piece in path.pieces {
            let a = piece.motion;
            normal += a.normal_impulse;
            normal_vector += a.normal_impulse_vector;
            impulse += a.impulse;
            let impact = piece.impact.map(|event| {
                let bracket = a.bracket.map(|t| piece.start + t);
                let law = event.law;
                let n = event.segment.normal;
                let p = event.segment.moving_points[law.loaded_end];
                let jn = law.impulse.dot(n);
                normal += jn;
                normal_vector += n * jn;
                impulse += law.impulse;
                InternalPointImpact {
                    local_bracket: bracket,
                    point: (p + deck * bracket[0]).to_array(),
                    normal: n.to_array(),
                    impulse_on_foot: law.impulse.to_array(),
                    angular_impulse_about_com: (p - a.center).cross(law.impulse).to_array(),
                    opposite_opening_speed: law.velocity_after[1 - law.loaded_end].dot(n),
                    sticking: law.sticking,
                    lost_energy: law.lost_energy,
                    work_boxes: event.work_boxes,
                }
            });
            intervals.push(BoundPointInterval {
                offset: piece.start,
                slip_growth:piece.slip_growth,
                end_material_velocity:(a.velocity+a.angular_velocity.cross(a.site.site.point-a.center)).to_array(),
                duration: a.bracket[0],
                feature: a.site.site.feature,
                sticking: piece.sticking,
                normal_impulse: a.normal_impulse,
                normal_impulse_on_foot: a.normal_impulse_vector.to_array(),
                impulse_on_foot: a.impulse.to_array(),
                angular_impulse_about_com: a.angular_impulse.to_array(),
                friction_work: a.friction_work,
                impact,
            })
        }
        Self {
            foot,
            support,
            start,
            duration,
            intervals,
            normal_impulse: normal,
            normal_impulse_on_foot: normal_vector.to_array(),
            impulse_on_foot: impulse.to_array(),
        }
    }
}
