// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Stage both the continuous and instantaneous reactions inside the final clock nanosecond.
use super::{sustained::RigidTreeMotion, *};
use gz_physics::{FractionalPointPath, SeparatingFeature};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FractionalPointEvent {
    /// Seconds from the public interval start. Retains the fractional nanosecond.
    pub time_s: f64,
    pub world_point: [f64; 3],
    pub normal: [f64; 3],
    pub impulse_on_foot: [f64; 3],
    pub angular_impulse_about_com: [f64; 3],
    pub dissipated_energy: f64,
    pub sticking: bool,
}
#[derive(Clone, Debug, PartialEq)]
pub struct FractionalPointReceipt {
    pub foot: BodyId,
    pub support: BodyId,
    pub native_boundary: Duration,
    pub first_bracket_s: [f64; 2],
    pub first_feature: SeparatingFeature,
    pub events: Vec<FractionalPointEvent>,
    /// Sum of all instantaneous and sustained contact reactions through the native boundary.
    pub impulse_on_foot: [f64; 3],
    pub normal_impulse: f64,
    pub normal_impulse_on_foot: [f64; 3],
    pub angular_impulse_about_com: [f64; 3],
    pub sustained_friction_work: f64,
    pub end_feature: SeparatingFeature,
    pub end_point: [f64; 3],
    pub end_normal: [f64; 3],
    pub end_sticking: bool,
    /// First-order material boundary estimate from the actual final supported site.
    pub approaching_limit: Option<(gz_physics::FeatureLimit, f64)>,
    /// A verified shorter native interval was selected after the original proof declined.
    pub next_native_boundary: Option<Duration>,
    /// Absolute times of certified shortened follow-up boundaries in this public interval.
    pub continued_boundaries: Vec<Duration>,
}
pub(crate) struct PreparedFractional {
    motion: RigidTreeMotion,
    receipt: FractionalPointReceipt,
}
impl PreparedFractional {
    pub(super) fn new(
        initial: RigidTreeMotion,
        initial_sole: Rotation,
        deck: Vector,
        pair: (BodyId, BodyId),
        boundary: Duration,
        path: FractionalPointPath,
    ) -> Self {
        let end = path
            .follow
            .last()
            .expect("qualified fractional path is nonempty");
        let state = end.motion;
        let t = boundary.as_secs_f64();
        let center = state.center + deck * t;
        let rot = state.moving.orientation * initial_sole.inverse();
        let motion = RigidTreeMotion {
            root_pose: Pose::from_parts(
                center + rot * (initial.root_pose.translation - initial.center),
                (rot * initial.root_pose.rotation).normalize(),
            ),
            center,
            center_velocity: state.velocity + deck,
            angular: state.angular_velocity,
            ..initial
        };
        let site = path.first_site;
        let first = path.first;
        let moment = (site.point - first.after.center_of_mass).cross(first.impulse);
        let mut events = vec![FractionalPointEvent {
            time_s: path.first_time,
            world_point: (site.point + deck * path.first_time).to_array(),
            normal: site.normal.to_array(),
            impulse_on_foot: first.impulse.to_array(),
            angular_impulse_about_com: moment.to_array(),
            dissipated_energy: first.lost_energy,
            sticking: first.sticking,
        }];
        let mut jn = site.normal.dot(first.impulse);
        let mut nvec = jn * site.normal;
        let mut angular = moment;
        let mut friction_work = 0.;
        for piece in &path.follow {
            let s = piece.motion;
            jn += s.normal_impulse;
            nvec += s.normal_impulse_vector;
            angular += s.angular_impulse;
            friction_work += s.friction_work;
            if let Some(hit) = piece.impact {
                let time = path.first_time + piece.start + s.bracket[0];
                let p = hit.segment.moving_points[hit.law.loaded_end];
                let j = hit.law.impulse;
                let n = hit.segment.normal;
                let arm = p - s.center;
                let normal = j.dot(n);
                jn += normal;
                nvec += normal * n;
                angular += arm.cross(j);
                events.push(FractionalPointEvent {
                    time_s: time,
                    world_point: (p + deck * time).to_array(),
                    normal: n.to_array(),
                    impulse_on_foot: j.to_array(),
                    angular_impulse_about_com: arm.cross(j).to_array(),
                    dissipated_energy: hit.law.lost_energy,
                    sticking: hit.law.sticking,
                });
            }
        }
        let receipt = FractionalPointReceipt {
            foot: pair.0,
            support: pair.1,
            native_boundary: boundary,
            first_feature: site.feature,
            first_bracket_s: path.crossing_bracket,
            events,
            impulse_on_foot: path.impulse().to_array(),
            normal_impulse: jn,
            normal_impulse_on_foot: nvec.to_array(),
            angular_impulse_about_com: angular.to_array(),
            sustained_friction_work: friction_work,
            end_feature: state.site.site.feature,
            end_point: (state.site.site.point + deck * t).to_array(),
            end_normal: state.site.site.normal.to_array(),
            end_sticking: end.sticking,
            approaching_limit: state.site.approaching_limit(),
            next_native_boundary: None,
            continued_boundaries: Vec::new(),
        };
        Self { motion, receipt }
    }
    pub(crate) fn commit(self, world: &mut RapierBackend) -> FractionalPointReceipt {
        self.motion.commit(world);
        self.receipt
    }
}
