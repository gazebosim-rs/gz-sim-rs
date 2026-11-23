// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{CoplanarPad, FrictionlessFaceSupport, SharedFaceStep, UniformSlidingStep};
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SharedSupportRequest {
    pub foot: BodyId,
    pub supports: [BodyId; 2],
    pub preferred_fraction_b: Option<f64>,
    pub uniform_kinetic: bool,
    pub settle_comoving: bool,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SharedSupportLoad {
    pub contact_at_end: bool,
    pub support: BodyId,
    /// Signed world normal is shared by both coplanar supports.
    pub selected_normal_impulse: f64,
    pub tangent_impulse_on_foot: [f64; 3],
    pub normal_impulse_bounds: [f64; 2],
    pub start_pressure_world: Option<[f64; 3]>,
    pub end_pressure_world: Option<[f64; 3]>,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SharedFaceReceipt {
    pub foot: BodyId,
    pub start: Duration,
    pub duration: Duration,
    /// Physical bounds inside the public interval. The Duration fields round
    /// phase boundaries to the public nanosecond clock.
    pub physical_start_seconds: f64,
    pub physical_duration_seconds: f64,
    pub comoving_rest: bool,
    pub normal: [f64; 3],
    pub total_normal_impulse: f64,
    pub dissipated_energy: f64,
    /// Tangential work from the prescribed pads in world coordinates.
    pub support_tangent_work: f64,
    pub slip_start_end: Option<[[f64; 2]; 2]>,
    pub angular_impulse_about_com: [f64; 3],
    pub feasible_constant_fraction_b: [f64; 2],
    pub enclosing_fraction_b: [f64; 2],
    pub selected_fraction_b: f64,
    pub loads: [SharedSupportLoad; 2],
}
#[derive(Clone, Copy)]
struct SharedPhase {
    start: EdgeSupportState,
    h: f64,
    solution: SharedFaceStep,
    tangent: Vector,
    dissipated: f64,
    pad_work: f64,
    slip: Option<[[f64; 2]; 2]>,
    rest: bool,
}
impl SharedPhase {
    fn normal(start: EdgeSupportState, h: f64, solution: SharedFaceStep, rest: bool) -> Self {
        Self {
            start,
            h,
            solution,
            tangent: Vector::ZERO,
            dissipated: 0.,
            pad_work: 0.,
            slip: rest.then_some([[0.; 2]; 2]),
            rest,
        }
    }
    fn kinetic(start: EdgeSupportState, h: f64, step: UniformSlidingStep) -> Self {
        Self {
            start,
            h,
            solution: step.shared,
            tangent: step.tangent_per_normal,
            dissipated: step.dissipated_energy,
            pad_work: step.support_tangent_work,
            slip: Some([step.slip_start, step.slip_end]),
            rest: false,
        }
    }
}
impl Frame {
    pub(super) fn shared(
        self,
        b: &RapierBackend,
        t0: Duration,
        dt: Duration,
        corners: [Vector; 4],
        req: SharedSupportRequest,
        batch: Option<&crate::approach_clearance::SharedNeighborhood>,
    ) -> Result<Option<PreparedBearing>, PhysicsError> {
        let [a, other] = req.supports;
        let Some(second) = b.registered_boxes(self.foot, other)? else {
            return Ok(None);
        };
        let rb = &b.world.bodies[second.pad];
        let n = second.pad_pose.rotation * Vector::Z;
        let top = second.pad_pose.translation + n * second.pad_half.z;
        let vel = rb.linvel() - self.deck_velocity;
        let eps = 512. * f64::EPSILON;
        if second.friction != 0. && !req.uniform_kinetic
            || rb.angvel() != Vector::ZERO
            || (n - self.normal).length() > eps
            || (top - self.top).dot(self.normal).abs() > self.tolerance
            || vel.dot(self.normal).abs() > eps * (1. + vel.length())
        {
            return Ok(None);
        }
        let axes = [Vector::X, Vector::Y].map(|e| self.boxes.pad_pose.rotation * e);
        let xy = |v: Vector| Vector::new(v.dot(axes[0]), v.dot(axes[1]), 0.).truncate();
        let ox = xy(second.pad_pose.rotation * Vector::X);
        let pads = [
            CoplanarPad {
                center: Default::default(),
                half: self.boxes.pad_half.truncate(),
                yaw: 0.,
                velocity: Default::default(),
            },
            CoplanarPad {
                center: xy(top - self.top),
                half: second.pad_half.truncate(),
                yaw: ox.y.atan2(ox.x),
                velocity: xy(vel),
            },
        ];
        let model = FrictionlessFaceSupport {
            mass: self.mass,
            inertia_body: self.inertia,
            corners,
            pad_axes: axes,
            pad_half: self.boxes.pad_half.truncate(),
            gravity: self.gravity,
        };
        let seconds = dt.as_secs_f64();
        let friction = [self.boxes.friction, second.friction];
        let (first, second_phase) = if friction == [0.; 2] {
            let Some(bound) = model.bind_coplanar(self.start, pads)? else {
                return Ok(None);
            };
            let Some(solution) = bound.advance(seconds, req.preferred_fraction_b)? else {
                return Ok(None);
            };
            (
                SharedPhase::normal(self.start, seconds, solution, false),
                None,
            )
        } else if req.uniform_kinetic {
            let comoving = req.settle_comoving && pads[0].velocity == pads[1].velocity;
            let closing_speed = xy(self.start.velocity).distance(pads[0].velocity);
            let may_stop = comoving
                && closing_speed
                    <= -self.gravity.dot(self.normal)
                        * friction[0]
                        * seconds
                        * (1. + 512. * f64::EPSILON);
            let stop = if may_stop {
                model.advance_comoving_stop(
                    self.start,
                    pads,
                    friction,
                    seconds,
                    req.preferred_fraction_b,
                )?
            } else {
                None
            };
            if let Some(step) = stop {
                let kinetic = step
                    .sliding
                    .map(|s| SharedPhase::kinetic(self.start, step.stop_seconds, s));
                let rest = step.resting.map(|s| {
                    SharedPhase::normal(
                        kinetic.map_or(self.start, |p| p.solution.state),
                        seconds - step.stop_seconds,
                        s,
                        true,
                    )
                });
                match (kinetic, rest) {
                    (Some(first), next) => (first, next),
                    (None, Some(rest)) => (rest, None),
                    (None, None) => return Ok(None),
                }
            } else if let Some(step) = model.advance_uniform_slide(
                self.start,
                pads,
                friction,
                seconds,
                req.preferred_fraction_b,
            )? {
                (SharedPhase::kinetic(self.start, seconds, step), None)
            } else {
                return Ok(None);
            }
        } else {
            return Ok(None);
        };
        let final_state = second_phase.unwrap_or(first).solution.state;
        let tree = b
            .world
            .multibody_joints
            .get_multibody(self.owner)
            .ok_or(PhysicsError::InvalidJoint)?;
        if !b.swept_locked_supports(
            tree,
            self.owner,
            self.boxes,
            Some(second),
            self.com,
            self.gravity,
            self.tolerance,
            self.start
                .velocity
                .length()
                .max(final_state.velocity.length())
                + self.gravity.length() * seconds,
            self.start.angular_velocity.length(),
            dt,
            batch,
        )? {
            return Ok(None);
        }
        let receipt = |phase: SharedPhase, off: f64, clock_start: Duration, clock_end: Duration| {
            let loads = std::array::from_fn(|k| {
                let site = phase.solution.sites[k];
                SharedSupportLoad {
                    support: [a, other][k],
                    contact_at_end: site.contact_at_end,
                    selected_normal_impulse: site.selected_normal_impulse,
                    tangent_impulse_on_foot: (phase.tangent * site.selected_normal_impulse)
                        .to_array(),
                    normal_impulse_bounds: site.normal_impulse_bounds,
                    start_pressure_world: site
                        .material_center
                        .map(|r| self.point(phase.start, r, off).to_array()),
                    end_pressure_world: site.material_center.map(|r| {
                        self.point(phase.solution.state, r, off + phase.h)
                            .to_array()
                    }),
                }
            });
            SharedFaceReceipt {
                foot: self.foot,
                start: t0 + clock_start,
                duration: clock_end - clock_start,
                physical_start_seconds: t0.as_secs_f64() + off,
                physical_duration_seconds: phase.h,
                comoving_rest: phase.rest,
                normal: self.normal.to_array(),
                total_normal_impulse: phase.solution.normal_impulse,
                dissipated_energy: phase.dissipated,
                support_tangent_work: phase.pad_work
                    + phase.solution.normal_impulse * phase.tangent.dot(self.deck_velocity),
                slip_start_end: phase.slip,
                angular_impulse_about_com: phase.solution.angular_impulse_about_com.to_array(),
                feasible_constant_fraction_b: phase.solution.inner_fraction_b,
                enclosing_fraction_b: phase.solution.outer_fraction_b,
                selected_fraction_b: phase.solution.selected_fraction_b,
                loads,
            }
        };
        let boundary = if second_phase.is_some() {
            Duration::from_secs_f64(first.h).min(dt)
        } else {
            dt
        };
        let record = receipt(first, 0., Duration::ZERO, boundary);
        let next = second_phase.map(|p| Box::new(receipt(p, first.h, boundary, dt)));
        Ok(Some(PreparedBearing {
            colliders: self.boxes.colliders,
            motion: self.motion(final_state, seconds),
            receipt: BearingResult::Shared(Box::new(record), next, second.colliders.1),
        }))
    }
}
