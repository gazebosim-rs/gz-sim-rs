// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-tick world edits retain the native body, collider, bindings and solver.
use super::*;
use gz_physics::BodyState;

impl Body {
    pub(crate) fn with_state(&self, state: BodyState) -> Self {
        Self {
            position: state.position,
            orientation: state.orientation,
            linear: state.velocity,
            angular: state.angular_velocity,
            ..self.clone()
        }
    }

    pub(crate) fn state(&self) -> BodyState {
        BodyState {
            position: self.position,
            orientation: self.orientation,
            velocity: self.linear,
            angular_velocity: self.angular,
        }
    }
}

pub(crate) fn admit_edit<'a>(
    old: &Body,
    next: &Body,
    residents: impl Iterator<Item = &'a Body>,
    inputs: &[PendingImpulse],
    step: Duration,
    ground: Option<GroundPlane>,
) -> Result<(), WorldError> {
    if &old.with_state(next.state()) != next
        || next
            .linear
            .into_iter()
            .chain(next.angular)
            .any(|v| !v.is_finite())
        || (next.tier == Tier::Fixed && (next.linear != [0.; 3] || next.angular != [0.; 3]))
    {
        return Err(PhysicsError::InvalidValue.into());
    }
    if next.tier == Tier::Prescribed {
        MultiFidelityBackend::new([0.; 3])?.validate_kinematic_velocity(
            next.linear,
            next.angular,
            step,
        )?;
    }
    let target = next.bounds()?;
    if old.position != next.position || old.orientation != next.orientation {
        if let Some(ground) = ground {
            ground.admits(next)?;
        }
        for other in residents.filter(|b| b.id != next.id) {
            if target.predicts_overlap(&other.bounds()?) {
                return Err(PhysicsError::TransitionDeferred {
                    reason: "edited box would intersect resident contact prediction bounds",
                }
                .into());
            }
        }
    }
    validate_queued_for(next, inputs)
}
