// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A finite rigid support with prescribed constant world twist during one tick.
use crate::sole_contact::{
    SoleAccuracy, SoleBody, SoleContact, SoleDeck, SoleDifference, SoleError, SoleMaterial,
    SoleState, SoleStep, adaptive::select_grid,
};
use glam::{DQuat, DVec3};

#[derive(Debug, Clone, Copy)]
pub struct SoleTurntable {
    /// Initial pose and constant center velocity of the support box.
    pub deck: SoleDeck,
    /// Constant world angular velocity about the moving support center.
    pub angular_velocity: DVec3,
}

impl SoleTurntable {
    /// Pose at elapsed time within this tick. Translation and rotation are
    /// independently prescribed about the panel center, with no drift.
    pub fn at(self, t: f64) -> SoleDeck {
        SoleDeck {
            center: self.deck.center + t * self.deck.velocity,
            orientation: (DQuat::from_scaled_axis(t * self.angular_velocity)
                * self.deck.orientation)
                .normalize(),
            ..self.deck
        }
    }

    /// Instantaneous body reaction using the deck pose stored in this value.
    pub fn contact(
        &self,
        body: &SoleBody,
        state: &SoleState,
        material: &SoleMaterial,
        depth: u8,
    ) -> Result<SoleContact, SoleError> {
        if !self.angular_velocity.is_finite() {
            return Err(SoleError::InvalidInput);
        }
        let relative = SoleState {
            velocity: state.velocity - self.angular_velocity.cross(state.center - self.deck.center),
            spin: state.spin - self.angular_velocity,
            ..*state
        };
        body.contact(&relative, &self.deck, material, depth)
    }

    fn spin_power(&self, s: SoleState, contact: SoleContact) -> f64 {
        let moment = contact.torque + (s.center - self.deck.center).cross(contact.force);
        moment.dot(self.angular_velocity)
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct TurntableStep {
    pub on_sole: SoleStep,
    /// Work supplied through both prescribed translation and rotation.
    pub support_work_j: f64,
    /// Portion contributed by prescribed rotation about the moving panel center.
    pub spin_work_j: f64,
}

#[allow(clippy::too_many_arguments)]
pub fn advance_turntable(
    body: &SoleBody,
    state: &mut SoleState,
    table: &SoleTurntable,
    material: &SoleMaterial,
    gravity: DVec3,
    external_force: DVec3,
    external_torque: DVec3,
    dt: f64,
    substeps: u32,
    depth: u8,
) -> Result<TurntableStep, SoleError> {
    if !table.angular_velocity.is_finite()
        || !dt.is_finite()
        || dt <= 0.
        || substeps == 0
        || !gravity.is_finite()
        || !external_force.is_finite()
        || !external_torque.is_finite()
    {
        return Err(SoleError::InvalidInput);
    }
    let mut s = *state;
    if table.angular_velocity == DVec3::ZERO {
        let on_sole = body.step_with_wrench(
            &mut s,
            &table.deck,
            material,
            gravity,
            dt,
            substeps,
            depth,
            external_force,
            external_torque,
        )?;
        let support_work_j = on_sole.impulse.dot(table.deck.velocity);
        if !support_work_j.is_finite() {
            return Err(SoleError::Nonfinite);
        }
        *state = s;
        return Ok(TurntableStep {
            on_sole,
            support_work_j,
            spin_work_j: 0.,
        });
    }
    let g = gravity + external_force / body.mass;
    let h = dt / f64::from(substeps);
    if 0.5 * h == 0. {
        return Err(SoleError::InvalidInput);
    }
    let mut out = TurntableStep::default();
    let r = &mut out.on_sole;
    let mut pending = false;
    // Sample the original table before constructing later orientations so an
    // invalid authored quaternion cannot be normalized into an accepted pose.
    let mut here = *table;
    for i in 0..substeps {
        let t = f64::from(i) * h;
        let a = here.contact(body, &s, material, depth)?;
        if pending && a.max_indentation == 0. {
            r.possible_missed_contacts += 1;
        }
        let mid = body.advance(s, s, a, g, external_torque, 0.5 * h);
        let middle = SoleTurntable {
            deck: table.at(t + 0.5 * h),
            ..*table
        };
        let b = middle.contact(body, &mid, material, depth)?;
        r.impulse += h * b.force;
        r.angular_impulse += h * b.torque;
        r.normal_impulse += h * b.normal_force;
        r.dissipated_energy += h * b.heat_power;
        r.sliding_work += h * b.sliding_power;
        r.boundary_work += h * b.boundary_power;
        r.peak_normal_force = r.peak_normal_force.max(a.normal_force).max(b.normal_force);
        out.spin_work_j += h * middle.spin_power(mid, b);
        pending = a.max_indentation == 0.
            && b.max_indentation == 0.
            && !body.separated_rotating_midpoint(
                s,
                mid,
                here.deck,
                here.angular_velocity,
                g + a.force / body.mass,
                h,
            );
        s = body.advance(s, mid, b, g, external_torque, h);
        here.deck = table.at(f64::from(i + 1) * h);
    }
    // Evaluate at the exact public endpoint, including its frame orientation.
    here.deck = table.at(dt);
    r.contact = here.contact(body, &s, material, depth)?;
    if pending && r.contact.max_indentation == 0. {
        r.possible_missed_contacts += 1;
    }
    r.peak_normal_force = r.peak_normal_force.max(r.contact.normal_force);
    out.support_work_j = r.impulse.dot(table.deck.velocity) + out.spin_work_j;
    if !r.impulse.is_finite()
        || !r.angular_impulse.is_finite()
        || ![
            r.normal_impulse,
            r.dissipated_energy,
            r.sliding_work,
            r.boundary_work,
            out.spin_work_j,
            out.support_work_j,
        ]
        .into_iter()
        .all(f64::is_finite)
    {
        return Err(SoleError::Nonfinite);
    }
    *state = s;
    Ok(out)
}

#[derive(Debug, Clone, Copy)]
pub struct TurntableDifference {
    pub contact: SoleDifference,
    pub spin_work_j: f64,
    pub support_work_j: f64,
    pub sliding_work_j: f64,
    pub rim_work_j: f64,
}
#[derive(Debug, Clone, Copy)]
pub struct AdaptiveTurntableStep {
    pub result: TurntableStep,
    pub substeps: u32,
    pub depth: u8,
    pub trials: u32,
    pub temporal: TurntableDifference,
    pub spatial: TurntableDifference,
}

#[allow(clippy::too_many_arguments)]
pub fn advance_turntable_adaptive(
    body: &SoleBody,
    state: &mut SoleState,
    table: &SoleTurntable,
    material: &SoleMaterial,
    gravity: DVec3,
    external_force: DVec3,
    external_torque: DVec3,
    dt: f64,
    accuracy: &SoleAccuracy,
    work_tolerance_j: f64,
) -> Result<AdaptiveTurntableStep, SoleError> {
    if !work_tolerance_j.is_finite() || work_tolerance_j < 0. {
        return Err(SoleError::InvalidInput);
    }
    let initial = *state;
    let grid = select_grid(
        accuracy,
        |n, d| -> Result<_, SoleError> {
            let mut s = initial;
            let r = advance_turntable(
                body,
                &mut s,
                table,
                material,
                gravity,
                external_force,
                external_torque,
                dt,
                n,
                d,
            )?;
            Ok((s, r))
        },
        |(a, u), (b, v)| TurntableDifference {
            contact: SoleDifference {
                impulse_ns: u.on_sole.impulse.distance(v.on_sole.impulse),
                angular_impulse_nms: u
                    .on_sole
                    .angular_impulse
                    .distance(v.on_sole.angular_impulse),
                heat_j: (u.on_sole.dissipated_energy - v.on_sole.dissipated_energy).abs(),
                ..SoleDifference::between_states(a, b)
            },
            spin_work_j: (u.spin_work_j - v.spin_work_j).abs(),
            support_work_j: (u.support_work_j - v.support_work_j).abs(),
            sliding_work_j: (u.on_sole.sliding_work - v.on_sole.sliding_work).abs(),
            rim_work_j: (u.on_sole.boundary_work - v.on_sole.boundary_work).abs(),
        },
        |e| {
            e.contact.within(accuracy)
                && [
                    e.spin_work_j,
                    e.support_work_j,
                    e.sliding_work_j,
                    e.rim_work_j,
                ]
                .into_iter()
                .all(|w| w <= work_tolerance_j)
        },
        |(_, step)| step.on_sole.unresolved(),
    )?;
    *state = grid.value.0;
    Ok(AdaptiveTurntableStep {
        result: grid.value.1,
        substeps: grid.substeps,
        depth: grid.depth,
        trials: grid.trials,
        temporal: grid.temporal,
        spatial: grid.spatial,
    })
}
#[cfg(test)]
mod tests;
