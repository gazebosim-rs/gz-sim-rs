// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Simultaneous reactions from separated or abutting rectangular deck panels.
//! Bodies and reactions use world coordinates. A panel translates at constant
//! velocity during one public tick. All load paths act on the same midpoint.
use crate::sole_contact::{
    SoleBody, SoleContact, SoleDeck, SoleError, SoleMaterial, SoleState, SoleStep,
};
use glam::DVec3;

#[derive(Clone, Copy, Debug)]
pub struct SolePanel {
    pub id: u64,
    pub deck: SoleDeck,
    pub material: SoleMaterial,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct PanelReaction {
    pub id: u64,
    pub on_sole: SoleStep,
    /// Work supplied by this moving panel through the contact forces.
    pub support_work_j: f64,
}

#[derive(Debug, thiserror::Error)]
pub enum PanelError {
    #[error(transparent)]
    Contact(#[from] SoleError),
    #[error("duplicate support identity {0}")]
    Duplicate(u64),
    #[error("panels {0} and {1} require a common facing direction")]
    DifferentNormals(u64, u64),
    #[error("the projected interiors of panels {0} and {1} overlap during the tick")]
    Overlap(u64, u64),
}

fn translated(mut deck: SoleDeck, t: f64) -> SoleDeck {
    deck.center += deck.velocity * t;
    deck
}

fn projected_radius(deck: SoleDeck, axis: DVec3, scale: f64) -> f64 {
    let local = deck.orientation.conjugate() * axis;
    local.x.abs() * (deck.half_extents.x * scale) + local.y.abs() * (deck.half_extents.y * scale)
}

fn check_pair(a: &SolePanel, b: &SolePanel, dt: f64) -> Result<(), PanelError> {
    if a.id == b.id {
        return Err(PanelError::Duplicate(a.id));
    }
    if (a.deck.orientation * DVec3::Z).distance(b.deck.orientation * DVec3::Z) > 1e-12 {
        return Err(PanelError::DifferentNormals(a.id, b.id));
    }
    // Swept separating axes: intersect the four intervals of time when each
    // projection overlaps in its interior. Touching an endpoint is admissible.
    // Shrink by a few ulps so a rotated, exactly abutting seam stays admissible.
    let (mut enter, mut leave) = (0_f64, dt);
    for axis in [
        a.deck.orientation * DVec3::X,
        a.deck.orientation * DVec3::Y,
        b.deck.orientation * DVec3::X,
        b.deck.orientation * DVec3::Y,
    ] {
        let project = |scale| {
            let x = axis.dot(b.deck.center * scale - a.deck.center * scale);
            let v = axis.dot(b.deck.velocity * scale - a.deck.velocity * scale);
            let extent =
                projected_radius(a.deck, axis, scale) + projected_radius(b.deck, axis, scale);
            let limit = extent - 16. * f64::EPSILON * extent;
            (x, v, limit)
        };
        let (mut x, mut v, mut limit) = project(1.);
        if ![x, v, limit, -limit - x, limit - x]
            .into_iter()
            .all(f64::is_finite)
        {
            // A common binary scaling leaves crossing times invariant and
            // avoids center, extent and relative-velocity overflow.
            (x, v, limit) = project(0.0625);
            if ![x, v, limit, -limit - x, limit - x]
                .into_iter()
                .all(f64::is_finite)
            {
                return Err(SoleError::Nonfinite.into());
            }
        }
        let end = v.mul_add(dt, x);
        if (x >= limit && end >= limit) || (x <= -limit && end <= -limit) {
            return Ok(());
        }
        if v != 0. {
            let p = (-limit - x) / v;
            let q = (limit - x) / v;
            // A positive overlap can be much shorter than one ulp of the
            // arrival time. Outward rounding prevents treating it as tangency.
            enter = enter.max(p.min(q).next_down());
            leave = leave.min(p.max(q).next_up());
            if enter >= leave {
                return Ok(());
            }
        }
    }
    Err(PanelError::Overlap(a.id, b.id))
}

/// Check finite panel geometry and ownership across the entire linear sweep.
pub fn validate_panels(panels: &[SolePanel], dt: f64) -> Result<(), PanelError> {
    if panels.is_empty() || !dt.is_finite() || dt <= 0. {
        return Err(SoleError::InvalidInput.into());
    }
    for (i, p) in panels.iter().enumerate() {
        let d = p.deck;
        if !d.center.is_finite()
            || !d.velocity.is_finite()
            || !translated(d, dt).center.is_finite()
            || !d.orientation.is_finite()
            || (d.orientation.length_squared() - 1.).abs() >= 1e-8
            || !d.half_extents.is_finite()
            || d.half_extents.min_element() <= 0.
        {
            return Err(SoleError::InvalidInput.into());
        }
        for other in &panels[..i] {
            check_pair(other, p, dt)?
        }
    }
    Ok(())
}

/// Only commit the foot after every panel and stage succeeds. Results stay in
/// stable public-ID order even when input enrollment order changes.
#[allow(clippy::too_many_arguments)]
pub fn advance_panels(
    body: &SoleBody,
    state: &mut SoleState,
    panels: &[SolePanel],
    gravity: DVec3,
    external_force: DVec3,
    external_torque: DVec3,
    dt: f64,
    substeps: u32,
    depth: u8,
) -> Result<Vec<PanelReaction>, PanelError> {
    let sorted = prepare_panels(panels, dt)?;
    advance_prepared(
        body,
        state,
        &sorted,
        gravity,
        external_force,
        external_torque,
        dt,
        substeps,
        depth,
    )
}

fn prepare_panels(panels: &[SolePanel], dt: f64) -> Result<Vec<&SolePanel>, PanelError> {
    validate_panels(panels, dt)?;
    let mut sorted: Vec<_> = panels.iter().collect();
    sorted.sort_unstable_by_key(|p| p.id);
    Ok(sorted)
}

#[allow(clippy::too_many_arguments)]
fn advance_prepared(
    body: &SoleBody,
    state: &mut SoleState,
    sorted: &[&SolePanel],
    gravity: DVec3,
    external_force: DVec3,
    external_torque: DVec3,
    dt: f64,
    substeps: u32,
    depth: u8,
) -> Result<Vec<PanelReaction>, PanelError> {
    if substeps == 0
        || !gravity.is_finite()
        || !external_force.is_finite()
        || !external_torque.is_finite()
    {
        return Err(SoleError::InvalidInput.into());
    }
    let mut rows: Vec<_> = sorted
        .iter()
        .map(|p| PanelReaction {
            id: p.id,
            ..Default::default()
        })
        .collect();
    let mut s = *state;
    // A missing endpoint is resolved by the next first-stage evaluation.
    // Scratch buffers are reused for every midpoint in this public tick.
    let mut first_free = vec![false; rows.len()];
    let mut mid_free = vec![false; rows.len()];
    let mut sweep_pending = vec![false; rows.len()];
    let h = dt / f64::from(substeps);
    if 0.5 * h == 0. {
        return Err(SoleError::InvalidInput.into());
    }
    let g = gravity + external_force / body.mass;
    for i in 0..substeps {
        let t = h * f64::from(i);
        let sample = |s: &SoleState,
                      t: f64,
                      weight: f64,
                      rows: &mut [PanelReaction],
                      free: &mut [bool]|
         -> Result<SoleContact, PanelError> {
            let mut sum = SoleContact::default();
            for ((p, row), free) in sorted.iter().zip(rows).zip(free) {
                let c = body.contact(s, &translated(p.deck, t), &p.material, depth)?;
                *free = c.max_indentation == 0.;
                sum.force += c.force;
                sum.torque += c.torque;
                let r = &mut row.on_sole;
                r.peak_normal_force = r.peak_normal_force.max(c.normal_force);
                r.impulse += weight * c.force;
                r.angular_impulse += weight * c.torque;
                r.normal_impulse += weight * c.normal_force;
                r.dissipated_energy += weight * c.heat_power;
                r.sliding_work += weight * c.sliding_power;
                r.boundary_work += weight * c.boundary_power;
            }
            Ok(sum)
        };
        let a = sample(&s, t, 0., &mut rows, &mut first_free)?;
        for ((row, &free), &pending) in rows.iter_mut().zip(&first_free).zip(&sweep_pending) {
            if pending && free {
                row.on_sole.possible_missed_contacts += 1;
            }
        }
        let mid = body.advance(s, s, a, g, external_torque, h * 0.5);
        let b = sample(&mid, t + h * 0.5, h, &mut rows, &mut mid_free)?;
        let accel = g + a.force / body.mass;
        for (i, p) in sorted.iter().enumerate() {
            sweep_pending[i] = first_free[i]
                && mid_free[i]
                && !body.separated_midpoint(s, mid, translated(p.deck, t), accel, h);
        }
        s = body.advance(s, mid, b, g, external_torque, h);
    }
    for ((p, row), pending) in sorted.iter().zip(&mut rows).zip(sweep_pending) {
        let r = &mut row.on_sole;
        r.contact = body.contact(&s, &translated(p.deck, dt), &p.material, depth)?;
        if pending && r.contact.max_indentation == 0. {
            r.possible_missed_contacts += 1;
        }
        r.peak_normal_force = r.peak_normal_force.max(r.contact.normal_force);
        row.support_work_j = r.impulse.dot(p.deck.velocity);
        if !r.impulse.is_finite()
            || !r.angular_impulse.is_finite()
            || ![
                r.normal_impulse,
                r.dissipated_energy,
                r.sliding_work,
                r.boundary_work,
                row.support_work_j,
            ]
            .into_iter()
            .all(f64::is_finite)
        {
            return Err(SoleError::Nonfinite.into());
        }
    }
    *state = s;
    Ok(rows)
}

mod adaptive;
pub use adaptive::{AdaptivePanelStep, PanelDifference, advance_panels_adaptive};
