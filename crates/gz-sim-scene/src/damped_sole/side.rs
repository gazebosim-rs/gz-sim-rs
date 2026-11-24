// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One frictionless corner spring on the bounded local -X face of a translating panel.
use super::*;
use gz_physics::sole_supports::{PanelReaction, validate_panels};

/// Measured upper contact, separate from the sole coating on the panel tops.
#[derive(Default, Clone, Copy, Debug)]
pub struct SideStep {
    pub support: u64,
    pub impulse: DVec3,
    /// About the instantaneous aggregate center of mass, in world axes.
    pub angular_impulse: DVec3,
    pub peak_force_n: f64,
    pub end_force_n: f64,
    pub end_elastic_energy_j: f64,
    pub support_work_j: f64,
}

#[derive(Clone, Copy)]
pub(super) struct Side {
    pub support: u64,
    pub stiffness: f64,
    pub upper: GuardBox,
}

#[derive(Default)]
struct Point {
    depth: f64,
    arm: DVec3,
    /// World normal out of the panel.
    normal: DVec3,
}

impl Side {
    /// Prove either complete swept separation or exclusive ownership of one vertex
    /// in the interior of the near face. Other features may not enter the box.
    fn point(
        self,
        body: &SoleBody,
        s: &SoleState,
        deck: &SoleDeck,
        reach: f64,
    ) -> Result<Point, SolveError> {
        if self.upper.separated(body, s, deck, reach).is_ok() {
            return Ok(Point::default());
        }
        let half = deck.half_extents;
        let rel = deck.orientation.conjugate();
        let mut closest = f64::INFINITY;
        let mut second = f64::INFINITY;
        let mut arm = DVec3::ZERO;
        let mut local = DVec3::ZERO;
        for i in 0..8 {
            let sign = |bit| if i & (1_i32 << bit) == 0 { -1. } else { 1. };
            let p = self.upper.rotation * (self.upper.half * DVec3::new(sign(0), sign(1), sign(2)));
            let r = s.orientation * (body.foot_offset + self.upper.center + p);
            let v = rel * (s.center + r - deck.center);
            let gap = -half.x - v.x;
            if gap < closest {
                second = closest;
                closest = gap;
                arm = r;
                local = v;
            } else {
                second = second.min(gap);
            }
        }
        let margin = reach
            + 128. * f64::EPSILON * (1. + s.center.length() + deck.center.length() + half.length());
        if second <= margin
            || local.y.abs() + margin >= half.y
            || local.z.abs() + margin >= half.z
            || local.x + margin >= half.x
        {
            return Err("upper side contact requires one corner inside a finite face, clear of its rim and opposite face".into());
        }
        Ok(Point {
            depth: (-closest).max(0.),
            arm,
            normal: deck.orientation * -DVec3::X,
        })
    }

    fn kick(self, body: &SoleBody, state: &mut SoleState, p: &Point, dt: f64, row: &mut SideStep) {
        let force = self.stiffness * p.depth;
        let impulse = dt * force * p.normal;
        let angular = p.arm.cross(impulse);
        state.velocity += impulse / body.mass;
        state.spin +=
            state.orientation * ((state.orientation.conjugate() * angular) / body.inertia);
        row.impulse += impulse;
        row.angular_impulse += angular;
        row.peak_force_n = row.peak_force_n.max(force);
        row.end_force_n = force;
        row.end_elastic_energy_j = 0.5 * force * p.depth;
    }
}

fn add(target: &mut SoleStep, s: SoleStep) {
    target.contact = s.contact;
    target.impulse += s.impulse;
    target.angular_impulse += s.angular_impulse;
    target.normal_impulse += s.normal_impulse;
    target.dissipated_energy += s.dissipated_energy;
    target.sliding_work += s.sliding_work;
    target.boundary_work += s.boundary_work;
    target.peak_normal_force = target.peak_normal_force.max(s.peak_normal_force);
    target.possible_missed_contacts += s.possible_missed_contacts;
}

/// Symmetric half impulses surround a full coupled lower-contact midpoint solve.
/// Validation and all speculative support motion remain local until scene commit.
#[allow(clippy::too_many_arguments)]
pub(super) fn advance(
    side: Side,
    body: &SoleBody,
    state: &mut SoleState,
    panels: &[SolePanel],
    gravity: DVec3,
    force: DVec3,
    torque: DVec3,
    dt: f64,
    substeps: u32,
    depth: u8,
) -> Result<(Vec<PanelReaction>, SideStep), SolveError> {
    validate_panels(panels, dt)?;
    let index = panels
        .iter()
        .position(|p| p.id == side.support)
        .expect("live side support");
    let h = dt / f64::from(substeps);
    let radius = side.upper.radius(body);
    if h * (side.stiffness * (body.mass.recip() + radius * radius / body.inertia.min_element()))
        .sqrt()
        > 0.25
    {
        return Err("upper spring requires a finer time grid".into());
    }
    let lower = GuardBox {
        center: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
        half: body.half_extents,
    };
    let mut row = SideStep {
        support: side.support,
        ..Default::default()
    };
    let mut sums: Vec<_> = panels
        .iter()
        .map(|p| PanelReaction {
            id: p.id,
            ..Default::default()
        })
        .collect();
    let mut moving = panels.to_vec();
    for i in 0..substeps {
        for (m, p) in moving.iter_mut().zip(panels) {
            m.deck.center = p.deck.center + (h * f64::from(i)) * p.deck.velocity;
        }
        let deck = moving[index].deck;
        let first = side.point(body, state, &deck, 0.)?;
        side.kick(body, state, &first, 0.5 * h, &mut row);
        let start = *state;
        let reactions = advance_panels(body, state, &moving, gravity, force, torque, h, 1, depth)?;
        // Bound the midpoint drift after the first kick, before the endpoint kick.
        let total_force: f64 = reactions
            .iter()
            .zip(panels)
            .map(|(r, p)| p.material.force_norm_bound(r.on_sole.peak_normal_force))
            .sum();
        let sole_radius = (body.foot_offset.abs() + body.half_extents).length();
        let acceleration = (gravity + force / body.mass).length() + total_force / body.mass;
        let total_torque = torque.length() + sole_radius * total_force;
        let momentum = (body.inertia * (start.orientation.conjugate() * start.spin)).length();
        let spin = (momentum + h * total_torque) / body.inertia.min_element();
        let speed = (start.velocity - deck.velocity).length() + h * acceleration;
        lower.separated(body, &start, &deck, h * (speed + spin * sole_radius))?;
        let witness = side.point(body, &start, &deck, h * (speed + spin * radius))?;
        let mut end = deck;
        end.center += h * deck.velocity;
        let last = side.point(body, state, &end, 0.)?;
        if witness.normal != DVec3::ZERO && first.depth == 0. && last.depth == 0. {
            // Both endpoints are free. A near tangent can enter and exit within
            // one interval; require a monotone signed gap or retry on a finer grid.
            let normal_speed = (start.velocity - deck.velocity + start.spin.cross(witness.arm))
                .dot(witness.normal);
            let angular_acceleration = (total_torque
                + spin * spin * (body.inertia.max_element() - body.inertia.min_element()))
                / body.inertia.min_element();
            if normal_speed.abs()
                <= h * (acceleration + radius * (spin * spin + angular_acceleration))
            {
                return Err("unresolved grazing upper corner; refine the time grid".into());
            }
        }
        side.kick(body, state, &last, 0.5 * h, &mut row);
        for (sum, r) in sums.iter_mut().zip(reactions) {
            add(&mut sum.on_sole, r.on_sole);
            sum.support_work_j += r.support_work_j;
        }
        for p in panels.iter().filter(|p| p.id != side.support) {
            let mut d = p.deck;
            d.center += h * f64::from(i) * d.velocity;
            let travel =
                h * ((start.velocity - d.velocity).length() + h * acceleration + spin * radius);
            side.upper.separated(body, &start, &d, travel)?;
        }
    }
    // The final half impulse changes the lower coating's velocity-dependent
    // endpoint pressure. Its integrated load still comes from the accepted drift.
    for (s, p) in sums.iter_mut().zip(panels) {
        let mut deck = p.deck;
        deck.center += dt * deck.velocity;
        let end = body.contact(state, &deck, &p.material, depth)?;
        s.on_sole.peak_normal_force = s.on_sole.peak_normal_force.max(end.normal_force);
        s.on_sole.contact = end;
    }
    row.support_work_j = row.impulse.dot(panels[index].deck.velocity);
    Ok((sums, row))
}
