//! Couple a loaded frictionless point and the unilateral slider rows on its native tree.
use super::{ContactConstraintsSet, JointConstraintsSet, Real, VelocitySolver};
use crate::alloc_prelude::*;

pub(super) fn finish(
    joints: &mut JointConstraintsSet,
    contacts: &mut ContactConstraintsSet,
    vs: &mut VelocitySolver,
) {
    if contacts.generic_velocity_constraints.is_empty() {
        return;
    }
    let rows = &mut joints.generic_velocity_constraints;
    let mut stops = Vec::new();
    for (i, row) in rows.iter().enumerate() {
        if row.travel_binds(&joints.generic_jacobians, &vs.generic_solver_vels) {
            stops.push(i);
        }
    }
    if stops.is_empty() {
        return;
    }
    let mut points = Vec::new();
    for (i, (builder, point)) in contacts
        .generic_velocity_constraints_builder
        .iter()
        .zip(&mut contacts.generic_velocity_constraints)
        .enumerate()
    {
        if stops
            .iter()
            .any(|&s| point.references_multibody(rows[s].solver_vel2))
            && builder.prepare_travel_target(point)
        {
            points.push(i);
        }
    }
    if points.is_empty() {
        return;
    }
    stops.retain(|&s| {
        points.iter().any(|&i| {
            contacts.generic_velocity_constraints[i].references_multibody(rows[s].solver_vel2)
        })
    });
    // Iterate only coupled native rows. Bank and write back the resulting contact
    // impulse exactly once through the ordinary native manifold path.
    for _ in 0..96 {
        let mut change: Real = 0.0;
        let mut scale: Real = 0.0;
        for &s in &stops {
            let row = &mut rows[s];
            let was = row.impulse;
            row.solve(
                &joints.generic_jacobians,
                &mut vs.solver_bodies,
                &mut vs.generic_solver_vels,
            );
            change = change.max((row.impulse - was).abs());
            scale = scale.max(row.impulse.abs());
        }
        for &i in &points {
            let contact = &mut contacts.generic_velocity_constraints[i];
            let was = contact.normal_part[0].impulse;
            contact.solve(
                &contacts.generic_jacobians,
                &mut vs.solver_bodies,
                &mut vs.generic_solver_vels,
                true,
                false,
            );
            change = change.max((contact.normal_part[0].impulse - was).abs());
            scale = scale.max(contact.normal_part[0].total_impulse().abs());
        }
        if change <= 16.0 * Real::EPSILON * scale {
            break;
        }
    }
}
