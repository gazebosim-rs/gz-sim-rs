//! Couple one loaded frictionless point and its marked native travel row.
use super::{ContactConstraintsSet, JointConstraintsSet, Real, VelocitySolver};

pub(super) fn project(
    joints: &mut JointConstraintsSet,
    contacts: &mut ContactConstraintsSet,
    vs: &mut VelocitySolver,
    final_target: bool,
) {
    if contacts.generic_velocity_constraints.is_empty() {
        return;
    }
    let rows = &mut joints.generic_velocity_constraints;
    for index in 0..rows.len() {
        let row = &rows[index];
        let joint_jac = &joints.generic_jacobians;
        if !row.travel_binds(joint_jac, &vs.generic_solver_vels) {
            continue;
        }

        // The two-row block is exact for one loaded manifold on this reduced
        // tree. A second loaded manifold or reduced body needs a larger block.
        let mut attached = contacts
            .generic_velocity_constraints
            .iter()
            .enumerate()
            .filter(|(_, point)| {
                point.references_multibody(row.solver_vel2)
                    && point.normal_part[..point.num_contacts as usize]
                        .iter()
                        .any(|n| n.total_impulse() > 0.0)
            });
        let Some((i, _)) = attached.next() else {
            continue;
        };
        if attached.next().is_some() {
            continue;
        }
        if rows.iter().enumerate().any(|(j, other)| {
            j != index && other.solver_vel2 == row.solver_vel2 && other.travel_velocities.is_some()
        }) {
            continue;
        }
        let row = &mut rows[index];
        let contact = &mut contacts.generic_velocity_constraints[i];
        if contact.generic_constraint_mask == 3
            || contact.num_contacts != 1
            || contact.limit != 0.0
            || contact.normal_part[0].r == 0.0
        {
            continue;
        }
        let contact_jac = &contacts.generic_jacobians;
        let normal_wj = if contact.generic_constraint_mask == 1 {
            contact.j_id + contact.ndofs1
        } else {
            contact.j_id + 2 * contact.ndofs1 + contact.ndofs2
        };
        let cross = joint_jac
            .rows(row.j_id2, row.ndofs2)
            .dot(&contact_jac.rows(normal_wj, row.ndofs2));
        let r = contact.normal_part[0].r;
        let reduction = r * cross * cross * row.inv_lhs;
        if 1.0 - reduction <= 32.0 * Real::EPSILON {
            continue;
        }

        if final_target {
            contacts.generic_velocity_constraints_builder[i].set_travel_target(contact);
        }

        // First try the unilateral solution with a free slider. If this would
        // cross a stop, eliminate the stop velocity from the normal equation.
        let travel = row
            .travel_velocities
            .replace([Real::NEG_INFINITY, Real::INFINITY]);
        row.solve(
            joint_jac,
            &mut vs.solver_bodies,
            &mut vs.generic_solver_vels,
        );
        row.travel_velocities = travel;
        contact.solve(
            contact_jac,
            &mut vs.solver_bodies,
            &mut vs.generic_solver_vels,
            true,
            false,
        );
        row.solve(
            joint_jac,
            &mut vs.solver_bodies,
            &mut vs.generic_solver_vels,
        );
        if row.impulse != 0.0 {
            let cfm = contact.cfm_factor;
            // For native normal compliance k, CFM = 1/(1 + k*r).
            // Keep k fixed when substituting the constrained effective mass.
            contact.normal_part[0].r = r / (1.0 - reduction);
            contact.cfm_factor = cfm * (1.0 - reduction) / (1.0 - cfm * reduction);
            contact.solve(
                contact_jac,
                &mut vs.solver_bodies,
                &mut vs.generic_solver_vels,
                true,
                false,
            );
            contact.normal_part[0].r = r;
            contact.cfm_factor = cfm;
            row.solve(
                joint_jac,
                &mut vs.solver_bodies,
                &mut vs.generic_solver_vels,
            );
        }
    }
}
