// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Admission for an unforced rigid motion of a passive revolute tree.
use super::*;

/// Inertial wrench about the system COM, accumulated over a native subtree.
#[derive(Clone, Copy, Default)]
struct Wrench {
    force: Vector,
    moment: Vector,
    moment_scale: f64,
}

impl RapierBackend {
    /// Accept a free-flight coast only if holding every hinge needs zero
    /// generalized torque. A subsequent input or change of collision domain
    /// returns the tree to dynamics before its next advancing solve.
    pub fn begin_articulation_free_coast(
        &mut self,
        root: BodyId,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        if dt.is_zero() {
            return Err(PhysicsError::InvalidValue);
        }
        let index = self.articulation_index(root)?;
        if let Some(coast) = &self.free_coast
            && coast.root == root
        {
            return if self.coast_disturbed(coast.len()) {
                Err(deferred("free coast already has a pending disturbance"))
            } else {
                Ok(())
            };
        }
        if self.articulation_tier(root)? != ArticulationTier::Dynamic
            || self.joints.construction_open()
        {
            return Err(deferred("free coast requires a committed dynamic assembly"));
        }
        let tree = self
            .world
            .multibody_joints
            .get_multibody(index)
            .expect("validated tree");
        let count = tree.num_links();
        if self.world.gravity != Vector::ZERO
            || count != self.world.bodies.len()
            || count != self.world.colliders.len()
        {
            return Err(deferred(
                "free coast requires one isolated weightless articulation",
            ));
        }
        let root_body = &self.world.bodies[tree.root().rigid_body_handle()];
        let origin = root_body.center_of_mass();
        let omega = root_body.angvel();
        let mut mass = 0.;
        let mut centroid = Vector::ZERO;
        let mut twice_internal_energy = 0.;
        let mut least_mass = f64::INFINITY;
        let mut least_moment = f64::INFINITY;
        // Offsets are measured relative to the root COM to avoid moments
        // about a distant world origin. The body velocities are COM twists.
        for link in tree.links() {
            let body = &self.world.bodies[link.rigid_body_handle()];
            if body.colliders().iter().any(|&c| {
                self.world
                    .narrow_phase
                    .contact_pairs_with(c)
                    .any(|p| p.has_any_active_contact())
            }) {
                return Err(deferred("free coast requires a contact-free assembly"));
            }
            let offset = body.center_of_mass() - origin;
            let rigid_linear = root_body.linvel() + omega.cross(offset);
            let linear_scale = 1. + root_body.linvel().length() + omega.length() * offset.length();
            let angular_scale = 1. + omega.length();
            if !rigid_linear.is_finite()
                || !linear_scale.is_finite()
                || body.linvel().distance(rigid_linear) > 1024. * f64::EPSILON * linear_scale
                || body.angvel().distance(omega) > 1024. * f64::EPSILON * angular_scale
            {
                return Err(deferred(
                    "free coast requires a common rigid twist at every link",
                ));
            }
            let moments = body.mass_properties().local_mprops.principal_inertia();
            least_mass = least_mass.min(body.mass());
            least_moment = least_moment.min(moments.min_element());
            // The COM velocity minimizes relative translational energy. Using
            // root velocity bounds that energy from above without subtracting
            // two large total kinetic energies. The largest principal moment
            // bounds each body's rotational energy in any orientation.
            twice_internal_energy += body.mass()
                * (body.linvel() - root_body.linvel()).length_squared()
                + moments.max_element() * body.angvel().length_squared();
            mass += body.mass();
            centroid += offset * body.mass();
        }
        centroid /= mass;
        if !centroid.is_finite() || !mass.is_finite() || mass <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        // Even complete redistribution of this isolated system's internal
        // energy cannot make any COM move faster than 1e-11 m/s relative to
        // the system COM, or any body turn faster than 1e-11 rad/s. This
        // certifies nearly exact translations of chains with unequal inertias.
        // All original velocity bits remain intact.
        const SPEED_RESOLUTION: f64 = 1e-11;
        let translation = twice_internal_energy.is_finite()
            && twice_internal_energy <= least_mass * SPEED_RESOLUTION.powi(2)
            && twice_internal_energy <= least_moment * SPEED_RESOLUTION.powi(2);
        if !translation {
            let mut sums = vec![Wrench::default(); count];
            // Native indices visit a parent before its descendants. In reverse
            // order, each subtree's complete wrench is ready at its root.
            for id in (0..count).rev() {
                let link = tree.link(id).expect("contiguous native links");
                let body = &self.world.bodies[link.rigid_body_handle()];
                let r = (body.center_of_mass() - origin) - centroid;
                let force = body.mass() * omega.cross(omega.cross(r));
                let spin_momentum = body.mass_properties().effective_angular_inertia() * omega;
                let s = &mut sums[id];
                s.force += force;
                s.moment += omega.cross(spin_momentum) + r.cross(force);
                s.moment_scale +=
                    omega.length() * spin_momentum.length() + r.length() * force.length();
                let full = *s;
                if !full.force.is_finite()
                    || !full.moment.is_finite()
                    || !full.moment_scale.is_finite()
                {
                    return Err(PhysicsError::InvalidValue);
                }
                if let Some(parent_id) = link.parent_id() {
                    let parent =
                        &self.world.bodies[tree.link(parent_id).unwrap().rigid_body_handle()];
                    let joint = &link.joint.data.local_frame1;
                    let axis = *parent.rotation() * joint.rotation * Vector::X;
                    let arm = (parent.translation() - origin) - centroid
                        + *parent.rotation() * joint.translation;
                    let torque = (full.moment - arm.cross(full.force)).dot(axis);
                    let scale = full.moment_scale + arm.length() * full.force.length();
                    if !torque.is_finite() || !scale.is_finite() {
                        return Err(PhysicsError::InvalidValue);
                    }
                    if torque.abs() > 4096. * f64::EPSILON * scale {
                        return Err(deferred(
                            "passive hinge requires torque to hold its coast configuration",
                        ));
                    }
                    sums[parent_id].force += full.force;
                    sums[parent_id].moment += full.moment;
                    sums[parent_id].moment_scale += full.moment_scale;
                } else if full.moment.length() > 4096. * f64::EPSILON * full.moment_scale {
                    return Err(deferred(
                        "free coast spin is not an aggregate principal axis",
                    ));
                }
            }
        }
        self.switch_articulation(root, ArticulationTier::Prescribed, dt)?;
        self.free_coast = Some(analytic_coast::FreeCoast::new(&self.world, index, root));
        Ok(())
    }

    fn coast_disturbed(&self, count: usize) -> bool {
        !self.held_wrenches.is_empty()
            || self.world.bodies.len() != count
            || self.world.colliders.len() != count
            || self.world.gravity != Vector::ZERO
    }

    pub(crate) fn resume_disturbed_coast(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        if let Some(coast) = &self.free_coast
            && self.coast_disturbed(coast.len())
        {
            self.switch_articulation(coast.root, ArticulationTier::Dynamic, dt)?;
        }
        Ok(())
    }
}
