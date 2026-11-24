// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Aggregate dynamics of a fully welded floating assembly.
use super::*;

/// World-frame SI properties at one committed boundary. Inertia is
/// column-major about `center`. The held wrench excludes gravity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldedAssemblyState {
    pub root: BodyId,
    pub links: usize,
    pub mass: f64,
    pub center: [f64; 3],
    pub inertia_columns: [[f64; 3]; 3],
    pub velocity: [f64; 3],
    pub spin: [f64; 3],
    pub linear_momentum: [f64; 3],
    pub angular_momentum: [f64; 3],
    pub force: [f64; 3],
    pub torque: [f64; 3],
}

impl RapierBackend {
    /// Returns the same COM, momenta, inertia and pending wrench for any
    /// member of the floating assembly. The per-link loads remain pending.
    pub fn welded_assembly_state(
        &self,
        member: BodyId,
    ) -> Result<WeldedAssemblyState, PhysicsError> {
        let handle = self.bodies.get(&member).ok_or(PhysicsError::MissingBody)?.0;
        let unsupported = || PhysicsError::Unsupported {
            engine: "rapier",
            capability: "aggregate dynamics require a floating assembly of authored fixed joints",
        };
        let index = self
            .world
            .multibody_joints
            .rigid_body_link(handle)
            .ok_or_else(unsupported)?
            .multibody;
        let tree = self
            .world
            .multibody_joints
            .get_multibody(index)
            .ok_or_else(unsupported)?;
        let root = &self.world.bodies[tree.root().rigid_body_handle()];
        if tree.ndofs() != 6 || tree.num_links() < 2 || !root.is_dynamic() {
            return Err(unsupported());
        }
        let count = self
            .joints
            .records
            .values()
            .filter(|j| {
                j.initial.authored_weld()
                    && self
                        .world
                        .multibody_joints
                        .rigid_body_link(self.bodies[&j.initial.child].0)
                        .is_some_and(|l| l.multibody == index)
            })
            .count();
        if count + 1 != tree.num_links() {
            return Err(unsupported());
        }
        let (mass, center) = self
            .locked_mass_center(tree, root)
            .ok_or_else(unsupported)?;
        let columns =
            [Vector::X, Vector::Y, Vector::Z].map(|a| self.locked_inertia_action(tree, center, a));
        let mut linear = Vector::ZERO;
        let mut angular = Vector::ZERO;
        let mut force = Vector::ZERO;
        let mut torque = Vector::ZERO;
        for link in tree.links() {
            let b = &self.world.bodies[link.rigid_body_handle()];
            let props = &b.mass_properties().local_mprops;
            let axes = *b.rotation() * props.principal_inertia_local_frame;
            let arm = b.center_of_mass() - center;
            let p = b.mass() * b.linvel();
            linear += p;
            angular += axes * (props.inv_principal_inertia.recip() * (axes.inverse() * b.angvel()))
                + arm.cross(p);
            force += b.user_force();
            torque += b.user_torque() + arm.cross(b.user_force());
        }
        if !columns
            .into_iter()
            .chain([linear, angular, force, torque])
            .all(|v| v.is_finite())
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(WeldedAssemblyState {
            root: BodyId::from_raw(root.user_data as u64),
            links: tree.num_links(),
            mass,
            center: center.to_array(),
            inertia_columns: columns.map(|v| v.to_array()),
            velocity: (linear / mass).to_array(),
            spin: root.angvel().to_array(),
            linear_momentum: linear.to_array(),
            angular_momentum: angular.to_array(),
            force: force.to_array(),
            torque: torque.to_array(),
        })
    }
}
