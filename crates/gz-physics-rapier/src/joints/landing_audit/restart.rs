// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native cut readback and independent continuations of a passive, fully unloaded tree.
use super::*;

#[derive(Debug, PartialEq)]
pub struct RestartPairHistory {
    pub pair_present: bool,
    pub manifolds: usize,
    pub solver_points: usize,
    pub native_points: usize,
    /// Sum of pointwise absolute normal, tangent magnitude and absolute torsional seeds.
    pub absolute_seeds: [f64; 3],
    pub discard_pending: bool,
}

#[derive(Debug, PartialEq)]
pub struct LandingRestartState {
    pub links: [BodyState; 3],
    pub generalized_velocity: Vec<f64>,
    pub joint_coordinates: Vec<Vec<f64>>,
    pub hinge: JointState,
    pub motor_enabled: bool,
    pub pending_hinge_effort: Option<f64>,
    pub gravity: [f64; 3],
    pub user_forces: [[f64; 3]; 3],
    pub user_torques: [[f64; 3]; 3],
    pub last_native_dt_s: f64,
    /// Sole–pad, upper–pad, then sole–upper.
    pub history: [RestartPairHistory; 3],
}

impl RapierBackend {
    pub fn airborne_restart_state(
        &self,
        root: BodyId,
        pad: BodyId,
    ) -> Result<LandingRestartState, PhysicsError> {
        let ids = self.landing_layout(root, pad)?;
        let native = &self.world;
        let bodies = ids.map(|id| &native.bodies[self.bodies[&id].0]);
        let index = self.articulation_index(root)?;
        let tree = native
            .multibody_joints
            .get_multibody(index)
            .ok_or_else(unsupported)?;
        let hinge_id = *self.joints.records.keys().next().ok_or_else(unsupported)?;
        let history = |i: usize, j: usize| -> Result<RestartPairHistory, PhysicsError> {
            let pair = native
                .narrow_phase
                .contact_pair(bodies[i].colliders()[0], bodies[j].colliders()[0]);
            let mut result = RestartPairHistory {
                pair_present: pair.is_some(),
                manifolds: 0,
                solver_points: 0,
                native_points: 0,
                absolute_seeds: [0.; 3],
                discard_pending: self.contact_warmstart_discard_pending(ids[i], ids[j])?,
            };
            if let Some(pair) = pair {
                for manifold in pair.manifolds() {
                    result.manifolds += 1;
                    result.native_points += manifold.points.len();
                    result.solver_points += manifold.data.solver_contacts.len();
                    for point in &manifold.points {
                        result.absolute_seeds[0] += point.data.warmstart_impulse.abs();
                        result.absolute_seeds[1] += point.data.warmstart_tangent_world.length();
                        result.absolute_seeds[2] += point.data.warmstart_twist_impulse.abs();
                    }
                }
            }
            Ok(result)
        };
        Ok(LandingRestartState {
            links: ids.map(|id| self.state(id).expect("qualified native link")),
            generalized_velocity: tree.generalized_velocity().iter().copied().collect(),
            joint_coordinates: tree.links().map(|l| l.joint.coords().to_vec()).collect(),
            hinge: self.joint_state(hinge_id).ok_or_else(unsupported)?,
            motor_enabled: !tree
                .link(1)
                .ok_or_else(unsupported)?
                .joint
                .data
                .motor_axes
                .is_empty(),
            pending_hinge_effort: self.pending_joint_effort(hinge_id),
            gravity: native.gravity.to_array(),
            user_forces: bodies.map(|body| body.user_force().to_array()),
            user_torques: bodies.map(|body| body.user_torque().to_array()),
            last_native_dt_s: native.integration_parameters.dt,
            history: [history(0, 2)?, history(1, 2)?, history(0, 1)?],
        })
    }

    pub fn copy_airborne_landing(
        &self,
        root: BodyId,
        pad: BodyId,
        cold: bool,
    ) -> Result<Self, PhysicsError> {
        let state = self.airborne_restart_state(root, pad)?;
        if !self.support_watch.is_inactive()
            || state.motor_enabled
            || state.pending_hinge_effort.is_some()
            || state.user_forces != [[0.; 3]; 3]
            || state.user_torques != [[0.; 3]; 3]
            || state
                .history
                .iter()
                .any(|p| p.discard_pending || p.absolute_seeds != [0.; 3])
        {
            return Err(unsupported());
        }
        let mut copy = self.copy_passive_interval()?;
        if cold {
            let w = &mut copy.world;
            w.broad_phase = Default::default();
            w.narrow_phase = NarrowPhase::new();
            // Reinsert all three actual colliders on the next solve. Marking their
            // identical world poses dirty preserves the multibody coordinates and
            // local collider attachments, including the authored inertial frames.
            for (_, collider) in w.colliders.iter_mut() {
                collider.set_position(*collider.position());
            }
        }
        Ok(copy)
    }
}
