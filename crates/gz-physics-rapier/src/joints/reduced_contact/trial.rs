// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Transactional native subintervals for an isolated passive contact tree.
use super::*;
use std::ops::ControlFlow;

/// One exclusive native owner and a warm trial world. Discarding it on any
/// numerical or contact refusal retains all state of the published world.
pub struct ArticulatedNativeInterval<'a> {
    owner: &'a mut RapierBackend,
    trial: RapierBackend,
    bodies: Vec<BodyId>,
    history: Option<WarmHistory>,
}
struct WarmHistory {
    start: RapierBackend,
    durations: Vec<Duration>,
}
impl RapierBackend {
    pub(in crate::joints) fn copy_native_world(w: &PhysicsWorld) -> PhysicsWorld {
        PhysicsWorld {
            gravity: w.gravity,
            integration_parameters: w.integration_parameters,
            islands: w.islands.clone(),
            broad_phase: w.broad_phase.clone(),
            narrow_phase: w.narrow_phase.clone(),
            bodies: w.bodies.clone(),
            colliders: w.colliders.clone(),
            impulse_joints: w.impulse_joints.clone(),
            multibody_joints: w.multibody_joints.clone(),
            soft_bodies: w.soft_bodies.clone(),
            // Rapier documents all three pipelines as temporary computation
            // workspaces. No manifolds, coordinates or multipliers live there.
            ..PhysicsWorld::default()
        }
    }

    pub(in crate::joints) fn copy_passive_interval(&self) -> Result<Self, PhysicsError> {
        let w = &self.world;
        let mut result = Self::new_multifidelity(w.gravity.to_array())?;
        result.world = Self::copy_native_world(w);
        result.welded_projection = self.welded_projection.as_ref().map(|p| p.duplicate());
        result.welded_exclusion = self.welded_exclusion;
        result.bodies = self.bodies.clone();
        result.joints = self.joints.clone();
        result.next_id = self.next_id;
        result.fleet_clock = self.fleet_clock;
        result.fleet_dispatch = fleet_isolation::Dispatch::Disabled;
        result.allow_isolation = self.allow_isolation;
        result.max_boundary_spin = self.max_boundary_spin;
        Ok(result)
    }
    pub fn begin_articulated_material_interval(
        &mut self,
        bodies: &[BodyId],
    ) -> Result<ArticulatedNativeInterval<'_>, PhysicsError> {
        let Some(&first) = bodies.first() else {
            return Err(PhysicsError::InvalidValue);
        };
        if bodies.len() > MAX_ARTICULATED_CONTACT_SITES
            || matches!(self.fleet_clock, FleetClock::Translating(_))
            || self.curves.authored()
            || !self.held_wrenches.is_empty()
            || !self.pending_wrenches.is_empty()
            || !self.joints.efforts.is_empty()
            || !self.spinning_assemblies.is_empty()
            || self.free_coast.is_some()
            || self.conserved_rotor.is_some()
            || !self.fleet_resume_positions.is_empty()
            || matches!(self.fleet_dispatch, fleet_isolation::Dispatch::Active(_))
        {
            return Err(unsupported(
                "recurrent material requires an isolated passive native tree with no external commands",
            ));
        }
        #[cfg(feature = "partition-checkpoint")]
        if self.pair_observer.is_some()
            || self.grasp_payload_observer.is_some()
            || self.native_profile.is_some()
            || self.join_queries_pending
        {
            return Err(unsupported(
                "recurrent material has one force and contact owner",
            ));
        }
        let handle = self.bodies.get(&first).ok_or(PhysicsError::MissingBody)?.0;
        let id = self
            .world
            .multibody_joints
            .rigid_body_link(handle)
            .ok_or(PhysicsError::InvalidJoint)?;
        let tree = self
            .world
            .multibody_joints
            .get_multibody(id.multibody)
            .ok_or(PhysicsError::InvalidJoint)?;
        if tree.num_links() != self.world.bodies.len()
            || !(2..=MAX_LINKS).contains(&tree.num_links())
        {
            return Err(unsupported(
                "the complete recurrent native world must be one passive articulation",
            ));
        }
        let sites = bodies
            .iter()
            .map(|&body| ArticulatedContactSite {
                body,
                local_point: [0.; 3],
            })
            .collect::<Vec<_>>();
        for &b in bodies {
            self.articulated_sphere_geometry(b)?;
        }
        self.prepare_articulated_contacts(&sites)?;
        let trial = self.copy_passive_interval()?;
        Ok(ArticulatedNativeInterval {
            owner: self,
            trial,
            bodies: bodies.to_vec(),
            history: None,
        })
    }
}
impl ArticulatedNativeInterval<'_> {
    /// A nested trial starts at the exact current native cut. Its commit
    /// advances only this enclosing, still-unpublished interval.
    pub fn branch(&mut self) -> Result<ArticulatedNativeInterval<'_>, PhysicsError> {
        let trial = self.trial.copy_passive_interval()?;
        self.history = None;
        Ok(ArticulatedNativeInterval {
            owner: &mut self.trial,
            trial,
            bodies: self.bodies.clone(),
            history: None,
        })
    }
    pub fn sphere_geometry(&self, id: BodyId) -> Result<([f64; 3], f64), PhysicsError> {
        self.trial.articulated_sphere_geometry(id)
    }
    pub fn geometry(&self) -> Result<Vec<([f64; 3], f64)>, PhysicsError> {
        self.bodies
            .iter()
            .map(|id| self.trial.articulated_sphere_geometry(*id))
            .collect()
    }
    pub fn geometry_after(&self, dt: Duration) -> Result<Vec<([f64; 3], f64)>, PhysicsError> {
        if dt.is_zero() {
            return self.geometry();
        }
        let mut next = self.trial.copy_passive_interval()?;
        next.step(dt)?;
        self.bodies
            .iter()
            .map(|id| next.articulated_sphere_geometry(*id))
            .collect()
    }
    /// Continue accepts the numerical successor. Break or a callback error
    /// reconstructs the last accepted warm input, including native joint
    /// caches, by replaying only the accepted durations after the checkpoint.
    pub fn inspect_step<B, C, E: From<PhysicsError>>(
        &mut self,
        dt: Duration,
        inspect: impl FnOnce(&[([f64; 3], f64)]) -> Result<ControlFlow<B, C>, E>,
    ) -> Result<ControlFlow<B, C>, E> {
        if self
            .history
            .as_ref()
            .is_none_or(|h| h.durations.len() >= 32)
        {
            self.history = Some(WarmHistory {
                start: self.trial.copy_passive_interval()?,
                durations: Vec::with_capacity(32),
            });
        }
        let result = self
            .trial
            .step(dt)
            .and_then(|()| self.geometry())
            .map_err(E::from)
            .and_then(|p| inspect(&p));
        if matches!(result, Ok(ControlFlow::Continue(_))) {
            self.history
                .as_mut()
                .expect("recorded warm start")
                .durations
                .push(dt);
        } else {
            let h = self.history.take().expect("recorded warm start");
            self.trial = h.start;
            for elapsed in h.durations {
                self.trial.step(elapsed)?;
            }
        }
        result
    }
    pub fn state(&self, id: BodyId) -> Option<BodyState> {
        self.trial.state(id)
    }
    pub fn step(&mut self, dt: Duration) -> Result<(), PhysicsError> {
        self.history = None;
        self.trial.step(dt)
    }
    pub fn prepare(
        &mut self,
        sites: &[ArticulatedContactSite],
    ) -> Result<ArticulatedContactTransaction<'_>, PhysicsError> {
        if sites.iter().any(|s| !self.bodies.contains(&s.body)) {
            return Err(PhysicsError::InvalidValue);
        }
        self.history = None;
        self.trial.prepare_articulated_contacts(sites)
    }
    pub fn commit(self) {
        self.owner.world = self.trial.world;
        self.owner.welded_projection = self.trial.welded_projection;
        self.owner.welded_exclusion = self.trial.welded_exclusion;
        self.owner.joints = self.trial.joints;
        self.owner.fleet_clock = self.trial.fleet_clock;
        self.owner.fleet_dispatch = self.trial.fleet_dispatch;
        self.owner.max_boundary_spin = self.trial.max_boundary_spin;
    }
}
