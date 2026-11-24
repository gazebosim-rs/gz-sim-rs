// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// Body wrenches and joint efforts sampled after public input preparation.
/// Velocities and impulses have already changed native state and are never replayed.
pub struct ExchangeInputs {
    owner: Arc<()>,
    observed: u32,
    count: u32,
    contacts: BTreeMap<(BodyId, BodyId), (RigidContactObservation, NormalReaction)>,
    wrenches: Vec<(BodyId, Vector, Vector)>,
    efforts: BTreeMap<JointId, f64>,
    angular_efforts: BTreeMap<JointId, Vector>,
}
impl RapierBackend {
    pub fn exchange_inputs(
        &self,
        dt: Duration,
        count: u32,
    ) -> Result<ExchangeInputs, PhysicsError> {
        if dt.is_zero() {
            return Err(PhysicsError::InvalidValue);
        }
        self.validate_contact_interval(dt, count, 1)?;
        if !self.parked_robots.is_empty()
            || !self.held_wrenches.is_empty()
            || !self.held_supports.is_empty()
        {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "local material exchanges with parked native constraints",
            });
        }
        Ok(ExchangeInputs {
            owner: self.exchange_epoch.clone(),
            observed: 0,
            count,
            contacts: BTreeMap::new(),
            wrenches: self
                .pending_wrenches
                .iter()
                .map(|&id| {
                    let rb = &self.world.bodies[self.bodies[&id].0];
                    (id, rb.user_force(), rb.user_torque())
                })
                .collect(),
            efforts: self.joints.efforts.clone(),
            angular_efforts: self.joints.angular_efforts.clone(),
        })
    }
    /// Reapply only after the previous positive native interval consumed its inputs.
    pub fn repeat_exchange_inputs(&mut self, held: &ExchangeInputs) -> Result<(), PhysicsError> {
        if !Arc::ptr_eq(&self.exchange_epoch, &held.owner)
            || held.observed == 0
            || held.observed == held.count
            || !self.joints.efforts.is_empty()
            || !self.joints.angular_efforts.is_empty()
            || held
                .efforts
                .keys()
                .chain(held.angular_efforts.keys())
                .any(|id| !self.joints.records.contains_key(id))
        {
            return Err(PhysicsError::InvalidJoint);
        }
        for &(id, f, t) in &held.wrenches {
            self.add_world_wrench(id, f.to_array(), t.to_array())?;
        }
        self.joints.efforts.clone_from(&held.efforts);
        self.joints
            .angular_efforts
            .clone_from(&held.angular_efforts);
        Ok(())
    }
}

impl RapierBackend {
    /// Accumulate actual solved rigid-pair loads before the next material exchange.
    pub fn observe_exchange(&self, held: &mut ExchangeInputs) -> Result<(), PhysicsError> {
        if !Arc::ptr_eq(&self.exchange_epoch, &held.owner) || held.observed == held.count {
            return Err(PhysicsError::InvalidValue);
        }
        held.observed += 1;
        let last = held.observed == held.count;
        let lookup = self.contact_body_lookup();
        for pair in self.world.narrow_phase.contact_pairs() {
            let (Some(a), Some(b)) = (lookup(pair.collider1), lookup(pair.collider2)) else {
                continue;
            };
            let key = (a.min(b), a.max(b));
            let delta = crate::partition::manifold_reaction(pair, a == key.0);
            let (scalar, vector) = held.contacts.entry(key).or_insert_with(|| {
                (
                    RigidContactObservation {
                        a: key.0,
                        b: key.1,
                        active: false,
                        normal_impulse: 0.,
                    },
                    NormalReaction::default(),
                )
            });
            scalar.active |= pair.has_any_active_contact();
            scalar.normal_impulse += delta.magnitude;
            vector.add(delta, last);
        }
        Ok(())
    }
    pub fn finish_exchanges(&mut self, held: ExchangeInputs) -> Result<(), PhysicsError> {
        if !Arc::ptr_eq(&self.exchange_epoch, &held.owner) || held.observed != held.count {
            return Err(PhysicsError::InvalidValue);
        }
        self.exchange_reactions = Some(held.contacts);
        Ok(())
    }
}
