// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Execute a selected number of native solves inside one private Bevy physics tick.
use super::*;
use bevy_ecs::prelude::{Query, Res, ResMut, Resource};
use gz_sim_core::{Phase, StepFault, UpdateInfo};

/// Loads from the first nine solves. Adding the final native contact receipt
/// reconstructs the public total after a barrier, edit, migration or recovery.
#[derive(Clone, Default, Debug, PartialEq, Serialize, Deserialize)]
pub(super) struct ContactPrefix {
    pub(super) pairs: Vec<ContactImpulse>,
    pub(super) supports: Vec<GroundSupport>,
}
impl ContactPrefix {
    pub(super) fn is_empty(&self) -> bool {
        self.pairs.is_empty() && self.supports.is_empty()
    }

    pub(super) fn valid_for(&self, s: &Snapshot) -> bool {
        if self.is_empty() {
            return true;
        }
        let valid_load = |x: f64| x.is_finite() && x >= 0.;
        let member = |id| s.bodies.binary_search_by_key(&id, |b| b.id).is_ok();
        s.impulses.iter().all(|p| valid_load(p.normal_impulse))
            && s.supports.iter().all(|p| valid_load(p.normal_impulse))
            && s.tick > 0
            && s.step == SimulationStep::Ms10
            && s.contact_history
            && self.pairs.len() <= s.contacts
            && self.supports.len() <= s.supports.len()
            && (self.supports.is_empty() || s.ground.is_some())
            && self
                .pairs
                .windows(2)
                .all(|p| (p[0].a, p[0].b) < (p[1].a, p[1].b))
            && self.supports.windows(2).all(|p| p[0].body < p[1].body)
            && self
                .pairs
                .iter()
                .all(|p| p.a < p.b && member(p.a) && member(p.b) && valid_load(p.normal_impulse))
            && self
                .supports
                .iter()
                .all(|p| member(p.body) && valid_load(p.normal_impulse))
    }
    pub(super) fn retain(&mut self, mut keep: impl FnMut(u64) -> bool) {
        self.pairs.retain(|p| keep(p.a) && keep(p.b));
        self.supports.retain(|p| keep(p.body));
    }

    pub(super) fn add_to(&self, frame: &mut Snapshot) {
        for prior in &self.pairs {
            match frame
                .impulses
                .binary_search_by_key(&(prior.a, prior.b), |p| (p.a, p.b))
            {
                Ok(i) => frame.impulses[i].normal_impulse += prior.normal_impulse,
                Err(i) => frame.impulses.insert(i, prior.clone()),
            }
        }
        for prior in &self.supports {
            match frame.supports.binary_search_by_key(&prior.body, |p| p.body) {
                Ok(i) => frame.supports[i].normal_impulse += prior.normal_impulse,
                Err(i) => frame.supports.insert(i, prior.clone()),
            }
        }
        frame.contacts = frame.impulses.len();
    }
}

#[derive(Resource)]
struct RefinedSolve {
    // The caller staged these loads on the native bodies for the first solve.
    repeat: Vec<(BodyId, [f64; 3], [f64; 3])>,
    ground: bool,
    saw_history: bool,
    prefix: ContactPrefix,
    end: ContactPrefix,
}

pub(super) fn validate_substeps(step: SimulationStep, substeps: u8) -> Result<(), WorldError> {
    if substeps == 1 || (step == SimulationStep::Ms10 && substeps == 10) {
        Ok(())
    } else {
        Err(PhysicsError::InvalidValue.into())
    }
}

impl Island {
    pub(super) fn install_step(sim: &mut Simulation, backend: MultiFidelityBackend) {
        // This private world has no external scene systems. Reset/hydration
        // builds a fresh Island. The public clock commits just once per tick.
        sim.world_mut()
            .insert_resource(PhysicsBackend::from(backend));
        sim.add_systems(Phase::PhysicsStep, step_system);
    }

    pub(super) fn stage_refinement(&mut self, inputs: &[BodyWrench]) {
        let world = self.sim.world();
        let repeat = inputs
            .iter()
            .filter_map(|w| {
                let entity = *self.index.get(&w.id)?;
                let handle = world.get::<Binding>(entity).unwrap().native;
                Some((handle, w.force, w.torque))
            })
            .collect();
        self.sim.world_mut().insert_resource(RefinedSolve {
            repeat,
            ground: self.committed.ground.is_some(),
            saw_history: false,
            prefix: ContactPrefix::default(),
            end: ContactPrefix::default(),
        });
    }

    pub(super) fn commit_refinement(&mut self) -> Option<ContactPrefix> {
        if let Some(solve) = self.sim.world_mut().remove_resource::<RefinedSolve>() {
            self.prefix = solve.prefix;
            self.committed.contact_history |= solve.saw_history;
            Some(solve.end)
        } else {
            self.prefix = ContactPrefix::default();
            None
        }
    }
}

fn step_system(
    time: Res<UpdateInfo>,
    fault: Res<StepFault>,
    mut backend: ResMut<PhysicsBackend>,
    refined: Option<ResMut<RefinedSolve>>,
    bindings: Query<&Binding>,
) {
    let result = match refined {
        None => backend.step(time.dt),
        Some(mut plan) => {
            let ids = bindings.iter().map(|b| (b.native, b.id)).collect();
            plan.run(
                backend.downcast_mut::<MultiFidelityBackend>().unwrap(),
                &ids,
            )
        }
    };
    if let Err(e) = result {
        fault.report(e);
    }
}

impl RefinedSolve {
    fn run(
        &mut self,
        backend: &mut MultiFidelityBackend,
        ids: &HashMap<BodyId, u64>,
    ) -> Result<(), PhysicsError> {
        let mut pairs: BTreeMap<(u64, u64), f64> = BTreeMap::new();
        let mut supports: BTreeMap<u64, f64> = BTreeMap::new();
        let mut rigid_receipt = Vec::new();
        let mut ground_receipt = Vec::new();
        for microstep in 0..10 {
            if microstep != 0 {
                for &(id, force, torque) in &self.repeat {
                    backend.add_world_wrench(id, force, torque)?;
                }
            }
            backend.step(Duration::from_millis(1))?;
            let last = microstep == 9;
            backend.read_partition_contacts(
                &mut rigid_receipt,
                self.ground.then_some(&mut ground_receipt),
            );
            for p in &rigid_receipt {
                self.saw_history = true;
                if p.active {
                    let (a, b) = (ids[&p.a], ids[&p.b]);
                    let pair = (a.min(b), a.max(b));
                    checked_impulse(&mut pairs, pair, p.normal_impulse, last)?;
                    if last {
                        self.end.pairs.push(ContactImpulse {
                            a: pair.0,
                            b: pair.1,
                            normal_impulse: p.normal_impulse,
                        });
                    }
                }
            }
            for p in &ground_receipt {
                self.saw_history = true;
                if p.active {
                    let body = ids[&p.body];
                    checked_impulse(&mut supports, body, p.normal_impulse, last)?;
                    if last {
                        self.end.supports.push(GroundSupport {
                            body,
                            normal_impulse: p.normal_impulse,
                        });
                    }
                }
            }
        }
        self.prefix.pairs = pairs
            .into_iter()
            .map(|((a, b), normal_impulse)| ContactImpulse {
                a,
                b,
                normal_impulse,
            })
            .collect();
        self.prefix.supports = supports
            .into_iter()
            .map(|(body, normal_impulse)| GroundSupport {
                body,
                normal_impulse,
            })
            .collect();
        backend.finish_refined_partition_step();
        Ok(())
    }
}

// Validate all ten contributions before the Bevy barrier commits. The final
// contribution stays in Rapier and is independently read back on hydration.
fn checked_impulse<K: Ord>(
    prefix: &mut BTreeMap<K, f64>,
    key: K,
    impulse: f64,
    final_solve: bool,
) -> Result<(), PhysicsError> {
    let total = prefix.get(&key).copied().unwrap_or(0.) + impulse;
    if !total.is_finite() || impulse < 0. {
        return Err(PhysicsError::InvalidValue);
    }
    if !final_solve {
        prefix.insert(key, total);
    }
    Ok(())
}

#[cfg(test)]
#[path = "../../tests/private/refined_barrier.rs"]
mod tests;
