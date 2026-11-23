// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BodyWrench {
    pub id: u64,
    /// World force (N) at the center of mass for one complete public step.
    pub force: [f64; 3],
    /// World torque (N m) for that same step.
    pub torque: [f64; 3],
}
impl PartialEq for BodyWrench {
    fn eq(&self, b: &Self) -> bool {
        self.id == b.id
            && self.force.map(f64::to_bits) == b.force.map(f64::to_bits)
            && self.torque.map(f64::to_bits) == b.torque.map(f64::to_bits)
    }
}
/// One physical interval in a uniform world acceleration field. User forces
/// act at the COM in world coordinates and are independent of gravity.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StepField {
    pub gravity: [f64; 3],
    pub wrenches: Vec<BodyWrench>,
}
impl PartialEq for StepField {
    fn eq(&self, other: &Self) -> bool {
        self.gravity.map(f64::to_bits) == other.gravity.map(f64::to_bits)
            && self.wrenches == other.wrenches
    }
}

/// Largest signed velocity change from a constant net force over one tick,
/// before a contact or native speed clamp changes the direction of travel.
#[derive(Clone, Copy)]
pub(crate) struct ForceMotion {
    pub id: u64,
    pub delta_velocity: [f64; 3],
    pub has_torque: bool,
}
pub(crate) fn validate_wrenches(
    step: SimulationStep,
    bodies: &[Body],
    inputs: &[BodyWrench],
) -> Result<Vec<ForceMotion>, WorldError> {
    field_motion(step, bodies, [0.; 3], inputs)
}

pub(crate) fn field_motion(
    step: SimulationStep,
    bodies: &[Body],
    gravity: [f64; 3],
    inputs: &[BodyWrench],
) -> Result<Vec<ForceMotion>, WorldError> {
    if !gravity.into_iter().all(f64::is_finite) {
        return Err(PhysicsError::InvalidValue.into());
    }
    if inputs.len() > MAX_PENDING_IMPULSES {
        return Err(WorldError::Pending);
    }
    let mut totals = BTreeMap::<u64, [f64; 6]>::new();
    for w in inputs {
        let b = bodies
            .iter()
            .find(|b| b.id == w.id)
            .ok_or(WorldError::Missing(w.id))?;
        if b.tier != Tier::Dynamic {
            return Err(PhysicsError::Unsupported {
                engine: "distributed",
                capability: "wrench on a prescribed body",
            }
            .into());
        }
        let total = totals.entry(w.id).or_default();
        for (t, add) in total.iter_mut().zip(w.force.into_iter().chain(w.torque)) {
            *t += add;
            if !t.is_finite() {
                return Err(PhysicsError::InvalidValue.into());
            }
        }
        // Keep trial velocities representable for this entire native interval.
        let dt_over_mass = step.duration().as_secs_f64() / b.mass;
        if (0..3).any(|i| !(b.linear[i] + total[i] * dt_over_mass).is_finite()) {
            return Err(PhysicsError::InvalidValue.into());
        }
    }
    let dt = step.duration().as_secs_f64();
    let mut motion = Vec::new();
    for b in bodies.iter().filter(|b| b.tier == Tier::Dynamic) {
        let zero = [0.; 6];
        let net = totals.get(&b.id).unwrap_or(&zero);
        let delta = gz_physics_multifidelity::partition_velocity_change(
            b.mass,
            gravity,
            [net[0], net[1], net[2]],
            dt,
        );
        if !delta.into_iter().all(f64::is_finite) {
            return Err(PhysicsError::InvalidValue.into());
        }
        let has_torque = net[3..] != [0.; 3];
        if delta != [0.; 3] || has_torque {
            motion.push(ForceMotion {
                id: b.id,
                delta_velocity: delta,
                has_torque,
            });
        }
    }
    Ok(motion)
}

impl Island {
    /// Advance an uninterrupted native world with forces held across every
    /// substep. The empty input advances an ordinary unforced barrier.
    pub fn step_with_wrenches(&mut self, inputs: &[BodyWrench]) -> Result<(), WorldError> {
        self.step_refined_with_wrenches(inputs, 1)
    }

    /// Advance one public tick using one native solve, or ten 1 ms solves
    /// for a 10 ms tick. COM impulses apply once; held wrenches span the tick.
    pub fn step_refined_with_wrenches(
        &mut self,
        inputs: &[BodyWrench],
        substeps: u8,
    ) -> Result<(), WorldError> {
        self.step_refined_in_field([0.; 3], inputs, substeps)
    }

    /// Run the same public barrier with a uniform gravity field held for
    /// each internal solve. All input validation precedes force application.
    pub fn step_refined_in_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        substeps: u8,
    ) -> Result<(), WorldError> {
        self.healthy()?;
        self.committed
            .tick
            .checked_add(1)
            .ok_or(WorldError::TimeOverflow)?;
        validate_substeps(self.committed.step, substeps)?;
        field_motion(self.committed.step, &self.committed.bodies, gravity, inputs)?;
        for w in inputs {
            self.add_step_wrench(w)?;
        }
        if substeps == 10 {
            self.stage_refinement(inputs);
        }
        self.advance_native_field(gravity)
    }
    pub(super) fn advance_native_field(&mut self, gravity: [f64; 3]) -> Result<(), WorldError> {
        if gravity == [0.; 3] {
            return self.step();
        }
        let set = |s: &mut Island, value| {
            s.sim
                .world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .set_partition_gravity(value)
        };
        set(self, gravity)?;
        let result = self.step();
        // A complete native image is always a barrier between field commands.
        // The next interval supplies its own gravity, including after restart.
        set(self, [0.; 3])?;
        result
    }

    /// Apply native force and torque across this solve's substeps. Source-
    /// ordered contributions accumulate before the native integration starts.
    pub(crate) fn add_step_wrench(&mut self, w: &BodyWrench) -> Result<(), WorldError> {
        self.healthy()?;
        let &e = self.index.get(&w.id).ok_or(WorldError::Missing(w.id))?;
        let world = self.sim.world_mut();
        let native = world.get::<Binding>(e).unwrap().native;
        world
            .resource_mut::<PhysicsBackend>()
            .add_world_wrench(native, w.force, w.torque)?;
        Ok(())
    }
}
