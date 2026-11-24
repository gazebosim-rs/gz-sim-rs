// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local staggered rigid/material exchange under the public Bevy clock.
use super::*;
use gz_physics::NativeStepGrid;
use gz_physics_xpbd::{
    ParticlePadImpulse, SelfContactImpulse, SelfContactLoad, SelfContactParticleImpulse,
    ShellConstraintImpulse,
};

pub(super) fn admit(
    root: roxmltree::Node,
    bodies: &BTreeMap<String, SoftBody>,
    surfaces: &[Surface],
    dt: Duration,
) -> Result<u32, String> {
    let Some(raw) = root.attribute("rigid_exchanges") else {
        return Ok(1);
    };
    let n: u32 = raw.parse().map_err(|_| "invalid rigid_exchanges")?;
    if !(1..=256).contains(&n) {
        return Err("rigid_exchanges must be between 1 and 256".into());
    }
    if n == 1 {
        return Ok(n);
    }
    if root.attributes().any(|a| a.name().starts_with("worker_"))
        || bodies.len() != 1
        || !surfaces.iter().any(|s| {
            matches!(
                s,
                Surface::Rigid {
                    mobility: Some(ReceiverMass::ArticulatedWrench { .. }),
                    ..
                }
            )
        })
        || !surfaces.iter().all(|s| {
            matches!(
                s,
                Surface::Plane { .. }
                    | Surface::Rigid {
                        static_box_pose: Some(_),
                        ..
                    }
                    | Surface::Rigid {
                        mobility: Some(ReceiverMass::ArticulatedWrench { .. }),
                        ..
                    }
            )
        })
    {
        return Err("rigid_exchanges requires one local material, articulated pads and optional fixed supports".into());
    }
    if bodies
        .values()
        .any(|b| u128::from(b.config().substeps) * u128::from(n) > dt.as_nanos())
    {
        return Err("material substeps exceed the rigid exchange interval".into());
    }
    Ok(n)
}
pub(crate) fn enabled(world: &World) -> bool {
    world
        .get_resource::<Deformables>()
        .is_some_and(|s| s.exchanges > 1)
}
pub(crate) fn step(world: &mut World) {
    let info = *world.resource::<UpdateInfo>();
    if info.dt.is_zero() {
        return;
    }
    super::start_positions(world);
    if world.resource::<StepFault>().error().is_some() {
        return;
    }
    let result = world.resource_scope(|world, mut soft: Mut<Deformables>| {
        let mut physics = world.resource_mut::<PhysicsBackend>();
        soft.with_forces(|soft| solve(soft, &mut physics, info))
    });
    if let Err(error) = result {
        world.resource::<StepFault>().report(error);
    }
}

#[derive(Default)]
struct Integrated {
    reactions: Vec<BoundaryImpulse>,
    nodes: BTreeMap<usize, SelfContactParticleImpulse>,
    pads: BTreeMap<(usize, usize), ParticlePadImpulse>,
    self_loads: Vec<SelfContactLoad>,
    structural: Vec<ShellConstraintImpulse>,
    self_impulse: SelfContactImpulse,
    path: u32,
    intervals: u32,
    refinements: u32,
}
fn add(target: &mut [f64; 3], v: [f64; 3]) {
    for (x, delta) in target.iter_mut().zip(v) {
        *x += delta;
    }
}
impl Integrated {
    fn sample(&mut self, soft: &Deformables, origins: &[Option<BodyState>]) {
        // Admission guarantees one material and excludes curved/mobile free receivers.
        let body = soft.current.first_key_value().unwrap().1;
        self.reactions
            .resize_with(soft.surfaces.len(), Default::default);
        for (i, r) in self.reactions.iter_mut().enumerate() {
            r.boundary = i;
        }
        for j in body.last_boundary_impulses() {
            let slot = soft.active_slots[j.boundary];
            let shift = soft.starts[slot]
                .zip(origins[slot])
                .map_or(DVec3::ZERO, |(here, origin)| {
                    DVec3::from(here.position) - DVec3::from(origin.position)
                });
            let total = &mut self.reactions[slot];
            add(&mut total.linear, j.linear);
            add(
                &mut total.angular,
                (DVec3::from(j.angular) + shift.cross(j.linear.into())).into(),
            );
        }
        for &j in body.last_pad_loads() {
            let slot = soft.active_slots[j.boundary];
            let p = self
                .pads
                .entry((slot, j.particle))
                .or_insert(ParticlePadImpulse {
                    boundary: slot,
                    particle: j.particle,
                    normal: [0.; 3],
                    tangent: [0.; 3],
                });
            add(&mut p.normal, j.normal);
            add(&mut p.tangent, j.tangent);
        }
        for &j in body.last_self_contact_particle_impulses() {
            let p = self
                .nodes
                .entry(j.particle)
                .or_insert(SelfContactParticleImpulse {
                    particle: j.particle,
                    ..Default::default()
                });
            add(&mut p.normal, j.normal);
            add(&mut p.tangent, j.tangent);
        }
        self.self_loads
            .extend_from_slice(body.last_self_contact_loads());
        let j = body.last_self_contact_impulse();
        self.self_impulse.normal += j.normal;
        self.self_impulse.tangent += j.tangent;
        self.self_impulse.friction_work += j.friction_work;
        let structural = body.last_shell_constraint_impulses();
        self.structural.resize(structural.len(), Default::default());
        for (sum, j) in self.structural.iter_mut().zip(structural) {
            sum.distance += j.distance;
            sum.bending += j.bending;
            sum.bend_damping += j.bend_damping;
        }
        self.path += body.last_path_substeps();
        self.refinements = self.refinements.max(body.last_sweep_refinements());
        self.intervals += body
            .last_local_shell_intervals()
            .unwrap_or_else(|| body.config().substeps << body.last_sweep_refinements());
    }
    fn publish(self, soft: &mut Deformables, origins: &[Option<BodyState>], dt: Duration) {
        let s = soft.samples.first_entry().unwrap().into_mut();
        s.exchanges = soft.exchanges;
        s.local_shell_intervals = Some(self.intervals);
        s.path_substeps = self.path;
        s.sweep_refinements = self.refinements;
        s.self_impulse = self.self_impulse;
        s.self_loads = self.self_loads;
        s.self_particle_impulses = self.nodes.into_values().collect();
        s.pad_loads = self.pads.into_values().collect();
        s.exchange_shell = self.structural;
        s.reactions = self.reactions;
        soft.pad_wrenches.clear();
        for (&slot, receiver) in soft.active_slots.iter().zip(&soft.receivers) {
            if let (Surface::Rigid { link, .. }, Some((_, end)), Some(start)) =
                (soft.surfaces[slot], receiver, origins[slot])
            {
                let j = s.reactions[slot];
                let shift = DVec3::from(start.position) - DVec3::from(end.position);
                let t = dt.as_secs_f64();
                soft.pad_wrenches.insert(
                    link,
                    ArticulatedPadWrench {
                        force_n: (DVec3::from(j.linear) / t).into(),
                        torque_nm: ((DVec3::from(j.angular) + shift.cross(j.linear.into())) / t)
                            .into(),
                    },
                );
            }
        }
    }
}
fn solve(
    soft: &mut Deformables,
    physics: &mut PhysicsBackend,
    info: UpdateInfo,
) -> Result<(), CouplingError> {
    let origins = soft.starts.clone();
    let mut integral = Integrated::default();
    let mut held = physics
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or(CouplingError::Binding)?
        .exchange_inputs(info.dt, soft.exchanges)?;
    let grid = NativeStepGrid::new(info.dt, soft.exchanges)?;
    for (index, h) in grid.intervals().enumerate() {
        if index > 0 {
            physics
                .downcast_mut::<MultiFidelityBackend>()
                .ok_or(CouplingError::Binding)?
                .repeat_exchange_inputs(&held)?;
        }
        // A removed pad retires its pending impulse. A surviving pad receives the
        // previous exchange's full impulse, even when adjacent intervals differ by a nanosecond.
        for (surface, start) in soft.surfaces.iter().zip(&mut soft.starts) {
            if let Surface::Rigid { body, .. } = *surface {
                if start.is_some() && index > 0 {
                    *start = Some(physics.state(body).ok_or(CouplingError::Binding)?);
                }
                if let Some((j, k)) = soft.pending_pad_impulses.remove(&body)
                    && start.is_some()
                {
                    physics.add_world_wrench(
                        body,
                        (j / h.as_secs_f64()).into(),
                        (k / h.as_secs_f64()).into(),
                    )?;
                }
            }
        }
        physics.step(h)?;
        physics
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .observe_exchange(&mut held)?;
        soft.advance(physics, h)?;
        integral.sample(soft, &origins);
    }
    physics
        .downcast_mut::<MultiFidelityBackend>()
        .unwrap()
        .finish_exchanges(held)?;
    soft.publish(info.sim_time, info.epoch);
    integral.publish(soft, &origins, info.dt);
    Ok(())
}

#[cfg(test)]
mod tests;
