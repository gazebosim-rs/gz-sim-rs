// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, RotationSpec};
use gz_physics_distributed::physics::{
    Body, BodyWrench, ContactImpulse, GroundPlane, GroundSupport, PendingImpulse, Tier,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::{collections::BTreeMap, error::Error, time::Duration};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const ZERO: [f64; 3] = [0.; 3];
/// A single native solver without worker, Bevy barrier, migration or archive logic.
pub struct Control {
    pub solver: MultiFidelityBackend,
    pub bodies: Vec<Body>,
    pub(super) handles: BTreeMap<u64, BodyId>,
}
impl Control {
    pub fn new(bodies: &[Body], ground: Option<GroundPlane>, gravity: [f64; 3]) -> Result<Self> {
        let mut solver = MultiFidelityBackend::new(ZERO)?;
        if let Some(g) = ground {
            solver.set_partition_plane(g.point(), g.normal(), g.friction())?;
        }
        let mut handles = BTreeMap::new();
        for b in bodies {
            let h = solver.spawn_partition_primitive(
                BodySpec {
                    shape: b.shape()?,
                    position: b.position,
                    velocity: b.linear,
                    mass: (b.tier != Tier::Fixed).then_some(b.mass),
                },
                RotationSpec {
                    orientation: b.orientation,
                    angular_velocity: b.angular,
                },
                b.tier == Tier::Prescribed,
                b.friction,
            )?;
            handles.insert(b.id, h);
        }
        solver.set_partition_gravity(gravity)?;
        Ok(Self {
            solver,
            bodies: bodies.to_vec(),
            handles,
        })
    }
    pub fn advance(
        &mut self,
        substeps: u8,
        loads: &[BodyWrench],
        pending: &[PendingImpulse],
    ) -> Result<(Vec<ContactImpulse>, Vec<GroundSupport>)> {
        self.advance_observing(substeps, loads, pending, |_| ())
    }
    pub fn advance_observing(
        &mut self,
        substeps: u8,
        loads: &[BodyWrench],
        pending: &[PendingImpulse],
        mut observe: impl FnMut(&MultiFidelityBackend),
    ) -> Result<(Vec<ContactImpulse>, Vec<GroundSupport>)> {
        for p in pending {
            self.solver
                .apply_impulse(self.handles[&p.id], p.impulse, ZERO)?;
        }
        let dt = Duration::from_millis(10 / u64::from(substeps));
        let mut sums = BTreeMap::<(u64, u64), f64>::new();
        let mut supports = BTreeMap::<u64, f64>::new();
        for _ in 0..substeps {
            for w in loads {
                self.solver
                    .add_world_wrench(self.handles[&w.id], w.force, w.torque)?;
            }
            self.solver.step(dt)?;
            observe(&self.solver);
            for p in self
                .solver
                .rigid_contact_observations()
                .into_iter()
                .filter(|p| p.active)
            {
                // Handles are allocated sequentially, independently of public IDs.
                let id = |h| self.bodies[h as usize].id;
                *sums.entry((id(p.a.raw()), id(p.b.raw()))).or_default() += p.normal_impulse;
            }
            for p in self
                .solver
                .partition_ground_contacts()
                .into_iter()
                .filter(|p| p.active)
            {
                *supports
                    .entry(self.bodies[p.body.raw() as usize].id)
                    .or_default() += p.normal_impulse;
            }
        }
        for b in &mut self.bodies {
            let s = self
                .solver
                .state(self.handles[&b.id])
                .ok_or("missing native body")?;
            (b.position, b.orientation, b.linear, b.angular) =
                (s.position, s.orientation, s.velocity, s.angular_velocity);
        }
        Ok((
            sums.into_iter()
                .map(|((a, b), normal_impulse)| ContactImpulse {
                    a,
                    b,
                    normal_impulse,
                })
                .collect(),
            supports
                .into_iter()
                .map(|(body, normal_impulse)| GroundSupport {
                    body,
                    normal_impulse,
                })
                .collect(),
        ))
    }
}
