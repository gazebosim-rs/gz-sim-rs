// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, JointId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    queue_joint_effort,
};

pub(super) struct Reference {
    sim: Simulation,
    links: [BodyId; 2],
    floor: BodyId,
    joint: JointId,
}

impl Reference {
    pub(super) fn load(spec: &ArticulationSpec) -> Result<Self> {
        let sim = load_world_with_sdf_step(
            SDF,
            SOURCE,
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let world = sim.world();
        let body = |public| {
            entity_by_id(world, public)
                .and_then(|e| world.get::<BodyHandle>(e))
                .map(BodyHandle::id)
                .ok_or("missing reference body")
        };
        let links = [body(spec.links[0].id)?, body(spec.links[1].id)?];
        let floor = body(spec.ground.ok_or("missing ground")?.link_id)?;
        let joint = entity_by_id(world, spec.joint_id)
            .and_then(|e| world.get::<JointHandle>(e))
            .ok_or("missing reference joint")?
            .id();
        Ok(Self {
            sim,
            links,
            floor,
            joint,
        })
    }

    pub(super) fn queue(&mut self, id: u64, torque: f64) -> Result<()> {
        Ok(queue_joint_effort(&mut self.sim, id, torque)?)
    }

    pub(super) fn step(&mut self) -> Result<()> {
        Ok(self.sim.step_exact(1)?)
    }

    fn solver(&self) -> &MultiFidelityBackend {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref()
            .expect("standalone reference")
    }

    pub(super) fn image(&self) -> Result<Vec<u8>> {
        Ok(self.solver().checkpoint_grounded_revolute()?)
    }

    pub(super) fn observe(
        &self,
        spec: &ArticulationSpec,
        tick: u64,
    ) -> Result<ArticulationObservation> {
        let world = self.sim.world();
        let backend = world.resource::<PhysicsBackend>();
        let joint = backend
            .joint_state(self.joint)
            .ok_or("missing reference joint state")?;
        let links = std::array::from_fn(|i| {
            let s = backend.state(self.links[i]).expect("reference body");
            ArticulationLinkState {
                id: spec.links[i].id,
                position: s.position,
                orientation: s.orientation,
                velocity: s.velocity,
                angular_velocity: s.angular_velocity,
            }
        });
        if self.sim.elapsed().as_millis() != u128::from(tick) {
            return Err("reference clock differs".into());
        }
        if tick > 0 {
            let sample = entity_by_id(world, spec.joint_id)
                .and_then(|e| world.get::<JointSample>(e))
                .ok_or("missing ECS joint")?;
            if sample.time != self.sim.elapsed() || sample.state != joint {
                return Err("public joint differs from native measurement".into());
            }
            for link in &links {
                let pose = entity_by_id(world, link.id)
                    .and_then(|e| world.get::<WorldPose>(e))
                    .ok_or("missing public pose")?;
                if pose.translation.map(f64::to_bits) != link.position.map(f64::to_bits)
                    || pose.rotation.map(f64::to_bits) != link.orientation.map(f64::to_bits)
                {
                    return Err("public link differs from native measurement".into());
                }
            }
        }
        let native = self.solver();
        let ground_reactions = Some([
            native.ground_reaction(self.links[0], self.floor)?,
            native.ground_reaction(self.links[1], self.floor)?,
        ]);
        Ok(ArticulationObservation {
            tick,
            joint_id: spec.joint_id,
            links,
            q: joint.position,
            qdot: joint.velocity,
            ground_reactions,
        })
    }

    /// Replay the prior drive on a fresh standalone world, then leave the next drive unqueued.
    pub(super) fn without_next_effort(
        spec: &ArticulationSpec,
        history: &[Vec<f64>],
    ) -> Result<Self> {
        let mut native = Self::load(spec)?;
        for efforts in history {
            for &effort in efforts {
                native.queue(spec.joint_id, effort)?;
            }
            native.step()?;
        }
        Ok(native)
    }
}
