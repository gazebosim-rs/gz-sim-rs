// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! An uninterrupted ordinary Bevy world used to check local worker trajectories.
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_distributed::articulation::{
    ArticulationLinkState, ArticulationObservation, ArticulationSpec,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    queue_joint_effort,
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

pub struct AuthoredHinge(Simulation);

impl AuthoredHinge {
    pub fn load(xml: &str, source: &str) -> Result<Self> {
        Ok(Self(load_world_with_sdf_step(
            xml,
            source,
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?))
    }

    pub fn advance(&mut self) -> Result<()> {
        Ok(self.0.step_exact(1)?)
    }

    pub fn queue(&mut self, id: u64, effort: f64) -> Result<()> {
        Ok(queue_joint_effort(&mut self.0, id, effort)?)
    }

    pub fn native(&self) -> Result<&MultiFidelityBackend> {
        self.0
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or_else(|| "missing ordinary multifidelity physics".into())
    }

    pub fn body(&self, id: u64, tick: u64) -> Result<(BodyId, ArticulationLinkState)> {
        let world = self.0.world();
        let entity = entity_by_id(world, id).ok_or("missing authored body")?;
        let handle = world
            .get::<BodyHandle>(entity)
            .ok_or("missing body handle")?
            .id();
        let state = world
            .resource::<PhysicsBackend>()
            .state(handle)
            .ok_or("missing native body state")?;
        if tick > 0 {
            let pose = world.get::<WorldPose>(entity).ok_or("missing ECS pose")?;
            if (pose.translation, pose.rotation) != (state.position, state.orientation) {
                return Err("published and native body pose disagree".into());
            }
        }
        Ok((
            handle,
            ArticulationLinkState {
                id,
                position: state.position,
                orientation: state.orientation,
                velocity: state.velocity,
                angular_velocity: state.angular_velocity,
            },
        ))
    }

    pub fn hinge(
        &self,
        spec: &ArticulationSpec,
        tick: u64,
    ) -> Result<([BodyId; 2], ArticulationObservation)> {
        if self.0.elapsed().as_millis() != u128::from(tick) {
            return Err("independent clock mismatch".into());
        }
        let world = self.0.world();
        let entity = entity_by_id(world, spec.joint_id).ok_or("missing SDF hinge")?;
        let handle = world
            .get::<JointHandle>(entity)
            .ok_or("missing joint handle")?
            .id();
        let joint = world
            .resource::<PhysicsBackend>()
            .joint_state(handle)
            .ok_or("missing native joint sample")?;
        if tick > 0 {
            let sample = world
                .get::<JointSample>(entity)
                .ok_or("missing ECS joint")?;
            if sample.state != joint || sample.time != self.0.elapsed() {
                return Err("published joint disagrees with native hinge".into());
            }
        }
        let (parent, parent_state) = self.body(spec.links[0].id, tick)?;
        let (child, child_state) = self.body(spec.links[1].id, tick)?;
        Ok((
            [parent, child],
            ArticulationObservation {
                tick,
                joint_id: spec.joint_id,
                links: [parent_state, child_state],
                q: joint.position,
                qdot: joint.velocity,
                ground_reactions: None,
            },
        ))
    }
}
