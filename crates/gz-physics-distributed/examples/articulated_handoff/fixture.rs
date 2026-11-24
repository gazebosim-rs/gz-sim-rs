// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, JointId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, Name, PhysicsEngine, load_world_with_sdf_step, queue_joint_effort,
    queue_link_world_wrench,
};

pub(super) const CUT: u16 = 120;
pub(super) const SPAN: u16 = 256;
pub(super) const PENDING: f64 = -0.8125;
const SDF: &str = include_str!("../../../../examples/multifidelity_articulation.sdf");

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub(super) struct Link {
    pub native_body: u64,
    pub public_link: u64,
    pub public_visual: u64,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub velocity: [f64; 3],
    pub angular_velocity: [f64; 3],
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub(super) struct Observation {
    pub links: [Link; 2],
    pub public_joint: u64,
    pub native_joint: u64,
    pub q: f64,
    pub qdot: f64,
}
pub(super) struct Tree {
    sim: Simulation,
    joint: JointId,
    public_joint: u64,
    links: [(BodyId, u64, u64); 2],
}
impl Tree {
    pub fn fresh() -> Result<Self> {
        let sim = load_world_with_sdf_step(
            SDF,
            "articulation.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let find = |name: &str| {
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0.ends_with(name)))
                .ok_or("missing SDF entity")
        };
        let joint = find("robot::hinge")?;
        let public_joint = joint.get::<EntityId>().ok_or("missing joint ID")?.get();
        let joint = joint.get::<JointHandle>().ok_or("missing joint")?.id();
        let ids = |link: &str, visual: &str| -> Result<_> {
            let e = find(link)?;
            Ok((
                e.get::<BodyHandle>().ok_or("missing body")?.id(),
                e.get::<EntityId>().ok_or("missing link ID")?.get(),
                find(visual)?
                    .get::<EntityId>()
                    .ok_or("missing visual ID")?
                    .get(),
            ))
        };
        let links = [
            ids("robot::base", "::base_visual")?,
            ids("robot::tool", "::tool_visual")?,
        ];
        if links.map(|l| l.0.raw()) != [0, 1]
            || joint.raw() != 0
            || links.map(|l| (l.1, l.2)) != [(3, 5), (6, 8)]
            || public_joint != 9
        {
            return Err("unexpected authored articulation bindings".into());
        }
        Ok(Self {
            sim,
            joint,
            public_joint,
            links,
        })
    }
    pub fn warm() -> Result<Self> {
        let mut w = Self::fresh()?;
        for (&(_, link, _), mass) in w.links.iter().zip([2., 0.8]) {
            queue_link_world_wrench(&mut w.sim, link, [250. * mass, 0., 0.], [0.; 3])?;
        }
        for i in 0..CUT {
            w.step((i < 43).then_some(4.))?;
        }
        w.hold_effort(PENDING)?;
        Ok(w)
    }
    pub fn restore(bytes: &[u8]) -> Result<Self> {
        let mut w = Self::fresh()?;
        w.install(bytes)?;
        Ok(w)
    }
    pub fn install(&mut self, bytes: &[u8]) -> Result<()> {
        self.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<MultiFidelityBackend>()
            .ok_or("wrong backend")?
            .restore_floating_revolute(bytes)?;
        Ok(())
    }
    pub fn image(&self) -> Result<Vec<u8>> {
        Ok(self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("wrong backend")?
            .checkpoint_floating_revolute()?)
    }
    pub fn time_ns(&self) -> u64 {
        self.sim.elapsed().as_nanos() as u64
    }
    pub fn observe(&self) -> Observation {
        let b = self.sim.world().resource::<PhysicsBackend>();
        let links = self.links.map(|(id, public_link, public_visual)| {
            let s = b.state(id).expect("authored body");
            Link {
                native_body: id.raw(),
                public_link,
                public_visual,
                position: s.position,
                orientation: s.orientation,
                velocity: s.velocity,
                angular_velocity: s.angular_velocity,
            }
        });
        let s = b.joint_state(self.joint).expect("authored joint");
        Observation {
            links,
            public_joint: self.public_joint,
            native_joint: self.joint.raw(),
            q: s.position,
            qdot: s.velocity,
        }
    }
    #[allow(dead_code)]
    pub fn check_publication(&self, sample: &Observation) -> Result<()> {
        use gz_sim_scene::{JointSample, Velocity, WorldPose};
        let world = self.sim.world();
        let joint = entity_by_id(world, self.public_joint)
            .and_then(|e| world.get::<JointSample>(e))
            .ok_or("missing public joint sample")?;
        if joint.state.position.to_bits() != sample.q.to_bits()
            || joint.state.velocity.to_bits() != sample.qdot.to_bits()
            || joint.time != self.sim.elapsed()
            || joint.epoch != self.sim.epoch()
        {
            return Err("public Bevy joint differs from its native state or clock".into());
        }
        for l in &sample.links {
            let e = entity_by_id(world, l.public_link).ok_or("missing public link")?;
            let pose = world.get::<WorldPose>(e).ok_or("missing link pose")?;
            let velocity = world.get::<Velocity>(e).ok_or("missing link twist")?;
            if pose.translation.map(f64::to_bits) != l.position.map(f64::to_bits)
                || pose.rotation.map(f64::to_bits) != l.orientation.map(f64::to_bits)
                || velocity.angular.map(f64::to_bits) != l.angular_velocity.map(f64::to_bits)
            {
                return Err("public Bevy link differs from its native state".into());
            }
        }
        Ok(())
    }

    pub fn hold_effort(&mut self, value: f64) -> Result<()> {
        self.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_joint_effort(self.joint, value)?;
        Ok(())
    }
    pub fn step(&mut self, effort: Option<f64>) -> Result<()> {
        if let Some(t) = effort {
            queue_joint_effort(&mut self.sim, self.public_joint, t)?;
        }
        self.sim.step_exact(1)?;
        Ok(())
    }
}
