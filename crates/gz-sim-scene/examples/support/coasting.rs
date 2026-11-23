// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Instrument the SDF inertias, all public body/joint IDs and native states.
use bevy_ecs::entity::Entity;
use glam::{DMat3, DQuat, DVec3};
use gz_physics::{BodyId, BodyState, JointId, JointState, JointVelocityMotor, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, Name, Velocity, WorldPose, queue_link_world_wrench,
};
use std::error::Error;

pub const ARRAY: &str = include_str!("../../../../examples/multifidelity_coasting_satellite.sdf");
pub fn stable_id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
#[derive(Debug, PartialEq)]
pub struct Mechanism {
    pub model: u64,
    pub links: Vec<(u64, BodyId)>,
    pub joints: Vec<(u64, JointId)>,
    inertias: Vec<(f64, DVec3)>,
}
#[derive(Debug, PartialEq)]
pub struct Sample {
    pub links: Vec<BodyState>,
    pub joints: Vec<JointState>,
    pub center: DVec3,
    pub momentum: DVec3,
    pub angular: DVec3,
    pub energy: f64,
}
impl Mechanism {
    pub fn bind(sim: &Simulation, xml: &str, name: &str) -> Self {
        let doc = roxmltree::Document::parse(xml).unwrap();
        let model = doc
            .descendants()
            .find(|e| e.has_tag_name("model") && e.attribute("name") == Some(name))
            .unwrap();
        let prefix = format!("default::{name}");
        let mut links = Vec::new();
        let mut joints = Vec::new();
        let mut inertias = Vec::new();
        for element in model.children().filter(roxmltree::Node::is_element) {
            let full = || format!("{prefix}::{}", element.attribute("name").unwrap());
            let entity = |id| entity_by_id(sim.world(), id).unwrap();
            if element.has_tag_name("link") {
                let id = stable_id(sim, &full());
                links.push((id, sim.world().get::<BodyHandle>(entity(id)).unwrap().id()));
                let value = |tag| {
                    element
                        .descendants()
                        .find(|n| n.has_tag_name(tag))
                        .unwrap()
                        .text()
                        .unwrap()
                        .parse::<f64>()
                        .unwrap()
                };
                // Both fixtures have a diagonal inertia at the unrotated link origin.
                inertias.push((
                    value("mass"),
                    DVec3::new(value("ixx"), value("iyy"), value("izz")),
                ));
            }
            if element.has_tag_name("joint") {
                let id = stable_id(sim, &full());
                joints.push((id, sim.world().get::<JointHandle>(entity(id)).unwrap().id()));
            }
        }
        Self {
            model: stable_id(sim, &prefix),
            links,
            joints,
            inertias,
        }
    }
    pub fn sample(&self, sim: &Simulation) -> Sample {
        let b = sim.world().resource::<PhysicsBackend>();
        let entity = |id| entity_by_id(sim.world(), id).unwrap();
        let mut sample = Sample {
            links: Vec::new(),
            joints: Vec::new(),
            center: DVec3::ZERO,
            momentum: DVec3::ZERO,
            angular: DVec3::ZERO,
            energy: 0.,
        };
        let mut mass = 0.;
        for (&(id, body), &(m, inertia)) in self.links.iter().zip(&self.inertias) {
            let state = b.state(body).unwrap();
            let e = entity(id);
            let pose = sim.world().get::<WorldPose>(e).unwrap();
            let twist = sim.world().get::<Velocity>(e).unwrap();
            assert_eq!(sim.world().get::<BodyHandle>(e).unwrap().id(), body);
            assert!(
                DVec3::from_array(pose.translation).distance(DVec3::from_array(state.position))
                    < 1e-12
            );
            assert!(
                DVec3::from_array(twist.linear).distance(DVec3::from_array(state.velocity)) < 1e-10
            );
            let p = DVec3::from_array(state.position);
            let v = DVec3::from_array(state.velocity);
            let q = DQuat::from_array(state.orientation);
            let w = DVec3::from_array(state.angular_velocity);
            let spin = q * (inertia * (q.inverse() * w));
            sample.center += m * p;
            sample.momentum += m * v;
            sample.angular += p.cross(m * v) + spin;
            sample.energy += 0.5 * (m * v.length_squared() + w.dot(spin));
            sample.links.push(state);
            mass += m;
        }
        sample.center /= mass;
        for &(id, joint) in &self.joints {
            let e = entity(id);
            assert_eq!(sim.world().get::<JointHandle>(e).unwrap().id(), joint);
            let state = b.joint_state(joint).unwrap();
            let observation = sim.world().get::<JointSample>(e).unwrap();
            assert_eq!(observation.time, sim.elapsed());
            assert_eq!(observation.state, state);
            sample.joints.push(state);
        }
        sample
    }
    pub fn mass(&self) -> f64 {
        self.inertias.iter().map(|p| p.0).sum()
    }
    pub fn impart_twist(&self, sim: &mut Simulation, omega: f64) -> Result<(), Box<dyn Error>> {
        let initial = self.sample(sim);
        let dt = sim.step_duration().as_secs_f64();
        let w = DVec3::Z * omega;
        for ((&(id, _), s), &(m, inertia)) in
            self.links.iter().zip(&initial.links).zip(&self.inertias)
        {
            let arm = DVec3::from_array(s.position) - initial.center;
            let dv = DVec3::new(0.18, 0.07, 0.) + w.cross(arm);
            let orient = DMat3::from_quat(DQuat::from_array(s.orientation));
            let torque = orient * (inertia * (orient.transpose() * w)) / dt;
            queue_link_world_wrench(sim, id, (m * dv / dt).to_array(), torque.to_array())?;
        }
        Ok(())
    }
    pub fn prepare(
        &self,
        sim: &mut Simulation,
        omega: f64,
        brake_ticks: u64,
        mut observe: impl FnMut(&Simulation, u64),
    ) -> Result<(), Box<dyn Error>> {
        self.impart_twist(sim, omega)?;
        for tick in 1..=brake_ticks {
            let mut b = sim.world_mut().resource_mut::<PhysicsBackend>();
            for &(_, j) in &self.joints {
                let q = b.joint_state(j).unwrap().position;
                b.set_joint_velocity_servo(
                    j,
                    Some(JointVelocityMotor {
                        target_velocity: -20. * q,
                        max_force: 500.,
                    }),
                )?;
            }
            sim.step_exact(1)?;
            observe(sim, tick);
        }
        if brake_ticks == 0 {
            sim.step_exact(1)?;
            observe(sim, 1);
        } else {
            let mut b = sim.world_mut().resource_mut::<PhysicsBackend>();
            for &(_, j) in &self.joints {
                b.set_joint_velocity_servo(j, None)?;
            }
        }
        Ok(())
    }
}
pub fn root_entity(sim: &Simulation, mechanism: &Mechanism) -> Entity {
    entity_by_id(sim.world(), mechanism.links[0].0).unwrap()
}
