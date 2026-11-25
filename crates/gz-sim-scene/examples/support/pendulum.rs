// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Public-entity observations for the two independently supported stock assemblies.
use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec3};
use gz_sim_core::Simulation;
use gz_sim_scene::{
    Name, PhysicsEngine, Velocity, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    load_world_with_runtime,
};
use serde::Serialize;
use std::error::Error;

pub const STOCK: &str = include_str!("../../../../examples/worlds/upstream_double_pendulum.sdf");
pub type Result<T> = std::result::Result<T, Box<dyn Error>>;

pub fn load(xml: &str, stiff_static_contacts: bool) -> Result<Simulation> {
    Ok(load_world_with_runtime(
        xml,
        "upstream_double_pendulum.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            stiff_static_contacts,
            ..Default::default()
        },
    )?)
}

/// Edit only the two lower links in a copy. The other links keep their implicit inertia.
pub fn with_lower_inertial(children: &str) -> Result<String> {
    let doc = roxmltree::Document::parse(STOCK)?;
    let mut offsets = Vec::new();
    for link in doc
        .descendants()
        .filter(|n| n.has_tag_name("link") && n.attribute("name") == Some("lower_link"))
    {
        let node = link
            .children()
            .find(|n| n.has_tag_name("inertial"))
            .ok_or("missing lower inertia")?;
        offsets.push(node.range().end - "</inertial>".len());
    }
    if offsets.len() != 2 {
        return Err("expected two lower links".into());
    }
    let mut xml = STOCK.to_owned();
    for i in offsets.into_iter().rev() {
        xml.insert_str(i, children);
    }
    Ok(xml)
}

#[derive(PartialEq, Debug, Clone, Copy, Serialize)]
pub struct Sample {
    pub tick: u64,
    pub model: usize,
    pub upper_q: f64,
    pub lower_q: f64,
    pub upper_dq: f64,
    pub lower_dq: f64,
    pub upper_roll: f64,
    pub lower_roll: f64,
    pub upper_com: [f64; 3],
    pub lower_com: [f64; 3],
    pub potential_j: f64,
    pub kinetic_j: f64,
    pub total_j: f64,
    pub anchor_error_m: f64,
    pub base_z_m: f64,
}

#[derive(Clone, Copy)]
struct Link {
    origin: DVec3,
    rot: DQuat,
    v: DVec3,
    w: DVec3,
}

fn state(sim: &Simulation, entity: Entity) -> Result<Link> {
    let pose = sim
        .world()
        .get::<WorldPose>(entity)
        .ok_or("missing link pose")?;
    let vel = sim
        .world()
        .get::<Velocity>(entity)
        .ok_or("missing link velocity")?;
    Ok(Link {
        origin: DVec3::from_array(pose.translation),
        rot: DQuat::from_array(pose.rotation),
        v: DVec3::from_array(vel.linear),
        w: DVec3::from_array(vel.angular),
    })
}

fn x_roll(rot: DQuat) -> f64 {
    let y = rot * DVec3::Y;
    y.z.atan2(y.y)
}

pub struct Assembly {
    model: usize,
    links: [Entity; 3],
    anchor: [DVec3; 2],
    rest: [DQuat; 2],
}

impl Assembly {
    pub fn new(sim: &mut Simulation, model: usize) -> Result<Self> {
        let world = sim.world_mut();
        let mut names = world.query::<(Entity, &Name)>();
        let mut named = |name: &str| -> Result<Entity> {
            let scope = format!("default::double_pendulum_with_base{model}::{name}");
            Ok(names
                .iter(world)
                .find(|(_, n)| n.0 == scope)
                .ok_or(scope)?
                .0)
        };
        let links = [named("base")?, named("upper_link")?, named("lower_link")?];
        let [base, upper, lower] = [
            state(sim, links[0])?,
            state(sim, links[1])?,
            state(sim, links[2])?,
        ];
        let rest = [
            base.rot.inverse() * upper.rot,
            upper.rot.inverse() * lower.rot,
        ];
        let anchor = [
            base.rot.inverse() * (upper.origin - base.origin),
            upper.rot.inverse() * (lower.origin - upper.origin),
        ];
        Ok(Self {
            model,
            links,
            rest,
            anchor,
        })
    }

    pub fn sample(&self, sim: &Simulation, lower_mass: f64, lower_inertia: f64) -> Result<Sample> {
        let links = [
            state(sim, self.links[0])?,
            state(sim, self.links[1])?,
            state(sim, self.links[2])?,
        ];
        let mut angle = [0.; 2];
        let mut rate = [0.; 2];
        let mut anchor_error: f64 = 0.;
        for (i, pair) in links.windows(2).enumerate() {
            let [parent, child] = [pair[0], pair[1]];
            let rel = parent.rot.inverse() * child.rot;
            angle[i] = x_roll(self.rest[i].inverse() * rel);
            rate[i] = (child.w - parent.w).dot(child.rot * DVec3::X);
            anchor_error = anchor_error
                .max((parent.origin + parent.rot * self.anchor[i] - child.origin).length());
        }
        let mut potential = 0.;
        let mut kinetic = 0.;
        let mut coms = [[0.; 3]; 3];
        for (i, link) in links.iter().enumerate() {
            let mass = [100., 1., lower_mass][i];
            let inertia = [1., 1., lower_inertia][i];
            let arm = link.rot * DVec3::Z * [0., 0.5, 0.5][i];
            let com = link.origin + arm;
            let com_v = link.v + link.w.cross(arm);
            coms[i] = com.to_array();
            potential += mass * 9.8 * com.z;
            kinetic += 0.5 * (mass * com_v.length_squared() + inertia * link.w.length_squared());
        }
        let sample = Sample {
            model: self.model,
            tick: sim.committed_update_info()?.iteration,
            upper_q: angle[0],
            lower_q: angle[1],
            upper_dq: rate[0],
            lower_dq: rate[1],
            upper_roll: x_roll(links[1].rot),
            lower_roll: x_roll(links[2].rot),
            upper_com: coms[1],
            lower_com: coms[2],
            potential_j: potential,
            kinetic_j: kinetic,
            total_j: potential + kinetic,
            anchor_error_m: anchor_error,
            base_z_m: links[0].origin.z,
        };
        if ![
            sample.upper_q,
            sample.lower_q,
            sample.upper_dq,
            sample.lower_dq,
            sample.total_j,
            sample.anchor_error_m,
        ]
        .iter()
        .all(|v| v.is_finite())
        {
            return Err(format!("nonfinite observation at tick {}", sample.tick).into());
        }
        Ok(sample)
    }
}
