// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::{Case, Grid, Result};
use bevy_ecs::prelude::{Res, ResMut, Resource};
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::{ContactIntervalReceipt, MultiFidelityBackend};
use gz_sim_core::{EntityId, Phase, Simulation, UpdateInfo};
use gz_sim_scene::{BodyHandle, Name, primary_contact, set_joint_force};
use std::time::Duration;

const BASE: &str = include_str!("../../../../examples/worlds/local_offset_gripper_carry.sdf");
pub const DT: Duration = Duration::from_micros(200);

/// Vary physical parameters of the same authored robot. For a heavier rigid
/// parcel at the same dimensions, scale its entire inertia tensor with mass.
pub fn world(case: Case) -> Result<String> {
    let doc = roxmltree::Document::parse(BASE)?;
    let parcel = doc
        .descendants()
        .find(|n| n.has_tag_name("model") && n.attribute("name") == Some("parcel"))
        .unwrap();
    let inertial = parcel
        .descendants()
        .find(|n| n.has_tag_name("inertial"))
        .unwrap();
    let mut edits = Vec::new();
    for node in doc.descendants().filter(|n| n.is_text()) {
        let parent = node.parent().unwrap();
        let tag = parent.tag_name().name();
        let value = match case {
            Case::Slick if matches!(tag, "mu" | "mu2") => Some("0.4".to_string()),
            Case::Mirror if tag == "pose" && parent.parent() == Some(inertial) => {
                Some("-0.0045 0 0 0 0 0".to_string())
            }
            Case::Heavy
                if parent.ancestors().any(|n| n == inertial)
                    && matches!(tag, "mass" | "ixx" | "iyy" | "izz" | "ixy" | "iyz" | "ixz") =>
            {
                Some((node.text().unwrap().parse::<f64>()? * (0.26 / 0.20)).to_string())
            }
            _ => None,
        };
        if let Some(value) = value {
            edits.push((node.range(), value));
        }
    }
    let mut xml = BASE.to_owned();
    for (range, value) in edits.into_iter().rev() {
        xml.replace_range(range, &value);
    }
    Ok(xml)
}

#[derive(Resource, Default)]
struct FineTick(ContactIntervalReceipt);

pub fn configure(sim: &mut Simulation, grid: Grid) -> Result<()> {
    if grid == Grid::Fine {
        assert_eq!(
            sim.remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)?,
            1
        );
        sim.world_mut().init_resource::<FineTick>();
        sim.add_systems(
            Phase::PhysicsStep,
            |t: Res<UpdateInfo>, mut p: ResMut<PhysicsBackend>, mut receipt: ResMut<FineTick>| {
                receipt.0 = p
                    .downcast_mut::<MultiFidelityBackend>()
                    .unwrap()
                    .step_contact_interval(t.dt, 200, 4)
                    .unwrap();
            },
        );
    }
    Ok(())
}

pub struct Rig {
    slides: [u64; 2],
    lift: u64,
    pads: [BodyId; 2],
    parcel: BodyId,
    center: DVec3,
}

pub struct Reading {
    pub parcel_z: f64,
    pub pad_z: f64,
    pub normal: [f64; 2],
    pub friction: [f64; 2],
    pub native_steps: u32,
}

impl Rig {
    pub fn new(sim: &mut Simulation, center: [f64; 3]) -> Self {
        let names = sim
            .world_mut()
            .query::<(&Name, &EntityId, Option<&BodyHandle>)>()
            .iter(sim.world())
            .map(|(n, id, body)| (n.0.clone(), (id.get(), body.map(BodyHandle::id))))
            .collect::<std::collections::BTreeMap<_, _>>();
        Self {
            slides: [
                "default::gripper::left_slide",
                "default::gripper::right_slide",
            ]
            .map(|n| names[n].0),
            lift: names["default::gripper::lift"].0,
            pads: ["default::gripper::left", "default::gripper::right"]
                .map(|n| names[n].1.unwrap()),
            parcel: names["default::parcel::body"].1.unwrap(),
            center: center.into(),
        }
    }

    pub fn tactile_speed(&self, sim: &Simulation) -> Option<f64> {
        gz_sim_scene::grasp_guard::bilateral_slip_velocity(
            sim,
            self.pads,
            self.parcel,
            [0., 0., 1.],
        )
    }

    pub fn grip(&self, sim: &mut Simulation, force: f64) -> Result<()> {
        for joint in self.slides {
            set_joint_force(sim, joint, force)?;
        }
        Ok(())
    }

    pub fn lift(&self, sim: &mut Simulation, force: f64) -> Result<()> {
        Ok(set_joint_force(sim, self.lift, force)?)
    }

    pub fn read(&self, sim: &Simulation) -> Reading {
        let engine = sim.world().resource::<PhysicsBackend>();
        let body = engine.state(self.parcel).unwrap();
        let parcel_z = body.position[2] + (DQuat::from_array(body.orientation) * self.center).z;
        let pad_z = self
            .pads
            .iter()
            .map(|&id| engine.state(id).unwrap().position[2])
            .sum::<f64>()
            / 2.;
        let tick = sim
            .world()
            .get_resource::<FineTick>()
            .map(|f| &f.0)
            .or_else(|| {
                primary_contact::last_refinement(sim)
                    .filter(|r| r.sim_time == sim.elapsed())
                    .map(|r| &r.interval)
            });
        let mut normal = [0.; 2];
        let mut friction = [0.; 2];
        let mut add = |a, b, impulse, tangent: f64| {
            if let Some(i) = self
                .pads
                .iter()
                .position(|&id| (a, b) == (id, self.parcel) || (a, b) == (self.parcel, id))
            {
                normal[i] += impulse / DT.as_secs_f64();
                friction[i] += if a == self.parcel { tangent } else { -tangent } / DT.as_secs_f64();
            }
        };
        if let Some(tick) = tick {
            for p in &tick.pairs {
                add(
                    p.a,
                    p.b,
                    p.normal_impulse,
                    p.tangent_impulse_on_a.unwrap()[2],
                );
            }
        } else {
            let native = engine.downcast_ref::<MultiFidelityBackend>().unwrap();
            for patch in native.rigid_contact_patches() {
                for point in patch.points {
                    add(
                        patch.a,
                        patch.b,
                        point.normal_impulse,
                        point.tangent_impulse_on_a.unwrap()[2],
                    );
                }
            }
        }
        Reading {
            parcel_z,
            pad_z,
            normal,
            friction,
            native_steps: tick.map_or(1, |r| r.native_steps),
        }
    }
}
