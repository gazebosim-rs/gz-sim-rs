// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::Result;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{BodyHandle, Name, set_joint_force};

pub const BASE: &str = include_str!("../../../../examples/worlds/local_offset_gripper_carry.sdf");

#[path = "interval.rs"]
mod interval;
pub(crate) use interval::{DT, Grid, configure};

pub struct Rig {
    slides: [u64; 2],
    lift: u64,
    pads: [BodyId; 2],
    parcel: BodyId,
    center: DVec3,
}

pub struct Reading {
    pub parcel_z: f64,
    pub parcel_vz: f64,
    pub vertical_total_n: [f64; 2],
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
        self.drive_jaws(sim, [force; 2])
    }

    pub fn drive_jaws(&self, sim: &mut Simulation, forces: [f64; 2]) -> Result<()> {
        for (joint, force) in self.slides.into_iter().zip(forces) {
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
        let tick = interval::last(sim);
        let mut normal = [0.; 2];
        let mut friction = [0.; 2];
        let mut total = [0.; 2];
        let mut add = |a, b, impulse, normal_z: f64, tangent: f64| {
            if let Some(i) = self
                .pads
                .iter()
                .position(|&id| (a, b) == (id, self.parcel) || (a, b) == (self.parcel, id))
            {
                normal[i] += impulse / DT.as_secs_f64();
                let sign = if a == self.parcel { 1. } else { -1. };
                friction[i] += sign * tangent / DT.as_secs_f64();
                total[i] += sign * (normal_z + tangent) / DT.as_secs_f64();
            }
        };
        if let Some(tick) = tick {
            for p in &tick.pairs {
                add(
                    p.a,
                    p.b,
                    p.normal_impulse,
                    p.normal_impulse_on_a[2],
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
                        patch.normal_on_a[2] * point.normal_impulse,
                        point.tangent_impulse_on_a.unwrap()[2],
                    );
                }
            }
        }
        Reading {
            parcel_z,
            parcel_vz: body.velocity[2],
            vertical_total_n: total,
            pad_z,
            normal,
            friction,
            native_steps: tick.map_or(1, |r| r.native_steps),
        }
    }
}
