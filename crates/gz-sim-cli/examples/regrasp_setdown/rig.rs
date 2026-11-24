// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Case, Result};
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{BodyHandle, Name, grasp_guard, set_joint_force};
use std::collections::BTreeMap;

const BASE: &str = include_str!("../../../../examples/worlds/local_offset_gripper_carry.sdf");

pub fn world(case: Case) -> Result<String> {
    let xml = super::payload::world(BASE, case.mass() / 0.20, case == Case::Mirror, None)?;
    // A narrow stand reaches between the long fingers. A misplaced receiver
    // still exists in the missing-receiver test but sits outside the workspace.
    let top = case.deck_top();
    let y = case.deck_y();
    let receiver = deck("receiver", case.deck_x(), y, top);
    let extra = if case == Case::Wrong {
        deck("obstacle", case.deck_x(), 0., top)
    } else {
        String::new()
    };
    Ok(xml.replace("  </world>", &format!("{receiver}{extra}  </world>")))
}

fn deck(name: &str, x: f64, y: f64, top: f64) -> String {
    format!(
        r#"<model name="{name}"><static>true</static><link name="deck"><pose>{x} {y} {} 0 0 0</pose>
    <collision name="deck"><geometry><box><size>0.020 0.040 0.010</size></box></geometry></collision>
    <visual name="deck"><geometry><box><size>0.020 0.040 0.010</size></box></geometry><material><diffuse>0.22 0.58 0.73 1</diffuse></material></visual>
    </link></model>"#,
        top - 0.005
    )
}

#[path = "../closed_loop_carry/interval.rs"]
mod interval;
pub(crate) use interval::{DT, Grid, configure};

pub struct Rig {
    slides: [u64; 2],
    lift: u64,
    pads: [BodyId; 2],
    parcel: BodyId,
    receiver: BodyId,
    obstacle: Option<BodyId>,
    offset: f64,
}
pub struct Reading {
    pub pad_z: f64,
    pub pad_v: f64,
    pub parcel_com: [f64; 3],
    pub parcel_velocity: [f64; 3],
    pub pad_gap: f64,
    pub clearance: f64,
    pub jaw: [f64; 2],
    pub pad_vertical: [f64; 2],
    pub receiver_vertical: f64,
    pub other_vertical: f64,
    pub receiver: f64,
    pub obstacle: f64,
    pub native: u32,
}
impl Rig {
    pub fn new(sim: &mut Simulation, case: Case) -> Self {
        let names: BTreeMap<_, _> = sim
            .world_mut()
            .query::<(&Name, &EntityId, Option<&BodyHandle>)>()
            .iter(sim.world())
            .map(|(n, id, b)| (n.0.clone(), (id.get(), b.map(BodyHandle::id))))
            .collect();
        Self {
            slides: ["left_slide", "right_slide"]
                .map(|n| names[&format!("default::gripper::{n}")].0),
            lift: names["default::gripper::lift"].0,
            pads: ["left", "right"].map(|n| names[&format!("default::gripper::{n}")].1.unwrap()),
            parcel: names["default::parcel::body"].1.unwrap(),
            receiver: names["default::receiver::deck"].1.unwrap(),
            obstacle: names.get("default::obstacle::deck").and_then(|v| v.1),
            offset: if case == Case::Mirror {
                -0.0045
            } else {
                0.0045
            },
        }
    }
    pub fn drive(&self, sim: &mut Simulation, jaw: [f64; 2], lift: f64) -> Result<()> {
        for (id, f) in self.slides.iter().zip(jaw) {
            set_joint_force(sim, *id, f)?;
        }
        Ok(set_joint_force(sim, self.lift, lift)?)
    }
    pub fn tactile(&self, sim: &Simulation) -> Option<f64> {
        grasp_guard::bilateral_slip_velocity(sim, self.pads, self.parcel, [0., 0., 1.])
    }
    pub fn read(&self, sim: &Simulation) -> Reading {
        let engine = sim.world().resource::<PhysicsBackend>();
        let [l, r] = self.pads.map(|id| engine.state(id).unwrap());
        let body = engine.state(self.parcel).unwrap();
        let com = DVec3::from(body.position)
            + DQuat::from_array(body.orientation) * DVec3::X * self.offset;
        let rotation = DQuat::from_array(body.orientation);
        let half_width = (rotation * DVec3::X).x.abs() * 0.015
            + (rotation * DVec3::Y).x.abs() * 0.010
            + (rotation * DVec3::Z).x.abs() * 0.012;
        let clearance = (body.position[0] - half_width - l.position[0] - 0.010)
            .min(r.position[0] - 0.010 - body.position[0] - half_width);
        let tick = interval::last(sim);
        let mut obs = Reading {
            clearance,
            pad_z: 0.5 * (l.position[2] + r.position[2]),
            pad_v: 0.5 * (l.velocity[2] + r.velocity[2]),
            parcel_com: com.into(),
            parcel_velocity: body.velocity,
            pad_gap: r.position[0] - l.position[0] - 0.02,
            jaw: [0.; 2],
            pad_vertical: [0.; 2],
            receiver_vertical: 0.,
            other_vertical: 0.,
            receiver: 0.,
            obstacle: 0.,
            native: tick.map_or(1, |r| r.native_steps),
        };
        let mut add = |a, b, mag, z: f64, tangent: f64| {
            let (other, sign) = if a == self.parcel {
                (b, 1.)
            } else if b == self.parcel {
                (a, -1.)
            } else {
                return;
            };
            let force = sign * z / DT.as_secs_f64();
            if let Some(i) = self.pads.iter().position(|id| *id == other) {
                obs.jaw[i] += mag / DT.as_secs_f64();
                obs.pad_vertical[i] += sign * (z + tangent) / DT.as_secs_f64();
            } else if other == self.receiver {
                obs.receiver_vertical += sign * (z + tangent) / DT.as_secs_f64();
            } else {
                obs.other_vertical += sign * (z + tangent) / DT.as_secs_f64();
            }
            if other == self.receiver {
                obs.receiver += force;
            }
            if Some(other) == self.obstacle {
                obs.obstacle += force;
            }
        };
        if let Some(r) = tick {
            for p in &r.pairs {
                add(
                    p.a,
                    p.b,
                    p.normal_impulse,
                    p.normal_impulse_on_a[2],
                    p.tangent_impulse_on_a.map_or(0., |v| v[2]),
                );
            }
        } else {
            for p in engine
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap()
                .rigid_contact_patches()
            {
                for j in &p.points {
                    add(
                        p.a,
                        p.b,
                        j.normal_impulse,
                        p.normal_on_a[2] * j.normal_impulse,
                        j.tangent_impulse_on_a.map_or(0., |v| v[2]),
                    );
                }
            }
        }
        obs
    }
}
