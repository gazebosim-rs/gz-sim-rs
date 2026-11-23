// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine,
    deformable::{DeformableSample, bodies},
    load_world_with_sdf_step, queue_link_world_wrench,
};
use std::error::Error;
pub const WORLD: &str = include_str!("../../../../examples/capsule_cloth_grasp.sdf");
pub const JAW_MASS: f64 = 0.010;
pub const CLOTH_MASS: f64 = 0.002;
pub const GRAVITY: f64 = 9.81;
pub const RADIUS: f64 = 0.008;
pub const HALF_LENGTH: f64 = 0.012;
pub const SKIN: f64 = 0.0025;
pub const SQUEEZE: f64 = 0.055;
pub const END: f64 = 1.05;
pub const HOLD: (f64, f64) = (0.57, 0.69);
pub const RELEASE: f64 = 0.73;
pub const LIFT: f64 = 0.080;
pub const CARRY: f64 = 0.055;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mode {
    Closed,
    Open,
    NoFriction,
    Particles,
}
pub struct Grasp {
    pub sim: Simulation,
    links: [(u64, BodyId); 2],
    mode: Mode,
    pub force: f64,
}
pub fn path(t: f64, start: f64, duration: f64, length: f64) -> (f64, f64) {
    let s = ((t - start) / duration).clamp(0., 1.);
    (
        length * s * s * s * (10. + s * (-15. + 6. * s)),
        length * 30. * s * s * (1. - s).powi(2) / duration,
    )
}
impl Grasp {
    pub fn new(mode: Mode, mut xml: String, force: f64) -> Result<Self, Box<dyn Error>> {
        xml = match mode {
            Mode::Open | Mode::Closed => xml,
            Mode::NoFriction => xml.replace("friction=\"0.65\"", "friction=\"0\""),
            Mode::Particles => xml.replace(
                "capsule_contact=\"material\"",
                "capsule_contact=\"particles\"",
            ),
        };
        let sim = load_world_with_sdf_step(
            &xml,
            "capsule_cloth_grasp.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let links = ["left", "right"].map(|n| {
            let name = format!("default::{n}::jaw");
            let e = sim
                .world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap();
            (
                e.get::<EntityId>().unwrap().get(),
                e.get::<BodyHandle>().unwrap().id(),
            )
        });
        Ok(Self {
            sim,
            links,
            mode,
            force,
        })
    }
    pub fn jaws(&self) -> [BodyState; 2] {
        self.links.map(|(_, id)| {
            self.sim
                .world()
                .resource::<PhysicsBackend>()
                .state(id)
                .unwrap()
        })
    }
    pub fn cloth(&self) -> &DeformableSample {
        bodies(&self.sim).next().unwrap().1
    }
    pub fn center(&self) -> DVec3 {
        let body = self.cloth();
        body.particles()
            .iter()
            .map(|p| DVec3::from(p.position) / p.inverse_mass)
            .sum::<DVec3>()
            / CLOTH_MASS
    }
    /// Closed-loop world force/torque from the actual native jaw states.
    pub fn feedback(&self) -> [(DVec3, DVec3); 2] {
        let t = self.sim.elapsed().as_secs_f64();
        let (x, vx) = path(t, 0.17, 0.32, CARRY);
        let (dz, vz) = path(t, 0.08, 0.40, LIFT);
        let z = 0.081 + dz;
        let q0 = DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
        let mut commands = [(DVec3::ZERO, DVec3::ZERO); 2];
        for (i, b) in self.jaws().into_iter().enumerate() {
            let sign = if i == 0 { 1. } else { -1. };
            let p = DVec3::from(b.position);
            let v = DVec3::from(b.velocity);
            let closing = self.mode != Mode::Open && t < RELEASE;
            let fy = if closing {
                -sign * self.force - 0.35 * v.y
            } else {
                50. * (sign * 0.052 - p.y) - 1.2 * v.y
            };
            let force = DVec3::new(
                80. * (x - p.x) + 1.55 * (vx - v.x),
                fy,
                JAW_MASS * GRAVITY + 80. * (z - p.z) + 1.55 * (vz - v.z),
            );
            let mut error = (q0 * DQuat::from_array(b.orientation).inverse()).normalize();
            if error.w < 0. {
                error = -error;
            }
            let torque =
                0.0030 * error.to_scaled_axis() - 0.00012 * DVec3::from(b.angular_velocity);
            commands[i] = (force, torque);
        }
        commands
    }
    /// Submit an external wrench tape to two ordinary dynamic links.
    pub fn step_wrenches(&mut self, commands: [(DVec3, DVec3); 2]) -> Result<(), Box<dyn Error>> {
        let t = self.sim.elapsed().as_secs_f64();
        for (i, (force, torque)) in commands.into_iter().enumerate() {
            queue_link_world_wrench(
                &mut self.sim,
                self.links[i].0,
                force.to_array(),
                torque.to_array(),
            )?;
        }
        self.sim
            .step_exact(1)
            .map_err(|e| format!("time {t}: {e}, fault {:?}", self.sim.fault()))?;
        Ok(())
    }
    pub fn step(&mut self) -> Result<[(DVec3, DVec3); 2], Box<dyn Error>> {
        let cmd = self.feedback();
        self.step_wrenches(cmd)?;
        Ok(cmd)
    }
}
