// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Six-second asymmetric rotor experiment and an independent small-step RK4 reference.
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, queue_link_world_wrench,
};
use std::error::Error;

pub const XML: &str = include_str!("../free_asymmetric_top.sdf");
pub const INERTIA: DVec3 = DVec3::new(0.045, 0.067, 0.09);
pub fn axes() -> DQuat {
    DQuat::from_rotation_z(0.18) * DQuat::from_rotation_y(-0.36) * DQuat::from_rotation_x(0.25)
}
pub fn without_integrator(xml: &str) -> String {
    let start = xml
        .find("    <plugin name=\"gz::sim::systems::FreeRotation\"")
        .unwrap();
    let end = start + xml[start..].find("</plugin>").unwrap() + "</plugin>".len();
    format!("{}{}", &xml[..start], &xml[end..])
}
pub struct Run {
    pub sim: Simulation,
    pub body: BodyId,
    link: u64,
}
impl Run {
    pub fn new(enabled: bool, ms: u32) -> Result<Self, Box<dyn Error>> {
        let mut xml = XML.replace(
            "<max_step_size>0.01",
            &format!("<max_step_size>{}", f64::from(ms) * 0.001),
        );
        if !enabled {
            xml = without_integrator(&xml);
        }
        let sim = load_world_with_sdf_step(
            &xml,
            "free_asymmetric_top.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let entity = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::top::body"))
            .ok_or("missing rotor link")?;
        let body = entity.get::<BodyHandle>().ok_or("missing rotor body")?.id();
        let link = entity
            .get::<EntityId>()
            .ok_or("missing rotor identity")?
            .get();
        Ok(Self { sim, body, link })
    }
    pub fn advance(&mut self, tau: DVec3) -> Result<(), Box<dyn Error>> {
        if tau != DVec3::ZERO {
            queue_link_world_wrench(&mut self.sim, self.link, [0.; 3], tau.to_array())?;
        }
        self.sim.step_exact(1)?;
        Ok(())
    }
    pub fn state(&self) -> BodyState {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .state(self.body)
            .unwrap()
    }
}
pub fn torque(start_ms: u32, driven: bool) -> DVec3 {
    if !driven {
        return DVec3::ZERO;
    }
    match start_ms {
        1000..2200 => DVec3::new(0.14, -0.10, 0.19),
        3400..4400 => DVec3::new(-0.19, 0.08, -0.07),
        _ => DVec3::ZERO,
    }
}
pub fn spatial(s: BodyState) -> (DVec3, f64) {
    let q = DQuat::from_array(s.orientation) * axes();
    let w = DVec3::from(s.angular_velocity);
    let l = q * (INERTIA * (q.inverse() * w));
    (l, 0.5 * l.dot(w))
}
#[derive(Clone, Copy)]
pub struct Reference {
    pub principal_to_world: DQuat,
    pub momentum: DVec3,
}
impl Reference {
    pub fn new(s: BodyState) -> Self {
        Self {
            principal_to_world: DQuat::from_array(s.orientation) * axes(),
            momentum: spatial(s).0,
        }
    }
    pub fn attitude(self) -> DQuat {
        (self.principal_to_world * axes().conjugate()).normalize()
    }
    pub fn spin(self) -> DVec3 {
        self.principal_to_world * ((self.principal_to_world.inverse() * self.momentum) / INERTIA)
    }
    pub fn energy(self) -> f64 {
        0.5 * self.momentum.dot(self.spin())
    }
    // q' = 1/2 [0, world omega] q, with world L(t) = L(0) + t*torque.
    // This independent reference neither fixes energy nor solves a midpoint equation.
    pub fn advance(&mut self, dt: f64, tau: DVec3, substeps: u32) {
        fn derivative(q: DQuat, l: DVec3) -> DQuat {
            let q = q.normalize();
            let w = q * ((q.inverse() * l) / INERTIA);
            DQuat::from_xyzw(w.x, w.y, w.z, 0.) * q * 0.5
        }
        let h = dt / f64::from(substeps);
        let initial = self.momentum;
        for i in 0..substeps {
            let t = h * f64::from(i);
            let q = self.principal_to_world;
            let l = initial + t * tau;
            let mid = l + 0.5 * h * tau;
            let a = derivative(q, l);
            let b = derivative(q + a * (0.5 * h), mid);
            let c = derivative(q + b * (0.5 * h), mid);
            let d = derivative(q + c * h, l + h * tau);
            self.principal_to_world = (q + (a + b * 2. + c * 2. + d) * (h / 6.)).normalize();
        }
        self.momentum = initial + dt * tau;
    }
}
pub fn attitude_error(a: DQuat, b: DQuat) -> f64 {
    let q = (a.inverse() * b).normalize();
    2. * DVec3::new(q.x, q.y, q.z).length().atan2(q.w.abs())
}
