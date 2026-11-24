// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::grasp::{self, Grasp, Mode};
use glam::DVec3;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine,
    deformable::{DeformableSample, bodies},
    load_world_with_sdf_step, queue_link_world_wrench,
};
use std::error::Error;
pub const PLATEN: &str = include_str!("../../../../examples/force_aware_platen.sdf");
pub type Wrenches = [(DVec3, DVec3); 2];
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CarryInput {
    Balanced,
    IdenticalTape,
    Feedback,
    Constant,
}
pub struct Platen {
    pub sim: Simulation,
    id: u64,
    native: BodyId,
}
impl Platen {
    pub fn new(aware: bool, n: u32, dt: f64) -> Result<Self, Box<dyn Error>> {
        let mut xml = PLATEN
            .replace("substeps=\"4\"", &format!("substeps=\"{n}\""))
            .replace("<max_step_size>0.004", &format!("<max_step_size>{dt}"));
        if !aware {
            xml = xml.replace(
                "linear_trajectory=\"force_aware\"",
                "linear_trajectory=\"native_chord\"",
            );
        }
        let sim = load_world_with_sdf_step(
            &xml,
            "force_aware_platen.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let e = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::platen::piston")
            })
            .unwrap();
        let id = e.get::<EntityId>().unwrap().get();
        let native = e.get::<BodyHandle>().unwrap().id();
        Ok(Self { sim, id, native })
    }
    pub fn state(&self) -> BodyState {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .state(self.native)
            .unwrap()
    }
    pub fn cloth(&self) -> &DeformableSample {
        bodies(&self.sim).next().unwrap().1
    }
    pub fn step(&mut self, f: f64) -> Result<(), Box<dyn Error>> {
        queue_link_world_wrench(&mut self.sim, self.id, [0., 0., f], [0.; 3])?;
        self.sim
            .step_exact(1)
            .map_err(|e| format!("{e}, fault {:?}", self.sim.fault()))?;
        Ok(())
    }
}
pub fn load(t: f64) -> f64 {
    4. + 1.5 * (std::f64::consts::TAU * t).sin()
}
pub fn pair(aware: bool, n: u32, dt: f64, case: CarryInput) -> Result<Grasp, Box<dyn Error>> {
    pair_at(aware, n, dt, case, 0.0625)
}
pub fn pair_at(
    aware: bool,
    n: u32,
    dt: f64,
    case: CarryInput,
    height: f64,
) -> Result<Grasp, Box<dyn Error>> {
    Grasp::new(
        Mode::Closed,
        pair_description(aware, n, dt, case, height),
        grasp::SQUEEZE,
    )
}
pub fn pair_description(aware: bool, n: u32, dt: f64, case: CarryInput, height: f64) -> String {
    let mut xml = grasp::WORLD
        .replace("substeps=\"4\"", &format!("substeps=\"{n}\""))
        .replace("<max_step_size>0.001", &format!("<max_step_size>{dt}"))
        .replace(
            "capsule_contact=\"material\"",
            "capsule_contact=\"material\" capsule_contact_history=\"true\"",
        );
    if aware {
        xml = xml.replace(
            "coupling=\"two_way\"",
            "coupling=\"two_way\" linear_trajectory=\"force_aware\"",
        );
    }
    if case == CarryInput::Constant {
        xml = xml.replace("<plane point=\"0 0 0\"", "<plane point=\"0 0 -0.3\"");
    }
    if case == CarryInput::Balanced {
        xml = xml
            .replace("0.081 1.570796", &format!("{height} 1.570796"))
            .replace("<plane point=\"0 0 0\"", "<plane point=\"0 0 -2\"");
    }
    xml
}
pub fn command(case: CarryInput, ref_run: &Grasp, actual: &Grasp) -> Wrenches {
    match case {
        CarryInput::IdenticalTape => ref_run.feedback(),
        CarryInput::Feedback => actual.feedback(),
        CarryInput::Balanced => {
            let t = ref_run.sim.elapsed().as_secs_f64();
            // Open-loop symmetric jaw forcing. If the cloth sticks, the whole
            // assembly follows x = A(1 - cos wt); each jaw supplies half the
            // cloth drive, weight, and their true fingertip moment.
            let omega = std::f64::consts::TAU / 1.05;
            let a = 0.006 * omega * omega * (omega * t).cos();
            let weight = grasp::GRAVITY * DVec3::Z;
            let carry = a * DVec3::X + weight;
            let squeeze = grasp::SQUEEZE * (1. + 0.16 * (2. * omega * t).sin());
            [1., -1.].map(|sign| {
                let arm = -sign * (grasp::RADIUS + grasp::HALF_LENGTH) * DVec3::Y;
                let on_cloth = 0.5 * grasp::CLOTH_MASS * carry;
                (
                    grasp::JAW_MASS * carry + on_cloth - sign * squeeze * DVec3::Y,
                    arm.cross(on_cloth),
                )
            })
        }
        CarryInput::Constant => {
            let support = 0.5 * grasp::CLOTH_MASS * grasp::GRAVITY;
            [1., -1.].map(|sign| {
                let force = DVec3::new(
                    0.,
                    -sign * grasp::SQUEEZE,
                    grasp::JAW_MASS * grasp::GRAVITY + support,
                );
                let normal_arm = -sign * (grasp::RADIUS + grasp::HALF_LENGTH) * DVec3::Y;
                let torque = -normal_arm.cross(-support * DVec3::Z);
                (force, torque)
            })
        }
    }
}

#[derive(Default)]
pub struct Window {
    pub duration: f64,
    pub count: usize,
    pub duty: f64,
    pub normal: f64,
    pub normal_squared: f64,
    pub slip_sum: f64,
    pub max_slip: f64,
    pub received: f64,
    pub max_force: f64,
}
impl Window {
    pub fn add(&mut self, s: &DeformableSample, dt: f64, boundaries: usize, nominal: f64) {
        self.duration += dt * boundaries as f64;
        let mut grouped = std::collections::BTreeMap::<(usize, u64, u64), f64>::new();
        for sample in s.last_capsule_contact_samples() {
            let [start, end] = sample.interval;
            let key = (sample.boundary, start.to_bits(), end.to_bits());
            *grouped.entry(key).or_default() += sample.normal_impulse;
            let speed = DVec3::from(sample.slip_velocity).length();
            self.slip_sum += sample.normal_impulse * speed;
            if sample.normal_impulse > 0.01 * nominal * (end - start) {
                self.max_slip = self.max_slip.max(speed);
            }
        }
        for ((_, a, b), j) in grouped {
            let h = f64::from_bits(b) - f64::from_bits(a);
            let normal = j / h;
            self.normal += j;
            self.normal_squared += h * normal * normal;
            self.max_force = self.max_force.max(normal);
            if normal > 0.01 * nominal {
                self.duty += h;
            }
            self.count += 1;
        }
        self.received -= s
            .last_boundary_impulses()
            .iter()
            .take(boundaries)
            .map(|j| j.linear[2])
            .sum::<f64>();
    }
    pub fn rms_variation(&self) -> f64 {
        (self.normal_squared / self.duration - (self.normal / self.duration).powi(2))
            .max(0.)
            .sqrt()
    }
    pub fn weighted_slip(&self) -> f64 {
        self.slip_sum / self.normal.max(f64::MIN_POSITIVE)
    }
}
