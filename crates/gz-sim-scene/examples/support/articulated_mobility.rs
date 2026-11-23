// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine,
    articulated_contact::{
        self, ArticulatedContactSample, ArticulatedImpulseReport, LinkContactSite,
    },
    load_world_with_sdf_step,
};
use std::error::Error;

pub const WORLD: &str = include_str!("../../../../examples/articulated_contact_mobility.sdf");
pub const NAMES: [&str; 3] = ["palm", "left_finger", "right_finger"];
pub const MASS: [f64; 3] = [2.4, 0.83, 0.76];
pub const CENTER: [[f64; 3]; 3] = [[0.; 3], [-0.035, 0.012, 0.008], [0.026, -0.018, 0.011]];
pub const INERTIA: [[f64; 3]; 3] = [
    [0.10, 0.09, 0.13],
    [0.026, 0.031, 0.039],
    [0.023, 0.033, 0.041],
];
pub const INERTIA_RPY: [[f64; 3]; 3] = [[0.; 3], [0.22, -0.14, 0.27], [-0.19, 0.15, 0.24]];
pub const LOCAL_POINTS: [[f64; 3]; 3] = [
    [0.045, -0.06, 0.085],
    [0.055, -0.075, 0.035],
    [0.038, 0.081, -0.024],
];

/// Matched centered grippers. A world revolute supports the rooted version
/// while all masses, local inertia orientations and fingertip sites coincide.
pub fn centered_fixture(grounded: bool) -> String {
    centered_xml(WORLD, grounded)
}
pub fn centered_xml(xml: &str, grounded: bool) -> String {
    let mut s = xml
        .replace("-0.035 0.012 0.008 0.22", "0 0 0 0.22")
        .replace("0.026 -0.018 0.011 -0.19", "0 0 0 -0.19");
    if grounded {
        s=s.replace(r#"<plugin filename="gz-sim-switchable-articulation-system" name="gz::sim::systems::SwitchableArticulation"/>"#,r#"<joint name="world_pivot" type="revolute"><parent>world</parent><child>palm</child><pose relative_to="palm">0 0 0 0 0 0</pose><axis><xyz expressed_in="__model__">0 0 1</xyz><limit><lower>-1.2</lower><upper>1.2</upper></limit></axis></joint>"#);
    }
    s
}
pub struct Gripper {
    pub sim: Simulation,
    pub id: u64,
    pub ids: [(u64, BodyId); 3],
}
impl Gripper {
    pub fn new(xml: &str) -> Result<Self, Box<dyn Error>> {
        let sim = load_world_with_sdf_step(
            xml,
            "articulated_contact_mobility.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let find = |name: &str| {
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap()
        };
        let id = find("default::gripper").get::<EntityId>().unwrap().get();
        let ids = NAMES.map(|name| {
            let e = find(&format!("default::gripper::{name}"));
            (
                e.get::<EntityId>().unwrap().get(),
                e.get::<BodyHandle>().unwrap().id(),
            )
        });
        Ok(Self { sim, id, ids })
    }
    pub fn states(&self) -> [BodyState; 3] {
        self.ids.map(|(_, id)| {
            self.sim
                .world()
                .resource::<PhysicsBackend>()
                .state(id)
                .unwrap()
        })
    }
    pub fn sites(&self) -> [LinkContactSite; 3] {
        std::array::from_fn(|i| LinkContactSite {
            link: self.ids[i].0,
            local_point: LOCAL_POINTS[i],
        })
    }
    pub fn sample(&mut self) -> Result<ArticulatedContactSample, Box<dyn Error>> {
        let sites = self.sites();
        Ok(articulated_contact::sample(&mut self.sim, &sites)?)
    }
    pub fn apply(&mut self, j: [DVec3; 3]) -> Result<ArticulatedImpulseReport, Box<dyn Error>> {
        let sites = self.sites();
        Ok(articulated_contact::apply(&mut self.sim, &sites, |_| {
            Ok(j.into_iter().map(|v| v.to_array()).collect())
        })?)
    }
}

pub fn pose(s: BodyState, local: [f64; 3]) -> DVec3 {
    DVec3::from(s.position) + DQuat::from_array(s.orientation) * DVec3::from(local)
}
pub fn point_velocity(i: usize, s: BodyState, local: [f64; 3]) -> DVec3 {
    DVec3::from(s.velocity)
        + DVec3::from(s.angular_velocity).cross(pose(s, local) - pose(s, CENTER[i]))
}
pub fn angular_mass(i: usize, s: BodyState, w: DVec3, inverse: bool) -> DVec3 {
    let [r, p, y] = INERTIA_RPY[i];
    let q = DQuat::from_array(s.orientation) * DQuat::from_euler(glam::EulerRot::ZYX, y, p, r);
    let diag = DVec3::from(INERTIA[i]);
    q * ((if inverse { diag.recip() } else { diag }) * (q.inverse() * w))
}
pub fn budgets(states: [BodyState; 3]) -> (DVec3, DVec3, f64) {
    let mut p = DVec3::ZERO;
    let mut l = DVec3::ZERO;
    let mut e = 0.;
    for (i, s) in states.into_iter().enumerate() {
        let v = DVec3::from(s.velocity);
        let w = DVec3::from(s.angular_velocity);
        let momentum = MASS[i] * v;
        let spin = angular_mass(i, s, w, false);
        p += momentum;
        l += pose(s, CENTER[i]).cross(momentum) + spin;
        e += 0.5 * (momentum.dot(v) + spin.dot(w));
    }
    (p, l, e)
}
