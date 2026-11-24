// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ordinary four-pad SDF and independent planar mechanics for its four-contact interval.
use glam::DVec3;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, deformable, load_world_with_sdf_step};
use std::error::Error;
pub const SDF: &str = include_str!("../../../../examples/articulated_friction_incline.sdf");
pub const SLOPE: f64 = 0.3;
pub const G: f64 = 9.81;
pub const MASS: f64 = 0.4;
pub const SKIN: f64 = 0.003;
pub const HALF_SPAN: f64 = 0.27;
pub const NAMES: [&str; 4] = ["palm", "left_finger", "right_finger", "rear_pad"];
pub fn normal() -> DVec3 {
    DVec3::new(SLOPE, 0., 1.).normalize()
}
pub fn downhill() -> DVec3 {
    DVec3::new(1., 0., -SLOPE).normalize()
}
pub fn acceleration(mu: f64, velocity_sign: f64) -> f64 {
    G * (SLOPE - velocity_sign * mu) * normal().z
}
/// Required per-site static Coulomb coefficient for the four initial contact
/// points. Their equal tangential shares act SKIN below the loaded midsurface.
/// The pressure difference between the two longitudinal pairs balances that couple.
pub fn critical() -> f64 {
    SLOPE / (1. - SKIN * SLOPE / HALF_SPAN)
}
pub fn sdf(mu: f64, speed: f64, width_ns: u64) -> String {
    let v = downhill() * speed;
    SDF.replace(r#"friction="0.45""#, &format!("friction=\"{mu}\""))
        .replace(
            r#"max_contact_step_ns="250000""#,
            &format!("max_contact_step_ns=\"{width_ns}\""),
        )
        .replace(
            r#"velocity="0 0 0""#,
            &format!("velocity=\"{} {} {}\"", v.x, v.y, v.z),
        )
}
pub struct Incline {
    pub sim: Simulation,
    pub ids: [(u64, BodyId); 4],
}
impl Incline {
    pub fn new(src: &str) -> Result<Self, Box<dyn Error>> {
        let sim = load_world_with_sdf_step(
            src,
            "inclined_sheet.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let ids = NAMES.map(|n| {
            let full = format!("default::gripper::{n}");
            let e = sim
                .world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|v| v.0 == full))
                .ok_or("missing inclined pad")?;
            Ok::<_, Box<dyn Error>>((
                e.get::<EntityId>().ok_or("missing stable ID")?.get(),
                e.get::<BodyHandle>().ok_or("missing rotor handle")?.id(),
            ))
        });
        let mut handles = vec![];
        for id in ids {
            handles.push(id?);
        }
        Ok(Self {
            sim,
            ids: handles.try_into().expect("four named pads"),
        })
    }
    pub fn material(&self) -> &deformable::DeformableSample {
        deformable::bodies(&self.sim).next().unwrap().1
    }
    pub fn states(&self) -> [BodyState; 4] {
        let physics = self.sim.world().resource::<PhysicsBackend>();
        self.ids
            .map(|(_, id)| physics.state(id).expect("native pad"))
    }
    pub fn centroid(&self) -> (DVec3, DVec3, f64) {
        let mut x = DVec3::ZERO;
        let mut v = DVec3::ZERO;
        let mut mass = 0.;
        for p in self.material().particles() {
            let m = 1. / p.inverse_mass;
            mass += m;
            x += m * DVec3::from(p.position);
            v += m * DVec3::from(p.velocity);
        }
        (x / mass, v / mass, mass)
    }
    pub fn step(&mut self) -> Result<(), Box<dyn Error>> {
        self.sim
            .step_exact(1)
            .map_err(|e| format!("at {:?}, {e}, {:?}", self.sim.elapsed(), self.sim.fault()).into())
    }
}
/// Gravity, static friction and kinetic friction admit an exact center-of-mass
/// trajectory as long as all four spherical contacts see the original plane.
pub fn planar_motion(t: f64, mu: f64, v0: f64) -> (f64, f64) {
    let a = if v0 < 0. {
        acceleration(mu, -1.)
    } else {
        acceleration(mu, 1.)
    };
    if v0 == 0. && mu >= critical() {
        return (0., 0.);
    }
    let turn = if a * v0 < 0. { -v0 / a } else { f64::INFINITY };
    if t <= turn {
        return (v0 * t + 0.5 * a * t * t, v0 + a * t);
    }
    let distance = 0.5 * v0 * turn;
    if mu >= critical() {
        (distance, 0.)
    } else {
        let elapsed = t - turn;
        let down = acceleration(mu, 1.);
        (distance + 0.5 * down * elapsed * elapsed, down * elapsed)
    }
}
