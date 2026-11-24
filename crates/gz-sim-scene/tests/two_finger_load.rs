// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use bevy_ecs::prelude::Entity;
use glam::DVec3;
use gz_physics::{BodyId, BodyState, JointState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, JointSample, Name, PhysicsEngine, load_world_with_sdf_step, set_joint_force,
};

const WORLD: &str = include_str!("../../../examples/worlds/local_two_finger_load.sdf");
const STEP_US: u32 = 100;
const MU: f64 = 0.5;
const SPHERE_INERTIA: f64 = 0.000018;
const MASS: f64 = 0.2;
const GRAVITY: f64 = 9.81;

fn named(sim: &Simulation, name: &str) -> Entity {
    #[allow(deprecated)]
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap()
        .id()
}

struct Rig {
    sim: Simulation,
    slides: [Entity; 2],
    pads: [BodyId; 2],
    parcel: BodyId,
    step_us: u32,
    gravity_z: f64,
}

impl Rig {
    fn new(world: &str, step_us: u32, g: f64) -> Self {
        let dt = f64::from(step_us) * 1e-6;
        let sdf = world
            .replace(">0.0001<", &format!(">{dt:.6}<"))
            .replace(">0 0 -9.81<", &format!(">0 0 {g}<"));
        let sim = load_world_with_sdf_step(
            &sdf,
            "two-finger-load.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let get_body = |name| {
            sim.world()
                .get::<BodyHandle>(named(&sim, name))
                .unwrap()
                .id()
        };
        let parcel = get_body("default::parcel::body");
        let pads = [
            get_body("default::gripper::left"),
            get_body("default::gripper::right"),
        ];
        let slides = [
            named(&sim, "default::gripper::left_slide"),
            named(&sim, "default::gripper::right_slide"),
        ];
        Self {
            sim,
            slides,
            pads,
            parcel,
            step_us,
            gravity_z: g,
        }
    }
    fn drive(&mut self, force: [f64; 2]) {
        for (slide, f) in self.slides.into_iter().zip(force) {
            let id = self.sim.world().get::<EntityId>(slide).unwrap().get();
            set_joint_force(&mut self.sim, id, f).unwrap();
        }
    }
    fn state(&self) -> BodyState {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .state(self.parcel)
            .unwrap()
    }
    fn slides(&self) -> [JointState; 2] {
        self.slides
            .map(|s| self.sim.world().get::<JointSample>(s).unwrap().state)
    }

    fn travel(&self) -> [f64; 2] {
        self.slides().map(|s| s.position)
    }

    fn read(&self) -> Reaction {
        let engine = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let mut sample = Reaction::default();
        for patch in engine.rigid_contact_patches() {
            let parcel_is_a = patch.a == self.parcel;
            let (pad, direction) = if parcel_is_a {
                (patch.b, 1.)
            } else {
                assert_eq!(patch.b, self.parcel);
                (patch.a, -1.)
            };
            let i = self.pads.iter().position(|&id| id == pad).unwrap();
            let normal = DVec3::from(patch.normal_on_a) * direction;
            for point in patch.points {
                let tangent = DVec3::from(point.tangent_impulse_on_a.unwrap()) * direction;
                let impulse = normal * point.normal_impulse + tangent;
                let lever = DVec3::from(if parcel_is_a {
                    point.lever_a
                } else {
                    point.lever_b
                });
                assert!(tangent.length() <= MU * point.normal_impulse + 1e-14);
                sample.normal[i] += point.normal_impulse;
                sample.tangent_z[i] += tangent.z;
                sample.linear[i] += impulse;
                sample.angular += lever.cross(impulse);
            }
        }
        sample
    }

    /// Accumulate both pads' delivered impulses over a physical interval, including every native Bevy tick.
    fn advance(&mut self, duration_us: u32) -> Reaction {
        assert_eq!(duration_us % self.step_us, 0);
        let start = self.state();
        let mut total = Reaction::default();
        for _ in 0..duration_us / self.step_us {
            self.sim.step_exact(1).unwrap();
            let step = self.read();
            for i in 0..2 {
                total.normal[i] += step.normal[i];
                total.tangent_z[i] += step.tangent_z[i];
                total.linear[i] += step.linear[i];
            }

            total.angular += step.angular;
        }
        let end = self.state();
        let duration = f64::from(duration_us) * 1e-6;
        let observed = MASS
            * (DVec3::from(end.velocity)
                - DVec3::from(start.velocity)
                - DVec3::Z * self.gravity_z * duration);
        let spin = SPHERE_INERTIA
            * (DVec3::from(end.angular_velocity) - DVec3::from(start.angular_velocity));
        let lin_error = (total.linear[0] + total.linear[1]).distance(observed);
        let ang_error = total.angular.distance(spin);
        println!(
            "phase duration={duration} dt_us={} normal={:?} tangent_z={:?} linres={lin_error:e} angres={ang_error:e}",
            self.step_us,
            total.normal.map(|x| x / duration),
            total.tangent_z.map(|x| x / duration)
        );
        assert!(
            lin_error < 1e-12,
            "linear impulse residual {lin_error:e} N s"
        );
        assert!(
            ang_error < 1e-14,
            "angular impulse residual {ang_error:e} N m s"
        );
        total
    }
}

#[derive(Default)]
struct Reaction {
    normal: [f64; 2],
    tangent_z: [f64; 2],
    linear: [DVec3; 2],
    angular: DVec3,
}

fn near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() < tolerance,
        "observed {actual:.12}, expected {expected:.12} within {tolerance:e}"
    );
}

#[test]
fn symmetric_jaws_hold_slip_then_release_through_both_travel_stops() {
    let mut previous_error = f64::INFINITY;
    for step_us in [200, 100, 50] {
        let mut rig = Rig::new(WORLD, step_us, -GRAVITY);
        rig.drive([3.; 2]);
        rig.advance(30_000);
        let held = rig.advance(20_000);
        for i in 0..2 {
            near(held.normal[i] / 0.02, 3., 0.002);
            near(held.tangent_z[i] / 0.02, MASS * GRAVITY / 2., 0.002);
        }
        let start = rig.state();
        near(start.position[2], 0.30, 1e-6);
        near(start.velocity[2], 0., 1e-8);
        near(start.angular_velocity[1], 0., 1e-6);
        rig.drive([1.5; 2]);
        let slip = rig.advance(110_000);
        let end = rig.state();
        let acceleration = 2. * MU * 1.5 / MASS - GRAVITY;
        for i in 0..2 {
            near(slip.normal[i] / 0.11, 1.5, 0.002);
            near(slip.tangent_z[i] / 0.11, 1.5 * MU, 0.002);
        }
        let trajectory_error =
            end.position[2] - start.position[2] - 0.5 * acceleration * 0.11_f64.powi(2);
        let velocity_error = end.velocity[2] - start.velocity[2] - acceleration * 0.11;
        println!(
            "slip dt={step_us}us dz={:.12} vel_error={velocity_error:e} traj_error={trajectory_error:e} jaw={:?} spin_y_rad_s={:.6}",
            end.position[2] - start.position[2],
            rig.travel(),
            end.angular_velocity[1]
        );
        near(velocity_error, 0., 1e-4);
        near(trajectory_error, 0., 2e-6);
        assert!(trajectory_error.abs() < previous_error);
        previous_error = trajectory_error.abs();
        near(end.angular_velocity[1], 0., 0.12);
        for q in rig.travel() {
            assert!((0. ..0.0003).contains(&q));
        }
        rig.drive([-3.; 2]);
        let open = rig.advance(80_000);
        assert_eq!(open.normal, [0.; 2]);
        assert_eq!(open.tangent_z, [0.; 2]);
        for q in rig.travel() {
            near(q, -0.008, 1e-8);
        }
        let fall = rig.state();
        near(fall.velocity[2] - end.velocity[2], -GRAVITY * 0.08, 1e-8);
        rig.drive([3.; 2]);
        let close = rig.advance(50_000);
        assert_eq!(close.normal, [0.; 2]);
        for q in rig.travel() {
            near(q, 0.008, 1e-8);
        }
        assert!(rig.state().position[2] < 0.18);
    }
}

#[test]
fn friction_threshold_brackets_weight_in_both_gravity_directions() {
    let critical = MASS * GRAVITY / (2. * MU);
    for direction in [-1., 1.] {
        for factor in [0.98, 1.02] {
            let mut rig = Rig::new(WORLD, STEP_US, direction * GRAVITY);
            let drive = critical * factor;
            rig.drive([drive; 2]);
            rig.advance(10_000);
            let start = rig.state();
            let loads = rig.advance(60_000);
            let end = rig.state();
            let wanted = GRAVITY * (1. - factor).max(0.);
            near(
                (end.velocity[2] - start.velocity[2]) * direction / 0.06,
                wanted,
                0.0005,
            );
            for i in 0..2 {
                near(loads.normal[i] / 0.06, drive, 0.001);
                near(
                    -direction * loads.tangent_z[i] / 0.06,
                    (0.5 * MASS * GRAVITY).min(MU * drive),
                    0.0015,
                );
            }
        }
    }
}

#[test]
fn loaded_stop_releases_and_transfers_to_the_other_jaw() {
    for first in [0, 1] {
        let mut rig = Rig::new(WORLD, STEP_US, -GRAVITY);
        rig.drive([3.; 2]);
        rig.advance(50_000);
        for stronger in [first, 1 - first] {
            let mut force = [2.5; 2];
            force[stronger] = 6.;
            rig.drive(force);
            rig.advance(180_000);
            let before = rig.slides();
            let settled = rig.advance(20_000);
            let after = rig.slides();
            let q = rig.travel();
            // The weaker jaw has travel available. Its drive therefore sets the normal load.
            near(q[stronger], 0.008, 5e-8);
            assert!((-0.00795..-0.006).contains(&q[1 - stronger]), "q={q:?}");
            for i in 0..2 {
                near(settled.normal[i] / 0.02, 2.5, 0.003);
                near(settled.tangent_z[i] / 0.02, MASS * GRAVITY / 2., 0.01);
            }
            let stop_loads = [0, 1].map(|i| {
                let inward = if i == 0 { 1. } else { -1. };
                let contact = settled.linear[i].x * inward / 0.02;
                let inertia = 0.08 * (after[i].velocity - before[i].velocity) / 0.02;
                force[i] - contact - inertia
            });
            near(stop_loads[stronger], 3.5, 0.003);
            near(stop_loads[1 - stronger], 0., 1e-9);
            println!("stronger={stronger} settled_stop_loads_n={stop_loads:?} travel_m={q:?}");
            let parcel = rig.state();
            near(parcel.velocity[2], 0., 1e-8);
            near(parcel.angular_velocity[1], 0., 1e-6);
            assert!(parcel.position[2] > 0.297);
            assert!(parcel.position[0] * (1. - 2. * stronger as f64) > 0.0074);
        }
    }
}

#[test]
fn parcel_first_preserves_signed_weight_support_on_each_pad() {
    let xml = roxmltree::Document::parse(WORLD).unwrap();
    let models: Vec<_> = xml
        .descendants()
        .filter(|n| n.has_tag_name("model"))
        .map(|n| n.range())
        .collect();
    let [gripper, parcel] = &models[..] else {
        panic!("two models required")
    };
    let mut switched = WORLD.to_owned();
    switched.replace_range(
        gripper.start..parcel.end,
        &format!("{}\n{}", &WORLD[parcel.clone()], &WORLD[gripper.clone()]),
    );
    let mut rig = Rig::new(&switched, STEP_US, -GRAVITY);
    assert!(rig.pads.iter().all(|&p| rig.parcel < p));
    rig.drive([3.; 2]);
    rig.advance(30_000);
    let held = rig.advance(20_000);
    for i in 0..2 {
        near(held.normal[i] / 0.02, 3., 0.002);
        near(held.tangent_z[i] / 0.02, MASS * GRAVITY / 2., 0.002);
    }
}
