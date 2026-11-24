// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use bevy_ecs::prelude::Entity;
use glam::{DQuat, DVec3};
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
        Self::from_sim(sim, step_us, g)
    }

    fn from_sim(sim: Simulation, step_us: u32, g: f64) -> Self {
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

#[path = "support/pad_rocking_balance.rs"]
mod pad_rocking_balance;

#[path = "support/pad_patch.rs"]
mod pad_patch;
const STIFF_PADS: &str = include_str!("../../../examples/worlds/local_stiff_pad_grasp.sdf");

fn overlaps(rig: &Rig) -> [f64; 2] {
    let sphere = DVec3::from(rig.state().position);
    let backend = rig.sim.world().resource::<PhysicsBackend>();
    rig.pads.map(|id| {
        let pad = backend.state(id).unwrap();
        let relative =
            DQuat::from_array(pad.orientation).inverse() * (sphere - DVec3::from(pad.position));
        // Only the broad inner face of each 20 mm pad touches the 15 mm sphere.
        assert!(relative.y.abs() < 0.015 && relative.z.abs() < 0.050);
        0.010 + 0.015 - relative.x.abs()
    })
}

fn pad_mass(xml: &str, mass: f64) -> String {
    let mut xml = xml.replace("<mass>0.08</mass>", &format!("<mass>{mass}</mass>"));
    // Keep shape, density distribution and center of mass the same on both sides.
    for value in [
        "7.26666666666667e-05",
        "6.93333333333333e-05",
        "8.66666666666667e-06",
    ] {
        xml = xml.replace(
            value,
            &(value.parse::<f64>().unwrap() * mass / 0.08).to_string(),
        );
    }
    xml
}

#[test]
fn authored_pad_compression_tracks_force_and_stiffness_across_three_inertias() {
    for (mass, stiffness, drive) in [
        (0.04, 30_000., 3.),
        (0.08, 30_000., 3.),
        (0.16, 30_000., 3.),
        (0.08, 15_000., 3.),
        (0.08, 60_000., 4.5),
    ] {
        let world = pad_mass(STIFF_PADS, mass).replace(
            ">30000</normal_stiffness>",
            &format!(">{stiffness}</normal_stiffness>"),
        );
        let mut rig = Rig::new(&world, 500, -GRAVITY);
        rig.drive([drive; 2]);
        rig.sim.step_exact(16_000).unwrap();
        let loads = rig.advance(20_000);
        let depth = overlaps(&rig);
        let expected = drive / stiffness;
        println!(
            "pad_mass_kg={mass} stiffness_n_m={stiffness} force_n={drive} expected_m={expected} overlaps_m={depth:?}"
        );
        for (side, overlap) in depth.into_iter().enumerate() {
            near(overlap, expected, 1e-7);
            near(loads.normal[side] / 0.020, drive, 0.001);
            near(loads.tangent_z[side] / 0.020, MASS * GRAVITY / 2., 0.003);
        }
        near(rig.state().position[2], 0.30, 3e-6);
    }
}

/// Integrate vertical slip under the analytic time-varying normal force.
/// A stationary sphere stays put while static friction can carry its weight.
fn compliant_slip_reference() -> (f64, f64) {
    let dt = 1e-6;
    let mut drop = 0.0;
    let mut downward_speed = 0.0;
    for tick in 0..110_000 {
        let t = (f64::from(tick) + 0.5) * dt;
        // The 3 N squeeze started 50 ms before this 1.5 N reduction.
        let normal = 3.0 * pad_transient::centered_squeeze_force(t + 0.050)
            - 1.5 * pad_transient::centered_squeeze_force(t);
        let accel = GRAVITY - 2.0 * MU * normal / MASS;
        let next_speed = (downward_speed + accel * dt).max(0.0);
        drop += 0.5 * (next_speed + downward_speed) * dt;
        downward_speed = next_speed;
    }
    (drop, downward_speed)
}

#[test]
fn stiffer_pads_preserve_slip_and_release_through_loaded_reset() {
    let (expected_drop, expected_speed) = compliant_slip_reference();
    let mut rig = Rig::new(STIFF_PADS, STEP_US, -GRAVITY);
    let mut results = Vec::new();
    for _ in 0..2 {
        rig.drive([3.; 2]);
        rig.advance(50_000);
        let initial = rig.state();
        rig.drive([1.5; 2]);
        let slip = rig.advance(110_000);
        let end = rig.state();
        let drop = initial.position[2] - end.position[2];
        near(initial.velocity[2], 0.0, 1e-8);
        near(drop, expected_drop, 1e-6);
        // Allow a quarter public tick of steady sliding acceleration at slip onset.
        let slip_accel = GRAVITY - 2. * MU * 1.5 / MASS;
        near(-end.velocity[2], expected_speed, slip_accel * 0.0001 / 4.0);
        near(end.angular_velocity[1], 0., 0.01);
        for f in slip.tangent_z {
            near(f / 0.11, 0.75, 0.002);
        }
        rig.drive([-3.; 2]);
        rig.advance(50_000);
        let start_free = rig.state();
        let free = rig.advance(30_000);
        let fall = rig.state();
        assert_eq!(free.normal, [0.; 2]);
        near(
            fall.velocity[2] - start_free.velocity[2],
            -GRAVITY * 0.030,
            1e-8,
        );
        println!(
            "pad_slip_m={drop} predicted_m={expected_drop} predicted_speed_m_s={expected_speed} pad_slip_spin_rad_s={} post_release_z_m={}",
            end.angular_velocity[1], fall.position[2]
        );
        results.push((
            initial.position,
            end.position,
            slip.normal,
            slip.tangent_z,
            fall.velocity,
        ));
        gz_sim_scene::reset_loaded_world(&mut rig.sim, |_| ()).unwrap();
        rig = Rig::from_sim(rig.sim, STEP_US, -GRAVITY);
        near(rig.state().position[2], 0.30, 1e-12);
    }
    assert_eq!(results[0], results[1]);
}

#[test]
fn contact_material_validates_pad_ownership_and_independent_properties() {
    let load = |xml: &str, engine| {
        load_world_with_sdf_step(
            xml,
            "materials.sdf",
            "default",
            &sdformat::NoResources,
            engine,
        )
    };
    let left = r#"<normal_stiffness link="left">30000</normal_stiffness>"#;
    let restitution = r#"<restitution link="left">0.2</restitution>"#;
    assert!(
        load(
            &STIFF_PADS.replace(left, &format!("{restitution}{left}")),
            PhysicsEngine::MultiFidelity
        )
        .is_ok()
    );
    for bad in [
        left.replace("30000", "-3"),
        left.replace("30000", "0"),
        left.replace("30000", "inf"),
        left.replace("30000", "NaN"),
        left.replace("30000", "missing"),
        left.replace("left", "absent"),
        left.replace("left", "wrist"),
        left.replace("normal_stiffness", "unknown"),
        left.replace("link=", "name="),
        left.replace("link=", "units='N/m' link="),
        left.replace("30000", "<value>30000</value>"),
        format!("{left}{left}"),
        format!("{restitution}{restitution}"),
        restitution.replace("0.2", "1.2"),
    ] {
        assert!(
            load(
                &STIFF_PADS.replace(left, &bad),
                PhysicsEngine::MultiFidelity
            )
            .is_err(),
            "{bad}"
        );
    }
    let marker = r#"<plugin name="gz::sim::systems::ReducedScalarJoints" filename="gz-sim-reduced-scalar-joints-system"/>"#;
    for unsupported in [
        STIFF_PADS.replace(marker, ""),
        STIFF_PADS.replace(
            "<limit><lower>-0.008</lower><upper>0.008</upper></limit>",
            "",
        ),
    ] {
        assert!(load(&unsupported, PhysicsEngine::MultiFidelity).is_err());
    }
    assert!(load(STIFF_PADS, PhysicsEngine::Rapier).is_err());
}

#[path = "support/pad_transient.rs"]
mod pad_transient;

#[path = "support/pad_rocking.rs"]
mod pad_rocking;

#[path = "support/pad_edge_impact.rs"]
mod pad_edge_impact;

#[path = "support/offset_carry.rs"]
mod offset_carry;
