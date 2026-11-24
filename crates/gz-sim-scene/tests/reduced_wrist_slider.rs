// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::{MultiFidelityBackend, RobotJoint};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, JointSample, Name, PhysicsEngine, load_local_replay_world,
    load_world_with_sdf_step, queue_link_force_at_point,
};
use std::time::Instant;

const SDF: &str = include_str!("../../../examples/worlds/local_reduced_wrist_slider.sdf");
const MARKER: &str = r#"<plugin name="gz::sim::systems::ReducedScalarJoints" filename="gz-sim-reduced-scalar-joints-system"/>"#;
const WRIST_I: f64 = 0.0004266666666666667;
const FINGER_I: f64 = 0.00003208666666666667;
const RAIL_Y: f64 = 0.10;
const FINGER_M: f64 = 0.08;
const SPHERE_M: f64 = 0.20;
const SPEED: f64 = 0.75;

fn xml(ns: u32, side: f64, offset: f64, restitution: f64) -> String {
    SDF.replace(">0.00001<", &format!(">{:.9}<", f64::from(ns) * 1e-9))
        .replace(
            "-0.02405 0.12",
            &format!("{} {}", -side * 0.02405, RAIL_Y + offset),
        )
        .replace(">0.75 0 0<", &format!(">{} 0 0<", side * SPEED))
        .replace(">0.6<", &format!(">{restitution}<"))
}

fn load(sdf: &str) -> Simulation {
    load_world_with_sdf_step(
        sdf,
        "wrist-slider.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn entity<'a>(sim: &'a Simulation, name: &str) -> bevy_ecs::world::EntityRef<'a> {
    #[allow(deprecated)]
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
}

#[derive(Clone, Copy)]
struct Rig {
    finger: u64,
    bodies: [BodyId; 3],
}
impl Rig {
    fn new(sim: &Simulation) -> Self {
        let names = [
            "default::wrist_rig::wrist",
            "default::wrist_rig::finger",
            "default::parcel::body",
        ];
        Self {
            finger: entity(sim, names[1]).get::<EntityId>().unwrap().get(),
            bodies: names.map(|n| entity(sim, n).get::<BodyHandle>().unwrap().id()),
        }
    }
    fn states(self, sim: &Simulation) -> [BodyState; 3] {
        let physics = sim.world().resource::<PhysicsBackend>();
        self.bodies.map(|id| physics.state(id).unwrap())
    }
    fn contact(self, sim: &Simulation) -> [f64; 2] {
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let mut result = [0.; 2];
        for patch in native.rigid_contact_patches() {
            assert!(
                [patch.a, patch.b].contains(&self.bodies[1])
                    && [patch.a, patch.b].contains(&self.bodies[2])
            );
            let on_a = patch.a == self.bodies[1];
            for p in &patch.points {
                let sign = if on_a { 1. } else { -1. };
                let force = patch.normal_on_a.map(|n| sign * n * p.normal_impulse);
                let r = if on_a { p.lever_a } else { p.lever_b };
                result[0] += force[0];
                result[1] += r[0] * force[1] - r[1] * force[0];
            }
        }
        result
    }
}

fn near(actual: f64, expected: f64, tol: f64) {
    assert!(
        (actual - expected).abs() <= tol,
        "{actual} != {expected} (+/- {tol})"
    );
}
fn joint(sim: &Simulation, name: &str) -> JointSample {
    *entity(sim, &format!("default::wrist_rig::{name}"))
        .get::<JointSample>()
        .unwrap()
}

#[test]
fn elastic_and_plastic_strikes_use_the_constrained_mass_at_first_contact() {
    for (side, offset, e) in [
        (1., 0.02, 0.6),
        (-1., 0.02, 0.6),
        (1., -0.02, 0.6),
        (1., 0., 0.6),
        (1., 0.02, 0.),
    ] {
        for ns in [10_000, 1_000, 100, 25] {
            let mut sim = load(&xml(ns, side, offset, e));
            let rig = Rig::new(&sim);
            let mass_inverse =
                1. / SPHERE_M + 1. / FINGER_M + offset * offset / (WRIST_I + FINGER_I);
            let impulse = side * (1. + e) * SPEED / mass_inverse;
            let w = -offset * impulse / (WRIST_I + FINGER_I);
            for _ in 0..100_000 / ns {
                sim.step_exact(1).unwrap();
                let [j, t] = rig.contact(&sim);
                if j == 0. {
                    continue;
                }
                near(j, impulse, 1e-9);
                near(t, -offset * impulse, 1e-10);
                let [wrist, finger, ball] = rig.states(&sim);
                near(ball.velocity[0], side * SPEED - impulse / SPHERE_M, 1e-8);
                near(wrist.angular_velocity[2], w, 1e-7);
                near(finger.angular_velocity[2], w, 1e-7);
                near(joint(&sim, "hinge").state.velocity, w, 1e-7);
                near(
                    joint(&sim, "slider").state.velocity,
                    impulse / FINGER_M + RAIL_Y * w,
                    1e-7,
                );
                break;
            }
            assert!(rig.contact(&sim)[0] != 0.);
        }
    }
}

#[test]
fn replay_reset_and_off_center_force_preserve_the_native_coordinates() {
    let sdf = xml(1000, 1., 0.07, 0.6).replace(
        MARKER,
        &MARKER.replace(
            "/>",
            ">\n<!-- native generalized coordinates -->\n</plugin>",
        ),
    );
    let mut sim = load_local_replay_world(&sdf, "wrist-slider.sdf", "default", [5; 32]).unwrap();
    let rig = Rig::new(&sim);
    let run = |sim: &mut Simulation| {
        queue_link_force_at_point(sim, rig.finger, [-0.009, 0.02, 0.], [40_000., 0., 0.]).unwrap();
        sim.step_exact(1).unwrap();
        let [wrist, finger, _] = rig.states(sim);
        let w = -0.02 * 0.04 / (WRIST_I + FINGER_I);
        near(wrist.angular_velocity[2], w, 1e-8);
        near(finger.angular_velocity[2], w, 1e-8);
        near(
            joint(sim, "slider").state.velocity,
            0.04 / FINGER_M + RAIL_Y * w,
            1e-8,
        );
        for _ in 0..200 {
            sim.step_exact(1).unwrap();
            assert_eq!(rig.contact(sim), [0.; 2]);
        }
        [joint(sim, "hinge").state, joint(sim, "slider").state]
    };
    let first = run(&mut sim);
    sim.reset().unwrap();
    for name in ["hinge", "slider"] {
        let state = joint(&sim, name);
        near(state.state.position, 0., 0.);
        near(state.state.velocity, 0., 0.);
    }
    assert_eq!(first, run(&mut sim));
}

#[test]
fn unsupported_ownership_and_joint_features_fail_before_a_world_is_returned() {
    let bad = [
        SDF.replace(MARKER,&MARKER.replace("/>", ">unexpected</plugin>")),
        SDF.replace("<parent>wrist</parent>","<parent>world</parent>"),
        SDF.replace(MARKER,&format!(r#"{MARKER}<plugin name="gz::sim::systems::ApplyJointForce" filename="gz-sim-apply-joint-force-system"><joint_name>hinge</joint_name></plugin>"#)),
        SDF.replace(MARKER,&format!("{MARKER}{MARKER}")),
        SDF.replace(MARKER,&MARKER.replace("/>","><unknown/></plugin>")),
        SDF.replace(MARKER,&MARKER.replace("/>"," extra=\"1\"/>")),
        SDF.replace("<xyz>0 0 1</xyz>","<xyz>0 0 1</xyz><limit><upper>0.1</upper></limit>"),
        SDF.replace("<xyz>1 0 0</xyz>","<xyz>1 0 0</xyz><dynamics><damping>0.1</damping></dynamics>"),
        SDF.replace("type=\"prismatic\"","type=\"revolute\""),
        SDF.replace("type=\"revolute\"","type=\"fixed\"").replace("<axis><xyz>0 0 1</xyz></axis>",""),
        SDF.replace(MARKER,&format!(r#"{MARKER}<plugin name="gz::sim::systems::SwitchableArticulation" filename="gz-sim-switchable-articulation-system"/>"#)),
        SDF.replace("<joint name=\"hinge\" type=\"revolute\"><parent>world</parent><child>wrist</child><axis><xyz>0 0 1</xyz></axis></joint>",""),
    ];
    for sdf in bad {
        assert!(
            load_world_with_sdf_step(
                &sdf,
                "case",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
    assert!(
        load_world_with_sdf_step(
            SDF,
            "case",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
    let mut sim = load(SDF);
    let rig = Rig::new(&sim);
    let mut p = sim.world_mut().resource_mut::<PhysicsBackend>();
    let native = p.downcast_mut::<MultiFidelityBackend>().unwrap();
    assert!(
        native
            .add_reduced_scalar_robot_joint(
                None,
                rig.bodies[1],
                [0.; 3],
                RobotJoint::Revolute {
                    axis: [1., 0., 0.],
                    limits: Some([-1., 1.])
                }
            )
            .is_err()
    );
}

#[test]
fn measure_the_actual_loaded_tick_in_both_native_representations() {
    for (mode, sdf) in [
        ("impulse", SDF.replace(MARKER, "")),
        ("reduced", SDF.to_string()),
    ] {
        let mut sim = load(&sdf);
        let rig = Rig::new(&sim);
        let mut samples = Vec::new();
        for _ in 0..201 {
            sim.step_exact(6).unwrap();
            assert_eq!(rig.contact(&sim), [0.; 2]);
            let start = Instant::now();
            sim.step_exact(1).unwrap();
            samples.push(start.elapsed().as_nanos());
            assert!(rig.contact(&sim)[0] > 0.);
            sim.reset().unwrap();
        }
        samples.sort();
        println!(
            "{mode} real 10 us impact tick, 201 repetitions: p50 {} ns, p95 {} ns, p99 {} ns",
            samples[100], samples[190], samples[198]
        );
    }
}

#[test]
fn mirror_trajectories_converge_to_the_independent_two_coordinate_flight() {
    // Independent Lagrange/RK4 reference at t = 24 ms. The sphere reaches the jaw
    // at 50 µm / 0.75 m/s. See the measured equations and oracle commit in the receipt.
    let finger = [0.019516200186981948, 0.09891081767604315];
    let sphere_x = -0.0138662726249825;
    for side in [1., -1.] {
        for ns in [10_000, 1_000] {
            let mut sim = load(&xml(ns, side, 0.02, 0.6));
            let rig = Rig::new(&sim);
            sim.step_exact(24_000_000 / u64::from(ns)).unwrap();
            let [_, pad, ball] = rig.states(&sim);
            let tolerance = 0.3 * f64::from(ns) * 1e-9;
            near(
                (pad.position[0] - side * finger[0]).hypot(pad.position[1] - finger[1]),
                0.,
                tolerance,
            );
            near(ball.position[0], side * sphere_x, tolerance);
            near(
                joint(&sim, "hinge").state.velocity,
                side * -2.7480297673202796,
                4e-5,
            );
            near(
                joint(&sim, "slider").state.velocity,
                side * 0.5428560256679728,
                5e-6,
            );
        }
    }
}
