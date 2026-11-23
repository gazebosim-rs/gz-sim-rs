// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3, DVec4};
use gz_physics::{
    BodyId, BodyState, JointId, JointState, JointVelocityMotor, PhysicsBackend, PhysicsError,
};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, FidelityError, JointHandle, JointSample, Name, PhysicsEngine,
    Velocity, WorldPose, load_world_with_sdf_step, model_fidelity, queue_joint_effort,
    queue_link_world_wrench, remove_model, reset_loaded_world, set_model_fidelity,
};
use sdformat::NoResources;
use std::time::Duration;

// All coordinates in the observations below come from Gazebo link frames. The
// three principal moments are equal on each link, but their masses, COM offsets,
// and inertias differ so a root-only handoff does not conserve the assemblage.
const EXAMPLE: &str = include_str!("../../../examples/multifidelity_articulation.sdf");
const MASS: [f64; 2] = [2.0, 0.8];
const INERTIA: [f64; 2] = [0.12, 0.036];
const COM: [DVec3; 2] = [DVec3::new(0.07, -0.03, 0.0), DVec3::new(-0.04, 0.02, 0.0)];

fn load(source: &str) -> Simulation {
    load_world_with_sdf_step(
        source,
        "multifidelity_articulation.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn load_at(dt: Duration) -> Simulation {
    let source = EXAMPLE.replace(
        "<max_step_size>0.001</max_step_size>",
        &format!("<max_step_size>{}</max_step_size>", dt.as_secs_f64()),
    );
    let sim = load(&source);
    assert_eq!(sim.step_duration(), dt);
    sim
}

#[derive(Debug, PartialEq)]
struct Arm {
    model: u64,
    links: [u64; 2],
    bodies: [BodyId; 2],
    hinge: u64,
    joint: JointId,
}

impl Arm {
    fn find(sim: &Simulation) -> Self {
        let id = |name: &str| {
            let name = format!("default::{name}");
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get()
        };
        let links = [id("robot::base"), id("robot::tool")];
        let bodies = links.map(|link| {
            sim.world()
                .get::<BodyHandle>(entity_by_id(sim.world(), link).unwrap())
                .unwrap()
                .id()
        });
        let hinge = id("robot::hinge");
        let joint = sim
            .world()
            .get::<JointHandle>(entity_by_id(sim.world(), hinge).unwrap())
            .unwrap()
            .id();
        Self {
            model: id("robot"),
            links,
            bodies,
            hinge,
            joint,
        }
    }

    fn sample(&self, sim: &Simulation) -> ([BodyState; 2], JointState) {
        let backend = sim.world().resource::<PhysicsBackend>();
        let links = std::array::from_fn(|i| {
            let state = backend.state(self.bodies[i]).unwrap();
            let entity = entity_by_id(sim.world(), self.links[i]).unwrap();
            let pose = sim.world().get::<WorldPose>(entity).unwrap();
            let twist = sim.world().get::<Velocity>(entity).unwrap();
            near(
                DVec3::from_array(state.position),
                DVec3::from_array(pose.translation),
                1e-12,
            );
            let solver_rotation = DQuat::from_array(state.orientation);
            let observed_rotation = DQuat::from_array(pose.rotation);
            let sign = solver_rotation.dot(observed_rotation).signum();
            assert!((solver_rotation - observed_rotation * sign).length() < 1e-12);
            near(
                DVec3::from_array(state.angular_velocity),
                DVec3::from_array(twist.angular),
                1e-10,
            );
            let arm = DQuat::from_array(pose.rotation) * COM[i];
            let observed_com_velocity =
                DVec3::from_array(twist.linear) + DVec3::from_array(twist.angular).cross(arm);
            near(
                observed_com_velocity,
                DVec3::from_array(state.velocity),
                1e-10,
            );
            state
        });
        let joint = backend.joint_state(self.joint).unwrap();
        let entity = entity_by_id(sim.world(), self.hinge).unwrap();
        let sample = sim.world().get::<JointSample>(entity).unwrap();
        assert_eq!(sample.time, sim.elapsed());
        assert!((sample.state.position - joint.position).abs() < 1e-10);
        assert!((sample.state.velocity - joint.velocity).abs() < 1e-8);
        (links, joint)
    }

    fn switch(&self, sim: &mut Simulation, tier: ArticulationTier) {
        let previous_time = sim.elapsed();
        set_model_fidelity(sim, self.model, tier).unwrap();
        assert_eq!(model_fidelity(sim, self.model).unwrap(), tier);
        assert_eq!(sim.elapsed(), previous_time);
    }

    fn expect_deferred(&self, sim: &mut Simulation) {
        let result = set_model_fidelity(sim, self.model, ArticulationTier::Prescribed);
        assert!(
            matches!(
                result,
                Err(FidelityError::Physics(
                    PhysicsError::TransitionDeferred { .. }
                ))
            ),
            "{result:?}"
        );
        assert_eq!(
            model_fidelity(sim, self.model).unwrap(),
            ArticulationTier::Dynamic
        );
    }

    fn kick(&self, sim: &mut Simulation, dv: DVec3) {
        let dt = sim.step_duration().as_secs_f64();
        for (&link, mass) in self.links.iter().zip(MASS) {
            let force = dv * (mass / dt);
            queue_link_world_wrench(sim, link, force.to_array(), [0.0; 3]).unwrap();
        }
    }
}

fn near(actual: DVec3, expected: DVec3, tolerance: f64) {
    assert!(
        (actual - expected).length() <= tolerance,
        "{actual:?}, expected {expected:?} ± {tolerance}"
    );
}

fn same_handoff(before: &([BodyState; 2], JointState), after: &([BodyState; 2], JointState)) {
    for (old, new) in before.0.iter().zip(&after.0) {
        near(
            DVec3::from_array(new.position),
            DVec3::from_array(old.position),
            1e-10,
        );
        near(
            DVec3::from_array(new.velocity),
            DVec3::from_array(old.velocity),
            1e-8,
        );
        near(
            DVec3::from_array(new.angular_velocity),
            DVec3::from_array(old.angular_velocity),
            1e-8,
        );
        let old = DQuat::from_array(old.orientation);
        let new = DQuat::from_array(new.orientation);
        assert!((old - new * old.dot(new).signum()).length() < 1e-10);
    }
    assert!((after.1.position - before.1.position).abs() < 1e-10);
    assert!((after.1.velocity - before.1.velocity).abs() < 1e-8);
}

struct Totals {
    center: DVec3,
    momentum: DVec3,
    angular_momentum: DVec3,
    kinetic_energy: f64,
}

fn totals(states: &[BodyState; 2]) -> Totals {
    let mut result = Totals {
        center: DVec3::ZERO,
        momentum: DVec3::ZERO,
        angular_momentum: DVec3::ZERO,
        kinetic_energy: 0.0,
    };
    for (i, state) in states.iter().enumerate() {
        let velocity = DVec3::from_array(state.velocity);
        let angular = DVec3::from_array(state.angular_velocity);
        let center =
            DVec3::from_array(state.position) + DQuat::from_array(state.orientation) * COM[i];
        let linear_momentum = MASS[i] * velocity;
        result.center += MASS[i] * center;
        result.momentum += linear_momentum;
        // Use one fixed world origin for both intrinsic spin and orbital momentum.
        result.angular_momentum += INERTIA[i] * angular + center.cross(linear_momentum);
        result.kinetic_energy +=
            0.5 * (MASS[i] * velocity.length_squared() + INERTIA[i] * angular.length_squared());
    }
    result.center /= MASS.iter().sum::<f64>();
    result
}

fn conserved(current: &Totals, initial: &Totals, tolerance: f64) {
    near(current.momentum, initial.momentum, tolerance);
    near(
        current.angular_momentum,
        initial.angular_momentum,
        tolerance,
    );
    assert!(
        (current.kinetic_energy - initial.kinetic_energy).abs() <= tolerance,
        "kinetic energy {} -> {}",
        initial.kinetic_energy,
        current.kinetic_energy
    );
}

#[test]
fn moving_bent_arm_preserves_current_joint_configuration_and_world_momentum() {
    let mut sim = load(EXAMPLE);
    let arm = Arm::find(&sim);
    let initial = arm.sample(&sim);
    arm.kick(&mut sim, DVec3::new(0.4, -0.25, 0.1));
    let servo = |velocity| {
        Some(JointVelocityMotor {
            target_velocity: velocity,
            max_force: 100.0,
        })
    };
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_servo(arm.joint, servo(1.25))
        .unwrap();
    sim.step_exact(250).unwrap();
    assert!(arm.sample(&sim).1.position > 0.2);
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_servo(arm.joint, servo(0.0))
        .unwrap();
    // Spin around the assembly's normal, which is a principal axis of this planar
    // geometry. Braking the internal hinge leaves a moving root and a bent arm.
    let normal = DQuat::from_array(initial.0[0].orientation) * DVec3::Z;
    queue_link_world_wrench(&mut sim, arm.links[0], [0.0; 3], (normal * 25.0).to_array()).unwrap();
    sim.step_exact(25).unwrap();
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_servo(arm.joint, None)
        .unwrap();
    let before = arm.sample(&sim);
    assert!(
        before.1.position > 0.2 && before.1.velocity.abs() < 1e-6,
        "{:?}",
        before.1
    );
    assert!(DVec3::from_array(before.0[0].angular_velocity).length() > 0.01);
    let reference = totals(&before.0);
    arm.switch(&mut sim, ArticulationTier::Prescribed);
    let demoted = arm.sample(&sim);
    same_handoff(&before, &demoted);
    conserved(&totals(&demoted.0), &reference, 1e-8);
    let start = sim.elapsed();
    sim.tick().unwrap();
    assert_eq!(sim.elapsed(), start);
    sim.step_exact(50).unwrap();
    let carried = arm.sample(&sim);
    assert!((carried.1.position - before.1.position).abs() < 1e-9);
    let tool_offset = |state: &[BodyState; 2]| {
        DQuat::from_array(state[0].orientation).inverse()
            * (DVec3::from_array(state[1].position) - DVec3::from_array(state[0].position))
    };
    near(tool_offset(&carried.0), tool_offset(&before.0), 1e-8);
    let origin = DVec3::from_array(carried.0[1].position) + normal;
    let hit = sim
        .world()
        .resource::<PhysicsBackend>()
        .cast_ray(
            origin.to_array(),
            (-normal).to_array(),
            2.0,
            Some(arm.bodies[0]),
        )
        .unwrap()
        .unwrap();
    assert!(
        (hit - 0.88).abs() < 1e-10,
        "ray against the 12 cm tool collider: {hit}"
    );
    let after = totals(&carried.0);
    let dt = (sim.elapsed() - start).as_secs_f64();
    let expected = reference.center + reference.momentum / MASS.iter().sum::<f64>() * dt;
    eprintln!(
        "50 ms prescribed coast: COM error={:e} m, ΔP={:e} kg m/s, ΔL={:e} kg m²/s, ΔK={:e} J",
        after.center.distance(expected),
        after.momentum.distance(reference.momentum),
        after.angular_momentum.distance(reference.angular_momentum),
        (after.kinetic_energy - reference.kinetic_energy).abs()
    );
    near(after.center, expected, 1e-7);
    conserved(&after, &reference, 1e-7);
    arm.switch(&mut sim, ArticulationTier::Dynamic);
    let resumed = arm.sample(&sim);
    same_handoff(&carried, &resumed);
    conserved(&totals(&resumed.0), &after, 1e-8);
    assert!((resumed.1.position - before.1.position).abs() < 1e-9);
    queue_joint_effort(&mut sim, arm.hinge, 15.0).unwrap();
    sim.step_exact(1).unwrap();
    assert!(arm.sample(&sim).1.velocity > 0.01);

    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(Arm::find(&sim), arm);
    assert_eq!(
        model_fidelity(&sim, arm.model).unwrap(),
        ArticulationTier::Dynamic
    );
    assert_eq!(arm.sample(&sim), initial);
}

#[test]
fn rejected_transitions_preserve_pending_joint_and_body_commands_exactly_once() {
    for joint_command in [true, false] {
        let mut trial = load(EXAMPLE);
        let mut control = load(EXAMPLE);
        let arm = Arm::find(&trial);
        for sim in [&mut trial, &mut control] {
            if joint_command {
                queue_joint_effort(sim, arm.hinge, 20.0).unwrap();
            } else {
                queue_link_world_wrench(sim, arm.links[1], [80.0, 0.0, 0.0], [0.0; 3]).unwrap();
            }
        }
        let before = arm.sample(&trial);
        arm.expect_deferred(&mut trial);
        assert_eq!(arm.sample(&trial), before);
        trial.tick().unwrap();
        for _ in 0..4 {
            trial.step_exact(1).unwrap();
            control.step_exact(1).unwrap();
            assert_eq!(arm.sample(&trial), arm.sample(&control));
        }
        let (states, joint) = arm.sample(&trial);
        if joint_command {
            assert!(joint.velocity.abs() > 0.01);
            arm.expect_deferred(&mut trial);
        } else {
            near(totals(&states).momentum, DVec3::new(0.08, 0.0, 0.0), 1e-6);
        }
    }
}

#[test]
fn internal_joint_couple_converges_to_zero_net_external_impulse() {
    let mut previous: Option<(f64, f64)> = None;
    for subdivisions in [1, 2, 4] {
        let mut sim = load_at(Duration::from_micros(1000 / subdivisions));
        let arm = Arm::find(&sim);
        // Same physical experiment: 20 Nm for 1 ms then coast for 3 ms.
        for _ in 0..subdivisions {
            queue_joint_effort(&mut sim, arm.hinge, 20.0).unwrap();
            sim.step_exact(1).unwrap();
        }
        sim.step_exact(3 * subdivisions).unwrap();
        let (states, joint) = arm.sample(&sim);
        assert!(joint.velocity > 0.01);
        let result = totals(&states);
        let linear = result.momentum.length();
        let angular = result.angular_momentum.length();
        eprintln!("subdivisions={subdivisions}: residual P={linear:e}, L={angular:e}");
        // Bound spurious external impulses to 0.1% of the applied angular
        // impulse, using the link-origin separation as the fixture length scale.
        let impulse = 20.0 * 0.001;
        let link_separation =
            DVec3::from_array(states[0].position).distance(DVec3::from_array(states[1].position));
        assert!(angular < 0.001 * impulse && linear * link_separation < 0.001 * impulse);
        if let Some((last_linear, last_angular)) = previous {
            assert!(linear < 0.65 * last_linear && angular < 0.65 * last_angular);
        }
        previous = Some((linear, angular));
    }
}

#[test]
fn nonprincipal_tumbling_is_deferred_without_losing_angular_momentum() {
    let mut sim = load(EXAMPLE);
    let arm = Arm::find(&sim);
    let brake = JointVelocityMotor {
        target_velocity: 0.0,
        max_force: 100.0,
    };
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_servo(arm.joint, Some(brake))
        .unwrap();
    queue_link_world_wrench(&mut sim, arm.links[0], [0.0; 3], [15.0, -6.0, 8.0]).unwrap();
    sim.step_exact(3).unwrap();
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_servo(arm.joint, None)
        .unwrap();
    let before = arm.sample(&sim);
    assert!(before.1.velocity.abs() < 1e-8);
    let state = totals(&before.0);
    let spin = state.angular_momentum - state.center.cross(state.momentum);
    let omega = DVec3::from_array(before.0[0].angular_velocity);
    assert!(spin.cross(omega).length() > 0.01 * spin.length() * omega.length());
    assert!(matches!(
        set_model_fidelity(&mut sim, arm.model, ArticulationTier::Prescribed),
        Err(FidelityError::Physics(PhysicsError::TransitionDeferred {
            reason: "torque-free motion requires a principal-axis spin"
        }))
    ));
    assert_eq!(arm.sample(&sim), before);
    assert_eq!(
        model_fidelity(&sim, arm.model).unwrap(),
        ArticulationTier::Dynamic
    );
}

#[test]
fn ten_thousand_round_trips_preserve_translating_assembly_energy_and_momentum() {
    let mut sim = load(EXAMPLE);
    let arm = Arm::find(&sim);
    let velocity = DVec3::new(0.13, -0.27, 0.05);
    arm.kick(&mut sim, velocity);
    sim.step_exact(1).unwrap();
    let before = totals(&arm.sample(&sim).0);
    near(before.momentum, MASS.iter().sum::<f64>() * velocity, 1e-9);
    let mut largest = DVec4::ZERO;
    for cycle in 1..=10_000 {
        arm.switch(&mut sim, ArticulationTier::Prescribed);
        sim.step_exact(1).unwrap();
        arm.switch(&mut sim, ArticulationTier::Dynamic);
        sim.step_exact(1).unwrap();
        let (states, joint) = arm.sample(&sim);
        let state = totals(&states);
        let expected_center = before.center + velocity * (f64::from(cycle) * 0.002);
        near(state.center, expected_center, 1e-7);
        conserved(&state, &before, 1e-7);
        largest = largest.max(DVec4::new(
            state.center.distance(expected_center),
            state.momentum.distance(before.momentum),
            state.angular_momentum.distance(before.angular_momentum),
            (state.kinetic_energy - before.kinetic_energy).abs(),
        ));
        assert!(joint.position.abs() < 1e-8 && joint.velocity.abs() < 1e-8);
    }
    eprintln!(
        "10,000 cycles: maximum COM error={:e} m, ΔP={:e} kg m/s, ΔL={:e} kg m²/s, ΔK={:e} J",
        largest.x, largest.y, largest.z, largest.w
    );
    assert_eq!(sim.elapsed(), Duration::from_millis(20_001));
    assert_eq!(Arm::find(&sim), arm);
}

#[test]
fn awaiting_promotion_retains_dynamic_wrench_without_accelerating_prescribed_links() {
    // Both partitions carry the assembly for 6 ms and request the same total
    // impulse. A held force acts over the next dynamic dt, once.
    for (dt, held_steps) in [
        (Duration::from_millis(2), 3),
        (Duration::from_micros(500), 12),
    ] {
        let mut sim = load_at(dt);
        let arm = Arm::find(&sim);
        arm.switch(&mut sim, ArticulationTier::Prescribed);
        let before = arm.sample(&sim);
        let impulse = DVec3::new(0.03, -0.05, 0.01);
        queue_link_world_wrench(
            &mut sim,
            arm.links[0],
            (impulse / dt.as_secs_f64()).to_array(),
            [0.0; 3],
        )
        .unwrap();
        sim.tick().unwrap();
        sim.step_exact(held_steps).unwrap();
        assert_eq!(sim.elapsed(), Duration::from_millis(6));
        let waiting = arm.sample(&sim);
        for state in &waiting.0 {
            assert_eq!(state.velocity, [0.0; 3]);
            assert_eq!(state.angular_velocity, [0.0; 3]);
        }
        assert_eq!(waiting.1, before.1);
        arm.switch(&mut sim, ArticulationTier::Dynamic);
        for _ in 0..3 {
            sim.step_exact(1).unwrap();
            near(totals(&arm.sample(&sim).0).momentum, impulse, 1e-6);
        }
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        arm.switch(&mut sim, ArticulationTier::Prescribed);
        queue_link_world_wrench(&mut sim, arm.links[0], [10.0, 0.0, 0.0], [0.0; 3]).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(
            model_fidelity(&sim, arm.model).unwrap(),
            ArticulationTier::Dynamic
        );
        sim.step_exact(2).unwrap();
        near(totals(&arm.sample(&sim).0).momentum, DVec3::ZERO, 1e-12);
    }
}

// Two broad flat feet leave the center of mass well inside the support polygon.
const FLOOR: &str = r#"<model name="floor"><static>true</static><pose>0 0 -0.1 0 0 0</pose>
  <link name="base"><collision name="shape"><geometry><box><size>8 8 0.2</size></box></geometry></collision></link>
</model>"#;
const PALLET: &str = r#"<model name="pallet"><pose>0 0 0.1 0 0 0</pose>
  <link name="deck"><inertial><mass>8</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>3</izz></inertia></inertial>
  <collision name="deck"><geometry><box><size>3 3 0.2</size></box></geometry></collision></link></model>"#;

#[test]
fn falling_and_dynamically_supported_arms_are_distinguished_from_static_support() {
    let arm_model = EXAMPLE
        .replace("<gravity>0 0 0</gravity>", "<gravity>0 0 -9.81</gravity>")
        .replace("1.5 -0.7 2.0 0.2 -0.3 0.4", "0 0 0.55 0 0 0")
        .replace(
            "<sphere><radius>0.16</radius></sphere>",
            "<box><size>0.4 0.4 0.32</size></box>",
        )
        .replace(
            "<sphere><radius>0.12</radius></sphere>",
            "<box><size>0.4 0.4 0.32</size></box>",
        );
    for support in ["", PALLET] {
        let mut sim = load(&arm_model.replace("</world>", &format!("{support}{FLOOR}</world>")));
        let arm = Arm::find(&sim);
        sim.step_exact(20).unwrap();
        let airborne = arm.sample(&sim);
        assert!(airborne.0.iter().all(|link| link.velocity[2] < -0.15));
        arm.expect_deferred(&mut sim);
        assert_eq!(airborne, arm.sample(&sim));
        sim.step_exact(2_000).unwrap();
        let settled = arm.sample(&sim);
        let expected_height = if support.is_empty() { 0.16 } else { 0.36 };
        for link in &settled.0 {
            assert!(
                (link.position[2] - expected_height).abs() < 0.01,
                "{link:?}"
            );
            assert!(
                DVec3::from_array(link.velocity).length() < 0.002,
                "{link:?}"
            );
        }
        if support.is_empty() {
            arm.switch(&mut sim, ArticulationTier::Prescribed);
            sim.step_exact(10).unwrap();
            for (old, new) in settled.0.into_iter().zip(arm.sample(&sim).0) {
                near(
                    DVec3::from_array(new.position),
                    DVec3::from_array(old.position),
                    2e-5,
                );
            }
            arm.switch(&mut sim, ArticulationTier::Dynamic);
            sim.step_exact(100).unwrap();
            let floor_body = sim
                .world()
                .iter_entities()
                .find(|e| {
                    e.get::<Name>()
                        .is_some_and(|name| name.0 == "default::floor::base")
                })
                .unwrap()
                .get::<BodyHandle>()
                .unwrap()
                .id();
            let floor_state = sim
                .world()
                .resource::<PhysicsBackend>()
                .state(floor_body)
                .unwrap();
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .set_body_state(
                    floor_body,
                    BodyState {
                        position: [20.0, 0.0, -0.1],
                        ..floor_state
                    },
                )
                .unwrap();
            arm.expect_deferred(&mut sim);
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .set_body_state(floor_body, floor_state)
                .unwrap();
            sim.step_exact(100).unwrap();
            let floor = sim
                .world()
                .iter_entities()
                .find(|e| {
                    e.get::<Name>()
                        .is_some_and(|name| name.0 == "default::floor")
                })
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get();
            remove_model(&mut sim, floor).unwrap();
            let unsupported = arm.sample(&sim);
            arm.expect_deferred(&mut sim);
            assert_eq!(arm.sample(&sim), unsupported);
            sim.step_exact(10).unwrap();
            assert!(
                arm.sample(&sim)
                    .0
                    .iter()
                    .all(|link| link.velocity[2] < -0.05)
            );
        } else {
            arm.expect_deferred(&mut sim);
            assert_eq!(
                model_fidelity(&sim, arm.model).unwrap(),
                ArticulationTier::Dynamic
            );
            assert_eq!(settled, arm.sample(&sim));
        }
    }
}
