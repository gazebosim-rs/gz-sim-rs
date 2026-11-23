// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Generalized effort is an internal couple, not a child-only external torque.
use glam::DVec3;
use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, JointId, JointPositionServo,
    JointVelocityMotor, PhysicsBackend, PhysicsError, RotationSpec, Shape,
};
use gz_physics_rapier::RapierBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, JointSample, JointStateSample, Name, PhysicsEngine, load_world_with_sdf_step,
    queue_joint_effort, remove_model, reset_loaded_world, set_joint_force,
};
use std::time::Duration;
fn near(a: f64, b: f64) {
    assert!(
        a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
        "{a} != {b}"
    );
}
fn vector(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        near(a, b);
    }
}
fn body(b: &mut RapierBackend, mass: Option<f64>, omega: [f64; 3]) -> BodyId {
    let id = b
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [0.; 3],
                velocity: [0.; 3],
                mass,
            },
            RotationSpec {
                angular_velocity: omega,
                ..Default::default()
            },
            Some(Inertia {
                center: [0.; 3],
                orientation: [0., 0., 0., 1.],
                tensor: [2., 2., 2., 0., 0., 0.],
            }),
        )
        .unwrap();
    b.set_self_collision_group(id, Some(7)).unwrap();
    id
}
fn hinge(b: &mut RapierBackend, reduced: bool, parent: Option<BodyId>, child: BodyId) -> JointId {
    if reduced {
        b.add_reduced_revolute(parent, child, [0.; 3], [0., 1., 0.])
            .unwrap()
    } else if let Some(parent) = parent {
        b.add_revolute(parent, child, [0.; 3], [0., 1., 0.])
            .unwrap()
    } else {
        b.add_world_revolute(child, [0.; 3], [0., 1., 0.]).unwrap()
    }
}
fn tick(b: &mut RapierBackend) {
    b.step(Duration::from_millis(1)).unwrap();
}
#[test]
fn moving_parent_internal_couple_has_strict_principal_response_and_zero_net_momentum() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.; 3]).unwrap();
        let p = body(&mut b, Some(2.), [0.; 3]);
        let c = body(&mut b, Some(2.), [0.; 3]);
        let j = hinge(&mut b, reduced, Some(p), c);
        b.add_joint_effort(j, 8.).unwrap();
        b.add_joint_effort(j, -2.).unwrap();
        let before = (b.state(p), b.state(c), b.joint_state(j));
        b.step(Duration::ZERO).unwrap();
        assert_eq!((b.state(p), b.state(c), b.joint_state(j)), before);
        tick(&mut b);
        vector(b.state(p).unwrap().angular_velocity, [0., -0.003, 0.]);
        vector(b.state(c).unwrap().angular_velocity, [0., 0.003, 0.]);
        near(b.joint_state(j).unwrap().velocity, 0.006);
        vector(b.state(p).unwrap().velocity, [0.; 3]);
        vector(b.state(c).unwrap().velocity, [0.; 3]);
        tick(&mut b);
        near(b.joint_state(j).unwrap().velocity, 0.006);
        b.add_joint_effort(j, -12.).unwrap();
        tick(&mut b);
        near(b.joint_state(j).unwrap().velocity, -0.006);
        vector(
            (DVec3::from_array(b.state(p).unwrap().angular_velocity)
                + DVec3::from_array(b.state(c).unwrap().angular_velocity))
            .to_array(),
            [0.; 3],
        );
    }
}
#[test]
fn moving_parent_axis_transport_and_positive_relative_work_use_measured_states() {
    for reduced in [false, true] {
        let mut driven = RapierBackend::new([0.; 3]).unwrap();
        let p = body(&mut driven, Some(2.), [0.2, 0., 0.]);
        let c = body(&mut driven, Some(2.), [0.2, 0., 0.]);
        let j = hinge(&mut driven, reduced, Some(p), c);
        let mut quiet = RapierBackend::new([0.; 3]).unwrap();
        let qp = body(&mut quiet, Some(2.), [0.2, 0., 0.]);
        let qc = body(&mut quiet, Some(2.), [0.2, 0., 0.]);
        hinge(&mut quiet, reduced, Some(qp), qc);
        for _ in 0..20 {
            tick(&mut driven);
            tick(&mut quiet);
        }
        let parent = driven.state(p).unwrap();
        let axis = glam::DQuat::from_array(parent.orientation) * DVec3::Y;
        assert!(axis.z.abs() > 1e-4);
        driven.add_joint_effort(j, 6.).unwrap();
        tick(&mut driven);
        tick(&mut quiet);
        let pstate = driven.state(p).unwrap();
        let cstate = driven.state(c).unwrap();
        let relative =
            DVec3::from_array(cstate.angular_velocity) - DVec3::from_array(pstate.angular_velocity);
        let quiet_relative = DVec3::from_array(quiet.state(qc).unwrap().angular_velocity)
            - DVec3::from_array(quiet.state(qp).unwrap().angular_velocity);
        assert!((relative - quiet_relative).dot(axis) > 1e-4);
        // Strict principal-axis momentum is checked separately from rest.
        // This rotating-axis witness uses measured causal work, not an exact
        // discrete conservation law for a nonprincipal articulated step.
        assert!(
            pstate
                .angular_velocity
                .iter()
                .chain(cstate.angular_velocity.iter())
                .all(|v| v.is_finite())
        );
        vector(pstate.velocity, [0.; 3]);
        vector(cstate.velocity, [0.; 3]);
    }
}
#[test]
fn shared_parent_accumulation_is_balanced_across_two_joint_owners() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.; 3]).unwrap();
        let p = body(&mut b, Some(2.), [0.; 3]);
        let a = body(&mut b, Some(2.), [0.; 3]);
        let c = body(&mut b, Some(2.), [0.; 3]);
        let ja = hinge(&mut b, reduced, Some(p), a);
        let jc = hinge(&mut b, reduced, Some(p), c);
        b.add_joint_effort(ja, 6.).unwrap();
        b.add_joint_effort(jc, -2.).unwrap();
        tick(&mut b);
        near(b.state(p).unwrap().angular_velocity[1], -0.002);
        near(b.state(a).unwrap().angular_velocity[1], 0.003);
        near(b.state(c).unwrap().angular_velocity[1], -0.001);
        near(
            [p, a, c]
                .into_iter()
                .map(|id| b.state(id).unwrap().angular_velocity[1] * 2.)
                .sum(),
            0.,
        );
    }
}
#[test]
fn moving_effort_numeric_and_parent_accumulator_failures_preserve_pending_and_body_states() {
    for reduced in [false, true] {
        let mut b = RapierBackend::new([0.; 3]).unwrap();
        let p = body(&mut b, Some(2.), [0.; 3]);
        let c = body(&mut b, Some(2.), [0.; 3]);
        let j = hinge(&mut b, reduced, Some(p), c);
        b.add_joint_effort(j, 6.).unwrap();
        let before = (b.state(p), b.state(c), b.joint_state(j));
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                b.add_joint_effort(j, value),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!((b.state(p), b.state(c), b.joint_state(j)), before);
        }
        assert_eq!(
            b.add_joint_effort(JointId::from_raw(u64::MAX), 1.),
            Err(PhysicsError::MissingJoint)
        );
        tick(&mut b);
        near(b.joint_state(j).unwrap().velocity, 0.006);
        b.add_joint_effort(j, -1e308).unwrap();
        b.add_world_wrench(p, [0.; 3], [0., 1e308, 0.]).unwrap();
        let before = (b.state(p), b.state(c), b.joint_state(j));
        assert_eq!(
            b.step(Duration::from_millis(1)),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!((b.state(p), b.state(c), b.joint_state(j)), before);
        b.add_joint_effort(j, 1e308).unwrap();
        b.add_world_wrench(p, [0.; 3], [0., -1e308, 0.]).unwrap();
        tick(&mut b);
        near(b.joint_state(j).unwrap().velocity, 0.006);
    }
}
#[test]
fn world_static_parents_reset_retirement_and_motor_domains_are_unchanged() {
    for reduced in [false, true] {
        for static_parent in [false, true] {
            let mut b = RapierBackend::new([0.; 3]).unwrap();
            let parent = static_parent.then(|| body(&mut b, None, [0.; 3]));
            let c = body(&mut b, Some(2.), [0.; 3]);
            let j = hinge(&mut b, reduced, parent, c);
            let initial = b.initial_state();
            b.add_joint_effort(j, 6.).unwrap();
            tick(&mut b);
            near(b.state(c).unwrap().angular_velocity[1], 0.003);
            b.reset().unwrap();
            tick(&mut b);
            near(b.joint_state(j).unwrap().velocity, 0.);
            let motor = JointVelocityMotor {
                target_velocity: 0.5,
                max_force: 2.,
            };
            b.validate_joint_velocity_motor(j, Some(motor)).unwrap();
            b.validate_joint_velocity_servo(j, Some(motor)).unwrap();
            if !reduced {
                assert!(matches!(
                    b.validate_joint_position_servo(
                        j,
                        Some(JointPositionServo {
                            target_position: 0.2,
                            max_velocity: 1.,
                            max_force: 2.
                        }),
                        Duration::from_millis(1)
                    ),
                    Err(PhysicsError::Unsupported {
                        capability: "continuous position servos on impulse joints",
                        ..
                    })
                ));
            }
            b.add_joint_effort(j, 6.).unwrap();
            let prepared = b.prepare_initial(&initial).unwrap();
            b.apply_initial(prepared).unwrap();
            tick(&mut b);
            near(b.joint_state(j).unwrap().velocity, 0.);
            assert!(b.remove(c));
            assert_eq!(b.add_joint_effort(j, 1.), Err(PhysicsError::MissingJoint));
        }
    }
}
fn scene() -> Simulation {
    let link = |name: &str| {
        format!(
            "<link name='{name}'><inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial><collision name='s'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision><visual name='v'><geometry><sphere><radius>0.2</radius></sphere></geometry></visual></link>"
        )
    };
    let text = format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='pair'><pose>0 0 10 0 0 0</pose>{}{}<joint name='j' type='revolute'><parent>parent</parent><child>child</child><axis><xyz>0 1 0</xyz></axis></joint><plugin filename='gz-sim-apply-joint-force-system' name='gz::sim::systems::ApplyJointForce'><joint_name>j</joint_name></plugin><plugin filename='gz-sim-joint-state-publisher-system' name='gz::sim::systems::JointStatePublisher'><topic>/pair/state</topic><update_rate>1000</update_rate></plugin></model></world></sdf>",
        link("parent"),
        link("child")
    );
    let mut sim = load_world_with_sdf_step(
        &text,
        "moving-effort",
        "w",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    sim.initialize_schedules().unwrap();
    sim
}
fn named(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world().get::<EntityId>(named(sim, name)).unwrap().get()
}
fn native(sim: &Simulation, name: &str) -> BodyState {
    let b = sim
        .world()
        .get::<BodyHandle>(named(sim, name))
        .unwrap()
        .id();
    sim.world().resource::<PhysicsBackend>().state(b).unwrap()
}
#[test]
fn scene_persistent_and_queued_moving_parent_effort_matches_committed_feedback_and_resets() {
    let mut sim = scene();
    let j = id(&sim, "w::pair::j");
    let model = id(&sim, "w::pair");
    set_joint_force(&mut sim, j, 6.).unwrap();
    queue_joint_effort(&mut sim, j, 1.).unwrap();
    let before = (
        native(&sim, "w::pair::parent"),
        native(&sim, "w::pair::child"),
    );
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(
        (
            native(&sim, "w::pair::parent"),
            native(&sim, "w::pair::child")
        ),
        before
    );
    assert!(set_joint_force(&mut sim, j, f64::NAN).is_err());
    sim.set_paused(false);
    sim.step_exact(1).unwrap();
    near(native(&sim, "w::pair::parent").angular_velocity[1], -0.0035);
    near(native(&sim, "w::pair::child").angular_velocity[1], 0.0035);
    let sample = sim
        .world()
        .get::<JointSample>(named(&sim, "w::pair::j"))
        .unwrap();
    near(sample.state.velocity, 0.007);
    let feedback = sim
        .world()
        .get::<JointStateSample>(named(&sim, "w::pair"))
        .unwrap();
    assert_eq!(feedback.time, sim.elapsed());
    near(feedback.joints[0].velocity, sample.state.velocity);
    set_joint_force(&mut sim, j, 0.).unwrap();
    sim.step_exact(1).unwrap();
    near(native(&sim, "w::pair::child").angular_velocity[1], 0.0035);
    sim.reset().unwrap();
    sim.step_exact(1).unwrap();
    near(native(&sim, "w::pair::child").angular_velocity[1], 0.);
    remove_model(&mut sim, model).unwrap();
    assert!(queue_joint_effort(&mut sim, j, 1.).is_err());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(id(&sim, "w::pair"), model);
    set_joint_force(&mut sim, j, 6.).unwrap();
    sim.step_exact(1).unwrap();
    near(native(&sim, "w::pair::child").angular_velocity[1], 0.003);
}
