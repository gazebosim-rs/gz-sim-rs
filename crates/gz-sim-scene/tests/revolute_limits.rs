// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{JointId, JointPositionServo, JointState, JointVelocityMotor, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, JointHandle, JointSample, Name, PhysicsEngine, load_world_with_sdf_step,
    reset_loaded_world, set_joint_force, set_model_fidelity,
};
use sdformat::NoResources;

const FLOATING: &str = include_str!("../../../examples/multifidelity_articulation.sdf");
const PINNED: &str = r#"<sdf version="1.12"><world name="default"><gravity>0 0 0</gravity>
  <physics name="p" type="ignored"><max_step_size>0.001</max_step_size></physics>
  <model name="robot"><link name="tool"><inertial><mass>1</mass><inertia>
  <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
  <collision name="shape"><geometry><sphere><radius>0.2</radius></sphere></geometry></collision>
  </link><joint name="hinge" type="revolute"><parent>world</parent><child>tool</child>
  <axis><xyz>0 0 1</xyz></axis></joint></model></world></sdf>"#;

fn with_limit(source: &str, limit: &str) -> String {
    source.replacen("</axis>", &format!("<limit>{limit}</limit></axis>"), 1)
}

fn load(source: &str) -> Simulation {
    load_world_with_sdf_step(
        source,
        "stops.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

fn read(sim: &Simulation, hinge: u64) -> (JointId, JointState) {
    let e = entity_by_id(sim.world(), hinge).unwrap();
    let id = sim.world().get::<JointHandle>(e).unwrap().id();
    let sample = sim.world().get::<JointSample>(e).unwrap();
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .joint_state(id)
        .unwrap();
    assert_eq!(sample.time, sim.elapsed());
    assert_eq!(sample.state, native);
    (id, native)
}

#[test]
fn sustained_effort_on_loaded_world_stops_and_reverses_after_reset_and_switching() {
    let mut sim = load(&with_limit(
        FLOATING,
        "<lower>-0.25</lower><upper>0.5</upper>",
    ));
    let hinge = id(&sim, "default::robot::hinge");
    let model = id(&sim, "default::robot");
    let joint = read(&sim, hinge).0;
    for _ in 0..2 {
        set_model_fidelity(&mut sim, model, ArticulationTier::Prescribed).unwrap();
        sim.step_exact(20).unwrap();
        assert_eq!(read(&sim, hinge).1.position, 0.0);
        set_model_fidelity(&mut sim, model, ArticulationTier::Dynamic).unwrap();
        for (force, stop) in [(1.0, 0.5), (-1.0, -0.25)] {
            set_joint_force(&mut sim, hinge, force).unwrap();
            sim.step_exact(900).unwrap();
            let (actual_id, state) = read(&sim, hinge);
            assert_eq!(actual_id, joint);
            assert!((state.position - stop).abs() < 1e-3, "{state:?}");
            assert!(state.velocity.abs() < 0.005, "{state:?}");
        }
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(
            read(&sim, hinge),
            (
                joint,
                JointState {
                    position: 0.0,
                    velocity: 0.0
                }
            )
        );
    }
}

#[test]
fn unwrapped_world_anchored_limits_constrain_velocity_and_position_motors() {
    let mut sim = load(&with_limit(PINNED, "<lower>-0.2</lower><upper>7</upper>"));
    let hinge = id(&sim, "default::robot::hinge");
    let joint = read(&sim, hinge).0;
    let motor = JointVelocityMotor {
        target_velocity: 5.0,
        max_force: 2.0,
    };
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_motor(joint, Some(motor))
        .unwrap();
    sim.step_exact(3200).unwrap();
    let upper = read(&sim, hinge).1;
    assert!((upper.position - 7.0).abs() < 0.002, "{upper:?}");
    let dt = sim.step_duration();
    let servo = JointPositionServo {
        target_position: -1.0,
        max_velocity: 6.0,
        max_force: 2.0,
    };
    for _ in 0..2500 {
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_joint_position_servo(joint, Some(servo), dt)
            .unwrap();
        sim.step_exact(1).unwrap();
    }
    let lower = read(&sim, hinge).1;
    assert!((lower.position + 0.2).abs() < 0.002, "{lower:?}");
}

#[test]
fn admission_rejects_one_sided_nonfinite_and_initially_violated_intervals() {
    for bounds in [
        "<lower>-1</lower>",
        "<upper>1</upper>",
        "<lower>0.1</lower><upper>1</upper>",
        "<lower>-2</lower><upper>-0.1</upper>",
        "<lower>1</lower><upper>-1</upper>",
        "<lower>NaN</lower><upper>1</upper>",
    ] {
        let src = with_limit(PINNED, bounds);
        assert!(
            load_world_with_sdf_step(
                &src,
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err(),
            "{bounds}"
        );
    }
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let src = with_limit(PINNED, "<lower>-1</lower><upper>1</upper>");
        assert!(
            load_world_with_sdf_step(&src, "legacy.sdf", "default", &NoResources, engine).is_err()
        );
    }
    let mut locked = load(&with_limit(PINNED, "<lower>0</lower><upper>0</upper>"));
    let hinge = id(&locked, "default::robot::hinge");
    set_joint_force(&mut locked, hinge, 1.0).unwrap();
    locked.step_exact(100).unwrap();
    assert!(read(&locked, hinge).1.position.abs() < 1e-3);
}

#[test]
fn peak_stop_penetration_is_bounded_by_one_incoming_free_step() {
    // 1 N m on 0.1 kg m², q=0.5 rad: impact speed sqrt(2*a*q)=sqrt(10).
    // The inequality allows one unconstrained public integration interval;
    // Rapier solves the unilateral stop on its internal substeps.
    for milliseconds in [1_u64, 2, 4] {
        let dt = milliseconds as f64 * 1e-3;
        let source = with_limit(PINNED, "<lower>-0.25</lower><upper>0.5</upper>")
            .replace("0.001</max_step_size>", &format!("{dt}</max_step_size>"));
        let mut sim = load(&source);
        let joint = id(&sim, "default::robot::hinge");
        set_joint_force(&mut sim, joint, 1.0).unwrap();
        let mut peak: f64 = 0.0;
        for _ in 0..(700 / milliseconds) {
            sim.step_exact(1).unwrap();
            peak = peak.max(read(&sim, joint).1.position - 0.5);
        }
        println!("dt={dt} s, peak unilateral stop penetration={peak:e} rad");
        assert!(peak <= 10_f64.sqrt() * dt + 5.0 * dt * dt, "{peak:e}");
        assert!(read(&sim, joint).1.position >= 0.499);
    }
}
