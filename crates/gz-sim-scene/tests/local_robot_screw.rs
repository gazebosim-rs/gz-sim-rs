// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![allow(deprecated)]
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    JointSample, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    queue_joint_effort, queue_link_world_wrench, reset_loaded_world, set_joint_force,
};
use sdformat::NoResources;
use std::f64::consts::TAU;

const SDF: &str = include_str!("../../../examples/worlds/local_robot_screw.sdf");
const LEAD: f64 = 0.4 / TAU;

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "screw.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn id(sim: &Simulation, local: &str) -> u64 {
    let name = format!("default::screw_drive::{local}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

fn sample(sim: &Simulation, joint: u64, nut: u64) -> (f64, f64, f64, f64) {
    let entity = entity_by_id(sim.world(), nut).unwrap();
    let x = sim.world().get::<WorldPose>(entity).unwrap().translation[2] - 1.;
    let dx = sim.world().get::<Velocity>(entity).unwrap().linear[2];
    let state = sim
        .world()
        .get::<JointSample>(entity_by_id(sim.world(), joint).unwrap())
        .unwrap()
        .state;
    (state.position, state.velocity, x, dx)
}

#[test]
fn axial_load_and_joint_torque_match_reflected_inertia_for_either_hand() {
    let free = SDF.replace("<gravity>0 0 -9.81", "<gravity>0 0 0");
    for (pitch, tag) in [
        (
            0.4,
            "<screw_thread_pitch>0.4</screw_thread_pitch>".to_string(),
        ),
        (
            -0.4,
            "<screw_thread_pitch>-0.4</screw_thread_pitch>".to_string(),
        ),
        (0.4, format!("<thread_pitch>{}</thread_pitch>", -TAU / 0.4)),
    ] {
        let lead = pitch / TAU;
        for (torque, force) in [(0.4, 0.), (0., 6.)] {
            let mut sim = load(&free.replace("<screw_thread_pitch>0.4</screw_thread_pitch>", &tag));
            let joint = id(&sim, "spindle");
            let nut = id(&sim, "nut");
            set_joint_force(&mut sim, joint, torque).unwrap();
            let acceleration = (torque + lead * force) / (0.020 + lead * lead);
            let mut peak = 0_f64;
            for _ in 0..200 {
                queue_link_world_wrench(&mut sim, nut, [0., 0., force], [0.; 3]).unwrap();
                sim.step_exact(1).unwrap();
                let (q, _, x, _) = sample(&sim, joint, nut);
                peak = peak.max((x - lead * q).abs());
            }
            let (q, omega, _, v) = sample(&sim, joint, nut);
            eprintln!(
                "pitch={pitch}, torque={torque}, force={force}, omega={omega:.8}, exact={:.8}, peak lead error={peak:.9} m",
                acceleration * 0.2
            );
            assert!((omega - acceleration * 0.2).abs() < 1e-5);
            assert!((q - acceleration * 0.02).abs() < 0.003);
            assert!((v - lead * omega).abs() < 0.0015 && peak < 0.0001);
        }
    }
}

#[test]
fn stop_after_one_full_turn_and_reverse_then_reset() {
    let mut sim = load(SDF);
    let joint = id(&sim, "spindle");
    let nut = id(&sim, "nut");
    let mut results = Vec::new();
    for _ in 0..2 {
        let mut peak = 0_f64;
        for tick in 0..1300 {
            sim.step_exact(1).unwrap();
            let (q, _, x, _) = sample(&sim, joint, nut);
            peak = peak.max((x - LEAD * q).abs());
            if tick >= 900 {
                assert!(
                    (q + 9.).abs() < 0.0001,
                    "hard stop rebounded at {tick} ms: {q}"
                );
            }
        }
        let lower = sample(&sim, joint, nut).0;
        set_joint_force(&mut sim, joint, 2.).unwrap();
        sim.step_exact(1200).unwrap();
        let upper = sample(&sim, joint, nut).0;
        eprintln!("lower={lower:.8}, upper={upper:.8} rad, peak gravity lead error={peak:.9} m");
        assert!((lower + 9.).abs() < 1e-5 && (upper - 2.).abs() < 1e-5 && peak < 0.000002);
        results.push((peak, lower, upper));
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(sample(&sim, joint, nut), (0., 0., 0., 0.));
    }
    assert_eq!(results[0], results[1]);
}

#[test]
fn rejects_invalid_pitch_and_enforces_authored_actuator_torque() {
    for invalid in [
        "<screw_thread_pitch>1e300</screw_thread_pitch>",
        "<thread_pitch>0</thread_pitch>",
    ] {
        let xml = SDF.replace("<screw_thread_pitch>0.4</screw_thread_pitch>", invalid);
        assert!(
            load_world_with_sdf_step(
                &xml,
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
    let mut sim = load(&SDF.replace("<limit>", "<limit><effort>1</effort>"));
    let joint = id(&sim, "spindle");
    assert!(set_joint_force(&mut sim, joint, 2.).is_err());
    queue_joint_effort(&mut sim, joint, 0.75).unwrap();
    assert!(queue_joint_effort(&mut sim, joint, 0.5).is_err());
    sim.step_exact(1).unwrap();
}

#[test]
fn floating_thread_returns_equal_and_opposite_reactions_to_the_driver() {
    let driver = r#"<link name="driver"><pose>0 0 1.3 0 0 0</pose>
        <inertial><mass>1</mass><inertia><ixx>0.014</ixx><iyy>0.014</iyy><izz>0.020</izz></inertia></inertial>
        <collision name="driver"><geometry><box><size>0.32 0.16 0.08</size></box></geometry></collision></link>"#;
    let xml = SDF
        .replace("<gravity>0 0 -9.81", "<gravity>0 0 0")
        .replace("<parent>world", "<parent>driver")
        .replace(
            "<link name=\"nut\">",
            &format!("{driver}<link name=\"nut\">"),
        );
    let mut sim = load(&xml);
    let joint = id(&sim, "spindle");
    let driver = id(&sim, "driver");
    let nut = id(&sim, "nut");
    set_joint_force(&mut sim, joint, 0.2).unwrap();
    sim.step_exact(200).unwrap();
    let velocity = |link| {
        sim.world()
            .get::<Velocity>(entity_by_id(sim.world(), link).unwrap())
            .unwrap()
    };
    let (_, speed, _, _) = sample(&sim, joint, nut);
    let linear_momentum =
        glam::DVec3::from(velocity(nut).linear) + glam::DVec3::from(velocity(driver).linear);
    let net_axial_spin = velocity(nut).angular[2] + velocity(driver).angular[2];
    let relative_speed = velocity(nut).linear[2] - velocity(driver).linear[2];
    eprintln!(
        "floating screw omega={speed}, net linear momentum={} kg m/s, net axial spin={net_axial_spin} rad/s",
        linear_momentum.length()
    );
    let exact = 0.2 * 0.2 / (0.010 + 0.5 * LEAD.powi(2));
    assert!((speed - exact).abs() < 0.002 && (relative_speed - LEAD * speed).abs() < 0.0002);
    assert!(linear_momentum.length() < 1e-9 && net_axial_spin.abs() < 1e-4);
}

#[test]
fn external_contact_stalls_the_helix_before_the_joint_stop() {
    for sign in [1., -1.] {
        let xml = SDF
            .replace("0 0 0.16 0", "0 0 0.68 0")
            .replace("<upper>2", "<upper>9")
            .replace(
                "<screw_thread_pitch>0.4",
                &format!("<screw_thread_pitch>{}", sign * 0.4),
            );
        let mut sim = load(&xml);
        let joint = id(&sim, "spindle");
        let nut = id(&sim, "nut");
        set_joint_force(&mut sim, joint, -0.2 * sign).unwrap();
        sim.step_exact(2000).unwrap();
        let (q, omega, x, v) = sample(&sim, joint, nut);
        eprintln!("shelf q={q} omega={omega}, travel={x} m, v={v}");
        // Top of the shelf is at 0.72 m and underside of the nut 0.04 m below its center.
        assert!((x + 0.24).abs() < 0.002 && (x - sign * LEAD * q).abs() < 0.0001);
        assert!(omega.abs() < 0.01 && v.abs() < 0.001);
        set_joint_force(&mut sim, joint, 2. * sign).unwrap();
        sim.step_exact(350).unwrap();
        assert!(sample(&sim, joint, nut).2 > x + 0.08);
    }
}

#[test]
fn zero_pitch_allows_pure_rotation_and_respects_a_one_sided_stop() {
    let xml = SDF
        .replace("<screw_thread_pitch>0.4", "<screw_thread_pitch>0")
        .replace("<lower>-9</lower>", "");
    let mut sim = load(&xml);
    let joint = id(&sim, "spindle");
    let nut = id(&sim, "nut");
    set_joint_force(&mut sim, joint, 1.).unwrap();
    sim.step_exact(600).unwrap();
    let (q, _, x, _) = sample(&sim, joint, nut);
    assert!((q - 2.).abs() < 0.0001 && x.abs() < 1e-5);
    set_joint_force(&mut sim, joint, -1.).unwrap();
    sim.step_exact(850).unwrap();
    let (q, omega, x, _) = sample(&sim, joint, nut);
    assert!(q < -9. && omega < -20. && x.abs() < 1e-5);
}

#[test]
fn capped_velocity_drive_rotates_and_advances_the_nut() {
    use gz_physics::{JointVelocityMotor, PhysicsBackend};
    use gz_sim_scene::JointHandle;
    let mut sim = load(
        &SDF.replace("<limit>", "<limit><effort>1</effort>")
            .replace("<gravity>0 0 -9.81", "<gravity>0 0 0"),
    );
    let spindle = id(&sim, "spindle");
    let nut = id(&sim, "nut");
    let joint = sim
        .world()
        .get::<JointHandle>(entity_by_id(sim.world(), spindle).unwrap())
        .unwrap()
        .id();
    let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
    let target = |max_force| {
        Some(JointVelocityMotor {
            target_velocity: -1.5,
            max_force,
        })
    };
    assert!(
        backend
            .set_joint_velocity_servo(joint, target(1.1))
            .is_err()
    );
    backend.set_joint_velocity_servo(joint, target(1.)).unwrap();
    sim.step_exact(300).unwrap();
    let (q, omega, x, v) = sample(&sim, spindle, nut);
    assert!((omega + 1.5).abs() < 0.005 && (v - LEAD * omega).abs() < 1e-4);
    assert!(q < -0.3 && (x - LEAD * q).abs() < 1e-5);
}
