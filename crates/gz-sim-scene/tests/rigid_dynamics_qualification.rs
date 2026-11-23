// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Analytic checks through the loaded-world SDF, solver step and Bevy pose commit.
use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec3};
use gz_sim_core::Simulation;
use gz_sim_scene::{
    JointSample, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
};
use sdformat::NoResources;
use std::time::Duration;

const GRAVITY: f64 = 9.81;

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "rigid_qualification.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn world(models: &str, dt_ms: u64) -> Simulation {
    let dt = Duration::from_millis(dt_ms);
    let source = format!(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -{GRAVITY}</gravity>
        <physics name="test" type="ignored"><max_step_size>{}</max_step_size></physics>
        {models}</world></sdf>"#,
        dt.as_secs_f64()
    );
    let sim = load(&source);
    assert_eq!(sim.step_duration(), dt);
    sim
}

fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}

fn pose(sim: &Simulation, entity: Entity) -> (DVec3, DQuat) {
    let pose = sim.world().get::<WorldPose>(entity).unwrap();
    (
        DVec3::from_array(pose.translation),
        DQuat::from_array(pose.rotation),
    )
}

fn velocity(sim: &Simulation, entity: Entity) -> (DVec3, DVec3) {
    let twist = sim.world().get::<Velocity>(entity).unwrap();
    (
        DVec3::from_array(twist.linear),
        DVec3::from_array(twist.angular),
    )
}

// Exact finite-amplitude physical-pendulum period T = 4 sqrt(I_p/(m g L)) K(k).
// K(sin(a/2)) = pi / (2 AGM(1,cos(a/2))). The pivot lies one metre from the bob COM.
fn period(angle: f64) -> f64 {
    let mut arithmetic = 1.0;
    let mut geometric = (angle * 0.5).cos();
    for _ in 0..8 {
        (arithmetic, geometric) = (
            (arithmetic + geometric) * 0.5,
            (arithmetic * geometric).sqrt(),
        );
    }
    std::f64::consts::TAU * (1.004 / GRAVITY).sqrt() / arithmetic
}

struct Pendulum {
    period_error: f64,
    relative_energy_error: f64,
    anchor_error: f64,
}

fn pendulum(dt_ms: u64) -> Pendulum {
    let dt = Duration::from_millis(dt_ms);
    let release = 0.6_f64;
    let source = include_str!("fixtures/pendulum.sdf")
        .replace("0 0.02 0", "0 0.6 0")
        .replace(
            "<max_step_size>0.001</max_step_size>",
            &format!("<max_step_size>{}</max_step_size>", dt.as_secs_f64()),
        );
    let mut sim = load(&source);
    let body = named(&sim, "default::robot::body");
    let hinge = named(&sim, "default::robot::hinge");
    let initial_energy = GRAVITY * (1.0 - release.cos());
    let mut relative_energy_error = 0.0_f64;
    let mut anchor_error = 0.0_f64;
    let mut crossings = Vec::new();
    let mut previous_angle = release;
    for _ in 0..(5_000 / dt_ms) {
        sim.step_exact(1).unwrap();
        let (position, rotation) = pose(&sim, body);
        let (linear, angular) = velocity(&sim, body);
        let measured = (-position.x).atan2(-position.z);
        let sample = sim.world().get::<JointSample>(hinge).unwrap();
        assert_eq!(sample.time, sim.elapsed());
        assert!((measured - (release + sample.state.position)).abs() < 1e-4);
        let pivot = position + rotation * DVec3::Z;
        anchor_error = anchor_error.max(pivot.length());
        let energy = GRAVITY * (position.z + 1.0)
            + 0.5 * (linear.length_squared() + 0.004 * angular.length_squared());
        relative_energy_error = relative_energy_error.max((energy / initial_energy - 1.0).abs());
        if previous_angle > 0.0 && measured <= 0.0 {
            // Interpolate the downward zero crossing within the committed timestep.
            crossings.push(
                (sim.elapsed() - dt).as_secs_f64()
                    + dt.as_secs_f64() * previous_angle / (previous_angle - measured),
            );
        }
        previous_angle = measured;
    }
    assert_eq!(crossings.len(), 3);
    let measured = (crossings[2] - crossings[0]) / 2.0;
    let period_error = (measured - period(release)).abs();
    eprintln!(
        "pendulum dt={dt_ms} ms: period={measured:.9} s (analytic {:.9}), max relative energy error={relative_energy_error:e}, anchor error={anchor_error:e} m",
        period(release)
    );
    Pendulum {
        period_error,
        relative_energy_error,
        anchor_error,
    }
}

#[test]
fn finite_amplitude_pendulum_period_energy_and_anchor_converge() {
    let coarse = pendulum(5);
    let refined = pendulum(1);
    // One-metre physical pendulum, released at 0.6 rad. A small-angle point-mass
    // oracle would miss tens of milliseconds per period.
    assert!(coarse.period_error < 0.005 && refined.period_error < 0.001);
    assert!(refined.period_error < coarse.period_error);
    assert!(
        refined.relative_energy_error < 0.003
            && refined.relative_energy_error < coarse.relative_energy_error
    );
    assert!(refined.anchor_error < 1e-5 && refined.anchor_error < coarse.anchor_error);
}

fn box_model(name: &str, center: DVec3, pitch: f64, size: DVec3, fixed: bool) -> String {
    // Use the analytic COM tensor of a 1 kg cuboid. The fixed base retains the same link frame.
    let squared = size * size / 12.0;
    format!(
        r#"<model name="{name}"><static>{fixed}</static><pose>{} {} {} 0 {pitch} 0</pose>
        <link name="body"><inertial><mass>1</mass><inertia><ixx>{}</ixx><iyy>{}</iyy><izz>{}</izz></inertia></inertial>
        <collision name="shape"><geometry><box><size>{} {} {}</size></box></geometry></collision></link></model>"#,
        center.x,
        center.y,
        center.z,
        squared.y + squared.z,
        squared.x + squared.z,
        squared.x + squared.y,
        size.x,
        size.y,
        size.z
    )
}

#[test]
fn coulomb_incline_holds_below_the_friction_cone_and_slides_above_it() {
    // Pinned Rapier's default solid friction is 0.5 on both touching shapes.
    const MU: f64 = 0.5;
    for dt_ms in [4, 1] {
        for tangent in [0.4_f64, 0.8] {
            let angle = tangent.atan();
            let frame = DQuat::from_rotation_y(angle);
            let downhill = frame * DVec3::X;
            let normal = frame * DVec3::Z;
            let mut models = box_model(
                "ramp",
                -0.1 * normal,
                angle,
                DVec3::new(8.0, 2.0, 0.2),
                true,
            );
            models.push_str(&box_model(
                "slider",
                0.1 * normal,
                angle,
                DVec3::new(0.5, 0.5, 0.2),
                false,
            ));
            let mut sim = world(&models, dt_ms);
            let block = named(&sim, "default::slider::body");
            sim.step_exact(200 / dt_ms).unwrap();
            let start = velocity(&sim, block).0.dot(downhill);
            sim.step_exact(600 / dt_ms).unwrap();
            let speed = velocity(&sim, block).0.dot(downhill);
            let (position, rotation) = pose(&sim, block);
            let distance = (position - 0.1 * normal).dot(downhill);
            let acceleration = (speed - start) / 0.6;
            let analytic = (GRAVITY * (angle.sin() - MU * angle.cos())).max(0.0);
            let inferred_mu = (angle.sin() - acceleration / GRAVITY) / angle.cos();
            eprintln!(
                "incline dt={dt_ms} ms tan(theta)={tangent}: downhill distance={distance:.8} m, speed={speed:.8} m/s, acceleration={acceleration:.8} m/s² (analytic {analytic:.8})"
            );
            assert!((position.dot(normal) - 0.1).abs() < 0.002);
            assert!(position.y.abs() < 0.002 && (rotation * DVec3::Z).dot(normal) > 0.999);
            if tangent < MU {
                assert!(distance.abs() < 0.002 && speed.abs() < 0.002);
            } else {
                assert!((inferred_mu - MU).abs() < 0.01);
                assert!((distance - 0.5 * analytic * 0.8_f64.powi(2)).abs() < 0.015);
            }
        }
    }
}

#[test]
fn stacked_cubes_settle_to_the_geometric_contact_heights_without_creep() {
    const N: usize = 6;
    const SIDE: f64 = 0.4;
    let mut models = box_model(
        "floor",
        DVec3::new(0.0, 0.0, -0.1),
        0.0,
        DVec3::new(4.0, 4.0, 0.2),
        true,
    );
    for i in 0..N {
        let center = DVec3::new(0.0, 0.0, (i as f64 + 0.5) * SIDE + (i + 1) as f64 * 0.005);
        models.push_str(&box_model(
            &format!("cube{i}"),
            center,
            0.0,
            DVec3::splat(SIDE),
            false,
        ));
    }
    let mut sim = world(&models, 1);
    let cubes: [_; N] = std::array::from_fn(|i| named(&sim, &format!("default::cube{i}::body")));
    sim.step_exact(1_000).unwrap();
    let mut max_vertical_overlap = 0.0_f64;
    let mut max_height_error = 0.0_f64;
    let mut max_drift = 0.0_f64;
    let mut max_speed = 0.0_f64;
    for _ in 0..100 {
        sim.step_exact(10).unwrap();
        let mut top = 0.0;
        for (i, &cube) in cubes.iter().enumerate() {
            let (position, rotation) = pose(&sim, cube);
            let radius = 0.5
                * SIDE
                * ((rotation * DVec3::X).z.abs()
                    + (rotation * DVec3::Y).z.abs()
                    + (rotation * DVec3::Z).z.abs());
            // World-Z intervals conservatively bound the penetration of adjacent tilted cubes.
            max_vertical_overlap = max_vertical_overlap.max(top - (position.z - radius));
            max_height_error = max_height_error.max((position.z - (i as f64 + 0.5) * SIDE).abs());
            max_drift = max_drift.max(position.x.hypot(position.y));
            let (linear, angular) = velocity(&sim, cube);
            // Bound the velocity anywhere in the cube by its circumradius.
            max_speed =
                max_speed.max(linear.length() + 0.5 * 3.0_f64.sqrt() * SIDE * angular.length());
            assert!((rotation * DVec3::Z).z > 0.999);
            top = position.z + radius;
        }
    }
    eprintln!(
        "six-cube stack, from 1 to 2 seconds: max adjacent vertical overlap={max_vertical_overlap:e} m, height error={max_height_error:e} m, lateral drift={max_drift:e} m, surface speed bound={max_speed:e} m/s"
    );
    assert!(max_vertical_overlap < 0.003 && max_height_error < 0.015);
    assert!(max_drift < 0.005 && max_speed < 0.01);
}
