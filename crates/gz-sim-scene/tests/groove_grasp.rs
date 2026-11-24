// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two open travel stops support a sphere sliding along a 90-degree V-groove.
use bevy_ecs::prelude::EntityRef;
use glam::DVec3;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, JointSample, Name, PhysicsEngine, load_world_with_sdf_step, set_joint_force,
};
use std::f64::consts::FRAC_1_SQRT_2;

const WORLD: &str = include_str!("../../../examples/worlds/local_groove_grasp.sdf");
const MASS: f64 = 0.2;
const RADIUS: f64 = 0.015;
const INERTIA: f64 = 0.4 * MASS * RADIUS * RADIUS;
const GRAVITY: f64 = 9.81;
const DRIVE: f64 = 0.5;

fn entity<'w>(sim: &'w Simulation, name: &str) -> EntityRef<'w> {
    #[allow(deprecated)]
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
}

fn body(sim: &Simulation, name: &str) -> BodyId {
    entity(sim, name).get::<BodyHandle>().unwrap().id()
}

#[derive(Default)]
struct Side {
    normal_ns: f64,
    drag_ns: f64,
    opening_stop_ns: f64,
}

fn trial(step_us: usize, direction: f64, mu: f64) -> (f64, f64) {
    let dt = step_us as f64 * 1e-6;
    let speed = 0.5 * direction;
    let xml = WORLD
        .replace(">0.0001<", &format!(">{dt:.8}<"))
        .replace(">0 0.5 0<", &format!(">0 {speed} 0<"))
        .replace(">0.5</mu", &format!(">{mu}</mu"));
    let mut sim = load_world_with_sdf_step(
        &xml,
        "groove.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let parcel = body(&sim, "default::parcel::body");
    let pads = ["left", "right"].map(|name| body(&sim, &format!("default::gripper::{name}")));
    let slides = ["left", "right"].map(|name| {
        let e = entity(&sim, &format!("default::gripper::{name}_slide"));
        let (joint, id) = (e.id(), e.get::<EntityId>().unwrap().get());
        set_joint_force(&mut sim, id, DRIVE).unwrap();
        joint
    });
    let support = MASS * GRAVITY / (2. * FRAC_1_SQRT_2);
    let drag = mu * support;
    let acceleration = 2. * drag / MASS;
    let angular_acceleration = 2. * drag * RADIUS * FRAC_1_SQRT_2 / INERTIA;
    let rolling_time = speed.abs() / (acceleration + RADIUS * FRAC_1_SQRT_2 * angular_acceleration);
    let mut sides = [Side::default(), Side::default()];
    let mut sample_count = 0;
    let mut rolling = None;
    let mut max_momentum_error = 0.0_f64;
    let mut max_angular_error = 0.0_f64;
    let mut max_stop_displacement = 0.0_f64;
    let mut max_path_error = 0.0_f64;
    let mut max_height_error = 0.0_f64;
    for tick in 1..=80_000 / step_us {
        let before = sim.world().resource::<PhysicsBackend>();
        let previous = before.state(parcel).unwrap();
        let pad_velocities = pads.map(|id| before.state(id).unwrap().velocity[0]);
        sim.step_exact(1).unwrap();
        let backend = sim.world().resource::<PhysicsBackend>();
        let sphere = backend.state(parcel).unwrap();
        let time = (tick * step_us) as f64 * 1e-6;
        let sample = (2_000..=10_000).contains(&(tick * step_us));
        let mut impulse = DVec3::ZERO;
        let mut torque = DVec3::ZERO;
        let mut seen = [false; 2];
        let mut max_slip = 0.0_f64;
        for patch in backend
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .rigid_contact_patches()
        {
            let on_first = patch.a == parcel;
            assert!(on_first || patch.b == parcel);
            let pad = if on_first { patch.b } else { patch.a };
            let side = pads.iter().position(|&id| id == pad).unwrap();
            let sign = if on_first { 1. } else { -1. };
            assert_eq!(patch.points.len(), 1);
            assert!(!seen[side]);
            seen[side] = true;
            for point in patch.points {
                let n = sign * DVec3::from(patch.normal_on_a) * point.normal_impulse;
                let t = sign * DVec3::from(point.tangent_impulse_on_a.unwrap());
                let j = n + t;
                impulse += j;
                let arm = if on_first {
                    point.lever_a
                } else {
                    point.lever_b
                };
                torque += DVec3::from(arm).cross(j);
                // Both sides can move. Measure the two material velocities at
                // the actual solver anchors instead of assuming stationary pads.
                let pad_state = backend.state(pad).unwrap();
                let pad_arm = if on_first {
                    point.lever_b
                } else {
                    point.lever_a
                };
                let sphere_v = DVec3::from(sphere.velocity)
                    + DVec3::from(sphere.angular_velocity).cross(DVec3::from(arm));
                let pad_v = DVec3::from(pad_state.velocity)
                    + DVec3::from(pad_state.angular_velocity).cross(DVec3::from(pad_arm));
                max_slip = max_slip.max((sphere_v.y - pad_v.y).abs());
                if sample {
                    assert!(point.normal_impulse > 0.);
                    let inward = if side == 0 { 1. } else { -1. };
                    let dv = backend.state(pad).unwrap().velocity[0] - pad_velocities[side];
                    // Positive force from the opening stop points inward.
                    sides[side].opening_stop_ns += inward * (0.08 * dv + j.x) - DRIVE * dt;
                    sides[side].normal_ns += point.normal_impulse;
                    sides[side].drag_ns -= direction * t.y;
                }
            }
        }
        if sample {
            assert_eq!(seen, [true; 2]);
            sample_count += 1;
            for joint in slides {
                let q = sim
                    .world()
                    .get::<JointSample>(joint)
                    .unwrap()
                    .state
                    .position;
                max_stop_displacement = max_stop_displacement.max(q.abs());
            }
        }
        let dp = MASS
            * (DVec3::from(sphere.velocity) - DVec3::from(previous.velocity)
                + DVec3::Z * GRAVITY * dt);
        let dl = INERTIA
            * (DVec3::from(sphere.angular_velocity) - DVec3::from(previous.angular_velocity));
        max_momentum_error = max_momentum_error.max((dp - impulse).length());
        max_angular_error = max_angular_error.max((dl - torque).length());
        if seen == [true; 2] && max_slip < 0.001 && rolling.is_none() {
            rolling = Some(time);
        }
        let sliding = time.min(rolling_time);
        let y = speed * time - direction * acceleration * sliding * (time - 0.5 * sliding);
        max_path_error = max_path_error.max((sphere.position[1] - y).abs());
        max_height_error = max_height_error.max((sphere.position[2] - 0.30).abs());
    }
    let duration = f64::from(sample_count) * dt;
    eprintln!(
        "us={step_us} direction={direction} mu={mu} rolling={rolling:?} expected={rolling_time} y_error_m={max_path_error} height_error_m={max_height_error} q_error_m={max_stop_displacement} dp_error_ns={max_momentum_error} dl_error_nms={max_angular_error}"
    );
    for s in sides {
        let (n, t, stop) = (
            s.normal_ns / duration,
            s.drag_ns / duration,
            s.opening_stop_ns / duration,
        );
        eprintln!("normal_n={n} drag_n={t} stop_n={stop}");
        assert!((n - support).abs() < 0.003);
        assert!((t - drag).abs() < 0.002);
        assert!((stop - (support * FRAC_1_SQRT_2 - DRIVE)).abs() < 0.004);
    }
    assert!(max_stop_displacement < 0.1e-6);
    assert!(max_momentum_error < 1e-12);
    assert!(max_angular_error < 1e-13);
    assert!(max_path_error < 20e-6 && max_height_error < 0.1e-3);
    if mu == 0. {
        assert!(rolling.is_none());
        let final_state = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(parcel)
            .unwrap();
        assert!(DVec3::from(final_state.angular_velocity).length() < 1e-8);
        assert!((final_state.velocity[1] - speed).abs() < 1e-8);
    } else {
        assert!((rolling.unwrap() - rolling_time).abs() < 0.0005);
    }
    (max_path_error, max_height_error)
}

#[test]
fn sliding_sphere_loads_both_opening_stops_and_rolls_in_either_direction() {
    for direction in [-1., 1.] {
        let mut previous = (f64::INFINITY, f64::INFINITY);
        for us in [100, 50, 25] {
            let errors = trial(us, direction, 0.5);
            assert!(errors.0 < previous.0 && errors.1 < previous.1);
            previous = errors;
        }
    }
}

#[test]
fn removing_friction_preserves_support_and_removes_spin_up() {
    let mut previous_height_error = f64::INFINITY;
    for us in [100, 50, 25] {
        let (_, height_error) = trial(us, 1., 0.);
        assert!(height_error < previous_height_error);
        previous_height_error = height_error;
    }
}
