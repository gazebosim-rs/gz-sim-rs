// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! MuJoCo-specific topology edits, frame conventions, engine faults and lifetime.
use gz_physics::{BodyId, DynamicBackend, PhysicsError, install};
use gz_physics_mujoco::{BodySpec, BodyState, Inertia, MujocoBackend, RotationSpec, Shape};
use gz_sim_core::{Simulation, StepFault};
use std::{sync::Arc, time::Duration};

const DT: Duration = Duration::from_millis(1);

fn ball(position: [f64; 3]) -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.5 },
        position,
        velocity: [0.3, -0.2, 1.0],
        mass: Some(1.5),
    }
}

/// A tumbling body with an offset, non-diagonal inertia: sensitive to any
/// change in pose, velocity or angular state.
fn tumbling(mujoco: &mut MujocoBackend) -> BodyId {
    mujoco
        .spawn_link(
            ball([0.0, 0.0, 5.0]),
            RotationSpec {
                orientation: [0.0, 0.0, 0.5_f64.sin(), 0.5_f64.cos()],
                angular_velocity: [1.0, -2.0, 3.0],
            },
            Some(Inertia {
                center: [0.1, -0.05, 0.02],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.2, 0.3, 0.4, 0.02, 0.0, 0.01],
            }),
        )
        .unwrap()
}

fn steps(mujoco: &mut MujocoBackend, count: usize) {
    for _ in 0..count {
        mujoco.step(DT).unwrap();
    }
}

#[test]
fn spawning_and_removing_other_bodies_preserves_free_body_state_exactly() {
    let mut reference = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let mut edited = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let tracked = tumbling(&mut reference);
    assert_eq!(tumbling(&mut edited), tracked);
    steps(&mut reference, 50);
    steps(&mut edited, 50);
    let before = edited.state(tracked).unwrap();
    let other = edited.spawn(ball([10.0, 0.0, 0.0])).unwrap();
    assert_eq!(edited.state(tracked).unwrap(), before);
    assert_eq!(edited.state(other).unwrap().position, [10.0, 0.0, 0.0]);
    steps(&mut reference, 50);
    steps(&mut edited, 50);
    assert!(edited.remove(other));
    assert_eq!(edited.state(tracked), reference.state(tracked));
    steps(&mut reference, 50);
    steps(&mut edited, 50);
    // Unconstrained motion is bitwise identical across recompiles.
    assert_eq!(edited.state(tracked), reference.state(tracked));
}

#[test]
fn hinge_state_survives_unrelated_topology_edits() {
    let build = || {
        let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
        let body = mujoco.spawn(ball([1.0, 0.0, 0.0])).unwrap();
        let joint = mujoco
            .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap();
        steps(&mut mujoco, 100);
        (mujoco, body, joint)
    };
    let (mut reference, body, joint) = build();
    let (mut mujoco, ..) = build();
    let hinge = mujoco.joint_state(joint).unwrap();
    let pose = mujoco.state(body).unwrap();
    let other = mujoco.spawn(ball([10.0, 0.0, 0.0])).unwrap();
    assert_eq!(mujoco.joint_state(joint), Some(hinge));
    assert_eq!(mujoco.state(body), Some(pose));
    assert!(mujoco.remove(other));
    assert_eq!(mujoco.joint_state(joint), Some(hinge));
    // Static bodies also survive recompiles in place.
    let ground = mujoco
        .spawn(BodySpec {
            mass: None,
            velocity: [0.0; 3],
            ..ball([0.0, 0.0, -5.0])
        })
        .unwrap();
    assert_eq!(mujoco.state(ground).unwrap().position, [0.0, 0.0, -5.0]);
    for _ in 0..2 {
        steps(&mut mujoco, 50);
        steps(&mut reference, 50);
        // Each spawn recompiles at the next step with the hinge copied exactly.
        mujoco.spawn(ball([20.0, 0.0, 0.0])).unwrap();
        assert_eq!(mujoco.joint_state(joint), reference.joint_state(joint));
        assert_eq!(mujoco.state(body), reference.state(body));
        assert_eq!(mujoco.state(ground).unwrap().position, [0.0, 0.0, -5.0]);
    }
    steps(&mut mujoco, 50);
    steps(&mut reference, 50);
    assert_eq!(mujoco.joint_state(joint), reference.joint_state(joint));
}

#[test]
fn hinging_a_compiled_moving_body_projects_its_velocity_and_impulses_follow_the_hinge() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let body = mujoco
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [1.0, 0.0, 0.0],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [1e-3, 1e-3, 1e-3, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    // Compiled as a free body, then given a velocity before the hinge exists.
    mujoco
        .set_world_velocity(body, [0.5, 2.0, 0.0], [0.0; 3])
        .unwrap();
    let joint = mujoco
        .add_world_revolute(body, [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    let hinge = mujoco.joint_state(joint).unwrap();
    assert_eq!(hinge.position, 0.0);
    assert!((hinge.velocity - 2.0 / 1.001).abs() < 1e-12);
    // An angular impulse about the hinge changes its rate by L / (m r^2 + I).
    mujoco
        .apply_impulse(body, [0.0; 3], [0.0, 0.0, 1.001])
        .unwrap();
    let after = mujoco.joint_state(joint).unwrap();
    assert!((after.velocity - hinge.velocity - 1.0).abs() < 1e-12);
    // A tangential linear impulse at r = 1 acts the same way.
    mujoco
        .apply_impulse(body, [0.0, 1.001, 0.0], [0.0; 3])
        .unwrap();
    let last = mujoco.joint_state(joint).unwrap();
    assert!((last.velocity - after.velocity - 1.0).abs() < 1e-12);
}

#[test]
fn adding_a_hinge_projects_spawn_velocity_and_removing_it_keeps_world_motion() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    // Unit point-like mass one meter from a Z hinge, moving tangentially and radially.
    let body = mujoco
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [1.0, 0.0, 0.0],
                velocity: [0.5, 2.0, 0.0],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [1e-3, 1e-3, 1e-3, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let joint = mujoco
        .add_world_revolute(body, [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    // Momentum-consistent projection: L = m r v_t / (m r^2 + I) about the hinge.
    let expected = 2.0 / 1.001;
    let hinge = mujoco.joint_state(joint).unwrap();
    assert_eq!(hinge.position, 0.0);
    assert!((hinge.velocity - expected).abs() < 1e-12);
    let state = mujoco.state(body).unwrap();
    assert!(state.velocity[0].abs() < 1e-12);
    assert!((state.velocity[1] - expected).abs() < 1e-12);
    steps(&mut mujoco, 300);
    let swung = mujoco.state(body).unwrap();
    let angle = mujoco.joint_state(joint).unwrap().position;
    assert!((angle - expected * 0.3).abs() < 1e-9);
    assert!(mujoco.remove_joint(joint));
    assert_eq!(mujoco.is_jointed(body), Some(false));
    let freed = mujoco.state(body).unwrap();
    for (a, b) in [
        (freed.position, swung.position),
        (freed.velocity, swung.velocity),
        (freed.angular_velocity, swung.angular_velocity),
    ] {
        for (a, b) in a.into_iter().zip(b) {
            assert!((a - b).abs() < 1e-12);
        }
    }
    for (a, b) in freed.orientation.into_iter().zip(swung.orientation) {
        assert!((a - b).abs() < 1e-15);
    }
    steps(&mut mujoco, 100);
    let ballistic = mujoco.state(body).unwrap();
    for i in 0..3 {
        let expected = freed.position[i] + freed.velocity[i] * 0.1;
        assert!((ballistic.position[i] - expected).abs() < 1e-9);
    }
}

#[test]
fn world_angular_velocity_and_com_velocity_conventions_hold_for_rotated_offset_bodies() {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    // Body rotated 90 degrees about X with its COM 0.5 m along body X.
    let body = mujoco
        .spawn_link(
            ball([0.0; 3]),
            RotationSpec {
                orientation: [s, 0.0, 0.0, s],
                angular_velocity: [0.0; 3],
            },
            Some(Inertia {
                center: [0.5, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    mujoco
        .set_world_velocity(body, [0.0; 3], [0.0, 0.0, 2.0])
        .unwrap();
    let state = mujoco.state(body).unwrap();
    assert!(state.velocity.into_iter().all(|v| v.abs() < 1e-12));
    assert!((state.angular_velocity[2] - 2.0).abs() < 1e-12);
    steps(&mut mujoco, 100);
    let state = mujoco.state(body).unwrap();
    // A spin about the COM: the origin circles the fixed COM at [0.5, 0, 0].
    // MuJoCo integrates the free joint at the body origin, so its first-order
    // centripetal error is about omega^2 r dt per step (2e-3 m/s^2 * 100 steps).
    let angle = 0.2_f64;
    assert!((state.position[0] - (0.5 - 0.5 * angle.cos())).abs() < 2e-4);
    assert!((state.position[1] + 0.5 * angle.sin()).abs() < 2e-4);
    assert!(state.position[2].abs() < 1e-12);
    // Orientation is the world-Z rotation applied after the initial X rotation.
    let (sz, cz) = (angle / 2.0).sin_cos();
    let expected = [s * cz, s * sz, s * sz, s * cz];
    for (actual, expected) in state.orientation.into_iter().zip(expected) {
        assert!((actual - expected).abs() < 1e-6, "{state:?}");
    }
}

#[test]
fn admission_applies_mujoco_compiler_rules_exactly_at_the_boundary() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let kept = mujoco.spawn(ball([0.0; 3])).unwrap();
    mujoco.step(DT).unwrap();
    let before = mujoco.state(kept).unwrap();
    let spawn = |mujoco: &mut MujocoBackend, mass, tensor, orientation| {
        mujoco.spawn_link(
            BodySpec {
                mass: Some(mass),
                ..ball([3.0, 0.0, 0.0])
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.0; 3],
                orientation,
                tensor,
            }),
        )
    };
    let identity = [0.0, 0.0, 0.0, 1.0];
    // Beyond the shared 1e-12 relative triangle slack: rejected.
    assert_eq!(
        spawn(
            &mut mujoco,
            1.0,
            [1.0, 1.0, 2.0 + 1e-11, 0.0, 0.0, 0.0],
            identity
        ),
        Err(PhysicsError::InvalidValue)
    );
    // mjMINVAL (1e-15) bounds mass and every principal moment, inclusively.
    for (mass, moment) in [(1e-15 * 0.999, 1.0), (1.0, 1e-15 * 0.999)] {
        assert_eq!(
            spawn(
                &mut mujoco,
                mass,
                [moment, moment, moment, 0.0, 0.0, 0.0],
                identity
            ),
            Err(PhysicsError::InvalidValue)
        );
    }
    assert_eq!(mujoco.state(kept), Some(before));
    // Inside the slack, even one ulp beyond MuJoCo's strict A + B >= C or after a
    // rotation that rounds the same tensor either way, bodies compile.
    let outside = f64::from_bits(2.0_f64.to_bits() + 1);
    let (s45, c45) = (
        std::f64::consts::FRAC_PI_8.sin(),
        std::f64::consts::FRAC_PI_8.cos(),
    );
    let s90 = std::f64::consts::FRAC_1_SQRT_2;
    let admitted: Vec<_> = [
        ([1.0, 1.0, 2.0, 0.0, 0.0, 0.0], identity),
        ([1.0, 1.0, outside, 0.0, 0.0, 0.0], identity),
        ([1.0, 1.0, 2.0, 0.0, 0.0, 0.0], [s45, 0.0, 0.0, c45]),
        ([1.0, 1.0, 2.0, 0.0, 0.0, 0.0], [s90, 0.0, 0.0, s90]),
    ]
    .into_iter()
    .map(|(tensor, orientation)| spawn(&mut mujoco, 1.0, tensor, orientation).unwrap())
    .collect();
    let tiny = spawn(
        &mut mujoco,
        1e-15,
        [1e-15, 1e-15, 1e-15, 0.0, 0.0, 0.0],
        identity,
    )
    .unwrap();
    assert_eq!(mujoco.state(kept), Some(before));
    mujoco.step(DT).unwrap();
    assert!(
        admitted
            .iter()
            .chain([&tiny])
            .all(|&id| mujoco.state(id).is_some())
    );
    assert_eq!(tiny.raw(), 5);
}

#[test]
fn numerical_engine_warnings_fault_the_step_instead_of_resetting_silently() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let body = mujoco.spawn(ball([0.0; 3])).unwrap();
    // Velocity inputs are bounded by mjMAXVAL; an extreme force still produces an
    // acceleration beyond it, after which MuJoCo resets its data.
    assert_eq!(
        mujoco.set_world_velocity(body, [1e11, 0.0, 0.0], [0.0; 3]),
        Err(PhysicsError::InvalidValue)
    );
    let bad_qacc = PhysicsError::Native {
        engine: "mujoco",
        message: "MuJoCo reset its data after bad qacc warnings: every body returned \
                  to its initial configuration and time to zero"
            .into(),
    };
    for _ in 0..2 {
        mujoco.step(DT).unwrap();
        mujoco
            .add_world_wrench(body, [1e20, 0.0, 0.0], [0.0; 3])
            .unwrap();
        // The second reset leaves MuJoCo's count at one; it is still reported.
        assert_eq!(mujoco.step(DT), Err(bad_qacc.clone()));
        // MuJoCo's reset already moved the body back and cleared its velocity.
        let state = mujoco.state(body).unwrap();
        assert_eq!(state.position, [0.0; 3]);
        assert_eq!(state.velocity, [0.0; 3]);
    }
    // The simulator records the fault through the shared installer.
    mujoco
        .add_world_wrench(body, [1e20, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let mut sim = Simulation::new(DT).unwrap();
    install(&mut sim, mujoco).unwrap();
    assert!(sim.step_exact(1).is_err());
    assert!(
        sim.world()
            .resource::<StepFault>()
            .error()
            .is_some_and(|e| e.to_string().contains("bad qacc"))
    );
}

#[test]
fn unrepresentable_construction_velocities_are_rejected_at_the_call() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let kept = mujoco.spawn(ball([0.0; 3])).unwrap();
    mujoco
        .set_world_velocity(kept, [1.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let before = mujoco.state(kept).unwrap();
    let offset = |center| {
        Some(Inertia {
            center,
            orientation: [0.0, 0.0, 0.0, 1.0],
            tensor: [0.1, 0.1, 0.1, 0.0, 0.0, 0.0],
        })
    };
    for (velocity, angular, inertia) in [
        // Reproduced review cases: an overflowing linear velocity, and a spin
        // whose body-origin velocity overflows through the COM offset.
        ([1e308, 0.0, 0.0], [0.0; 3], None),
        ([0.0; 3], [0.0, 0.0, 1e308], offset([10.0, 0.0, 0.0])),
        // Finite but beyond mjMAXVAL once converted to the body origin.
        ([0.0; 3], [0.0, 0.0, 1e9], offset([100.0, 0.0, 0.0])),
    ] {
        assert_eq!(
            mujoco.spawn_link(
                BodySpec {
                    velocity,
                    mass: Some(2.0),
                    ..ball([5.0, 0.0, 0.0])
                },
                RotationSpec {
                    angular_velocity: angular,
                    ..RotationSpec::default()
                },
                inertia,
            ),
            Err(PhysicsError::InvalidValue)
        );
    }
    // A representable free velocity whose hinge projection is not: a point-like
    // body 1 mm from the hinge axis turning at v / r = 8e12 rad/s.
    let body = mujoco
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 1e-4 },
                position: [5.001, 0.0, 0.0],
                velocity: [0.0, 8e9, 0.0],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            offset([0.0; 3]).map(|inertia| Inertia {
                tensor: [1e-12, 1e-12, 1e-12, 0.0, 0.0, 0.0],
                ..inertia
            }),
        )
        .unwrap();
    assert_eq!(
        mujoco.add_world_revolute(body, [5.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(mujoco.is_jointed(body), Some(false));
    assert_eq!(mujoco.state(kept), Some(before));
    mujoco.step(DT).unwrap();
    assert!(mujoco.state(kept).is_some() && mujoco.state(body).is_some());
    assert_eq!(body.raw(), 1);
}

#[test]
fn uncompiled_answers_match_what_the_compiled_model_reports() {
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let mover = mujoco.spawn(ball([10.0, 0.0, 0.0])).unwrap();
    mujoco
        .set_world_velocity(mover, [1.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let hinged = mujoco
        .spawn_link(
            BodySpec {
                velocity: [0.2, 0.7, -0.1],
                ..ball([1.0, 0.5, 0.0])
            },
            RotationSpec {
                orientation: [s, 0.0, 0.0, s],
                angular_velocity: [0.3, -0.4, 1.2],
            },
            Some(Inertia {
                center: [0.1, 0.2, -0.05],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.2, 0.3, 0.4, 0.01, 0.0, 0.02],
            }),
        )
        .unwrap();
    let joint = mujoco
        .add_world_revolute(hinged, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0])
        .unwrap();
    let free = mujoco.spawn(ball([20.0, 0.0, 0.0])).unwrap();
    let uncompiled = (
        mujoco.joint_state(joint),
        mujoco.state(hinged),
        mujoco.state(free),
    );
    // An input on another body compiles pending edits without moving these.
    mujoco
        .set_world_velocity(mover, [0.0; 3], [0.0; 3])
        .unwrap();
    assert_eq!(
        (
            mujoco.joint_state(joint),
            mujoco.state(hinged),
            mujoco.state(free)
        ),
        uncompiled
    );
    // After a step the native model reports; the hinge continued from that rate.
    let rate = uncompiled.0.unwrap().velocity;
    mujoco.step(DT).unwrap();
    let after = mujoco.joint_state(joint).unwrap();
    assert!((after.position - rate * 0.001).abs() < 1e-9 * rate.abs().max(1.0));
}

#[test]
fn zero_steps_change_nothing_and_keep_joint_construction_open() {
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let body = mujoco.spawn(ball([0.0; 3])).unwrap();
    let before = mujoco.state(body).unwrap();
    mujoco
        .add_world_wrench(body, [1.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    mujoco.step(Duration::ZERO).unwrap();
    assert_eq!(mujoco.state(body), Some(before));
    mujoco
        .add_world_revolute(body, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
        .unwrap();
    assert_eq!(mujoco.gravity(), [0.0, 0.0, -9.81]);
}

#[test]
fn prepared_and_retained_resets_restore_initial_bodies_and_retire_later_ids() {
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let initial_body = tumbling(&mut mujoco);
    let state = mujoco.state(initial_body).unwrap();
    let initial = mujoco.initial_state();
    assert!(mujoco.owns_initial_state(&initial));
    steps(&mut mujoco, 10);
    assert_ne!(mujoco.state(initial_body), Some(state));
    assert!(mujoco.remove(initial_body));
    let runtime = mujoco.spawn(ball([0.0, 0.0, 10.0])).unwrap();
    let runtime_state = mujoco.state(runtime);
    let prepared = mujoco.prepare_initial(&initial).unwrap();
    assert_eq!(mujoco.state(initial_body), None);
    assert_eq!(mujoco.state(runtime), runtime_state);
    let later = mujoco.spawn(ball([0.0, 0.0, 20.0])).unwrap();
    mujoco.apply_initial(prepared).unwrap();
    assert_eq!(mujoco.state(initial_body), Some(state));
    assert_eq!(mujoco.state(runtime), None);
    assert_eq!(mujoco.state(later), None);
    let fresh = mujoco.spawn(ball([0.0, 0.0, 30.0])).unwrap();
    assert!(fresh > later);
    let foreign = MujocoBackend::new([0.0; 3]).unwrap();
    assert!(!foreign.owns_initial_state(&initial));
    assert_eq!(
        foreign
            .prepare_initial(&initial)
            .err()
            .map(|e| e.to_string()),
        Some(PhysicsError::ForeignInitialState.to_string())
    );
    let prepared = mujoco.prepare_initial(&initial).unwrap();
    let mut other = MujocoBackend::new([0.0; 3]).unwrap();
    assert_eq!(
        other.apply_initial(prepared),
        Err(PhysicsError::ForeignInitialState)
    );
    mujoco.reset().unwrap();
    assert_eq!(mujoco.state(initial_body), Some(state));
    assert_eq!(mujoco.state(fresh).unwrap().position, [0.0, 0.0, 30.0]);
}

#[test]
fn backends_are_sendable_shareable_and_drop_independently() {
    let shared = Arc::new({
        let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
        mujoco.spawn(ball([0.0; 3])).unwrap();
        steps(&mut mujoco, 10);
        mujoco
    });
    let id = BodyId::from_raw(0);
    let expected = shared.state(id);
    let readers: Vec<_> = (0..4)
        .map(|_| {
            let shared = shared.clone();
            std::thread::spawn(move || (0..100).map(|_| shared.state(id)).collect::<Vec<_>>())
        })
        .collect();
    for reader in readers {
        assert!(reader.join().unwrap().into_iter().all(|s| s == expected));
    }
    let workers: Vec<_> = (0..4)
        .map(|n| {
            std::thread::spawn(move || {
                let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
                for k in 0..20 {
                    let body = mujoco.spawn(ball([k as f64 * 2.0, n as f64, 0.0])).unwrap();
                    mujoco.step(DT).unwrap();
                    if k % 2 == 0 {
                        assert!(mujoco.remove(body));
                    }
                }
                mujoco
            })
        })
        .collect();
    let backends: Vec<MujocoBackend> = workers.into_iter().map(|w| w.join().unwrap()).collect();
    let states: Vec<Option<BodyState>> = backends
        .iter()
        .map(|b| b.state(BodyId::from_raw(1)))
        .collect();
    assert!(states.iter().all(Option::is_some));
    // Dropping on another thread than the creator is part of the contract.
    std::thread::spawn(move || drop(backends)).join().unwrap();
    drop(shared);
}
