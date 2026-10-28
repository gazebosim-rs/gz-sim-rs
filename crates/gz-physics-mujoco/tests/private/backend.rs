// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn ball() -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.5 },
        position: [0.0; 3],
        velocity: [0.0; 3],
        mass: Some(1.0),
    }
}

fn compiles(mujoco: &MujocoBackend) -> u64 {
    mujoco.compiles.load(Ordering::Relaxed)
}

#[test]
fn constructing_many_bodies_and_joints_costs_one_compile() {
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    assert_eq!(compiles(&mujoco), 1);
    let bodies: Vec<_> = (0..200)
        .map(|i| {
            mujoco
                .spawn(BodySpec {
                    position: [2.0 * i as f64, 0.0, 0.0],
                    velocity: [0.0, 0.0, 1.0],
                    ..ball()
                })
                .unwrap()
        })
        .collect();
    // As the scene does: query each joint right after adding it.
    for &body in &bodies[..50] {
        let joint = mujoco
            .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap();
        assert_eq!(mujoco.joint_state(joint).unwrap().position, 0.0);
        assert!(mujoco.state(body).is_some());
    }
    assert!(mujoco.remove(bodies[199]));
    mujoco
        .add_world_wrench(bodies[0], [1.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    // Bodies at their construction state answer from the records.
    let state = mujoco.state(bodies[100]).unwrap();
    assert_eq!(state.velocity, [0.0, 0.0, 1.0]);
    assert_eq!(compiles(&mujoco), 1);
    mujoco.step(Duration::from_millis(1)).unwrap();
    mujoco.step(Duration::from_millis(1)).unwrap();
    assert_eq!(compiles(&mujoco), 2);
    // Moved bodies answer from the committed model while others are edited.
    mujoco.spawn(ball()).unwrap();
    assert!(mujoco.state(bodies[100]).is_some());
    assert_eq!(compiles(&mujoco), 2);
    // A retained reset recompiles once, lazily.
    mujoco.reset().unwrap();
    assert!(mujoco.remove(bodies[198]));
    assert!(mujoco.state(bodies[10]).is_some());
    assert_eq!(compiles(&mujoco), 2);
    mujoco.step(Duration::from_millis(1)).unwrap();
    assert_eq!(compiles(&mujoco), 3);
    assert_eq!(mujoco.engine.slots.len(), 199);
}

#[test]
fn exhausted_identifiers_leave_records_unchanged() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let body = mujoco.spawn(ball()).unwrap();
    mujoco.records.next_body = u64::MAX;
    assert_eq!(mujoco.spawn(ball()), Err(PhysicsError::IdOverflow));
    assert_eq!(mujoco.records.bodies.len(), 1);
    mujoco.records.next_joint = u64::MAX;
    assert_eq!(
        mujoco.add_world_revolute(body, [0.0; 3], [0.0, 0.0, 1.0]),
        Err(PhysicsError::JointIdOverflow)
    );
    assert!(mujoco.records.joints.is_empty());
    mujoco.step(Duration::from_millis(1)).unwrap();
    assert_eq!(mujoco.engine.world.nv(), 6);
}

#[test]
fn static_only_worlds_and_static_inputs_have_no_degrees_of_freedom() {
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let fixed = mujoco
        .spawn_link(
            BodySpec {
                mass: None,
                ..ball()
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    mujoco.apply_impulse(fixed, [1.0; 3], [1.0; 3]).unwrap();
    mujoco
        .set_world_velocity(fixed, [1.0; 3], [1.0; 3])
        .unwrap();
    assert_eq!(mujoco.engine.world.nv(), 0);
    mujoco.add_world_wrench(fixed, [1.0; 3], [1.0; 3]).unwrap();
    mujoco.step(Duration::from_millis(1)).unwrap();
    let state = mujoco.state(fixed).unwrap();
    assert_eq!(state.position, [0.0; 3]);
    assert_eq!(state.velocity, [0.0; 3]);
    assert!(mujoco.wrenches.is_empty());
    assert_eq!(mujoco.is_jointed(fixed), Some(false));
}

#[test]
fn reset_and_rebuilds_keep_the_last_timestep_and_reopen_joints() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    assert_eq!(mujoco.timestep(), 0.001);
    let body = mujoco.spawn(ball()).unwrap();
    mujoco.step(Duration::from_millis(2)).unwrap();
    assert!(!mujoco.joints_open);
    mujoco.spawn(ball()).unwrap();
    assert!(mujoco.state(body).is_some());
    assert_eq!(mujoco.timestep(), 0.002);
    mujoco
        .add_world_wrench(body, [1.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    mujoco.reset().unwrap();
    assert!(mujoco.joints_open);
    assert!(mujoco.wrenches.is_empty());
    assert!(mujoco.state(body).is_some());
    assert_eq!(mujoco.pending_engine().unwrap().world.time(), 0.0);
    assert_eq!(mujoco.timestep(), 0.002);
    let prepared = mujoco.prepare_initial(&mujoco.initial_state()).unwrap();
    mujoco.step(Duration::from_millis(3)).unwrap();
    mujoco.apply_initial(prepared).unwrap();
    assert!(mujoco.joints_open);
    assert_eq!(mujoco.timestep(), 0.003);
    // The trait object exposes the concrete adapter.
    let object: &mut dyn DynamicBackend = &mut mujoco;
    assert!(
        object
            .as_any_mut()
            .downcast_mut::<MujocoBackend>()
            .is_some()
    );
    assert!(object.as_any().downcast_ref::<MujocoBackend>().is_some());
}

#[test]
fn velocities_that_overflow_free_coordinates_are_rejected_atomically() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let body = mujoco
        .spawn_link(
            ball(),
            RotationSpec::default(),
            Some(Inertia {
                center: [0.1, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let before = mujoco.state(body);
    // Finite input, but the body-origin velocity v - w x c overflows.
    assert_eq!(
        mujoco.set_world_velocity(body, [0.0, f64::MAX, 0.0], [0.0, 0.0, -f64::MAX]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(mujoco.state(body), before);
}

#[test]
fn recoverable_warnings_are_reported_and_reset_warnings_fault_the_step() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    mujoco.spawn(ball()).unwrap();
    mujoco.step(Duration::from_millis(1)).unwrap();
    // Counts a step would raise; MuJoCo raises these from buffer overflow.
    mujoco
        .engine
        .world
        .inject_warning(native::sys::mjWARN_CONTACTFULL as usize);
    mujoco
        .engine
        .world
        .inject_warning(native::sys::mjWARN_CNSTRFULL as usize);
    mujoco.step(Duration::from_millis(1)).unwrap();
    assert_eq!(
        mujoco.step_warnings(),
        ["contact buffer full", "constraint buffer full"]
    );
    mujoco.step(Duration::from_millis(1)).unwrap();
    assert!(mujoco.step_warnings().is_empty());
    mujoco
        .engine
        .world
        .inject_warning(native::sys::mjWARN_INERTIA as usize);
    mujoco
        .engine
        .world
        .inject_warning(native::sys::mjWARN_BADQACC as usize);
    let error = mujoco.step(Duration::from_millis(1)).unwrap_err();
    assert!(error.to_string().contains("bad qacc"), "{error}");
    assert_eq!(mujoco.step_warnings(), ["inertia"]);
    mujoco.reset().unwrap();
    assert!(mujoco.step_warnings().is_empty());
}

#[test]
fn engine_compile_faults_fail_the_step_without_changing_state() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let kept = mujoco.spawn(ball()).unwrap();
    mujoco.step(Duration::from_millis(1)).unwrap();
    mujoco
        .add_world_wrench(kept, [1.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    let before = mujoco.state(kept).unwrap();
    // Admission makes this unreachable through the API; inject a record the
    // compiler refuses to stand in for an engine fault.
    let bad = BodyId::from_raw(mujoco.records.next_body);
    mujoco.records.next_body += 1;
    mujoco.records.bodies.insert(
        bad,
        Construction {
            spec: BodySpec {
                mass: Some(1e-20),
                ..ball()
            },
            rotation: RotationSpec::default(),
            inertial: Inertial {
                mass: 1e-20,
                center: [0.0; 3],
                principal: [1.0; 3],
                quaternion: [1.0, 0.0, 0.0, 0.0],
            },
            free_start: [0.0; 6],
        },
    );
    mujoco.pristine.insert(bad);
    mujoco.mark(Edit::Transfer);
    // Compiled bodies whose joint is unchanged still answer.
    assert_eq!(mujoco.state(kept), Some(before));
    let error = mujoco.step(Duration::from_millis(1)).unwrap_err();
    assert!(error.to_string().contains("mjMINVAL"), "{error}");
    assert_eq!(
        mujoco.set_world_velocity(kept, [1.0; 3], [0.0; 3]),
        Err(error)
    );
    assert!(mujoco.wrenches.contains_key(&kept));
    assert!(mujoco.remove(bad));
    assert_eq!(mujoco.state(kept), Some(before));
}

#[test]
fn native_readback_agrees_with_answers_given_before_compiling() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let rotation = RotationSpec {
        orientation: [0.0, s, 0.0, s],
        angular_velocity: [0.4, -0.3, 0.9],
    };
    let inertia = Some(Inertia {
        center: [0.1, -0.2, 0.05],
        orientation: [0.0, 0.0, 0.0, 1.0],
        tensor: [0.2, 0.3, 0.4, 0.01, 0.02, 0.0],
    });
    let spec = BodySpec {
        velocity: [0.3, -0.6, 0.2],
        ..ball()
    };
    let hinged = mujoco.spawn_link(spec, rotation, inertia).unwrap();
    let joint = mujoco
        .add_world_revolute(hinged, [0.5, 0.0, 0.0], [1.0, 2.0, 2.0])
        .unwrap();
    let free = mujoco
        .spawn_link(
            BodySpec {
                position: [5.0, 0.0, 0.0],
                ..spec
            },
            rotation,
            inertia,
        )
        .unwrap();
    let answered = (
        mujoco.joint_state(joint).unwrap(),
        mujoco.state(hinged).unwrap(),
    );
    let free_answer = mujoco.state(free).unwrap();
    mujoco.settle().unwrap();
    let engine = &mujoco.engine;
    // The hinge coordinates MuJoCo holds are exactly the answered ones.
    let (qpos, dof) = engine.world.joint_address(engine.slots[&hinged]).unwrap();
    assert_eq!(engine.world.qpos()[qpos], answered.0.position);
    assert_eq!(engine.world.qvel()[dof], answered.0.velocity);
    // MuJoCo's own body readback agrees to rounding.
    for (native, answer) in [
        (engine.snapshot(hinged).unwrap().state(), answered.1),
        (engine.snapshot(free).unwrap().state(), free_answer),
    ] {
        let pairs = [
            (native.position, answer.position),
            (native.velocity, answer.velocity),
            (native.angular_velocity, answer.angular_velocity),
        ];
        for (a, b) in pairs.into_iter().flat_map(|(a, b)| a.into_iter().zip(b)) {
            assert!((a - b).abs() < 1e-14, "{native:?} versus {answer:?}");
        }
        for (a, b) in native.orientation.into_iter().zip(answer.orientation) {
            assert!((a - b).abs() < 1e-15);
        }
    }
}
