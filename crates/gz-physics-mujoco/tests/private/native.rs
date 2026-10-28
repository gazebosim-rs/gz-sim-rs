// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn sphere(position: [f64; 3], joint: BodyJoint) -> BodyModel {
    BodyModel {
        position,
        quaternion: [1.0, 0.0, 0.0, 0.0],
        joint,
        geometry: Geometry::Sphere(0.1),
        inertial: Inertial {
            mass: 1.0,
            center: [0.0; 3],
            principal: [0.004; 3],
            quaternion: [1.0, 0.0, 0.0, 0.0],
        },
    }
}

#[test]
fn library_search_prefers_explicit_file_then_prefix_then_soname() {
    assert_eq!(
        library_path(Some("/x/libm.so".into()), Some("/p".into())),
        OsString::from("/x/libm.so")
    );
    assert_eq!(
        library_path(None, Some("/p".into())),
        OsString::from("/p/lib/libmujoco.so.3.11.0")
    );
    assert_eq!(library_path(None, None), OsString::from(LIBRARY_NAME));
}

#[test]
fn loader_rejects_missing_libraries_symbols_and_versions() {
    let missing = load(&"/nonexistent/libmujoco.so.3.11.0".into())
        .err()
        .unwrap();
    assert!(
        missing.starts_with("cannot load /nonexistent/"),
        "{missing}"
    );
    // The C library loads but lacks every MuJoCo symbol.
    let foreign = load(&"libc.so.6".into()).err().unwrap();
    assert!(foreign.contains("mj_compile"), "{foreign}");
    assert_eq!(check_version(3_011_000), Ok(()));
    assert_eq!(
        check_version(3_010_000),
        Err("loaded MuJoCo version 3010000 does not match bindings version 3011000".into())
    );
    assert!(api().is_ok());
}

#[test]
fn log_messages_print_warnings_abort_errors_and_ignore_info() {
    assert_eq!(
        log_action(sys::mjLOG_WARNING, "w"),
        LogAction::Print("MuJoCo warning: w\n".into())
    );
    assert_eq!(
        log_action(sys::mjLOG_ERROR, "e"),
        LogAction::Abort("MuJoCo fatal error: e\n".into())
    );
    assert_eq!(log_action(sys::mjLOG_INFO, "i"), LogAction::Ignore);
    let mut message = sys::mjLogMessage {
        level: sys::mjLOG_INFO as c_int,
        topic: 0,
        subject: [0; 1024],
        body: std::ptr::null(),
        func: std::ptr::null(),
        file: std::ptr::null(),
        line: 0,
        timestamp: false,
    };
    for (slot, byte) in message.subject.iter_mut().zip(b"handler test") {
        *slot = *byte as std::ffi::c_char;
    }
    for level in [sys::mjLOG_INFO, sys::mjLOG_WARNING] {
        message.level = level as c_int;
        // SAFETY: a valid message with a NUL-terminated subject; neither level aborts.
        unsafe { log_handler(&message) };
    }
}

#[test]
fn large_static_worlds_recompile_with_upstream_arena_size() {
    let bodies: Vec<_> = (0..1400)
        .map(|i| sphere([i as f64, 0.0, 0.0], BodyJoint::Fixed))
        .collect();
    let world = World::compile([0.0; 3], &bodies).unwrap();
    assert_eq!(world.nbody(), 1401);
    assert!(world.model().narena >= 1401 * 1401 * 8);
    assert_eq!(world.body_position(1400), [1399.0, 0.0, 0.0]);
    assert_eq!(world.dofs(1), (0, 0));
    assert_eq!(world.joint_address(1), None);
}

#[test]
fn compiled_layout_options_and_quaternion_order_match_the_lowering() {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let rotated = BodyModel {
        quaternion: [s, 0.0, 0.0, s],
        ..sphere([1.0, 2.0, 3.0], BodyJoint::Free)
    };
    let hinge = sphere(
        [0.0, 0.0, -1.0],
        BodyJoint::Hinge {
            anchor: [0.0, 0.0, 1.0],
            axis: [0.0, 1.0, 0.0],
        },
    );
    let mut world = World::compile([0.0, 0.0, -9.81], &[rotated, hinge]).unwrap();
    assert_eq!(world.timestep(), 0.001);
    assert_eq!(
        world.model().opt.integrator,
        sys::mjINT_IMPLICITFAST as c_int
    );
    assert_eq!(world.gravity(), [0.0, 0.0, -9.81]);
    assert_eq!(world.nv(), 7);
    // MuJoCo stores [w,x,y,z]; free qpos holds the world pose of body 1.
    assert_eq!(world.body_quaternion(1), [s, 0.0, 0.0, s]);
    assert_eq!(world.qpos()[..7], [1.0, 2.0, 3.0, s, 0.0, 0.0, s]);
    assert_eq!(world.joint_address(1), Some((0, 0)));
    assert_eq!(world.joint_address(2), Some((7, 6)));
    assert_eq!(world.dofs(2), (6, 1));
    world.qvel_mut()[6] = 2.0;
    world.refresh();
    // Hinge about +Y through the origin: a body at -Z moves along -X... rotate +Y
    // takes -Z to -X, so its COM velocity is omega x r = (0,2,0) x (0,0,-1).
    let velocity = world.com_velocity(2);
    assert_eq!(velocity[..3], [0.0, 2.0, 0.0]);
    assert!((velocity[3] + 2.0).abs() < 1e-15 && velocity[5].abs() < 1e-15);
    world.set_time(1.5);
    world.step(0.002);
    assert_eq!(world.timestep(), 0.002);
    assert_eq!(world.time(), 1.502);
    assert_eq!(world.warnings(), [0; WARNING_NAMES.len()]);
}

#[test]
fn compile_errors_return_the_native_message() {
    let massless = BodyModel {
        inertial: Inertial {
            mass: 0.0,
            center: [0.0; 3],
            principal: [1.0; 3],
            quaternion: [1.0, 0.0, 0.0, 0.0],
        },
        ..sphere([0.0; 3], BodyJoint::Free)
    };
    let error = World::compile([0.0; 3], &[massless]).err().unwrap();
    assert!(
        error.contains("mass and inertia of moving bodies"),
        "{error}"
    );
}

#[test]
fn the_compiler_applies_exactly_the_admission_triangle_rule() {
    // MuJoCo's strict A + B >= C on the exact doubles passed in.
    let edge = 2.0_f64;
    let outside = f64::from_bits(edge.to_bits() + 1);
    for (moment, accepted) in [(edge, true), (outside, false)] {
        let body = BodyModel {
            inertial: Inertial {
                mass: 1.0,
                center: [0.0; 3],
                principal: [1.0, 1.0, moment],
                quaternion: [1.0, 0.0, 0.0, 0.0],
            },
            ..sphere([0.0; 3], BodyJoint::Free)
        };
        assert_eq!(
            crate::validate::mujoco_accepts(1.0, [1.0, 1.0, moment]),
            accepted
        );
        let compiled = World::compile([0.0; 3], &[body]);
        assert_eq!(compiled.is_ok(), accepted);
        if let Err(error) = compiled {
            assert!(error.contains("A + B >= C"), "{error}");
        }
    }
}

#[test]
fn empty_worlds_expose_empty_state_arrays() {
    let mut world = World::compile([0.0; 3], &[]).unwrap();
    assert_eq!(world.nbody(), 1);
    assert!(world.qpos().is_empty());
    assert!(world.qvel().is_empty());
    assert!(world.qpos_mut().is_empty());
    assert!(world.qvel_mut().is_empty());
    assert_eq!(world.xfrc_applied_mut().len(), 6);
}

#[test]
fn warning_classification_detects_repeated_resets_and_new_recoverable_warnings() {
    let qvel = sys::mjWARN_BADQVEL as usize;
    let contact = sys::mjWARN_CONTACTFULL as usize;
    let mut previous = [0; WARNING_NAMES.len()];
    previous[qvel] = 1;
    previous[contact] = 2;
    // A repeated reset leaves the fatal count at one; time going back reveals it.
    let mut counts = [0; WARNING_NAMES.len()];
    counts[qvel] = 1;
    assert_eq!(
        classify_warnings(counts, previous, true),
        (vec!["bad qvel"], vec![])
    );
    assert_eq!(classify_warnings(counts, counts, false), (vec![], vec![]));
    let mut more = previous;
    more[contact] = 3;
    assert_eq!(
        classify_warnings(more, previous, false),
        (vec![], vec!["contact buffer full"])
    );
}

#[test]
#[should_panic(expected = "outside the compiled model")]
fn native_body_queries_refuse_unknown_indices() {
    let world = World::compile([0.0; 3], &[]).unwrap();
    world.com_velocity(1);
}

#[test]
fn a_real_contact_buffer_overflow_is_recoverable() {
    // Sixty overlapping spheres produce more contacts than a 1 MiB arena holds.
    // MuJoCo drops the excess and warns (logging only the first occurrence)
    // without resetting its data.
    let bodies: Vec<_> = (0..60)
        .map(|i| sphere([0.0, 0.0, i as f64 * 1e-3], BodyJoint::Free))
        .collect();
    let mut world = World::compile_with_arena([0.0; 3], &bodies, 1 << 20).unwrap();
    for step in 1..=2 {
        let before = world.warnings();
        world.step(0.001);
        let (fatal, recoverable) = classify_warnings(world.warnings(), before, false);
        assert!(fatal.is_empty(), "{fatal:?}");
        assert_eq!(recoverable, ["contact buffer full"]);
        assert_eq!(world.time(), 0.001 * step as f64);
    }
}
