// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{BodyId, BodySpec, ContactSegment, DynamicBackend, RapierBackend, Shape};
use std::time::Duration;

const DT: Duration = Duration::from_micros(300);

fn scene(shape: Shape, z: f64) -> (RapierBackend, BodyId, BodyId) {
    let mut world = RapierBackend::new([0., 0., -9.81]).unwrap();
    let block = Shape::Box {
        half_extents: [0.1; 3],
    };
    let pad = world
        .spawn(BodySpec {
            shape: block,
            position: [0.; 3],
            velocity: [0.; 3],
            mass: None,
        })
        .unwrap();
    let foot = world
        .spawn(BodySpec {
            shape,
            position: [0., 0., z],
            velocity: [0., 0., -0.2],
            mass: Some(1.),
        })
        .unwrap();
    (world, foot, pad)
}
fn segments(native: &RapierBackend) -> [ContactSegment; 3] {
    [10, 2, 4].map(|native_steps| ContactSegment {
        duration: DT / 3,
        native_steps,
        iterations: native.ordinary_contact_iterations(),
    })
}

#[test]
fn no_reaction_non_box_and_loaded_contact_preserve_the_original_grid() {
    let block = Shape::Box {
        half_extents: [0.1; 3],
    };
    for (shape, z) in [
        (block, 1.),
        (Shape::Sphere { radius: 0.1 }, 0.199),
        (block, 0.198),
    ] {
        let (mut native, foot, pad) = scene(shape, z);
        let (mut control, other, _) = scene(shape, z);
        let cuts = segments(&native);
        let mut fine = cuts;
        fine[1].native_steps = 10;
        let expected = control.step_contact_schedule(DT, &fine, &[]).unwrap();
        let mut elapsed = vec![];
        let actual = native
            .step_after_box_release(
                DT,
                &cuts,
                (foot, pad),
                10,
                &[],
                Some(&mut |s| elapsed.push(s.elapsed)),
            )
            .unwrap();
        assert_eq!(actual, expected);
        assert_eq!(native.state(foot), control.state(other));
        assert_eq!(elapsed.len(), 24);
        assert_eq!(elapsed.last(), Some(&DT));
    }
}

#[test]
fn invalid_release_schedules_never_advance_the_world_or_observer() {
    let (mut native, foot, pad) = scene(
        Shape::Box {
            half_extents: [0.1; 3],
        },
        0.2,
    );
    let bad = BodyId::from_raw(99999);
    let before = native.state(foot);
    for (pair, n, ms) in [
        ((pad, pad), 10, 300),
        ((bad, pad), 10, 300),
        ((bad, bad), 10, 300),
        ((pad, foot), 2, 300),
        ((foot, pad), u32::MAX, 300),
        ((foot, pad), 200_000, 300),
        ((foot, pad), 10, 301),
    ] {
        let cuts = segments(&native);
        assert!(
            native
                .step_after_box_release(
                    Duration::from_micros(ms),
                    &cuts,
                    pair,
                    n,
                    &[],
                    Some(&mut |_| panic!("unexpected advancement"))
                )
                .is_err()
        );
        assert_eq!(native.state(foot), before);
    }
}
