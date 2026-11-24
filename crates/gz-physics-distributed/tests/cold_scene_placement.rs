// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, RotationSpec, Shape};
use gz_physics_distributed::{SceneBackend, physics::STEP};
use std::path::Path;

fn scene(width: u8, contact: bool) -> SceneBackend {
    let scene = SceneBackend::new(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        [0.; 3],
        STEP,
    )
    .unwrap()
    .with_worker_count(width)
    .unwrap();
    if contact {
        scene.with_contact_placement().unwrap()
    } else {
        scene
    }
}

fn cube(x: f64) -> BodySpec {
    BodySpec {
        position: [x, 0., 0.],
        velocity: [0.; 3],
        mass: Some(1.),
        shape: Shape::Box {
            half_extents: [0.3; 3],
        },
    }
}

fn owners(scene: &SceneBackend, count: u64) -> Vec<u8> {
    (0..count)
        .map(|id| {
            if let Some(pool) = scene.pool_frame() {
                pool.body(id).unwrap().0
            } else {
                scene.frame().unwrap().body(id).unwrap().0
            }
        })
        .collect()
}

#[test]
fn crossing_command_votes_for_a_common_owner_and_reset_reuses_unforced_geometry() {
    for width in 2..=8 {
        let mut placed = scene(width, true);
        let mut spatial = scene(width, false);
        // Tied y/z coordinates, unequal remainders and two very close bodies
        // straddling the spatial median. Only the queued force closes their gap.
        let xs = [-60., -40., -20., -2., 2., 20., 40., 60.];
        for engine in [&mut placed, &mut spatial] {
            for x in xs {
                engine
                    .spawn_link(cube(x), RotationSpec::default(), None)
                    .unwrap();
            }
            engine
                .add_world_wrench(BodyId::from_raw(3), [4.0e4, 0., 0.], [0.; 3])
                .unwrap();
            engine.initialize().unwrap();
        }
        let contact = owners(&placed, 8);
        assert_eq!(contact[3], contact[4], "held wrench at width {width}");
        placed.step(STEP).unwrap();
        spatial.step(STEP).unwrap();
        for i in 0..8 {
            let id = BodyId::from_raw(i);
            assert_eq!(
                placed.state(id),
                spatial.state(id),
                "body {i}, width {width}"
            );
        }
        placed.reset().unwrap();
        let fresh = owners(&placed, 8);
        assert_ne!(
            fresh[3], fresh[4],
            "reset discards held wrench, width {width}"
        );
        placed
            .set_world_velocity(BodyId::from_raw(3), [380., 0., 0.], [0.; 3])
            .unwrap();
        placed.step(STEP).unwrap();
        let after_edit = owners(&placed, 8);
        assert_eq!(after_edit[3], after_edit[4], "live edit at width {width}");
    }
}

#[test]
fn contact_placement_preserves_overlap_and_native_component_capacity_admission() {
    for width in [2, 4, 8] {
        let mut placed = scene(width, true);
        for i in 0..33 {
            let spec = cube(i as f64 * 0.70);
            assert_eq!(
                placed
                    .spawn_link(spec, RotationSpec::default(), None)
                    .unwrap()
                    .raw(),
                i
            );
            assert!(
                placed
                    .spawn_link(spec, RotationSpec::default(), None)
                    .is_err()
            );
        }
        assert!(placed.initialize().is_err());
        assert!(placed.frame().is_none() && placed.pool_frame().is_none());
        assert!(placed.remove(BodyId::from_raw(32)));
        placed.initialize().unwrap();
        assert!(owners(&placed, 32).iter().all(|w| *w == 0));
        placed.step(STEP).unwrap();
    }
}
