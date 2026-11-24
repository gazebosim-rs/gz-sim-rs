// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/oriented_ground.rs"]
mod ground;
#[test]
fn ordinary_oriented_halfspace_matches_inertial_ramp_and_warm_recovery() {
    ground::run(std::path::Path::new(env!("CARGO_BIN_EXE_physics_worker"))).unwrap();
}

#[test]
fn portable_plane_identity_and_oblique_support_are_checked_before_insertion() {
    use gz_physics_distributed::physics::{GroundPlane, Island, SimulationStep, Snapshot, fixture};
    let n = [0.6, 0., 0.8];
    let p = [3., -0., -2.];
    let g = GroundPlane::oriented(p, n, -0.).unwrap();
    let image = bincode::serialize(&(g, 17u32)).unwrap();
    assert_eq!(image.len(), 68);
    assert_eq!(
        bincode::deserialize::<(GroundPlane, u32)>(&image).unwrap(),
        (g, 17)
    );
    assert_eq!(
        serde_json::from_slice::<GroundPlane>(&serde_json::to_vec(&g).unwrap()).unwrap(),
        g
    );
    assert_ne!(g, GroundPlane::oriented([3., 0., -2.], n, -0.).unwrap());
    for normal in [[0.; 3], [1., 1., 0.], [f64::NAN, 0., 1.]] {
        assert!(GroundPlane::oriented(p, normal, 0.).is_err());
    }
    assert!(GroundPlane::oriented([f64::INFINITY, 0., 0.], n, 0.).is_err());
    assert!(GroundPlane::oriented(p, n, -1.).is_err());
    let mut seed = Snapshot {
        step: SimulationStep::Ms5,
        ground: Some(g),
        ..Default::default()
    };
    // A long box lies parallel to the inclined plane. Its eight corners
    // clear the plane by 1 mm, although its world AABB penetrates deeply.
    let body = fixture()[0].promoted();
    let theta = 0.6_f64.asin();
    for (gap, ok) in [(0.001, true), (-0.001, false), (0., false)] {
        let mut b = body.clone();
        b.size = [8., 0.4, 0.2];
        b.orientation = [0., (theta / 2.).sin(), 0., (theta / 2.).cos()];
        b.position = std::array::from_fn(|i| p[i] + (0.1 + gap) * n[i]);
        b.linear = [0.; 3];
        seed.bodies = vec![b];
        assert_eq!(Island::restore(&seed).is_ok(), ok, "gap {gap}");
    }
}

#[test]
fn swept_wall_proximity_admits_ground_deflection_toward_a_different_worker() {
    use gz_physics_distributed::physics::{
        ComponentInfo, GroundPlane, Snapshot, fixture, partitions_may_interact,
    };
    let mut a = fixture()[0].promoted();
    a.position = [0.4, 0., 50.];
    a.linear = [-30., 0., 0.];
    let mut b = fixture()[1].clone();
    b.position = [1.65, 0., 50.];
    b.linear = [0.; 3];
    let comps = |id| {
        [ComponentInfo {
            ids: vec![id],
            contact_history: false,
        }]
    };
    let (ca, cb) = (comps(a.id), comps(b.id));
    let mut left = Snapshot {
        bodies: vec![a],
        ..Default::default()
    };
    let mut right = Snapshot {
        bodies: vec![b],
        ..Default::default()
    };
    assert!(!partitions_may_interact(&left, &ca, &right, &cb, &[]));
    let wall = GroundPlane::oriented([0.; 3], [1., 0., 0.], 0.).unwrap();
    left.ground = Some(wall);
    right.ground = Some(wall);
    assert!(partitions_may_interact(&left, &ca, &right, &cb, &[]));
}

#[test]
fn a_sphere_loads_vertical_and_inverted_halfspaces_and_restores_at_contact() {
    use gz_physics::{BodySpec, DynamicBackend, Shape};
    use gz_physics_multifidelity::MultiFidelityBackend;
    use std::time::Duration;
    let p = [-0.75, 0.83, 1.3];
    let dt = Duration::from_millis(2);
    for n in [
        [1., 0., 0.],
        [-1., 0., 0.],
        [0., 1., 0.],
        [0., 0., -1.],
        [0., -0.8, -0.6],
    ] {
        let mut native = MultiFidelityBackend::new([0.; 3]).unwrap();
        native.set_partition_step(dt).unwrap();
        native.set_partition_plane(p, n, 0.6).unwrap();
        assert!(native.partition_ground_height().is_err());
        let s = native
            .spawn_partition_primitive(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.2 },
                    position: std::array::from_fn(|i| p[i] + 0.4 * n[i]),
                    velocity: [0.; 3],
                    mass: Some(2.),
                },
                Default::default(),
                false,
                0.6,
            )
            .unwrap();
        let initial = native.initial_state();
        let mut load = 0.;
        for tick in 1..=300 {
            native.set_partition_gravity(n.map(|x| -9.81 * x)).unwrap();
            native.step(dt).unwrap();
            native.set_partition_gravity([0.; 3]).unwrap();
            if tick > 200 {
                let j = native.partition_ground_contacts();
                assert_eq!(j[0].body, s);
                load += j[0].normal_impulse;
            }
            if tick == 190 {
                let image = native.checkpoint_solver().unwrap();
                native = MultiFidelityBackend::restore_trusted_solver(&image).unwrap();
                assert_eq!(native.partition_plane().unwrap().unwrap().normal, n);
            }
        }
        assert!((load / 0.2 - 19.62).abs() < 0.01, "{n:?} {load:e}");
        native.reset().unwrap();
        assert_eq!(native.partition_plane().unwrap().unwrap().normal, n);
        // Initial snapshots are bound to a live solver, including its immutable terrain.
        assert!(native.prepare_initial(&initial).is_err());
    }
}

#[test]
fn late_ground_admission_preserves_loaded_bodies_and_uses_the_actual_link_pose() {
    use gz_physics::{BodySpec, DynamicBackend, PlaneSpec, RotationSpec, Shape};
    use gz_physics_distributed::{
        SceneBackend,
        physics::{GroundPlane, STEP},
    };
    let mut scene = SceneBackend::new(
        std::path::Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        [0.; 3],
        STEP,
    )
    .unwrap();
    let body = scene
        .spawn_link(
            BodySpec {
                mass: Some(1.),
                position: [2., 0., 0.],
                velocity: [0.; 3],
                shape: Shape::Sphere { radius: 0.2 },
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let old = scene.state(body).unwrap();
    let half = std::f64::consts::FRAC_1_SQRT_2;
    let plane = PlaneSpec {
        position: [0.; 3],
        orientation: [0., half, 0., half],
        size: [1., 1.],
    };
    let invalid = GroundPlane::oriented([3., 0., 0.], [1., 0., 0.], 0.).unwrap();
    assert!(scene.spawn_ground_geometry(plane, invalid).is_err());
    for bad in [
        PlaneSpec {
            orientation: [0.; 4],
            ..plane
        },
        PlaneSpec {
            position: [f64::NAN; 3],
            ..plane
        },
        PlaneSpec {
            size: [0., 1.],
            ..plane
        },
    ] {
        assert!(scene.spawn_ground_surface(bad, 0.).is_err());
    }
    let ground = scene.spawn_static_plane(plane).unwrap();
    assert!(scene.is_shared_ground(ground));
    assert!(!scene.is_shared_ground(body));
    assert_eq!(scene.state(body).unwrap(), old);
    assert_eq!(scene.state(ground).unwrap().orientation, plane.orientation);
    let distance = scene
        .cast_ray([1., 0., 10.], [-1., 0., 0.], 2., None)
        .unwrap()
        .unwrap();
    assert!((distance - 1.).abs() < 1e-13);
    assert!(scene.spawn_static_plane(plane).is_err());
    scene.initialize().unwrap();
    assert!(scene.spawn_ground_surface(plane, 0.7).is_err());
}

#[test]
fn malformed_oblique_authoring_and_truncated_archives_do_not_alias_a_legacy_plane() {
    use gz_physics_distributed::physics::GroundPlane;
    for text in [
        r#"{"point":[0,0,1],"normal":[0,0,0],"friction":0.4}"#,
        r#"{"point":[0,2],"normal":[1,0,0],"friction":0.4}"#,
        r#"{"point":[0,0,0],"normal":[0,1,0]}"#,
        r#"{"point":[0,1,0],"normal":[0,1,0],"friction":0.4,"height":1}"#,
    ] {
        assert!(serde_json::from_str::<GroundPlane>(text).is_err());
    }
    let bytes =
        bincode::serialize(&GroundPlane::oriented([1., 2., 3.], [0.6, 0., 0.8], 0.25).unwrap())
            .unwrap();
    for len in 0..bytes.len() {
        assert!(bincode::deserialize::<GroundPlane>(&bytes[..len]).is_err());
    }
}
