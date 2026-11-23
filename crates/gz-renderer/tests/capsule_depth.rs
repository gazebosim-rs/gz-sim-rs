// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "gpu")]
use glam::{DQuat, DVec3};
use gz_render_frame::{WorldPose, camera::Calibration};
use gz_renderer::{
    BoxVisual, CapsuleVisual, CylinderVisual, EllipsoidVisual, PrimitiveSnapshot, PrimitiveVisual,
    RenderError, Renderer, Snapshot, SphereVisual,
};
use std::time::{Duration, Instant};
fn camera() -> WorldPose {
    WorldPose {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
    }
}
fn calibration(near: f64) -> Calibration {
    Calibration::from_horizontal_fov(64, 64, std::f64::consts::FRAC_PI_2, near, 20.).unwrap()
}
fn snapshot(visual: PrimitiveVisual) -> PrimitiveSnapshot {
    PrimitiveSnapshot {
        epoch: 3,
        time: Duration::from_millis(7),
        origin: DVec3::ZERO,
        visuals: vec![visual],
    }
}
fn capsule(radius: f64, length: f64, rotation: DQuat) -> CapsuleVisual {
    CapsuleVisual {
        id: 1,
        pose: WorldPose {
            translation: DVec3::new(4., 0., 0.),
            rotation,
        },
        radius,
        length,
        color: [1., 0., 0.],
    }
}
fn rgb(r: &mut Renderer) -> gz_renderer::Frame {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        match r.request_readback() {
            Ok(()) => break,
            Err(RenderError::NotReady) => {
                r.update().unwrap();
                std::thread::sleep(Duration::from_millis(2));
            }
            Err(e) => panic!("{e:?}"),
        }
        assert!(Instant::now() < deadline);
    }
    loop {
        r.update().unwrap();
        if let Some(frame) = r.take_frame().unwrap() {
            return frame;
        }
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(2));
    }
}
fn depth(r: &mut Renderer) -> gz_renderer::DepthFrame {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        match r.request_depth_readback() {
            Ok(()) => break,
            Err(RenderError::NotReady) => {
                r.update().unwrap();
                std::thread::sleep(Duration::from_millis(2));
            }
            Err(e) => panic!("{e:?}"),
        }
        assert!(Instant::now() < deadline);
    }
    loop {
        r.update().unwrap();
        if let Some(frame) = r.take_depth_frame().unwrap() {
            return frame;
        }
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(2));
    }
}
fn roots(a: f64, b: f64, c: f64) -> Vec<f64> {
    let d = b * b - 4. * a * c;
    if a <= 0. || d < 0. {
        return vec![];
    }
    vec![(-b - d.sqrt()) / (2. * a), (-b + d.sqrt()) / (2. * a)]
}
// Independent analytic union of finite cylinder and its two cap-center spheres.
// The camera ray has +X component one, so the resulting parameter is axial depth.
fn capsule_depth(v: &CapsuleVisual, x: usize, y: usize) -> f64 {
    let o = v.pose.rotation.inverse() * (-v.pose.translation);
    let d = v.pose.rotation.inverse()
        * DVec3::new(
            1.,
            (32. - (x as f64 + 0.5)) / 32.,
            (32. - (y as f64 + 0.5)) / 32.,
        );
    let mut hits = Vec::new();
    for t in roots(
        d.x * d.x + d.y * d.y,
        2. * (o.x * d.x + o.y * d.y),
        o.x * o.x + o.y * o.y - v.radius * v.radius,
    ) {
        if t > 0. && (o.z + t * d.z).abs() <= v.length / 2. {
            hits.push(t);
        }
    }
    for z in [-v.length / 2., v.length / 2.] {
        let c = o - DVec3::new(0., 0., z);
        for t in roots(
            d.length_squared(),
            2. * c.dot(d),
            c.length_squared() - v.radius * v.radius,
        ) {
            if t > 0. {
                hits.push(t);
            }
        }
    }
    hits.into_iter().min_by(f64::total_cmp).unwrap()
}
#[test]
fn capsule_actual_mesh_depth_matches_side_caps_and_rotated_analytic_rays() {
    let mut r = Renderer::new().unwrap();
    for length in [0.5, 2., 3.] {
        for q in [
            DQuat::IDENTITY,
            DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2),
            DQuat::from_rotation_z(0.3) * DQuat::from_rotation_y(0.4),
        ] {
            let c = capsule(0.75, length, q);
            r.set_calibrated_primitive_depth_snapshot(
                &snapshot(PrimitiveVisual::Capsule(c.clone())),
                calibration(0.1),
                camera(),
            )
            .unwrap();
            let frame = depth(&mut r);
            assert_eq!((frame.epoch, frame.time), (3, Duration::from_millis(7)));
            for (x, y) in [(32, 32), (30, 30), (33, 29)] {
                let expected = capsule_depth(&c, x, y);
                let actual = f64::from(frame.metres[y * 64 + x]);
                // 32 sectors/18 latitude rings bound radial sagitta; these interior
                // rays stay away from grazing incidence. Include f32 readback error.
                let bound = 2.
                    * c.radius
                    * ((1. - (std::f64::consts::PI / 18.).cos())
                        + (1. - (std::f64::consts::PI / 32.).cos()))
                    + 0.002;
                assert!(
                    actual.is_finite()
                        && actual >= expected - 0.002
                        && (actual - expected).abs() < bound,
                    "{actual} vs {expected}, {c:?}"
                );
            }
        }
    }
}
#[test]
fn capsule_rgb_length_changes_silhouette_and_invalid_late_visual_preserves_prior_frame() {
    let mut r = Renderer::new().unwrap();
    let mut s = snapshot(PrimitiveVisual::Capsule(capsule(0.5, 0.5, DQuat::IDENTITY)));
    r.set_calibrated_primitive_snapshot(&s, calibration(0.1), camera())
        .unwrap();
    let short = rgb(&mut r);
    s.visuals = vec![PrimitiveVisual::Capsule(capsule(0.5, 3., DQuat::IDENTITY))];
    r.set_calibrated_primitive_snapshot(&s, calibration(0.1), camera())
        .unwrap();
    let long = rgb(&mut r);
    let count = |frame: &gz_renderer::Frame| {
        frame
            .rgba
            .chunks_exact(4)
            .filter(|p| p[0] > 200 && p[1] < 20 && p[2] < 20)
            .count()
    };
    assert!(count(&long) > count(&short) + 10);
    for (radius, length) in [
        (0., 1.),
        (-1., 1.),
        (0.5, 0.),
        (f64::NAN, 1.),
        (0.5, f64::INFINITY),
        (f64::MAX, 1.),
        (1e-300, 1.),
    ] {
        let mut invalid = s.clone();
        let mut c = capsule(radius, length, DQuat::IDENTITY);
        c.id = 2;
        invalid.visuals.push(PrimitiveVisual::Capsule(c));
        assert!(
            r.set_calibrated_primitive_snapshot(&invalid, calibration(0.1), camera())
                .is_err()
        );
        assert_eq!(rgb(&mut r).rgba, long.rgba);
    }
    let mut too_many = s.clone();
    too_many.visuals = vec![s.visuals[0].clone(); 129];
    assert!(matches!(
        r.set_calibrated_primitive_snapshot(&too_many, calibration(0.1), camera()),
        Err(RenderError::InvalidIds)
    ));
    assert_eq!(rgb(&mut r).rgba, long.rgba);
}
fn curved(kind: usize, x: f64) -> PrimitiveVisual {
    let pose = WorldPose {
        translation: DVec3::new(x, 0., 0.),
        rotation: DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4),
    };
    match kind {
        0 => PrimitiveVisual::Sphere(SphereVisual {
            id: 1,
            pose,
            radius: 0.5,
            color: [1., 0., 0.],
        }),
        1 => PrimitiveVisual::Ellipsoid(EllipsoidVisual {
            id: 1,
            pose,
            radii: DVec3::new(0.5, 0.25, 0.25),
            color: [1., 0., 0.],
        }),
        2 => PrimitiveVisual::Cylinder(CylinderVisual {
            id: 1,
            pose,
            radius: 0.5,
            length: 1.,
            color: [1., 0., 0.],
        }),
        _ => PrimitiveVisual::Capsule(CapsuleVisual {
            id: 1,
            pose,
            radius: 0.5,
            length: 1.,
            color: [1., 0., 0.],
        }),
    }
}
#[test]
fn tight_curved_extrema_admit_real_depth_where_enclosing_boxes_cross_near() {
    let mut r = Renderer::new().unwrap();
    for kind in 0..4 {
        let support = if kind == 1 {
            (0.5_f64.powi(2) / 2. + 0.25_f64.powi(2) / 2.).sqrt()
        } else {
            0.5
        };
        let x = 1. + support + 0.05;
        let size = if kind == 1 {
            DVec3::new(1., 0.5, 0.5)
        } else if kind == 3 {
            DVec3::new(1., 1., 2.)
        } else {
            DVec3::ONE
        };
        let bounds = Snapshot {
            epoch: 1,
            time: Duration::ZERO,
            origin: DVec3::ZERO,
            visuals: vec![BoxVisual {
                id: 1,
                pose: WorldPose {
                    translation: DVec3::new(x, 0., 0.),
                    rotation: DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4),
                },
                size,
                color: [1., 0., 0.],
            }],
        };
        assert!(matches!(
            r.set_calibrated_depth_snapshot(&bounds, calibration(1.), camera()),
            Err(RenderError::DepthNearClip)
        ));
        r.set_calibrated_primitive_depth_snapshot(
            &snapshot(curved(kind, x)),
            calibration(1.),
            camera(),
        )
        .unwrap();
        let frame = depth(&mut r);
        assert!(frame.metres.iter().any(|x| x.is_finite()));
        assert!(
            frame
                .metres
                .iter()
                .all(|x| *x == f32::INFINITY || x.is_finite() && *x >= 1.)
        );
        let origin = DVec3::splat(1e9);
        let turn = DQuat::from_rotation_y(0.3) * DQuat::from_rotation_z(0.2);
        let mut moved = snapshot(curved(kind, x));
        moved.origin = origin;
        for visual in &mut moved.visuals {
            let pose = match visual {
                PrimitiveVisual::Box(v) => &mut v.pose,
                PrimitiveVisual::Cylinder(v) => &mut v.pose,
                PrimitiveVisual::Sphere(v) => &mut v.pose,
                PrimitiveVisual::Ellipsoid(v) => &mut v.pose,
                PrimitiveVisual::Capsule(v) => &mut v.pose,
            };
            pose.translation = origin + turn * pose.translation;
            pose.rotation = turn * pose.rotation;
        }
        r.set_calibrated_primitive_depth_snapshot(
            &moved,
            calibration(1.),
            WorldPose {
                translation: origin,
                rotation: turn,
            },
        )
        .unwrap();
        let transformed = depth(&mut r);
        // Interior rays witness continuous depth under the common rigid move;
        // avoid demanding identical raster edge coverage after f32 conversion.
        for index in [32 * 64 + 32, 30 * 64 + 30, 29 * 64 + 33] {
            let (a, b) = (frame.metres[index], transformed.metres[index]);
            assert!(a.is_finite() && b.is_finite() && (a - b).abs() < 1e-4);
        }
        for index in [0, 63, 63 * 64, 64 * 64 - 1] {
            assert_eq!(frame.metres[index], f32::INFINITY);
            assert_eq!(transformed.metres[index], f32::INFINITY);
        }
    }
}
#[test]
fn curved_tangent_intersection_behind_and_invalid_dimensions_refuse_before_state_change() {
    let mut r = Renderer::new().unwrap();
    let old = snapshot(PrimitiveVisual::Capsule(capsule(0.5, 1., DQuat::IDENTITY)));
    r.set_calibrated_primitive_depth_snapshot(&old, calibration(1.), camera())
        .unwrap();
    let expected = depth(&mut r).metres;
    for kind in 0..4 {
        for x in [1.49, 0.5, -2.] {
            let mut s = snapshot(curved(kind, x));
            if kind == 1 {
                s.visuals = vec![PrimitiveVisual::Ellipsoid(EllipsoidVisual {
                    id: 1,
                    pose: WorldPose {
                        translation: DVec3::new(if x == 1.49 { 1.25 } else { x }, 0., 0.),
                        rotation: DQuat::IDENTITY,
                    },
                    radii: DVec3::splat(0.25),
                    color: [1., 0., 0.],
                })];
            }
            assert!(matches!(
                r.set_calibrated_primitive_depth_snapshot(&s, calibration(1.), camera()),
                Err(RenderError::DepthNearClip)
            ));
            assert_eq!(depth(&mut r).metres, expected);
        }
    }
    let tangent = snapshot(PrimitiveVisual::Sphere(SphereVisual {
        id: 1,
        pose: WorldPose {
            translation: DVec3::new(1.5, 0., 0.),
            rotation: DQuat::IDENTITY,
        },
        radius: 0.5,
        color: [1., 0., 0.],
    }));
    assert!(matches!(
        r.set_calibrated_primitive_depth_snapshot(&tangent, calibration(1.), camera()),
        Err(RenderError::DepthNearClip)
    ));
    for value in [0., -1., f64::NAN, f64::INFINITY, 1e-300] {
        let bad = snapshot(PrimitiveVisual::Capsule(capsule(
            value,
            1.,
            DQuat::IDENTITY,
        )));
        assert!(
            r.set_calibrated_primitive_depth_snapshot(&bad, calibration(1.), camera())
                .is_err()
        );
        assert_eq!(depth(&mut r).metres, expected);
    }
}

#[test]
fn capsule_full_visual_bound_dimension_replacement_and_empty_retirement_are_real_frames() {
    let mut renderer = Renderer::new().unwrap();
    let visuals = (1..=128)
        .map(|id| {
            let mut c = capsule(
                0.05 + f64::from(id) / 1000.,
                0.1 + f64::from(id) / 100.,
                DQuat::IDENTITY,
            );
            c.id = id as u64;
            c.pose.translation = DVec3::new(
                5.,
                (f64::from(id % 8) - 3.5) * 0.25,
                (f64::from(id / 8) - 7.5) * 0.1,
            );
            PrimitiveVisual::Capsule(c)
        })
        .collect();
    let mut s = PrimitiveSnapshot {
        epoch: 1,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals,
    };
    renderer
        .set_calibrated_primitive_snapshot(&s, calibration(0.1), camera())
        .unwrap();
    let frame = rgb(&mut renderer);
    assert!(frame.rgba.chunks_exact(4).any(|p| p[0] > 200));
    s.visuals.clear();
    s.time = Duration::from_millis(1);
    renderer
        .set_calibrated_primitive_snapshot(&s, calibration(0.1), camera())
        .unwrap();
    let empty = rgb(&mut renderer);
    // The established empty-scene contract is opaque [43, 44, 47, 255],
    // independent of the visual type that was retired.
    assert!(empty.rgba.chunks_exact(4).all(|p| p == [43, 44, 47, 255]));
    s.visuals
        .push(PrimitiveVisual::Capsule(capsule(0.75, 2., DQuat::IDENTITY)));
    s.time = Duration::from_millis(2);
    renderer
        .set_calibrated_primitive_depth_snapshot(&s, calibration(0.1), camera())
        .unwrap();
    assert!(depth(&mut renderer).metres[32 * 64 + 32].is_finite());
}
