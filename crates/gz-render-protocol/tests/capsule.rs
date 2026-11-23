// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{
    Camera, Geometry, Request, ShapeVisual, Stamp, read_request, write_request,
};
fn request(depth: bool) -> Request {
    let camera = Box::new(Camera {
        width: 64,
        height: 64,
        fx: 32.,
        fy: 32.,
        cx: 32.,
        cy: 32.,
        skew: 0.,
        tx: 0.,
        ty: 0.,
        near: 0.1,
        far: 20.,
        translation: [0.; 3],
        rotation: [0., 0., 0., 1.],
    });
    let visuals = vec![ShapeVisual {
        id: 1,
        translation: [4., 0., 0.],
        rotation: [0., 0., 0., 1.],
        geometry: Geometry::Capsule {
            radius: 0.5,
            length: 2.,
        },
        color: [1., 0., 0.],
    }];
    let stamp = Stamp {
        epoch: 2,
        seconds: 3,
        nanoseconds: 4,
    };
    if depth {
        Request::CalibratedDepthShapeSnapshot {
            request_id: 7,
            stamp,
            origin: [0.; 3],
            visuals,
            camera,
        }
    } else {
        Request::CalibratedShapeSnapshot {
            request_id: 7,
            stamp,
            origin: [0.; 3],
            visuals,
            camera,
        }
    }
}
#[test]
fn capsule_wire_geometry_round_trips_segment_length_without_changing_old_variants() {
    for depth in [false, true] {
        let r = request(depth);
        let mut bytes = Vec::new();
        write_request(&mut bytes, &r).unwrap();
        assert_eq!(read_request(&mut bytes.as_slice()).unwrap(), Some(r));
    }
    assert_ne!(
        Geometry::Capsule {
            radius: 0.5,
            length: 2.
        },
        Geometry::Cylinder {
            radius: 0.5,
            length: 2.
        }
    );
}
#[test]
fn capsule_wire_nonfinite_components_and_global_visual_cap_remain_closed() {
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut r = request(false);
        let Request::CalibratedShapeSnapshot { visuals, .. } = &mut r else {
            unreachable!()
        };
        visuals[0].geometry = Geometry::Capsule {
            radius: value,
            length: 1.,
        };
        assert!(write_request(&mut Vec::new(), &r).is_err());
    }
    let mut r = request(true);
    let Request::CalibratedDepthShapeSnapshot { visuals, .. } = &mut r else {
        unreachable!()
    };
    *visuals = vec![visuals[0].clone(); 129];
    assert!(write_request(&mut Vec::new(), &r).is_err());
}
