// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Stack-only clearance probes for a robot tool, in a committed world pose.
use gz_physics_rapier::{PhysicsError, SolidRay};

const I: [f64; 4] = [0., 0., 0., 1.];
const X: [f64; 3] = [1., 0., 0.];
const Z: [f64; 3] = [0., 0., 1.];

fn close(ray: &SolidRay, expected: f64) {
    let actual = ray.distance().expect("expected a clearance stop");
    assert!((actual - expected).abs() < 1e-10, "{actual} != {expected}");
}

#[test]
fn tool_clearance_uses_nearest_valid_surface_independent_of_insertion_order()
-> Result<(), PhysicsError> {
    for forward in [true, false] {
        let mut ray = SolidRay::new([0.; 3], X, 5.)?;
        for i in 0..5 {
            match if forward { i } else { 4 - i } {
                0 => ray.include_halfspace([3., 0., 0.], [-1., 0., 0.])?,
                1 => ray.include_box([2., 0.4, 0.5], [2.5, 0., 0.], I)?,
                2 => ray.include_sphere(0.4, [2.1, 0., 0.], I)?,
                3 => ray.include_cylinder(0.25, 0.7, [2.8, 0., 0.], I)?,
                _ => ray.include_capsule(0.15, 0.5, [2.2, 0., 0.], I)?,
            }
        }
        close(&ray, 1.5);
    }
    let mut down = SolidRay::new([0., 0., 0.75], [0., 0., -1.], 0.5)?;
    down.include_horizontal_ground(0.25)?;
    close(&down, 0.5);
    down.include_sphere(0.1, [0., 0., 0.75], I)?;
    close(&down, 0.);
    Ok(())
}

#[test]
fn a_long_tool_ray_resolves_a_cylinder_graze_and_rotated_end_caps() -> Result<(), PhysicsError> {
    let q = std::f64::consts::FRAC_1_SQRT_2;
    for (y, expected) in [
        (1., Some(1000.)),
        (1.000001, None),
        (
            0.999999,
            Some(1000. - (1_f64 - 0.999999_f64.powi(2)).sqrt()),
        ),
    ] {
        let mut ray = SolidRay::new([-1000., y, 0.], X, 1001.)?;
        ray.include_cylinder(1., 0.15, [0.; 3], I)?;
        if let Some(expected) = expected {
            close(&ray, expected);
        } else {
            assert_eq!(ray.distance(), None);
        }
    }
    let mut cap = SolidRay::new([-3., 0., 0.], X, 3.)?;
    cap.include_cylinder(0.25, 0.7, [0.; 3], [0., q, 0., q])?;
    close(&cap, 2.3);
    let mut shell = SolidRay::new([0., 0., -3.], Z, 4.)?;
    shell.include_capsule(0.25, 0.7, [0.; 3], [0., q, 0., q])?;
    close(&shell, 2.75);
    let mut outside = SolidRay::new([0., 1., -3.], Z, 4.)?;
    outside.include_cylinder(0.25, 0.7, [0.; 3], I)?;
    assert_eq!(outside.distance(), None);
    Ok(())
}

#[test]
fn rejected_tool_geometry_keeps_a_previous_valid_clearance() -> Result<(), PhysicsError> {
    let mut ray = SolidRay::new([0.; 3], X, 4.)?;
    ray.include_sphere(1., [3., 0., 0.], I)?;
    for r in [
        ray.include_sphere(-1., [0.; 3], I),
        ray.include_box([1., 0., 2.], [0.; 3], I),
        ray.include_cylinder(0., 1., [0.; 3], I),
        ray.include_cylinder(1., f64::NAN, [0.; 3], I),
        ray.include_capsule(f64::NAN, 1., [0.; 3], I),
        ray.include_capsule(1., 0., [0.; 3], I),
        ray.include_halfspace([0.; 3], [2., 0., 0.]),
        ray.include_horizontal_ground(f64::INFINITY),
        ray.include_box([1.; 3], [0.; 3], [0.; 4]),
        ray.include_sphere(1., [0.; 3], [0.; 4]),
        ray.include_cylinder(1., 1., [0.; 3], [0.; 4]),
        ray.include_capsule(1., 1., [0.; 3], [0.; 4]),
        ray.include_halfspace([f64::NAN, 0., 0.], Z),
    ] {
        assert_eq!(r, Err(PhysicsError::InvalidValue));
    }
    close(&ray, 2.);
    Ok(())
}
