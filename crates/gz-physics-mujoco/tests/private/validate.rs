// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn principal_moments_match_known_spectra() {
    assert_eq!(
        principal_axes([1.0, 2.0, 3.0, 0.0, 0.0, 0.0]),
        ([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0])
    );
    let (mut moments, _) = principal_axes([2.0, 2.0, 3.0, 1.0, 0.0, 0.0]);
    moments.sort_by(f64::total_cmp);
    for (actual, expected) in moments.into_iter().zip([1.0, 3.0, 3.0]) {
        assert!((actual - expected).abs() < 1e-14, "{moments:?}");
    }
    let tensor = [4.0, 5.0, 6.0, 0.5, 0.25, -0.75];
    let (moments, [w, x, y, z]) = principal_axes(tensor);
    // The frame diagonalizes the tensor: R diag R^T reproduces it.
    let rebuilt = rotated_tensor(Inertia {
        center: [0.0; 3],
        orientation: [x, y, z, w],
        tensor: [moments[0], moments[1], moments[2], 0.0, 0.0, 0.0],
    });
    for (actual, expected) in rebuilt.into_iter().zip(tensor) {
        assert!((actual - expected).abs() < 1e-12, "{rebuilt:?}");
    }
    // Negligible coupling leaves the diagonal moments unchanged.
    let (nearly, _) = principal_axes([1.0, 2.0, 3.0, 1e-300, 0.0, 0.0]);
    assert_eq!(nearly, [1.0, 2.0, 3.0]);
    // Invariants: trace and determinant of the tensor.
    assert!((moments.iter().sum::<f64>() - 15.0).abs() < 1e-12);
    let det = 4.0 * (30.0 - 0.5625) - 0.5 * (3.0 + 0.1875) + 0.25 * (-0.375 - 1.25);
    assert!((moments.iter().product::<f64>() - det).abs() < 1e-10);
}

#[test]
fn half_turn_matrices_convert_to_quaternions_on_every_branch() {
    for (diagonal, expected) in [
        ([1.0, -1.0, -1.0], [0.0, 1.0, 0.0, 0.0]),
        ([-1.0, 1.0, -1.0], [0.0, 0.0, 1.0, 0.0]),
        ([-1.0, -1.0, 1.0], [0.0, 0.0, 0.0, 1.0]),
        ([1.0, 1.0, 1.0], [1.0, 0.0, 0.0, 0.0]),
    ] {
        let [a, b, c] = diagonal;
        assert_eq!(
            matrix_quaternion([[a, 0.0, 0.0], [0.0, b, 0.0], [0.0, 0.0, c]]),
            expected
        );
    }
}

#[test]
fn rotations_and_matrix_helpers_use_xyzw_and_row_major_conventions() {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let z90 = [0.0, 0.0, s, s];
    let rotated = quaternion_rotate(z90, [1.0, 0.0, 0.0]);
    assert!((rotated[1] - 1.0).abs() < 1e-15 && rotated[0].abs() < 1e-15);
    let back = quaternion_rotate_inverse(z90, rotated);
    assert!((back[0] - 1.0).abs() < 1e-15);
    let unit = normalized([3.0, 0.0, 4.0]).unwrap();
    assert!((unit[0] - 0.6).abs() < 1e-15 && (unit[2] - 0.8).abs() < 1e-15);
    assert_eq!(normalized([0.0; 3]), None);
    // The inertial orientation rotates the tensor into body axes.
    let tensor = rotated_tensor(Inertia {
        center: [0.0; 3],
        orientation: z90,
        tensor: [2.0, 3.0, 4.0, 0.0, 0.0, 0.0],
    });
    for (actual, expected) in tensor.into_iter().zip([3.0, 2.0, 4.0, 0.0, 0.0, 0.0]) {
        assert!((actual - expected).abs() < 1e-14, "{tensor:?}");
    }
}

#[test]
fn lowering_keeps_construction_values_and_mass_source() {
    let spec = BodySpec {
        shape: Shape::Box {
            half_extents: [1.0, 2.0, 3.0],
        },
        position: [1.0, 2.0, 3.0],
        velocity: [0.0; 3],
        mass: Some(2.0),
    };
    let rotation = RotationSpec {
        orientation: [0.0, 1.0, 0.0, 0.0],
        angular_velocity: [0.0; 3],
    };
    let inferred = body_model(
        spec,
        rotation,
        spawn(spec, rotation, None).unwrap(),
        BodyJoint::Free,
    );
    assert_eq!(inferred.quaternion, [0.0, 0.0, 1.0, 0.0]);
    assert_eq!(inferred.geometry, Geometry::Box([1.0, 2.0, 3.0]));
    assert_eq!(
        inferred.inertial,
        Inertial {
            mass: 2.0,
            center: [0.0; 3],
            principal: shape_moments(spec.shape, 2.0),
            quaternion: [1.0, 0.0, 0.0, 0.0],
        }
    );
    let inertia = Inertia {
        center: [0.1, 0.0, 0.0],
        orientation: [0.0, 0.0, 0.0, 1.0],
        tensor: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
    };
    let lowered = spawn(spec, rotation, Some(inertia)).unwrap();
    let explicit = body_model(spec, rotation, lowered, BodyJoint::Free);
    assert_eq!(
        explicit.inertial,
        Inertial {
            mass: 2.0,
            center: [0.1, 0.0, 0.0],
            principal: [1.0; 3],
            quaternion: [1.0, 0.0, 0.0, 0.0],
        }
    );
    let fixed_spec = BodySpec { mass: None, ..spec };
    assert_eq!(
        spawn(fixed_spec, rotation, Some(inertia))
            .unwrap()
            .principal,
        [1.0; 3]
    );
    // Geom-inferred mass needs a volume MuJoCo will assign it to.
    let tiny = BodySpec {
        shape: Shape::Sphere { radius: 1e-5 },
        ..spec
    };
    assert_eq!(
        spawn(tiny, RotationSpec::default(), None),
        Err(PhysicsError::InvalidValue)
    );
    assert!(spawn(tiny, RotationSpec::default(), Some(inertia)).is_ok());
    // Fixed bodies skip MuJoCo's moving-body minimums but not physical validity.
    let fixed = BodySpec { mass: None, ..spec };
    for tensor in [
        [-1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        [10.0, 1.0, 1.0, 0.0, 0.0, 0.0],
    ] {
        assert_eq!(
            spawn(
                fixed,
                RotationSpec::default(),
                Some(Inertia { tensor, ..inertia })
            ),
            Err(PhysicsError::InvalidValue)
        );
    }
    assert_eq!(
        spawn(
            fixed,
            RotationSpec::default(),
            Some(Inertia {
                tensor: [1e-20, 1e-20, 1e-20, 0.0, 0.0, 0.0],
                ..inertia
            })
        )
        .map(|inertial| inertial.principal),
        Ok([1.0; 3])
    );
    // A moving body beyond mjMAXVAL or with a center beyond the conversion bound.
    for body in [
        BodySpec {
            position: [1.1e10, 0.0, 0.0],
            ..spec
        },
        BodySpec { ..spec },
    ] {
        let center = if body.position[0] > 1e10 { 0.0 } else { 2e100 };
        assert_eq!(
            spawn(
                body,
                RotationSpec::default(),
                Some(Inertia {
                    center: [center, 0.0, 0.0],
                    ..inertia
                })
            ),
            Err(PhysicsError::InvalidValue)
        );
    }
}
