// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify that the native principal frame reconstructs the authored link-frame inertia.
use super::*;

fn reconstructs(properties: &MassProperties, target: Matrix) -> bool {
    let inverse = properties.inv_principal_inertia;
    if !inverse.is_finite() || inverse.min_element() <= 0. {
        return false;
    }
    let principal = Matrix::from_quat(properties.principal_inertia_local_frame);
    let reconstructed = principal * Matrix::from_diagonal(inverse.recip()) * principal.transpose();
    let scale = target
        .to_cols_array()
        .into_iter()
        .map(f64::abs)
        .fold(0., f64::max);
    let tolerance = 512. * f64::EPSILON * scale;
    reconstructed
        .to_cols_array()
        .into_iter()
        .zip(target.to_cols_array())
        .all(|(a, b)| (a - b).abs() <= tolerance)
}
pub(super) fn properties(
    local_com: Vector,
    mass: f64,
    frame: Rotation,
    tensor: Matrix,
) -> Result<MassProperties, PhysicsError> {
    let matrix = Matrix::from_quat(frame);
    let target = matrix * tensor * matrix.transpose();
    let diagonal = Vector::new(tensor.x_axis.x, tensor.y_axis.y, tensor.z_axis.z);
    let repeated = diagonal.x == diagonal.y || diagonal.x == diagonal.z || diagonal.y == diagonal.z;
    // A repeated moment does not determine a unique principal frame. Respect
    // explicitly authored principal axes when present, including for soft
    // receivers which integrate the orientation of that frame at substeps.
    if frame != Rotation::IDENTITY && repeated && tensor == Matrix::from_diagonal(diagonal) {
        let declared =
            MassProperties::with_principal_inertia_frame(local_com, mass, diagonal, frame);
        return reconstructs(&declared, target)
            .then_some(declared)
            .ok_or(PhysicsError::InvalidValue);
    }
    let ordinary = MassProperties::with_inertia_matrix(local_com, mass, target);
    if reconstructs(&ordinary, target) {
        return Ok(ordinary);
    }
    // Near repeated principal moments, the native symmetric decomposition can
    // yield a non-orthogonal eigenbasis before quaternion normalization. Respect
    // explicit axes when supplied, and use scaled symmetric QR otherwise.
    let (moments, axes) = if tensor == Matrix::from_diagonal(diagonal) {
        (diagonal, frame)
    } else {
        let scale = tensor
            .to_cols_array()
            .into_iter()
            .map(f64::abs)
            .fold(0., f64::max);
        if !scale.is_finite() || scale == 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let m = rapier3d_f64::na::Matrix3::from_column_slice(&(tensor / scale).to_cols_array());
        let eig = rapier3d_f64::na::SymmetricEigen::try_new(m, 8. * f64::EPSILON, 64)
            .ok_or(PhysicsError::InvalidValue)?;
        let moments =
            Vector::new(eig.eigenvalues[0], eig.eigenvalues[1], eig.eigenvalues[2]) * scale;
        let col = |i| {
            Vector::new(
                eig.eigenvectors[(0, i)],
                eig.eigenvectors[(1, i)],
                eig.eigenvectors[(2, i)],
            )
        };
        let mut basis = Matrix::from_cols(col(0), col(1), col(2));
        if basis.determinant() < 0. {
            basis.z_axis = -basis.z_axis;
        }
        (moments, (frame * Rotation::from_mat3(&basis)).normalize())
    };
    if !moments.is_finite() || moments.min_element() <= 0. {
        return Err(PhysicsError::InvalidValue);
    }
    let result = MassProperties::with_principal_inertia_frame(local_com, mass, moments, axes);
    reconstructs(&result, target)
        .then_some(result)
        .ok_or(PhysicsError::InvalidValue)
}
