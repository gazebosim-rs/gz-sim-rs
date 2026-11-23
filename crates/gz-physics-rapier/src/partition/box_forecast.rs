// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use rapier3d_f64::parry::{query, shape::Cuboid};
use std::time::Duration;

/// Forecast contact over one interval for two centered boxes with constant world
/// twists. Sizes are full side lengths in metres. A pair already within Rapier's
/// prediction distance also qualifies. A cast that reports an uncertain hit
/// qualifies for refinement. External loads and subsequent impacts may change the path.
pub fn predicts_box_contact(
    sizes: [[f64; 3]; 2],
    states: [BodyState; 2],
    horizon: Duration,
) -> Result<bool, PhysicsError> {
    let dt = horizon.as_secs_f64();
    let body = |i: usize| {
        let s = &states[i];
        let linear = Vector::from(s.velocity);
        let angular = Vector::from(s.angular_velocity);
        let half = Vector::from(sizes[i]) * 0.5;
        let position = Vector::from(s.position);
        if !half.length_squared().is_finite()
            || half.min_element() <= 0.
            || !angular.length_squared().is_finite()
            || !position.is_finite()
            || !(position + linear * dt).is_finite()
            || !(angular * dt).length_squared().is_finite()
            || !linear.length_squared().is_finite()
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok((
            Pose::from_parts(position, rotation(s.orientation)?),
            Cuboid::new(half),
            linear,
            angular,
        ))
    };
    let (a, cuboid_a, va, wa) = body(0)?;
    let (b, cuboid_b, vb, wb) = body(1)?;
    let delta = b.translation - a.translation;
    let travel = (vb - va) * dt;
    if !delta.length_squared().is_finite() || !travel.length_squared().is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    // A rotating centered box stays in its circumsphere. If the relative
    // center segment stays outside the combined radii on one axis, even a
    // grazing hit is impossible. Leave rounding-scale separations to Parry.
    let radius = cuboid_a.half_extents.length() + cuboid_b.half_extents.length();
    let scale = a
        .translation
        .abs()
        .max(b.translation.abs())
        .max(travel.abs())
        .max_element();
    let limit =
        radius + partition_prediction_distance() + 64. * f64::EPSILON * scale.max(radius).max(1.);
    let end = delta + travel;
    if delta.min(end).max_element() > limit || delta.max(end).min_element() < -limit {
        return Ok(false);
    }
    let err = |_| PhysicsError::InvalidValue;
    let gap = query::distance(&a, &cuboid_a, &b, &cuboid_b)
        .map_err(err)?
        .distance;
    if !gap.is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    if gap <= partition_prediction_distance() {
        return Ok(true);
    }
    if horizon.is_zero() {
        return Ok(false);
    }
    let hit = if wa == Vector::ZERO && wb == Vector::ZERO {
        query::cast_shapes(
            &a,
            va,
            &cuboid_a,
            &b,
            vb,
            &cuboid_b,
            query::ShapeCastOptions::with_max_time_of_impact(dt),
        )
    } else {
        let ma = query::NonlinearRigidMotion::new(a, Vector::ZERO, va, wa);
        let mb = query::NonlinearRigidMotion::new(b, Vector::ZERO, vb, wb);
        query::cast_shapes_nonlinear(&ma, &cuboid_a, &mb, &cuboid_b, 0., dt, true)
    };
    if hit.map_err(err)?.is_some() {
        return Ok(true);
    }

    // A nonlinear cast can miss a grazing tip that enters and exits between
    // its search samples. Only skip refinement after proving separation.
    let sweep = Sweep {
        orientations: [a.rotation, b.rotation],
        half: [cuboid_a.half_extents, cuboid_b.half_extents],
        spins: [wa, wb],
        radii: [
            cuboid_a.half_extents.length(),
            cuboid_b.half_extents.length(),
        ],
        delta,
        relative_velocity: vb - va,
        roundoff: 64. * f64::EPSILON * scale.max(radius).max(1.),
    };
    Ok(sweep.might_overlap(dt * 0.5, dt * 0.5, 6))
}

struct Sweep {
    orientations: [Rotation; 2],
    half: [Vector; 2],
    spins: [Vector; 2],
    radii: [f64; 2],
    delta: Vector,
    relative_velocity: Vector,
    roundoff: f64,
}

impl Sweep {
    fn might_overlap(&self, mid: f64, span: f64, levels: u8) -> bool {
        let axes = std::array::from_fn::<_, 2, _>(|i| {
            let rotation = Rotation::from_scaled_axis(self.spins[i] * mid) * self.orientations[i];
            [Vector::X, Vector::Y, Vector::Z].map(|a| rotation * a)
        });
        let offset = self.delta + self.relative_velocity * mid;
        let drift = self.relative_velocity * span;
        // Every point moves by at most r * angle (and at most one diameter)
        // relative to its midpoint orientation within this subinterval.
        let shell = self.roundoff
            + (0..2)
                .map(|i| self.radii[i] * (self.spins[i].length() * span).min(2.))
                .sum::<f64>();
        let separated = |axis: Vector| {
            let length = axis.length();
            if length < 1e-7 {
                return false;
            }
            let radius = (0..2)
                .map(|i| {
                    let projected = Vector::from_array(axes[i].map(|a| a.dot(axis).abs()));
                    projected.dot(self.half[i])
                })
                .sum::<f64>();
            offset.dot(axis).abs() > radius + drift.dot(axis).abs() + length * shell
        };
        if axes.into_iter().flatten().any(separated)
            || axes[0]
                .iter()
                .any(|&a| axes[1].iter().any(|&b| separated(a.cross(b))))
        {
            return false;
        }
        if levels == 0 {
            return true;
        }
        let half = span * 0.5;
        self.might_overlap(mid - half, half, levels - 1)
            || self.might_overlap(mid + half, half, levels - 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stationary(x: f64) -> BodyState {
        BodyState {
            position: [x, 0., 0.],
            orientation: [0., 0., 0., 1.],
            velocity: [0.; 3],
            angular_velocity: [0.; 3],
        }
    }

    #[test]
    fn moving_pair_respects_horizon_and_native_contact_skin() {
        let a = stationary(0.);
        let mut b = stationary(1.03);
        b.velocity[0] = -4.;
        let query =
            |b, ms| predicts_box_contact([[1.; 3]; 2], [a, b], Duration::from_millis(ms)).unwrap();
        assert!(!query(b, 0));
        assert!(!query(b, 5));
        assert!(query(b, 10));
        b.velocity[0] = 4.;
        assert!(!query(b, 10));
        b.position[0] = 1. + 0.5 * partition_prediction_distance();
        assert!(query(b, 0));
    }

    #[test]
    fn swept_bounds_preserve_hits_in_both_directions_and_at_large_offsets() {
        for center in [-1e7, 0., 1e7] {
            for sign in [-1., 1.] {
                let a = stationary(center);
                let mut b = stationary(center + sign * 4.);
                b.velocity[0] = -sign * 400.;
                let dt = Duration::from_millis(10);
                assert!(predicts_box_contact([[1.; 3]; 2], [a, b], dt).unwrap());
                assert!(predicts_box_contact([[1.; 3]; 2], [b, a], dt).unwrap());
                b.position[1] = 3.;
                b.angular_velocity[2] = 12.;
                assert!(!predicts_box_contact([[1.; 3]; 2], [a, b], dt).unwrap());
            }
        }
    }

    #[test]
    fn grazing_spinning_tip_can_cross_and_leave_between_cast_samples() {
        let sizes = [[16., 0.0016, 0.0016], [0.0016; 3]];
        let mut bar = stationary(0.);
        bar.orientation = Rotation::from_rotation_z(-0.075).to_array();
        bar.velocity[1] = -400.;
        bar.angular_velocity[2] = 50.;
        let mut target = stationary(8.);
        target.position[1] = -0.6;
        // After 1.5 ms the bar is horizontal and its tip coincides with
        // the target center. Both endpoint poses of the 10 ms interval miss.
        for (dims, states) in [
            (sizes, [bar, target]),
            ([sizes[1], sizes[0]], [target, bar]),
        ] {
            assert!(predicts_box_contact(dims, states, Duration::from_millis(10)).unwrap());
            assert!(!predicts_box_contact(dims, states, Duration::ZERO).unwrap());
        }
    }

    #[test]
    fn malformed_or_unrepresentable_geometry_is_rejected() {
        let states = [stationary(0.), stationary(2.)];
        for value in [0., -1., f64::INFINITY, f64::MAX, f64::NAN] {
            assert!(predicts_box_contact([[value; 3], [1.; 3]], states, Duration::ZERO).is_err());
        }
        for i in 0..2 {
            for value in [f64::INFINITY, f64::NAN] {
                let mut bad = states;
                bad[i].velocity[1] = value;
                assert!(predicts_box_contact([[1.; 3]; 2], bad, Duration::ZERO).is_err());
                bad = states;
                bad[i].angular_velocity[2] = value;
                assert!(predicts_box_contact([[1.; 3]; 2], bad, Duration::ZERO).is_err());
            }
            let mut bad = states;
            bad[i].orientation = [0.; 4];
            assert!(predicts_box_contact([[1.; 3]; 2], bad, Duration::ZERO).is_err());
        }
        assert!(
            predicts_box_contact(
                [[1.; 3]; 2],
                [stationary(-f64::MAX), stationary(f64::MAX)],
                Duration::ZERO
            )
            .is_err()
        );
    }
}
