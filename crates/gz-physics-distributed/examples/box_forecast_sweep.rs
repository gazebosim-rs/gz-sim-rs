// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plant a known interior intersection at a random time in each 10 ms box sweep.
use gz_physics::BodyState;
use gz_physics_multifidelity::{partition_prediction_distance, predicts_box_contact};
use rapier3d_f64::{
    parry::{
        query::{self, NonlinearRigidMotion},
        shape::Cuboid,
    },
    prelude::{Pose, Rotation, Vector},
};
use serde::Serialize;
use std::{error::Error, time::Duration};

const DT: Duration = Duration::from_millis(10);
const SEED: u64 = 11;
const CASES: usize = 12_000;

// This sequence reproduces the independent thin-bar review that found misses in the native cast.
struct Samples(u64);
impl Samples {
    fn unit(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(0x5851_f42d_4c95_7f2d).wrapping_add(1);
        (self.0 >> 11) as f64 / (1_u64 << 53) as f64
    }
    fn vector(&mut self) -> Vector {
        Vector::new(self.unit() - 0.5, self.unit() - 0.5, self.unit() - 0.5)
    }
}

#[derive(Clone, Copy)]
struct BoxMotion {
    pose: Pose,
    linear: Vector,
    spin: Vector,
    half: Vector,
}
impl BoxMotion {
    fn motion(self) -> NonlinearRigidMotion {
        NonlinearRigidMotion::new(self.pose, Vector::ZERO, self.linear, self.spin)
    }
    fn state(self) -> BodyState {
        BodyState {
            position: self.pose.translation.to_array(),
            orientation: self.pose.rotation.to_array(),
            velocity: self.linear.to_array(),
            angular_velocity: self.spin.to_array(),
        }
    }
}
fn forecast(pair: [BoxMotion; 2]) -> Result<bool, gz_physics::PhysicsError> {
    predicts_box_contact(
        pair.map(|b| (2. * b.half).to_array()),
        pair.map(BoxMotion::state),
        DT,
    )
}
fn raw_cast([a, b]: [BoxMotion; 2]) -> Result<bool, query::Unsupported> {
    let [sa, sb] = [a, b].map(|body| Cuboid::new(body.half));
    let t = DT.as_secs_f64();
    let hit = if a.spin == Vector::ZERO && b.spin == Vector::ZERO {
        query::cast_shapes(
            &a.pose,
            a.linear,
            &sa,
            &b.pose,
            b.linear,
            &sb,
            query::ShapeCastOptions::with_max_time_of_impact(t),
        )?
    } else {
        query::cast_shapes_nonlinear(&a.motion(), &sa, &b.motion(), &sb, 0., t, true)?
    };
    Ok(hit.is_some())
}

#[derive(Default, Serialize)]
struct Counts {
    seed: u64,
    generated: usize,
    initial_prediction_distance_m: f64,
    initially_within_prediction_distance: usize,
    confirmed_intersections: usize,
    production_positive_forward: usize,
    production_positive_reverse: usize,
    missed_by_raw_cast_forward: usize,
    missed_by_raw_cast_reverse: usize,
    missed_by_raw_cast_both: usize,
    confirmed_with_both_angular_travels_at_most_pi_over_four: usize,
    raw_forward_misses_in_that_subset: usize,
    displaced_negative: usize,
    displaced_order_disagreements: usize,
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut samples = Samples(SEED);
    let mut report = Counts {
        seed: SEED,
        generated: CASES,
        initial_prediction_distance_m: partition_prediction_distance(),
        ..Default::default()
    };
    let dt = DT.as_secs_f64();
    for i in 0..CASES {
        let direction = samples.vector().normalize();
        let angular_direction = samples.vector();
        let velocity_direction = samples.vector();
        let first_orientation = samples.vector();
        let second_orientation = samples.vector();
        let displacement_direction = samples.vector();
        let origin = if i % 3 == 0 {
            direction * 1e7
        } else {
            Vector::ZERO
        };
        let witness_time = (samples.unit() * 0.95 + 0.025) * dt;
        let spin_scale = if i % 2 == 0 { 0.7 } else { 6.2 };
        let a = BoxMotion {
            pose: Pose::from_parts(origin, Rotation::from_scaled_axis(first_orientation)),
            linear: velocity_direction * 100.,
            spin: if i % 4 == 0 {
                Vector::ZERO
            } else {
                angular_direction * spin_scale / dt
            },
            half: Vector::new(1., 0.0001, 0.0001),
        };
        // The small box's center coincides with an interior point of the bar at witness_time.
        let witness = a.motion().position_at_time(witness_time)
            * Vector::new(0.9 + samples.unit() * 0.1, 0., 0.);
        let velocity = second_orientation * 25.;
        let b = BoxMotion {
            pose: Pose::from_parts(
                witness - velocity * witness_time,
                Rotation::from_scaled_axis(second_orientation),
            ),
            linear: velocity,
            spin: if i % 3 == 0 {
                Vector::ZERO
            } else {
                second_orientation * spin_scale / dt
            },
            half: Vector::new(0.0004, 0.0003, 0.0005),
        };
        let [sa, sb] = [a, b].map(|body| Cuboid::new(body.half));
        let gap = query::distance(&a.pose, &sa, &b.pose, &sb)?.distance;
        if gap <= report.initial_prediction_distance_m {
            report.initially_within_prediction_distance += 1;
            continue;
        }
        let contact = query::contact(
            &a.motion().position_at_time(witness_time),
            &sa,
            &b.motion().position_at_time(witness_time),
            &sb,
            0.,
        )?;
        if !contact.is_some_and(|c| c.dist < 0.) {
            return Err(format!("case {i} lacks its planted strict overlap").into());
        }
        report.confirmed_intersections += 1;
        let forward = forecast([a, b])?;
        let reverse = forecast([b, a])?;
        report.production_positive_forward += usize::from(forward);
        report.production_positive_reverse += usize::from(reverse);
        if !(forward && reverse) {
            return Err(format!(
                "case {i} missed a planted contact, forward={forward}, reverse={reverse}"
            )
            .into());
        }
        let miss_forward = !raw_cast([a, b])?;
        let miss_reverse = !raw_cast([b, a])?;
        report.missed_by_raw_cast_forward += usize::from(miss_forward);
        report.missed_by_raw_cast_reverse += usize::from(miss_reverse);
        report.missed_by_raw_cast_both += usize::from(miss_forward && miss_reverse);
        if a.spin.length().max(b.spin.length()) * dt <= std::f64::consts::FRAC_PI_4 {
            report.confirmed_with_both_angular_travels_at_most_pi_over_four += 1;
            report.raw_forward_misses_in_that_subset += usize::from(miss_forward);
        }

        // Perturb the target and check argument order. Some near perturbations still hit.
        let mut moved = b;
        moved.pose.translation += displacement_direction * if i % 2 == 0 { 100. } else { 0.03 };
        let xy = forecast([a, moved])?;
        let yx = forecast([moved, a])?;
        report.displaced_negative += usize::from(!xy);
        report.displaced_order_disagreements += usize::from(xy != yx);
    }
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}
