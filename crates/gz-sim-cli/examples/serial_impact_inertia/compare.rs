// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Serialize)]
pub(super) struct Native {
    pub left_ns: f64,
    pub right_ns: f64,
    pub native_steps: u32,
    pub refined_public_ms: Vec<u32>,
    other_ns: f64,
    first_loaded_end_ns: Option<u128>,
    last_loaded_public_ms: Option<u32>,
    max_abs_axial_momentum_residual_ns: f64,
    peak_abs_joint_speed: [f64; 6],
    min_joint_position: [f64; 6],
    max_joint_position: [f64; 6],
}
impl Native {
    fn from_run(r: &Run) -> Self {
        let mut x = Self {
            left_ns: 0.,
            right_ns: 0.,
            native_steps: 0,
            refined_public_ms: Vec::new(),
            other_ns: 0.,
            first_loaded_end_ns: None,
            last_loaded_public_ms: None,
            max_abs_axial_momentum_residual_ns: 0.,
            peak_abs_joint_speed: [0.; 6],
            min_joint_position: [0.; 6],
            max_joint_position: [0.; 6],
        };
        let mut normal_x = 0.;
        for s in &r.samples {
            let i = &s.impact;
            x.left_ns += i.left_ns;
            x.right_ns += i.right_ns;
            x.native_steps += s.native_steps;
            if s.native_steps > 1 {
                x.refined_public_ms.push(s.ms);
            }
            x.other_ns += i.other_ns;
            normal_x += i.normal_on_parcel_ns[0];
            if i.left_ns + i.right_ns > 0. {
                x.first_loaded_end_ns = x.first_loaded_end_ns.or(i.first_end_ns);
                x.last_loaded_public_ms = Some(s.ms);
            }
            // All incoming velocity is along the face normal. Gravity contributes no X impulse.
            let delta_p = 0.2 * (s.bodies[5].v[0] - r.incoming[5].v[0]);
            x.max_abs_axial_momentum_residual_ns = x
                .max_abs_axial_momentum_residual_ns
                .max((delta_p - normal_x).abs());
            for j in 0..6 {
                x.peak_abs_joint_speed[j] =
                    x.peak_abs_joint_speed[j].max(s.joint_velocity[j].abs());
                x.min_joint_position[j] = x.min_joint_position[j].min(s.joint_position[j]);
                x.max_joint_position[j] = x.max_joint_position[j].max(s.joint_position[j]);
            }
        }
        x
    }
}

#[derive(Default, Serialize)]
pub(super) struct Difference {
    pub max_translation_mm: f64,
    pub max_rotation_deg: f64,
    pub impulse_percent: Option<f64>,
}

fn compare(a: &Run, b: &Run, measured: &Native, reference: &Native) -> Difference {
    assert_eq!(a.incoming, b.incoming);
    assert_eq!(a.initial_patch_points, 0);
    assert_eq!(b.initial_patch_points, 0);
    assert_eq!(a.samples.len(), b.samples.len());
    let mut d = Difference::default();
    let total = |s: &Native| if a.trial.right { s.right_ns } else { s.left_ns };
    if total(reference) > 0. {
        d.impulse_percent = Some(100. * (total(measured) / total(reference) - 1.).abs());
    }
    for (x, y) in a.samples.iter().zip(&b.samples) {
        assert_eq!(x.ms, y.ms);
        for (a, b) in x.bodies.iter().zip(&y.bodies) {
            let distance = a
                .xyz
                .iter()
                .zip(b.xyz)
                .map(|(u, v)| (u - v).powi(2))
                .sum::<f64>()
                .sqrt();
            let dot = a.q.iter().zip(b.q).map(|(u, v)| u * v).sum::<f64>();
            let degrees = (2. * dot.abs().min(1.).acos()).to_degrees();
            d.max_translation_mm = d.max_translation_mm.max(1000. * distance);
            d.max_rotation_deg = d.max_rotation_deg.max(degrees);
        }
    }
    d
}

#[derive(Serialize)]
pub(super) struct Comparison<'a> {
    trial: &'a Trial,
    reference_first_native_steps: Option<u32>,
    tensors_kg_m2: [[f64; 3]; 5],
    native_left_tensor_kg_m2: [[f64; 3]; 3],
    native_robot_mass_kg: f64,
    pub primary: Native,
    check_250ns: Native,
    fine_125ns: Native,
    pub primary_vs_fine: Difference,
    check_vs_fine: Difference,
}

pub(super) fn against_fine(a: &Run) -> Result<Comparison<'_>> {
    let first = if a.trial.stop_guard {
        Some(a.samples[0].native_steps)
    } else {
        a.trial.first_native_steps
    };
    let fork = |substeps| {
        run_with_first_solver(
            Trial {
                substeps,
                automatic: false,
                impact_iterations: None,
                impact_pgs: None,
                first_native_steps: first,
                stop_guard: false,
                touching_budget: false,
                early_face_step_ns: None,
                early_face_max_steps: None,
                replay: None,
                ..a.trial.clone()
            },
            if first.is_some() {
                a.first_solver
            } else {
                None
            },
            first.and(a.first_early_schedule),
        )
    };
    let b = fork(4000)?;
    let c = fork(8000)?;
    if first.is_some() {
        for r in [&b, &c] {
            assert_eq!(a.samples[0].bodies, r.samples[0].bodies);
            assert_eq!(a.samples[0].joint_velocity, r.samples[0].joint_velocity);
            assert_eq!(a.samples[0].joint_position, r.samples[0].joint_position);
        }
    }
    let primary = Native::from_run(a);
    let check = Native::from_run(&b);
    let fine = Native::from_run(&c);
    Ok(Comparison {
        trial: &a.trial,
        reference_first_native_steps: first,
        tensors_kg_m2: a.tensors_kg_m2,
        native_left_tensor_kg_m2: a.native_left_tensor_kg_m2,
        native_robot_mass_kg: a.native_robot_mass_kg,
        primary_vs_fine: compare(a, &c, &primary, &fine),
        check_vs_fine: compare(&b, &c, &check, &fine),
        primary,
        check_250ns: check,
        fine_125ns: fine,
    })
}
