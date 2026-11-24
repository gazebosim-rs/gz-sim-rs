// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn fixture() -> (SoleBody, SoleDeck, SoleState, SoleMaterial) {
    (
        SoleBody {
            half_extents: DVec3::new(0.2, 0.1, 0.05),
            foot_offset: DVec3::ZERO,
            mass: 2.,
            inertia: DVec3::splat(0.05),
        },
        SoleDeck {
            half_extents: DVec3::new(0.5, 0.5, 0.05),
            center: DVec3::ZERO,
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
        },
        SoleState {
            center: DVec3::new(0., 0., 0.096),
            orientation: DQuat::IDENTITY,
            velocity: DVec3::ZERO,
            spin: DVec3::ZERO,
        },
        SoleMaterial::new(4e5, 4e5, 0.8, 0.6, 0.005).unwrap(),
    )
}
fn close(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} vs {b}, tolerance {tol}");
}
fn same(a: DVec3, b: DVec3, tol: f64) {
    assert!(a.distance(b) <= tol, "{a} vs {b}, tolerance {tol}");
}

#[test]
fn flat_slide_integrates_uniform_pressure_friction_heat_and_energy() {
    let (b, d, mut s, m) = fixture();
    s.velocity = DVec3::new(0.3, 0.4, -0.25);
    let p = b.contact(&s, &d, &m, 1).unwrap();
    let area = 0.08;
    let compression = 0.004;
    let pe = 2e5 * compression;
    let load = area * pe * 1.2;
    let shear = -0.6 * load * DVec3::new(0.3, 0.4, 0.) / 0.5_f64.hypot(0.005);
    same(p.force, DVec3::Z * load + shear, 1e-11);
    let cop = DVec3::new(0., 0., 0.046);
    same(p.center_of_pressure.unwrap(), cop, 1e-14);
    same(p.torque, (cop - s.center).cross(p.force), 1e-13);
    close(p.heat_power, area * pe * 0.2 * 0.25, 1e-12);
    close(p.strain_energy, 0.5 * area * pe * compression, 1e-14);
    close(p.sliding_power, shear.dot(s.velocity), 1e-12);
    assert_eq!(p.polygon().len(), 4);
}

#[test]
fn rim_cuts_only_overhang_and_exports_recoverable_energy() {
    let (b, d, mut s, m) = fixture();
    s.center.x = 0.5;
    s.velocity.x = 0.3;
    let p = b.contact(&s, &d, &m, 0).unwrap();
    close(p.area, 0.04, 1e-14);
    close(p.center_of_pressure.unwrap().x, 0.4, 1e-14);
    close(
        p.boundary_power,
        -0.2 * 0.3 * 0.5 * 2e5 * 0.004_f64.powi(2),
        1e-13,
    );
    s.center.x = 0.71;
    let e = b.contact(&s, &d, &m, 0).unwrap();
    assert_eq!((e.normal_force, e.strain_energy, e.area), (0., 0., 0.));
    assert!(e.polygon().is_empty());
}

#[test]
fn unloading_clips_part_of_sole_and_retains_heat_on_unloaded_area() {
    let (b, d, mut s, m) = fixture();
    // v_z=1.25 - 5 x. The tensile clamp removes x<=0.
    s.velocity.z = 1.25;
    s.spin.y = 5.;
    let p = b.contact(&s, &d, &m, 1).unwrap();
    close(p.area, 0.04, 1e-14);
    close(p.normal_force, 800. * 4. * 0.2 * 0.2 * 0.2 / 2., 1e-12);
    close(p.center_of_pressure.unwrap().x, 2. * 0.2 / 3., 1e-13);
    // Compare normal heat to the integral of v*(p-p_elastic) in the two halves.
    let h = 0.2;
    let active = 0.8 * (1.25 * 1.25 * h - 1.25 * 5. * h * h + 25. * h * h * h / 3.);
    let unload = 1.25 * h + 2.5 * h * h;
    close(p.heat_power, 800. * 0.2 * (active + unload), 1e-11);
    assert!(p.polygon().iter().all(|p| p.x >= -1e-14));
    s.spin = DVec3::ZERO;
    s.velocity.z = 3.;
    let e = b.contact(&s, &d, &m, 0).unwrap();
    close(e.heat_power, 3. * 0.08 * 800., 1e-10);
    assert_eq!(e.normal_force, 0.);
    assert!(e.strain_energy > 0. && e.center_of_pressure.is_none() && e.polygon().is_empty());
}

#[test]
fn world_rotation_translation_and_boost_preserve_relative_solution() {
    let (b, d, mut s, m) = fixture();
    s.spin = DVec3::new(2., -1., 0.4);
    s.velocity = DVec3::new(0.4, -0.2, -0.1);
    let q = DQuat::from_rotation_y(0.42) * DQuat::from_rotation_x(-0.95);
    let origin = DVec3::new(3., -4., 5.);
    let boost = DVec3::new(-2., 5., 0.8);
    let moving = SoleDeck {
        center: origin,
        velocity: boost,
        orientation: q,
        ..d
    };
    let mut other = SoleState {
        center: origin + q * s.center,
        orientation: q * s.orientation,
        velocity: boost + q * s.velocity,
        spin: q * s.spin,
    };
    let g = -DVec3::Z * 9.8;
    let f = DVec3::new(2., -4., 1.);
    let tau = DVec3::new(0.1, -0.3, 0.2);
    let (dt, n) = (0.001, 20);
    let a = b
        .step_with_wrench(&mut s, &d, &m, g, dt, n, 0, f, tau)
        .unwrap();
    let z = b
        .step_with_wrench(&mut other, &moving, &m, q * g, dt, n, 0, q * f, q * tau)
        .unwrap();
    same(other.center, origin + boost * dt + q * s.center, 1e-13);
    same(other.spin, q * s.spin, 1e-12);
    same(other.velocity, boost + q * s.velocity, 1e-12);
    same(z.impulse, q * a.impulse, 1e-13);
    same(z.angular_impulse, q * a.angular_impulse, 1e-13);
    same(z.contact.force, q * a.contact.force, 1e-11);
    same(
        z.contact.center_of_pressure.unwrap(),
        origin + dt * boost + q * a.contact.center_of_pressure.unwrap(),
        1e-12,
    );
    for (&p, &r) in a.contact.polygon().iter().zip(z.contact.polygon()) {
        same(r, origin + dt * boost + q * p, 1e-12);
    }
}

#[test]
fn free_asymmetric_body_conserves_world_angular_momentum_under_torque() {
    let (mut b, d, mut s, m) = fixture();
    b.inertia = DVec3::new(0.02, 0.03, 0.045);
    s.center.z = 1.;
    s.velocity = DVec3::new(-1., 0.5, 0.2);
    s.spin = DVec3::new(2., -3., 4.);
    let l = |s: SoleState| s.orientation * (b.inertia * (s.orientation.conjugate() * s.spin));
    let initial = s;
    let torque = DVec3::new(0.3, -0.8, 0.1);
    let force = DVec3::new(2., 0.4, 6.);
    let dt = 0.05;
    let step = b
        .step_with_wrench(&mut s, &d, &m, DVec3::ZERO, dt, 100, 0, force, torque)
        .unwrap();
    same(l(s), l(initial) + dt * torque, 1e-13);
    same(
        s.center,
        initial.center + dt * initial.velocity + 0.5 * dt * dt * force / b.mass,
        1e-13,
    );
    assert_eq!(step.impulse, DVec3::ZERO);
}

#[test]
fn airborne_ballistic_skip_does_not_miss_touchdown() {
    let (b, d, mut s, m) = fixture();
    s.center.z = 1.;
    s.velocity = -DVec3::Z;
    let dt = 0.02;
    let g = -DVec3::Z * 10.;
    let initial = s;
    let a = b.step(&mut s, &d, &m, g, dt, 1000, 0).unwrap();
    same(
        s.center,
        initial.center + dt * initial.velocity + 0.5 * g * dt * dt,
        1e-15,
    );
    assert_eq!(a.normal_impulse, 0.);
    s.center.z = 0.101;
    s.velocity = -DVec3::Z;
    let a = b.step(&mut s, &d, &m, g, 0.005, 50, 0).unwrap();
    assert!(a.normal_impulse > 0.);
}

#[test]
fn invalid_state_and_through_coating_refuse_the_whole_tick() {
    let (b, d, mut s, m) = fixture();
    let z = s.center;
    s.velocity.z = -20.;
    assert!(matches!(
        b.step(&mut s, &d, &m, DVec3::ZERO, 0.01, 1, 0),
        Err(SoleError::OutsideCoating)
    ));
    assert_eq!(s.center, z);
    assert_eq!(s.velocity.z, -20.);
    assert!(matches!(
        b.step(&mut s, &d, &m, DVec3::NAN, 0.01, 1, 0),
        Err(SoleError::InvalidInput)
    ));
    assert!(SoleMaterial::new(2., 3., 1., -1., 0.1).is_err());
    assert!(SoleMaterial::new(2., 3., 1., 0.1, 0.).is_err());
}

fn tolerances() -> SoleAccuracy {
    SoleAccuracy {
        min_substeps: 1,
        max_substeps: 128,
        min_depth: 0,
        max_depth: 3,
        position_m: 1e-8,
        rotation_rad: 1e-8,
        impulse_ns: 1e-6,
        angular_impulse_nms: 1e-6,
        heat_j: 1e-6,
    }
}

#[test]
fn adaptive_refines_time_for_flat_drop_and_rolls_back_on_budget_failure() {
    let (b, d, mut s, m) = fixture();
    s.velocity.z = -0.4;
    let start = s;
    let p = tolerances();
    let t = b
        .step_adaptive(
            &mut s,
            &d,
            &m,
            DVec3::ZERO,
            0.002,
            &p,
            DVec3::ZERO,
            DVec3::ZERO,
        )
        .unwrap();
    assert!(t.substeps > 2 && t.depth == 1 && t.temporal.impulse_ns <= p.impulse_ns);
    let strict = SoleAccuracy {
        max_substeps: 2,
        ..p
    };
    s = start;
    assert!(matches!(
        b.step_adaptive(
            &mut s,
            &d,
            &m,
            DVec3::ZERO,
            0.002,
            &strict,
            DVec3::ZERO,
            DVec3::ZERO
        ),
        Err(SoleError::AccuracyBudget)
    ));
    assert_eq!(s.center, start.center);
    assert_eq!(s.velocity, start.velocity);
}

#[test]
fn adaptive_refines_space_independently_for_twisting_friction() {
    let (b, d, mut s, m) = fixture();
    s.spin.z = 4.;
    s.velocity.x = 0.07;
    let p = SoleAccuracy {
        impulse_ns: 3e-7,
        angular_impulse_nms: 1e-7,
        position_m: 1.,
        rotation_rad: 1.,
        heat_j: 1.,
        max_depth: 5,
        ..tolerances()
    };
    let r = b
        .step_adaptive(
            &mut s,
            &d,
            &m,
            DVec3::ZERO,
            0.00001,
            &p,
            DVec3::ZERO,
            DVec3::ZERO,
        )
        .unwrap();
    assert!(r.substeps == 2 && r.depth > 1, "{r:?}");
}

#[test]
fn narrow_support_crossed_between_samples_forces_temporal_refinement() {
    let (mut b, mut d, mut s, m) = fixture();
    b.half_extents.x = 0.004;
    d.half_extents.x = 0.004;
    s.center.x = -0.027;
    s.velocity.x = 1.;
    let mut coarse = s;
    let a = b.step(&mut coarse, &d, &m, DVec3::ZERO, 0.2, 2, 0).unwrap();
    assert_eq!(a.normal_impulse, 0.);
    assert!(a.possible_missed_contacts > 0);
    let p = SoleAccuracy {
        impulse_ns: 1.,
        angular_impulse_nms: 1.,
        position_m: 1.,
        rotation_rad: 1.,
        heat_j: 1.,
        ..tolerances()
    };
    let r = b
        .step_adaptive(
            &mut s,
            &d,
            &m,
            DVec3::ZERO,
            0.2,
            &p,
            DVec3::ZERO,
            DVec3::ZERO,
        )
        .unwrap();
    assert!(r.substeps > 2 && r.step.normal_impulse > 0.);
    assert_eq!(r.step.possible_missed_contacts, 0);
}

#[test]
fn adaptive_certifies_tangent_and_diagonally_separated_free_motion() {
    let (mut b, d, mut s, m) = fixture();
    s.center.z = 0.1;
    s.velocity.x = 0.02;
    let tangent = b
        .step_adaptive(
            &mut s,
            &d,
            &m,
            DVec3::ZERO,
            0.001,
            &tolerances(),
            DVec3::ZERO,
            DVec3::ZERO,
        )
        .unwrap();
    assert_eq!(tangent.step.normal_impulse, 0.);
    // The axis-aligned bounds overlap both x and y: separation is along
    // the diagonal normal of the long, narrow sole.
    b.half_extents.y = 0.01;
    s.center = DVec3::new(0.55, 0.55, 0.096);
    s.orientation = DQuat::from_rotation_z(-std::f64::consts::FRAC_PI_4);
    let outside = b
        .step_adaptive(
            &mut s,
            &d,
            &m,
            DVec3::ZERO,
            0.001,
            &tolerances(),
            DVec3::ZERO,
            DVec3::ZERO,
        )
        .unwrap();
    assert_eq!(outside.step.possible_missed_contacts, 0);
    assert_eq!(outside.step.normal_impulse, 0.);
}

#[test]
fn exact_translating_tilted_patch_matches_independent_rotating_quadrature_limit() {
    let (b, d, mut s, m) = fixture();
    s.center.x = 0.49;
    s.orientation = DQuat::from_rotation_x(-0.03) * DQuat::from_rotation_y(0.06);
    s.velocity = DVec3::new(0.05, -0.14, -0.3);
    let exact = b.contact(&s, &d, &m, 4).unwrap();
    s.spin = DVec3::splat(1e-15);
    let numerical = b.contact(&s, &d, &m, 2).unwrap();
    assert!(exact.area > 0. && exact.boundary_power < 0.);
    same(exact.force, numerical.force, 1e-12);
    same(exact.torque, numerical.torque, 1e-13);
    close(exact.heat_power, numerical.heat_power, 1e-13);
    same(
        exact.center_of_pressure.unwrap(),
        numerical.center_of_pressure.unwrap(),
        1e-13,
    );
}

#[test]
fn first_stage_only_contact_gets_a_nonzero_integrated_impulse() {
    let (mut b, mut d, mut s, m) = fixture();
    b.half_extents.x = 0.002;
    d.half_extents.x = 0.002;
    s.velocity.x = 2.;
    let p = SoleAccuracy {
        impulse_ns: 1.,
        angular_impulse_nms: 1.,
        position_m: 1.,
        rotation_rad: 1.,
        heat_j: 1.,
        ..tolerances()
    };
    let mut coarse = s;
    let old = b.step(&mut coarse, &d, &m, DVec3::ZERO, 0.1, 2, 0).unwrap();
    assert!(
        old.peak_normal_force > 0. && old.normal_impulse == 0. && old.possible_missed_contacts == 0
    );
    let r = b
        .step_adaptive(
            &mut s,
            &d,
            &m,
            DVec3::ZERO,
            0.1,
            &p,
            DVec3::ZERO,
            DVec3::ZERO,
        )
        .unwrap();
    assert!(r.substeps > 2 && r.step.normal_impulse > 0.);
}

#[test]
fn nonfinite_transformed_geometry_never_certifies_airborne_motion() {
    let (mut b, d, mut s, m) = fixture();
    b.foot_offset.z = f64::MAX;
    s.center.z = f64::MAX;
    let start = s;
    assert!(matches!(
        b.step(&mut s, &d, &m, DVec3::ZERO, 0.001, 2, 0),
        Err(SoleError::Nonfinite)
    ));
    assert_eq!(s.center, start.center);
}

#[test]
fn distant_fast_support_does_not_round_away_free_world_motion() {
    let (b, mut d, mut s, m) = fixture();
    d.center.z = -1e16;
    d.velocity.z = -1e16;
    s.center.z = 1.;
    s.velocity.z = 0.25;
    let start = s;
    let r = b.step(&mut s, &d, &m, DVec3::ZERO, 0.5, 32, 0).unwrap();
    assert_eq!(s.center.z, 1.125);
    assert_eq!(s.velocity.z, 0.25);
    assert_eq!(r.normal_impulse, 0.);
    let elastic = SoleMaterial::new(2e5, 2e5, 0., 0.4, 0.005).unwrap();
    let mut exact = start;
    b.try_flat_elastic_step(
        &mut exact,
        &d,
        &elastic,
        DVec3::ZERO,
        0.5,
        DVec3::ZERO,
        DVec3::ZERO,
    )
    .unwrap()
    .unwrap();
    assert_eq!(exact.center, s.center);
    assert_eq!(exact.velocity, s.velocity);
    let mut damped = start;
    b.try_flat_damped_step(
        &mut damped,
        &d,
        &m,
        DVec3::ZERO,
        0.5,
        32,
        DVec3::ZERO,
        DVec3::ZERO,
    )
    .unwrap()
    .unwrap();
    assert_eq!(damped.center, s.center);
    assert_eq!(damped.velocity, s.velocity);
}
