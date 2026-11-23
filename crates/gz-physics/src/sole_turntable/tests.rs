// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn fixture() -> (SoleBody, SoleState, SoleTurntable, SoleMaterial) {
    let b = SoleBody {
        half_extents: DVec3::new(0.1, 0.06, 0.04),
        foot_offset: DVec3::ZERO,
        mass: 2.,
        inertia: DVec3::new(0.03, 0.04, 0.05),
    };
    let d = SoleDeck {
        center: DVec3::ZERO,
        orientation: DQuat::IDENTITY,
        velocity: DVec3::ZERO,
        half_extents: DVec3::new(0.4, 0.2, 0.05),
    };
    let s = SoleState {
        center: DVec3::new(0.12, 0., 0.087),
        orientation: DQuat::IDENTITY,
        spin: DVec3::ZERO,
        velocity: DVec3::ZERO,
    };
    (
        b,
        s,
        SoleTurntable {
            deck: d,
            angular_velocity: DVec3::Z * 2.,
        },
        SoleMaterial::new(4e5, 2e5, 0.8, 0.5, 0.005).unwrap(),
    )
}

#[test]
fn corotating_translating_tilted_sole_has_zero_slip_and_normal_heat() {
    let (b, mut s, mut table, m) = fixture();
    table.deck.center = DVec3::new(-1., 2., -0.2);
    table.deck.orientation = DQuat::from_rotation_y(0.8);
    table.deck.velocity = DVec3::new(0.4, 0.1, -0.6);
    table.angular_velocity = DVec3::new(0.8, -1.1, 0.25);
    s.center = table.deck.center + table.deck.orientation * s.center;
    s.orientation = table.deck.orientation;
    s.velocity = table.deck.velocity + table.angular_velocity.cross(s.center - table.deck.center);
    s.spin = table.angular_velocity;
    let c = table.contact(&b, &s, &m, 1).unwrap();
    assert!(
        c.force
            .distance(table.deck.orientation * DVec3::Z * c.normal_force)
            < 1e-11
    );
    assert!(c.heat_power < 1e-20 && c.sliding_power.abs() < 1e-20);
    assert!(c.normal_force > 0.);
}

#[test]
fn rotating_table_spins_up_sliding_sole_and_accounts_for_work() {
    let (b, mut s, mut t, m) = fixture();
    s.center.x = 0.;
    let g = -DVec3::Z * 9.8;
    t.deck.velocity = DVec3::new(0.2, -0.4, 0.);
    s.velocity = t.deck.velocity;
    let start = s;
    let initial = t.contact(&b, &s, &m, 3).unwrap();
    let dt = 0.001;
    let out =
        advance_turntable(&b, &mut s, &t, &m, g, DVec3::ZERO, DVec3::ZERO, dt, 64, 2).unwrap();
    assert!(
        s.spin.z > 0.
            && out.on_sole.angular_impulse.z > 0.
            && out.spin_work_j > 0.
            && out.on_sole.sliding_work < 0.
    );
    let kinetic = |s: SoleState| {
        0.5 * (b.mass * s.velocity.length_squared()
            + (s.orientation.conjugate() * s.spin)
                .dot(b.inertia * (s.orientation.conjugate() * s.spin)))
    };
    let residual = kinetic(s) - kinetic(start) + out.on_sole.contact.strain_energy
        - initial.strain_energy
        - b.mass * g.dot(s.center - start.center)
        + out.on_sole.dissipated_energy
        - out.on_sole.sliding_work
        - out.on_sole.boundary_work
        - out.support_work_j;
    assert!(residual.abs() < 1e-9, "residual {residual}");
    assert!(
        (out.support_work_j - (out.on_sole.impulse.dot(t.deck.velocity) + out.spin_work_j)).abs()
            < 1e-14
    );
}

#[test]
fn moving_rotated_rim_power_matches_centered_strain_derivative() {
    let (b, mut s, mut t, m) = fixture();
    s.center = DVec3::new(0.38, 0.08, 0.087);
    s.velocity = DVec3::new(0.2, -0.1, -0.055);
    s.spin = DVec3::new(0.5, -0.6, 0.7);
    t.angular_velocity = DVec3::new(0.4, -0.2, 2.);
    t.deck.velocity = DVec3::new(0.15, 0.21, 0.03);
    let c = t.contact(&b, &s, &m, 2).unwrap();
    let energy = |dt: f64| {
        let next = SoleState {
            center: s.center + dt * s.velocity,
            orientation: DQuat::from_scaled_axis(dt * s.spin) * s.orientation,
            ..s
        };
        SoleTurntable {
            deck: t.at(dt),
            ..t
        }
        .contact(&b, &next, &m, 2)
        .unwrap()
        .strain_energy
    };
    let h = 1e-7;
    let stored = (energy(h) - energy(-h)) / (2. * h);
    let body_power = c.force.dot(s.velocity) + c.torque.dot(s.spin);
    let support = c.force.dot(t.deck.velocity) + t.spin_power(s, c);
    let residual =
        body_power + stored + c.heat_power - c.sliding_power - c.boundary_power - support;
    assert!(
        c.boundary_power.abs() > 1e-4 && residual.abs() < 3e-8,
        "rim {} residual {residual}",
        c.boundary_power
    );
}

#[test]
fn narrow_rotating_rim_crosses_between_all_coarse_stage_samples() {
    let (mut b, mut s, mut t, _) = fixture();
    b.half_extents.x = 0.002;
    b.half_extents.y = 0.002;
    t.deck.half_extents.y = 0.003;
    s.center.x = 0.18;
    s.center.y = 0.18;
    let dt = 0.2;
    t.angular_velocity = DVec3::Z * std::f64::consts::TAU / dt;
    let m = SoleMaterial::new(1e3, 1e3, 0.1, 0.2, 0.01).unwrap();
    let mut coarse = s;
    let a = advance_turntable(
        &b,
        &mut coarse,
        &t,
        &m,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        dt,
        2,
        0,
    )
    .unwrap();
    assert_eq!(a.on_sole.normal_impulse, 0.);
    assert!(a.on_sole.possible_missed_contacts > 0);
    let p = SoleAccuracy {
        min_substeps: 1,
        max_substeps: 512,
        min_depth: 0,
        max_depth: 2,
        position_m: 1.,
        rotation_rad: 1.,
        impulse_ns: 1.,
        angular_impulse_nms: 1.,
        heat_j: 1.,
    };
    let r = advance_turntable_adaptive(
        &b,
        &mut s,
        &t,
        &m,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        dt,
        &p,
        1.,
    )
    .unwrap();
    assert!(
        r.substeps > 2
            && r.result.on_sole.normal_impulse > 0.
            && r.result.on_sole.possible_missed_contacts == 0
    );
}

#[test]
fn stationary_spin_control_matches_old_wrench_path_and_failure_rolls_back() {
    let (b, s, mut t, m) = fixture();
    t.angular_velocity = DVec3::ZERO;
    let mut a = s;
    let mut z = s;
    let applied = DVec3::new(0.7, -0.8, 0.2);
    let old = b
        .step_with_wrench(
            &mut a,
            &t.deck,
            &m,
            DVec3::ZERO,
            0.001,
            16,
            1,
            applied,
            applied,
        )
        .unwrap();
    let fresh = advance_turntable(
        &b,
        &mut z,
        &t,
        &m,
        DVec3::ZERO,
        applied,
        applied,
        0.001,
        16,
        1,
    )
    .unwrap();
    assert_eq!(a.center, z.center);
    assert_eq!(fresh.on_sole.impulse, old.impulse);
    t.angular_velocity = DVec3::Z;
    z.velocity.z = -100.;
    let before = z;
    assert!(
        advance_turntable(
            &b,
            &mut z,
            &t,
            &m,
            DVec3::ZERO,
            DVec3::ZERO,
            DVec3::ZERO,
            0.01,
            4,
            0
        )
        .is_err()
    );
    assert_eq!(z.center, before.center);
    assert_eq!(z.spin, before.spin);
    assert_eq!(z.velocity, before.velocity);
}

#[test]
fn invalid_pose_spin_and_nonrepresentable_microstep_cannot_commit() {
    let (b, start, t, m) = fixture();
    for (q, w, dt) in [
        (DQuat::from_xyzw(0., 0., 0., 0.), DVec3::Z, 0.001),
        (DQuat::IDENTITY, DVec3::NAN, 0.001),
        (DQuat::IDENTITY, DVec3::Z, f64::from_bits(1)),
    ] {
        let table = SoleTurntable {
            deck: SoleDeck {
                orientation: q,
                ..t.deck
            },
            angular_velocity: w,
        };
        let mut s = start;
        assert!(
            advance_turntable(
                &b,
                &mut s,
                &table,
                &m,
                DVec3::ZERO,
                DVec3::ZERO,
                DVec3::ZERO,
                dt,
                2,
                0
            )
            .is_err()
        );
        assert_eq!(s.center, start.center);
        assert_eq!(s.spin, start.spin);
    }
}
