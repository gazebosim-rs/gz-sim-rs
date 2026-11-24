// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec2, DVec3};
use gz_physics::{
    sole_balance::{SupportFootprint, SupportWrench},
    sole_contact::{SoleBody, SoleDeck, SoleMaterial, SoleState},
    sole_supports::{PanelError, SolePanel, advance_panels, validate_panels},
};
fn panel(id: u64, x: f64) -> SolePanel {
    SolePanel {
        id,
        deck: SoleDeck {
            center: DVec3::new(x, 0., -0.05),
            half_extents: DVec3::new(0.12, 0.18, 0.05),
            orientation: DQuat::IDENTITY,
            velocity: DVec3::ZERO,
        },
        material: SoleMaterial::new(2e6, 2e6, 0.5, 0.4, 0.01).unwrap(),
    }
}
fn body() -> (SoleBody, SoleState) {
    (
        SoleBody {
            mass: 2.,
            half_extents: DVec3::new(0.18, 0.1, 0.05),
            foot_offset: DVec3::ZERO,
            inertia: DVec3::splat(0.03),
        },
        SoleState {
            center: DVec3::new(0., 0., 0.048),
            orientation: DQuat::IDENTITY,
            velocity: DVec3::ZERO,
            spin: DVec3::ZERO,
        },
    )
}
#[test]
fn projected_interiors_must_be_disjoint_for_the_entire_tick() {
    let a = panel(1, -0.12);
    let mut b = panel(2, 0.12);
    validate_panels(&[a, b], 0.1).unwrap();
    assert!(matches!(
        validate_panels(&[a, a], 0.1),
        Err(PanelError::Duplicate(1))
    ));
    b.deck.center.x = 0.5;
    b.deck.velocity.x = -10.;
    assert!(matches!(
        validate_panels(&[a, b], 0.1),
        Err(PanelError::Overlap(..))
    ));
    b.deck.center.y = 0.5;
    validate_panels(&[a, b], 0.1).unwrap();
    b.deck.orientation = DQuat::from_rotation_y(0.1);
    assert!(matches!(
        validate_panels(&[a, b], 0.1),
        Err(PanelError::DifferentNormals(..))
    ));
}
#[test]
fn shared_seam_matches_unsplit_drop_and_order_is_canonical() {
    let (b, mut state) = body();
    let mut whole = state;
    let mut rev = state;
    let a = panel(10, -0.12);
    let c = panel(4, 0.12);
    let mut d = panel(8, 0.);
    d.deck.half_extents.x = 0.24;
    let h = 0.004;
    let grav = -9.81 * DVec3::Z;
    let one = b
        .step(&mut whole, &d.deck, &d.material, grav, h, 128, 0)
        .unwrap();
    let step = |s: &mut SoleState, p: &[SolePanel]| {
        advance_panels(&b, s, p, grav, DVec3::ZERO, DVec3::ZERO, h, 128, 0).unwrap()
    };
    let rows = step(&mut state, &[a, c]);
    let reversed = step(&mut rev, &[c, a]);
    assert_eq!(state.center, rev.center);
    assert_eq!(state.spin, rev.spin);
    assert_eq!(rows[0].id, 4);
    assert_eq!(rows[0].on_sole.impulse, reversed[0].on_sole.impulse);
    assert!(state.center.distance(whole.center) < 1e-14);
    assert!(state.velocity.distance(whole.velocity) < 1e-12);
    assert!(
        (rows[0].on_sole.normal_impulse + rows[1].on_sole.normal_impulse - one.normal_impulse)
            .abs()
            < 1e-12
    );
}
#[test]
fn failed_second_panel_does_not_advance_the_first() {
    let (b, mut s) = body();
    let input = s;
    let a = panel(1, -0.12);
    let mut d = panel(2, 0.12);
    d.deck.velocity.z = 100.;
    assert!(
        advance_panels(
            &b,
            &mut s,
            &[a, d],
            DVec3::ZERO,
            DVec3::ZERO,
            DVec3::ZERO,
            0.01,
            16,
            0
        )
        .is_err()
    );
    assert_eq!(s.center, input.center);
    assert_eq!(s.orientation, input.orientation);
    assert_eq!(s.velocity, input.velocity);
    assert_eq!(s.spin, input.spin);
}
#[test]
fn load_cell_preserves_force_and_free_torque_across_sensor_origins() {
    let (body, mut s) = body();
    s.spin.z = 1.;
    s.velocity.x = 0.3;
    let p = panel(1, 0.08);
    let rows = advance_panels(
        &body,
        &mut s,
        &[p],
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        1e-5,
        2,
        1,
    )
    .unwrap();
    let r = &rows[0];
    let axis = DQuat::from_rotation_z(0.7);
    let sensor = DVec3::new(0.17, -0.24, -0.01);
    let w = SupportWrench::at_sensor(r, s.center, sensor, axis).unwrap();
    assert!((axis * w.force_n + r.on_sole.contact.force).length() < 1e-12);
    assert!(
        (axis * w.moment_nm
            + (sensor - s.center).cross(axis * w.force_n)
            + r.on_sole.contact.torque)
            .length()
            < 1e-12
    );
    let (z, twist) = w.zero_moment_point(1.).unwrap();
    assert!((z.extend(0.).cross(w.force_n) + twist * DVec3::Z - w.moment_nm).length() < 1e-12);
    assert!(w.zero_moment_point(1e6).is_none());
}
#[test]
fn support_margin_tracks_load_bearing_edges_and_projected_gravity() {
    let (body, s) = body();
    let a = panel(1, -0.12);
    let b = panel(2, 0.12);
    for (panels, expected) in [(vec![a, b], 0.1), (vec![a], 0.)] {
        let mut state = s;
        let rows = advance_panels(
            &body,
            &mut state,
            &panels,
            DVec3::ZERO,
            DVec3::ZERO,
            DVec3::ZERO,
            1e-9,
            1,
            0,
        )
        .unwrap();
        let fp = SupportFootprint::from_panels(&rows, DVec3::ZERO, DQuat::IDENTITY, 0.).unwrap();
        assert!((fp.margin_m(DVec2::ZERO).unwrap() - expected).abs() < 1e-10);
        assert!(fp.margin_m(fp.pressure_center).unwrap() > 0.);
        assert!(fp.margin_m(DVec2::splat(0.4)).unwrap() < 0.);
        assert!(fp.vertices().len() >= 4);
        let p = fp
            .load_line(DVec3::new(0., 0., 0.5), DVec3::new(0.1, 0., -1.))
            .unwrap();
        assert_eq!(p, DVec2::new(0.05, 0.));
        assert!(fp.load_line(DVec3::ZERO, DVec3::X).is_none());
        assert!(SupportFootprint::from_panels(&rows, DVec3::ZERO, DQuat::IDENTITY, 1e6).is_none());
    }
}
