// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DQuat;

fn setup() -> (SoleBody, SoleState, Vec<SolePanel>, SoleAccuracy) {
    let b = SoleBody {
        half_extents: DVec3::new(0.08, 0.1, 0.05),
        foot_offset: DVec3::ZERO,
        mass: 2.,
        inertia: DVec3::splat(0.05),
    };
    let s = SoleState {
        center: DVec3::new(0., 0., 0.096),
        orientation: DQuat::IDENTITY,
        spin: DVec3::new(0.8, 0.4, 1.),
        velocity: DVec3::new(0., 0., -0.2),
    };
    let panels = [(12, -0.07, -0.3, 3e5), (8, 0.07, 0.3, 4e5)]
        .map(|(id, y, v, stiffness)| SolePanel {
            id,
            deck: SoleDeck {
                center: DVec3::new(0., y, 0.),
                orientation: DQuat::IDENTITY,
                half_extents: DVec3::new(0.2, 0.05, 0.05),
                velocity: DVec3::X * v,
            },
            material: SoleMaterial::new(stiffness, 2e5, 0.8, 0.4, 0.005).unwrap(),
        })
        .to_vec();
    let p = SoleAccuracy {
        min_substeps: 1,
        max_substeps: 128,
        min_depth: 0,
        max_depth: 4,
        position_m: 1e-8,
        rotation_rad: 1e-7,
        impulse_ns: 1e-5,
        angular_impulse_nms: 1e-6,
        heat_j: 1e-5,
    };
    (b, s, panels, p)
}

#[test]
fn individually_budgeted_opposing_belts_match_fine_and_enrollment_order() {
    let (b, start, mut panels, p) = setup();
    let mut s = start;
    let dt = 0.002;
    let a = advance_panels_adaptive(
        &b,
        &mut s,
        &panels,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        dt,
        &p,
        1e-5,
    )
    .unwrap();
    assert!(a.substeps > 2 && a.panels[0].id == 8);
    let mut fine = start;
    let reference = advance_panels(
        &b,
        &mut fine,
        &panels,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        dt,
        256,
        2,
    )
    .unwrap();
    for (x, y) in a.panels.iter().zip(reference) {
        assert!(x.on_sole.impulse.distance(y.on_sole.impulse) < p.impulse_ns);
        assert!((x.support_work_j - y.support_work_j).abs() < 1e-5);
        assert_eq!(x.on_sole.possible_missed_contacts, 0);
    }
    assert!(a.panels[0].on_sole.impulse.x * a.panels[1].on_sole.impulse.x < 0.);
    panels.reverse();
    let mut reversed = start;
    let r = advance_panels_adaptive(
        &b,
        &mut reversed,
        &panels,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        dt,
        &p,
        1e-5,
    )
    .unwrap();
    assert_eq!((r.substeps, r.depth), (a.substeps, a.depth));
    assert_eq!(s.center, reversed.center);
    assert_eq!(s.spin, reversed.spin);
}

#[test]
fn per_support_comparison_preserves_cancelling_errors_and_budget_failure_is_atomic() {
    let (b, start, panels, p) = setup();
    let empty = [8, 12].map(|id| PanelReaction {
        id,
        ..Default::default()
    });
    let mut opposing = empty;
    for (r, sign) in opposing.iter_mut().zip([-1., 1.]) {
        r.on_sole.impulse.x = 0.1 * sign;
        r.on_sole.angular_impulse.y = 0.2 * sign;
        r.on_sole.boundary_work = 0.3 * sign;
        r.support_work_j = 0.4 * sign;
    }
    let e = PanelDifference::compare(&(start, empty.to_vec()), &(start, opposing.to_vec()));
    assert_eq!(
        (
            e.contact.impulse_ns,
            e.contact.angular_impulse_nms,
            e.rim_work_j,
            e.support_work_j
        ),
        (0.1, 0.2, 0.3, 0.4)
    );
    let budget = SoleAccuracy {
        max_substeps: 2,
        max_depth: 1,
        ..p
    };
    let mut s = start;
    assert!(matches!(
        advance_panels_adaptive(
            &b,
            &mut s,
            &panels,
            DVec3::ZERO,
            DVec3::ZERO,
            DVec3::ZERO,
            0.002,
            &budget,
            0.
        ),
        Err(PanelError::Contact(SoleError::AccuracyBudget))
    ));
    assert_eq!(s.center, start.center);
    assert_eq!(s.velocity, start.velocity);
}

#[test]
fn short_second_panel_crossing_triggers_time_refinement_while_first_panel_carries_load() {
    let (mut b, mut s, mut panels, p) = setup();
    // A broad, very soft left rail remains in contact. A narrow right rail is
    // crossed between all initial midpoints without ever overlapping its peer.
    b.half_extents.x = 0.002;
    s.center.x = -0.027;
    s.spin = DVec3::ZERO;
    s.velocity = DVec3::X;
    panels[0].deck.center.x = 0.;
    panels[0].deck.velocity = DVec3::ZERO;
    panels[1].deck.center.x = 0.;
    panels[1].deck.velocity = DVec3::ZERO;
    panels[1].deck.half_extents.x = 0.004;
    for panel in &mut panels {
        panel.material = SoleMaterial::new(1e2, 1e2, 0.1, 0., 0.01).unwrap();
    }
    let mut coarse = s;
    let a = advance_panels(
        &b,
        &mut coarse,
        &panels,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        0.2,
        2,
        0,
    )
    .unwrap();
    assert_eq!(a[0].on_sole.normal_impulse, 0.);
    assert!(a[0].on_sole.possible_missed_contacts > 0);
    assert!(a[1].on_sole.normal_impulse > 0.);
    let p = SoleAccuracy {
        position_m: 1.,
        rotation_rad: 1.,
        impulse_ns: 1.,
        angular_impulse_nms: 1.,
        heat_j: 1.,
        ..p
    };
    let r = advance_panels_adaptive(
        &b,
        &mut s,
        &panels,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::ZERO,
        0.2,
        &p,
        1.,
    )
    .unwrap();
    assert!(
        r.substeps > 2
            && r.panels
                .iter()
                .all(|row| row.on_sole.normal_impulse > 0.
                    && row.on_sole.possible_missed_contacts == 0)
    );
}

#[test]
fn raised_panel_clears_while_other_end_of_inclined_sole_remains_loaded() {
    // Incoming accepted state at 109 ms in the moving-seam public scene.
    let b = SoleBody {
        half_extents: DVec3::new(0.18, 0.1, 0.06),
        foot_offset: DVec3::ZERO,
        mass: 2.,
        inertia: DVec3::splat(0.05),
    };
    let mut s = SoleState {
        center: DVec3::new(
            4.066716116007467e-6,
            2.683905239541868e-18,
            0.059979604469908854,
        ),
        orientation: DQuat::from_xyzw(
            2.4032499943540696e-18,
            -0.00010619599368374028,
            -9.801002923499686e-17,
            0.9999999943612056,
        ),
        velocity: DVec3::new(
            -0.014834303142608517,
            -1.9202245887149704e-17,
            0.01669654428846405,
        ),
        spin: DVec3::new(
            1.375333139340191e-15,
            -0.25957336640495365,
            1.0810474062144246e-16,
        ),
    };
    let m = SoleMaterial::new(2e8, 4e6, 30., 0.4, 0.005).unwrap();
    let panels =
        [(0, -0.13, -0.03), (1, 0.27, -0.029999999999999995)].map(|(id, x, z)| SolePanel {
            id,
            deck: SoleDeck {
                center: DVec3::new(x, 0., z),
                half_extents: DVec3::new(0.2, 0.16, 0.03),
                velocity: DVec3::ZERO,
                orientation: DQuat::IDENTITY,
            },
            material: m,
        });
    let policy = SoleAccuracy {
        min_substeps: 4,
        max_substeps: 512,
        min_depth: 0,
        max_depth: 3,
        position_m: 1e-7,
        rotation_rad: 1e-6,
        impulse_ns: 2e-5,
        angular_impulse_nms: 2e-6,
        heat_j: 1e-5,
    };
    let r = advance_panels_adaptive(
        &b,
        &mut s,
        &panels,
        -9.81 * DVec3::Z,
        DVec3::ZERO,
        DVec3::ZERO,
        0.0005,
        &policy,
        1e-5,
    )
    .unwrap();
    assert!(r.panels[0].on_sole.normal_impulse > 0.);
    assert_eq!(r.panels[1].on_sole.possible_missed_contacts, 0);
}

#[test]
fn swept_ownership_sees_sub_ulp_crossing_and_extreme_separation() {
    let (_, _, mut panels, _) = setup();
    panels[0].deck.center = DVec3::ZERO;
    panels[0].deck.velocity = DVec3::ZERO;
    panels[1].deck.center = DVec3::X;
    panels[1].deck.velocity = -DVec3::X * 1e20;
    for p in &mut panels {
        p.deck.half_extents.x = 1e-20;
    }
    // Both floating-point slab crossing times round to 1e-20 seconds.
    assert!(matches!(
        validate_panels(&panels, 2e-20),
        Err(PanelError::Overlap(..))
    ));
    panels[0].deck.center.x = -1e308;
    panels[1].deck.center.x = 1e308;
    panels[1].deck.velocity = DVec3::ZERO;
    validate_panels(&panels, 1.).unwrap();
}

#[test]
fn invalid_and_underflowed_tick_grids_preserve_incoming_body() {
    let (b, start, panels, _) = setup();
    for dt in [f64::NAN, f64::INFINITY, f64::from_bits(1), 0., -1.] {
        let mut s = start;
        assert!(
            advance_panels(
                &b,
                &mut s,
                &panels,
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
