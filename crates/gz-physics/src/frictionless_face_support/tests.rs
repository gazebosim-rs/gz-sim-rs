// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn example() -> (FrictionlessFaceSupport, EdgeSupportState) {
    let model = FrictionlessFaceSupport {
        mass: 2.,
        inertia_body: DMat3::from_cols(
            DVec3::new(0.041, 0.001, -0.008),
            DVec3::new(0.001, 0.052, 0.006),
            DVec3::new(-0.008, 0.006, 0.057),
        ),
        corners: std::array::from_fn(|k| {
            DVec3::new(
                if k & 2 == 0 { -0.18 } else { 0.18 },
                if k & 1 == 0 { -0.1 } else { 0.1 },
                -0.13,
            )
        }),
        pad_axes: [DVec3::X, DVec3::Y],
        pad_half: DVec2::new(0.41, 0.41),
        gravity: DVec3::new(0.03, -0.04, -9.81),
    };
    let s = EdgeSupportState {
        center: DVec3::new(0.33, 0.01, 0.13),
        velocity: DVec3::new(0.52, -0.02, 0.),
        orientation: DQuat::from_rotation_z(0.29),
        angular_velocity: 3.6 * DVec3::Z,
    };
    (model, s)
}
#[test]
fn moving_pressure_recovers_yaw_momentum_with_only_partial_sole_coverage() {
    let (m, s) = example();
    let b = m.bind(s).unwrap().unwrap();
    let init = b.sample(0.).unwrap();
    let dt = 0.00005;
    let FaceBearingAdvance::Supported(end) = b.advance(dt).unwrap() else {
        panic!()
    };
    assert!(init.overlap.area > 0. && init.overlap.area < 0.072);
    let torque = |x: FaceBearingSample| {
        (x.center_of_pressure - x.state.center).cross(DVec3::Z * x.normal_force)
    };
    let j = (torque(init) + 4. * torque(b.sample(0.5 * dt).unwrap()) + torque(end)) * (dt / 6.);
    assert!(end.angular_impulse_about_com.distance(j) < 1e-15);
    assert!((end.normal_impulse - 2. * 9.81 * dt).abs() < 1e-16);
    assert!((end.sole_center - init.sole_center).length() < 1e-13);
    let FaceBearingAdvance::Exits(edge) = b.advance(0.3).unwrap() else {
        panic!()
    };
    assert_eq!(edge.pad_edges, 2);
    let hit = b.sample(edge.seconds).unwrap();
    assert!((hit.pad_center.x - 0.41).abs() < 2e-15 && hit.overlap.area > 0.013);
    assert!(m.bind(hit.state).unwrap().is_none());
}
#[test]
fn clipped_polygon_preserves_octagon_area_and_rejects_release_outside_the_center() {
    let q = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4);
    let square = [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]
        .map(|(x, y)| (q * DVec3::new(x, y, 0.)).truncate());
    let oct = FaceOverlap::clip(square, DVec2::ONE).unwrap();
    assert_eq!(oct.count, 8);
    assert!((oct.area - 8. * (2f64.sqrt() - 1.)).abs() < 1e-14);
    let (m, mut s) = example();
    s.center.x = 0.42;
    assert!(m.bind(s).unwrap().is_none());
    s.center.x = 0.;
    s.angular_velocity.x = 0.01;
    assert!(m.bind(s).unwrap().is_none());
}
#[test]
fn center_can_leave_and_return_with_both_endpoints_inside() {
    let (mut m, mut s) = example();
    m.gravity = DVec3::new(-20., 0., -9.81);
    s.angular_velocity = DVec3::ZERO;
    s.center.x = 0.40;
    s.velocity.x = 1.;
    let b = m.bind(s).unwrap().unwrap();
    assert!(b.sample(0.1).unwrap().pad_center.x < 0.41);
    let FaceBearingAdvance::Exits(hit) = b.advance(0.1).unwrap() else {
        panic!()
    };
    let expected = (1. - 0.6f64.sqrt()) / 20.;
    assert!((hit.seconds - expected).abs() < 5e-15);
}
#[test]
fn rotated_translated_world_and_tangent_frame_preserve_the_same_reaction() {
    let (m, s) = example();
    let q = DQuat::from_rotation_y(-0.32) * DQuat::from_rotation_x(0.44);
    let b = m.bind(s).unwrap().unwrap();
    let t = 0.00375;
    let rot = FrictionlessFaceSupport {
        pad_axes: m.pad_axes.map(|x| q * x),
        gravity: q * m.gravity,
        ..m
    };
    let state = EdgeSupportState {
        center: q * s.center,
        velocity: q * s.velocity,
        orientation: q * s.orientation,
        angular_velocity: q * s.angular_velocity,
    };
    let a = b.sample(t).unwrap();
    let c = rot.bind(state).unwrap().unwrap().sample(t).unwrap();
    assert!(c.center_of_pressure.distance(q * a.center_of_pressure) < 2e-15);
    assert!(
        c.angular_impulse_about_com
            .distance(q * a.angular_impulse_about_com)
            < 2e-16
    );
    assert!((c.overlap.area - a.overlap.area).abs() < 2e-16);
    let mut no_gravity = m;
    no_gravity.gravity = DVec3::X;
    assert!(no_gravity.bind(s).unwrap().is_none());
    let bad = FrictionlessFaceSupport {
        pad_axes: [DVec3::X, DVec3::X],
        ..m
    };
    assert!(bad.bind(s).is_err());
    let bad = FrictionlessFaceSupport {
        inertia_body: DMat3::ZERO,
        ..m
    };
    assert!(bad.bind(s).is_err());
}
#[test]
fn outward_pressure_exit_encloses_the_moving_finite_line_and_rotating_reaction() {
    let (m, s) = example();
    let full = m.bind(s).unwrap().unwrap();
    let (hit, line) = full.departing_deck_edge(0.3).unwrap().unwrap();
    let init = full.sample(hit.seconds).unwrap();
    let bounds = line.enclose(0.0005).unwrap().unwrap();
    let mut cursor = line.trajectory();
    for _ in 0..8 {
        let p = cursor.advance(0.0000625, 0.0000625).unwrap();
        assert!(
            p.signed_tilt > 0.
                && p.normal_force >= bounds.normal_force[0]
                && p.normal_force <= bounds.normal_force[1]
        );
        assert!(
            p.center_along_edge >= bounds.center_along_edge[0]
                && p.center_along_edge <= bounds.center_along_edge[1]
        );
        assert!(p.equivalent_endpoint_force.iter().all(|k| *k > 0.));
        assert!((p.center_of_pressure.x - 0.41).abs() < 1e-14);
        assert!(
            p.state.velocity.length() <= bounds.center_speed_bound
                && p.state.angular_velocity.length() <= bounds.angular_speed_bound
        );
    }
    let p = line.sample(0.035, 0.000125).unwrap();
    assert!(p.sole_normal.x > 1e-4 && p.integrated_force.x > 0.);
    assert!((p.center_of_pressure - p.state.center).dot(p.sole_normal) < 0.);
    assert!(
        p.integrated_force
            .distance(m.mass * (p.state.velocity - init.state.velocity - m.gravity * 0.035))
            < 1e-15
    );
    let near = m.bind_deck_edge(p.state, 1).unwrap().unwrap();
    assert!(near.enclose(0.0005).unwrap().is_some());
    assert!(line.enclose(3.).unwrap().is_none());
    assert!(line.enclose(f64::NAN).is_err());
}
#[test]
fn wrong_tip_direction_and_endpoint_loading_are_declined() {
    let (m, s) = example();
    let (_, edge) = m
        .bind(s)
        .unwrap()
        .unwrap()
        .departing_deck_edge(0.3)
        .unwrap()
        .unwrap();
    let tip = edge.sample(0.03, 0.000125).unwrap();
    assert!(m.bind_deck_edge(tip.state, 0).unwrap().is_none());
    let mut outside = tip.state;
    outside.center.y += 2.;
    assert!(m.bind_deck_edge(outside, 1).unwrap().is_none());
    assert!(m.bind_deck_edge(tip.state, 4).is_err());
}
#[test]
fn two_finite_moving_pads_supply_the_same_wrench_for_distinct_witnessed_loads() {
    let (m, mut s) = example();
    s.center = DVec3::new(0., 0., 0.13);
    s.velocity = DVec3::new(0.28, 0.04, 0.);
    let pads = [
        CoplanarPad {
            center: DVec2::new(-0.14, 0.01),
            half: DVec2::new(0.12, 0.13),
            yaw: 0.02,
            velocity: DVec2::new(-0.2, 0.02),
        },
        CoplanarPad {
            center: DVec2::new(0.14, -0.01),
            half: DVec2::new(0.12, 0.15),
            yaw: -0.06,
            velocity: DVec2::new(0.08, 0.02),
        },
    ];
    let body = m.bind_coplanar(s, pads).unwrap().unwrap();
    let dt = 0.0125;
    let lower = body.advance(dt, Some(0.)).unwrap().unwrap();
    let upper = body.advance(dt, Some(1.)).unwrap().unwrap();
    assert!(upper.selected_fraction_b - lower.selected_fraction_b > 0.3);
    assert_eq!(lower.state.center, upper.state.center);
    assert_eq!(
        lower.angular_impulse_about_com,
        upper.angular_impulse_about_com
    );
    for sample in [lower, upper] {
        assert!(
            sample.outer_fraction_b[0] <= sample.selected_fraction_b
                && sample.outer_fraction_b[1] >= sample.selected_fraction_b
        );
        for k in 0..=50 {
            let t = dt * (k as f64) / 50.;
            let q = DQuat::from_rotation_z(s.angular_velocity.z * t) * s.orientation;
            let c = s.center + s.velocity * t + 0.5 * m.gravity.truncate().extend(0.) * t * t;
            let mut torque = DVec3::ZERO;
            for (p, site) in sample.sites.iter().zip(pads) {
                let arm = q * p.material_center.unwrap();
                let world = (c + arm).truncate();
                let pos =
                    DVec2::from_angle(-site.yaw).rotate(world - site.center - site.velocity * t);
                assert!((site.half - pos.abs()).min_element() >= -1e-12);
                torque += arm.cross(DVec3::Z * p.selected_normal_impulse / dt);
            }
            let h = q * (m.inertia_body * (q.conjugate() * s.angular_velocity));
            assert!(torque.distance(s.angular_velocity.cross(h)) < 1e-10);
        }
    }
}
#[test]
fn removing_a_pad_clears_its_load_while_the_other_carries_the_entire_weight() {
    let (m, mut s) = example();
    s.center = DVec3::new(0.03, 0., 0.13);
    s.velocity = DVec3::ZERO;
    let f = CoplanarPad {
        center: DVec2::ZERO,
        half: DVec2::new(0.15, 0.14),
        yaw: 0.,
        velocity: DVec2::ZERO,
    };
    let two = m.bind_coplanar(s, [f, f]).unwrap().unwrap();
    assert!(
        (two.advance(0.0005, Some(0.26))
            .unwrap()
            .unwrap()
            .selected_fraction_b
            - 0.26)
            .abs()
            < 1e-13
    );
    let remote = CoplanarPad {
        center: DVec2::new(-0.7, 0.),
        ..f
    };
    let one = m
        .bind_coplanar(s, [remote, f])
        .unwrap()
        .unwrap()
        .advance(0.0005, Some(0.26))
        .unwrap()
        .unwrap();
    assert_eq!(one.sites[0].normal_impulse_bounds, [0., 0.]);
    assert_eq!(one.sites[0].material_center, None);
    assert_eq!(one.selected_fraction_b, 1.);
    assert!((one.sites[1].selected_normal_impulse - 0.00981).abs() < 1e-14);
    let no = m.bind_coplanar(s, [remote, remote]).unwrap().unwrap();
    assert!(no.advance(0.0005, None).unwrap().is_none());
}
#[test]
fn uniform_shear_shifts_center_and_delivers_closed_form_deceleration_and_energy() {
    let (mut m, mut s) = example();
    m.gravity = DVec3::new(0.18, 0., -9.81);
    s.velocity = DVec3::new(0.75, 0., 0.);
    s.center.x = 0.;
    s.angular_velocity = DVec3::ZERO;
    let mut pads = [
        CoplanarPad {
            center: DVec2::new(-0.14, 0.),
            half: DVec2::new(0.115, 0.14),
            velocity: DVec2::new(-0.3, 0.),
            yaw: 0.05,
        },
        CoplanarPad {
            center: DVec2::new(0.14, 0.),
            half: DVec2::new(0.11, 0.12),
            velocity: DVec2::new(0.25, 0.),
            yaw: -0.16,
        },
    ];
    let h = 0.0125;
    let u = 0.24;
    let out = m
        .advance_uniform_slide(s, pads, [u, u], h, Some(0.35))
        .unwrap()
        .unwrap();
    let expected = m.gravity.x - u * 9.81;
    assert!((out.shared.state.velocity.x - (0.75 + expected * h)).abs() < 1e-14);
    assert_eq!(out.shared.state.angular_velocity, DVec3::ZERO);
    let travel = out.shared.state.center.x - s.center.x;
    let kinetic =
        0.5 * m.mass * (out.shared.state.velocity.length_squared() - s.velocity.length_squared());
    assert!(
        (kinetic - m.mass * m.gravity.x * travel - out.support_tangent_work
            + out.dissipated_energy)
            .abs()
            < 1e-13
    );
    let mut torque = DVec3::ZERO;
    for site in out.shared.sites {
        if let Some(local) = site.material_center {
            let arm = s.orientation * local;
            torque += arm.cross((DVec3::Z + out.tangent_per_normal) * site.selected_normal_impulse);
        }
    }
    assert!(torque.length() < 1e-13);
    assert!(
        m.advance_uniform_slide(s, pads, [u, u], 0.3, None)
            .unwrap()
            .is_none()
    );
    assert!(
        m.advance_uniform_slide(s, pads, [u, 0.38], h, None)
            .unwrap()
            .is_none()
    );
    pads[0].velocity.y = 0.03;
    assert!(
        m.advance_uniform_slide(s, pads, [u, u], h, None)
            .unwrap()
            .is_none()
    );
}

#[test]
fn a_common_translating_belt_arrests_at_a_fractional_clock_and_preserves_energy() {
    let (mut model, mut start) = example();
    model.gravity = -9.81 * DVec3::Z;
    start.angular_velocity = DVec3::ZERO;
    start.center.x = 0.;
    let axis = DVec2::from_angle(0.29);
    let belt = DVec2::new(0.13, -0.09);
    let mu = 0.36;
    let h = 0.0005;
    let pads = [-0.11, 0.11].map(|x| CoplanarPad {
        center: DVec2::new(x, 0.),
        half: DVec2::new(0.12, 0.135),
        yaw: -x,
        velocity: belt,
    });
    for stop in [0., 0.25e-9, 0.00021381976, h - 0.25e-9] {
        let initial = 9.81 * mu * stop;
        start.velocity = (belt + axis * initial).extend(0.);
        let r = model
            .advance_comoving_stop(start, pads, [mu; 2], h, None)
            .unwrap()
            .unwrap();
        assert!((r.stop_seconds - stop).abs() < 1e-15);
        let rest = r.resting.unwrap();
        assert_eq!(rest.state.velocity, belt.extend(0.));
        assert!(
            rest.state
                .center
                .distance(start.center + (belt * h + axis * 0.5 * initial * stop).extend(0.))
                < 1e-14
        );
        let kinetic = r.sliding.map_or(0., |s| s.dissipated_energy);
        assert!((kinetic - 0.5 * model.mass * initial.powi(2)).abs() < 1e-14);
        assert!(
            (r.sliding.map_or(0., |s| s.shared.normal_impulse) + rest.normal_impulse
                - 9.81 * model.mass * h)
                .abs()
                < 1e-14
        );
    }
    let mut different = pads;
    different[1].velocity += DVec2::X * 0.01;
    assert!(
        model
            .advance_comoving_stop(start, different, [mu; 2], h, None)
            .unwrap()
            .is_none()
    );
    assert!(
        model
            .advance_comoving_stop(start, pads, [0.35, mu], h, None)
            .unwrap()
            .is_none()
    );
    model.gravity.x = 0.01;
    assert!(
        model
            .advance_comoving_stop(start, pads, [mu; 2], h, None)
            .unwrap()
            .is_none()
    );
    model.gravity.x = 0.;
    let edge = pads.map(|p| CoplanarPad {
        center: DVec2::new(-0.1, 0.),
        half: DVec2::new(0.10, 0.2),
        yaw: 0.,
        ..p
    });
    start.velocity = belt.extend(0.);
    assert!(
        model
            .advance_comoving_stop(start, edge, [mu; 2], h, None)
            .unwrap()
            .is_none()
    );
}

#[test]
fn shared_pressure_uses_the_internal_quadratic_extremum_on_both_finite_pads() {
    let (mut model, mut foot) = example();
    model.gravity = DVec3::new(-8., 0., -9.81);
    foot.center = DVec3::new(0., 0., 0.13);
    foot.angular_velocity = DVec3::ZERO;
    foot.orientation = DQuat::IDENTITY;
    let pads = |right| {
        [-0.075, 0.075].map(|y| CoplanarPad {
            center: DVec2::new(right - 0.06, y),
            half: DVec2::new(0.06, 0.04),
            yaw: 0.,
            velocity: DVec2::ZERO,
        })
    };
    let h = 0.002;
    foot.velocity = DVec3::new(0.008, 0., 0.);
    let at = model.bind_coplanar(foot, pads(2e-6)).unwrap().unwrap();
    assert!(at.instantaneous().unwrap().is_some());
    assert!(at.advance(h, None).unwrap().is_none());
    assert!(at.advance(h / 8., None).unwrap().is_some());
    foot.velocity = DVec3::ZERO;
    // Monotone inward motion from 30 nm inside the pressure boundary.
    // A symmetric |acceleration| h²/8 erosion would reject this interval.
    let inward = model
        .bind_coplanar(foot, pads(3e-8))
        .unwrap()
        .unwrap()
        .advance(h, None)
        .unwrap()
        .unwrap();
    assert!(inward.sites.iter().all(|p| p.selected_normal_impulse > 0.));
    assert!((inward.state.center.x + 4. * h * h).abs() < 1e-14);
}
