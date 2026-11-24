// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "distributed-physics")]
#[allow(dead_code)]
#[path = "../examples/support/rotating_pad.rs"]
mod fixture;
use fixture::{CABLE, Rotor};
use glam::{DQuat, DVec3};
use gz_sim_scene::deformable::articulated_impact::distributed_rotor_material_contact_spec;
use std::time::Duration;
#[test]
fn actual_rotor_construction_and_complete_first_public_cut_match_independent_rigid_cable_solution()
{
    let seed = distributed_rotor_material_contact_spec(CABLE, "rotating_pad_cable.sdf", "default")
        .unwrap();
    let mut ordinary = Rotor::new(CABLE).unwrap();
    let pivot = DVec3::new(0., 0., 0.62);
    let axis = DVec3::Z;
    let initial = ordinary.state();
    assert_eq!(seed.step, Duration::from_millis(400));
    assert_eq!(seed.material.particles(), ordinary.material().particles());
    assert_eq!(seed.pad.link.id, ordinary.link);
    assert_eq!(seed.pad.link.position, initial.position);
    assert_eq!(seed.pad.link.orientation, initial.orientation);
    assert_eq!(seed.pad.link.velocity, initial.velocity);
    assert_eq!(seed.pad.link.angular_velocity, initial.angular_velocity);
    assert_eq!(seed.pad.anchor, pivot.to_array());
    assert_eq!(seed.pad.axis, axis.to_array());
    assert_eq!((seed.pad.lower, seed.pad.upper), (-100., 100.));
    assert_eq!(
        seed.pad.link.inertia_tensor,
        [0.028, 0.036, 0.032, 0., 0., 0.]
    );
    assert_eq!(seed.pad.link.radius, 0.035);
    assert_eq!(seed.pad.link.mass, 1.2);
    assert_eq!(seed.material_name, "material");
    let dt = 0.4;
    let r = 0.45;
    let w = 4.;
    let inflated = 0.04;
    let target_angle = 0.8;
    let root = (target_angle - 2. * f64::asin(inflated / (2. * r))) / w;
    let t = (root * 1e9).round() / 1e9;
    let at = |angle: f64| pivot + DQuat::from_rotation_z(angle) * (r * DVec3::X);
    let pad = at(w * t);
    let target = at(target_angle);
    let n = (target - pad).normalize();
    let l = (pad - pivot).cross(n).z;
    let inertia = 0.032 + 1.2 * r * r;
    let impulse = (1. + 0.55) * w * l / (1. / 0.4 + l * l / inertia);
    let end_w = w - impulse * l / inertia;
    let theta = w * t + end_w * (dt - t);
    let end = at(theta);
    let velocity = end_w * axis.cross(end - pivot);
    let material_v = impulse / 0.4 * n;
    let before = ordinary.material().particles().to_vec();
    ordinary.advance(1).unwrap();
    let observed = ordinary.state();
    assert!(DVec3::from(observed.position).distance(end) < 2e-9);
    assert!(DVec3::from(observed.velocity).distance(velocity) < 3e-9);
    assert!(DVec3::from(observed.angular_velocity).distance(end_w * axis) < 3e-9);
    assert!(
        DQuat::from_array(observed.orientation)
            .dot(DQuat::from_rotation_z(theta))
            .abs()
            > 1. - 1e-12
    );
    let mut max_dx = 0f64;
    for (p, cold) in ordinary.material().particles().iter().zip(before) {
        max_dx = max_dx.max(
            DVec3::from(p.position).distance(DVec3::from(cold.position) + (dt - t) * material_v),
        );
        assert!(DVec3::from(p.velocity).distance(material_v) < 3e-9);
    }
    assert!(max_dx < 2e-9);
    println!(
        "Physical cut {t:.9} s, exact onset {root:.12} s, axial moment {moment:.12} N m s, impulse {impulse:.12} N s. At {dt} s, rotor link center {end:?} m, COM velocity {velocity:?} m/s, phase {theta:.12} rad, spin {end_w:.12} rad/s, both cable nodes velocity {material_v:?} m/s and max endpoint discrepancy {max_dx:e} m.",
        moment = -impulse * l
    );
}
#[test]
fn unrepresentable_motion_or_material_fails_the_public_construction() {
    for xml in [
        CABLE.replace("<gravity>0 0 0", "<gravity>0 0 -9.81"),
        CABLE.replace("first_impact", "recurrent_contact"),
        CABLE.replace("<upper>100</upper>", "<upper>1</upper>"),
        CABLE.replace("<angular_velocity>0 0 4", "<angular_velocity>1 0 4"),
        CABLE.replace(
            "<collision name=\"pad_collider\">",
            "<collision name=\"pad_collider\"><pose>0.01 0 0 0 0 0</pose>",
        ),
        CABLE.replace(
            "<link name=\"pad\">",
            "<link name=\"pad\"><wind>true</wind>",
        ),
        CABLE.replace("friction=\"0\"", "friction=\"0.2\""),
    ] {
        assert!(
            distributed_rotor_material_contact_spec(&xml, "invalid.sdf", "default").is_err(),
            "{xml}"
        );
    }
}
