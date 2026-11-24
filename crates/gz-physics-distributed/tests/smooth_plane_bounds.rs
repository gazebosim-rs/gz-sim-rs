// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::physics::{
    Body, BodyWrench, ComponentInfo, GroundPlane, Island, Primitive, Snapshot, Tier,
    potential_contact_groups_in_field, potential_contact_groups_on_smooth_plane,
};
use std::error::Error;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const G: [f64; 3] = [0., 0., -9.81];
fn sphere(id: u64, x: f64) -> Body {
    Body {
        id,
        primitive: Primitive::Sphere,
        size: [0.4; 3],
        position: [x, 0., 0.205],
        linear: [0.; 3],
        angular: [75., -180., 25.],
        orientation: [0.5; 4],
        tier: Tier::Dynamic,
        friction: 0.,
        mass: 1.,
    }
}
fn singles(b: &[Body]) -> Vec<ComponentInfo> {
    b.iter()
        .map(|b| ComponentInfo {
            ids: vec![b.id],
            contact_history: true,
        })
        .collect()
}
#[test]
fn rotating_spheres_follow_the_tangential_envelope_during_loaded_native_ground_contact()
-> Result<()> {
    let bodies = vec![sphere(1, 0.), sphere(2, 3.4)];
    let mut native = Island::restore(&Snapshot {
        bodies: bodies.clone(),
        ground: Some(GroundPlane::new(0.)?),
        ..Default::default()
    })?;
    let groups = singles(&bodies);
    let mut support = 0;
    for tick in 0..400 {
        let before = native.snapshot();
        let sideways = if tick % 100 < 50 { 32. } else { -32. };
        let f: Vec<_> = bodies
            .iter()
            .map(|b| BodyWrench {
                id: b.id,
                force: [sideways, 9., -75.],
                torque: [42., -17., 6.],
            })
            .collect();
        assert_eq!(
            potential_contact_groups_on_smooth_plane(&before, &groups, &[], G, &f)?,
            [vec![0], vec![1]]
        );
        assert_eq!(
            potential_contact_groups_in_field(&before, &groups, &[], G, &f)?,
            [vec![0, 1]]
        );
        native.step_refined_in_field(G, &f, 10)?;
        let after = native.snapshot();
        assert!(after.impulses.is_empty());
        support += after
            .supports
            .iter()
            .filter(|s| s.normal_impulse > 0.)
            .count();
        for (a, b) in before.bodies.iter().zip(after.bodies) {
            let distance = (a.position[0] - b.position[0]).hypot(a.position[1] - b.position[1]);
            let reach = a.linear[0].hypot(a.linear[1]) * 0.01 + sideways.hypot(9.) * 0.0001;
            assert!(
                distance <= reach + 1e-11,
                "tick {tick}: {distance} > {reach}"
            );
        }
    }
    assert!(support > 780);
    Ok(())
}
#[test]
fn finite_hits_restore_full_reach_for_secondary_partners_and_exclusions_keep_the_old_bound()
-> Result<()> {
    let mut scene = Snapshot {
        bodies: vec![sphere(1, 0.), sphere(2, 0.45), sphere(3, 3.8)],
        ground: Some(GroundPlane::new(0.)?),
        ..Default::default()
    };
    let groups = singles(&scene.bodies);
    let swept = |s: &Snapshot, c: &[ComponentInfo]| {
        potential_contact_groups_on_smooth_plane(s, c, &[], G, &[])
    };
    assert_eq!(swept(&scene, &groups)?, [vec![0], vec![1], vec![2]]);
    scene.bodies[0].linear = [8., 0., 0.];
    assert_eq!(swept(&scene, &groups)?, [vec![0, 1, 2]]);
    scene.bodies[0].linear = [0.; 3];
    scene.bodies[1].position[0] = 3.;
    scene.bodies[2].position[0] = 6.;
    for plane in [
        GroundPlane::with_friction(0., 0.4)?,
        GroundPlane::oriented([0.; 3], [0.6, 0., 0.8], 0.)?,
    ] {
        scene.ground = Some(plane);
        assert_eq!(
            swept(&scene, &groups)?,
            potential_contact_groups_in_field(&scene, &groups, &[], G, &[])?
        );
    }
    scene.ground = Some(GroundPlane::new(0.)?);
    for b in &mut scene.bodies {
        b.friction = 0.4;
    }
    assert_eq!(
        swept(&scene, &groups)?,
        potential_contact_groups_in_field(&scene, &groups, &[], G, &[])?
    );
    for b in &mut scene.bodies {
        b.friction = 0.;
    }
    let warm = [
        ComponentInfo {
            ids: vec![1, 2],
            contact_history: true,
        },
        groups[2].clone(),
    ];
    assert_eq!(
        swept(&scene, &warm)?,
        potential_contact_groups_in_field(&scene, &warm, &[], G, &[])?
    );
    Ok(())
}
