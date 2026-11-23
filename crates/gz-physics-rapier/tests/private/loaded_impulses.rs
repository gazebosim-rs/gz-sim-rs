// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn reported_stack_load_uses_solved_points_and_preserves_the_native_cache() {
    let h = std::time::Duration::from_millis(1);
    let mut world = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    world.set_partition_step(h).unwrap();
    world.set_partition_ground_material(0., 0.5).unwrap();
    let mut ids = Vec::new();
    for (m, z) in [(1., 0.64 + 0.03 * 2.), (2., 2.14 + 0.06 * 5.)] {
        ids.push(
            world
                .spawn_partition_box(
                    BodySpec {
                        shape: Shape::Box {
                            half_extents: [0.275, 0.275, 0.2],
                        },
                        position: [-28., 32., z],
                        velocity: [0.; 3],
                        mass: Some(m),
                    },
                    RotationSpec::default(),
                    false,
                    1.,
                )
                .unwrap(),
        );
    }
    let advance = |world: &mut RapierBackend| {
        world.set_partition_gravity([0., 0., -9.81]).unwrap();
        world.step(h).unwrap();
        world.set_partition_gravity([0.; 3]).unwrap();
    };
    for _ in 0..2500 {
        advance(&mut world);
    }
    let native = world.checkpoint_boxes().unwrap();
    let retained = world.partition_warmstart();
    let mut resumed = RapierBackend::restore_trusted_boxes(&native).unwrap();
    assert!(retained.1.iter().any(|&n| f64::from_bits(n) > 0.));
    assert_eq!(retained, resumed.partition_warmstart());
    let mut legacy = 0.;
    let mut support = 0.;
    let mut reaction = 0.;
    let before = world.state(ids[1]).unwrap().velocity[2];
    for _ in 0..500 {
        advance(&mut world);
        advance(&mut resumed);
        let raw = world
            .world
            .narrow_phase
            .contact_pairs()
            .find(|p| {
                world.world.colliders[p.collider1].parent().is_some()
                    && world.world.colliders[p.collider2].parent().is_some()
            })
            .unwrap();
        legacy += raw.total_impulse_magnitude();
        let published = world.rigid_contact_observations()[0].normal_impulse;
        assert_eq!(
            published.to_bits(),
            resumed.rigid_contact_observations()[0]
                .normal_impulse
                .to_bits()
        );
        reaction += published;
        let floor = world.partition_ground_contacts()[0].normal_impulse;
        assert_eq!(
            floor.to_bits(),
            resumed.partition_ground_contacts()[0]
                .normal_impulse
                .to_bits()
        );
        support += floor;
        for &id in &ids {
            assert_eq!(world.state(id), resumed.state(id));
        }
    }
    assert_eq!(world.partition_warmstart(), resumed.partition_warmstart());
    let net_force = 2. * 9.81 + 2. * (world.state(ids[1]).unwrap().velocity[2] - before) / 0.5;
    assert!((reaction / 0.5 - net_force).abs() < 1e-4);
    assert!((support / 0.5 - 3. * 9.81).abs() < 1e-4);
    assert!(legacy / 0.5 - net_force > 5.);
    eprintln!(
        "vertical load {net_force:.9} N, selected normal {:.9} N, all retained points {:.9} N, floor {:.9} N",
        reaction / 0.5,
        legacy / 0.5,
        support / 0.5
    );
}

#[test]
fn empty_contact_reports_positive_zero() {
    let pair = rapier3d_f64::geometry::ContactPair::default();
    assert_eq!(normal_impulse(&pair).to_bits(), 0.0_f64.to_bits());
}
