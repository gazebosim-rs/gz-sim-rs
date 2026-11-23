// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn body(id: u64, x: f64) -> Body {
    Body {
        id,
        position: [x, 0., 1.],
        linear: [0.; 3],
        angular: [0.; 3],
        friction: 0.35,
        ..fixture()[1].clone()
    }
}

#[test]
fn cold_seed_layouts_preserve_order_capacity_and_friction() {
    for (index, grouped) in [
        NativeSeedLayout::Standard32,
        NativeSeedLayout::Extended64,
        NativeSeedLayout::Canonical64,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(
            bincode::serialize(&grouped).unwrap(),
            (index as u32).to_le_bytes()
        );
        let point = grouped.with_friction(NativeFrictionModel::PerPoint);
        assert_eq!(
            bincode::serialize(&point).unwrap(),
            ((index + 3) as u32).to_le_bytes()
        );
        assert_eq!(point.with_friction(NativeFrictionModel::Grouped), grouped);
        assert_eq!(point.is_canonical(), grouped.is_canonical());
        assert_eq!(point.profile().capacity_profile(), grouped.profile());
        assert_eq!(point.profile().friction(), NativeFrictionModel::PerPoint);
        let cold = NativeSeedLayout::from(point.profile());
        assert!(!cold.is_canonical());
        assert_eq!(cold.profile(), point.profile());
    }
}

#[test]
fn new_birth_import_edit_split_rejoin_and_restore_preserve_each_islands_model()
-> Result<(), WorldError> {
    let seed = Snapshot {
        bodies: vec![body(1, 0.), body(2, 64.)],
        ground: Some(GroundPlane::new(0.)?),
        ..Default::default()
    };
    for capacity in [
        NativeSeedLayout::Standard32,
        NativeSeedLayout::Extended64,
        NativeSeedLayout::Canonical64,
    ] {
        for friction in [NativeFrictionModel::Grouped, NativeFrictionModel::PerPoint] {
            let layout = capacity.with_friction(friction);
            let profile = layout.profile();
            let mut world = Partition::restore_with_layout(&seed, layout)?;
            assert_eq!(
                world.solvers.len(),
                if layout.is_canonical() { 1 } else { 2 }
            );
            world.step_scheduled([0.; 3], &[], SolveSchedule::Islands(&[]))?;
            assert_eq!(world.solvers.len(), 2);
            world.import(body(3, -32.))?;
            world.set_velocity(3, [0.1, 0., 0.], [0., 0., 0.2])?;
            world.step()?;
            world.retire(3)?;
            let current = world.snapshot();
            assert_eq!(current.bodies.len(), 2);
            // Warm each disjoint source in its own Bevy world. A worker may
            // otherwise coalesce its solvers before the explicit join is tested.
            let loaded = |member: Body| -> Result<Partition, WorldError> {
                let mut single = Partition::restore_with_layout(
                    &Snapshot {
                        tick: current.tick,
                        ground: current.ground,
                        bodies: vec![member],
                        ..Default::default()
                    },
                    layout,
                )?;
                for _ in 0..65 {
                    single.step_in_field([0., 0., -9.81], &[])?;
                }
                assert!(single.solvers[0].committed.contact_history);
                Ok(single)
            };
            let mut world = loaded(current.bodies[0].clone())?;
            let mut donor = loaded(current.bodies[1].clone())?;
            let incoming = donor.solvers.pop().expect("loaded donor");
            world.check_insert_component(&incoming)?;
            world.insert_validated(incoming);
            assert_eq!(world.solvers.len(), 2);
            let joined = world.joined_solver(&[0, 1])?;
            assert_eq!(joined.profile, profile);
            world.solvers = vec![joined];
            world.publish(world.committed.tick);
            world.step_scheduled([0.; 3], &[], SolveSchedule::Islands(&[]))?;
            assert_eq!(world.solvers.len(), 2, "loaded split under {layout:?}");
            for island in &world.solvers {
                let image = island.solver_image(&[], profile)?;
                assert!(image.matches_profile(profile));
                let restored = Island::restore_solver_with_profile(&image, profile)?;
                assert_eq!(restored.snapshot(), island.snapshot());
            }
            let mut image = world.image()?;
            assert_eq!(image.profile, profile);
            let foreign_model = if friction == NativeFrictionModel::Grouped {
                NativeFrictionModel::PerPoint
            } else {
                NativeFrictionModel::Grouped
            };
            let other = profile.with_friction(foreign_model);
            let before = world.snapshot();
            image.profile = other;
            assert!(Partition::from_image(&image).is_err());
            let incoming = Island::restore_with_profile(
                &Snapshot {
                    tick: before.tick,
                    ground: before.ground,
                    bodies: vec![body(4, 10.)],
                    ..Default::default()
                },
                other,
            )?;
            assert!(world.check_insert_component(&incoming).is_err());
            assert_eq!(world.snapshot(), before);
            image.profile = profile;
            let mut restored = Partition::from_image(&image)?;
            world.step()?;
            restored.step()?;
            assert_eq!(world.snapshot(), restored.snapshot());
        }
    }
    Ok(())
}

#[test]
fn reversing_public_ids_applies_the_opposite_reaction_to_the_other_lever() {
    use gz_physics_multifidelity::{RigidContactPatch, RigidPatchPoint};
    for tangent in [None, Some([0.3, -0.1, 0.])] {
        let point = RigidPatchPoint {
            lever_a: [1., 2., 3.],
            lever_b: [4., 5., 6.],
            normal_impulse: 2.,
            tangent_impulse_on_a: tangent,
        };
        let native = RigidContactPatch {
            a: BodyId::from_raw(0),
            b: BodyId::from_raw(1),
            normal_on_a: [0., 0., 1.],
            points: vec![point],
        };
        let direct = FiniteContactPatch::from_native(native.clone(), 3, 9);
        assert_eq!((direct.a, direct.b), (3, 9));
        assert_eq!(direct.points[0], point);
        let swapped = FiniteContactPatch::from_native(native, 9, 3);
        assert_eq!((swapped.a, swapped.b), (3, 9));
        assert_eq!(swapped.normal_on_a, [0., 0., -1.]);
        assert_eq!(swapped.points[0].lever_a, point.lever_b);
        assert_eq!(swapped.points[0].lever_b, point.lever_a);
        assert_eq!(swapped.points[0].normal_impulse, point.normal_impulse);
        assert_eq!(
            swapped.points[0].tangent_impulse_on_a,
            tangent.map(|t| t.map(|v| -v))
        );
    }
}
