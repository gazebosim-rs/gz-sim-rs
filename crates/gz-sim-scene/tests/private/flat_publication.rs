// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::change_detection::DetectChanges;
use std::fmt::Write;

fn equal(a: WorldPose, b: WorldPose) {
    assert_eq!(
        a.translation.map(f64::to_bits),
        b.translation.map(f64::to_bits)
    );
    assert_eq!(a.rotation.map(f64::to_bits), b.rotation.map(f64::to_bits));
}

#[test]
fn spinning_authored_fleet_keeps_canonical_offsets_twists_change_ticks_and_reset() {
    // Non-identity link and visual frames and angular commands exercise the full
    // native rotation path. The spacing keeps the rotating decks physically disjoint.
    let mut sdf = String::from(
        "<sdf version='1.12'><world name='fleet'><physics name='fixed' type='ignored'><max_step_size>0.01</max_step_size></physics>",
    );
    for i in 0..8193 {
        let x = i * 4;
        write!(sdf, "<model name='m{i}'><pose>{x} 0 0 0 0 0.3</pose><plugin name='gz::sim::systems::PrescribedMotion' filename='gz-sim-prescribed-motion-system'><linear_velocity>0.25 0 0</linear_velocity><angular_velocity>0 0 0.4</angular_velocity></plugin><link name='body'><pose>0.2 -0.1 0.06 0.1 0.2 -0.3</pose><collision name='c'><geometry><box><size>0.3 0.2 0.1</size></box></geometry></collision><visual name='v'><pose>0.4 0 0 0 0.25 0</pose><geometry><box><size>0.3 0.2 0.1</size></box></geometry></visual></link></model>").unwrap();
    }
    sdf.push_str("</world></sdf>");
    let mut sim = load_world_with_sdf_step(
        &sdf,
        "rotating-fleet.sdf",
        "fleet",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let inspect = |sim: &Simulation| {
        let world = sim.world();
        assert_eq!(world.resource::<Published>().0, parallel_pose_updates(8193));
        let backend = world.resource::<PhysicsBackend>();
        let mut rows = Vec::new();
        for link in world.iter_entities().filter(|e| e.contains::<BodyHandle>()) {
            let parent = world.entity(link.get::<ChildOf>().unwrap().parent());
            let root = world.entity(parent.get::<ChildOf>().unwrap().parent());
            let anchor = parent.get::<ModelAnchor>().unwrap();
            let state = backend
                .state(link.get::<BodyHandle>().unwrap().id())
                .unwrap();
            let native = WorldPose {
                translation: state.position,
                rotation: state.orientation,
            };
            let model = native.compose(anchor.link_to_model);
            equal(*link.get::<WorldPose>().unwrap(), native);
            equal(*parent.get::<WorldPose>().unwrap(), model);
            equal(
                parent.get::<LocalPose>().unwrap().0,
                local_from_world(*root.get::<WorldPose>().unwrap(), &model).0,
            );
            equal(
                link.get::<LocalPose>().unwrap().0,
                local_from_world(model, &native).0,
            );
            for child in link.get::<Children>().unwrap().iter() {
                let child = world.entity(child);
                equal(
                    *child.get::<WorldPose>().unwrap(),
                    native.compose(child.get::<LocalPose>().unwrap().0),
                );
                assert!(child.get_ref::<WorldPose>().unwrap().is_changed());
            }
            assert!(parent.get_ref::<WorldPose>().unwrap().is_changed());
            assert!(parent.get_ref::<LocalPose>().unwrap().is_changed());
            assert!(link.get_ref::<LocalPose>().unwrap().is_changed());
            let velocity = link.get::<Velocity>().unwrap();
            assert_eq!(velocity.angular, state.angular_velocity);
            rows.push((link.get::<EntityId>().unwrap().get(), state));
        }
        assert_eq!(rows.len(), 8193);
        rows.sort_by_key(|r| r.0);
        rows
    };
    sim.world_mut().clear_trackers();
    sim.step_exact(1).unwrap();
    let first = inspect(&sim);
    sim.step_exact(4).unwrap();
    assert_ne!(first[0].1.orientation, inspect(&sim)[0].1.orientation);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.world_mut().clear_trackers();
    sim.step_exact(1).unwrap();
    assert_eq!(first, inspect(&sim));
}

#[test]
fn fallback_recomputes_nonflat_rows_and_accepts_restored_topology() {
    let mut sim = Simulation::new(Duration::from_millis(10)).unwrap();
    let pose = WorldPose {
        translation: [1., 2., 3.],
        rotation: DQuat::from_rotation_y(-0.4).to_array(),
    };
    let root = sim.world_mut().spawn(pose).id();
    let mut rows = Vec::new();
    for i in 0..8193 {
        let parent = sim
            .world_mut()
            .spawn((ChildOf(root), pose, LocalPose(pose)))
            .id();
        let link = sim
            .world_mut()
            .spawn((
                BodyHandle(BodyId::from_raw(i)),
                ChildOf(parent),
                WorldPose {
                    translation: [i as f64, 1., 2.],
                    ..pose
                },
                LocalPose(pose),
            ))
            .id();
        let id = assign_id(sim.world_mut(), link).unwrap();
        sim.world_mut().entity_mut(parent).insert(ModelAnchor {
            link: id,
            link_to_model: pose,
        });
        rows.push((parent, link));
    }
    install(&mut sim);
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().resource::<Published>().0,
        parallel_pose_updates(8193)
    );
    let (model, link) = rows[8020];
    let original_local = sim.world().get::<LocalPose>(link).unwrap().0;
    // A reader can edit a component directly between commits. Correct each half
    // independently, including a non-finite quaternion and a different translation.
    sim.world_mut()
        .get_mut::<LocalPose>(link)
        .unwrap()
        .0
        .rotation[1] = f64::NAN;
    sim.step_exact(1).unwrap();
    equal(
        sim.world().get::<LocalPose>(link).unwrap().0,
        original_local,
    );
    sim.world_mut()
        .get_mut::<LocalPose>(link)
        .unwrap()
        .0
        .translation[2] = 246.;
    sim.step_exact(1).unwrap();
    equal(
        sim.world().get::<LocalPose>(link).unwrap().0,
        original_local,
    );
    sim.world_mut().clear_trackers();
    sim.step_exact(1).unwrap();
    let unchanged = sim.world().entity(link).get_ref::<LocalPose>().unwrap();
    equal(unchanged.0, original_local);
    assert!(unchanged.is_changed());
    let (other, _) = rows[310];
    sim.world_mut().entity_mut(model).insert(ChildOf(other));
    sim.step_exact(1).unwrap();
    assert!(!sim.world().resource::<Published>().0);
    let world = sim.world();
    let child = *world.get::<WorldPose>(model).unwrap();
    equal(
        world.get::<LocalPose>(model).unwrap().0,
        local_from_world(*world.get::<WorldPose>(other).unwrap(), &child).0,
    );
    sim.world_mut().entity_mut(model).insert(ChildOf(root));
    let other_root = sim.world_mut().spawn(pose).id();
    sim.step_exact(1).unwrap();
    assert!(!sim.world().resource::<Published>().0);
    sim.world_mut().despawn(other_root);
    let extra = sim
        .world_mut()
        .spawn((ChildOf(root), pose, LocalPose(pose)))
        .id();
    sim.step_exact(1).unwrap();
    assert!(!sim.world().resource::<Published>().0);
    sim.world_mut().despawn(extra);
    #[derive(Component)]
    struct SplitTable;
    sim.world_mut().entity_mut(model).insert(SplitTable);
    sim.step_exact(1).unwrap();
    assert!(!sim.world().resource::<Published>().0);
    sim.world_mut().entity_mut(model).remove::<SplitTable>();
    sim.world_mut().entity_mut(link).insert(SplitTable);
    sim.step_exact(1).unwrap();
    assert!(!sim.world().resource::<Published>().0);
    sim.world_mut().entity_mut(link).remove::<SplitTable>();
    // Both tables performed the same swap-remove and append, restoring row ownership.
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().resource::<Published>().0,
        parallel_pose_updates(8193)
    );
    sim.world_mut().entity_mut(link).insert(ChildOf(other));
    sim.step_exact(1).unwrap();
    assert!(!sim.world().resource::<Published>().0);
    let world = sim.world();
    equal(
        world.get::<LocalPose>(link).unwrap().0,
        local_from_world(
            *world.get::<WorldPose>(other).unwrap(),
            world.get::<WorldPose>(link).unwrap(),
        )
        .0,
    );
    sim.world_mut().entity_mut(link).insert(ChildOf(model));
    let other_id = sim.world().get::<EntityId>(rows[6].1).unwrap().get();
    sim.world_mut().get_mut::<ModelAnchor>(model).unwrap().link = other_id;
    sim.step_exact(1).unwrap();
    assert!(!sim.world().resource::<Published>().0);
    let world = sim.world();
    equal(
        *world.get::<WorldPose>(model).unwrap(),
        world.get::<WorldPose>(rows[6].1).unwrap().compose(pose),
    );
}
