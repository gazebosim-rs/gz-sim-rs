// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::{reserve_id, restore_id};

#[test]
fn dense_model_publication_follows_the_live_link_across_reordering_and_replacement() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let offset = WorldPose {
        translation: [0.25, -0.1, 0.35],
        rotation: DQuat::from_rotation_y(-0.75).to_array(),
    };
    let mut owners = Vec::new();
    // An additional table grows in descending public ID order after replacement.
    #[derive(Component)]
    struct RestoredLink;
    for i in 0..9000 {
        let pose = WorldPose {
            translation: [f64::from(i), 2., -3.],
            rotation: DQuat::from_rotation_z(0.4).to_array(),
        };
        let link = sim.world_mut().spawn(pose).id();
        let id = assign_id(sim.world_mut(), link).unwrap();
        let model = sim
            .world_mut()
            .spawn((
                ModelAnchor {
                    link: id,
                    link_to_model: offset,
                },
                pose,
            ))
            .id();
        owners.push((model, link));
    }
    sim.add_systems(Phase::PhysicsCommit, sync_models);
    sim.add_systems(Phase::Reset, sync_models);
    sim.step_exact(1).unwrap();

    for (model, link) in owners.iter_mut().rev().step_by(3) {
        let reservation = reserve_id(sim.world(), *link).unwrap();
        let mut pose = *sim.world().get::<WorldPose>(*link).unwrap();
        pose.translation[1] += 5.;
        pose.rotation = DQuat::from_rotation_x(-0.8).to_array();
        sim.world_mut().despawn(*link);
        *link = sim.world_mut().spawn((pose, RestoredLink)).id();
        restore_id(sim.world_mut(), *link, &reservation).unwrap();
        assert_eq!(
            sim.world().get::<ModelAnchor>(*model).unwrap().link,
            reservation.get()
        );
    }
    sim.step_exact(1).unwrap();
    sim.reset().unwrap();
    for (model, link) in owners {
        let source = sim.world().get::<WorldPose>(link).unwrap();
        let published = sim.world().get::<WorldPose>(model).unwrap();
        let rotation = DQuat::from_array(source.rotation);
        let translation = DVec3::from_array(source.translation)
            + rotation * DVec3::from_array(offset.translation);
        assert_eq!(published.translation, translation.to_array());
        assert_eq!(
            published.rotation,
            (rotation * DQuat::from_array(offset.rotation)).to_array()
        );
    }
}
