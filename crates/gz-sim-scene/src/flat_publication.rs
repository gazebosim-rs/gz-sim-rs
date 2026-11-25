// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Publish a dense row of independent canonical models together with their relative link poses.
use super::*;

#[derive(Resource, Default)]
struct Published(bool);

pub(super) fn install(sim: &mut Simulation) {
    sim.world_mut().init_resource::<Published>();
    for phase in [Phase::PhysicsCommit, Phase::Reset] {
        sim.add_systems(
            phase,
            (
                publish.after(sync_bodies),
                (sync_models, sync_local_poses)
                    .chain()
                    .run_if(|published: Res<Published>| !published.0),
                sync_attached_poses,
            )
                .chain(),
        );
    }
}

type Models<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static ChildOf,
        &'static ModelAnchor,
        &'static mut WorldPose,
        &'static mut LocalPose,
    ),
    (Without<BodyHandle>, PoseUpdateFilter),
>;
type Links<'w, 's> = Query<
    'w,
    's,
    (
        &'static EntityId,
        &'static ChildOf,
        &'static WorldPose,
        &'static mut LocalPose,
    ),
    (With<BodyHandle>, Without<ModelAnchor>, PoseUpdateFilter),
>;

type RootFilter = (Without<ChildOf>, Without<ModelAnchor>);
type LocalFilter = (
    With<ChildOf>,
    With<WorldPose>,
    With<LocalPose>,
    PoseUpdateFilter,
);

fn publish(
    mut published: ResMut<Published>,
    roots: Query<(Entity, &WorldPose), RootFilter>,
    all_models: Query<(), With<ModelAnchor>>,
    all_locals: Query<(), LocalFilter>,
    mut models: Models,
    mut links: Links,
) {
    published.0 = false;
    let count = all_models.count();
    if !parallel_pose_updates(count) || all_locals.count() != 2 * count {
        return;
    }
    let Ok((root_entity, &root)) = roots.single() else {
        return;
    };
    let (Ok(mut model_tables), Ok(mut link_tables)) =
        (models.contiguous_iter_mut(), links.contiguous_iter_mut())
    else {
        return;
    };
    let Some((entities, parents, anchors, models, model_locals)) =
        model_tables.find(|t| t.0.len() == count)
    else {
        return;
    };
    let Some((ids, link_parents, link_poses, link_locals)) =
        link_tables.find(|t| t.0.len() == count)
    else {
        return;
    };

    let (models, _) = bevy_ecs::change_detection::ContiguousMut::split(models);
    let (model_locals, _) = bevy_ecs::change_detection::ContiguousMut::split(model_locals);
    let (link_locals, _) = bevy_ecs::change_detection::ContiguousMut::split(link_locals);
    let batch = pose_batch_rows(count);
    let results = bevy_tasks::ComputeTaskPool::get().scope(|scope| {
        for (chunk, ((model_poses, model_locals), link_locals)) in models
            .chunks_mut(batch)
            .zip(model_locals.chunks_mut(batch))
            .zip(link_locals.chunks_mut(batch))
            .enumerate()
        {
            scope.spawn(async move {
                for (offset, ((model, model_local), link_local)) in model_poses
                    .iter_mut()
                    .zip(model_locals)
                    .zip(link_locals)
                    .enumerate()
                {
                    let i = chunk * batch + offset;
                    if parents[i].parent() != root_entity
                        || link_parents[i].parent() != entities[i]
                        || anchors[i].link != ids[i].get()
                    {
                        return false;
                    }
                    let link = &link_poses[i];
                    *model = link.compose(anchors[i].link_to_model);
                    *model_local = local_from_world(root, model);
                    let relative = local_from_world(*model, link);
                    // Link-local transforms on rigid decks often stay bit-for-bit fixed.
                    // The table split already marked the public component changed.
                    if link_local.0.translation.map(f64::to_bits)
                        != relative.0.translation.map(f64::to_bits)
                        || link_local.0.rotation.map(f64::to_bits)
                            != relative.0.rotation.map(f64::to_bits)
                    {
                        *link_local = relative;
                    }
                }
                true
            });
        }
    });
    // Any reordered or reparented row sends the whole publication through the
    // normal systems before another stage observes it. Only derived destinations
    // were written above. All source link observations are still intact.
    published.0 = results.into_iter().all(|ok| ok);
}

#[cfg(test)]
#[path = "../tests/private/flat_publication.rs"]
mod tests;
