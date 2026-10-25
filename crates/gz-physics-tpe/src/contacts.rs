// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::Motion;
use bevy_ecs::prelude::*;
use gz_sim_core::{Phase, Simulation, StepFault};
use parry3d_f64::{
    bounding_volume::Aabb,
    math::{Pose, Rotation, Vector},
    partitioning::Bvh,
};
use std::collections::BTreeMap;

/// Aggregate model-local bounds, after composing fixed link/collision frames.
#[derive(Component, Clone, Debug)]
pub struct ModelBounds {
    local: Aabb,
    is_static: bool,
    mask: u16,
}

impl ModelBounds {
    /// Whether this model is excluded from dynamic contact pairs.
    pub fn is_static(&self) -> bool {
        self.is_static
    }

    pub fn new(
        min: [f64; 3],
        max: [f64; 3],
        is_static: bool,
        mask: u16,
    ) -> Result<Self, ContactError> {
        if min.iter().chain(&max).any(|v| !v.is_finite())
            || min.iter().zip(max).any(|(min, max)| *min > max)
        {
            return Err(ContactError::InvalidBounds);
        }
        Ok(Self {
            local: Aabb::new(Vector::from_array(min), Vector::from_array(max)),
            is_static,
            mask,
        })
    }
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum ContactError {
    #[error("model bounds must be finite and ordered")]
    InvalidBounds,
    #[error("TPE bounds overflow for Bevy model {0}")]
    Overflow(Entity),
    #[error("TPE contact count exceeds configured limit {0}")]
    LimitExceeded(usize),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contact {
    pub models: [Entity; 2],
    pub point: [f64; 3],
}

/// Maximum output pairs per collection; exceeding it invalidates the run.
#[derive(Resource)]
pub struct ContactLimit(pub usize);

impl Default for ContactLimit {
    fn default() -> Self {
        Self(1_000_000)
    }
}

/// Current model-level overlap points. Query order is unspecified.
#[derive(Resource, Default)]
pub struct Contacts(Vec<Contact>);

impl Contacts {
    pub fn as_slice(&self) -> &[Contact] {
        &self.0
    }
}

struct Record {
    visited: bool,
    entity: Entity,
    bounds: Aabb,
    is_static: bool,
    mask: u16,
}

#[derive(Resource, Default)]
struct ContactIndex {
    tree: Bvh,
    models: BTreeMap<u32, Record>,
    scratch: Vec<Contact>,
}

pub(super) fn install(sim: &mut Simulation) {
    sim.world_mut().init_resource::<Contacts>();
    sim.world_mut().init_resource::<ContactIndex>();
    sim.world_mut().init_resource::<ContactLimit>();
    sim.add_systems(Phase::PhysicsCommit, collect);
    sim.add_systems(Phase::Reset, collect.after(crate::MotionReset));
}

fn collect(
    models: Query<(Entity, &ModelBounds, &Motion)>,
    mut index: ResMut<ContactIndex>,
    mut contacts: ResMut<Contacts>,
    fault: Res<StepFault>,
    limit: Res<ContactLimit>,
) {
    let ContactIndex {
        tree,
        models: indexed,
        scratch,
    } = &mut *index;
    for record in indexed.values_mut() {
        record.visited = false;
    }
    for (entity, model, motion) in &models {
        let state = motion.state();
        let rotation = Rotation::from_xyzw(
            state.orientation[0],
            state.orientation[1],
            state.orientation[2],
            state.orientation[3],
        );
        let pose = Pose::from_parts(Vector::from_array(state.position), rotation);
        let bounds = model.local.transform_by(&pose);
        if !bounds.mins.is_finite() || !bounds.maxs.is_finite() {
            fault.report(ContactError::Overflow(entity));
            return;
        }
        let id = entity.index_u32();
        if indexed.get(&id).is_none_or(|old| old.bounds != bounds) {
            tree.insert(bounds, id);
        }
        indexed.insert(
            id,
            Record {
                visited: true,
                entity,
                bounds,
                is_static: model.is_static,
                mask: model.mask,
            },
        );
    }
    indexed.retain(|id, record| {
        if record.visited {
            true
        } else {
            tree.remove(*id);
            false
        }
    });
    scratch.clear();
    for (id, a) in &*indexed {
        if a.is_static {
            continue;
        }
        for other in tree.intersect_aabb(&a.bounds) {
            if other == *id {
                continue;
            }
            let b = &indexed[&other];
            if (!b.is_static && other < *id) || a.mask & b.mask == 0 {
                continue;
            }
            // The tree may retain a larger envelope after a model's bounds shrink.
            if let Some(overlap) = a.bounds.intersection(&b.bounds) {
                if scratch.len() == limit.0 {
                    fault.report(ContactError::LimitExceeded(limit.0));
                    return;
                }
                scratch.push(Contact {
                    models: if *id < other {
                        [a.entity, b.entity]
                    } else {
                        [b.entity, a.entity]
                    },
                    point: std::array::from_fn(|i| overlap.mins[i].midpoint(overlap.maxs[i])),
                });
            }
        }
    }
    std::mem::swap(&mut contacts.0, scratch);
}
