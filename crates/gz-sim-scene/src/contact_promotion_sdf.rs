// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Authored start tier and predictive contact policy for floating trees.
use crate::{
    ArticulationTier, EntityKind, LoadError, Name, PhysicsEngine, articulation, contact_promotion,
};
use bevy_ecs::prelude::*;
use gz_physics::PhysicsReset;
use gz_sim_core::{EntityId, Phase, Simulation, StepFault, entity_by_id};
use std::collections::{BTreeMap, BTreeSet};
use std::time::Duration;

pub(super) fn is_plugin(p: &sdformat::Plugin) -> bool {
    p.name == "gz::sim::systems::ContactPromotion"
        && p.filename == "gz-sim-contact-promotion-system"
}
pub(super) struct Plan {
    model: String,
    obstacles: Vec<String>,
    start: ArticulationTier,
}
pub(super) fn parse(w: &sdformat::World, engine: PhysicsEngine) -> Result<Vec<Plan>, LoadError> {
    let mut plans = Vec::new();
    let mut used = BTreeSet::new();
    let invalid = |reason: &str| LoadError::ContactPromotion {
        entity: w.name.clone(),
        reason: reason.into(),
    };
    for p in w.plugins.iter().filter(|p| is_plugin(p)) {
        if engine != PhysicsEngine::MultiFidelity {
            return Err(invalid("requires the multifidelity engine"));
        }
        let doc = roxmltree::Document::parse(&p.xml).map_err(|e| invalid(&e.to_string()))?;
        let root = doc.root_element();
        if root.tag_name().name() != "plugin"
            || root.namespaces().next().is_some()
            || root.attributes().len() != 2
            || root.attribute("filename") != Some(&p.filename)
            || root.attribute("name") != Some(&p.name)
        {
            return Err(invalid("invalid plugin attributes"));
        }
        let (mut model, mut obstacles, mut start) = (None, Vec::new(), None);
        for n in root.children() {
            if n.is_comment() || n.is_text() && n.text().unwrap_or_default().trim().is_empty() {
                continue;
            }
            let value = n.text().unwrap_or_default().trim();
            if !n.is_element()
                || n.namespaces().next().is_some()
                || n.attributes().len() != 0
                || !n.children().all(|x| x.is_text())
                || value.is_empty()
            {
                return Err(invalid(
                    "expected plain model, obstacle or initial_tier fields",
                ));
            }
            match n.tag_name().name() {
                "model" if model.is_none() => model = Some(value.to_owned()),
                "obstacle" => obstacles.push(value.to_owned()),
                "initial_tier" if start.is_none() => {
                    start = Some(match value {
                        "prescribed" => ArticulationTier::Prescribed,
                        "dynamic" => ArticulationTier::Dynamic,
                        _ => return Err(invalid("initial_tier must be prescribed or dynamic")),
                    })
                }
                _ => return Err(invalid("duplicate or unknown policy field")),
            }
        }
        let model = model.ok_or_else(|| invalid("missing scoped model name"))?;
        obstacles.sort_unstable();
        if obstacles.is_empty()
            || obstacles.windows(2).any(|a| a[0] == a[1])
            || !used.insert(model.clone())
        {
            return Err(invalid(
                "each model needs a single policy with distinct obstacle links",
            ));
        }
        plans.push(Plan {
            model,
            obstacles,
            start: start.unwrap_or(ArticulationTier::Dynamic),
        });
    }
    Ok(plans)
}
#[derive(Clone)]
struct InitialPolicy {
    model: u64,
    obstacles: Vec<u64>,
    start: ArticulationTier,
}
#[derive(Resource)]
pub(super) struct Authored {
    plans: Vec<InitialPolicy>,
    dt: Duration,
}
pub(super) fn install(sim: &mut Simulation, plans: Vec<Plan>) -> Result<(), LoadError> {
    if plans.is_empty() {
        return Ok(());
    }
    let names: BTreeMap<_, _> = sim
        .world()
        .iter_entities()
        .filter_map(|e| {
            Some((
                e.get::<Name>()?.0.clone(),
                (e.get::<EntityId>()?.get(), *e.get::<EntityKind>()?),
            ))
        })
        .collect();
    let mut bound = Vec::new();
    for p in plans {
        let err = |reason: String| LoadError::ContactPromotion {
            entity: p.model.clone(),
            reason,
        };
        let find = |name: &str, kind| {
            names
                .get(name)
                .filter(|(_, k)| *k == kind)
                .map(|v| v.0)
                .ok_or_else(|| err(format!("unknown scoped name or wrong entity kind: {name}")))
        };
        let model = find(&p.model, EntityKind::Model)?;
        let obstacles = p
            .obstacles
            .iter()
            .map(|n| find(n, EntityKind::Link))
            .collect::<Result<Vec<_>, _>>()?;
        contact_promotion::configure(sim, model, obstacles.clone())
            .map_err(|e| err(e.to_string()))?;
        crate::set_model_fidelity(sim, model, p.start).map_err(|e| err(e.to_string()))?;
        bound.push(InitialPolicy {
            model,
            obstacles,
            start: p.start,
        });
    }
    let dt = sim.step_duration();
    sim.world_mut()
        .insert_resource(Authored { plans: bound, dt });
    sim.add_systems(Phase::Reset, restore.after(PhysicsReset));
    Ok(())
}
fn restore(world: &mut World) {
    world.resource_scope(|world, authored: Mut<Authored>| {
        for p in &authored.plans {
            // A bare clock reset may intentionally leave a removed model absent.
            // A loaded-world reset restores public identities before this phase.
            if entity_by_id(world, p.model).is_none() {
                continue;
            }
            let active: Vec<_> = p
                .obstacles
                .iter()
                .copied()
                .filter(|id| entity_by_id(world, *id).is_some())
                .collect();
            if active.is_empty() {
                continue;
            }
            world
                .resource_mut::<contact_promotion::Policies>()
                .0
                .insert(p.model, active.into());
            if let Err(e) = articulation::switch(world, p.model, p.start, authored.dt) {
                world.resource::<StepFault>().report(e);
                break;
            }
        }
    });
}
