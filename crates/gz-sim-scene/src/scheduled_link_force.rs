// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Authored world-frame forces acting at a link COM over half-open simulation tick intervals.
use crate::{BodyHandle, LoadError, Name, PhysicsEngine, joint_posture::PostureActuation};
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, PhysicsBackend};
use gz_sim_core::{EntityId, EntityLookup, Phase, Simulation, StepFault, UpdateInfo};
use std::collections::{BTreeMap, BTreeSet};

pub(crate) fn is_plugin(p: &sdformat::Plugin) -> bool {
    p.name == "gz::sim::systems::ScheduledLinkForce"
        && p.filename == "gz-sim-scheduled-link-force-system"
}
pub(crate) struct Entry {
    link: String,
    begin: u64,
    end: u64,
    force: [f64; 3],
}
fn invalid(entity: &str, reason: &str) -> LoadError {
    LoadError::ScheduledLinkForce {
        entity: entity.to_owned(),
        reason: reason.to_owned(),
    }
}
pub(crate) fn parse(
    world: &sdformat::World,
    engine: PhysicsEngine,
) -> Result<Vec<Entry>, LoadError> {
    let mut plugins = world.plugins.iter().filter(|p| is_plugin(p));
    let Some(p) = plugins.next() else {
        return Ok(vec![]);
    };
    let err = |why: &str| invalid(&world.name, why);
    if plugins.next().is_some()
        || !matches!(engine, PhysicsEngine::Rapier | PhysicsEngine::MultiFidelity)
    {
        return Err(err(
            "expected one scheduled force system using a native rigid backend",
        ));
    }
    let xml = roxmltree::Document::parse(&p.xml).map_err(|e| err(&e.to_string()))?;
    let root = xml.root_element();
    if root.tag_name().namespace().is_some()
        || root
            .attributes()
            .any(|a| a.namespace().is_some() || !matches!(a.name(), "name" | "filename"))
    {
        return Err(err("unexpected plugin attribute"));
    }
    let empty = |n: roxmltree::Node| {
        n.is_comment() || n.is_text() && n.text().unwrap_or_default().trim().is_empty()
    };
    let mut out = Vec::new();
    for row in root.children().filter(|n| !empty(*n)) {
        if !row.is_element()
            || row.tag_name().namespace().is_some()
            || row.tag_name().name() != "force"
            || row.attributes().any(|a| {
                a.namespace().is_some()
                    || !matches!(a.name(), "link" | "begin_tick" | "end_tick" | "xyz")
            })
            || !row.children().all(empty)
        {
            return Err(err(
                "expected force with link, begin_tick, end_tick and xyz attributes",
            ));
        }
        let attr = |name| {
            row.attribute(name)
                .ok_or_else(|| err("incomplete scheduled force"))
        };
        let link = attr("link")?.to_owned();
        let begin = attr("begin_tick")?
            .parse::<u64>()
            .map_err(|_| err("invalid first input tick"))?;
        let end = attr("end_tick")?
            .parse::<u64>()
            .map_err(|_| err("invalid exclusive end tick"))?;
        let values: Vec<_> = attr("xyz")?
            .split_ascii_whitespace()
            .map(|x| {
                x.parse::<f64>()
                    .ok()
                    .filter(|v| v.is_finite())
                    .ok_or_else(|| err("nonfinite world force"))
            })
            .collect::<Result<_, _>>()?;
        let force: [f64; 3] = values
            .try_into()
            .map_err(|_| err("world force needs three components in newtons"))?;
        if link.is_empty() || begin >= end {
            return Err(err("missing link or empty scheduled interval"));
        }
        out.push(Entry {
            link,
            begin,
            end,
            force,
        });
    }
    if out.is_empty() {
        return Err(err("no scheduled forces"));
    }
    Ok(out)
}

#[derive(Clone, Copy)]
struct Link {
    id: u64,
    body: BodyId,
}
struct Cut {
    begin: u64,
    loads: Vec<(Link, [f64; 3])>,
}
#[derive(Resource)]
struct Schedule(Vec<Cut>);
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct ScheduledForces;

pub(crate) fn install(sim: &mut Simulation, entries: Vec<Entry>) -> Result<(), LoadError> {
    if entries.is_empty() {
        return Ok(());
    }
    let world = sim.world();
    let handles: BTreeMap<_, _> = world
        .iter_entities()
        .filter_map(|e| {
            Some((
                e.get::<Name>()?.0.as_str(),
                Link {
                    id: e.get::<EntityId>()?.get(),
                    body: e.get::<BodyHandle>()?.id(),
                },
            ))
        })
        .collect();
    let mut changes = BTreeMap::<u64, Vec<(usize, bool)>>::new();
    let mut rows = Vec::new();
    for e in entries {
        let target = *handles
            .get(e.link.as_str())
            .ok_or_else(|| invalid(&e.link, "missing constructed link"))?;
        if world.resource::<PhysicsBackend>().is_dynamic(target.body) != Some(true) {
            return Err(invalid(&e.link, "expected a dynamic COM target"));
        }
        let i = rows.len();
        rows.push((target, e.force));
        changes.entry(e.begin).or_default().push((i, true));
        changes.entry(e.end).or_default().push((i, false));
    }
    let mut active = BTreeSet::new();
    let mut cuts = Vec::with_capacity(changes.len());
    for (begin, events) in changes {
        for (index, starts) in events {
            if starts {
                active.insert(index);
            } else {
                active.remove(&index);
            }
        }
        let mut sums = BTreeMap::<u64, (Link, [f64; 3])>::new();
        for &index in &active {
            let (link, force) = rows[index];
            let (_, sum) = sums.entry(link.id).or_insert((link, [0.; 3]));
            for (s, f) in sum.iter_mut().zip(force) {
                *s += f;
                if !s.is_finite() {
                    return Err(invalid(
                        "scheduled force",
                        "overlapping forces overflow their target",
                    ));
                }
            }
        }
        cuts.push(Cut {
            begin,
            loads: sums.into_values().filter(|(_, f)| *f != [0.; 3]).collect(),
        });
    }
    sim.world_mut().insert_resource(Schedule(cuts));
    sim.add_systems(
        Phase::PreStep,
        apply.in_set(ScheduledForces).before(PostureActuation),
    );
    Ok(())
}
fn apply(
    clock: Res<UpdateInfo>,
    schedule: Res<Schedule>,
    identities: EntityLookup,
    bodies: Query<&BodyHandle>,
    mut be: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
) {
    if clock.dt.is_zero() {
        return;
    }
    let next = schedule
        .0
        .partition_point(|cut| cut.begin < clock.iteration);
    if next == 0 {
        return;
    }
    for (link, force) in &schedule.0[next - 1].loads {
        if identities
            .get(link.id)
            .and_then(|e| bodies.get(e).ok())
            .map(BodyHandle::id)
            != Some(link.body)
        {
            continue;
        }
        if let Err(e) = be.add_world_wrench(link.body, *force, [0.; 3]) {
            fault.report(e);
            break;
        }
    }
}
