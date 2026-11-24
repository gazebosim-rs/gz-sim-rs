// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare one uninterrupted native scene with an independent native world per assembly.
#[path = "support/native_contact_control.rs"]
mod native;
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod scene;
use clap::Parser;
use gz_physics_distributed::physics::{ContactImpulse, GroundPlane};
use serde_json::json;
use std::{collections::BTreeMap, error::Error};
#[derive(Parser)]
struct Args {
    #[arg(long, value_enum, default_value = "adjacent")]
    layout: scene::Layout,
    #[arg(long, default_value_t = 100_100)]
    ticks: u64,
    /// Measure the actual finite contact graph of the uninterrupted whole scene.
    #[arg(long)]
    native_graph: bool,
    /// Save the finite graph, floor receipts and native body geometry at these ticks.
    #[arg(long, value_delimiter = ',', requires = "native_graph")]
    inspect_at: Vec<u64>,
}
fn graph(contacts: &[ContactImpulse]) -> Vec<Vec<u64>> {
    let mut parents = BTreeMap::new();
    fn root(parents: &BTreeMap<u64, u64>, mut id: u64) -> u64 {
        while let Some(&p) = parents.get(&id) {
            if p == id {
                break;
            }
            id = p;
        }
        id
    }
    for p in contacts {
        let a = root(&parents, p.a);
        let b = root(&parents, p.b);
        parents.insert(a.max(b), a.min(b));
        parents.entry(a.min(b)).or_insert(a.min(b));
    }
    let mut graphs: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
    for &id in parents.keys() {
        graphs.entry(root(&parents, id)).or_default().push(id);
    }
    let mut components: Vec<_> = graphs.into_values().collect();
    components.sort_unstable_by_key(|v| (std::cmp::Reverse(v.len()), v[0]));
    components
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let groups = scene::seeds(args.layout);
    let all = groups.concat();
    let ground = Some(GroundPlane::new(0.)?);
    let mut whole = native::Control::new(&all, ground, scene::GRAVITY)?;
    if args.native_graph {
        if args.inspect_at.iter().any(|&t| t == 0 || t > args.ticks) {
            return Err("requested tick outside native interval".into());
        }
        let mut max_ids = vec![];
        let mut largest_tick = 0;
        let mut exceeding = 0;
        let mut inspected = vec![];
        for tick in 1..=args.ticks {
            let w = scene::wrenches((tick - 1) % scene::END + 1, &groups);
            let (p, f) = whole.advance(10, &w, &[])?;
            let components = graph(&p);
            if let Some(largest) = components.first() {
                if largest.len() > 32 {
                    exceeding += 1;
                }
                if largest.len() > max_ids.len() {
                    max_ids.clone_from(largest);
                    largest_tick = tick;
                }
            }
            if args.inspect_at.contains(&tick) {
                inspected.push(json!({"tick":tick,"finite_contact_components":components,"pairs":p,"floors":f,"bodies":whole.bodies}));
            }
        }
        println!(
            "{}",
            json!({"layout":args.layout,"ticks":args.ticks,"max_component_members":max_ids.len(),
            "largest_first_tick":largest_tick,"largest_ids":max_ids,"intervals_over_32":exceeding,"inspected":inspected})
        );
        return Ok(());
    }
    let mut controls = groups
        .iter()
        .map(|g| native::Control::new(g, ground, scene::GRAVITY))
        .collect::<Result<Vec<_>, _>>()?;
    for tick in 1..=args.ticks {
        let w = scene::wrenches((tick - 1) % scene::END + 1, &groups);
        let (wp, wf) = whole.advance(10, &w, &[])?;
        let mut cp = vec![];
        let mut cf = vec![];
        for (g, c) in groups.iter().zip(&mut controls) {
            let loads = w
                .iter()
                .filter(|w| g.binary_search_by_key(&w.id, |b| b.id).is_ok())
                .cloned()
                .collect::<Vec<_>>();
            let (p, f) = c.advance(10, &loads, &[])?;
            cp.extend(p);
            cf.extend(f);
        }
        let states = controls.iter().flat_map(|c| &c.bodies).collect::<Vec<_>>();
        if bincode::serialize(&(&whole.bodies, &wp, &wf))?
            != bincode::serialize(&(&states, &cp, &cf))?
        {
            let first = whole.bodies.iter().zip(states).find(|(a, b)| a != b);
            println!(
                "{}",
                json!({"tick":tick,"first_mismatch":first,"whole_pairs":wp,"assembly_pairs":cp,"whole_floors":wf,"assembly_floors":cf})
            );
            return Ok(());
        }
    }
    println!("{}", json!({"exact_ticks":args.ticks}));
    Ok(())
}
