// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recompute prospective solver membership on a public refused-barrier capture.
use clap::Parser;
use gz_physics_distributed::{
    physics::{
        BodyWrench, ComponentInfo, Snapshot, potential_contact_groups_in_field,
        potential_contact_groups_on_smooth_plane,
    },
    protocol::Observation,
};
use serde::Deserialize;
use serde_json::json;
use std::{collections::BTreeMap, error::Error, fs::File, path::PathBuf};

#[derive(Parser)]
struct Args {
    /// admission.json from placement_endurance.
    source: PathBuf,
    /// Directory with solver-*.json receipts from the matching isolated native capture.
    #[arg(long)]
    native_cache: Option<PathBuf>,
    /// Repeat the native fission check without a next-step acceleration.
    #[arg(long)]
    no_field: bool,
}
#[derive(Deserialize)]
struct Evidence {
    last_committed_tick: u64,
    workers: Vec<Observation>,
    wrenches: Vec<BodyWrench>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let capture: Evidence = serde_json::from_reader(File::open(args.source)?)?;
    let mut s = Snapshot {
        tick: capture.last_committed_tick,
        step: capture.workers[0].snapshot.step,
        ground: capture.workers[0].snapshot.ground,
        ..Default::default()
    };
    let mut retained = vec![];
    let mut pairs = vec![];
    for w in capture.workers {
        s.bodies.extend(w.snapshot.bodies);
        retained.extend(w.components);
        pairs.extend(w.snapshot.impulses);
    }
    s.bodies.sort_by_key(|b| b.id);
    let singles: Vec<_> = s
        .bodies
        .iter()
        .map(|b| ComponentInfo {
            ids: vec![b.id],
            contact_history: false,
        })
        .collect();
    let mut parents: BTreeMap<u64, u64> = s.bodies.iter().map(|b| (b.id, b.id)).collect();
    fn root(p: &BTreeMap<u64, u64>, mut id: u64) -> u64 {
        while p[&id] != id {
            id = p[&id];
        }
        id
    }
    let mut edges: Vec<_> = pairs.iter().map(|p| (p.a, p.b)).collect();
    let source = if let Some(dir) = args.native_cache {
        #[derive(Deserialize)]
        struct Pair {
            a: u64,
            b: u64,
        }
        #[derive(Deserialize)]
        struct Cache {
            tick: u64,
            native: Vec<Pair>,
        }
        for entry in std::fs::read_dir(dir)? {
            let cache: Cache = serde_json::from_reader(File::open(entry?.path())?)?;
            if cache.tick != s.tick {
                return Err("cache belongs to a different barrier".into());
            }
            edges.extend(cache.native.into_iter().map(|p| (p.a, p.b)));
        }
        "published_and_native_cache_edges"
    } else {
        "published_receipt_edges"
    };
    for (id_a, id_b) in edges {
        let a = root(&parents, id_a);
        let b = root(&parents, id_b);
        parents.insert(a.max(b), a.min(b));
    }
    let mut groups: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
    for b in &s.bodies {
        groups.entry(root(&parents, b.id)).or_default().push(b.id);
    }
    let receipts: Vec<_> = groups
        .into_values()
        .map(|ids| ComponentInfo {
            contact_history: ids.len() > 1,
            ids,
        })
        .collect();
    let mut report = vec![];
    let gravity = if args.no_field {
        [0.; 3]
    } else {
        [0., 0., -9.81]
    };
    let field = if args.no_field {
        &[][..]
    } else {
        &capture.wrenches
    };
    for (name, components) in [
        ("retained_native", &retained),
        (source, &receipts),
        ("all_singletons", &singles),
    ] {
        let baseline = potential_contact_groups_in_field(&s, components, &[], gravity, field)?;
        let planar = potential_contact_groups_on_smooth_plane(&s, components, &[], gravity, field)?;
        let canonical = |groups: Vec<Vec<usize>>| {
            let mut out: Vec<Vec<u64>> = groups
                .iter()
                .map(|g| {
                    let mut ids: Vec<_> = g
                        .iter()
                        .flat_map(|&i| components[i].ids.iter().copied())
                        .collect();
                    ids.sort_unstable();
                    ids
                })
                .collect();
            out.sort_unstable_by_key(|g| (std::cmp::Reverse(g.len()), g[0]));
            out
        };
        report.push(json!({"source_membership":name,"unrestricted":canonical(baseline),"smooth_plane":canonical(planar)}));
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"last_committed_tick":s.tick,"component_graphs":report})
        )?
    );
    Ok(())
}
