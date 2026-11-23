// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolate native allocation order without any worker, handoff, or native join.
#[path = "support/native_contact_control.rs"]
mod native;
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod scene;
use clap::Parser;
use gz_physics_distributed::physics::{
    Body, BodyWrench, ContactImpulse, GroundPlane, GroundSupport,
};
use serde::{Deserialize, Serialize};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{error::Error, fs, path::PathBuf};
type Sample = (Vec<Body>, Vec<ContactImpulse>, Vec<GroundSupport>);
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    /// order-replay.json with the actual first divergent native layout.
    evidence: PathBuf,
    #[arg(long, default_value_t = 15_400, value_parser = clap::value_parser!(u64).range(1..))]
    ticks: u64,
}
#[derive(Deserialize)]
struct Evidence {
    before: Before,
    tick: u64,
}
#[derive(Deserialize)]
struct Before {
    worker_handles: Vec<(u64, u64)>,
}
#[derive(Default, Serialize)]
struct FiniteActivity {
    receipts: u64,
    positive: u64,
    last_receipt_tick: Option<u64>,
    last_positive_tick: Option<u64>,
    last_receipts: Vec<ContactImpulse>,
    last_positive: Vec<ContactImpulse>,
}
impl FiniteActivity {
    fn observe(&mut self, tick: u64, pairs: &[ContactImpulse]) {
        self.receipts += pairs.len() as u64;
        if !pairs.is_empty() {
            self.last_receipt_tick = Some(tick);
            self.last_receipts.clear();
            self.last_receipts.extend_from_slice(pairs);
        }
        if pairs.iter().any(|p| p.normal_impulse > 0.) {
            self.last_positive_tick = Some(tick);
            self.last_positive.clear();
            self.last_positive
                .extend(pairs.iter().filter(|p| p.normal_impulse > 0.).cloned());
            self.positive += self.last_positive.len() as u64;
        }
    }
}
fn physical(control: &mut native::Control, loads: &[BodyWrench]) -> Result<Sample> {
    let (mut pairs, floors) = control.advance(10, loads, &[])?;
    let mut bodies = control.bodies.clone();
    bodies.sort_by_key(|b| b.id);
    for p in &mut pairs {
        if p.a > p.b {
            std::mem::swap(&mut p.a, &mut p.b);
        }
    }
    pairs.sort_by_key(|p| (p.a, p.b));
    Ok((bodies, pairs, floors))
}
fn main() -> Result<()> {
    let args = Args::parse();
    let evidence: Evidence = serde_json::from_slice(&fs::read(&args.evidence)?)?;
    let groups = scene::seeds(scene::Layout::Adjacent);
    let canonical = groups.concat();
    let mut marks = evidence.before.worker_handles;
    marks.sort_by_key(|x| x.1);
    if marks.len() != canonical.len()
        || marks
            .iter()
            .enumerate()
            .any(|(i, &(_, handle))| handle != i as u64)
    {
        return Err("expected one distinct consecutive native handle per fixture body".into());
    }
    let mut by_id = canonical
        .iter()
        .map(|b| (b.id, b))
        .collect::<std::collections::BTreeMap<_, _>>();
    let permuted = marks
        .iter()
        .map(|&(id, _)| {
            by_id
                .remove(&id)
                .cloned()
                .ok_or("inconsistent public ID in worker binding")
        })
        .collect::<std::result::Result<Vec<_>, _>>()?;
    let mut exchange = canonical.clone();
    let a = exchange
        .binary_search_by_key(&29, |b| b.id)
        .map_err(|_| "missing target ID")?;
    let b = exchange
        .binary_search_by_key(&58, |b| b.id)
        .map_err(|_| "missing target ID")?;
    exchange.swap(a, b);
    let ground = Some(GroundPlane::new(0.)?);
    let mut reference = native::Control::new(&canonical, ground, scene::GRAVITY)?;
    let mut arms = [
        (
            "worker_final_order",
            native::Control::new(&permuted, ground, scene::GRAVITY)?,
            None,
            0,
        ),
        (
            "only_exchange_29_and_58",
            native::Control::new(&exchange, ground, scene::GRAVITY)?,
            None,
            0,
        ),
    ];
    let mut event = vec![];
    let mut trajectories = [Sha256::new(), Sha256::new(), Sha256::new()];
    let mut finite: [FiniteActivity; 3] = std::array::from_fn(|_| FiniteActivity::default());
    for tick in 1..=args.ticks {
        let loads = scene::wrenches((tick - 1) % scene::END + 1, &groups);
        let r = physical(&mut reference, &loads)?;
        finite[0].observe(tick, &r.1);
        trajectories[0].update(bincode::serialize(&(tick, &r.0, &r.1, &r.2))?);
        for (index, (name, c, first, count)) in arms.iter_mut().enumerate() {
            let s = physical(c, &loads)?;
            finite[1 + index].observe(tick, &s.1);
            trajectories[1 + index].update(bincode::serialize(&(tick, &s.0, &s.1, &s.2))?);
            if r != s {
                first.get_or_insert(tick);
                *count += 1;
            }
            if tick == evidence.tick {
                let hash =
                    |s: &(Vec<Body>, Vec<ContactImpulse>, Vec<GroundSupport>)| -> Result<String> {
                        Ok(format!(
                            "{:x}",
                            Sha256::digest(bincode::serialize(&(tick, &s.0, &s.1, &s.2))?)
                        ))
                    };
                event.push(json!({"world":name,"tick":tick,"reference_sample":hash(&r)?,"permuted_sample":hash(&s)?,
                    "reference_29_58":r.1.iter().find(|p|(p.a,p.b)==(29,58)),
                    "permuted_29_58":s.1.iter().find(|p|(p.a,p.b)==(29,58))}));
            }
        }
    }
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"ticks":args.ticks,"finite_activity_canonical_worker_order_pair_exchange":finite,"trajectory_sha256_canonical_worker_order_pair_exchange":trajectories.map(|hash|format!("{:x}",hash.finalize())),"results":arms.into_iter().map(|(name,_,first,count)|json!({"world":name,"first_difference":first,"differing_ticks":count})).collect::<Vec<_>>(),"encounter":event})
        )?
    );
    Ok(())
}
