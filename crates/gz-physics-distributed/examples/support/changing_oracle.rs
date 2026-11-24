// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cache the independently solved short changing-contact trajectory for comparison across trials.
use crate::fixture;
#[path = "native_contact_control.rs"]
mod native;
use gz_physics_distributed::{
    physics::{Body, ContactImpulse, GroundPlane, GroundSupport},
    pool::PoolFrame,
};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, error::Error};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
pub struct Oracle {
    pub groups: Vec<Vec<Body>>,
    pub ticks: Vec<Vec<u8>>,
    pub digest: String,
    pub pairs: u64,
    pub floors: u64,
}
fn canonical(
    tick: u64,
    mut bodies: Vec<&Body>,
    mut pairs: Vec<&ContactImpulse>,
    mut floors: Vec<&GroundSupport>,
) -> Result<Vec<u8>> {
    bodies.sort_unstable_by_key(|b| b.id);
    pairs.sort_unstable_by_key(|p| (p.a, p.b));
    floors.sort_unstable_by_key(|p| p.body);
    Ok(bincode::serialize(&(tick, bodies, pairs, floors))?)
}
impl Oracle {
    pub fn new(layout: fixture::Layout) -> Result<Self> {
        Self::with_schedule(layout, &fixture::LAUNCHES)
    }
    pub fn with_schedule(layout: fixture::Layout, launch_ticks: &[u64; 6]) -> Result<Self> {
        let groups = fixture::seeds(layout);
        let ground = Some(GroundPlane::new(0.)?);
        let mut controls = groups
            .iter()
            .map(|g| native::Control::new(g, ground, fixture::GRAVITY))
            .collect::<Result<Vec<_>>>()?;
        let mut ticks = Vec::new();
        let mut sum = Sha256::new();
        let (mut pairs, mut floors) = (0, 0);
        for t in 1..=fixture::END {
            let field = fixture::wrenches_at(t, &groups, launch_ticks);
            let (mut pi, mut fl) = (Vec::new(), Vec::new());
            for (control, g) in controls.iter_mut().zip(&groups) {
                let local = field
                    .iter()
                    .filter(|w| g.binary_search_by_key(&w.id, |b| b.id).is_ok())
                    .cloned()
                    .collect::<Vec<_>>();
                let (p, f) = control.advance(10, &local, &[])?;
                pi.extend(p);
                fl.extend(f);
            }
            pairs += pi.len() as u64;
            floors += fl.len() as u64;
            let row = canonical(
                t,
                controls.iter().flat_map(|c| &c.bodies).collect(),
                pi.iter().collect(),
                fl.iter().collect(),
            )?;
            sum.update(&row);
            ticks.push(row);
        }
        Ok(Self {
            groups,
            ticks,
            digest: format!("{:x}", sum.finalize()),
            pairs,
            floors,
        })
    }
    pub fn verify(&self, frame: &PoolFrame, owners: &BTreeMap<u64, (u8, u64)>) -> Result<()> {
        let tick = frame.tick;
        let states = &frame.workers;
        let row = canonical(
            tick,
            states.iter().flat_map(|w| &w.snapshot.bodies).collect(),
            states.iter().flat_map(|w| &w.snapshot.impulses).collect(),
            states.iter().flat_map(|w| &w.snapshot.supports).collect(),
        )?;
        if self.ticks.get(tick as usize - 1) != Some(&row) {
            return Err(format!("physics differed from native controls at {tick}").into());
        }
        for (&id, &(owner, generation)) in owners {
            if states[owner as usize].generations.get(&id) != Some(&generation) {
                return Err(format!("ID {id} owner/generation mismatch at tick {tick}").into());
            }
        }
        Ok(())
    }
}
