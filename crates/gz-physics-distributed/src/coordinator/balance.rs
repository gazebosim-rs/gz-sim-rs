// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Opt-in spatial bisection using measured per-solver work at a published tick.
use super::*;
use crate::physics::{ComponentWork, potential_contact_groups};
use routing::RoutingScene;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoadWindow {
    run: u64,
    serial: u64,
    start: u64,
    span: u16,
}
#[derive(Clone, Debug)]
pub struct LoadSample {
    window: LoadWindow,
    rows: [Vec<ComponentWork>; 2],
}
impl LoadSample {
    pub fn workers(&self) -> &[Vec<ComponentWork>; 2] {
        &self.rows
    }
    pub fn measured_steps(&self) -> u16 {
        self.window.span
    }
}
/// Minimum decrease in the busier worker's mean native solve time. Both the
/// absolute and fractional thresholds must be met by a proposed handoff.
#[derive(Clone, Copy, Debug)]
pub struct BalancePolicy {
    pub minimum_savings_ns: u64,
    pub minimum_savings_per_mille: u16,
}
impl Default for BalancePolicy {
    fn default() -> Self {
        Self {
            minimum_savings_ns: 0,
            minimum_savings_per_mille: 10,
        }
    }
}

/// One complete native component moving toward the selected spatial cut.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BalanceDecision {
    pub members: Vec<u64>,
    pub source: u8,
    pub destination: u8,
    pub axis: usize,
    pub before_ns: [u64; 2],
    pub estimated_after_ns: [u64; 2],
    pub warm: bool,
}

impl Coordinator<WorkerProcess> {
    /// Time the next `span` local solves on both workers. Sampling changes no
    /// solver input. A topology or authority change requires a fresh window.
    pub fn start_load_sample(&mut self, span: u16) -> Result<LoadWindow, Error> {
        self.require_committed()?;
        let start = self.frame.tick;
        if span == 0 || start.checked_add(u64::from(span)).is_none() {
            return Err(Error::Invalid("invalid load window"));
        }
        for peer in &mut self.peers {
            peer.start_probe(start, span)?;
        }
        Ok(LoadWindow {
            run: self.run,
            serial: self.serial,
            start,
            span,
        })
    }
    /// Read an exact completed window. Replacements discard diagnostic timers.
    pub fn finish_load_sample(&mut self, window: LoadWindow) -> Result<LoadSample, Error> {
        self.require_committed()?;
        self.check_load_window(window)?;
        let mut rows: [Vec<ComponentWork>; 2] = Default::default();
        for (i, peer) in self.peers.iter_mut().enumerate() {
            rows[i] = peer
                .read_probe(window.start, window.span)?
                .ok_or(Error::Invalid(
                    "load window interrupted or component membership changed",
                ))?;
        }
        let sample = LoadSample { window, rows };
        self.check_load_rows(&sample)?;
        Ok(sample)
    }
}

struct Unit {
    owner: usize,
    position: [f64; 3],
    cost: u64,
    movable: bool,
    members: usize,
    first_id:u64,
    new_ids:usize,
}
impl<P: Peer> Coordinator<P> {
    fn check_load_window(&self, w: LoadWindow) -> Result<(), Error> {
        if w.run != self.run
            || w.serial != self.serial
            || w.start.checked_add(u64::from(w.span)) != Some(self.frame.tick)
            || w.span == 0
        {
            return Err(Error::Invalid("load sample is stale"));
        }
        Ok(())
    }
    fn check_load_rows(&self, s: &LoadSample) -> Result<(), Error> {
        self.check_load_window(s.window)?;
        for (w, rows) in self.frame.workers.iter().zip(&s.rows) {
            if rows
                .iter()
                .map(|r| &r.members)
                .ne(w.components.iter().map(|c| &c.ids))
            {
                return Err(Error::Invalid(
                    "load report does not match native component membership",
                ));
            }
        }
        Ok(())
    }

    /// Consider both orientations of each axial bisection through component
    /// centers. Choose the feasible cut with lowest predicted maximum load,
    /// then fewest transferred bodies. At most one cost-reducing move is
    /// offered per sample so subsequent decisions use new measurements.
    pub fn plan_rebalance(
        &self,
        sample: &LoadSample,
        policy: BalancePolicy,
    ) -> Result<Option<BalanceDecision>, Error> {
        self.check_load_rows(sample)?;
        if policy.minimum_savings_per_mille > 1000 {
            return Err(Error::Invalid("invalid load threshold"));
        }
        self.frame.check_sweeps(&self.queued)?;
        let scene = RoutingScene::from_frame(&self.frame);
        let groups = potential_contact_groups(&scene.snapshot, &scene.components, &self.queued)
            .map_err(|_| Error::Colocation)?;
        let mut isolated = vec![false; scene.components.len()];
        for g in groups {
            if let [index] = g[..] {
                isolated[index] = true;
            }
        }
        let available=[0,1].map(|w|MAX_BODY_IDS-self.frame.known_ids(w));
        let mut before = [0_u64; 2];
        let mut units = Vec::new();
        for (i, (c, row)) in scene
            .components
            .iter()
            .zip(sample.rows.iter().flatten())
            .enumerate()
        {
            let owner = scene.owners[i];
            let cost = row.elapsed_ns.div_ceil(u64::from(sample.window.span));
            before[owner] = before[owner].checked_add(cost).ok_or(Error::Capacity)?;
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for id in &c.ids {
                let b = self.frame.body(*id).unwrap().1;
                for a in 0..3 {
                    lo[a] = lo[a].min(b.position[a]);
                    hi[a] = hi[a].max(b.position[a]);
                }
            }
            let new_ids=c.ids.iter().filter(|id|self.frame.workers[owner].generations[id]==0).count();
            let movable = isolated[i]
                && new_ids<=available[1-owner]
                && !self
                    .queued
                    .iter()
                    .any(|p| c.ids.binary_search(&p.id).is_ok());
            units.push(Unit {
                owner,
                position: std::array::from_fn(|a| lo[a].midpoint(hi[a])),
                cost,
                movable,
                members:c.ids.len(),
                first_id:c.ids[0],
                new_ids,
            });
        }
        let mut options = Vec::with_capacity(units.len());
        for unit in &units {
            options.push(projected_peak(unit, before, policy)?);
        }
        let Some((axis, index)) = best_cut(&units, &options, available, before) else {
            return Ok(None);
        };
        let unit = &units[index];
        let component = &scene.components[index];
        let mut after = before;
        after[unit.owner] -= unit.cost;
        after[1 - unit.owner] += unit.cost;
        Ok(Some(BalanceDecision {
            members: component.ids.clone(),
            source: unit.owner as u8,
            destination: (1 - unit.owner) as u8,
            axis,
            before_ns: before,
            estimated_after_ns: after,
            warm: component.contact_history || component.ids.len() > 1,
        }))
    }

    /// Apply one measured handoff through the regular idempotent ownership
    /// protocol. On delivery loss retry with the same sample and policy.
    pub fn rebalance_one(
        &mut self,
        sample: &LoadSample,
        policy: BalancePolicy,
    ) -> Result<Option<BalanceDecision>, Error> {
        let decision = self.plan_rebalance(sample, policy)?;
        if let Some(d) = &decision {
            if d.warm {
                self.transfer_component(d.members[0])?;
            } else {
                self.transfer(d.members[0], false)?;
            }
        } else {
            self.require_committed()?;
        }
        Ok(decision)
    }
}

fn projected_peak(
    unit: &Unit,
    before: [u64; 2],
    policy: BalancePolicy,
) -> Result<Option<u64>, Error> {
    let src = unit.owner;
    if !unit.movable || before[src] <= before[1 - src] {
        return Ok(None);
    }
    let dest = before[1 - src]
        .checked_add(unit.cost)
        .ok_or(Error::Capacity)?;
    let peak = dest.max(before[src] - unit.cost);
    let savings = before[src].saturating_sub(peak);
    Ok((savings > 0
        && savings >= policy.minimum_savings_ns
        && u128::from(savings) * 1000
            >= u128::from(before[src]) * u128::from(policy.minimum_savings_per_mille))
    .then_some(peak))
}

// The two sides grow in opposite directions in sorted space. Prefix and suffix
// tallies permit every possible cut to be evaluated in constant time after sorting.
type MovePriority = (u64, usize, u64, usize);
#[derive(Clone, Copy, Default)]
struct Side {
    load: u128,
    moved_bodies: usize,
    new_ids: usize,
    pinned: bool,
    best: Option<MovePriority>,
}
impl Side {
    fn include(&mut self, index:usize, destination:usize, units:&[Unit], options:&[Option<u64>]){
        let unit=&units[index];
        self.load+=u128::from(unit.cost);
        if unit.owner==destination {return;}
        self.moved_bodies+=unit.members;
        self.new_ids+=unit.new_ids;
        self.pinned|=!unit.movable;
        if let Some(peak)=options[index]{
            let priority=(peak,unit.members,unit.first_id,index);
            self.best=Some(self.best.map_or(priority,|old|old.min(priority)));
        }
    }
    fn fits(&self, available:usize)->bool{!self.pinned && self.new_ids<=available}
}

fn best_cut(units:&[Unit],options:&[Option<u64>],available:[usize;2],before:[u64;2])->Option<(usize,usize)>{
    if !options.iter().any(Option::is_some){return None;}
    let mut winner=None;
    let mut best=(u128::from(before[0].max(before[1])),0,0,0,0);
    let mut order=(0..units.len()).collect::<Vec<_>>();
    let mut suffix=vec![Side::default();units.len()+1];
    for axis in 0..3 {
        order.sort_unstable_by(|&i,&j|units[i].position[axis].total_cmp(&units[j].position[axis]).then(units[i].first_id.cmp(&units[j].first_id)));
        for left in 0..2{
            let right=1-left;
            for (j,&i) in order.iter().enumerate().rev(){
                suffix[j]=suffix[j+1];
                suffix[j].include(i,right,units,options);
            }
            let mut prefix=Side::default();
            for cut in 1..order.len(){
                let previous=order[cut-1];let next=order[cut];
                prefix.include(previous,left,units,options);
                let far=suffix[cut];
                if units[previous].position[axis]==units[next].position[axis] || !prefix.fits(available[left]) || !far.fits(available[right]){
                    continue;
                }
                let key=(prefix.load.max(far.load),prefix.moved_bodies+far.moved_bodies,axis,cut,left);
                if key<best && let Some((_,_,_,i))=prefix.best.into_iter().chain(far.best).min(){
                    best=key;winner=Some((axis,i));
                }
            }
        }
    }
    winner
}

#[cfg(test)]
#[path = "../../tests/private/balance.rs"]
mod tests;
