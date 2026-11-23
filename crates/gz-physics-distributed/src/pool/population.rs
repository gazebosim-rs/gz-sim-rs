// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-tick population changes with stable IDs and committed ownership fences.
use super::*;
use crate::{
    coordinator::validate_separated_birth,
    physics::{ComponentInfo, forced_groups_with_profile},
};

impl PoolFrame {
    pub(super) fn highest_body_id(&self) -> Option<u64> {
        self.workers
            .iter()
            .filter_map(|o| o.generations.last_key_value().map(|(&id, _)| id))
            .max()
    }
}

impl WorkerPool {
    pub(crate) fn has_population_edits(&self) -> bool {
        self.population_high_water.is_some()
    }

    fn highest_issued(&self) -> Option<u64> {
        self.population_high_water
            .or_else(|| self.frame.highest_body_id())
    }

    /// Insert a separated primitive at the published tick. Its ID must exceed
    /// every ID issued since the original seed, including permanently retired IDs.
    pub fn insert_body(&mut self, body: Body) -> Result<&PoolFrame, Error> {
        self.insert_body_in_field(body, [0.; 3], &[])
    }

    /// Check the next held field and pending impulses before publishing a birth.
    pub fn insert_body_in_field(
        &mut self,
        body: Body,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<&PoolFrame, Error> {
        if self.highest_issued().is_some_and(|last| body.id <= last) {
            return Err(Error::Invalid(
                "new body ID must exceed all previously issued IDs",
            ));
        }
        validate_separated_birth(
            &body,
            &self.frame.workers[0].snapshot,
            self.frame.workers.iter().flat_map(|o| &o.snapshot.bodies),
            self.profile,
        )?;
        let mut owners: Vec<_> = (0..self.peers.len()).collect();
        owners.sort_by_key(|&w| self.frame.workers[w].generations.len());
        let mut failure = Error::Capacity;
        for w in owners {
            if self.frame.workers[w].generations.len() + self.retired[w].len() >= MAX_BODY_IDS {
                continue;
            }
            let mut projected = self.frame.clone();
            let obs = &mut projected.workers[w];
            // A newly issued ID sorts after every current native component and body.
            obs.snapshot.bodies.push(body.clone());
            obs.generations.insert(body.id, 0);
            obs.components.push(ComponentInfo {
                ids: vec![body.id],
                contact_history: false,
            });
            if forced_groups_with_profile(
                &obs.snapshot,
                &obs.components,
                &self.queued,
                &[],
                self.profile,
            )
            .is_err()
            {
                failure = Error::Colocation;
                continue;
            }
            let moves = match self.routes_in(&projected, gravity, &self.queued, inputs) {
                Ok(m) => m,
                Err(e @ (Error::Capacity | Error::Colocation)) => {
                    failure = e;
                    continue;
                }
                Err(e) => return Err(e),
            };
            if self
                .command_counts_in(&projected, &moves)
                .iter()
                .any(|&n| n > self.limit)
            {
                failure = Error::Capacity;
                continue;
            }
            let mut counts = vec![0; self.peers.len()];
            counts[w] = 1;
            self.reserve(&counts)?;
            let mut stage = self.stage()?;
            let id = body.id;
            let tick = stage.frame.tick;
            let reply = self.exchange(&mut stage, w, Command::InsertBody { tick, body })?;
            if !matches!(reply,Outcome::State(actual) if actual==projected.workers[w]) {
                return Err(Error::Invalid("pooled birth changed surviving state"));
            }
            stage.frame = projected;
            self.commit(stage)?;
            self.population_high_water = Some(id);
            return Ok(&self.frame);
        }
        Err(failure)
    }

    /// Permanently retire a body and its queued impulses at this tick. Clear every
    /// participant's previous release fence only after all workers acknowledge.
    pub fn retire(&mut self, id: u64) -> Result<&PoolFrame, Error> {
        let (owner, _) = self
            .frame
            .body(id)
            .ok_or(Error::Invalid("unknown retired body"))?;
        let high = self
            .highest_issued()
            .expect("owner located in committed frame");
        self.reserve(&vec![1; self.peers.len()])?;
        let mut stage = self.stage()?;
        let generation = stage.frame.workers[usize::from(owner)].generations[&id];
        stage.frame.workers[usize::from(owner)].project_retirement(id);
        for w in 0..self.peers.len() {
            // A worker not involved in the last handoff may remember an older release
            // or may never have owned the ID. Bind the command to its exact local fence.
            let expected = if w == usize::from(owner) {
                generation
            } else {
                stage.retired[w].get(&id).copied().unwrap_or(0)
            };
            let command = Command::RetireBody {
                tick: stage.frame.tick,
                id,
                owner,
                generation: expected,
            };
            let answer = self.exchange(&mut stage, w, command)?;
            if !matches!(answer,Outcome::State(actual) if actual==stage.frame.workers[w]) {
                return Err(Error::Invalid("pooled retirement changed surviving state"));
            }
            stage.retired[w].remove(&id);
        }
        stage.queued.retain(|input| input.id != id);
        self.commit(stage)?;
        self.population_high_water = Some(high);
        Ok(&self.frame)
    }
}
