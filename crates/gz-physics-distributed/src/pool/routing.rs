// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::physics::{ComponentInfo, ForceMotion, field_motion, forced_groups_with_profile};

type FieldPlan = (Vec<(u64, usize)>, Vec<u64>);

pub(super) struct ProspectiveGroups {
    world: Snapshot,
    pub(super) components: Vec<ComponentInfo>,
    pub(super) owners: Vec<usize>,
    motion: Vec<ForceMotion>,
    pub(super) groups: Vec<Vec<usize>>,
}
impl ProspectiveGroups {
    pub(super) fn new(
        frame: &PoolFrame,
        gravity: [f64; 3],
        pending: &[PendingImpulse],
        inputs: &[BodyWrench],
        profile: NativeSolverProfile,
    ) -> Result<Self, Error> {
        let mut world = Snapshot {
            tick: frame.tick,
            step: frame.step(),
            ground: frame.ground(),
            ..Default::default()
        };
        let mut components = Vec::new();
        let mut owners = Vec::new();
        for (w, obs) in frame.workers.iter().enumerate() {
            world.bodies.extend_from_slice(&obs.snapshot.bodies);
            components.extend_from_slice(&obs.components);
            owners.extend(std::iter::repeat_n(w, obs.components.len()));
        }
        world.bodies.sort_by_key(|b| b.id);
        let motion = field_motion(world.step, &world.bodies, gravity, inputs)
            .map_err(|_| Error::Invalid("invalid pool gravity or held wrench"))?;
        let groups = forced_groups_with_profile(&world, &components, pending, &motion, profile)
            .map_err(|_| Error::Colocation)?;
        Ok(Self {
            world,
            components,
            owners,
            motion,
            groups,
        })
    }
}

impl WorkerPool {
    // One global sweep includes indirect contact chains, every warm component,
    // impending floor deflection and the actual gravity for this barrier.
    pub(super) fn routes(
        &self,
        gravity: [f64; 3],
        pending: &[PendingImpulse],
        inputs: &[BodyWrench],
    ) -> Result<Vec<(u64, usize)>, Error> {
        self.routes_in(&self.frame, gravity, pending, inputs)
    }
    pub(super) fn routes_in(
        &self,
        frame: &PoolFrame,
        gravity: [f64; 3],
        pending: &[PendingImpulse],
        inputs: &[BodyWrench],
    ) -> Result<Vec<(u64, usize)>, Error> {
        Ok(self.plan_field(frame, gravity, pending, inputs, false)?.0)
    }

    pub(super) fn plan_field(
        &self,
        frame: &PoolFrame,
        gravity: [f64; 3],
        pending: &[PendingImpulse],
        inputs: &[BodyWrench],
        island_policy: bool,
    ) -> Result<FieldPlan, Error> {
        let plan = ProspectiveGroups::new(frame, gravity, pending, inputs, self.profile)?;
        self.route_plan(frame, pending, island_policy, &plan)
    }

    pub(super) fn route_plan(
        &self,
        frame: &PoolFrame,
        pending: &[PendingImpulse],
        island_policy: bool,
        plan: &ProspectiveGroups,
    ) -> Result<FieldPlan, Error> {
        let ProspectiveGroups {
            world,
            components,
            owners,
            motion,
            groups,
        } = plan;
        let refined = if island_policy {
            crate::adaptive::group_refinements(world, components, groups, pending, motion)?
        } else {
            Vec::new()
        };
        // When every prospective connected group already belongs to one worker,
        // no shard can gain inputs, bodies or tombstones. The full global sweep
        // above still validates native component/step capacity in this case.
        let mut crossings = groups
            .iter()
            .filter(|group| {
                let owner = owners[group[0]];
                group.iter().any(|&i| owners[i] != owner)
            })
            .peekable();
        if crossings.peek().is_none() {
            return Ok((Vec::new(), refined));
        }
        let mut reserved: Vec<BTreeSet<_>> = self
            .retired
            .iter()
            .zip(&frame.workers)
            .map(|(retired, obs)| {
                retired
                    .keys()
                    .chain(obs.generations.keys())
                    .copied()
                    .collect()
            })
            .collect();
        let inputs: Vec<_> = components
            .iter()
            .map(|c| {
                pending
                    .iter()
                    .filter(|p| c.ids.binary_search(&p.id).is_ok())
                    .count()
            })
            .collect();
        let mut local_inputs = vec![0; self.peers.len()];
        for (i, &owner) in owners.iter().enumerate() {
            local_inputs[owner] += inputs[i];
        }
        let mut moves = Vec::new();
        for group in crossings {
            // Maximize bodies that stay on their existing owner. The stable
            // worker ID breaks ties. Reserve tombstone capacity for all arrivals.
            let members: BTreeSet<_> = group
                .iter()
                .flat_map(|&i| components[i].ids.iter().copied())
                .collect();
            let mut counts = vec![0; self.peers.len()];
            for &i in group {
                counts[owners[i]] += components[i].ids.len();
            }
            let d = (0..counts.len())
                .filter(|&w| {
                    local_inputs[w]
                        + group
                            .iter()
                            .filter(|&&i| owners[i] != w)
                            .map(|&i| inputs[i])
                            .sum::<usize>()
                        <= MAX_PENDING_IMPULSES
                })
                .filter(|&w| reserved[w].union(&members).count() <= MAX_BODY_IDS)
                .min_by_key(|&w| (std::cmp::Reverse(counts[w]), w))
                .ok_or(Error::Capacity)?;
            reserved[d].extend(members);
            for &i in group {
                if owners[i] != d {
                    local_inputs[owners[i]] -= inputs[i];
                    local_inputs[d] += inputs[i];
                    moves.push((components[i].ids[0], d));
                }
            }
        }
        Ok((moves, refined))
    }
}
