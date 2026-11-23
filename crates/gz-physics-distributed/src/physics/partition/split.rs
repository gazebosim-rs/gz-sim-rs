// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Split separated native components at a published barrier.
use super::*;

/// The coordinator can see the published receipt. The worker additionally
/// checks retained native edges before installing the proposed fragments.
pub(crate) fn separated_groups(
    snapshot: &Snapshot,
    component: &ComponentInfo,
    inputs: &[PendingImpulse],
) -> Result<Vec<ComponentInfo>, WorldError> {
    let mut local = Snapshot {
        tick: snapshot.tick,
        step: snapshot.step,
        ground: snapshot.ground,
        ..Default::default()
    };
    local.bodies.extend(
        snapshot
            .bodies
            .iter()
            .filter(|b| component.ids.binary_search(&b.id).is_ok())
            .cloned(),
    );
    let mut seeds = singleton_seeds(component);
    for p in &snapshot.impulses {
        if component.ids.binary_search(&p.a).is_ok() {
            keep_together(&mut seeds, p.a, p.b)?;
        }
    }
    separated_candidates(&local, seeds, inputs)
}

fn singleton_seeds(component: &ComponentInfo) -> Vec<ComponentInfo> {
    component
        .ids
        .iter()
        .map(|&id| ComponentInfo {
            ids: vec![id],
            contact_history: component.contact_history,
        })
        .collect()
}

fn separated_candidates(
    local: &Snapshot,
    mut seeds: Vec<ComponentInfo>,
    inputs: &[PendingImpulse],
) -> Result<Vec<ComponentInfo>, WorldError> {
    if seeds.len() == 1 {
        return Ok(seeds);
    }
    seeds.sort_unstable_by_key(|g| g.ids[0]);
    let groups = potential_contact_groups(local, &seeds, inputs)?;
    Ok(groups
        .into_iter()
        .map(|indices| {
            let mut ids: Vec<_> = indices
                .iter()
                .flat_map(|&i| seeds[i].ids.iter().copied())
                .collect();
            ids.sort_unstable();
            ComponentInfo {
                ids,
                contact_history: seeds[indices[0]].contact_history,
            }
        })
        .collect())
}

/// Native pairs may remain cached after disappearing from the public receipt.
/// Preserve these and transient receipt edges, and give each warm connected
/// group the full deflection envelope before searching for secondary contacts.
fn native_candidates(
    source: &Island,
    inputs: &[PendingImpulse],
) -> Result<Vec<ComponentInfo>, WorldError> {
    let frame = &source.committed;
    let mut seeds = singleton_seeds(&ComponentInfo::from_snapshot(frame));
    let world = source.sim.world();
    let ids: BTreeMap<_, _> = source
        .index
        .iter()
        .map(|(&id, &e)| (world.get::<Binding>(e).unwrap().native, id))
        .collect();
    let backend = world
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    for p in backend.rigid_contact_observations() {
        keep_together(&mut seeds, ids[&p.a], ids[&p.b])?;
    }
    for p in &frame.impulses {
        keep_together(&mut seeds, p.a, p.b)?;
    }
    separated_candidates(frame, seeds, inputs)
}

/// Groups are disjoint fragments of one admitted native solver.
fn keep_together(groups: &mut Vec<ComponentInfo>, a: u64, b: u64) -> Result<(), WorldError> {
    let owner = |id| {
        groups
            .iter()
            .position(|g| g.ids.binary_search(&id).is_ok())
            .ok_or(WorldError::Missing(id))
    };
    let (a, b) = (owner(a)?, owner(b)?);
    if a != b {
        let source = groups.swap_remove(a.max(b));
        let dest = &mut groups[a.min(b)];
        dest.ids.extend(source.ids);
        dest.ids.sort_unstable();
    }
    Ok(())
}

impl Partition {
    pub(crate) fn split_component(&mut self, member: u64) -> Result<(), WorldError> {
        self.healthy()?;
        let i = self.owner(member)?;
        let groups = separated_groups(&self.committed, &self.components[i], &self.pending)?;
        if groups.len() <= 1 {
            return Ok(());
        }
        if native_candidates(&self.solvers[i], &self.pending)? != groups {
            return Err(WorldError::ContactHistory);
        }
        let fragments = self.split_solver(&self.solvers[i], groups)?;
        self.solvers.remove(i);
        self.solvers.extend(fragments);
        self.publish(self.committed.tick);
        Ok(())
    }

    /// Fission only at completed solver boundaries, before publishing the next
    /// worker observation. Retained native edges and transient receipts keep a
    /// group together. A later field or impulse is checked by the usual merger
    /// preflight before any fragment can move again.
    pub(super) fn split_separated_solvers(&mut self) -> Result<(), WorldError> {
        let mut staged = Vec::new();
        for (index, source) in self.solvers.iter().enumerate() {
            let frame = &source.committed;
            if frame.bodies.len() < 2 || (frame.bodies.len() == 2 && !frame.impulses.is_empty()) {
                continue;
            }
            let component = ComponentInfo::from_snapshot(frame);
            if frame.impulses.len() >= component.ids.len() - 1
                && active_contact_group_with_profile(frame, &component.ids, self.profile)
            {
                continue;
            }
            let groups = native_candidates(source, &[])?;
            if groups.len() > 1 {
                staged.push((index, self.split_solver(source, groups)?));
            }
        }
        for (index, fragments) in staged.into_iter().rev() {
            self.solvers.remove(index);
            self.solvers.extend(fragments);
        }
        Ok(())
    }

    fn split_solver(
        &self,
        source: &Island,
        groups: Vec<ComponentInfo>,
    ) -> Result<Vec<Island>, WorldError> {
        // Clone the original native world for each fragment, then remove other
        // members. All retained handles, cached impulses, current inertia, and
        // solver configuration survive. Publish only after every image checks.
        let world = source.sim.world();
        let backend = world
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let handles: BTreeMap<_, _> = source
            .index
            .iter()
            .map(|(&id, &e)| (id, world.get::<Binding>(e).unwrap().native))
            .collect();
        let mut owner = BTreeMap::new();
        for (index, g) in groups.iter().enumerate() {
            for id in &g.ids {
                owner.insert(handles[id], index);
            }
        }
        if backend
            .rigid_contact_observations()
            .iter()
            .any(|p| owner[&p.a] != owner[&p.b])
        {
            return Err(WorldError::ContactHistory);
        }
        let native = backend.checkpoint_solver_with_profile(self.profile)?;
        let mut fragments = Vec::with_capacity(groups.len());
        for g in groups {
            let mut copy =
                MultiFidelityBackend::restore_solver_with_profile(&native, self.profile)?;
            let mut keep = BTreeMap::new();
            for (&id, &handle) in &handles {
                if g.ids.binary_search(&id).is_ok() {
                    keep.insert(id, handle);
                } else if !copy.remove(handle) {
                    return Err(WorldError::Missing(id));
                }
            }
            let mut expected = source.committed.clone();
            expected.bodies.retain(|b| keep.contains_key(&b.id));
            expected
                .impulses
                .retain(|p| keep.contains_key(&p.a) && keep.contains_key(&p.b));
            expected.supports.retain(|p| keep.contains_key(&p.body));
            expected.contacts = expected.impulses.len();
            let mut prefix = source.prefix.clone();
            prefix.retain(|id| keep.contains_key(&id));
            let mut fragment = Island::install(self.profile, copy, keep, expected.clone())?;
            fragment.prefix = prefix;
            fragment.refresh(expected.tick);
            if fragment.committed != expected {
                return Err(WorldError::Checkpoint);
            }
            fragments.push(fragment);
        }
        Ok(fragments)
    }
}
