// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native geometry and dynamics from two or more previously contacting domains.
use super::*;

impl Partition {
    pub(super) fn joined_solver(&self, indices: &[usize]) -> Result<Island, WorldError> {
        let resident = *indices
            .iter()
            .find(|&&i| self.components[i].contact_history)
            .ok_or(WorldError::Checkpoint)?;
        let donors = indices
            .iter()
            .copied()
            .filter(|&i| i != resident)
            .collect::<Vec<_>>();
        let backend = |i: usize| {
            self.solvers[i]
                .sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap()
        };
        let refs = donors.iter().map(|&i| backend(i)).collect::<Vec<_>>();
        let (merged, maps) =
            backend(resident).join_box_solvers_with_profile(&refs, self.profile)?;
        let mut handles = BTreeMap::new();
        let mut prefix = ContactPrefix::default();
        let mut snapshot = self.solvers[resident].committed.clone();
        snapshot.bodies.clear();
        snapshot.impulses.clear();
        snapshot.supports.clear();
        for (i, map) in
            std::iter::once((resident, None)).chain(donors.into_iter().zip(maps.iter().map(Some)))
        {
            let source = &self.solvers[i];
            prefix.pairs.extend_from_slice(&source.prefix.pairs);
            prefix.supports.extend_from_slice(&source.prefix.supports);
            let world = source.sim.world();
            for (&id, &entity) in &source.index {
                let old = world.get::<Binding>(entity).unwrap().native;
                let new = if let Some(map) = map {
                    *map.get(&old).ok_or(WorldError::Missing(id))?
                } else {
                    old
                };
                if handles.insert(id, new).is_some() {
                    return Err(WorldError::Duplicate(id));
                }
            }
            snapshot.bodies.extend_from_slice(&source.committed.bodies);
            snapshot
                .supports
                .extend_from_slice(&source.committed.supports);
            snapshot
                .impulses
                .extend_from_slice(&source.committed.impulses);
        }
        snapshot.bodies.sort_by_key(|b| b.id);
        snapshot.impulses.sort_by_key(|p| (p.a, p.b));
        snapshot.supports.sort_by_key(|p| p.body);
        snapshot.contacts = snapshot.impulses.len();
        prefix.pairs.sort_by_key(|p| (p.a, p.b));
        prefix.supports.sort_by_key(|p| p.body);
        let mut joined = Island::install(self.profile, merged, handles, snapshot)?;
        joined.prefix = prefix;
        Ok(joined)
    }
}
