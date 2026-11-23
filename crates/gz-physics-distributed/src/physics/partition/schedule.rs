// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// A resolution decision on the public clock. Stable global IDs survive
/// migration and native handle reassignment. Only the coordinator selects IDs.
#[derive(Clone, Copy)]
pub(crate) enum SolveSchedule<'a> {
    Uniform(u8),
    Islands(&'a [u64]),
}

impl SolveSchedule<'_> {
    pub(super) fn validate(self, snapshot: &Snapshot) -> Result<(), WorldError> {
        match self {
            Self::Uniform(n) => validate_substeps(snapshot.step, n),
            Self::Islands(ids) => {
                validate_substeps(snapshot.step, 10)?;
                if ids.len() > snapshot.bodies.len()
                    || ids.windows(2).any(|p| p[0] >= p[1])
                    || ids
                        .iter()
                        .any(|id| snapshot.bodies.binary_search_by_key(id, |b| b.id).is_err())
                {
                    return Err(WorldError::IslandResolution);
                }
                Ok(())
            }
        }
    }

    /// The same prospective grouping is used to stage native mergers. Reject
    /// a stale or inconsistent mask before changing any resident solver.
    pub(super) fn check_group(
        self,
        components: &[ComponentInfo],
        group: &[usize],
    ) -> Result<(), WorldError> {
        if let Self::Islands(ids) = self {
            let fine = ids.binary_search(&components[group[0]].ids[0]).is_ok();
            if group
                .iter()
                .flat_map(|&i| &components[i].ids)
                .any(|id| ids.binary_search(id).is_ok() != fine)
            {
                return Err(WorldError::IslandResolution);
            }
        }
        Ok(())
    }

    pub(super) fn fine(self, member: u64) -> bool {
        match self {
            Self::Uniform(n) => n == 10,
            Self::Islands(ids) => ids.binary_search(&member).is_ok(),
        }
    }
}
