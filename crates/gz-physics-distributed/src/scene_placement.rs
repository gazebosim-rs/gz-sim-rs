// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Deterministic cold worker seeds. Hot routing always uses the ordinary barriers.
use super::{Bodies, Refinement};
use crate::physics::{
    Body, BodyWrench, ComponentInfo, MAX_PARTITION_BODIES, NativeSolverProfile, PendingImpulse,
    Snapshot, field_motion, forced_groups_with_profile,
};
use std::cmp::Reverse;

pub(super) fn spatial(bodies: &Bodies, width: usize) -> Vec<Vec<Body>> {
    let mut ordered: Vec<_> = bodies.values().cloned().collect();
    let extent = |axis: usize| {
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for b in &ordered {
            lo = lo.min(b.position[axis]);
            hi = hi.max(b.position[axis]);
        }
        hi - lo
    };
    let axis = (0..3)
        .max_by(|&a, &b| extent(a).total_cmp(&extent(b)).then(b.cmp(&a)))
        .unwrap();
    ordered.sort_by(|a, b| {
        a.position[axis]
            .total_cmp(&b.position[axis])
            .then(a.id.cmp(&b.id))
    });
    let n = ordered.len();
    let mut sorted = ordered.into_iter();
    (0..width)
        .map(|i| {
            sorted
                .by_ref()
                .take(n / width + usize::from(i < n % width))
                .collect()
        })
        .collect()
}

struct Group {
    indices: Vec<usize>,
    work: usize,
}

fn assign(groups: &mut [Group], width: usize, capacity: usize) -> Option<Vec<Vec<usize>>> {
    groups.sort_unstable_by_key(|g| (Reverse(g.work), g.indices[0]));
    let mut assigned = vec![vec![]; width];
    let mut work = vec![0; width];
    for group in groups {
        let owner = (0..width)
            .filter(|&i| assigned[i].len() + group.indices.len() <= capacity)
            .min_by_key(|&i| (work[i], i))?;
        work[owner] += group.work;
        assigned[owner].extend_from_slice(&group.indices);
    }
    Some(assigned)
}

pub(super) fn contact_groups(
    snapshot: &Snapshot,
    width: usize,
    refinement: Refinement,
    profile: NativeSolverProfile,
    gravity: [f64; 3],
    pending: &[PendingImpulse],
    wrenches: &[BodyWrench],
) -> Option<Vec<Vec<Body>>> {
    // Worker construction groups cold solvers without applied forces. Start
    // with those same indivisible groups before forecasting the first field.
    let singles: Vec<_> = snapshot
        .bodies
        .iter()
        .map(|b| ComponentInfo {
            ids: vec![b.id],
            contact_history: false,
        })
        .collect();
    let initial = forced_groups_with_profile(snapshot, &singles, &[], &[], profile).ok()?;
    let components: Vec<_> = initial
        .iter()
        .map(|indices| ComponentInfo {
            ids: indices.iter().map(|&i| snapshot.bodies[i].id).collect(),
            contact_history: false,
        })
        .collect();
    let forces = field_motion(snapshot.step, &snapshot.bodies, gravity, wrenches).ok()?;
    let groups =
        forced_groups_with_profile(snapshot, &components, pending, &forces, profile).ok()?;
    let fine = match refinement {
        Refinement::Island => {
            crate::adaptive::group_refinements(snapshot, &components, &groups, pending, &forces)
                .ok()?
        }
        // A common solve rate is a constant factor for all owners.
        Refinement::Scene | Refinement::Fixed => vec![],
    };
    let mut groups: Vec<_> = groups
        .into_iter()
        .map(|members| {
            let mut indices: Vec<_> = members
                .into_iter()
                .flat_map(|i| initial[i].iter().copied())
                .collect();
            indices.sort_unstable();
            let scale = if fine.binary_search(&snapshot.bodies[indices[0]].id).is_ok() {
                10
            } else {
                1
            };
            Group {
                work: (1 + indices.len()) * scale,
                indices,
            }
        })
        .collect();
    let assigned = assign(&mut groups, width, MAX_PARTITION_BODIES)?;
    Some(
        assigned
            .into_iter()
            .map(|mut ids| {
                ids.sort_unstable();
                ids.into_iter()
                    .map(|i| snapshot.bodies[i].clone())
                    .collect()
            })
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn largest_groups_keep_stable_ties_and_respect_capacity_at_all_widths() {
        for width in 2..=8 {
            let mut groups: Vec<_> = (0..width * 2)
                .rev()
                .map(|i| Group {
                    indices: vec![i * 2, i * 2 + 1],
                    work: if i < width { 60 } else { 3 },
                })
                .collect();
            let bins = assign(&mut groups, width, 4).unwrap();
            for (i, bin) in bins.into_iter().enumerate() {
                let tail = (width + i) * 2;
                assert_eq!(bin, [i * 2, i * 2 + 1, tail, tail + 1]);
            }
            assert!(assign(&mut groups, width, 3).is_none());
        }
    }
}
