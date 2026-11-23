// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent one-way material rows can read the same guide trajectory concurrently.
use super::*;
use bevy_tasks::{TaskPool, TaskPoolBuilder};
use std::sync::{Arc, OnceLock};

/// Keep the caller as one solve lane. Lease reconstruction reuses its bounded helper pool.
#[derive(Clone, Default)]
pub(crate) struct Executor(Arc<OnceLock<TaskPool>>);
pub(super) struct Batch {
    lanes: usize,
    max_lanes: usize,
    assignments: Vec<usize>,
    executor: Executor,
}
impl Batch {
    pub fn new(bodies: &[SoftBody], limit: usize, executor: Executor) -> Self {
        let available = std::thread::available_parallelism().map_or(1, usize::from);
        let max_lanes = limit.min(available);
        let weights = bodies.iter().map(weight).collect::<Vec<_>>();
        let (lanes, assignments) = assign(&weights, max_lanes);
        Self {
            lanes,
            max_lanes,
            assignments,
            executor,
        }
    }
    pub fn lanes(&self) -> usize {
        self.lanes
    }
    pub fn step(
        &self,
        bodies: &mut [SoftBody],
        dt: Duration,
        bounds: &[Boundary],
        tracks: &[(usize, TranslationBoundary)],
    ) -> Result<()> {
        let solve = |body: &mut SoftBody, paths: &[(usize, &dyn BoundaryPath)]| {
            body.step_with_boundary_paths(dt, bounds, paths, DEFAULT_PATH_TOLERANCE)
        };
        if self.lanes == 1 {
            let paths = references(tracks);
            for b in bodies {
                solve(b, &paths)?;
            }
            return Ok(());
        }
        let mut groups: Vec<Vec<(usize, &mut SoftBody)>> =
            (0..self.lanes).map(|_| Vec::new()).collect();
        for (i, (b, &group)) in bodies.iter_mut().zip(&self.assignments).enumerate() {
            groups[group].push((i, b));
        }
        let pool = self.executor.0.get_or_init(|| {
            TaskPoolBuilder::new()
                .num_threads(self.max_lanes - 1)
                .thread_name("gz material solver".to_owned())
                .build()
        });
        let run = |rows: Vec<(usize, &mut SoftBody)>| {
            let paths = references(tracks);
            rows.into_iter()
                .find_map(|(i, b)| solve(b, &paths).err().map(|e| (i, e)))
        };
        let failures = pool.scope_with_executor(false, None, |scope| {
            for (i, group) in groups.into_iter().enumerate() {
                let run = &run;
                if i == 0 {
                    scope.spawn_on_scope(async move { run(group) });
                } else {
                    scope.spawn(async move { run(group) });
                }
            }
        });
        // The complete graph is private until every lane has finished and its cut is verified.
        // Return the earliest authored row's error independent of completion order.
        if let Some((_, e)) = failures.into_iter().flatten().min_by_key(|(i, _)| *i) {
            return Err(e.into());
        }
        Ok(())
    }
}
fn references(tracks: &[(usize, TranslationBoundary)]) -> Vec<(usize, &dyn BoundaryPath)> {
    tracks
        .iter()
        .map(|(i, v)| (*i, v as &dyn BoundaryPath))
        .collect()
}

pub(super) fn weight(body: &SoftBody) -> u64 {
    let cfg = body.config();
    let rows =
        body.particles().len() + body.constraints().len() + 4 * body.bending_constraints().len();
    rows as u64 * u64::from(cfg.substeps) * u64::from(cfg.iterations)
}
pub(super) fn distribute(weights: &[u64], n: usize) -> (Vec<u64>, Vec<usize>) {
    let mut order: Vec<usize> = (0..weights.len()).collect();
    order.sort_by_key(|&i| (std::cmp::Reverse(weights[i]), i));
    let mut assignment = vec![0; weights.len()];
    let mut work = vec![0u64; n];
    for i in order {
        let lane = (0..n).min_by_key(|&j| (work[j], j)).unwrap();
        work[lane] += weights[i];
        assignment[i] = lane;
    }
    (work, assignment)
}
fn assign(weights: &[u64], limit: usize) -> (usize, Vec<usize>) {
    let n = weights.len().min(limit);
    if n < 2 {
        return (1, vec![0; weights.len()]);
    }
    let (work, assignment) = distribute(weights, n);
    let total = work.iter().sum::<u64>();
    let saving = total - work.iter().max().unwrap();
    if saving < 4096 || saving < total / 8 {
        (1, vec![0; weights.len()])
    } else {
        (n, assignment)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cap_tiny_and_dominated_bundles_and_balance_interleaved_heavy_rows() {
        assert_eq!(assign(&[100, 100], 4).0, 1);
        assert_eq!(assign(&[90000, 300, 600, 200], 4).0, 1);
        assert_eq!(assign(&[6000, 7000, 6000, 7000], 2), (2, vec![0, 0, 1, 1]));
        assert_eq!(assign(&[7000; 8], 4), (4, vec![0, 1, 2, 3, 0, 1, 2, 3]));
        assert_eq!(assign(&[7000; 8], 1), (1, vec![0; 8]));
    }
}
