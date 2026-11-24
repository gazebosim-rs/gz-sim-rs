// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Schedule disjoint native worlds on the caller and one worker-local helper.
use super::{Island, SolveSchedule, WorldError};
use std::{cmp::Reverse, ops::Range, sync::Mutex};

type LaneResult = Result<(), (usize, WorldError)>;

#[derive(Debug, PartialEq, Eq)]
pub(super) enum Plan {
    Fixed(Range<usize>),
    Shared,
}

fn cost(island: &Island, schedule: SolveSchedule<'_>) -> (usize, bool) {
    let state = &island.committed;
    let fine = schedule.fine(state.bodies[0].id);
    let units = 1 + state.bodies.len() + state.impulses.len() + state.supports.len();
    (units * if fine { 10 } else { 1 }, fine)
}

pub(super) fn plan(worlds: &[Island], schedule: SolveSchedule<'_>) -> Option<Plan> {
    select(worlds.iter().map(|s| cost(s, schedule)))
}

fn select(costs: impl ExactSizeIterator<Item = (usize, bool)>) -> Option<Plan> {
    let count = costs.len();
    let mut first = None;
    let mut equal = true;
    let mut total = 0;
    let mut fine_count = 0;
    let mut largest = (0, 0);
    for (index, (weight, fine)) in costs.enumerate() {
        equal &= *first.get_or_insert((weight, fine)) == (weight, fine);
        total += weight;
        if fine {
            fine_count += 1;
            if weight > largest.0 {
                largest = (weight, index);
            }
        }
    }
    if fine_count < 2 {
        return None;
    }
    if equal {
        return Some(Plan::Fixed(0..count / 2));
    }
    // One indivisible fine world already takes at least half the total work.
    if largest.0 >= total - largest.0 {
        return Some(Plan::Fixed(largest.1..largest.1 + 1));
    }
    Some(Plan::Shared)
}

impl Plan {
    pub(super) fn run(
        self,
        pool: &bevy_tasks::TaskPool,
        worlds: &mut [Island],
        schedule: SolveSchedule<'_>,
        advance: impl Fn(usize, &mut Island) -> Result<(), WorldError> + Sync,
    ) -> Result<(), WorldError> {
        let results = match self {
            Self::Fixed(range) => {
                let (before, rest) = worlds.split_at_mut(range.start);
                let (middle, after) = rest.split_at_mut(range.len());
                let batch = |worlds: &mut [Island], start: usize| {
                    worlds.iter_mut().enumerate().try_for_each(|(i, s)| {
                        advance(start + i, s).map_err(|err| (start + i, err))
                    })
                };
                let values = pool.scope_with_executor(false, None, |scope| {
                    #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
                    let trace = crate::barrier_trace::helper_submit();
                    scope.spawn(async move {
                        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
                        if let Some(record) = &trace {
                            record.helper_entry();
                        }
                        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
                        let _done = crate::barrier_trace::LaneEnd::new(trace);
                        batch(middle, range.start)
                    });
                    scope.spawn_on_scope(async {
                        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
                        let _done =
                            crate::barrier_trace::LaneEnd::new(crate::barrier_trace::current());
                        batch(before, 0)?;
                        batch(after, range.end)
                    });
                });
                #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
                crate::barrier_trace::scope_returned();
                values
            }
            Self::Shared => share(pool, worlds, schedule, advance),
        };
        // Wall time determines who claims each world. Fault publication still
        // uses canonical world order, after both lanes have joined.
        results
            .into_iter()
            .filter_map(Result::err)
            .min_by_key(|(i, _)| *i)
            .map_or(Ok(()), |(_, error)| Err(error))
    }
}

struct Job<'a> {
    index: usize,
    weight: usize,
    fine: bool,
    world: &'a mut Island,
}

fn share(
    pool: &bevy_tasks::TaskPool,
    worlds: &mut [Island],
    schedule: SolveSchedule<'_>,
    advance: impl Fn(usize, &mut Island) -> Result<(), WorldError> + Sync,
) -> Vec<LaneResult> {
    let mut jobs: Vec<_> = worlds
        .iter_mut()
        .enumerate()
        .map(|(index, world)| {
            let (weight, fine) = cost(world, schedule);
            Job {
                index,
                weight,
                fine,
                world,
            }
        })
        .collect();
    jobs.sort_unstable_by_key(|j| (Reverse(j.weight), j.index));
    // Reserve the two most expensive fine worlds, one on each lane. The
    // remainder stays in descending estimated cost, including coarse worlds.
    let mut fine = jobs.iter().enumerate().filter(|(_, j)| j.fine);
    let first = fine.next().unwrap().0;
    let second = fine.next().unwrap().0;
    let helper = jobs.remove(second);
    let caller = jobs.remove(first);
    let queue = Mutex::new(jobs.into_iter());
    let run = |first| {
        let mut failure: Option<(usize, WorldError)> = None;
        let mut next = Some(first);
        while let Some(Job { index, world, .. }) = next {
            if failure.as_ref().is_none_or(|(failed, _)| index < *failed)
                && let Err(error) = advance(index, world)
            {
                failure = Some((index, error));
            }
            // Drop the queue guard before starting the next native solve.
            next = queue.lock().unwrap().next();
        }
        failure.map_or(Ok(()), Err)
    };
    let values = pool.scope_with_executor(false, None, |scope| {
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        let trace = crate::barrier_trace::helper_submit();
        scope.spawn(async move {
            #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
            if let Some(record) = &trace {
                record.helper_entry();
            }
            #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
            let _done = crate::barrier_trace::LaneEnd::new(trace);
            run(helper)
        });
        scope.spawn_on_scope(async {
            #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
            let _done = crate::barrier_trace::LaneEnd::new(crate::barrier_trace::current());
            run(caller)
        });
    });
    #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
    crate::barrier_trace::scope_returned();
    values
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_splits_are_optimal_and_sharing_reserves_two_fine_worlds() {
        let kinds = [(1, false), (7, false), (10, true), (70, true)];
        for n in 0..=7 {
            for layout in 0..4usize.pow(n as u32) {
                let jobs: Vec<_> = (0..n).map(|i| kinds[(layout >> (2 * i)) & 3]).collect();
                let fine = jobs.iter().filter(|j| j.1).count();
                let total: usize = jobs.iter().map(|j| j.0).sum();
                let peak = |range: Range<usize>| {
                    let block = &jobs[range];
                    let n_fine = block.iter().filter(|j| j.1).count();
                    let work: usize = block.iter().map(|j| j.0).sum();
                    (0 < n_fine && n_fine < fine).then_some(work.max(total - work))
                };
                match select(jobs.iter().copied()) {
                    None => assert!(fine < 2),
                    Some(Plan::Fixed(r)) => {
                        let best = (0..n)
                            .flat_map(|i| (i + 1..=n).map(move |j| i..j))
                            .filter_map(&peak)
                            .min();
                        assert_eq!(peak(r), best);
                    }
                    Some(Plan::Shared) => {
                        assert!(fine >= 2);
                        assert!(jobs.iter().filter(|j| j.1).all(|j| j.0 < total - j.0));
                        assert!(jobs.windows(2).any(|w| w[0] != w[1]));
                    }
                }
            }
        }
    }
}
