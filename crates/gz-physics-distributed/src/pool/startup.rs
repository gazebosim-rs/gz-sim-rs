// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded startup for independent, index-ordered pool endpoints.
use std::{io, thread};

/// Every launch owns a private endpoint and seed. Open at most two at a time
/// and preserve slot order regardless of how quickly either child becomes ready.
pub(super) fn start_in_pairs<S: Sync, T: Send, E: From<io::Error> + Send>(
    seeds: &[S],
    start: impl Fn(usize, &S) -> Result<T, E> + Sync,
) -> Result<Vec<T>, E> {
    let mut started = Vec::with_capacity(seeds.len());
    let mut pairs = seeds.chunks_exact(2);
    for pair in &mut pairs {
        let index = started.len();
        let ready = thread::scope(|scope| -> Result<_, E> {
            let left = thread::Builder::new().spawn_scoped(scope, || start(index, &pair[0]))?;
            let right = start(index + 1, &pair[1]);
            // Even on failure, collect both results and drop this pair and its
            // predecessors before returning. Managed child destructors kill and reap.
            let left = left
                .join()
                .map_err(|_| io::Error::other("worker bootstrap panicked"))??;
            Ok([left, right?])
        })?;
        started.extend(ready);
    }
    if let [last] = pairs.remainder() {
        started.push(start(started.len(), last)?);
    }
    Ok(started)
}
