// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualification and traffic observations for a loaded four-group ownership change.
use super::Result;
use gz_physics_distributed::{coordinator::Error, physics::Body, pool::WorkerPool};
use serde::Serialize;
use std::time::Instant;

// Some readers of the shared recovery fixture exercise only the ordinary single-slab handoff.
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum SplitHandoff {
    Encoding(bool),
    CompareBatch { atomic: bool },
}
#[derive(Default, Serialize)]
pub struct WireUsage {
    pub packed: bool,
    pub atomic: bool,
    pub negotiated_peers: Vec<usize>,
    pub dropped_prepare: u64,
    /// Number of the four groups published on their destination when a prepare ACK was lost.
    pub already_moved_at_failure: usize,
    pub transfers: Vec<Transfer>,
}
#[derive(Serialize)]
pub struct Transfer {
    pub ids: Vec<u64>,
    pub requests: u64,
    pub replies: u64,
    pub request_bytes: u64,
    pub reply_bytes: u64,
    pub successful_call_us: f64,
}
impl SplitHandoff {
    pub fn usage(self) -> WireUsage {
        WireUsage {
            packed: !matches!(self, Self::Encoding(false)),
            atomic: matches!(self, Self::CompareBatch { atomic: true }),
            ..Default::default()
        }
    }
}
impl WireUsage {
    pub fn negotiate(&mut self, pool: &mut WorkerPool) -> Result<()> {
        let peers = pool.set_packed_handoffs(self.packed);
        if peers != usize::from(self.packed) * 4 {
            return Err("native handoff encoding unavailable".into());
        }
        self.negotiated_peers.push(peers);
        Ok(())
    }
    pub fn execute(
        &mut self,
        pool: &mut WorkerPool,
        groups: &[&Vec<Body>],
        destination: u8,
        loaded: u64,
        mode: SplitHandoff,
    ) -> Result<()> {
        // The encoding experiment drops the loaded slab's prepare. The batching experiment drops
        // the third group's prepare after the destination has handled two prepare/commit pairs.
        let fault = match mode {
            SplitHandoff::Encoding(_) => groups
                .iter()
                .position(|g| g[0].id == loaded)
                .ok_or("missing loaded group")?,
            SplitHandoff::CompareBatch { .. } => 2,
        };
        let moves: Vec<_> = groups.iter().map(|g| (g[0].id, destination)).collect();
        self.dropped_prepare = moves[fault].0;
        let n = if self.atomic { groups.len() } else { 1 };
        for (i, chunk) in groups.chunks(n).enumerate() {
            let begin = i * n;
            let plan = &moves[begin..begin + chunk.len()];
            let transfer = |p: &mut WorkerPool| -> std::result::Result<(), Error> {
                if self.atomic {
                    p.transfer_components(plan)?;
                } else {
                    p.transfer_component(plan[0].0, destination)?;
                }
                Ok(())
            };
            if (begin..begin + chunk.len()).contains(&fault) {
                let before = pool.frame().clone();
                pool.drop_acknowledgement_after(destination, 2 * (fault - begin))?;
                if !matches!(transfer(pool), Err(Error::Transport(_))) || pool.frame() != &before {
                    return Err("lost prepare ACK changed published ownership".into());
                }
                self.already_moved_at_failure = groups
                    .iter()
                    .filter(|g| before.body(g[0].id).is_some_and(|(w, _)| w == destination))
                    .count();
                if self.already_moved_at_failure != begin {
                    return Err("incorrect partial publication".into());
                }
            }
            let traffic = pool.traffic();
            let started = Instant::now();
            transfer(pool)?;
            let elapsed = started.elapsed();
            let mut sample = Transfer {
                ids: chunk.iter().flat_map(|g| g.iter().map(|b| b.id)).collect(),
                requests: 0,
                replies: 0,
                request_bytes: 0,
                reply_bytes: 0,
                successful_call_us: elapsed.as_secs_f64() * 1e6,
            };
            for (a, b) in traffic.iter().zip(pool.traffic()) {
                sample.requests += b.requests - a.requests;
                sample.replies += b.replies - a.replies;
                sample.request_bytes += b.request_bytes - a.request_bytes;
                sample.reply_bytes += b.reply_bytes - a.reply_bytes;
            }
            if chunk.iter().any(|g| {
                g.iter()
                    .any(|b| pool.frame().body(b.id).map(|v| v.0) != Some(destination))
            }) {
                return Err("handoff omitted group members".into());
            }
            self.transfers.push(sample);
        }
        Ok(())
    }
}
