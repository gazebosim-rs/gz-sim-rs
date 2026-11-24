// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{io, path::Path};
#[derive(Serialize, Deserialize)]
pub(super) enum Job {
    Batch(Vec<(u8, Request)>),
    Fetch(u8),
}
#[derive(Serialize, Deserialize)]
pub(super) enum Answer {
    Inline(Vec<(u8, Reply)>),
    Paged(Vec<u8>),
}
const ALLOW_INLINE: usize = crate::moving::worker::LIMIT - 4096;
/// A real process can hold several independent native worlds. One command prepares
/// every local assembly; large receipts can be fetched in bounded individual frames.
pub fn serve(slot: u8, lease: &Path) -> io::Result<()> {
    serve_members(slot, lease, MemberLane::Standard)
}
pub fn serve_warm(slot: u8, lease: &Path) -> io::Result<()> {
    serve_members(slot, lease, MemberLane::Warm)
}
pub fn serve_prior(slot: u8, lease: &Path) -> io::Result<()> {
    serve_members(slot, lease, MemberLane::RetainedPrior)
}
fn serve_members(slot: u8, lease: &Path, lane: MemberLane) -> io::Result<()> {
    if slot >= 8 || !lease.is_absolute() {
        return Err(io::Error::other("invalid grasp fleet worker slot"));
    }
    crate::moving::worker::serve_profile(slot, lease, lane.wire(), move |spec: GraspFleetSpec| {
        spec.check_ids()?;
        let mut roles: Vec<RoleWorker> = (0..spec.assemblies.len())
            .map(|_| RoleWorker::default())
            .collect();
        let mut pages: BTreeMap<u8, Reply> = BTreeMap::new();
        let mut page_tick = None;
        Ok(move |tick, job: Job, steps: &mut u64| match job {
            Job::Fetch(group) if page_tick == Some(tick) => {
                let reply = pages
                    .remove(&group)
                    .ok_or(invalid("missing grasp fleet receipt page"))?;
                Ok(Answer::Inline(vec![(group, reply)]))
            }
            Job::Fetch(_) => Err(invalid("stale grasp fleet receipt page")),
            Job::Batch(requests) => {
                if !pages.is_empty()
                    || requests.is_empty()
                    || requests.len() > roles.len()
                    || !requests.windows(2).all(|w| w[0].0 < w[1].0)
                {
                    return Err(invalid("incorrect or uncollected grasp fleet batch"));
                }
                let mut answers = Vec::with_capacity(requests.len());
                for (id, req) in requests {
                    if matches!(&req.op, Op::WarmMember) && lane == MemberLane::Standard {
                        return Err(invalid(
                            "warm export requires the v2 native grasp fleet worker",
                        ));
                    }
                    if matches!(&req.op, Op::PriorWarm { .. }) && lane != MemberLane::RetainedPrior
                    {
                        return Err(invalid(
                            "retained prior proof requires v3 grasp fleet worker",
                        ));
                    }
                    if !matches!(
                        &req.op,
                        Op::Stage(Seed::Install(Piece::Coupled(_)))
                            | Op::Activate(_)
                            | Op::Fence(_)
                            | Op::ClearMember
                            | Op::WarmMember
                            | Op::PriorWarm { .. }
                            | Op::Queue(..)
                            | Op::Motor(..)
                            | Op::Step { bounds: true, .. }
                    ) {
                        return Err(invalid(
                            "grasp fleet request requires a coupled assembly and conservative bounds",
                        ));
                    }
                    let i = id as usize;
                    let r = roles
                        .get_mut(i)
                        .ok_or(invalid("unknown grasp fleet assembly"))?
                        .handle(
                            &spec.assemblies[i],
                            tick,
                            req,
                            steps,
                            lane == MemberLane::RetainedPrior,
                        )?;
                    answers.push((id, r));
                }
                if answers.len() > 1
                    && bincode::serialized_size(&answers).map_err(io::Error::other)?
                        > ALLOW_INLINE as u64
                {
                    let groups = answers.iter().map(|x| x.0).collect();
                    pages.extend(answers);
                    page_tick = Some(tick);
                    Ok(Answer::Paged(groups))
                } else {
                    Ok(Answer::Inline(answers))
                }
            }
        })
    })
}
use std::collections::BTreeMap;
