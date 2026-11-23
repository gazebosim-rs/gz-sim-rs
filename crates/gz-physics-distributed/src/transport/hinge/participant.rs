// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exactly-once sequencing and generation gates for one worker's managed hinge slot.
use super::*;
use crate::articulation::NativeArticulation;

struct Resident {
    world: NativeArticulation,
    image: Vec<u8>,
    generation: u64,
    active: bool,
}

pub(super) struct Participant {
    run: u64,
    id: u8,
    spec: ArticulationSpec,
    resident: Option<Resident>,
    retired: Option<(u64, u64)>,
    last: Option<([u8; 32], HingeReply)>,
}

impl Participant {
    pub(super) fn new(run: u64, id: u8, spec: ArticulationSpec) -> Self {
        Self {
            run,
            id,
            spec,
            resident: None,
            retired: None,
            last: None,
        }
    }

    pub(super) fn apply(&mut self, req: HingeRequest) -> Result<HingeReply, TransportError> {
        if req.run != self.run || req.worker != self.id {
            return Err(wire("wrong hinge request address"));
        }
        // Hash the typed request, including the IEEE bits of joint effort. An
        // exact retry receives the previous result even after a fence or step.
        let fingerprint = req.digest()?;
        let previous = self.last.as_ref().map_or(0, |(_, r)| r.sequence);
        if let Some((hash, r)) = &self.last
            && req.sequence == previous
            && *hash == fingerprint
        {
            return Ok(r.clone());
        }
        if previous.checked_add(1) != Some(req.sequence) {
            return Err(wire("out of order or conflicting hinge sequence"));
        }
        let generation = req.generation;
        let tick = req.tick;
        match req.command {
            HingeCommand::Stage { image } => {
                if self.resident.is_some()
                    || self
                        .retired
                        .is_some_and(|(g, t)| generation <= g || tick < t)
                {
                    return Err(wire("occupied or retired hinge generation"));
                }
                let world = NativeArticulation::restore(&self.spec, tick, &image).map_err(wire)?;
                self.resident = Some(Resident {
                    world,
                    image,
                    generation,
                    active: false,
                });
            }
            command => {
                let r = self
                    .resident
                    .as_mut()
                    .ok_or_else(|| wire("empty hinge worker"))?;
                if r.generation != generation || r.world.tick() != tick {
                    return Err(wire("stale hinge generation or tick"));
                }
                match command {
                    HingeCommand::Stage { .. } => unreachable!(),
                    HingeCommand::Activate { digest }
                        if !r.active && same_digest(&r.image, digest) =>
                    {
                        r.active = true;
                    }
                    HingeCommand::Fence { digest } if r.active && same_digest(&r.image, digest) => {
                        self.retired = Some((generation, tick));
                        self.resident = None;
                    }
                    HingeCommand::Observe if r.active => {}
                    HingeCommand::QueueEffort { .. } | HingeCommand::Advance if r.active => {
                        let updated = match command {
                            HingeCommand::QueueEffort { joint_id, effort } => {
                                r.world.effort(joint_id, effort)
                            }
                            _ => r.world.advance(),
                        }
                        .and_then(|()| r.world.capture());
                        match updated {
                            Ok(image) => r.image = image,
                            Err(e) => {
                                // A failed native step may have modified solver caches.
                                // Reinstall the previous acknowledged image before any retry.
                                match NativeArticulation::restore(&self.spec, tick, &r.image) {
                                    Ok(world) => r.world = world,
                                    Err(failure) => {
                                        self.resident = None;
                                        self.retired = Some((generation, tick));
                                        return Err(wire(format!(
                                            "hinge slot fenced after failed rollback: {failure}"
                                        )));
                                    }
                                }
                                return Err(wire(e));
                            }
                        }
                    }
                    _ => {
                        return Err(wire(
                            "hinge operation does not match the prepared image or ownership phase",
                        ));
                    }
                }
            }
        }
        let state = self.resident.as_ref().map(|r| r.world.observe());
        let reply = HingeReply {
            request_digest: fingerprint,
            run: self.run,
            worker: self.id,
            sequence: req.sequence,
            generation,
            tick: state.as_ref().map_or(tick, |s| s.tick),
            state,
            image: self
                .resident
                .as_ref()
                .map_or_else(Vec::new, |r| r.image.clone()),
        };
        self.last = Some((fingerprint, reply.clone()));
        Ok(reply)
    }
}

fn same_digest(image: &[u8], expected: [u8; 32]) -> bool {
    <[u8; 32]>::from(Sha256::digest(image)) == expected
}
