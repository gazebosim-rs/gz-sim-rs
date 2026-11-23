// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::io::{self, BufRead};
struct Participant {
    slot: u8,
    session: durable::Session,
    owner: Option<fs::File>,
    phase: Phase,
    epoch: u32,
    tick: u16,
    world: Option<Tree>,
    ticket: Option<Ticket>,
    image: Vec<u8>,
    last: Option<(Step, Reply)>,
}
impl Participant {
    fn new(slot: u8, session: durable::Session) -> Self {
        Self {
            slot,
            session,
            owner: None,
            phase: Phase::Idle,
            epoch: 0,
            tick: 0,
            world: None,
            ticket: None,
            image: Vec::new(),
            last: None,
        }
    }
    fn status(&self) -> Reply {
        Reply::Status {
            phase: self.phase,
            epoch: self.epoch,
            tick: self.tick,
        }
    }
    fn observed(&self) -> Result<Reply> {
        let w = self.world.as_ref().ok_or("missing owned world")?;
        Ok(Reply::Observed {
            epoch: self.epoch,
            tick: self.tick,
            local_time_ns: w.time_ns(),
            digest: Sha256::digest(w.image()?).into(),
            state: w.observe(),
        })
    }
    fn execute(&mut self, cmd: Op, image: &[u8], decision: &Path) -> Result<(Reply, Vec<u8>)> {
        let reject = |d| Ok((Reply::Denied(d), Vec::new()));
        if !matches!(cmd, Op::Prepare(_)) && !image.is_empty() {
            return reject(Denial::Invalid);
        }
        let reply = match cmd {
            Op::Inspect => self.status(),
            Op::Observe => {
                if self.phase != Phase::Active {
                    return reject(Denial::Fenced);
                }
                self.observed()?
            }
            Op::Boot => {
                if self.slot != 0
                    || decision.try_exists()?
                    || decision.with_extension("retired").try_exists()?
                {
                    return reject(Denial::Fenced);
                }
                if self.phase == Phase::Idle {
                    let Some(owner) = durable::try_lease(decision, "owner")? else {
                        return reject(Denial::Busy);
                    };
                    if decision.try_exists()? || decision.with_extension("retired").try_exists()? {
                        return reject(Denial::Fenced);
                    }
                    self.world = Some(Tree::warm()?);
                    self.owner = Some(owner);
                    self.epoch = 1;
                    self.tick = CUT;
                    self.phase = Phase::Active;
                }
                if self.phase != Phase::Active || self.tick != CUT {
                    return reject(Denial::Conflict);
                }
                self.status()
            }
            Op::Freeze { epoch, tick } => {
                if self.slot != 0
                    || epoch != self.epoch
                    || tick != CUT
                    || tick != self.tick
                    || !matches!(self.phase, Phase::Active | Phase::Frozen)
                {
                    return reject(Denial::Fenced);
                }
                if self.phase == Phase::Active {
                    let b = self.world.as_ref().ok_or("missing source")?.image()?;
                    self.ticket = Some(Ticket::new(self.session, &b));
                    self.image = b;
                    self.phase = Phase::Frozen;
                }
                return Ok((
                    Reply::Frozen(self.ticket.ok_or("missing freeze receipt")?),
                    self.image.clone(),
                ));
            }
            Op::Prepare(t) => {
                if self.slot != 1 || t.session != self.session || !t.check(image) {
                    return reject(Denial::Invalid);
                }
                if decision.try_exists()? && durable::committed(decision)?.0.ticket != t {
                    return reject(Denial::Conflict);
                }
                if self.phase == Phase::Idle {
                    let Ok(w) = Tree::restore(image) else {
                        return reject(Denial::Invalid);
                    };
                    if w.image()? != image {
                        return reject(Denial::Invalid);
                    }
                    self.world = Some(w);
                    self.ticket = Some(t);
                    self.phase = Phase::Prepared;
                    self.tick = t.tick;
                    self.epoch = t.target_epoch;
                }
                if self.phase != Phase::Prepared || self.ticket != Some(t) {
                    return reject(Denial::Conflict);
                }
                Reply::Prepared(t)
            }
            Op::Retire(t) => {
                if self.slot != 0
                    || self.ticket != Some(t)
                    || !matches!(self.phase, Phase::Frozen | Phase::Retired)
                {
                    return reject(Denial::Fenced);
                }
                if self.phase == Phase::Frozen {
                    durable::write_record(
                        &decision.with_extension("retired"),
                        Decision {
                            ticket: t,
                            retired: t.retirement()?,
                        },
                        &self.image,
                    )?;
                }
                self.phase = Phase::Retired;
                self.world = None;
                self.owner = None;
                self.image.clear();
                Reply::Retired(t.retirement()?)
            }
            Op::Activate(t) => {
                if self.slot != 1
                    || self.ticket != Some(t)
                    || !matches!(self.phase, Phase::Prepared | Phase::Active)
                {
                    return reject(Denial::Fenced);
                }
                if !decision.try_exists()? {
                    return reject(Denial::NoDecision);
                }
                let (d, _) = durable::committed(decision)?;
                if d.ticket != t {
                    return reject(Denial::Conflict);
                }
                if self.phase == Phase::Prepared {
                    let Some(owner) = durable::try_lease(decision, "owner")? else {
                        return reject(Denial::Busy);
                    };
                    self.owner = Some(owner);
                }
                self.phase = Phase::Active;
                Reply::Activated(t)
            }
            Op::Advance(s) => {
                if self.phase != Phase::Active
                    || self.epoch != s.epoch
                    || self.slot == 0 && decision.try_exists()?
                {
                    return reject(Denial::Fenced);
                }
                if let Some((previous, reply)) = self.last
                    && s.tick == previous.tick
                {
                    if s != previous {
                        return reject(Denial::Conflict);
                    }
                    return Ok((reply, Vec::new()));
                }
                if self.tick.checked_add(1) != Some(s.tick) || s.tick > CUT + SPAN {
                    return reject(Denial::Fenced);
                }
                let effort = s.effort.map(f64::from_bits);
                if effort.is_some_and(|e| !e.is_finite() || e.abs() > 4.) {
                    return reject(Denial::Invalid);
                }
                self.world
                    .as_mut()
                    .ok_or("missing active tree")?
                    .step(effort)?;
                self.tick = s.tick;
                let receipt = self.observed()?;
                self.last = Some((s, receipt));
                receipt
            }
        };
        Ok((reply, Vec::new()))
    }
}
pub(super) fn serve(slot: u8, decision: &Path) -> Result<()> {
    if slot > 1 {
        return Err("unknown hinge participant slot".into());
    }
    let mut w = Participant::new(slot, durable::session(decision)?);
    let mut input = io::stdin().lock();
    let mut out = io::stdout().lock();
    loop {
        if input.fill_buf()?.is_empty() {
            return Ok(());
        }
        let (r, img): (Request, _) = wire::read(&mut input)?;
        let (reply, body) = if r.session == w.session {
            w.execute(r.op, &img, decision)?
        } else {
            (Reply::Denied(Denial::Fenced), Vec::new())
        };
        if !r.omit_reply {
            wire::frame(&mut out, &Answer { id: r.id, reply }, &body)?
        }
    }
}
