// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::physics::ComponentInfo;

impl<P: Peer> Coordinator<P> {
    /// At the current published tick transfer a single isolated body to the
    /// opposite worker, optionally promoting prescribed translation to dynamics.
    pub fn transfer(&mut self, body_id: u64, promote: bool) -> Result<&Frame, Error> {
        if self.queued.iter().any(|p| p.id == body_id) {
            return Err(Error::Invalid(
                "pending inputs on this body require a warm transfer",
            ));
        }
        let (source, body) = self
            .frame
            .body(body_id)
            .ok_or(Error::Invalid("unknown transfer body"))?;
        let source_obs = &self.frame.workers[usize::from(source)];
        if source_obs
            .components
            .iter()
            .any(|c| c.contact_history && c.ids.binary_search(&body_id).is_ok())
        {
            return Err(Error::Invalid(
                "contact history prevents cache-free transfer",
            ));
        }
        if promote && body.tier == crate::physics::Tier::Fixed {
            return Err(Error::Invalid("cannot promote a fixed body"));
        }
        let expected = if promote {
            body.promoted()
        } else {
            body.clone()
        };
        let dest = 1 - source;
        let (s, d) = (usize::from(source), usize::from(dest));
        if !self.frame.can_receive(d, &[body_id]) {
            return Err(Error::Capacity);
        }
        let generation = self.frame.workers[s].generations[&body_id]
            .checked_add(1)
            .ok_or(Error::Capacity)?;
        let ticket = Ticket {
            run_id: self.run,
            serial: self.serial.checked_add(1).ok_or(Error::Capacity)?,
            source,
            destination: dest,
            body_id,
            generation,
            tick: self.frame.tick,
            promote,
        };
        let mut topology = self.frame.workers.each_ref().map(|w| w.components.clone());
        topology[s].retain_mut(|c| {
            c.ids.retain(|&id| id != body_id);
            !c.ids.is_empty()
        });
        topology[s].sort_by_key(|c| c.ids[0]);
        topology[d].push(ComponentInfo {
            ids: vec![body_id],
            contact_history: false,
        });
        topology[d].sort_by_key(|c| c.ids[0]);
        let mut projected = self.frame.workers[d].snapshot.clone();
        projected.bodies.push(expected.clone());
        if !partition_can_step(&projected, &topology[d], &self.queued) {
            return Err(Error::Colocation);
        }
        let mut counts = [2; 2];
        counts[s] = 3;
        let mut cursor = self.begin(Action::Transfer(ticket), counts)?;
        let Outcome::Exported(handoff) = self.call(&mut cursor, s, Command::Export { ticket })?
        else {
            return Err(Error::Invalid("export acknowledgement"));
        };
        if handoff.ticket != ticket || &handoff.body != self.frame.body(body_id).unwrap().1 {
            return Err(Error::Invalid("export changed identity or state"));
        }
        let Outcome::Prepared(prepared) = self.call(
            &mut cursor,
            d,
            Command::Prepare {
                handoff: handoff.clone(),
            },
        )?
        else {
            return Err(Error::Invalid("prepare acknowledgement"));
        };
        if prepared.handoff != handoff {
            return Err(Error::Invalid("prepare changed transfer"));
        }
        let Outcome::Released(released) =
            self.call(&mut cursor, s, Command::Release { prepared })?
        else {
            return Err(Error::Invalid("release acknowledgement"));
        };
        if released.handoff != handoff {
            return Err(Error::Invalid("release changed transfer"));
        }
        let Outcome::Activated(on_dest) =
            self.call(&mut cursor, d, Command::Activate { released })?
        else {
            return Err(Error::Invalid("activation acknowledgement"));
        };
        let on_source = observation(self.call(&mut cursor, s, Command::Observe)?)?;
        let mut observations = [on_source, on_dest];
        if s == 1 {
            observations.swap(0, 1);
        }
        let next = Frame::new(self.frame.tick, observations)?;
        if next.workers[s].generations.len() + 1 != self.frame.workers[s].generations.len()
            || next.workers[d].generations.len() != self.frame.workers[d].generations.len() + 1
        {
            return Err(Error::Invalid(
                "cold transfer changed body count or contact history",
            ));
        }
        if next.body(body_id) != Some((dest, &expected))
            || next.workers[d].generations[&body_id] != generation
        {
            return Err(Error::Invalid("transfer discontinuity"));
        }
        for obs in &self.frame.workers {
            for other in &obs.snapshot.bodies {
                if other.id == body_id {
                    continue;
                }
                let (owner, _) = self.frame.body(other.id).unwrap();
                if next.body(other.id) != Some((owner, other))
                    || next.workers[owner as usize].generations[&other.id]
                        != obs.generations[&other.id]
                {
                    return Err(Error::Invalid("unrelated body changed during transfer"));
                }
            }
        }
        for (i, current) in self.frame.workers.iter().enumerate() {
            let next = &next.workers[i];
            if next.components != topology[i]
                || next.snapshot.contact_history != current.snapshot.contact_history
                || next.snapshot.impulses != current.snapshot.impulses
            {
                return Err(Error::Invalid(
                    "cold transfer changed native component or contact state",
                ));
            }
        }
        self.frame = next;
        self.serial = ticket.serial;
        self.commit_action();
        Ok(&self.frame)
    }
}
