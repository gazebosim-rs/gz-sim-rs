// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded input replay for a common solver with independently verified observable history.
use super::*;
use crate::floating_tree::cluster::{ClusterSnapshot, ClusterWorld, Version};
use std::io::BufWriter;
const MAX_EVENTS: u32 = 16_384;
const MAX_INTERVALS: u32 = 8_192;

/// Bounded replay is optional. A world with an exhausted journal can continue
/// separated until its swept bounds require a recorded merge.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GraspReplayBudget {
    pub max_intervals: u32,
    pub max_events: u32,
}
impl GraspReplayBudget {
    pub(super) fn validate(self) -> Result<()> {
        if self.max_intervals == 0
            || self.max_intervals > MAX_INTERVALS
            || self.max_events == 0
            || self.max_events > MAX_EVENTS
        {
            return Err(invalid(
                "grasp replay budget requires 1–8192 intervals and 1–16384 input events",
            ));
        }
        Ok(())
    }
}
/// Diagnostic wall time for the last verified promotion on this process.
/// No wall-clock reading contributes to a physical archive.
#[derive(Clone, Copy, Debug, Default)]
pub struct GraspReplayTiming {
    pub events: u32,
    pub replay_us: u64,
    pub install_us: u64,
    pub first_step_us: u64,
}
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
pub(super) enum Event {
    Queue(u64, u64),
    Motor(u64, Option<MotorSetting>),
    Step {
        efforts: Vec<(u64, u64)>,
        forces: Vec<(u64, [u64; 3])>,
    },
}
impl Event {
    pub fn step(es: &[(u64, f64)], fs: &[(u64, [f64; 3])]) -> Self {
        Self::Step {
            efforts: es.iter().map(|&(id, e)| (id, e.to_bits())).collect(),
            forces: fs
                .iter()
                .map(|&(id, f)| (id, f.map(f64::to_bits)))
                .collect(),
        }
    }
    fn valid(&self) -> bool {
        match self {
            Self::Queue(_, e) => f64::from_bits(*e).is_finite(),
            Self::Motor(_, setting) => setting.is_none_or(MotorSetting::valid),
            Self::Step { efforts, forces } => {
                efforts.len() <= 14
                    && forces.len() <= 2
                    && efforts.iter().all(|(_, e)| f64::from_bits(*e).is_finite())
                    && forces
                        .iter()
                        .all(|(_, f)| f.iter().all(|&x| f64::from_bits(x).is_finite()))
            }
        }
    }
    fn apply(&self, w: &mut ClusterWorld) -> Result<()> {
        match self {
            Self::Queue(id, e) => w.queue(*id, f64::from_bits(*e)),
            Self::Motor(id, m) => w.motor(*id, *m),
            Self::Step { efforts, forces } => {
                let e = efforts
                    .iter()
                    .map(|&(id, e)| (id, f64::from_bits(e)))
                    .collect::<Vec<_>>();
                let f = forces
                    .iter()
                    .map(|&(id, f)| (id, f.map(f64::from_bits)))
                    .collect::<Vec<_>>();
                w.advance(&e, &f)
            }
        }
    }
}
#[derive(Clone, Serialize, Deserialize)]
struct Run {
    event: Event,
    count: u32,
}
type Public<'a> = (
    &'a FloatingTreeObservation,
    &'a PayloadReceipt,
    &'a [Option<MotorSetting>],
);
fn chain(previous: [u8; 32], input: Option<&Event>, rows: [Public; 2]) -> Result<[u8; 32]> {
    let mut h = Sha256::new();
    h.update(b"GZGRASP-OBSERVABLE-HISTORY-1");
    h.update(previous);
    let mut stream = BufWriter::new(&mut h);
    bincode::serialize_into(&mut stream, &(input, rows)).map_err(io::Error::other)?;
    std::io::Write::flush(&mut stream)?;
    drop(stream);
    Ok(h.finalize().into())
}
fn snapshot_rows(c: &ClusterSnapshot) -> [Public<'_>; 2] {
    std::array::from_fn(|i| {
        let m = &c.members[i];
        (&m.tree, &m.part, &m.motors[..])
    })
}
fn saved_rows(c: &Saved) -> Result<[Public<'_>; 2]> {
    Ok([
        c.member(0).ok_or(invalid("missing first replay receipt"))?,
        c.member(1)
            .ok_or(invalid("missing second replay receipt"))?,
    ])
}
#[derive(Clone, Serialize, Deserialize)]
struct Journal {
    initial: [u8; 32],
    runs: Vec<Run>,
    expected: Vec<[u8; 32]>,
    steps: u32,
}
#[derive(Clone, Serialize, Deserialize)]
enum History {
    Recording(Journal),
    Expired { intervals: u32, events: u32 },
    Coupled,
}
#[derive(Clone, Serialize, Deserialize)]
pub(super) struct Replay {
    pub budget: GraspReplayBudget,
    history: History,
}
impl Replay {
    pub fn new(budget: GraspReplayBudget, c: &Saved) -> Result<Self> {
        budget.validate()?;
        let history = match c {
            Saved::Coupled { .. } => History::Coupled,
            Saved::Separate(_) => History::Recording(Journal {
                initial: chain([0; 32], None, saved_rows(c)?)?,
                runs: vec![],
                expected: vec![],
                steps: 0,
            }),
        };
        Ok(Self { budget, history })
    }
    pub fn validate(&self, c: &Saved) -> Result<()> {
        self.budget.validate()?;
        match (&self.history, c) {
            (History::Coupled, Saved::Coupled { .. }) => Ok(()),
            (History::Expired { intervals, events }, Saved::Separate(_))
                if *intervals <= self.budget.max_intervals
                    && *events <= self.budget.max_events
                    && u64::from(*intervals) <= c.tick()
                    && (*intervals == self.budget.max_intervals
                        || *events == self.budget.max_events) =>
            {
                Ok(())
            }
            (History::Recording(j), Saved::Separate(_)) => {
                let n = j.runs.iter().map(|r| u64::from(r.count)).sum::<u64>();
                let ticks = j
                    .runs
                    .iter()
                    .filter(|r| matches!(r.event, Event::Step { .. }))
                    .map(|r| u64::from(r.count))
                    .sum::<u64>();
                if n != j.expected.len() as u64
                    || n > u64::from(self.budget.max_events)
                    || ticks != u64::from(j.steps)
                    || ticks != c.tick()
                    || j.steps > self.budget.max_intervals
                    || j.runs.iter().any(|r| r.count == 0 || !r.event.valid())
                {
                    return Err(invalid("invalid bounded grasp replay history"));
                }
                Ok(())
            }
            _ => Err(invalid(
                "grasp replay history disagrees with native placement",
            )),
        }
    }
    pub fn record(&mut self, c: &Saved, e: Event) -> Result<()> {
        if matches!(c, Saved::Coupled { .. }) {
            self.history = History::Coupled;
            return Ok(());
        }
        let History::Recording(j) = &mut self.history else {
            return Ok(());
        };
        let step = matches!(e, Event::Step { .. });
        let events = j.expected.len() as u32;
        if events == self.budget.max_events || (step && j.steps == self.budget.max_intervals) {
            self.history = History::Expired {
                intervals: j.steps,
                events,
            };
            return Ok(());
        }
        let h = chain(
            j.expected.last().copied().unwrap_or(j.initial),
            Some(&e),
            saved_rows(c)?,
        )?;
        if let Some(last) = j.runs.last_mut().filter(|r| r.event == e) {
            last.count += 1;
        } else {
            j.runs.push(Run { event: e, count: 1 });
        }
        j.expected.push(h);
        j.steps += u32::from(step);
        Ok(())
    }
    pub fn recreate(&self, s: &GraspFleetSpec, c: &FleetCut) -> Result<(ClusterSnapshot, u32)> {
        let j = match &self.history {
            History::Recording(j) => j,
            History::Expired { intervals, events } => {
                return Err(ArticulationError::GraspReplayExhausted {
                    epoch: c.epoch,
                    tick: c.tick,
                    recorded_intervals: *intervals,
                    recorded_events: *events,
                });
            }
            History::Coupled => {
                return Err(invalid(
                    "coupled fleet cannot replay an independent history",
                ));
            }
        };
        let mut w = ClusterWorld::new(s, Version::ObservedShear)?;
        let mut event_number = 0usize;
        let mut h = chain([0; 32], None, snapshot_rows(&w.capture_without_image()?))?;
        if h != j.initial {
            return Err(ArticulationError::GraspReplayDiverged {
                epoch: c.epoch,
                tick: c.tick,
                first_event: 0,
                first_physical_tick: 0,
            });
        }
        for run in &j.runs {
            for _ in 0..run.count {
                run.event.apply(&mut w)?;
                let state = w.capture_without_image()?;
                h = chain(h, Some(&run.event), snapshot_rows(&state))?;
                if h != j.expected[event_number]
                    || state.pairs.iter().any(|r| {
                        r.reaction.active
                            || r.reaction.normal_impulse != 0.
                            || r.reaction.warmstart_normal_impulse != 0.
                            || r.reaction.tangent_impulse != [0.; 3]
                    })
                {
                    return Err(ArticulationError::GraspReplayDiverged {
                        epoch: c.epoch,
                        tick: c.tick,
                        first_event: event_number as u32 + 1,
                        first_physical_tick: state.tick,
                    });
                }
                event_number += 1;
            }
        }
        let snapshot = w.capture()?;
        if snapshot.tick != c.tick || event_number != j.expected.len() {
            return Err(invalid("verified replay ended at an incorrect interval"));
        }
        for (i, actual) in snapshot.members.iter().enumerate() {
            let source = &c.groups[i];
            if actual.tree != source.state
                || Some(&actual.part) != source.image.payload()
                || Some(&actual.motors[..]) != source.image.motors()
            {
                return Err(invalid(
                    "final verified replay state differs from accepted native members",
                ));
            }
        }
        Ok((snapshot, event_number as u32))
    }
}
