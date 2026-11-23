// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local-process timing on the same Linux monotonic clock. All output is opt-in.
#[path = "barrier_trace_sched.rs"]
mod sched;
use rustix::time::{ClockId, clock_gettime};
use sched::{Sample, ThreadStat};
use serde::Serialize;
use std::{
    cell::RefCell,
    fs::OpenOptions,
    io::{self, Write},
    path::PathBuf,
    sync::{Arc, Mutex},
    thread::{self, ThreadId},
};
use zenoh::handlers::{Callback, FifoChannel, FifoChannelHandler, IntoHandler};

pub(crate) fn ns(clock: ClockId) -> u64 {
    let t = clock_gettime(clock);
    t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64
}

pub(crate) fn now() -> u64 {
    ns(ClockId::Monotonic)
}

#[derive(Serialize)]
struct Event {
    name: &'static str,
    ns: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    peer: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    world: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    helper: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    cpu_ns: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    sched: Option<Sample>,
}

pub(crate) struct Record {
    caller: ThreadId,
    scheduler: bool,
    join: bool,
    caller_stat: Option<Arc<ThreadStat>>,
    helper_stat: Arc<Mutex<Option<Arc<ThreadStat>>>>,
    events: Mutex<Vec<Event>>,
}
thread_local! {
    // Worker processes reuse one request thread and one helper. The analyzer
    // also checks the endpoint TIDs when a pool has been replaced.
    static HELPER_STAT: Arc<Mutex<Option<Arc<ThreadStat>>>> = Arc::default();
    static CURRENT: RefCell<Option<Arc<Record>>> = const { RefCell::new(None) };
}

pub(crate) struct Scope {
    path: PathBuf,
    tick: u64,
    previous: Option<Arc<Record>>,
}

impl Scope {
    pub(crate) fn begin(tick: u64, worker: Option<u8>) -> Option<Self> {
        let dir = std::env::var_os("GZ_SIM_BARRIER_TRACE_DIR")?;
        let role = worker.map_or_else(|| "coord".to_owned(), |w| format!("worker-{w}"));
        let path = PathBuf::from(dir).join(format!("{role}-{}.jsonl", std::process::id()));
        let join = std::env::var_os("GZ_SIM_JOIN_TRACE").is_some();
        let previous = CURRENT.replace(Some(Arc::new(Record {
            caller: thread::current().id(),
            scheduler: join || std::env::var_os("GZ_SIM_SCHED_TRACE").is_some(),
            join,
            caller_stat: if join { ThreadStat::current() } else { None },
            helper_stat: HELPER_STAT.with(Clone::clone),
            events: Mutex::new(Vec::with_capacity(64)),
        })));
        Some(Self {
            path,
            tick,
            previous,
        })
    }
}

impl Drop for Scope {
    fn drop(&mut self) {
        let Some(record) = CURRENT.replace(self.previous.take()) else {
            return;
        };
        let write = || -> io::Result<()> {
            #[derive(Serialize)]
            struct Row<'a> {
                tick: u64,
                events: &'a [Event],
            }
            let events = record.events.lock().expect("trace lock");
            let mut bytes = serde_json::to_vec(&Row {
                tick: self.tick,
                events: &events,
            })?;
            bytes.push(b'\n');
            OpenOptions::new()
                .create(true)
                .append(true)
                .open(&self.path)?
                .write_all(&bytes)
        };
        if let Err(e) = write() {
            eprintln!("barrier trace {}: {e}", self.path.display());
        }
    }
}

pub(crate) fn current() -> Option<Arc<Record>> {
    CURRENT.with_borrow(Clone::clone)
}
impl Record {
    fn at(&self, name: &'static str, ns: u64, peer: Option<u8>) {
        self.events.lock().expect("trace lock").push(Event {
            name,
            ns,
            peer,
            world: None,
            helper: None,
            cpu_ns: None,
            sched: None,
        });
    }

    fn sample(&self, name: &'static str, peer: Option<u8>, stat: Option<&ThreadStat>, own: bool) {
        let sched = stat.and_then(ThreadStat::sample);
        let cpu_ns = own.then(|| ns(ClockId::ThreadCPUTime));
        let ns = now();
        self.events.lock().expect("trace lock").push(Event {
            name,
            ns,
            peer,
            world: None,
            helper: None,
            cpu_ns,
            sched,
        });
    }

    // Both task-exit samples refer to the calling worker thread. When the
    // helper finishes last it reads the caller's cached procfs descriptor.
    fn join_sample(&self, name: &'static str) {
        let helper = thread::current().id() != self.caller;
        let sched = self.caller_stat.as_deref().and_then(ThreadStat::sample);
        let cpu_ns = (!helper).then(|| ns(ClockId::ThreadCPUTime));
        let stamp = now();
        self.events.lock().expect("trace lock").push(Event {
            name,
            ns: stamp,
            peer: None,
            world: None,
            helper: Some(helper),
            cpu_ns,
            sched,
        });
    }

    pub(crate) fn helper_entry(&self) {
        let current = ThreadStat::current();
        self.sample("helper_entry", None, current.as_deref(), true);
        *self.helper_stat.lock().expect("trace lock") = current;
    }

    pub(crate) fn native<T>(&self, world: u64, solve: impl FnOnce() -> T) -> T {
        let helper = thread::current().id() != self.caller;
        let start_cpu = ns(ClockId::ThreadCPUTime);
        let start = now();
        let result = solve();
        let end = now();
        let end_cpu = ns(ClockId::ThreadCPUTime);
        let mut events = self.events.lock().expect("trace lock");
        for (name, stamp, cpu) in [
            ("native_start", start, start_cpu),
            ("native_end", end, end_cpu),
        ] {
            events.push(Event {
                name,
                ns: stamp,
                peer: None,
                world: Some(world),
                helper: Some(helper),
                cpu_ns: Some(cpu),
                sched: None,
            });
        }
        result
    }
}
// Capture the target thread's counters on the submitting thread. No first-tick
// value is invented when this pool's helper has never run before.
pub(crate) fn helper_submit() -> Option<Arc<Record>> {
    let trace = current().filter(|r| r.scheduler)?;
    {
        let target = trace.helper_stat.lock().expect("trace lock");
        trace.sample("helper_submit", None, target.as_deref(), false);
    }
    Some(trace)
}

/// Marks the task body returning, including when a solver returns an error.
pub(crate) struct LaneEnd(Option<Arc<Record>>);

impl LaneEnd {
    pub(crate) fn new(trace: Option<Arc<Record>>) -> Self {
        Self(trace.filter(|r| r.join))
    }
}
impl Drop for LaneEnd {
    fn drop(&mut self) {
        if let Some(trace) = &self.0 {
            trace.join_sample("lane_done");
        }
    }
}

pub(crate) fn scope_returned() {
    if let Some(trace) = current().filter(|r| r.join) {
        trace.join_sample("scope_return");
    }
}

pub(crate) fn at(name: &'static str, stamp: u64) {
    if let Some(trace) = current() {
        trace.at(name, stamp, None);
    }
}
pub(crate) fn mark(name: &'static str, peer: Option<u8>) {
    if let Some(trace) = current() {
        if trace.join && matches!(name, "batch_start" | "batch_end") {
            trace.join_sample(name);
        } else if trace.scheduler && matches!(name, "collect_start" | "received" | "decoded") {
            trace.sample(name, peer, ThreadStat::current().as_deref(), true);
        } else {
            trace.at(name, now(), peer);
        }
    }
}
pub(crate) fn reply_fifo(
    peer: u8,
) -> (
    Callback<zenoh::query::Reply>,
    FifoChannelHandler<zenoh::query::Reply>,
) {
    let (cb, fifo) = FifoChannel::new(1).into_handler();
    match current() {
        Some(record) => (
            Callback::from(move |reply| {
                record.at("ready", now(), Some(peer));
                cb.call(reply);
            }),
            fifo,
        ),
        None => (cb, fifo),
    }
}
