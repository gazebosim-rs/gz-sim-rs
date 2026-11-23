// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Experimental private-process ownership transfer for one complete native hinge.
use clap::{Parser, Subcommand};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    fs,
    io::{BufReader, Read},
    path::{Path, PathBuf},
    process::{Child, ChildStdin, Command, Stdio},
    sync::mpsc::{self, Receiver, RecvTimeoutError},
    time::{Duration, Instant},
};
#[path = "articulated_handoff/durable.rs"]
mod durable;
#[path = "articulated_handoff/fixture.rs"]
mod fixture;
#[path = "articulated_handoff/participant_faults.rs"]
mod participant_faults;
#[path = "articulated_handoff/recovery.rs"]
mod recovery;
#[path = "articulated_handoff/run.rs"]
mod run;
#[path = "articulated_handoff/wire.rs"]
mod wire;
#[path = "articulated_handoff/worker.rs"]
mod worker;
use fixture::{CUT, Observation, SPAN, Tree};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
type Hash = [u8; 32];
#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    mode: Mode,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum, Serialize, Deserialize)]
enum Stage {
    Frozen,
    Prepared,
    Retired,
    DecisionStaged,
    DecisionLinked,
    Decided,
    Activated,
    IntentStaged,
    Intent,
    Applied,
    ReceiptStaged,
    ReceiptLinked,
    Receipted,
}
#[derive(Subcommand)]
enum Mode {
    /// Run an independent reference, two fenced workers, persistent cut, and fault checks.
    Run { out: PathBuf },
    /// Time separate transfer phases, excluding process start and source warm-up.
    Bench {
        out: PathBuf,
        #[arg(default_value_t = 32)]
        trials: usize,
        /// Create and sync trial records on this local filesystem.
        #[arg(long)]
        storage: Option<PathBuf>,
    },
    /// Resume a local session to its last durable step, resolving a pending input once.
    Resume { dir: PathBuf },
    /// Hard-kill coordinators at protocol, file publication and physics interval boundaries.
    CrashProof { out: PathBuf },
    /// Kill actual source and target processes while their controller survives.
    WorkerProof { out: PathBuf },
    #[command(hide = true)]
    CrashRole { dir: PathBuf, at: Stage },
    #[command(hide = true)]
    Worker { slot: u8, decision: PathBuf },
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
struct Ticket {
    session: durable::Session,
    from: u8,
    to: u8,
    source_epoch: u32,
    target_epoch: u32,
    tick: u16,
    bodies: [u64; 2],
    joint: u64,
    image: Hash,
}
impl Ticket {
    fn new(session: durable::Session, image: &[u8]) -> Self {
        Self {
            session,
            from: 0,
            to: 1,
            source_epoch: 1,
            target_epoch: 2,
            tick: CUT,
            bodies: [0, 1],
            joint: 0,
            image: Sha256::digest(image).into(),
        }
    }
    fn check(self, image: &[u8]) -> bool {
        matches!(image.len(), 369 | 377) && self == Self::new(self.session, image)
    }
    fn retirement(self) -> Result<Hash> {
        let mut h = Sha256::new();
        h.update(b"GZHNG001-retired");
        h.update(wire::encode(&self)?);
        Ok(h.finalize().into())
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
struct Step {
    epoch: u32,
    tick: u16,
    effort: Option<u64>,
}
impl Step {
    fn new(tick: u16, effort: Option<f64>) -> Self {
        Self {
            epoch: 2,
            tick,
            effort: effort.map(f64::to_bits),
        }
    }
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
enum Op {
    Boot,
    Freeze { epoch: u32, tick: u16 },
    Prepare(Ticket),
    Retire(Ticket),
    Activate(Ticket),
    Advance(Step),
    Observe,
    Inspect,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct Request {
    session: durable::Session,
    id: u64,
    op: Op,
    omit_reply: bool,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum Phase {
    Idle,
    Active,
    Frozen,
    Prepared,
    Retired,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum Denial {
    Fenced,
    Conflict,
    Invalid,
    NoDecision,
    Busy,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[allow(
    clippy::large_enum_variant,
    reason = "A single fixed-size two-link readback avoids an allocation on every step and its receipt replay."
)]
enum Reply {
    Status {
        phase: Phase,
        epoch: u32,
        tick: u16,
    },
    Frozen(Ticket),
    Prepared(Ticket),
    Retired(Hash),
    Activated(Ticket),
    Observed {
        epoch: u32,
        tick: u16,
        local_time_ns: u64,
        digest: Hash,
        state: Observation,
    },
    Denied(Denial),
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
struct Decision {
    ticket: Ticket,
    retired: Hash,
}
fn load_decision(path: &Path) -> Result<(Decision, Vec<u8>)> {
    let mut f = fs::File::open(path)?;
    let (d, img): (Decision, _) = wire::read(&mut f)?;
    if d.ticket.session != durable::session(path)?
        || !d.ticket.check(&img)
        || d.retired != d.ticket.retirement()?
        || f.read(&mut [0; 1])? != 0
    {
        return Err("invalid durable hinge decision".into());
    }
    Ok((d, img))
}
fn publish(path: &Path, record: Decision, img: &[u8]) -> Result<()> {
    let retired = load_decision(&path.with_extension("retired"))?;
    if retired.0 != record || retired.1 != img {
        return Err("commit lacks exact source retirement".into());
    }
    durable::write_record(path, record, img)
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct Answer {
    id: u64,
    reply: Reply,
}
type Incoming = std::result::Result<(Answer, Vec<u8>), String>;
struct Client {
    session: durable::Session,
    child: Child,
    tx: ChildStdin,
    rx: Receiver<Incoming>,
    next: u64,
}
impl Client {
    fn start(slot: u8, decision: &Path) -> Result<Self> {
        let session = durable::session(decision)?;
        let mut child = Command::new(std::env::current_exe()?)
            .arg("worker")
            .arg(slot.to_string())
            .arg(decision)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        let tx = child.stdin.take().unwrap();
        let mut reader = BufReader::new(child.stdout.take().unwrap());
        let (sender, rx) = mpsc::sync_channel(1);
        std::thread::spawn(move || {
            loop {
                let read = wire::read(&mut reader).map_err(|e| e.to_string());
                let done = read.is_err();
                if sender.send(read).is_err() || done {
                    break;
                }
            }
        });
        Ok(Self {
            session,
            child,
            tx,
            rx,
            next: 0,
        })
    }
    fn send(&mut self, op: Op, img: &[u8], omit_reply: bool) -> Result<u64> {
        self.next = self
            .next
            .checked_add(1)
            .ok_or("hinge request sequence exhausted")?;
        wire::frame(
            &mut self.tx,
            &Request {
                session: self.session,
                id: self.next,
                op,
                omit_reply,
            },
            img,
        )?;
        Ok(self.next)
    }
    fn receive_on(
        rx: &Receiver<Incoming>,
        id: u64,
        timeout: Duration,
    ) -> Result<Option<(Reply, Vec<u8>)>> {
        let until = Instant::now() + timeout;
        loop {
            let Some(rem) = until.checked_duration_since(Instant::now()) else {
                return Ok(None);
            };
            match rx.recv_timeout(rem) {
                Ok(Ok((answer, img))) if answer.id == id => return Ok(Some((answer.reply, img))),
                Ok(Ok((answer, _))) if answer.id < id => continue,
                Ok(Ok(_)) => return Err("future hinge reply ID".into()),
                Ok(Err(e)) => return Err(e.into()),
                Err(RecvTimeoutError::Disconnected) => {
                    return Err("hinge worker disconnected".into());
                }
                Err(RecvTimeoutError::Timeout) => return Ok(None),
            }
        }
    }
    fn call(&mut self, op: Op, img: &[u8]) -> Result<(Reply, Vec<u8>)> {
        let id = self.send(op, img, false)?;
        Self::receive_on(&self.rx, id, Duration::from_secs(5))?
            .ok_or_else(|| "unconfirmed hinge request timed out".into())
    }
    fn omit(&mut self, op: Op, img: &[u8]) -> Result<()> {
        let id = self.send(op, img, true)?;
        if Self::receive_on(&self.rx, id, Duration::from_millis(20))?.is_some() {
            return Err("expected a missing worker acknowledgement".into());
        }
        Ok(())
    }
    fn kill_wait(&mut self) -> Result<u32> {
        let id = self.child.id();
        self.child.kill()?;
        let status = self.child.wait()?;
        #[cfg(unix)]
        {
            use std::os::unix::process::ExitStatusExt;
            if status.signal() != Some(9) {
                return Err(
                    format!("worker {id} exited without observed SIGKILL: {status}").into(),
                );
            }
        }
        #[cfg(not(unix))]
        if status.success() {
            return Err("worker exited normally before requested death".into());
        }
        Ok(id)
    }
}
impl Drop for Client {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}
fn main() -> Result<()> {
    if !cfg!(unix) {
        return Err("the experimental local hinge endpoint requires a Unix filesystem with exclusive locks and hard links".into());
    }
    match Args::parse().mode {
        Mode::Run { out } => run::run(&out),
        Mode::Resume { dir } => recovery::resume(&dir),
        Mode::CrashRole { dir, at } => recovery::role(&dir, at),
        Mode::CrashProof { out } => recovery::proof(&out),
        Mode::WorkerProof { out } => participant_faults::qualify(&out),
        Mode::Bench {
            out,
            trials,
            storage,
        } => run::bench(&out, trials, storage.as_deref()),
        Mode::Worker { slot, decision } => worker::serve(slot, &decision),
    }
}

#[cfg(test)]
#[path = "articulated_handoff/validation.rs"]
mod validation;
