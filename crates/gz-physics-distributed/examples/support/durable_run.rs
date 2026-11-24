// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Crash and restart a recorded coordinator at an ambiguous active-contact handoff.
#[path = "replay.rs"]
mod replay;
#[path = "warm_process.rs"]
mod warm_process;
use gz_physics_distributed::{
    coordinator::{Coordinator, Error as BarrierError, Frame, RecoveryPlan},
    physics::{Body, fixture},
    protocol::Observation,
    transport::{TransportError, WorkerProcess},
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{self, Write},
    net::{SocketAddr, TcpStream},
    path::Path,
    process::{Command, Stdio},
    thread,
    time::{Duration, Instant},
};
use warm_process::{IMPULSE, Process};

pub type Result<T> = std::result::Result<T, Box<dyn Error>>;
const END_TICK: u64 = 200;
const JOURNAL: &str = "published.gzrec";
const WITNESS: &str = "original.json";

#[derive(Serialize, Deserialize)]
struct Instance {
    pid: u32,
    endpoint: String,
    session: String,
}
impl Instance {
    fn get(worker: &WorkerProcess) -> Self {
        Self {
            pid: worker.pid(),
            endpoint: worker.endpoint().to_owned(),
            session: worker.session_id().to_owned(),
        }
    }
}

#[derive(Serialize, Deserialize)]
struct Original {
    coordinator: u32,
    workers: [Instance; 2],
    // Independent evidence from the original process. Never used to hydrate workers.
    samples: Vec<[Observation; 2]>,
}

fn publish(path: &Path, v: &impl Serialize) -> Result<()> {
    let mut f = tempfile::NamedTempFile::new_in(path.parent().ok_or("missing directory")?)?;
    serde_json::to_writer(f.as_file_mut(), v)?;
    f.as_file_mut().write_all(b"\n")?;
    f.as_file().sync_all()?;
    f.persist_noclobber(path)?;
    Ok(())
}
fn seed() -> Vec<Body> {
    fixture().iter().map(Body::promoted).collect()
}

pub fn original(worker: &Path, dir: &Path) -> Result<()> {
    let run = u64::from(std::process::id());
    let peers = [
        Process::spawn(worker, run, 0, &seed(), false)?,
        Process::spawn(worker, run, 1, &[], true)?,
    ];
    let instances = peers.each_ref().map(|p| Instance::get(&p.worker));
    let mut group = Coordinator::new_recorded(run, peers, 512)?;
    let mut samples = vec![group.frame().workers.clone()];
    loop {
        let frame = group.advance()?;
        if frame.workers[0]
            .snapshot
            .impulses
            .iter()
            .any(|p| p.normal_impulse > 1e-3)
        {
            samples.push(frame.workers.clone());
            break;
        }
        if frame.tick % 4 == 0 {
            samples.push(frame.workers.clone());
        }
        if frame.tick >= END_TICK {
            return Err("no active contact in qualification interval".into());
        }
    }
    let published = group.frame().clone();
    group.queue_impulse(1, IMPULSE)?;
    assert!(matches!(
        group.transfer_group(1),
        Err(BarrierError::Transport(TransportError::Unavailable(_)))
    ));
    assert_eq!(group.frame(), &published);
    assert!(matches!(group.advance(), Err(BarrierError::Pending)));
    group.save_recovery(dir.join(JOURNAL))?;
    publish(
        &dir.join(WITNESS),
        &Original {
            coordinator: std::process::id(),
            workers: instances,
            samples,
        },
    )?;
    println!(
        "Original coordinator {} saved published tick {} after losing the activation reply; the queued impulse is {:?} N s.",
        run, published.tick, IMPULSE
    );
    io::stdout().flush()?;
    // The supervisor holds this pipe until it kills the coordinator Child.
    // A supervisor which itself exits releases the waiting process.
    io::copy(&mut io::stdin().lock(), &mut io::sink())?;
    Ok(())
}

fn frame(workers: [Observation; 2]) -> Frame {
    Frame {
        tick: workers[0].snapshot.tick,
        workers,
    }
}
fn total_momentum(frame: &Frame) -> f64 {
    frame
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.bodies)
        .map(|b| b.mass * b.linear[0])
        .sum()
}

pub fn replacement(worker: &Path, dir: &Path) -> Result<()> {
    let evidence: Original = serde_json::from_reader(File::open(dir.join(WITNESS))?)?;
    let original = evidence
        .samples
        .last()
        .ok_or("empty original transcript")?
        .clone();
    let mut instances = Vec::new();
    let mut recovered = RecoveryPlan::open(dir.join(JOURNAL))?.resume(|run, id, bodies| {
        let w = WorkerProcess::spawn(worker, run, id, bodies)?;
        instances.push(Instance::get(&w));
        Ok(w)
    })?;
    assert_eq!(recovered.frame().workers, original);
    assert_ne!(std::process::id(), evidence.coordinator);
    for new in &instances {
        assert!(
            evidence
                .workers
                .iter()
                .all(|old| old.session != new.session)
        );
    }
    let run = u64::from(std::process::id());
    let mut control = Coordinator::new(
        run,
        [
            WorkerProcess::spawn(worker, run, 0, &seed())?,
            WorkerProcess::spawn(worker, run, 1, &[])?,
        ],
        512,
    )?;
    while control.frame().tick < recovered.frame().tick {
        control.advance()?;
    }
    let barrier = recovered.frame().tick;
    let before = total_momentum(control.frame());
    control.queue_impulse(1, IMPULSE)?;
    assert_eq!(control.frame(), recovered.frame());
    let mut frames: Vec<_> = evidence.samples.into_iter().map(frame).collect();
    let next = recovered.transfer_group(1)?;
    assert_eq!(next, control.transfer_group(1)?);
    assert_eq!(next.tick, barrier);
    assert_eq!(next.body(1).unwrap().0, 1);
    frames.push(next.clone());
    let mut digest = Sha256::new();
    let mut max_residual = 0.0_f64;
    let mut contact_impulse = 0.;
    for tick in barrier + 1..=END_TICK {
        let sample = recovered.advance()?;
        assert_eq!(sample, control.advance()?, "post-restart frame {tick}");
        bincode::serialize_into(&mut digest, &sample.workers)?;
        max_residual = max_residual.max((total_momentum(sample) - before - IMPULSE[0]).abs());
        contact_impulse += sample.workers[1]
            .snapshot
            .impulses
            .iter()
            .map(|p| p.normal_impulse)
            .sum::<f64>();
        if tick % 4 == 0 || tick == barrier + 1 {
            frames.push(sample.clone());
        }
    }
    assert!(max_residual < 1e-10 && contact_impulse > 0.2);
    replay::write(&dir.join("replay"), &frames)?;
    let proof = format!("{:x}", digest.finalize());
    println!(
        "Restored tick {barrier} in coordinator {}. {} subsequent complete frames matched the uninterrupted coordinator bitwise. Once-applied x impulse residual {max_residual:.3e} N s; post-restart normal contact impulse {contact_impulse:.6} N s. Transcript SHA-256 {proof}.",
        std::process::id(),
        END_TICK - barrier
    );
    publish(
        &dir.join("verification.json"),
        &serde_json::json!({
            "coordinator_pid":std::process::id(),"original_coordinator_pid":evidence.coordinator,
            "original_workers":evidence.workers,"replacement_workers":instances,"recovered_tick":barrier,
            "post_recovery_ticks":END_TICK-barrier,"post_recovery_frame_sha256":proof,
            "peak_momentum_residual_ns":max_residual,"post_recovery_normal_impulse_ns":contact_impulse,
            "replay_frames":frames.len()
        }),
    )?;
    for mut w in recovered
        .into_peers()
        .into_iter()
        .chain(control.into_peers())
    {
        w.shutdown()?;
    }
    Ok(())
}

/// Own both coordinator processes. Kill only the original Child handle. Check
/// that the associated worker endpoints have closed before starting recovery.
pub fn supervise(mut stage: Command, mut resume: Command, dir: &Path) -> Result<()> {
    fs::create_dir_all(dir)?;
    for name in [WITNESS, JOURNAL, "verification.json", "replay"] {
        if dir.join(name).exists() {
            return Err(format!("output exists: {}", dir.join(name).display()).into());
        }
    }
    let mut child = stage.stdin(Stdio::piped()).spawn()?;
    let prepared = (|| -> Result<Original> {
        let until = Instant::now() + Duration::from_secs(30);
        while !dir.join(WITNESS).exists() {
            if let Some(status) = child.try_wait()? {
                return Err(format!("original exited before checkpoint: {status}").into());
            }
            if Instant::now() >= until {
                return Err("original did not publish checkpoint before deadline".into());
            }
            thread::sleep(Duration::from_millis(20));
        }
        Ok(serde_json::from_reader(File::open(dir.join(WITNESS))?)?)
    })();
    let killed = child.kill();
    let status = child.wait()?;
    let witness = prepared?;
    killed?;
    assert!(!status.success());
    #[cfg(unix)]
    {
        use std::os::unix::process::ExitStatusExt;
        assert_eq!(status.signal(), Some(9));
    }
    let until = Instant::now() + Duration::from_secs(5);
    for w in &witness.workers {
        let addr: SocketAddr = w
            .endpoint
            .strip_prefix("tcp/")
            .ok_or("unexpected worker endpoint")?
            .parse()?;
        loop {
            match TcpStream::connect_timeout(&addr, Duration::from_millis(100)) {
                Err(e) if e.kind() == io::ErrorKind::ConnectionRefused => break,
                // A reset/timeout can arrive while the listening child is exiting. Only
                // a refused connection establishes that this endpoint is closed.
                Err(e) if !matches!(e.kind(), io::ErrorKind::ConnectionReset |
                    io::ErrorKind::ConnectionAborted | io::ErrorKind::TimedOut | io::ErrorKind::Interrupted
                ) => return Err(e.into()),
                _ if Instant::now() >= until => {
                    return Err("old worker endpoint did not close after coordinator death".into());
                }
                _ => thread::sleep(Duration::from_millis(10)),
            }
        }
    }
    println!(
        "Original coordinator {} ended with {status}. Both old worker endpoints closed before replacement.",
        witness.coordinator
    );
    let status = resume.status()?;
    if !status.success() {
        return Err(format!("replacement failed: {status}").into());
    }
    Ok(())
}
