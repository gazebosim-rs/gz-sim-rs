// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify a crossing and collision on two independent physics worker processes.
#[path = "replay.rs"]
mod replay;
use gz_physics_distributed::{
    coordinator::{Coordinator, Error as BarrierError, Frame, Peer},
    physics::{Body, Island, STEP, Snapshot, Tier, fixture},
    protocol::{Command, Reply, Request},
    transport::{TransportError, WorkerProcess},
};
use std::{
    error::Error,
    io,
    path::Path,
    time::{Duration, Instant},
};

// Inject failures at explicit protocol boundaries. Both normal and injected
// exchanges use the same worker executable and TCP stack.
struct FaultPeer {
    process: WorkerProcess,
    lose_release: bool,
    kill_at: Option<u64>,
}
impl Peer for FaultPeer {
    fn exchange(&mut self, req: &Request) -> Result<Reply, TransportError> {
        if self.lose_release && matches!(req.command, Command::Release { .. }) {
            self.lose_release = false;
            return self
                .process
                .request_dropping_reply(req, Duration::from_millis(150));
        }
        if let Command::Step { tick } = req.command
            && self.kill_at == Some(tick)
        {
            self.kill_at = None;
            self.process.kill()?;
        }
        self.process.request(req)
    }
    fn restart(&mut self) -> io::Result<()> {
        self.process.restart()
    }
}

#[derive(Default, Debug)]
struct Errors {
    position: f64,
    orientation: f64,
    linear: f64,
    angular: f64,
    impulse: f64,
}

fn distance<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    a.into_iter()
        .zip(b)
        .fold(0.0, |norm, (x, y)| norm.hypot(x - y))
}

impl Errors {
    fn observe(&mut self, f: &Frame, oracle: &Island) {
        let reference = oracle.snapshot();
        assert_eq!(f.tick, reference.tick);
        for b in reference.bodies {
            let (_, a) = f.body(b.id).unwrap();
            self.position = self.position.max(distance(a.position, b.position));
            self.linear = self.linear.max(distance(a.linear, b.linear));
            self.angular = self.angular.max(distance(a.angular, b.angular));
            // q and -q represent the same rotation.
            self.orientation = self.orientation.max(
                distance(a.orientation, b.orientation)
                    .min(distance(a.orientation, b.orientation.map(|x| -x))),
            );
        }
        let impulses: Vec<_> = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .collect();
        assert_eq!(
            f.workers.iter().map(|w| w.snapshot.contacts).sum::<usize>(),
            reference.contacts
        );
        assert_eq!(impulses.len(), reference.impulses.len());
        for (a, b) in impulses.into_iter().zip(reference.impulses) {
            assert_eq!((a.a, a.b), (b.a, b.b));
            self.impulse = self
                .impulse
                .max((a.normal_impulse - b.normal_impulse).abs());
        }
        assert!(
            self.position < 1e-8
                && self.orientation < 1e-8
                && self.linear < 1e-8
                && self.angular < 1e-8
                && self.impulse < 1e-8,
            "tick {}: {self:?}",
            f.tick
        );
    }
}

fn required_outage(result: Result<&Frame, BarrierError>) -> Result<(), Box<dyn Error>> {
    match result {
        Err(BarrierError::Transport(TransportError::Unavailable(_))) => Ok(()),
        Err(e) => Err(e.into()),
        Ok(_) => Err(io::Error::other("fault injection failed to interrupt publication").into()),
    }
}

pub fn run(worker: &Path, capture: Option<&Path>) -> Result<(), Box<dyn Error>> {
    let run = u64::from(std::process::id());
    let initial = fixture();
    let make = |id: usize| -> Result<FaultPeer, Box<dyn Error>> {
        let process = WorkerProcess::spawn(worker, run, id as u8, &initial[id..id + 1])?;
        println!("worker {id}: PID {}, {}", process.pid(), process.endpoint());
        Ok(FaultPeer {
            process,
            lose_release: id == 0,
            kill_at: (id == 1).then_some(145),
        })
    };
    let peers = [make(0)?, make(1)?];
    let initial_owner_pid = peers[1].process.pid();
    let mut world = Coordinator::new(run, peers, 256)?;
    let reference = |bodies| {
        Island::restore(&Snapshot {
            bodies,
            ..Default::default()
        })
    };
    let mut switched = reference(initial.clone())?;
    let mut rigid = reference(initial.iter().map(Body::promoted).collect())?;
    let mut errors = [Errors::default(), Errors::default()];
    let mut frames = vec![world.frame().clone()];
    let mut handoff = None;
    let mut first_impact = None;
    let mut impulse = 0.;
    let mut recovery = None;
    for tick in 0..200 {
        let (owner, body) = world.frame().body(1).unwrap();
        if owner == 0
            && body.position[0] + 0.5 * body.size[0] + STEP.as_secs_f64() * body.linear[0] >= 0.
        {
            let old = world.frame().clone();
            required_outage(world.transfer(1, true))?;
            assert_eq!(world.frame(), &old);
            assert!(matches!(world.advance(), Err(BarrierError::Pending)));
            let f = world.transfer(1, true)?;
            let (owner, promoted) = f.body(1).unwrap();
            assert_eq!(
                (owner, promoted.tier, f.workers[1].generations[&1]),
                (1, Tier::Dynamic, 1)
            );
            assert_eq!(promoted, &old.body(1).unwrap().1.promoted());
            if frames.last() != Some(&old) {
                frames.push(old);
            }
            frames.push(f.clone());
            let local = switched.export(1)?.promoted();
            switched.remove(1)?;
            switched.import(local)?;
            println!(
                "tick {tick}: boundary handoff retried after a lost release acknowledgment, x={:.6} m",
                promoted.position[0]
            );
            handoff = Some(tick);
        }
        if tick == 145 {
            assert!(first_impact.is_some() && world.frame().workers[1].snapshot.contact_history);
            let before = world.frame().clone();
            required_outage(world.advance())?;
            assert_eq!(world.frame(), &before);
            let start = Instant::now();
            world.recover(1)?;
            recovery = Some(start.elapsed());
        }
        let f = world.advance()?;
        switched.step()?;
        rigid.step()?;
        errors[0].observe(f, &switched);
        errors[1].observe(f, &rigid);
        for pair in f.workers.iter().flat_map(|w| &w.snapshot.impulses) {
            assert_eq!((pair.a, pair.b), (1, 2));
            if pair.normal_impulse > 1e-5 {
                first_impact.get_or_insert(f.tick);
            }
            impulse += pair.normal_impulse;
        }
        if f.tick % 4 == 0 || Some(f.tick) == first_impact {
            frames.push(f.clone());
        }
    }
    let (owner, a) = world.frame().body(1).unwrap();
    let (_, b) = world.frame().body(2).unwrap();
    assert_eq!(owner, 1);
    let delta_p = b.mass * (b.linear[0] - initial[1].linear[0]);
    let residual = a.mass * (a.linear[0] - initial[0].linear[0]) + delta_p;
    assert!(impulse > 1.0 && (impulse - delta_p).abs() < 1e-8 && residual.abs() < 1e-8);
    assert!(handoff.unwrap() < first_impact.unwrap());
    println!(
        "impact tick {}, cumulative normal impulse {impulse:.12} N s, receiver delta-p {delta_p:.12} kg m/s, momentum residual {residual:.3e} kg m/s",
        first_impact.unwrap()
    );
    println!("final vx {:.9} and {:.9} m/s", a.linear[0], b.linear[0]);
    println!(
        "peak errors vs local promotion {:?}\npeak errors vs continuously dynamic {:?}",
        errors[0], errors[1]
    );
    let peers = world.into_peers();
    assert_ne!(initial_owner_pid, peers[1].process.pid());
    println!(
        "recovered collision owner {initial_owner_pid} -> {} at tick 145 in {:.1} ms",
        peers[1].process.pid(),
        recovery.unwrap().as_secs_f64() * 1000.
    );
    for (i, mut peer) in peers.into_iter().enumerate() {
        peer.process.shutdown()?;
        let traffic = peer.process.traffic();
        println!(
            "worker {i}: {} requests, {} replies, {} sent / {} received application bytes",
            traffic.requests, traffic.replies, traffic.request_bytes, traffic.reply_bytes
        );
    }
    if let Some(path) = capture {
        replay::write(path, &frames)?;
        println!(
            "saved {} measured frames to {}",
            frames.len(),
            path.display()
        );
    }
    Ok(())
}
