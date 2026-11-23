// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Monitor the exported native contact group and selectively suppress an activation reply.
use gz_physics_distributed::{
    coordinator::Peer,
    physics::Body,
    protocol::{Command, Outcome, Reply, Request},
    transport::{TransportError, WorkerProcess},
};
use std::{io, path::Path, time::Duration};

pub const IMPULSE: [f64; 3] = [1.2, 0., 0.];

pub struct Process {
    pub worker: WorkerProcess,
    pub lose_activation: bool,
    pub native_bytes: Option<usize>,
}
impl Peer for Process {
    fn exchange(&mut self, request: &Request) -> Result<Reply, TransportError> {
        if self.lose_activation && matches!(request.command, Command::ActivateWarm { .. }) {
            self.lose_activation = false;
            return self
                .worker
                .request_dropping_reply(request, Duration::from_millis(200));
        }
        let reply = self.worker.request(request)?;
        if let Outcome::WarmExported(handoff) = &reply.outcome {
            let checkpoint = &handoff.checkpoint;
            let contacts = &checkpoint.snapshot.impulses;
            assert_eq!(contacts.len(), checkpoint.snapshot.bodies.len() - 1);
            assert_eq!(checkpoint.snapshot.contacts, contacts.len());
            assert!(contacts.iter().all(|c| c.normal_impulse > 0.));
            assert_eq!(checkpoint.pending.len(), 1);
            assert_eq!(checkpoint.pending[0].id, 1);
            assert_eq!(checkpoint.pending[0].impulse, IMPULSE);
            self.native_bytes = Some(checkpoint.native_bytes());
            let points = checkpoint.warmstart_normal_impulses().collect::<Vec<_>>();
            assert_eq!(checkpoint.manifold_count, contacts.len());
            assert!(
                points.iter().any(|j| *j > 0.) && points.iter().all(|j| j.is_finite() && *j >= 0.)
            );
            println!(
                "source native checkpoint has {} manifolds and {} points, cached normal impulses {points:?} N s",
                checkpoint.manifold_count,
                points.len()
            );
        }
        Ok(reply)
    }
    fn restart(&mut self) -> io::Result<()> {
        self.worker.restart()
    }
}

impl Process {
    pub fn spawn(
        executable: &Path,
        run: u64,
        id: u8,
        initial: &[Body],
        lose_activation: bool,
    ) -> io::Result<Self> {
        let worker = WorkerProcess::spawn(executable, run, id, initial)?;
        println!("worker {id} PID {} at {}", worker.pid(), worker.endpoint());
        Ok(Self {
            worker,
            lose_activation: id == 1 && lose_activation,
            native_bytes: None,
        })
    }
}
