// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "replay.rs"]
mod replay;
use gz_physics_distributed::{
    coordinator::{BodyRoute, Coordinator, Error, Peer},
    physics::{Body, Island, Snapshot, fixture},
    protocol::{Command, Reply, Request},
    transport::{TransportError, WorkerProcess},
};
use std::{error::Error as StdError, io, path::Path, time::Duration};

struct Endpoint {
    worker: WorkerProcess,
    suppress: bool,
}
impl Peer for Endpoint {
    fn exchange(&mut self, r: &Request) -> Result<Reply, TransportError> {
        if self.suppress && matches!(r.command, Command::Activate { .. }) {
            self.suppress = false;
            return self
                .worker
                .request_dropping_reply(r, Duration::from_millis(200));
        }
        self.worker.request(r)
    }
    fn restart(&mut self) -> io::Result<()> {
        self.worker.restart()
    }
}

fn box_at(id: u64, x: f64, z: f64, v: f64) -> Body {
    Body {
        id,
        position: [x, 0., z],
        linear: [v, 0., 0.],
        ..fixture()[0].promoted()
    }
}

pub fn run(worker: &Path, capture: Option<&Path>, loss: bool) -> Result<u64, Box<dyn StdError>> {
    let seed = vec![
        box_at(1, -0.36, 0., 0.9),
        box_at(2, 0.36, 0., -0.9),
        box_at(3, -7.2, 0., 5.),
        box_at(4, -1.2, replay::PHYSICAL_GAP, 0.4),
        box_at(5, 1.2, -replay::PHYSICAL_GAP, -0.3),
    ];
    let run = u64::from(std::process::id());
    let start = |id, indices: &[usize]| -> io::Result<_> {
        let bodies: Vec<_> = indices.iter().map(|&i| seed[i].clone()).collect();
        Ok(Endpoint {
            worker: WorkerProcess::spawn(worker, run, id, &bodies)?,
            suppress: id == 0 && loss,
        })
    };
    let mut world = Coordinator::new(run, [start(0, &[0, 1, 3])?, start(1, &[2, 4])?], 512)?;
    let mut native = Island::restore(&Snapshot {
        bodies: seed,
        ..Snapshot::default()
    })?;
    let mut frames = vec![world.frame().clone()];
    let mut routed = None;
    let mut first_impact = None;
    let mut contact_impulse = 0.;
    let mut prior_contact = 0.;
    for _ in 0..220 {
        let routes = world.contact_routes()?;
        if !routes.is_empty() {
            assert_eq!(
                routes,
                [BodyRoute {
                    body: 3,
                    destination: 0
                }]
            );
            assert!(routed.is_none() && prior_contact > 0.);
            let before = world.frame().clone();
            if loss {
                assert!(matches!(
                    world.advance_routed(),
                    Err(Error::Transport(TransportError::Unavailable(_)))
                ));
                assert_eq!(world.frame(), &before);
            }
            let colocated = world.colocate_contacts()?;
            assert_eq!(colocated.tick, before.tick);
            for id in 1..=5 {
                assert_eq!(colocated.body(id).unwrap().1, before.body(id).unwrap().1);
            }
            assert_eq!(
                colocated.workers[0].snapshot.impulses,
                before.workers[0].snapshot.impulses
            );
            routed = Some(colocated.tick);
            if frames.last() != Some(&before) {
                frames.push(before);
            }
            frames.push(colocated.clone());
        }
        let f = world.advance_routed()?;
        native.step()?;
        let sample = native.snapshot();
        for b in &sample.bodies {
            let (w, actual) = f.body(b.id).unwrap();
            assert_eq!(w, u8::from(b.id == 5 || (b.id == 3 && routed.is_none())));
            assert_eq!(actual, b, "body {} at tick {}", b.id, f.tick);
        }
        let impulses = &f.workers[0].snapshot.impulses;
        assert_eq!(
            impulses.len(),
            sample.impulses.len(),
            "contact count at {}",
            f.tick
        );
        for (a, b) in impulses.iter().zip(&sample.impulses) {
            assert_eq!((a.a, a.b), (b.a, b.b), "contact IDs at {}", f.tick);
            assert_eq!(a, b, "contact impulse at tick {}", f.tick);
            if a.b == 3 && a.normal_impulse > 0. {
                first_impact.get_or_insert(f.tick);
                contact_impulse += a.normal_impulse;
            } else if routed.is_none() {
                prior_contact += a.normal_impulse;
            }
        }
        if f.tick % 4 == 0 {
            frames.push(f.clone());
        }
    }
    let tick = routed.ok_or("route never committed")?;
    let impact = first_impact.ok_or("new contact did not solve")?;
    assert!(impact > tick && contact_impulse > 1.);
    println!(
        "Routed incoming body 3 at tick {tick}; first subsequent impact at tick {impact}. Prior pair impulse {prior_contact:.12} N s; contact with incoming box supplied {contact_impulse:.12} N s. All 220 body and contact samples matched the uninterrupted native solver bitwise."
    );

    if let Some(path) = capture {
        replay::write(path, &frames)?;
        println!(
            "Wrote {} frames to {}. Vertical bystander spacing is compressed.",
            frames.len(),
            path.display()
        );
    }
    for mut e in world.into_peers() {
        e.worker.shutdown()?;
    }
    Ok(tick)
}
