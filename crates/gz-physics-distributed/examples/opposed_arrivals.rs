// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Simultaneous cross-worker arrivals at a loaded native contact pair.
#[path = "support/replay.rs"]
mod replay;
use clap::Parser;
use gz_physics_distributed::{
    coordinator::{BodyRoute, Coordinator},
    physics::{Body, Island, Snapshot, fixture},
    transport::WorkerProcess,
};
use std::{error::Error, io, path::PathBuf};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    capture: Option<PathBuf>,
    #[arg(long)]
    worker: Option<PathBuf>,
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let path = match args.worker {
        Some(path) => path,
        None => std::env::current_exe()?
            .ancestors()
            .nth(2)
            .ok_or_else(|| io::Error::other("pass --worker"))?
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX)),
    };
    let run = 20261119;
    let mut bodies = Vec::new();
    for (id, x, v) in [
        (1, -0.36, 0.9),
        (2, 0.36, -0.9),
        (3, -7.2, 5.),
        (4, 7.2, -5.),
    ] {
        bodies.push(Body {
            id,
            position: [x, 0., 0.],
            linear: [v, 0., 0.],
            ..fixture()[0].promoted()
        });
    }
    let mut c = Coordinator::new(
        run,
        [
            WorkerProcess::spawn(&path, run, 0, &bodies[..2])?,
            WorkerProcess::spawn(&path, run, 1, &bodies[2..])?,
        ],
        2048,
    )?;
    let mut control = Island::restore(&Snapshot {
        bodies,
        ..Default::default()
    })?;
    let mut frames = vec![c.frame().clone()];
    let mut routed = None;
    let mut first_three = None;
    let mut peak = 0_f64;
    for _ in 0..190 {
        let moves = c.contact_routes()?;
        if !moves.is_empty() {
            assert!(routed.is_none());
            assert_eq!(
                moves,
                [
                    BodyRoute {
                        body: 3,
                        destination: 0
                    },
                    BodyRoute {
                        body: 4,
                        destination: 0
                    }
                ]
            );
            routed = Some(c.frame().tick);
            c.colocate_contacts()?;
            frames.push(c.frame().clone());
        }
        for (id, impulse) in [(1, 0.014), (2, -0.014)] {
            c.queue_impulse(id, [impulse, 0., 0.])?;
            control.queue_impulse(id, [impulse, 0., 0.])?;
        }
        let frame = c.advance_overlapped()?;
        control.step()?;
        let reference = control.snapshot();
        for body in &reference.bodies {
            assert_eq!(frame.body(body.id).unwrap().1, body);
        }
        let mut pairs: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.clone())
            .collect();
        pairs.sort_by_key(|p| (p.a, p.b));
        assert_eq!(pairs, reference.impulses);
        if pairs.len() == 3 {
            first_three.get_or_insert(frame.tick);
        }
        for p in &pairs {
            peak = peak.max(p.normal_impulse);
        }
        if frame.tick % 3 == 0 || frame.tick == 190 {
            frames.push(frame.clone());
        }
    }
    let tick = routed.ok_or("cold boxes never routed")?;
    let contact = first_three.ok_or("three simultaneous pairs never observed")?;
    if let Some(path) = args.capture {
        replay::write(&path, &frames)?;
    }
    println!(
        "Both boxes routed at tick {tick}. Three simultaneous pairs at tick {contact}. Peak normal impulse {peak:.9} N s. All four bodies and impulses agree bit for bit with an all-seeded uninterrupted native solver for 190 ticks."
    );
    Ok(())
}
