// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::replay;
use gz_physics_distributed::{
    coordinator::{Coordinator, Frame},
    physics::{Body, BodyWrench, Island, Snapshot, fixture},
    transport::WorkerProcess,
};
use std::{error::Error, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn compare(frame: &Frame, reference: &Snapshot) {
    assert_eq!(frame.tick, reference.tick);
    for b in &reference.bodies {
        assert_eq!(
            frame.body(b.id).unwrap().1,
            b,
            "body {} at {}",
            b.id,
            frame.tick
        );
    }
    let mut pairs: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| w.snapshot.impulses.clone())
        .collect();
    pairs.sort_by_key(|p| (p.a, p.b));
    assert_eq!(pairs, reference.impulses);
}
pub fn run(worker: &Path, capture: Option<&Path>) -> Result<()> {
    let run = 0x425249444745;
    let mut bodies: Vec<_> = [(-0.72, 1.4), (0., 0.), (0.72, -1.4)]
        .into_iter()
        .enumerate()
        .map(|(i, (x, v))| Body {
            id: i as u64 + 1,
            position: [x - 4.5, 0., 0.],
            linear: [v, 0., 0.],
            ..fixture()[0].promoted()
        })
        .collect();
    bodies.push(Body {
        id: 4,
        position: [0., 0., replay::PHYSICAL_GAP],
        linear: [0.; 3],
        ..fixture()[0].clone()
    });
    let mut c = Coordinator::new_recorded(
        run,
        [
            WorkerProcess::spawn(worker, run, 0, &bodies[..3])?,
            WorkerProcess::spawn(worker, run, 1, &bodies[3..])?,
        ],
        1500,
    )?;
    let mut native = Island::restore(&Snapshot {
        bodies,
        ..Default::default()
    })?;
    let mut samples = vec![c.frame().clone()];
    let mut split = None;
    let mut loaded = false;
    for tick in 1..=125 {
        for (id, j) in [(1, 0.02), (2, -0.02)] {
            c.queue_impulse(id, [j, 0., 0.])?;
            native.queue_impulse(id, [j, 0., 0.])?;
        }
        let mut wrench = Vec::new();
        if tick == 33 {
            assert!(loaded, "original triple must carry two active contacts");
            c.queue_impulse(3, [24., 0., 0.])?;
            native.queue_impulse(3, [24., 0., 0.])?;
            wrench.push(BodyWrench {
                id: 3,
                force: [0., 1., 0.],
                torque: [0., 0.31, 0.52],
            });
        }
        if tick == 77 {
            c.queue_impulse(3, [-24., 0., 0.])?;
            native.queue_impulse(3, [-24., 0., 0.])?;
        }
        if tick > 32 && split.is_none() {
            let prior = c.frame().clone();
            let new = c.split_component(2)?;
            assert_eq!(new.workers[0].snapshot, prior.workers[0].snapshot);
            if new.workers[0].components.len() == 2 {
                assert_eq!(new.workers[0].components[0].ids, [1, 2]);
                assert_eq!(new.workers[0].components[1].ids, [3]);
                assert!(new.workers[0].components.iter().all(|g| g.contact_history));
                let pair = new.workers[0]
                    .snapshot
                    .impulses
                    .iter()
                    .find(|p| (p.a, p.b) == (1, 2))
                    .ok_or("resident pair lost contact")?;
                println!(
                    "split at tick {}: resident normal impulse {:.9} N s; detached center x {:.6} m",
                    new.tick,
                    pair.normal_impulse,
                    new.body(3).unwrap().1.position[0]
                );
                split = Some(new.tick);
                samples.push(new.clone());
                c.transfer_component(3)
                    .map_err(|e| format!("component transfer at tick {tick}: {e}"))?;
                c.compact()
                    .map_err(|e| format!("compaction at tick {tick}: {e}"))?;
                c.recover(1)
                    .map_err(|e| format!("replacement at tick {tick}: {e}"))?;
                assert_eq!(c.frame().workers[1].generations[&3], 1);
                samples.push(c.frame().clone());
            }
        }
        if tick == 111 {
            c.transfer_component(3)
                .map_err(|e| format!("component transfer at tick {tick}: {e}"))?;
        }
        c.advance_routed_with_wrenches(&wrench)
            .map_err(|e| format!("advance to tick {tick}: {e}"))?;
        native.step_with_wrenches(&wrench)?;
        let reference = native.snapshot();
        compare(c.frame(), &reference);
        loaded |= reference.contacts == 2;
        if tick % 2 == 0 || tick == 125 {
            samples.push(c.frame().clone());
        }
    }
    assert!(split.is_some());
    assert_eq!(c.frame().workers[0].generations[&3], 2);
    if let Some(path) = capture {
        replay::write(path, &samples)?;
    }
    println!(
        "125 exact monolithic native frames including 2 complete-solver handoffs, a verified native compaction, worker replacement, the resident contact and a spinning detached singleton."
    );
    for mut peer in c.into_peers() {
        peer.shutdown()?;
    }
    Ok(())
}
