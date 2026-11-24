// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "replay.rs"]
mod replay;
#[path = "warm_process.rs"]
mod warm_process;
use gz_physics_distributed::{
    coordinator::{Coordinator, Error as BarrierError, Frame},
    physics::{Body, Island, Snapshot, fixture},
    transport::TransportError,
};
use std::{error::Error, path::Path};
use warm_process::{IMPULSE, Process};

const STEPS: u64 = 200;

fn compare(frame: &Frame, reference: &Snapshot, owner: usize) {
    // Body and ContactImpulse equality compares f64 bit patterns, including signed zero.
    assert_eq!(frame.tick, reference.tick);
    assert_eq!(
        &frame.workers[owner].snapshot, reference,
        "tick {}",
        frame.tick
    );
    let empty = &frame.workers[1 - owner].snapshot;
    assert!(empty.bodies.is_empty() && empty.contacts == 0 && empty.impulses.is_empty());
    for body in &reference.bodies {
        assert_eq!(frame.workers[owner].generations[&body.id], owner as u64);
    }
}

fn momentum(bodies: &[Body], axis: usize) -> f64 {
    bodies.iter().map(|b| b.mass * b.linear[axis]).sum()
}

/// Returns the tick at which both bodies changed owner, at the same committed poses.
pub fn run(
    worker: &Path,
    capture: Option<&Path>,
    lose_activation: bool,
) -> Result<u64, Box<dyn Error>> {
    let run = u64::from(std::process::id());
    let seed = fixture().iter().map(Body::promoted).collect::<Vec<_>>();
    let mut pair = Coordinator::new(
        run,
        [
            Process::spawn(worker, run, 0, &seed, lose_activation)?,
            Process::spawn(worker, run, 1, &[], lose_activation)?,
        ],
        256,
    )?;
    let initial = Snapshot {
        bodies: seed,
        ..Default::default()
    };
    let mut uninterrupted = Island::restore(&initial)?;
    let mut unforced = Island::restore(&initial)?;
    let mut frames = vec![pair.frame().clone()];
    compare(pair.frame(), &initial, 0);
    let mut handoff = None;
    let mut contact_after = 0.;
    let mut max_residual = 0.0_f64;
    for _ in 0..STEPS {
        let frame = pair.advance()?;
        uninterrupted.step()?;
        unforced.step()?;
        let reference = uninterrupted.snapshot();
        let control = unforced.snapshot();
        let owner = usize::from(handoff.is_some());
        compare(frame, &reference, owner);
        if owner == 1 {
            let normal: f64 = reference.impulses.iter().map(|p| p.normal_impulse).sum();
            contact_after += normal;
            for (axis, input) in IMPULSE.iter().enumerate() {
                let difference =
                    momentum(&reference.bodies, axis) - momentum(&control.bodies, axis);
                max_residual = max_residual.max((difference - input).abs());
            }
        } else {
            assert_eq!(reference, control);
        }
        if frame.tick % 4 == 0 {
            frames.push(frame.clone());
        }
        if owner == 0 && reference.impulses.iter().any(|p| p.normal_impulse > 1e-3) {
            let tick = frame.tick;
            let old = frame.clone();
            let queued = pair.queue_impulse(1, IMPULSE)?;
            compare(queued, &reference, 0);
            uninterrupted.queue_impulse(1, IMPULSE)?;
            if lose_activation {
                assert!(matches!(
                    pair.transfer_island(0),
                    Err(BarrierError::Transport(TransportError::Unavailable(_)))
                ));
                assert_eq!(pair.frame(), &old);
                assert!(matches!(pair.advance(), Err(BarrierError::Pending)));
            }
            let switched = pair.transfer_island(0)?;
            compare(switched, &reference, 1);
            if frames.last() != Some(&old) {
                frames.push(old);
            }
            frames.push(switched.clone());
            println!(
                "tick {tick} active pair (1,2), normal impulse {:.12} N s; moved both bodies with pending +{:.1} N s, lost activation reply {lose_activation}",
                reference.impulses[0].normal_impulse, IMPULSE[0]
            );
            handoff = Some(tick);
        }
    }
    let tick = handoff.ok_or("fixture did not enter warm transfer")?;
    let end = uninterrupted.snapshot();
    let control = unforced.snapshot();
    let receiver_delta =
        end.bodies[1].mass * (end.bodies[1].linear[0] - control.bodies[1].linear[0]);
    assert!(max_residual < 1e-10 && receiver_delta > 0.2 && contact_after > 0.2);
    let [mut source, mut dest] = pair.into_peers();
    let native_bytes = source.native_bytes.ok_or("native export not observed")?;
    assert!(native_bytes > 0 && !dest.lose_activation);
    source.worker.shutdown()?;
    dest.worker.shutdown()?;
    println!(
        "{STEPS} steps bit-identical to uninterrupted solver for both bodies and every contact impulse; native checkpoint {native_bytes} bytes"
    );
    println!(
        "post-handoff contact impulse {contact_after:.12} N s, receiver momentum gain versus unforced {receiver_delta:.12} kg m/s, max system momentum residual versus single input {max_residual:.3e} kg m/s"
    );
    if let Some(path) = capture {
        replay::write(path, &frames)?;
        println!(
            "saved {} committed frames to {}",
            frames.len(),
            path.display()
        );
    }
    Ok(tick)
}
