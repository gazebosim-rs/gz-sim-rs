// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "replay.rs"]
mod replay;
#[path = "warm_process.rs"]
mod warm_process;
use gz_physics_distributed::{
    coordinator::{Coordinator, Error as BarrierError, Frame},
    physics::{Body, Island, Partition, Snapshot, fixture},
    transport::TransportError,
};
use std::{error::Error, path::Path};
use warm_process::{IMPULSE, Process};

const STEPS: u64 = 200;
fn inputs(size: usize) -> [(u64, [f64; 3]); 3] {
    [
        (size as u64 + 1, [1.05, 0., 0.]),
        (1, IMPULSE),
        (size as u64 + 2, [-1.125, 0., 0.]),
    ]
}

fn seeds(size: usize) -> Vec<Body> {
    let template = fixture()[0].promoted();
    let mut seed = if size == 2 {
        fixture().iter().map(Body::promoted).collect::<Vec<_>>()
    } else {
        // Each adjacent gap closes at 1 m/s. The common drift
        // keeps the chain moving after the simultaneous compressive impact.
        (0..size)
            .map(|i| {
                let offset = i as f64 - (size - 1) as f64 / 2.;
                Body {
                    id: i as u64 + 1,
                    position: [1.5 * offset, 0., 0.],
                    linear: [0.5 - offset, 0., 0.],
                    mass: if i == size / 2 { 2. } else { 1. },
                    size: [0.5; 3],
                    ..template.clone()
                }
            })
            .collect()
    };
    for (id, x, z, vx, mass) in [
        (size as u64 + 1, -1.65, replay::PHYSICAL_GAP, 0.65, 1.5),
        (size as u64 + 2, 1.95, -replay::PHYSICAL_GAP, -0.85, 2.5),
    ] {
        seed.push(Body {
            id,
            position: [x, 0., z],
            linear: [vx, 0., 0.],
            mass,
            ..template.clone()
        });
    }
    seed
}

fn compare(frame: &Frame, reference: &Snapshot, moved: bool, size: usize) {
    assert_eq!(frame.tick, reference.tick);
    assert_eq!(
        frame
            .workers
            .iter()
            .map(|w| w.snapshot.bodies.len())
            .sum::<usize>(),
        size + 2
    );
    for expected in &reference.bodies {
        let (owner, actual) = frame.body(expected.id).unwrap();
        let changes_owner = expected.id <= size as u64 && moved;
        let expected_owner = u8::from(changes_owner || expected.id == size as u64 + 2);
        assert_eq!(owner, expected_owner);
        assert_eq!(
            actual, expected,
            "body {} at tick {}",
            expected.id, frame.tick
        );
        assert_eq!(
            frame.workers[usize::from(owner)].generations[&expected.id],
            u64::from(changes_owner)
        );
    }
    let contacts = frame
        .workers
        .iter()
        .map(|w| w.snapshot.contacts)
        .sum::<usize>();
    let impulses = frame
        .workers
        .iter()
        .flat_map(|w| w.snapshot.impulses.iter())
        .collect::<Vec<_>>();
    assert_eq!(contacts, reference.contacts);
    assert!(
        impulses.into_iter().eq(&reference.impulses),
        "contact impulses at tick {}",
        frame.tick
    );
    assert!(
        reference
            .impulses
            .iter()
            .all(|pair| pair.b == pair.a + 1 && pair.b <= size as u64)
    );
}

fn momentum(bodies: &[Body]) -> f64 {
    bodies.iter().map(|body| body.mass * body.linear[0]).sum()
}

/// Check each consumed input independently: the shared contact group and each
/// disconnected bystander have independently conserved momentum.
fn residuals(actual: &Snapshot, control: &Snapshot, size: usize) -> [f64; 3] {
    std::array::from_fn(|i| {
        let (span, input) = match i {
            0 => (0..size, IMPULSE[0]),
            1 => (size..size + 1, inputs(size)[0].1[0]),
            _ => (size + 1..size + 2, inputs(size)[2].1[0]),
        };
        (momentum(&actual.bodies[span.clone()]) - momentum(&control.bodies[span]) - input).abs()
    })
}

pub fn run(
    worker: &Path,
    capture: Option<&Path>,
    lose_activation: bool,
    size: usize,
) -> Result<u64, Box<dyn Error>> {
    let run = u64::from(std::process::id());
    let seed = seeds(size);
    let controls = [&seed[..size + 1], &seed[size + 1..]];
    let mut uninterrupted_workers = Vec::new();
    for bodies in controls {
        uninterrupted_workers.push(Partition::restore(&Snapshot {
            bodies: bodies.to_vec(),
            ..Default::default()
        })?);
    }
    let mut group = Coordinator::new(
        run,
        [
            Process::spawn(worker, run, 0, &seed[..size + 1], lose_activation)?,
            Process::spawn(worker, run, 1, &seed[size + 1..], lose_activation)?,
        ],
        256,
    )?;
    let initial = Snapshot {
        bodies: seed,
        ..Default::default()
    };
    let mut oracle = Island::restore(&initial)?;
    let mut unforced = Island::restore(&initial)?;
    let mut frames = vec![group.frame().clone()];
    compare(group.frame(), &initial, false, size);
    let mut handoff = None;
    let mut post_contact = vec![0.; size - 1];
    let mut peak_residual = [0.0_f64; 3];
    for _ in 0..STEPS {
        let frame = group.advance()?;
        for control in &mut uninterrupted_workers {
            control.step()?;
            let state = control.snapshot();
            for body in &state.bodies {
                assert_eq!(
                    frame.body(body.id).unwrap().1,
                    body,
                    "uninterrupted source partition at tick {}",
                    frame.tick
                );
            }
            let selected = frame
                .workers
                .iter()
                .flat_map(|w| &w.snapshot.impulses)
                .filter(|contact| state.bodies.iter().any(|b| b.id == contact.a));
            assert!(
                selected.eq(&state.impulses),
                "contact changed relative to the uninterrupted partition at tick {}",
                frame.tick
            );
        }
        oracle.step()?;
        unforced.step()?;
        let expected = oracle.snapshot();
        let control = unforced.snapshot();
        compare(frame, &expected, handoff.is_some(), size);
        if handoff.is_some() {
            for (peak, error) in peak_residual
                .iter_mut()
                .zip(residuals(&expected, &control, size))
            {
                *peak = peak.max(error);
            }
            for pair in &expected.impulses {
                post_contact[pair.a as usize - 1] += pair.normal_impulse;
            }
        } else {
            assert_eq!(expected, control);
        }
        if frame.tick % 4 == 0 {
            frames.push(frame.clone());
        }
        if handoff.is_none()
            && expected.impulses.len() == size - 1
            && expected.impulses.iter().all(|p| p.normal_impulse > 1e-3)
        {
            let old = frame.clone();
            for (id, input) in inputs(size) {
                assert_eq!(group.queue_impulse(id, input)?, &old);
                oracle.queue_impulse(id, input)?;
                let original_owner = usize::from(id == size as u64 + 2);
                uninterrupted_workers[original_owner].queue_impulse(id, input)?;
            }
            // Select a single member. Both bystanders must keep their original owner.
            if lose_activation {
                assert!(matches!(
                    group.transfer_group(2),
                    Err(BarrierError::Transport(TransportError::Unavailable(_)))
                ));
                assert_eq!(group.frame(), &old);
                assert!(matches!(group.advance(), Err(BarrierError::Pending)));
            }
            let next = group.transfer_group(2)?;
            compare(next, &expected, true, size);
            if frames.last() != Some(&old) {
                frames.push(old);
            }
            frames.push(next.clone());
            println!(
                "tick {} moved complete {size}-body contact group 0 to 1; active contacts {:?}; bodies {} and {} retained ownership",
                next.tick,
                expected.impulses,
                size + 1,
                size + 2
            );
            handoff = Some(next.tick);
        }
    }
    let tick = handoff.ok_or("no active-contact component migrated")?;
    let end = oracle.snapshot();
    let control = unforced.snapshot();
    let receiver = &end.bodies[size - 1];
    let receiver_gain = receiver.mass * (receiver.linear[0] - control.bodies[size - 1].linear[0]);
    assert!(
        post_contact.iter().all(|j| *j > 0.2)
            && receiver_gain > 0.2
            && peak_residual.iter().all(|r| *r < 1e-10)
    );
    for body in &end.bodies[size..] {
        let before = &initial.bodies[usize::try_from(body.id)? - 1];
        assert!((body.position[0] - before.position[0]).abs() > 1.);
        println!(
            "body {} kept generation 0: x {:.3} to {:.3} m, vx {:.3} to {:.3} m/s",
            body.id, before.position[0], body.position[0], before.linear[0], body.linear[0]
        );
    }
    let [mut left, mut right] = group.into_peers();
    let bytes = left.native_bytes.ok_or("missing selective checkpoint")?;
    assert!(!right.lose_activation);
    left.worker.shutdown()?;
    right.worker.shutdown()?;
    println!(
        "{STEPS} ticks matched all {} body states and contact impulses bitwise against monolithic and uninterrupted partition controls; {bytes} native bytes, lost activation reply {lose_activation}",
        size + 2
    );
    println!(
        "post-handoff contact impulses {post_contact:?} N s; leading box momentum gain {receiver_gain:.12} kg m/s; largest momentum residuals for contact group and each bystander {peak_residual:?} kg m/s"
    );
    if let Some(path) = capture {
        replay::write(path, &frames)?;
        println!(
            "saved {} committed frames to {}. Display tracks keep horizontal metres and box dimensions. The 12 m vertical spacing between physical lanes is shown as 1.15 m.",
            frames.len(),
            path.display()
        );
    }
    Ok(tick)
}
