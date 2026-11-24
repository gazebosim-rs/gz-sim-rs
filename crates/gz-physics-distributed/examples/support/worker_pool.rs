// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    physics::{Body, Island, Primitive, SimulationStep, Snapshot, Tier},
    pool::WorkerPool,
};
use serde_json::{Value, json};
use std::{collections::BTreeSet, error::Error, path::Path};

fn participant(id: u64, shape: Primitive, x: f64, y: f64, speed: f64) -> Body {
    Body {
        id,
        primitive: shape,
        tier: Tier::Dynamic,
        mass: 1.,
        friction: 0.,
        size: match shape {
            Primitive::Box => [0.5, 0.4, 0.6],
            Primitive::Sphere => [0.5; 3],
            Primitive::Cylinder | Primitive::Capsule => [0.5, 0.5, 0.7],
        },
        position: [x, y, 0.],
        orientation: [0., 0., 0., 1.],
        linear: [speed, 0., 0.],
        angular: [0.; 3],
    }
}

/// Two independent impacts begin across different process boundaries. Native
/// reference worlds know nothing about the pool's handoffs or its worker loss.
pub fn qualify(executable: &Path) -> Result<Value, Box<dyn Error>> {
    let step = SimulationStep::Ms5;
    let seeds = vec![
        vec![participant(11, Primitive::Sphere, -0.8, 0., 1.3)],
        vec![participant(19, Primitive::Box, 0.8, 0., -0.7)],
        vec![participant(31, Primitive::Capsule, -0.8, 16., 1.3)],
        vec![participant(47, Primitive::Sphere, 0.8, 16., -0.7)],
    ];
    let mut control = Island::restore(&Snapshot {
        step,
        bodies: seeds.iter().flatten().cloned().collect(),
        ..Default::default()
    })?;
    let mut pool = WorkerPool::spawn(executable, 71003, seeds, step, None, 37)?;
    let pids: Vec<_> = (0..4).map(|i| pool.worker_pid(i).unwrap()).collect();
    assert_eq!(pids.iter().collect::<BTreeSet<_>>().len(), 4);
    let mut routed = [None; 2];
    let mut contacted = [None; 2];
    let mut peak_impulse = [0f64; 2];
    let mut maxima = [0f64; 3];
    let mut body_samples = 0;
    let mut exact = 0;
    let mut report_events = Vec::new();
    let mut compacted_at = None;
    let mut prior_entries = pool.retained_entries()[0];
    for tick in 1..=240 {
        let before = pool.frame().clone();
        if tick == 135 {
            // Both endpoints have a live native contact. Move the first pair
            // through an owner that has never seen it, then to another owner.
            for d in [3, 2, 0] {
                let from = pool.frame().body(11).unwrap().0;
                assert_ne!(from, d);
                let native_pair = pool.frame().workers[from as usize]
                    .snapshot
                    .impulses
                    .clone();
                assert!(native_pair.iter().any(|p| (p.a, p.b) == (11, 19)));
                pool.transfer_component(11, d)?;
                for p in native_pair.iter().filter(|p| p.a == 11) {
                    assert!(
                        pool.frame().workers[d as usize]
                            .snapshot
                            .impulses
                            .contains(p)
                    );
                }
                report_events.push(json!({"tick":tick-1,"migrated":[11,19],"from":from,"to":d}));
            }
            pool.compact()?;
            assert!(pool.retained_entries().iter().all(|&n| n == 0));
            compacted_at = Some(tick - 1);
        }
        if tick == 141 {
            let old = pool.worker_pid(2).unwrap();
            pool.terminate_worker(2)?;
            assert!(pool.advance().is_err());
            assert_eq!(pool.frame(), &before);
            pool.recover(2)?;
            let new = pool.worker_pid(2).unwrap();
            assert_ne!(old, new);
            report_events.push(json!({"interrupted_barrier":tick,"last_published":before.tick,"replaced_worker":2,"old_pid":old,"new_pid":new}));
        }
        pool.advance()?;
        control.step()?;
        let frame = pool.frame();
        assert_eq!(frame.tick, tick);
        assert_eq!(
            frame
                .workers
                .iter()
                .map(|w| w.snapshot.bodies.len())
                .sum::<usize>(),
            4
        );
        let reference = control.snapshot();
        for b in &reference.bodies {
            body_samples += 1;
            let (_, live) = frame.body(b.id).unwrap();
            exact += usize::from(live == b);
            for j in 0..3 {
                maxima[0] = maxima[0].max((b.position[j] - live.position[j]).abs());
                maxima[1] = maxima[1].max((b.linear[j] - live.linear[j]).abs());
                maxima[2] = maxima[2].max((b.angular[j] - live.angular[j]).abs());
            }
            assert!(
                b.orientation
                    .iter()
                    .zip(live.orientation)
                    .all(|(a, c)| (*a - c).abs() < 1e-10)
            );
        }
        for (i, (a, b)) in [(11, 19), (31, 47)].into_iter().enumerate() {
            let owner = frame.body(a).unwrap().0;
            if frame.body(b).unwrap().0 == owner {
                routed[i].get_or_insert(tick);
                if let Some(p) = frame.workers[owner as usize]
                    .snapshot
                    .impulses
                    .iter()
                    .find(|p| (p.a, p.b) == (a, b))
                {
                    contacted[i].get_or_insert(tick);
                    peak_impulse[i] = peak_impulse[i].max(p.normal_impulse);
                    let reference = reference
                        .impulses
                        .iter()
                        .find(|q| (q.a, q.b) == (a, b))
                        .expect("native contact");
                    assert!((reference.normal_impulse - p.normal_impulse).abs() < 1e-10);
                }
            }
        }
        if pool.retained_entries()[0] < prior_entries {
            compacted_at.get_or_insert(tick);
        }
        prior_entries = pool.retained_entries()[0];
    }
    assert!(
        routed
            .iter()
            .zip(contacted)
            .all(|(r, c)| r.is_some() && c.is_some() && *r < c)
    );
    assert!(peak_impulse.into_iter().all(|p| p > 0.5));
    assert!(maxima.into_iter().all(|n| n < 1e-10), "{maxima:?}");
    assert!(compacted_at.is_some());
    Ok(
        json!({"workers":4,"initial_pids":pids,"steps":240,"dt_ms":5,"first_colocated_tick":routed,"first_native_contact_tick":contacted,"peak_normal_impulse_Ns":peak_impulse,"max_abs_position_m":maxima[0],"max_abs_velocity_m_s":maxima[1],"max_abs_spin_rad_s":maxima[2],"bitwise_body_samples":exact,"body_samples":body_samples,"journal_limit":37,"retained_entries":pool.retained_entries(),"events":report_events}),
    )
}
