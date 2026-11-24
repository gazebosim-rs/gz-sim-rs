// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two independently loaded two-body solvers converge under continuous force.
use gz_physics_distributed::coordinator::Frame;
use gz_physics_distributed::{
    coordinator::Coordinator,
    physics::{Body, BodyWrench, Island, Snapshot, fixture},
    transport::WorkerProcess,
};
use std::{error::Error, path::Path};

pub fn run(worker: &Path, mut capture: impl FnMut(&Frame)) -> Result<(), Box<dyn Error>> {
    let bodies: Vec<_> = [-6.84, -6.18, 6.18, 6.84]
        .into_iter()
        .enumerate()
        .map(|(i, x)| Body {
            id: 1 + i as u64,
            position: [x, 0., 0.],
            linear: [if i < 2 { 1.4 } else { -1.4 }, 0., 0.],
            mass: 1.,
            ..fixture()[0].promoted()
        })
        .collect();
    let run = 0x41523;
    let peers = [
        WorkerProcess::spawn(worker, run, 0, &bodies[..2]).unwrap(),
        WorkerProcess::spawn(worker, run, 1, &bodies[2..]).unwrap(),
    ];
    let mut c = Coordinator::new_recorded(run, peers, 1024).unwrap();
    let mut native = Island::restore(&Snapshot {
        bodies,
        ..Default::default()
    })
    .unwrap();
    let mut route = None;
    let mut impact = None;
    let mut errors = [0f64; 3];
    let mut exact = 0;
    let inputs = [(1, 3.), (4, -3.)].map(|(id, f)| BodyWrench {
        id,
        force: [f, 0., 0.],
        torque: [0.; 3],
    });
    let mut both_loaded = false;
    capture(c.frame());
    for tick in 1..=305 {
        let old = c.frame().clone();
        c.advance_in_field_overlapped([0.; 3], &inputs).unwrap();
        native.step_with_wrenches(&inputs).unwrap();
        let f = c.frame();
        let n = native.snapshot();
        if f.workers
            .iter()
            .all(|w| w.snapshot.impulses.iter().any(|p| p.normal_impulse > 1e-4))
        {
            both_loaded = true;
        }
        if f.body(1).unwrap().0 == f.body(4).unwrap().0 && route.is_none() {
            assert!(
                old.workers.iter().all(|w| w
                    .snapshot
                    .impulses
                    .iter()
                    .any(|p| p.normal_impulse > 1e-4))
            );
            route = Some(tick);
        }
        let mut impulses = f
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.clone())
            .collect::<Vec<_>>();
        impulses.sort_by_key(|p| (p.a, p.b));
        if impulses
            .iter()
            .any(|p| p.a == 2 && p.b == 3 && p.normal_impulse > 0.)
        {
            impact.get_or_insert(tick);
        }
        assert_eq!(
            impulses.iter().map(|p| (p.a, p.b)).collect::<Vec<_>>(),
            n.impulses.iter().map(|p| (p.a, p.b)).collect::<Vec<_>>(),
            "contacts at {tick}"
        );
        let mut identical = impulses == n.impulses;
        for (a, b) in impulses.iter().zip(&n.impulses) {
            errors[2] = errors[2].max((a.normal_impulse - b.normal_impulse).abs());
        }
        for b in &n.bodies {
            let actual = f.body(b.id).unwrap().1;
            identical &= actual == b;
            for (a, r) in actual.position.into_iter().zip(b.position) {
                errors[0] = errors[0].max((a - r).abs());
            }
            for (a, r) in actual.linear.into_iter().zip(b.linear) {
                errors[1] = errors[1].max((a - r).abs());
            }
            for (a, r) in actual.orientation.into_iter().zip(b.orientation) {
                assert!(
                    (a - r).abs() < 1e-6,
                    "orientation {tick}: {actual:?} vs {b:?}"
                );
            }
        }
        exact += u64::from(identical);
        if tick % 4 == 0 || tick == 305 || route == Some(tick) || impact == Some(tick) {
            capture(f);
        }
        if route == Some(tick) {
            c.compact().unwrap();
            c.recover(0).unwrap();
            c.recover(1).unwrap();
        }
    }
    println!(
        "both independently loaded {both_loaded}, route {route:?}, cross impact {impact:?}, {exact}/305 bit-exact, maximum position/velocity/impulse differences {errors:?}"
    );
    assert!(both_loaded && route.is_some() && impact.is_some() && route < impact);
    assert!(errors[0] < 1e-6 && errors[1] < 1e-5 && errors[2] < 1e-5);
    assert_eq!(exact, 305);
    for mut p in c.into_peers() {
        p.shutdown()?;
    }
    Ok(())
}
