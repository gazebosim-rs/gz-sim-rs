// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measured Coulomb sliding and static friction on the same fixed ramp.
use gz_physics::{BodySpec, DynamicBackend, PhysicsBackend, RotationSpec, Shape};
use gz_physics_distributed::{SceneBackend, coordinator::Frame, physics::Tier};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured, reset_loaded_world};
use sdformat::NoResources;
use std::{error::Error, path::Path, time::Duration};

fn frame(sim: &Simulation) -> &Frame {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
        .frame()
        .unwrap()
}
fn along(v: [f64; 3]) -> f64 {
    0.25_f64.cos() * v[0] - 0.25_f64.sin() * v[2]
}

pub fn run(worker: &Path) -> Result<(), Box<dyn Error>> {
    let xml = include_str!("../../../../examples/distributed_friction_incline.sdf");
    let mut sim = load_world_configured(
        xml,
        "distributed_friction_incline.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?;
    let seed = frame(&sim).clone();
    let h = Duration::from_millis(5);
    let gravity = [0., 0., -9.81];
    let mut direct = MultiFidelityBackend::new([0.; 3])?;
    direct.set_partition_step(h)?;
    let mut ids = Vec::new();
    for (id, mu) in [0.8, 0.12, 1.].into_iter().enumerate() {
        let b = seed.body(id as u64).unwrap().1;
        assert_eq!(b.friction, mu);
        ids.push(direct.spawn_partition_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: b.size.map(|s| s * 0.5),
                },
                position: b.position,
                velocity: b.linear,
                mass: (b.tier != Tier::Fixed).then_some(b.mass),
            },
            RotationSpec {
                orientation: b.orientation,
                angular_velocity: b.angular,
            },
            false,
            mu,
        )?);
    }
    let mut first = [0; 2];
    let mut v1 = 0.;
    let mut rest_start = 0.;
    let mut rest_drift = 0f64;
    let mut normal = [0.; 2];
    for tick in 1..=600 {
        sim.step_exact(1)?;
        direct.set_partition_gravity(gravity)?;
        direct.step(h)?;
        direct.set_partition_gravity([0.; 3])?;
        let f = frame(&sim);
        for (index, &id) in ids.iter().enumerate() {
            let state = direct.state(id).ok_or("missing native box")?;
            let b = f.body(index as u64).unwrap().1;
            assert_eq!(
                (
                    b.position.map(f64::to_bits),
                    b.orientation.map(f64::to_bits),
                    b.linear.map(f64::to_bits),
                    b.angular.map(f64::to_bits)
                ),
                (
                    state.position.map(f64::to_bits),
                    state.orientation.map(f64::to_bits),
                    state.velocity.map(f64::to_bits),
                    state.angular_velocity.map(f64::to_bits)
                ),
                "body {index}, tick {tick}"
            );
        }
        let mut observations = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(|p| (p.a, p.b, p.normal_impulse.to_bits()))
            .collect::<Vec<_>>();
        observations.sort();
        let native = direct
            .rigid_contact_observations()
            .into_iter()
            .filter(|p| p.active)
            .map(|p| (p.a.raw(), p.b.raw(), p.normal_impulse.to_bits()))
            .collect::<Vec<_>>();
        assert_eq!(observations, native, "contacts at {tick}");
        for (a, b, j) in observations {
            if a == 0 && f64::from_bits(j) > 0. {
                let i = (b - 1) as usize;
                if first[i] == 0 {
                    first[i] = tick;
                }
                if tick > 200 {
                    normal[i] += f64::from_bits(j);
                }
            }
        }
        if tick == 200 {
            v1 = along(f.body(1).unwrap().1.linear);
            rest_start = along(f.body(2).unwrap().1.position);
        }
        if tick > 200 {
            rest_drift = rest_drift.max((along(f.body(2).unwrap().1.position) - rest_start).abs());
        }
    }
    let f = frame(&sim);
    let sliding = f.body(1).unwrap().1;
    let a = (along(sliding.linear) - v1) / 2.;
    let ideal = 9.81 * (0.25_f64.sin() - 0.12 * 0.25_f64.cos());
    let load = 2. * 9.81 * 0.25_f64.cos();
    let reaction = normal.map(|j| j / 2.);
    println!(
        "incline 0.25 rad, 600 x 5 ms; contact ticks {first:?}; sliding acceleration {a:.10} m/s² (analytical {ideal:.10}), sticking drift {rest_drift:.3e} m over final 2 s, mean normal reactions {reaction:?} N (analytical {load:.10}); every native body and impulse bit matched"
    );
    assert!(first.into_iter().all(|t| t > 0 && t < 100));
    assert!((a - ideal).abs() < 0.02 && rest_drift < 0.002);
    assert!(reaction.iter().all(|&f| (f - load).abs() < 0.02));
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(frame(&sim), &seed);
    Ok(())
}
