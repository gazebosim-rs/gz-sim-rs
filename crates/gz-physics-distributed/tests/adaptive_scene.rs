// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, PhysicsBackend, RotationSpec};
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, Tier},
    protocol::Observation,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured, reset_loaded_world};
use sdformat::NoResources;
use std::{error::Error, path::Path, time::Duration};

const WORLD: &str = include_str!("../../../examples/distributed_boxes.sdf");

fn load(width: u8, step: Option<Duration>) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_configured(
        WORLD,
        "boxes.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedAdaptive(width),
            step,
            distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
            noise_seed: [0; 32],
        },
    )
}

fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
}

fn workers(backend: &SceneBackend) -> &[Observation] {
    if let Some(pool) = backend.pool_frame() {
        &pool.workers
    } else {
        &backend.frame().unwrap().workers
    }
}

fn add_body(native: &mut MultiFidelityBackend, b: &Body) -> Result<BodyId, Box<dyn Error>> {
    Ok(native.spawn_partition_primitive(
        BodySpec {
            shape: b.shape()?,
            mass: (b.tier != Tier::Fixed).then_some(b.mass),
            position: b.position,
            velocity: b.linear,
        },
        RotationSpec {
            orientation: b.orientation,
            angular_velocity: b.angular,
        },
        b.tier == Tier::Prescribed,
        b.friction,
    )?)
}

#[test]
fn configured_sdf_uses_one_public_tick_for_the_sum_of_native_contact_solves()
-> Result<(), Box<dyn Error>> {
    for width in [2, 3, 4, 8] {
        let mut sim = load(width, None)?;
        assert_eq!(backend(&sim).last_substeps(), None);
        let mut native = MultiFidelityBackend::new([0.; 3])?;
        let mut bodies: Vec<_> = workers(backend(&sim))
            .iter()
            .flat_map(|o| &o.snapshot.bodies)
            .collect();
        bodies.sort_by_key(|b| b.id);
        for body in bodies {
            assert_eq!(add_body(&mut native, body)?.raw(), body.id);
        }
        let mut coarse = 0;
        let mut fine = 0;
        let mut impact = 0.;
        let mut at_impact = None;
        for tick in 1..=132 {
            sim.step_exact(1)?;
            assert_eq!(sim.elapsed(), Duration::from_millis(tick * 10));
            let scene = backend(&sim);
            let n = scene.last_substeps().unwrap();
            match n {
                1 => coarse += 1,
                10 => fine += 1,
                _ => panic!("unexpected refinement count {n}"),
            };
            let mut expected = None;
            for _ in 0..n {
                native.step(Duration::from_millis(10 / u64::from(n)))?;
                for contact in native
                    .rigid_contact_observations()
                    .iter()
                    .filter(|p| p.active)
                {
                    *expected.get_or_insert(0.) += contact.normal_impulse;
                }
            }
            let mut published = None;
            for worker in workers(scene) {
                assert_eq!(worker.snapshot.tick, tick);
                for body in &worker.snapshot.bodies {
                    let state = native.state(BodyId::from_raw(body.id)).unwrap();
                    assert_eq!(body.position, state.position);
                    assert_eq!(body.orientation, state.orientation);
                    assert_eq!(body.linear, state.velocity);
                    assert_eq!(body.angular, state.angular_velocity);
                }
                for receipt in &worker.snapshot.impulses {
                    assert!(published.replace(receipt.normal_impulse).is_none());
                }
            }
            assert_eq!(published, expected, "width {width}, tick {tick}");
            if published.unwrap_or(0.) > 0. {
                at_impact.get_or_insert(n);
                impact += published.unwrap();
            }
        }
        assert!(coarse > 0 && fine > 0);
        assert_eq!(at_impact, Some(10));
        assert!((impact - 1.5_f64).abs() < 1e-6, "impulse {impact}");
        reset_loaded_world(&mut sim, |_| ())?;
        assert_eq!(backend(&sim).last_substeps(), None);
        assert_eq!(sim.elapsed(), Duration::ZERO);
        sim.set_paused(true);
        assert!(sim.tick()?.dt.is_zero());
        assert_eq!(backend(&sim).last_substeps(), None);
    }
    Ok(())
}

#[test]
fn adaptive_scene_requires_ten_millisecond_public_barriers() {
    assert!(
        load(4, Some(Duration::from_millis(5)))
            .err()
            .expect("incompatible public step")
            .to_string()
            .contains("10 ms")
    );
}

/// The same sphere meets two distinct surfaces in one ordinary SDF world.
/// Between them it has a genuine free-flight interval, allowing coarsening.
#[test]
fn sphere_refines_on_a_finite_cylinder_then_on_shared_ground() -> Result<(), Box<dyn Error>> {
    let gravity = [1.25, 0., -9.81];
    let public = Duration::from_millis(10);
    let mut sim = load_world_configured(
        include_str!("../../../examples/distributed_cylinder_tabletop.sdf"),
        "tabletop.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedAdaptive(4),
            step: Some(public),
            distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
            noise_seed: [0; 32],
        },
    )?;
    let mut native = MultiFidelityBackend::new(gravity)?;
    native.set_partition_step(public)?;
    native.set_partition_ground_material(0., 0.)?;
    let mut bodies: Vec<_> = workers(backend(&sim))
        .iter()
        .flat_map(|w| &w.snapshot.bodies)
        .collect();
    bodies.sort_unstable_by_key(|b| b.id);
    assert_eq!(bodies.len(), 2);
    for b in bodies {
        assert_eq!(add_body(&mut native, b)?.raw(), b.id);
    }
    let mut first_cap = None;
    let mut last_cap = 0;
    let mut first_ground = None;
    let mut coarse_ticks = Vec::new();
    let mut total_support = 0.;
    for tick in 1..=220 {
        sim.step_exact(1)?;
        let scene = backend(&sim);
        let n = scene.last_substeps().unwrap();
        if n == 1 {
            coarse_ticks.push(tick);
        }
        let mut cap = None;
        let mut ground = None;
        for _ in 0..n {
            native.step(public / u32::from(n))?;
            for j in native
                .rigid_contact_observations()
                .iter()
                .filter(|j| j.active)
            {
                *cap.get_or_insert(0.) += j.normal_impulse;
            }
            for j in native
                .partition_ground_contacts()
                .iter()
                .filter(|j| j.active)
            {
                *ground.get_or_insert(0.) += j.normal_impulse;
            }
        }
        let mut received_cap = None;
        let mut received_ground = None;
        for w in workers(scene) {
            assert_eq!(w.snapshot.tick, tick);
            for b in &w.snapshot.bodies {
                let state = native.state(BodyId::from_raw(b.id)).unwrap();
                assert_eq!(
                    b.position.map(f64::to_bits),
                    state.position.map(f64::to_bits)
                );
                assert_eq!(
                    b.orientation.map(f64::to_bits),
                    state.orientation.map(f64::to_bits)
                );
                assert_eq!(b.linear.map(f64::to_bits), state.velocity.map(f64::to_bits));
                assert_eq!(
                    b.angular.map(f64::to_bits),
                    state.angular_velocity.map(f64::to_bits)
                );
            }
            for j in &w.snapshot.impulses {
                assert!(received_cap.replace(j.normal_impulse).is_none());
            }
            for j in &w.snapshot.supports {
                assert!(received_ground.replace(j.normal_impulse).is_none());
            }
        }
        assert_eq!(cap.map(f64::to_bits), received_cap.map(f64::to_bits));
        assert_eq!(ground.map(f64::to_bits), received_ground.map(f64::to_bits));
        if cap.unwrap_or(0.) > 0. {
            first_cap.get_or_insert(tick);
            last_cap = tick;
            assert_eq!(n, 10);
        }
        if ground.unwrap_or(0.) > 0. {
            first_ground.get_or_insert(tick);
            assert_eq!(n, 10);
        }
        if tick > 200 {
            total_support += ground.unwrap_or(0.);
        }
    }
    let cap = first_cap.unwrap();
    let land = first_ground.unwrap();
    assert!((24..=26).contains(&cap) && (140..=147).contains(&last_cap));
    assert!((186..=193).contains(&land));
    assert!(coarse_ticks.contains(&1) && coarse_ticks.iter().any(|&t| t > last_cap && t < land));
    // Settled normal load equals the weight of the SDF's 0.6 kg sphere.
    assert!((total_support / 0.2 - 0.6 * 9.81).abs() < 1e-8);
    println!(
        "cap contact {cap}..{last_cap}, ground touchdown {land}, {} coarse and {} refined ticks",
        coarse_ticks.len(),
        220 - coarse_ticks.len()
    );
    Ok(())
}

#[test]
fn configured_islands_refine_individual_ground_arrivals_and_reset_the_receipt()
-> Result<(), Box<dyn Error>> {
    let dt = Duration::from_millis(10);
    let gravity = [0., 0., -9.81];
    for width in [2, 4, 8] {
        let mut sim = load_world_configured(
            include_str!("../../../examples/distributed_island_drops.sdf"),
            "island_drops.sdf",
            "default",
            &NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::DistributedIslands(width),
                step: Some(dt),
                noise_seed: [0; 32],
                distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
            },
        )?;
        let mut refs = Vec::new();
        for b in workers(backend(&sim))
            .iter()
            .flat_map(|w| &w.snapshot.bodies)
        {
            let mut native = MultiFidelityBackend::new(gravity)?;
            native.set_partition_step(dt)?;
            native.set_partition_ground_material(0., b.friction)?;
            let handle = add_body(&mut native, b)?;
            refs.push((b.id, b.position[0], native, handle));
        }
        assert_eq!(refs.len(), 3);
        let mut first_refined = std::collections::BTreeMap::new();
        for tick in 1..=105 {
            sim.step_exact(1)?;
            let scene = backend(&sim);
            let ids = scene.last_island_refined_bodies().unwrap();
            for (id, x, control, handle) in &mut refs {
                let refined = ids.binary_search(id).is_ok();
                if refined {
                    first_refined.entry(x.to_bits()).or_insert(tick);
                }
                let steps = if refined { 10 } else { 1 };
                for _ in 0..steps {
                    control.step(dt / steps)?;
                }
                let state = control.state(*handle).unwrap();
                let observed = workers(scene)
                    .iter()
                    .flat_map(|w| &w.snapshot.bodies)
                    .find(|b| b.id == *id)
                    .unwrap();
                assert_eq!(
                    observed.position.map(f64::to_bits),
                    state.position.map(f64::to_bits)
                );
                assert_eq!(
                    observed.linear.map(f64::to_bits),
                    state.velocity.map(f64::to_bits)
                );
            }
            if tick == 1 {
                assert_eq!(ids.len(), 1);
            }
            if tick == 75 {
                assert_eq!(ids.len(), 2);
            }
        }
        assert_eq!(first_refined[&(-7_f64).to_bits()], 1);
        assert!((47..=51).contains(&first_refined[&0_f64.to_bits()]));
        assert!((95..=100).contains(&first_refined[&7_f64.to_bits()]));
        reset_loaded_world(&mut sim, |_| ())?;
        assert_eq!(backend(&sim).last_island_refined_bodies(), None);
        sim.step_exact(1)?;
        assert_eq!(backend(&sim).last_island_refined_bodies().unwrap().len(), 1);
    }
    Ok(())
}

#[test]
fn diagonal_sphere_stacks_keep_independent_loaded_contacts_in_the_normal_scene()
-> Result<(), Box<dyn Error>> {
    let dt = Duration::from_millis(10);
    for width in [2, 4] {
        let mut sim = load_world_configured(
            include_str!("../../../examples/distributed_diagonal_stacks.sdf"),
            "diagonal_stacks.sdf",
            "default",
            &NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::DistributedIslands(width),
                step: Some(dt),
                noise_seed: [0; 32],
                distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
            },
        )?;
        // Each column's independent native control receives just that column's two spheres.
        let mut columns = Vec::new();
        for (x, y) in [(-6.5, 0.), (0., 6.5), (6.5, 0.), (0., -6.5)] {
            let mut native = MultiFidelityBackend::new([0., 0., -9.81])?;
            native.set_partition_step(dt)?;
            native.set_partition_ground_material(0., 0.)?;
            let mut bodies: Vec<_> = workers(backend(&sim))
                .iter()
                .flat_map(|w| &w.snapshot.bodies)
                .filter(|b| b.position[0] == x && b.position[1] == y)
                .collect();
            bodies.sort_by_key(|b| b.id);
            let mut handles = Vec::new();
            for body in bodies {
                handles.push((body.id, add_body(&mut native, body)?));
            }
            assert_eq!(handles.len(), 2);
            columns.push((native, handles));
        }
        for tick in 1..=110 {
            sim.step_exact(1)?;
            let scene = backend(&sim);
            let mut observations = std::collections::BTreeMap::new();
            let mut impulses = std::collections::BTreeMap::new();
            let mut supports = std::collections::BTreeMap::new();
            for w in workers(scene) {
                for b in &w.snapshot.bodies {
                    observations.insert(b.id, b);
                }
                for j in &w.snapshot.impulses {
                    impulses.insert((j.a, j.b), j.normal_impulse);
                }
                for j in &w.snapshot.supports {
                    supports.insert(j.body, j.normal_impulse);
                }
                for c in &w.components {
                    assert!(c.ids.len() <= 2, "unrelated columns joined at tick {tick}");
                }
            }
            for (native, handles) in &mut columns {
                let fine = scene.last_island_refined_bodies().unwrap();
                assert!(handles.iter().all(|(id, _)| fine.binary_search(id).is_ok()));
                let mut pair = None;
                let mut floor = std::collections::BTreeMap::new();
                for _ in 0..10 {
                    native.step(dt / 10)?;
                    for j in native
                        .rigid_contact_observations()
                        .iter()
                        .filter(|p| p.active)
                    {
                        *pair.get_or_insert(0.) += j.normal_impulse;
                    }
                    for j in native
                        .partition_ground_contacts()
                        .iter()
                        .filter(|p| p.active)
                    {
                        *floor.entry(j.body).or_insert(0.) += j.normal_impulse;
                    }
                }
                for &(id, h) in handles.iter() {
                    let b = observations[&id];
                    let state = native.state(h).unwrap();
                    assert_eq!(
                        b.position.map(f64::to_bits),
                        state.position.map(f64::to_bits)
                    );
                    assert_eq!(b.linear.map(f64::to_bits), state.velocity.map(f64::to_bits));
                    assert_eq!(supports.get(&id), floor.get(&h));
                }
                assert_eq!(impulses.get(&(handles[0].0, handles[1].0)), pair.as_ref());
            }
            if tick == 110 {
                assert_eq!((impulses.len(), supports.len()), (4, 4));
            }
        }
    }
    Ok(())
}
