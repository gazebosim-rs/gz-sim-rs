// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Deterministic cargo routing on an SDF fleet, with two delayed input waves.
use super::observation;
use bevy_ecs::prelude::Entity;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{BodyHandle, Name, Velocity, WorldPose, queue_link_world_wrench};
use std::{collections::BTreeMap, error::Error, time::Instant};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const STEPS: usize = 220;
struct Cargo {
    id: u64,
    entity: Entity,
    direction: f64,
}
struct Traffic {
    sim: Simulation,
    cargo: Vec<Cargo>,
    decks: Vec<(BodyId, Entity)>,
}
impl Traffic {
    fn new(count: usize, automatic: bool) -> Result<Self> {
        if !(256..=100_000).contains(&count) {
            return Err("expected 256..=100000 traffic decks".into());
        }
        let mut sim = observation::scene(count, true)?;
        if !automatic {
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .disable_fleet_isolation();
        }
        let cols = count.isqrt() + usize::from(count.isqrt().pow(2) != count);
        let mut cargo = BTreeMap::new();
        let mut decks = BTreeMap::new();
        for e in sim.world().iter_entities() {
            let (Some(handle), Some(name), Some(public)) =
                (e.get::<BodyHandle>(), e.get::<Name>(), e.get::<EntityId>())
            else {
                continue;
            };
            let name = name.0.split("::").nth(1).unwrap();
            if let Some(index) = name.strip_prefix("payload_") {
                let index: usize = index.parse()?;
                cargo.insert(
                    index,
                    Cargo {
                        id: public.get(),
                        entity: e.id(),
                        direction: if index % cols < cols / 2 { 1. } else { -1. },
                    },
                );
            } else if let Some(index) = name.strip_prefix("deck_") {
                decks.insert(index.parse::<usize>()?, (handle.id(), e.id()));
            }
        }
        Ok(Self {
            sim,
            cargo: cargo.into_values().collect(),
            decks: decks.into_values().collect(),
        })
    }
    fn backend(&self) -> &MultiFidelityBackend {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref()
            .unwrap()
    }
    fn step(&mut self, tick: usize) -> Result<()> {
        // Each third of the fleet launches on a different barrier. The 1 kg
        // boxes receive real force for one 10 ms tick, then fly and land.
        for (lane, c) in self.cargo.iter().enumerate() {
            let phase = lane % 3 * 7;
            let pulse = if tick == 26 + phase {
                Some((510., 185.))
            } else if tick == 126 + phase {
                Some((-620., 157.))
            } else {
                None
            };
            if let Some((x, z)) = pulse {
                queue_link_world_wrench(&mut self.sim, c.id, [x * c.direction, 0., z], [0.; 3])?;
            }
        }
        self.sim.step_exact(1)?;
        Ok(())
    }
    fn pairs(&self) -> BTreeMap<(BodyId, BodyId), (bool, f64)> {
        self.backend()
            .rigid_contact_observations()
            .into_iter()
            .map(|p| ((p.a, p.b), (p.active, p.normal_impulse)))
            .collect()
    }
}
fn max_delta<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(x, y)| {
            assert!(x.is_finite() && y.is_finite());
            (x - y).abs()
        })
        .fold(0., f64::max)
}
pub fn verify(count: usize) -> Result<()> {
    let mut native = Traffic::new(count, false)?;
    let mut auto = Traffic::new(count, true)?;
    let mut residuals = [0.0_f64; 6];
    let mut observations = 0usize;
    let mut min_contacts = usize::MAX;
    let mut destinations = std::collections::BTreeSet::new();
    for tick in 1..=STEPS {
        native.step(tick)?;
        auto.step(tick)?;
        let (n, a) = (native.pairs(), auto.pairs());
        if n.keys().ne(a.keys()) {
            return Err(format!(
                "pair set diverged at tick {tick}: native {}, auto {}",
                n.len(),
                a.len()
            )
            .into());
        }
        let mut contacts = 0;
        for (pair, (active, impulse)) in n {
            let r = a[&pair];
            assert_eq!(active, r.0, "tick {tick}, pair {pair:?}");
            residuals[4] = residuals[4].max((impulse - r.1).abs());
            observations += 1;
            if active {
                contacts += 1;
                destinations.insert(pair);
            }
        }
        min_contacts = min_contacts.min(contacts);
        for (n, a) in native.cargo.iter().zip(&auto.cargo) {
            let np = native.sim.world().get::<WorldPose>(n.entity).unwrap();
            let ap = auto.sim.world().get::<WorldPose>(a.entity).unwrap();
            let nv = native.sim.world().get::<Velocity>(n.entity).unwrap();
            let av = auto.sim.world().get::<Velocity>(a.entity).unwrap();
            for (i, d) in [
                max_delta(np.translation, ap.translation),
                max_delta(np.rotation, ap.rotation),
                max_delta(nv.linear, av.linear),
                max_delta(nv.angular, av.angular),
            ]
            .into_iter()
            .enumerate()
            {
                residuals[i] = residuals[i].max(d);
            }
        }
        for (n, a) in native.decks.iter().zip(&auto.decks) {
            let np = native.sim.world().get::<WorldPose>(n.1).unwrap();
            let ap = auto.sim.world().get::<WorldPose>(a.1).unwrap();
            residuals[5] = residuals[5].max(max_delta(np.translation, ap.translation));
            assert_eq!(np.rotation, ap.rotation);
        }
    }
    println!(
        "ticks={STEPS} decks={count} cargo={} pairs_compared={observations} distinct_active_pairs={} min_contacts={min_contacts} cargo_position_m={:.3e} quaternion={:.3e} cargo_linear_m_s={:.3e} cargo_angular_rad_s={:.3e} pair_impulse_ns={:.3e} deck_position_m={:.3e}",
        auto.cargo.len(),
        destinations.len(),
        residuals[0],
        residuals[1],
        residuals[2],
        residuals[3],
        residuals[4],
        residuals[5]
    );
    assert!(destinations.len() > auto.cargo.len() && min_contacts < auto.cargo.len());
    assert!(residuals.iter().all(|&v| v < 1e-8));
    Ok(())
}
pub fn measure(count: usize, automatic: bool) -> Result<()> {
    let mut traffic = Traffic::new(count, automatic)?;
    for tick in 1..=20 {
        traffic.step(tick)?;
    }
    let mut times = Vec::new();
    let start = Instant::now();
    for tick in 21..=STEPS {
        let t = Instant::now();
        traffic.step(tick)?;
        times.push(t.elapsed());
    }
    let wall = start.elapsed().as_secs_f64();
    times.sort_unstable();
    let imu = observation::sample(&traffic.sim);
    assert_eq!(imu.time, traffic.sim.elapsed());
    assert_eq!(imu.reading.as_ref().unwrap().specific_force, [0., 0., 9.81]);
    println!(
        "traffic={} decks={count} cargo={} poses={} contacts={} warmup=20 measured=200 dt_ms=10 wall_s={wall:.6} real_time_factor={:.5} p50_ms={:.4} p99_ms={:.4} isolation={:?}",
        if automatic { "auto" } else { "native" },
        traffic.cargo.len(),
        traffic
            .sim
            .world()
            .iter_entities()
            .filter(|e| e.contains::<WorldPose>())
            .count(),
        traffic.backend().active_contact_pairs(),
        2. / wall,
        times[99].as_secs_f64() * 1000.,
        times[197].as_secs_f64() * 1000.,
        traffic.backend().fleet_isolation_stats()
    );
    Ok(())
}
