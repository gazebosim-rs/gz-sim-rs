// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Offline comparison of one native world against a known contact partition.
//! The second mode assumes the fixture's contact topology in advance; it does
//! not dispatch arbitrary scenes or estimate the cost of discovering that split.
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, BodySpec, BodyState, DynamicBackend, PhysicsBackend, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Phase, Simulation};
use std::{
    collections::BTreeMap,
    error::Error,
    time::{Duration, Instant},
};

const DT: Duration = Duration::from_millis(10);
const STRIDE: usize = 100;
const WARMUP: u64 = 20;
const STEPS: u64 = 100;
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Component)]
struct Observed {
    id: BodyId,
    state: BodyState,
}
fn publish(backend: Res<PhysicsBackend>, mut rows: Query<&mut Observed>) {
    backend.read_states(&mut |states| {
        for mut row in &mut rows {
            row.state = states.state(row.id).expect("fleet body");
        }
    });
}
struct Fixture {
    worlds: Vec<Simulation>,
    // Fixed physical order enables direct comparison across solver-local IDs.
    actors: Vec<(usize, Entity)>,
    active_ids: BTreeMap<BodyId, usize>,
}
impl Fixture {
    fn new(count: usize, split: bool, automatic: bool) -> Result<Self> {
        let columns = count.isqrt() + usize::from(count.isqrt().pow(2) != count);
        let mut result = Self {
            worlds: Vec::new(),
            actors: Vec::new(),
            active_ids: BTreeMap::new(),
        };
        for _ in 0..if split { 2 } else { 1 } {
            let mut sim = Simulation::new(DT)?;
            let mut solver = MultiFidelityBackend::new([0., 0., -9.81])?;
            if !automatic {
                solver.disable_fleet_isolation();
            }
            gz_physics::install(&mut sim, solver)?;
            sim.add_systems(Phase::PhysicsCommit, publish);
            result.worlds.push(sim);
        }
        for i in 0..count {
            let loaded = i % STRIDE == 0;
            let solver = usize::from(split && !loaded);
            let x = (i % columns) as f64 * 1.1;
            let y = (i / columns) as f64 * 1.1;
            result.add(
                solver,
                i * 2,
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [0.5, 0.5, 0.1],
                    },
                    position: [x, y, 0.],
                    velocity: [0.25, 0., 0.],
                    mass: Some(1.),
                },
                true,
            )?;
            if loaded {
                result.add(
                    0,
                    i * 2 + 1,
                    BodySpec {
                        shape: Shape::Box {
                            half_extents: [0.1; 3],
                        },
                        position: [x, y, 0.201],
                        velocity: [0.; 3],
                        mass: Some(1.),
                    },
                    false,
                )?;
            }
        }
        Ok(result)
    }
    fn add(
        &mut self,
        solver: usize,
        global: usize,
        spec: BodySpec,
        prescribed: bool,
    ) -> Result<()> {
        let world = self.worlds[solver].world_mut();
        let mut resource = world.resource_mut::<PhysicsBackend>();
        let backend = resource.downcast_mut::<MultiFidelityBackend>().unwrap();
        let id = if prescribed {
            backend.spawn_kinematic_link(spec, Default::default(), None)?
        } else {
            backend.spawn_link(spec, Default::default(), None)?
        };
        let state = backend.state(id).unwrap();
        if solver == 0 {
            self.active_ids.insert(id, global);
        }
        self.actors
            .push((solver, world.spawn(Observed { id, state }).id()));
        Ok(())
    }
    fn step(&mut self) -> Result<()> {
        for simulation in &mut self.worlds {
            simulation.step_exact(1)?;
        }
        Ok(())
    }
    fn states(&self) -> impl Iterator<Item = BodyState> {
        self.actors.iter().map(|&(owner, entity)| {
            self.worlds[owner]
                .world()
                .get::<Observed>(entity)
                .unwrap()
                .state
        })
    }
    fn pairs(&self) -> BTreeMap<(usize, usize), f64> {
        let mut result = BTreeMap::new();
        let solver = self.worlds[0]
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        for p in solver
            .rigid_contact_observations()
            .into_iter()
            .filter(|p| p.active)
        {
            let a = self.active_ids[&p.a];
            let b = self.active_ids[&p.b];
            // Every actual contact must be one deck and its own payload.
            assert_eq!(a.min(b) % (STRIDE * 2), 0);
            assert_eq!(a.max(b) - a.min(b), 1);
            assert!(p.normal_impulse.is_finite() && p.normal_impulse >= 0.);
            assert!(
                result
                    .insert((a.min(b), a.max(b)), p.normal_impulse)
                    .is_none()
            );
        }
        result
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
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mode = args
        .next()
        .ok_or("expected unified, isolated, auto, verify or verify-auto")?;
    let count: usize = args.next().ok_or("expected deck count")?.parse()?;
    if !(1..=100_000).contains(&count) || args.next().is_some() {
        return Err("expected 1..=100000 decks".into());
    }
    if mode == "verify" || mode == "verify-auto" {
        let mut whole = Fixture::new(count, false, false)?;
        let mut split = Fixture::new(count, mode == "verify", mode == "verify-auto")?;
        assert_eq!(whole.actors.len(), count + count.div_ceil(STRIDE));
        assert_eq!(whole.actors.len(), split.actors.len());
        let mut maxima = [0.0f64; 5];
        let mut frame_pairs = 0;
        for _ in 0..WARMUP + STEPS {
            whole.step()?;
            split.step()?;
            for (a, b) in whole.states().zip(split.states()) {
                for (m, d) in maxima[..4].iter_mut().zip([
                    max_delta(a.position, b.position),
                    max_delta(a.orientation, b.orientation),
                    max_delta(a.velocity, b.velocity),
                    max_delta(a.angular_velocity, b.angular_velocity),
                ]) {
                    *m = m.max(d);
                }
            }
            let a = whole.pairs();
            let b = split.pairs();
            assert!(a.keys().eq(b.keys()));
            for (key, impulse) in &a {
                maxima[4] = maxima[4].max((impulse - b[key]).abs());
            }
            frame_pairs += a.len();
        }
        assert_eq!(whole.pairs().len(), count.div_ceil(STRIDE));
        println!(
            "ticks={} bodies={} compared_active_contacts={frame_pairs} max_position_m={:.6e} max_quaternion_component={:.6e} max_velocity_m_s={:.6e} max_angular_rad_s={:.6e} max_pair_impulse_ns={:.6e}",
            WARMUP + STEPS,
            whole.actors.len(),
            maxima[0],
            maxima[1],
            maxima[2],
            maxima[3],
            maxima[4]
        );
        assert!(
            maxima.iter().all(|&delta| delta < 1e-8),
            "partition residuals: {maxima:?}"
        );
        return Ok(());
    }
    let split = match mode.as_str() {
        "unified" | "auto" => false,
        "isolated" => true,
        _ => return Err("expected unified, isolated, auto, verify or verify-auto".into()),
    };
    let mut sim = Fixture::new(count, split, mode == "auto")?;
    for _ in 0..WARMUP {
        sim.step()?;
    }
    let mut ticks = Vec::new();
    let start = Instant::now();
    for _ in 0..STEPS {
        let t = Instant::now();
        sim.step()?;
        ticks.push(t.elapsed());
    }
    let wall = start.elapsed().as_secs_f64();
    ticks.sort_unstable();
    let contacts = sim.pairs().len();
    assert_eq!(contacts, count.div_ceil(STRIDE));
    assert_eq!(sim.states().count(), count + contacts);
    println!(
        "mode={mode} decks={count} contacts={contacts} warmup={WARMUP} measured={STEPS} wall_s={wall:.6} real_time_factor={:.5} p50_ms={:.4} p99_ms={:.4}",
        STEPS as f64 * DT.as_secs_f64() / wall,
        ticks[49].as_secs_f64() * 1000.,
        ticks[98].as_secs_f64() * 1000.
    );
    if mode == "auto" {
        let backend = sim.worlds[0]
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        println!("fleet_partition={:?}", backend.fleet_isolation_stats());
    }
    Ok(())
}
