// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare Rapier contact settings after a common loaded, discrete 10 ms trajectory.
#[path = "support/seeded.rs"]
mod seeded;
#[path = "support/train_fixture.rs"]
mod train_fixture;

use clap::Parser;
use gz_physics_distributed::physics::{Body, BodyWrench, Island, SimulationStep, Snapshot};
use rapier3d_f64::{geometry::NEW_CONTACT_BIT, parry::query, prelude::*};
use serde::Serialize;
use std::{error::Error, io::Write};
use train_fixture::train_fixture;

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Debug, Clone, Copy, Serialize)]
enum Mode {
    Discrete,
    Soft25mm,
    Soft100mm,
    Swept,
    SweptFour,
    SubstepUs(u32),
    Predictive,
}

impl Mode {
    fn apply(self, world: &mut PhysicsWorld) {
        match self {
            Self::Discrete | Self::Predictive => {}
            Self::Soft25mm | Self::Soft100mm => {
                let distance = if matches!(self, Self::Soft25mm) {
                    0.025
                } else {
                    0.1
                };
                for (_, body) in world.bodies.iter_mut() {
                    body.set_soft_ccd_prediction(distance);
                }
            }
            Self::Swept | Self::SweptFour => {
                if matches!(self, Self::SweptFour) {
                    world.integration_parameters.max_ccd_substeps = 4;
                }
                for (_, body) in world.bodies.iter_mut() {
                    body.enable_ccd(true);
                }
            }
            Self::SubstepUs(us) => {
                world.integration_parameters.dt =
                    std::time::Duration::from_micros(us.into()).as_secs_f64()
            }
        }
    }
}

struct Native {
    world: PhysicsWorld,
    handles: Vec<RigidBodyHandle>,
}

impl Native {
    fn new(bodies: &[Body]) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world.integration_parameters.dt = 0.01;
        world.integration_parameters.contact_recycling = false;
        let mut handles = Vec::new();
        for body in bodies {
            let half = Vector::from(body.size) * 0.5;
            let rigid = RigidBodyBuilder::dynamic()
                .can_sleep(false)
                .pose(Pose::from_parts(
                    Vector::from(body.position),
                    Rotation::from_array(body.orientation),
                ))
                .linvel(Vector::from(body.linear))
                .angvel(Vector::from(body.angular));
            let collider = ColliderBuilder::cuboid(half.x, half.y, half.z)
                .mass(body.mass)
                .friction(body.friction);
            let h = world.insert(rigid, collider).0;
            world.bodies[h].recompute_mass_properties_from_colliders(&world.colliders);
            handles.push(h);
        }
        Self { world, handles }
    }

    fn check(&self, expected: &[Body]) -> Result<()> {
        for (state, &handle) in expected.iter().zip(&self.handles) {
            let b = &self.world.bodies[handle];
            if b.translation().to_array().map(f64::to_bits) != state.position.map(f64::to_bits)
                || b.rotation().to_array().map(f64::to_bits) != state.orientation.map(f64::to_bits)
                || b.linvel().to_array().map(f64::to_bits) != state.linear.map(f64::to_bits)
                || b.angvel().to_array().map(f64::to_bits) != state.angular.map(f64::to_bits)
            {
                return Err(format!(
                    "independent Rapier prehistory mismatch on body {}",
                    state.id
                )
                .into());
            }
        }
        Ok(())
    }

    fn left_momentum(&self, bodies: &[Body]) -> Vector {
        (0..2)
            .map(|i| self.world.bodies[self.handles[i]].linvel() * bodies[i].mass)
            .sum()
    }

    fn advance(&mut self, inputs: &[BodyWrench]) {
        for w in inputs {
            let b = &mut self.world.bodies[self.handles[w.id as usize - 1]];
            b.add_force(Vector::from(w.force), true);
            b.add_torque(Vector::from(w.torque), true);
        }
        self.world.step();
        for w in inputs {
            let b = &mut self.world.bodies[self.handles[w.id as usize - 1]];
            b.reset_forces(false);
            b.reset_torques(false);
        }
    }

    fn cross_impulse(&self) -> f64 {
        let mut impulse = 0.;
        for &a in &self.handles[..2] {
            for &b in &self.handles[2..] {
                let handles = [a, b].map(|h| self.world.bodies[h].colliders()[0]);
                if let Some(pair) = self.world.narrow_phase.contact_pair(handles[0], handles[1]) {
                    for manifold in pair.solver_manifolds() {
                        for (i, p) in manifold.points.iter().enumerate() {
                            if manifold
                                .data
                                .solver_contacts
                                .iter()
                                .any(|row| (row.contact_id[0] & !NEW_CONTACT_BIT) as usize == i)
                            {
                                impulse += p.data.impulse;
                            }
                        }
                    }
                }
            }
        }
        impulse
    }

    fn read_bodies(&self, bodies: &mut [Body]) -> Result<()> {
        for (state, &handle) in bodies.iter_mut().zip(&self.handles) {
            let rigid = &self.world.bodies[handle];
            let collider = &self.world.colliders[rigid.colliders()[0]];
            let cuboid = collider
                .shape()
                .as_cuboid()
                .ok_or("train requires a cuboid")?;
            state.size = (cuboid.half_extents * 2.).to_array();
            state.position = collider.translation().to_array();
            state.orientation = collider.rotation().to_array();
            state.linear = rigid.linvel().to_array();
            state.angular = rigid.angvel().to_array();
        }
        Ok(())
    }

    /// Only the four opposing-train pairs participate. Stored loads and future
    /// contact impulses are not included in this constant-twist test.
    fn approaching_train(&self, bodies: &mut [Body]) -> Result<bool> {
        self.read_bodies(bodies)?;
        for left in &bodies[..2] {
            for right in &bodies[2..] {
                if left.predicts_box_contact(right, SimulationStep::Ms10)? {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    fn gap(&self) -> Result<f64> {
        let mut gap = f64::INFINITY;
        for &a in &self.handles[..2] {
            for &b in &self.handles[2..] {
                let [ca, cb] =
                    [a, b].map(|h| &self.world.colliders[self.world.bodies[h].colliders()[0]]);
                let distance =
                    query::distance(ca.position(), ca.shape(), cb.position(), cb.shape())?.distance;
                let signed = if distance == 0. {
                    query::contact(ca.position(), ca.shape(), cb.position(), cb.shape(), 0.)?
                        .map_or(0., |c| c.dist)
                } else {
                    distance
                };
                gap = gap.min(signed);
            }
        }
        Ok(gap)
    }
}

#[derive(Serialize)]
struct Report {
    seed: u64,
    mode: Mode,
    prescribed_fork_after_tick: Option<usize>,
    first_refined_tick: Option<usize>,
    refined_public_intervals: usize,
    native_solves: usize,
    exact_control_ticks: usize,
    first_positive_end_s: Option<f64>,
    min_gap_m: f64,
    peak_substep_cross_impulse_ns: f64,
    summed_cross_impulse_ns: f64,
    net_left_contact_impulse_ns: [f64; 3],
    peak_substep_left_momentum_residual_ns: f64,
}

#[derive(Serialize)]
struct PublicFrame<'a> {
    tick: usize,
    time_s: f64,
    bodies: &'a [Body],
    gap_m: f64,
    impulse_ns: f64,
    substeps: u32,
}

fn trial(
    seed: u64,
    fork_after_tick: usize,
    mode: Mode,
    mut trace: Option<&mut dyn Write>,
) -> Result<Report> {
    let (bodies, inputs) = train_fixture(seed);
    let mut native = Native::new(&bodies);
    let mut expected = Island::restore(&Snapshot {
        bodies: bodies.clone(),
        ..Default::default()
    })?;
    let mut report = Report {
        seed,
        mode,
        prescribed_fork_after_tick: (!matches!(mode, Mode::Predictive)).then_some(fork_after_tick),
        first_refined_tick: None,
        refined_public_intervals: 0,
        native_solves: 0,
        exact_control_ticks: 0,
        first_positive_end_s: None,
        min_gap_m: f64::INFINITY,
        peak_substep_cross_impulse_ns: 0.,
        summed_cross_impulse_ns: 0.,
        net_left_contact_impulse_ns: [0.; 3],
        peak_substep_left_momentum_residual_ns: 0.,
    };
    let mut prediction_bodies = bodies.clone();
    let initial_momentum = native.left_momentum(&bodies);
    for tick in 1..=670 {
        if !matches!(mode, Mode::Predictive) && tick == fork_after_tick + 1 {
            mode.apply(&mut native.world);
        }
        let steps = match mode {
            Mode::Predictive => {
                if native.approaching_train(&mut prediction_bodies)? {
                    native.world.integration_parameters.dt = 0.001;
                    10
                } else {
                    native.world.integration_parameters.dt = 0.01;
                    1
                }
            }
            Mode::SubstepUs(us) if tick > fork_after_tick => 10_000 / us,
            _ => 1,
        };
        if steps > 1 {
            report.first_refined_tick.get_or_insert(tick);
            report.refined_public_intervals += 1;
        }
        report.native_solves += steps as usize;
        let dt = native.world.integration_parameters.dt;
        let mut public_impulse = 0.;
        for i in 1..=steps {
            let before = native.left_momentum(&bodies);
            native.advance(&inputs);
            let impulse = native.cross_impulse();
            public_impulse += impulse;
            report.summed_cross_impulse_ns += impulse;
            let residual =
                (native.left_momentum(&bodies) - before - Vector::from(inputs[0].force) * dt)
                    .length();
            report.peak_substep_cross_impulse_ns =
                report.peak_substep_cross_impulse_ns.max(impulse);
            report.peak_substep_left_momentum_residual_ns =
                report.peak_substep_left_momentum_residual_ns.max(residual);
            report.min_gap_m = report.min_gap_m.min(native.gap()?);
            if impulse > 1e-5 {
                report
                    .first_positive_end_s
                    .get_or_insert((tick - 1) as f64 * 0.01 + i as f64 * dt);
            }
        }
        if let Some(out) = trace.as_deref_mut() {
            native.read_bodies(&mut prediction_bodies)?;
            serde_json::to_writer(
                &mut *out,
                &PublicFrame {
                    tick,
                    time_s: tick as f64 * 0.01,
                    bodies: &prediction_bodies,
                    gap_m: native.gap()?,
                    impulse_ns: public_impulse,
                    substeps: steps,
                },
            )?;
            writeln!(out)?;
        }
        let coarse_control = match mode {
            Mode::Predictive => report.first_refined_tick.is_none(),
            Mode::Discrete => true,
            _ => tick <= fork_after_tick,
        };
        if coarse_control {
            expected.step_with_wrenches(&inputs)?;
            native.check(&expected.snapshot().bodies)?;
            report.exact_control_ticks += 1;
        }
    }
    report.net_left_contact_impulse_ns =
        (native.left_momentum(&bodies) - initial_momentum - Vector::from(inputs[0].force) * 6.70)
            .to_array();
    Ok(report)
}

const CASES: [(u64, usize); 6] = [
    (20261161424, 237),
    (20261184268, 234),
    (20261199941, 235),
    (20261120100, 194),
    (20261122691, 237),
    (20261186018, 268),
];

#[derive(Clone, Copy, Serialize)]
enum BenchMode {
    Coarse,
    Predictive,
    Fine,
}

#[derive(Serialize)]
struct Timing {
    seed: u64,
    mode: BenchMode,
    native_solves: usize,
    elapsed_ns: Vec<u64>,
    median_ns: f64,
    query_only_ns: Vec<u64>,
    median_query_only_ns: Option<f64>,
}

struct Run {
    total_ns: u64,
    query_ns: u64,
    solves: usize,
}

fn measure(seed: u64, mode: BenchMode, time_queries: bool) -> Result<Run> {
    use std::time::{Duration, Instant};
    let (mut bodies, inputs) = train_fixture(seed);
    let mut native = Native::new(&bodies);
    let mut query_time = Duration::ZERO;
    let mut solves = 0;
    let clock = Instant::now();
    for _ in 0..670 {
        let refine = match mode {
            BenchMode::Coarse => false,
            BenchMode::Fine => true,
            BenchMode::Predictive => {
                let start = time_queries.then(Instant::now);
                let decision = native.approaching_train(&mut bodies)?;
                if let Some(start) = start {
                    query_time += start.elapsed();
                }
                decision
            }
        };
        native.world.integration_parameters.dt = if refine { 0.001 } else { 0.01 };
        let steps = if refine { 10 } else { 1 };
        for _ in 0..steps {
            native.advance(&inputs);
        }
        solves += steps;
    }
    let elapsed = clock.elapsed();
    for &h in &native.handles {
        let b = &native.world.bodies[h];
        std::hint::black_box((b.position(), b.linvel(), b.angvel()));
    }
    Ok(Run {
        total_ns: elapsed.as_nanos().try_into()?,
        query_ns: query_time.as_nanos().try_into()?,
        solves,
    })
}

fn median(samples: &[u64]) -> f64 {
    let mut values = samples.to_owned();
    values.sort_unstable();
    (values[(values.len() - 1) / 2] as f64 + values[values.len() / 2] as f64) * 0.5
}

fn benchmark() -> Result<()> {
    let modes = [BenchMode::Coarse, BenchMode::Predictive, BenchMode::Fine];
    let orders = [
        [0, 1, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
        [1, 0, 2],
        [0, 2, 1],
    ];
    for (seed, _) in CASES {
        let mut rows = modes.map(|mode| Timing {
            seed,
            mode,
            native_solves: 0,
            elapsed_ns: Vec::new(),
            median_ns: 0.,
            query_only_ns: Vec::new(),
            median_query_only_ns: None,
        });
        for mode in modes {
            measure(seed, mode, false)?;
        }
        for order in orders {
            for ix in order {
                let run = measure(seed, modes[ix], false)?;
                rows[ix].elapsed_ns.push(run.total_ns);
                rows[ix].native_solves = run.solves;
            }
            // Separate instrumented replay keeps per-query clock calls out of
            // the competing whole-run times. It includes pose and shape reads.
            let query_run = measure(seed, BenchMode::Predictive, true)?;
            rows[1].query_only_ns.push(query_run.query_ns);
        }
        for mut row in rows {
            row.median_ns = median(&row.elapsed_ns);
            row.median_query_only_ns =
                (!row.query_only_ns.is_empty()).then(|| median(&row.query_only_ns));
            println!("{}", serde_json::to_string(&row)?);
        }
    }
    Ok(())
}

#[derive(Parser)]
struct Args {
    /// Time six interleaved repetitions of each stepping mode on the six train fixtures.
    #[arg(long)]
    benchmark: bool,
    /// Emit every public barrier of the prospective run for this seed as JSON lines.
    #[arg(long, conflicts_with = "benchmark")]
    trace_adaptive: Option<u64>,
}

fn main() -> Result<()> {
    let args = Args::parse();
    if let Some(seed) = args.trace_adaptive {
        let mut out = std::io::BufWriter::new(std::io::stdout().lock());
        trial(seed, 0, Mode::Predictive, Some(&mut out))?;
        out.flush()?;
        return Ok(());
    }
    if args.benchmark {
        return benchmark();
    }
    // Fork at the last discrete barrier before the first sampled overlap.
    // The distant control branches alongside seed 61424. The near miss branches
    // two ticks before its closest sampled approach (6.107 mm at tick 270).
    for (seed, fork) in CASES {
        for mode in [
            Mode::Discrete,
            Mode::Soft25mm,
            Mode::Soft100mm,
            Mode::Swept,
            Mode::SweptFour,
            Mode::SubstepUs(1000),
            Mode::SubstepUs(500),
            Mode::SubstepUs(250),
            Mode::SubstepUs(125),
            Mode::Predictive,
        ] {
            println!(
                "{}",
                serde_json::to_string(&trial(seed, fork, mode, None)?)?
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prospective_refinement_detects_grazes_and_respects_clearance() {
        for (seed, fork) in [(20261161424, 237), (20261184268, 234), (20261199941, 235)] {
            // An adaptive run neither reads a fork tick nor sees the reference impacts.
            let adaptive = trial(seed, 0, Mode::Predictive, None).unwrap();
            let fine = trial(seed, fork, Mode::SubstepUs(125), None).unwrap();
            let coarse = trial(seed, 0, Mode::Discrete, None).unwrap();
            assert_eq!(coarse.first_positive_end_s, None);
            assert_eq!(adaptive.exact_control_ticks, fork);
            assert!(adaptive.first_positive_end_s.is_some());
            assert!(adaptive.min_gap_m > -0.0001);
            assert!(adaptive.refined_public_intervals < 20);
            assert_eq!(
                adaptive.native_solves,
                670 + 9 * adaptive.refined_public_intervals
            );
            let vector_error = Vector::from(adaptive.net_left_contact_impulse_ns)
                .distance(Vector::from(fine.net_left_contact_impulse_ns));
            assert!(vector_error < 0.02, "seed {seed}: {vector_error} N s");
        }

        for (seed, distant) in [(20261122691, true), (20261186018, false)] {
            let adaptive = trial(seed, 0, Mode::Predictive, None).unwrap();
            assert!(adaptive.first_positive_end_s.is_none());
            assert!(adaptive.min_gap_m > 0.);
            assert_eq!(adaptive.refined_public_intervals == 0, distant);
            assert_eq!(adaptive.peak_substep_cross_impulse_ns, 0.);
        }
    }
}
