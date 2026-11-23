// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the normal distributed grid and a full native world against free-fall,
//! zero horizontal motion, support load, momentum balance and settled geometry.
#[path = "support/stack_mechanics_native.rs"]
mod diagnostic;
#[path = "support/grid.rs"]
mod grid;
#[path = "support/grid_control.rs"]
mod grid_control;
#[path = "support/stack_mechanics_plot.rs"]
mod plot;
use clap::Parser;
use grid_control::{Control, Pairs, Supports};
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_distributed::{SceneBackend, coordinator::Frame, physics::Body};
use gz_sim_core::Simulation;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured};
use sdformat::NoResources;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Duration,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const G: f64 = 9.81;
const END_MS: u64 = 3000;
const WINDOW_MS: u64 = 500;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    output: PathBuf,
    /// Even box populations for the ordinary distributed scene. The native study uses 256 boxes.
    #[arg(long, value_delimiter = ',', default_value = "64,248,250,256")]
    bodies: Vec<usize>,
    #[arg(long, value_delimiter = ',', default_value = "1,2,5")]
    steps_ms: Vec<u64>,
    /// Run native-only isolated and loaded controls at the order threshold, with additional inner solver sweeps.
    #[arg(long)]
    native_study: bool,
}
fn frame(sim: &Simulation) -> &Frame {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
        .frame()
        .unwrap()
}
fn state(b: &Body) -> BodyState {
    BodyState {
        position: b.position,
        orientation: b.orientation,
        velocity: b.linear,
        angular_velocity: b.angular,
    }
}
fn norm(a: [f64; 3]) -> f64 {
    a[0].hypot(a[1]).hypot(a[2])
}

#[derive(Default, Clone, Copy)]
struct Mechanical {
    /// Deviations from the symmetric solution (initial XY axes and upright boxes).
    worst_xy: f64,
    worst_z: f64,
    worst_tilt: f64,
    worst_speed: f64,
    worst_spin: f64,
    mean_kinetic: f64,
    mean_energy: f64,
    floor_penetration: f64,
}
#[derive(Default)]
struct Stats {
    peak_xy: f64,
    peak_tilt: f64,
    peak_floor_penetration: f64,
    energy_gain: f64,
    peak_cell_momentum_gap: f64,
    peak_total_momentum_gap: f64,
    late_z: f64,
    late_speed: f64,
    late_spin: f64,
    late_ke: f64,
    late_floor: Vec<f64>,
    late_pair: Vec<f64>,
    cumulative_floor: Vec<f64>,
    last_unsettled_ms: u64,
    pair_ticks: Vec<u64>,
    latest: Mechanical,
}
impl Stats {
    fn new(cells: usize) -> Self {
        Self {
            late_floor: vec![0.; cells],
            late_pair: vec![0.; cells],
            cumulative_floor: vec![0.; cells],
            pair_ticks: vec![0; cells],
            ..Default::default()
        }
    }
    fn observe(
        &mut self,
        seeds: &[Body],
        states: &[BodyState],
        pairs: &Pairs,
        ground: &Supports,
        ms: u64,
    ) {
        let cells = seeds.len() / 2;
        let elapsed = ms as f64 / 1000.;
        let mut m = Mechanical::default();
        let mut total_gap = 0.;
        let mut present = 0;
        let mut energy_excess = 0.;
        for (cell, (seed, pair)) in seeds
            .chunks_exact(2)
            .zip(states.chunks_exact(2))
            .enumerate()
        {
            let floor: f64 = seed
                .iter()
                .map(|s| ground.get(&s.id).copied().unwrap_or(0.))
                .sum();
            let interface = pairs.get(&(seed[0].id, seed[1].id)).copied().unwrap_or(0.);
            if floor > 0. && interface > 0. {
                present += 1;
            }
            if interface > 0. && self.pair_ticks[cell] == 0 {
                self.pair_ticks[cell] = ms;
            }
            self.cumulative_floor[cell] += floor;
            let pz = seed
                .iter()
                .zip(pair)
                .map(|(s, b)| s.mass * b.velocity[2])
                .sum::<f64>();
            // On a horizontal floor its normal impulse is exactly vertical. Both
            // friction and every inter-box impulse cancel out of this equation.
            let residual = self.cumulative_floor[cell] - (3. * G * elapsed + pz);
            self.peak_cell_momentum_gap = self.peak_cell_momentum_gap.max(residual.abs());
            total_gap += residual;
            if ms > END_MS - WINDOW_MS {
                self.late_floor[cell] += floor;
                self.late_pair[cell] += interface;
            }
            for (layer, (s, b)) in seed.iter().zip(pair).enumerate() {
                let [x, y, z, w] = b.orientation;
                // Components of the rotated local Z direction and world Z in body axes.
                let axis = [
                    2. * (x * z + y * w),
                    2. * (y * z - x * w),
                    1. - 2. * (x * x + y * y),
                ];
                let projection = 0.275
                    * ((2. * (x * z - y * w)).abs() + (2. * (y * z + x * w)).abs())
                    + 0.20 * axis[2].abs();
                m.floor_penetration = m.floor_penetration.max(projection - b.position[2]);
                m.worst_xy = m
                    .worst_xy
                    .max((b.position[0] - s.position[0]).hypot(b.position[1] - s.position[1]));
                m.worst_z = m
                    .worst_z
                    .max((b.position[2] - (0.2 + 0.4 * layer as f64)).abs());
                m.worst_tilt = m.worst_tilt.max(axis[0].hypot(axis[1]).atan2(axis[2]));
                let speed = norm(b.velocity);
                let spin = norm(b.angular_velocity);
                m.worst_speed = m.worst_speed.max(speed);
                m.worst_spin = m.worst_spin.max(spin);
                let axial_spin: f64 = b
                    .angular_velocity
                    .iter()
                    .zip(axis)
                    .map(|(a, b)| a * b)
                    .sum();
                let transverse = s.mass * (0.55f64.powi(2) + 0.4f64.powi(2)) / 12.;
                let axial = s.mass * 0.55f64.powi(2) / 6.;
                let kinetic = 0.5
                    * (s.mass * speed * speed
                        + transverse * spin * spin
                        + (axial - transverse) * axial_spin * axial_spin);
                m.mean_kinetic += kinetic;
                m.mean_energy += kinetic + s.mass * G * b.position[2];
                energy_excess += kinetic + s.mass * G * (b.position[2] - s.position[2]);
            }
        }
        m.mean_kinetic /= cells as f64;
        m.mean_energy /= cells as f64;
        self.peak_xy = self.peak_xy.max(m.worst_xy);
        self.peak_tilt = self.peak_tilt.max(m.worst_tilt);
        self.energy_gain = self.energy_gain.max(energy_excess / cells as f64);
        self.peak_floor_penetration = self.peak_floor_penetration.max(m.floor_penetration);
        self.peak_total_momentum_gap = self.peak_total_momentum_gap.max(total_gap.abs());
        if !(present == cells && m.worst_speed < 0.01 && m.worst_spin < 0.1 && m.worst_z < 0.005) {
            self.last_unsettled_ms = ms;
        }
        if ms > END_MS - WINDOW_MS {
            self.late_z = self.late_z.max(m.worst_z);
            self.late_speed = self.late_speed.max(m.worst_speed);
            self.late_spin = self.late_spin.max(m.worst_spin);
            self.late_ke = self.late_ke.max(m.mean_kinetic);
        }
        self.latest = m;
    }
    fn mean_load(impulses: &[f64]) -> f64 {
        impulses.iter().sum::<f64>() / (impulses.len() as f64 * WINDOW_MS as f64 / 1000.)
    }
    fn load_error(impulses: &[f64], ideal: f64) -> f64 {
        impulses
            .iter()
            .map(|n| (n * 1000. / WINDOW_MS as f64 - ideal).abs())
            .fold(0., f64::max)
    }
}

#[derive(Clone)]
struct Row {
    count: usize,
    step: u64,
    solver: &'static str,
    peak_xy: f64,
    late_z: f64,
    settle: u64,
    floor_load_error: f64,
    pair_load_error: f64,
    peak_delta: f64,
}
struct Outputs {
    metrics: BufWriter<File>,
    trace: BufWriter<File>,
    rows: Vec<Row>,
}
fn run(worker: &Path, count: usize, hms: u64, output: &mut Outputs) -> Result<()> {
    let h = Duration::from_millis(hms);
    let mut sim = load_world_configured(
        &grid::sdf(count),
        "grid.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: Some(h),
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?;
    let seeds = (0..count)
        .map(|i| frame(&sim).body(i as u64).unwrap().1.clone())
        .collect::<Vec<_>>();
    let mut native = Control::new(&seeds, h)?;
    let mut statistics = [Stats::new(count / 2), Stats::new(count / 2)];
    let mut first = None;
    let mut peak_delta = 0f64;
    let mut worst_late_delta = 0f64;
    let mut observed = vec![state(&seeds[0]); count];
    let mut reference = observed.clone();
    for tick in 1..=END_MS / hms {
        sim.step_exact(1).map_err(|e| {
            format!(
                "{count} bodies {hms} ms at tick {tick}: {e} ({:?})",
                sim.world().resource::<gz_sim_core::StepFault>().error()
            )
        })?;
        native.step(h)?;
        let f = frame(&sim);
        let mut exact = true;
        let mut delta = 0f64;
        for b in f.workers.iter().flat_map(|w| &w.snapshot.bodies) {
            let id = b.id as usize;
            let n = native.state(id);
            observed[id] = state(b);
            reference[id] = n;
            if !grid_control::exact(b, n) {
                exact = false;
                delta = delta.max(grid_control::state_error(b, n)[0]);
            }
        }
        let floor = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.supports)
            .map(|s| (s.body, s.normal_impulse))
            .collect::<Supports>();
        let pairs = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(|s| ((s.a, s.b), s.normal_impulse))
            .collect::<Pairs>();
        let mut native_pairs = Pairs::new();
        let mut native_floor = Supports::new();
        native.contacts(&mut native_pairs, &mut native_floor);
        exact &= grid_control::impulses(&floor, &native_floor).0
            && grid_control::impulses(&pairs, &native_pairs).0;
        if !exact {
            first.get_or_insert(tick * hms);
        }
        peak_delta = peak_delta.max(delta);
        let ms = tick * hms;
        if ms > END_MS - WINDOW_MS {
            worst_late_delta = worst_late_delta.max(delta);
        }
        for (i, (states, pairs, ground)) in [
            (&observed, &pairs, &floor),
            (&reference, &native_pairs, &native_floor),
        ]
        .into_iter()
        .enumerate()
        {
            let stats = &mut statistics[i];
            stats.observe(&seeds, states, pairs, ground, ms);
            if ms.is_multiple_of(10) {
                let m = stats.latest;
                writeln!(
                    output.trace,
                    "{count},{hms},{},{ms},{:.9e},{:.9e},{:.9e},{:.9e},{:.9e},{:.9e},{delta:.9e}",
                    ["distributed", "monolithic"][i],
                    m.worst_xy,
                    m.worst_z,
                    m.worst_speed,
                    m.mean_kinetic,
                    m.mean_energy,
                    m.floor_penetration
                )?;
            }
        }
    }
    for (solver, s) in ["distributed", "monolithic"].into_iter().zip(statistics) {
        let floor_err = Stats::load_error(&s.late_floor, 3. * G);
        let pair_err = Stats::load_error(&s.late_pair, 2. * G);
        let settle = s.last_unsettled_ms + hms;
        let first_error = seeds
            .chunks_exact(2)
            .zip(&s.pair_ticks)
            .map(|(pair, hit)| {
                let impact = (2. * (pair[1].position[2] - 0.6) / G).sqrt();
                (impact - (*hit as f64 / 1000.)).abs()
            })
            .fold(0., f64::max);
        writeln!(
            output.metrics,
            "{count},{hms},{solver},{},{peak_delta:.12e},{worst_late_delta:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{settle},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{floor_err:.12e},{:.12e},{pair_err:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{first_error:.12e}",
            first.unwrap_or(0),
            s.peak_xy,
            s.peak_tilt,
            s.peak_floor_penetration,
            s.late_z,
            s.late_speed,
            s.late_spin,
            s.late_ke,
            s.latest.mean_energy,
            Stats::mean_load(&s.late_floor),
            Stats::mean_load(&s.late_pair),
            s.peak_cell_momentum_gap,
            s.peak_total_momentum_gap,
            s.energy_gain,
            s.latest.worst_xy
        )?;
        // Gravity and ideal horizontal symmetry are independent of layout or population.
        assert!(s.pair_ticks.iter().all(|&n| n > 0));
        assert!(s.energy_gain < 1e-8, "energy above release");
        assert!(s.peak_cell_momentum_gap < 1e-8 && s.peak_total_momentum_gap < 1e-7);
        assert!(settle <= END_MS - WINDOW_MS && s.late_z < 0.005);
        assert!(floor_err < 0.01 && pair_err < 0.01, "settled support loads");
        output.rows.push(Row {
            count,
            step: hms,
            solver,
            peak_xy: s.peak_xy,
            late_z: s.late_z,
            settle,
            floor_load_error: floor_err,
            pair_load_error: pair_err,
            peak_delta,
        });
    }
    output.metrics.flush()?;
    output.trace.flush()?;
    eprintln!(
        "{count} bodies {hms} ms: first {} ms, peak position difference {peak_delta:.8} m",
        first.unwrap_or(0)
    );
    Ok(())
}
fn main() -> Result<()> {
    let args = Args::parse();
    if !args.steps_ms.iter().all(|h| [1, 2, 5, 10].contains(h)) {
        return Err("steps-ms accepts 1, 2, 5 and 10".into());
    }
    if !args
        .bodies
        .iter()
        .all(|&n| (2..=256).contains(&n) && n.is_multiple_of(2))
    {
        return Err("bodies must be an even number between 2 and 256".into());
    }
    std::fs::create_dir_all(&args.output)?;
    if args.native_study {
        return diagnostic::study(&args.worker, &args.output, &args.steps_ms);
    }
    let mut out = Outputs {
        metrics: BufWriter::new(File::create(args.output.join("metrics.csv"))?),
        trace: BufWriter::new(File::create(args.output.join("trace.csv"))?),
        rows: vec![],
    };
    writeln!(
        out.metrics,
        "bodies,step_ms,solver,first_difference_ms,peak_position_difference_m,late_position_difference_m,peak_horizontal_m,peak_tilt_rad,peak_floor_penetration_m,late_z_error_m,settled_from_ms,late_speed_m_s,late_spin_rad_s,late_mean_kinetic_J_stack,final_mean_mechanical_energy_J_stack,late_mean_floor_N,late_worst_floor_error_N,late_mean_interface_N,late_worst_interface_error_N,max_cell_vertical_momentum_residual_Ns,max_total_vertical_momentum_residual_Ns,max_energy_above_seed_J_stack,final_horizontal_m,max_top_impact_time_error_s"
    )?;
    writeln!(
        out.trace,
        "bodies,step_ms,solver,t_ms,max_horizontal_m,max_z_error_m,max_speed_m_s,mean_kinetic_J_stack,mean_energy_J_stack,max_floor_penetration_m,max_position_difference_m"
    )?;
    for n in args.bodies {
        for &h in &args.steps_ms {
            run(&args.worker, n, h, &mut out)?;
        }
    }
    plot::write(&out.rows, &args.output.join("mechanics.svg"))?;
    Ok(())
}
