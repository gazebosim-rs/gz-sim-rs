// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sample the stock world and measure timestep sensitivity and lower-link load limits.
#[path = "support/pendulum.rs"]
mod pendulum;

use pendulum::{Assembly, Result, STOCK, Sample, load, with_lower_inertial};
use serde::Serialize;
use serde_json::{Value, json};
use std::{
    f64::consts::{PI, TAU},
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
    time::{Duration, Instant},
};

type History = Vec<[Sample; 2]>;

fn continuous(previous: f64, current: f64) -> f64 {
    current + TAU * ((previous - current) / TAU).round()
}

fn run(xml: &str, step_us: u64, mass: f64, inertia: f64, stiff: bool) -> Result<History> {
    let mut sim = load(xml, stiff)?;
    let dt = Duration::from_micros(step_us);
    if sim.step_duration() != dt {
        return Err("loaded clock differs from requested grid".into());
    }
    let assemblies = [Assembly::new(&mut sim, 0)?, Assembly::new(&mut sim, 1)?];
    let mut history: History = Vec::with_capacity(601);
    for index in 0..=600 {
        if index > 0 {
            sim.step_exact(10_000 / step_us)?;
        }
        if sim.elapsed() != Duration::from_millis(index * 10) {
            return Err("unexpected committed observation time".into());
        }
        let mut row = [
            assemblies[0].sample(&sim, mass, inertia)?,
            assemblies[1].sample(&sim, mass, inertia)?,
        ];
        if let Some(previous) = history.last() {
            for (new, old) in row.iter_mut().zip(previous) {
                new.upper_q = continuous(old.upper_q, new.upper_q);
                new.lower_q = continuous(old.lower_q, new.lower_q);
            }
        }
        history.push(row);
    }
    Ok(history)
}

#[derive(Clone, Serialize)]
struct Summary {
    max_energy_drift_fraction: f64,
    max_anchor_error_m: f64,
    min_base_height_m: f64,
    initial_energy_j: f64,
    descending_crossings_s: Vec<f64>,
    first_return_period_s: f64,
}

impl Summary {
    fn stable(&self) -> bool {
        self.max_energy_drift_fraction < 0.05 && self.max_anchor_error_m < 0.005
    }
}

fn summary(h: &History) -> Result<Vec<Summary>> {
    let mut models = vec![];
    for model in 0..2 {
        let initial = h[0][model].total_j;
        let mut max_drift: f64 = 0.;
        let mut max_anchor: f64 = 0.;
        let mut min_base = h[0][model].base_z_m;
        let mut downward = Vec::new();
        // Relative to the SDF release angle, a downward link is at -π + 1.5708.
        let level = -PI - h[0][model].upper_roll;
        for (i, pair) in h.windows(2).enumerate() {
            let (a, b) = (pair[0][model], pair[1][model]);
            max_drift = max_drift.max((b.total_j - initial).abs() / initial.abs());
            min_base = min_base.min(b.base_z_m);
            max_anchor = max_anchor.max(b.anchor_error_m);
            if a.upper_q > level && b.upper_q <= level {
                downward.push((i as f64 + (level - a.upper_q) / (b.upper_q - a.upper_q)) * 0.01);
            }
        }
        let period = downward
            .windows(2)
            .next()
            .ok_or_else(|| format!("no complete upper swing, crossings {downward:?}"))?;
        models.push(Summary {
            max_energy_drift_fraction: max_drift,
            max_anchor_error_m: max_anchor,
            min_base_height_m: min_base,
            initial_energy_j: initial,
            first_return_period_s: period[1] - period[0],
            descending_crossings_s: downward,
        });
    }
    Ok(models)
}

fn max_difference(a: &History, b: &History) -> [f64; 3] {
    let mut largest = [0.0_f64; 3];
    for (coarse, fine) in a.iter().flatten().zip(b.iter().flatten()) {
        for (limit, difference) in largest.iter_mut().zip([
            coarse.upper_q - fine.upper_q,
            coarse.lower_q - fine.lower_q,
            coarse.total_j - fine.total_j,
        ]) {
            *limit = limit.max(difference.abs());
        }
    }
    largest
}

fn write_csv(path: PathBuf, h: &History) -> Result<()> {
    let mut out = BufWriter::new(File::create(path)?);
    writeln!(
        out,
        "tick,model,upper_q,lower_q,upper_dq,lower_dq,upper_roll,lower_roll,upper_com_x,upper_com_y,upper_com_z,lower_com_x,lower_com_y,lower_com_z,potential_j,kinetic_j,total_j"
    )?;
    for s in h.iter().flatten() {
        write!(out, "{},{}", s.tick, s.model)?;
        for x in [
            s.upper_q,
            s.lower_q,
            s.upper_dq,
            s.lower_dq,
            s.upper_roll,
            s.lower_roll,
        ]
        .into_iter()
        .chain(s.upper_com)
        .chain(s.lower_com)
        .chain([s.potential_j, s.kinetic_j, s.total_j])
        {
            write!(out, ",{x:.17}")?;
        }
        writeln!(out)?;
    }
    out.flush()?;
    Ok(())
}

#[derive(Serialize)]
struct Survey {
    nominal_models: Vec<Summary>,
    timestep_sweep: Vec<Value>,
    adjacent_grids: Vec<Value>,
    load_sweep: Vec<Value>,
    stable_within_5pct_energy_and_5mm_anchor: bool,
    all_grid_angle_differences_contract: bool,
}

fn survey(nominal: &History, stiff: bool) -> Result<Survey> {
    let nominal_models = summary(nominal)?;
    let mut grids = vec![];
    let mut stable = true;
    let mut previous = None;
    let mut differences = Vec::new();
    let mut pair_metrics = vec![];
    for step_us in [2000, 1000, 500, 250, 125] {
        let h = if step_us == 1000 {
            nominal.clone()
        } else {
            let xml = STOCK.replace(
                "<max_step_size>0.001</max_step_size>",
                &format!("<max_step_size>{}</max_step_size>", step_us as f64 / 1e6),
            );
            run(&xml, step_us, 1., 1., stiff)?
        };
        let models = summary(&h)?;
        stable &= models.iter().all(Summary::stable);
        grids.push(json!({"step_us":step_us,"models":models}));
        if let Some(prev) = previous {
            let delta = max_difference(&prev, &h);
            pair_metrics.push(json!({"coarse_us":step_us*2, "fine_us":step_us,
                "upper_max_rad":delta[0],"lower_max_rad":delta[1],"energy_max_j":delta[2]}));
            differences.push(delta);
        }
        previous = Some(h);
    }
    // The nominal 1 kg / 1 kg m² lower member is covered by the unchanged stock run.
    let mut loads = vec![];
    for mass in [0.5, 1., 2.] {
        for inertia in [0.5, 1., 2.] {
            let models = if mass == 1. && inertia == 1. {
                nominal_models.clone()
            } else {
                let xml = with_lower_inertial(&format!(
                    "<mass>{mass}</mass><inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz></inertia>"
                ))?;
                summary(&run(&xml, 1000, mass, inertia, stiff)?)?
            };
            stable &= models.iter().all(Summary::stable);
            loads.push(json!({"lower_mass_kg":mass,"lower_isotropic_inertia_kg_m2":inertia,"models":models}));
        }
    }
    let angles_contract = |pair: &[[f64; 3]]| pair[1][0] < pair[0][0] && pair[1][1] < pair[0][1];
    let monotonic = differences.windows(2).all(angles_contract);
    Ok(Survey {
        nominal_models,
        timestep_sweep: grids,
        adjacent_grids: pair_metrics,
        load_sweep: loads,
        stable_within_5pct_energy_and_5mm_anchor: stable,
        all_grid_angle_differences_contract: monotonic,
    })
}

fn main() -> Result<()> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("pass an output directory")?,
    );
    fs::create_dir_all(&dir)?;
    let start = Instant::now();
    let stock = run(STOCK, 1000, 1., 1., false)?;
    write_csv(dir.join("rust.csv"), &stock)?;
    let stiff = run(STOCK, 1000, 1., 1., true)?;
    write_csv(dir.join("rust-stiff.csv"), &stiff)?;
    let baseline = survey(&stock, false)?;
    let qualified = survey(&stiff, true)?;
    let passed = qualified.stable_within_5pct_energy_and_5mm_anchor
        && qualified.all_grid_angle_differences_contract;
    let report = json!({"engine":"multifidelity", "seconds":6, "sample_ms":10, "default_60hz":baseline,
        "stiff_static_contacts_120hz":qualified, "elapsed_wall_s":start.elapsed().as_secs_f64()});
    fs::write(
        dir.join("qualification.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    if !passed {
        return Err("120 Hz load stability or timestep convergence gate failed".into());
    }
    Ok(())
}
