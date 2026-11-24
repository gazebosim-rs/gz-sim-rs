// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify the vertical momentum of a free parcel from the recorded pad and receiver reactions.
use clap::Parser;
use serde::{Deserialize, Serialize};
use std::{
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
#[command(about)]
struct Args {
    /// Recording directories produced by local_regrasp or regrasp_setdown.
    #[arg(required = true)]
    recordings: Vec<PathBuf>,
    /// Optional upper bound on the absolute momentum residual per measured interval, in N·s.
    #[arg(long)]
    max_error: Option<f64>,
}

#[derive(Deserialize)]
struct Run {
    parcel_mass_kg: f64,
    #[serde(alias = "dt_ns")]
    public_dt_ns: u64,
}

#[derive(Deserialize)]
struct Sample {
    time_ns: u64,
    parcel_vz_m_s: f64,
    pad_vertical_force_n: [f64; 2],
    #[serde(default)]
    receiver_vertical_force_n: f64,
    #[serde(default)]
    other_vertical_force_n: f64,
}

#[derive(Default, Serialize)]
struct Balance {
    intervals: u64,
    worst_end_ns: u64,
    max_abs_residual_n_s: f64,
    accumulated_residual_n_s: f64,
    absolute_residual_sum_n_s: f64,
    pad_impulses_n_s: [f64; 2],
    receiver_impulse_n_s: f64,
    other_impulse_n_s: f64,
}

/// Begin at the first recorded velocity. The forces in each following row
/// are averages across the interval *ending* at that row.
fn integrate(input: impl BufRead, mass_kg: f64, period_ns: u64) -> Result<Balance> {
    if !mass_kg.is_finite() || mass_kg <= 0. {
        return Err("expected positive finite parcel mass".into());
    }
    if period_ns == 0 {
        return Err("expected a positive public timestep".into());
    }
    let mut previous: Option<Sample> = None;
    let mut balance = Balance::default();
    for line in input.lines() {
        let end: Sample = serde_json::from_str(&line?)?;
        if let Some(start) = previous {
            if end.time_ns.checked_sub(start.time_ns) != Some(period_ns) {
                return Err("missing, duplicated or out-of-order public timestep".into());
            }
            let dt = period_ns as f64 * 1e-9;
            let pads = end.pad_vertical_force_n.map(|force| force * dt);
            let receiver = end.receiver_vertical_force_n * dt;
            let other = end.other_vertical_force_n * dt;
            let predicted = pads[0] + pads[1] + receiver + other - 9.81 * mass_kg * dt;
            let residual = mass_kg * (end.parcel_vz_m_s - start.parcel_vz_m_s) - predicted;
            if !residual.is_finite() {
                return Err("non-finite momentum balance".into());
            }
            if residual.abs() > balance.max_abs_residual_n_s {
                balance.worst_end_ns = end.time_ns;
                balance.max_abs_residual_n_s = residual.abs();
            }
            balance.intervals += 1;
            balance.accumulated_residual_n_s += residual;
            balance.absolute_residual_sum_n_s += residual.abs();
            for (sum, impulse) in balance.pad_impulses_n_s.iter_mut().zip(pads) {
                *sum += impulse;
            }
            balance.receiver_impulse_n_s += receiver;
            balance.other_impulse_n_s += other;
        }
        previous = Some(end);
    }
    if balance.intervals == 0 {
        return Err("expected at least two parcel observations".into());
    }
    Ok(balance)
}

fn inspect(path: &Path) -> Result<Balance> {
    let run: Run = serde_json::from_reader(File::open(path.join("summary.json"))?)?;
    let ticks = BufReader::new(File::open(path.join("ticks.jsonl"))?);
    integrate(ticks, run.parcel_mass_kg, run.public_dt_ns)
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.max_error.is_some_and(|n| !n.is_finite() || n < 0.) {
        return Err("expected a finite nonnegative momentum tolerance".into());
    }
    for path in args.recordings {
        let balance = inspect(&path)?;
        println!(
            "{}",
            serde_json::json!({"recording":path,"balance":balance})
        );
        if args
            .max_error
            .is_some_and(|n| balance.max_abs_residual_n_s > n)
        {
            return Err(format!(
                "{} exceeded the permitted momentum residual",
                path.display()
            )
            .into());
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn row(t: u64, vz: f64, pads: [f64; 2], receiver: f64, other: f64) -> String {
        format!(
            "{}\n",
            json!({"time_ns":t,"parcel_vz_m_s":vz,"pad_vertical_force_n":pads,
            "receiver_vertical_force_n":receiver,"other_vertical_force_n":other})
        )
    }

    #[test]
    fn accepts_signed_reactions_on_the_public_clock() {
        let mut rows = row(100_000, 0.3, [1., 2.], 0., 0.);
        rows += &row(1_100_000, 0.3, [7., 2.81], 0., 0.);
        rows += &row(2_100_000, 0.3 - 0.001 * 5., [-1., 0.], 6., -0.19);
        let r = integrate(rows.as_bytes(), 1., 1_000_000).unwrap();
        assert_eq!(r.intervals, 2);
        assert!(r.max_abs_residual_n_s < 1e-15);
        assert!((r.pad_impulses_n_s[0] - 0.006).abs() < 1e-15);
        assert!((r.receiver_impulse_n_s - 0.006).abs() < 1e-15);
        assert!(r.other_impulse_n_s < 0.);
    }

    #[test]
    fn detects_an_unreported_vertical_impulse_at_its_tick() {
        let initial = row(1, 0., [0.; 2], 0., 0.);
        let traces = initial + &row(1_000_001, 0.01, [0.; 2], 0., 0.);
        let r = integrate(traces.as_bytes(), 1., 1_000_000).unwrap();
        assert_eq!(r.worst_end_ns, 1_000_001);
        assert!((r.accumulated_residual_n_s - 0.01981).abs() < 1e-12);
        assert_eq!(r.max_abs_residual_n_s, r.absolute_residual_sum_n_s);
    }

    #[test]
    fn rejects_unknown_mass_empty_or_invalid_data_and_clock_reversal() {
        let start = row(2, 0., [0.; 2], 0., 0.);
        assert!(integrate(start.as_bytes(), 1., 0).is_err());
        assert!(integrate(start.as_bytes(), 0., 1_000_000).is_err());
        assert!(integrate(start.as_bytes(), 1., 1_000_000).is_err());
        assert!(
            integrate(
                (start + &row(1, 0., [0.; 2], 0., 0.)).as_bytes(),
                1.,
                1_000_000
            )
            .is_err()
        );
        assert!(integrate("{}".as_bytes(), 1., 1_000_000).is_err());
        let overflow = row(1, 0., [0.; 2], 0., 0.) + &row(u64::MAX, 0., [1e308; 2], 0., 0.);
        assert!(integrate(overflow.as_bytes(), 1., u64::MAX - 1).is_err());
    }
}
