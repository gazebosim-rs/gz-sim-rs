// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot signed gap and contact impulse for matched warm_join_sweep trials.
use clap::Parser;
use plotters::prelude::*;
use serde::Deserialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::PathBuf,
};

#[derive(Parser)]
struct Args {
    /// SVG output path.
    #[arg(long)]
    figure: PathBuf,
    /// JSONL trial summaries (one or more seeds per file).
    #[arg(required = true)]
    trials: Vec<PathBuf>,
}
#[derive(Deserialize)]
struct Trial {
    seed: u64,
    step_ms: u32,
    ticks: u32,
    bitwise_frames: u32,
    closest_cross_surfaces_m: Option<f64>,
    peak_cross_impulse_ns: f64,
    error: Option<String>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let mut rows = BTreeMap::<u64, BTreeMap<u32, (f64, f64)>>::new();
    let mut steps = BTreeSet::new();
    let mut duration = None;
    for path in args.trials {
        for line in BufReader::new(File::open(path)?).lines() {
            let t: Trial = serde_json::from_str(&line?)?;
            let gap = t.closest_cross_surfaces_m.ok_or("missing signed gap")?;
            let impulse = t.peak_cross_impulse_ns;
            if t.error.is_some()
                || t.ticks == 0
                || t.bitwise_frames != t.ticks
                || t.step_ms == 0
                || !gap.is_finite()
                || !impulse.is_finite()
                || impulse < 0.
            {
                return Err(
                    "expected completed native-matched trials with finite diagnostics".into(),
                );
            }
            let time = u64::from(t.step_ms) * u64::from(t.ticks);
            if duration.replace(time).is_some_and(|prior| prior != time) {
                return Err("inconsistent observation windows".into());
            }
            if rows
                .entry(t.seed)
                .or_default()
                .insert(t.step_ms, (1000. * gap, impulse))
                .is_some()
            {
                return Err("duplicate seed and timestep".into());
            }
            steps.insert(t.step_ms);
        }
    }
    let seconds = duration.ok_or("no observations")? as f64 / 1000.;
    if !rows.values().all(|entries| entries.len() == steps.len()) {
        return Err("incomplete matched timestep matrix".into());
    }
    let width = (206 + 175 * u32::try_from(steps.len())?).max(720);
    let height = 171 + 75 * u32::try_from(rows.len())?;
    let out = SVGBackend::new(&args.figure, (width, height)).into_drawing_area();
    out.fill(&RGBColor(255, 255, 255))?;
    let ink = RGBColor(25, 43, 56);
    let muted = RGBColor(65, 85, 100);
    let label = |text: String, x, y, size, color: RGBColor, style| {
        out.draw(&Text::new(
            text,
            (x, y),
            ("sans-serif", size).into_font().style(style).color(&color),
        ))
    };
    label(
        "Rotating box trains at different timesteps".into(),
        18,
        12,
        28,
        ink,
        FontStyle::Bold,
    )?;
    label(
        format!("Same initial states and applied forces, observed over {seconds:.2} s."),
        20,
        53,
        17,
        muted,
        FontStyle::Normal,
    )?;
    label("Seed".into(), 20, 88, 18, ink, FontStyle::Bold)?;
    for (j, step) in steps.iter().rev().enumerate() {
        label(
            format!("{step} ms"),
            196 + 175 * j as i32,
            88,
            18,
            ink,
            FontStyle::Bold,
        )?;
    }
    for (i, (seed, series)) in rows.iter().enumerate() {
        let y = 122 + 75 * i as i32;
        label(seed.to_string(), 20, y + 19, 20, ink, FontStyle::Normal)?;
        for (j, (_, (gap, impulse))) in series.iter().rev().enumerate() {
            let x = 184 + 175 * j as i32;
            let (fill, accent) = if *impulse == 0. {
                (RGBColor(255, 238, 220), RGBColor(139, 65, 24))
            } else {
                (RGBColor(233, 244, 245), RGBColor(21, 88, 97))
            };
            out.draw(&Rectangle::new([(x, y), (x + 163, y + 65)], fill.filled()))?;
            label(
                format!("{gap:+.3} mm"),
                x + 12,
                y + 10,
                21,
                ink,
                FontStyle::Bold,
            )?;
            label(
                format!("{impulse:.3} N·s"),
                x + 12,
                y + 39,
                16,
                accent,
                FontStyle::Normal,
            )?;
        }
    }
    label(
        "The upper value is the minimum sampled surface gap. A negative value indicates overlap."
            .into(),
        20,
        height as i32 - 44,
        15,
        muted,
        FontStyle::Normal,
    )?;
    label(
        "The lower value is the peak cross-train normal impulse. Orange cells recorded zero impulse.".into(),
        20,
        height as i32 - 22,
        15,
        muted,
        FontStyle::Normal,
    )?;
    out.present()?;
    Ok(())
}
