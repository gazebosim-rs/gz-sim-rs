// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fit one synthetic training drop per layout and predict held-out experiments.
use gz_physics::{
    flat_landing::FlatLanding,
    sole_calibration::{DropFit, ImpactMeasurement, MeasurementRange, common_damping},
};
use plotters::prelude::*;
use serde::Deserialize;
use serde_json::json;
use std::{error::Error, fs, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Deserialize)]
struct Sample {
    t_s: f64,
    closing_m_s: f64,
    compression_m: f64,
    normal_n: f64,
}
#[derive(Deserialize)]
struct Case {
    case: usize,
    layout: String,
    drop: String,
    mass_kg: f64,
    impact_m_s: f64,
    covered_area_m2: f64,
    samples: Vec<Sample>,
}
impl Case {
    fn outgoing(&self) -> Result<f64> {
        Ok(-self.samples.last().ok_or("empty landing")?.closing_m_s)
    }
    fn depth(&self) -> f64 {
        self.samples
            .iter()
            .map(|s| s.compression_m)
            .fold(0., f64::max)
    }
}
fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1);
    let cases: Vec<Case> =
        serde_json::from_slice(&fs::read(args.next().ok_or("provide cases.json")?)?)?;
    let out = PathBuf::from(args.next().ok_or("provide output directory")?);
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    fs::create_dir_all(&out)?;
    let mut metrics = vec![];
    let mut trace = None;
    for seed in cases.iter().filter(|s| s.drop == "standard") {
        // Heterogeneous tiles yield the area-weighted effective modulus of the footprint.
        let fit = DropFit::measure(
            seed.mass_kg,
            seed.covered_area_m2,
            seed.impact_m_s,
            seed.outgoing()?,
            ImpactMeasurement::MaximumCompressionM(seed.depth()),
        )
        .ok_or("invalid training drop")?;
        let mut observations = vec![];
        let mut predicted = vec![];
        for name in ["heavy", "fast", "rising_deck", "inclined_moving_deck"] {
            let target = cases
                .iter()
                .find(|c| c.layout == seed.layout && c.drop == name)
                .ok_or("missing held-out drop")?;
            let model = FlatLanding::new(
                target.mass_kg,
                fit.effective_n_m3 * target.covered_area_m2,
                fit.damping_s_m,
                target.impact_m_s,
            )
            .ok_or("invalid held-out model")?;
            let release = model.at_phase(std::f64::consts::PI).unwrap();
            let mut points = vec![];
            let mut force_error = 0_f64;
            let mut scale = 0_f64;
            for sample in &target.samples {
                let f = if sample.t_s > release.elapsed_s {
                    0.
                } else {
                    model
                        .at_time(sample.t_s)
                        .ok_or("query outside trajectory")?
                        .normal_force_n
                };
                force_error = force_error.max((f - sample.normal_n).abs());
                scale = scale.max(sample.normal_n);
                points.push((sample.t_s * 1000., sample.normal_n, f));
            }
            let speed_error = (release.closing_m_s + target.outgoing()?).abs();
            let depth_error = (model.maximum_compression_m() - target.depth()).abs();
            if force_error / scale > 2e-4 || speed_error > 1e-4 || depth_error > 2e-7 {
                return Err(
                    format!("failed held-out force/speed/strain in case {}", target.case).into(),
                );
            }
            let range = |v| {
                MeasurementRange::positive(v - 0.0001, v + 0.0001).ok_or("invalid instrument range")
            };
            observations.push((range(target.impact_m_s)?, range(target.outgoing()?)?));
            predicted.push(json!({"test_case":target.case,
                "test_condition":name,
                "max_force_error_n":force_error,
                "max_force_error_fraction":force_error/scale,
                "rebound_error_m_s":speed_error,
                "depth_error_m":depth_error}));
            if seed.layout == "wide_deck" && name == "fast" {
                trace = Some(points)
            }
        }
        let common =
            common_damping(&observations).ok_or("inconsistent repeated speed measurements")?;
        if !(common.low..=common.high).contains(&fit.damping_s_m) {
            return Err("calibration outside held-out instrument envelope".into());
        }
        metrics.push(json!({"layout":seed.layout,
            "training_case":seed.case,
            "damping_s_m":fit.damping_s_m,
            "effective_stiffness_n_m":fit.effective_n_m3*seed.covered_area_m2,
            "effective_layer_n_m3":fit.effective_n_m3,
            "common_damping_s_m":[common.low,common.high],
            "predictions":predicted}));
    }
    if metrics.len() != 10 {
        return Err("expected ten training layouts".into());
    }
    let path = out.join("held-out-force.svg");
    let fig = SVGBackend::new(&path, (880, 460)).into_drawing_area();
    fig.fill(&WHITE)?;
    let points = trace.ok_or("missing force trace")?;
    let max = points.iter().map(|p| p.1).fold(0_f64, f64::max);
    let mut chart = ChartBuilder::on(&fig)
        .margin(20)
        .caption(
            "Predicting a faster landing from a slower drop",
            ("sans-serif", 24),
        )
        .x_label_area_size(48)
        .y_label_area_size(70)
        .build_cartesian_2d(0. ..points.last().unwrap().0, 0. ..max * 1.16)?;
    chart
        .configure_mesh()
        .x_desc("Time after touch (ms)")
        .y_desc("Normal force (N)")
        .draw()?;
    chart
        .draw_series(LineSeries::new(
            points.iter().map(|p| (p.0, p.2)),
            RGBColor(196, 98, 14).stroke_width(3),
        ))?
        .label("Predicted from 1 m/s training drop")
        .legend(|(x, y)| {
            PathElement::new([(x, y), (x + 20, y)], RGBColor(196, 98, 14).stroke_width(3))
        });
    chart
        .draw_series(
            points
                .iter()
                .map(|p| Circle::new((p.0, p.1), 4, RGBColor(14, 102, 130).filled())),
        )?
        .label("3D solver at 2.5 m/s")
        .legend(|(x, y)| Circle::new((x + 10, y), 4, RGBColor(14, 102, 130).filled()));
    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.95))
        .border_style(BLACK.mix(0.2))
        .draw()?;
    fig.present()?;
    fs::write(
        out.join("holdouts.json"),
        serde_json::to_vec_pretty(&metrics)?,
    )?;
    println!("10 training cases, 40 held-out predictions verified");
    Ok(())
}
