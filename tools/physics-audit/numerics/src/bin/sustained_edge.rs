// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolate continuous vertical momentum acquired after the registered instantaneous edge impulse.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{env, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn input(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn number(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|v| v.is_finite())
        .ok_or_else(|| "missing finite number".into())
}
fn array(v: &Value) -> Result<&Vec<Value>> {
    v.as_array().ok_or_else(|| "expected array".into())
}
fn z_momentum(f: &Value) -> Result<f64> {
    // Both link centers coincide with their inertial origins and have 1 kg mass.
    Ok(number(&f["foot"]["velocity"][2])? + number(&f["upper"]["velocity"][2])?)
}
fn main() -> Result<()> {
    let arg = env::args_os().nth(1).ok_or("sustained_edge DATA_DIR")?;
    let dir = Path::new(&arg);
    let setup = input(&dir.join("comparison.json"))?;
    let catalog = input(&dir.join("study.json"))?;
    let row = array(&catalog["rows"])?
        .iter()
        .find(|r| r["case"] == setup["case"])
        .ok_or("missing displayed case")?;
    // The original zero-gravity catalogs omit this field. Later campaigns publish it explicitly.
    let gravity_z = match catalog.get("gravity_m_s2") {
        Some(g) => number(&g[2])?,
        None => 0.,
    };
    let mut traces = Vec::new();
    let mut exact = vec![];
    let mut checks = 0;
    for (lane, profile) in array(&setup["lanes"])?.iter().zip(array(&row["profiles"])?) {
        let source = input(&dir.join(lane["file"].as_str().ok_or("trace file")?))?;
        let frames = array(&source["frames"])?;
        let obs = array(&profile["samples"])?;
        let initial = z_momentum(&frames[0])?;
        let t0 = number(&frames[0]["time_us"])?;
        let initial_j = number(&obs[0]["instantaneous_ns"][1])?;
        let pts: Vec<(f64, f64)> = frames
            .iter()
            .enumerate()
            .map(|(i, f)| -> Result<_> {
                let j = if i == 0 {
                    0.
                } else {
                    let seconds = (number(&f["time_us"])? - t0) * 1e-6;
                    z_momentum(f)? - initial - initial_j - 2. * gravity_z * seconds
                };
                Ok((number(&f["time_us"])? / 1000., 1000. * j))
            })
            .collect::<Result<_>>()?;
        let mut reference = vec![(0., 0.)];
        for s in obs {
            let tick = s["tick"].as_u64().ok_or("tick")? as usize;
            let expected = number(&s["momentum_post_impulse_ns"][2])?;
            let measured = pts[tick].1 * 0.001;
            if (measured - expected).abs() > 1e-10 {
                return Err(
                    "source velocities disagree with archived postimpact vertical impulse".into(),
                );
            }
            reference.push((
                pts[tick].0,
                1000. * number(&s["reference_post_impulse_ns"][2])?,
            ));
            checks += 1;
        }
        if exact.is_empty() {
            exact = reference;
        } else if exact != reference {
            return Err("unequal analytic continuations".into());
        }
        let calls = profile["calls"].as_u64().ok_or("calls")?;
        let follows = obs
            .iter()
            .any(|s| s["followed_ticks"].as_u64().is_some_and(|v| v > 0));
        if follows && lane["label"] != "arc" {
            return Err("unexpected continuous-mode lane".into());
        }
        let name = if follows {
            "Admitted continuous edge".to_owned()
        } else {
            format!("{calls} calls")
        };
        traces.push((name, pts, calls));
    }
    if traces.len() != 3 {
        return Err("expected three measured native grids".into());
    }
    let end = traces[0].1.last().ok_or("missing final state")?.0;
    let path = dir.join("view/sustained-impulse.svg");
    let root = SVGBackend::new(&path, (960, 710)).into_drawing_area();
    root.fill(&WHITE)?;
    let zones = root
        .titled(
            "Sustained vertical reaction after the initial edge impulse",
            ("sans-serif", 25),
        )?
        .split_evenly((2, 1));
    let colors = [
        RGBColor(186, 73, 23),
        RGBColor(13, 127, 102),
        RGBColor(57, 91, 166),
    ];
    for (area, (bound, title)) in zones.iter().zip([
        (1., "First two public intervals"),
        (end, "Complete loaded interval"),
    ]) {
        let ymax = traces
            .iter()
            .flat_map(|(_, v, _)| v.iter())
            .chain(exact.iter())
            .filter(|(t, _)| *t <= bound)
            .map(|(_, y)| *y)
            .fold(0.001, f64::max)
            * 1.16;
        let mut p = ChartBuilder::on(area)
            .margin(10)
            .caption(title, ("sans-serif", 18))
            .x_label_area_size(38)
            .y_label_area_size(82)
            .build_cartesian_2d(0.0..bound, 0.0..ymax)?;
        p.configure_mesh()
            .max_light_lines(1)
            .x_desc("Time after the initial impact (ms)")
            .y_desc("Additional vertical impulse (mN·s)")
            .draw()?;
        for ((name, v, _), &color) in traces.iter().zip(&colors) {
            p.draw_series(LineSeries::new(
                v.iter().copied().filter(|(t, _)| *t <= bound),
                color.stroke_width(2),
            ))?
            .label(name.clone())
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 22, y)], color.stroke_width(2)));
        }
        p.draw_series(
            exact
                .iter()
                .filter(|(t, _)| *t <= bound)
                .map(|p| TriangleMarker::new(*p, 6, BLACK.filled())),
        )?
        .label("Constrained reference")
        .legend(|(x, y)| TriangleMarker::new((x + 10, y), 5, BLACK.filled()));
        p.configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .background_style(WHITE.mix(0.9))
            .border_style(BLACK.mix(0.15))
            .draw()?;
    }
    root.present()?;
    fs::write(
        dir.join("checks/sustained-impulse.json"),
        serde_json::to_vec_pretty(
            &json!({"case":setup["case"],"public_velocity_samples":traces.iter().map(|(_,p,_)|p.len()).sum::<usize>(),
        "gravity_z_m_s2":gravity_z,"total_mass_kg":2.0,"crosschecked_samples":checks,"reference_samples":exact.len()-1,"first_postimpact_vertical_millinewton_seconds":traces.iter().map(|(name,p,n)|json!({"mode":name,"configured_calls":n,"value":p[1].1})).collect::<Vec<_>>() }),
        )?,
    )?;
    Ok(())
}
