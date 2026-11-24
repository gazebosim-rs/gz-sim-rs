// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::Result;
use plotters::{coord::Shift, prelude::*};
use std::{collections::BTreeMap, fs, path::Path};

/// All three replicate p95 values stay visible. Markers are medians and bars span the runs.
fn panel(
    area: &DrawingArea<SVGBackend<'_>, Shift>,
    file: &Path,
    title: &str,
    levels: &[usize],
    legend: bool,
) -> Result<()> {
    let contents = fs::read_to_string(file)?;
    let mut runs = BTreeMap::<(usize, usize), Vec<f64>>::new();
    for line in contents.lines().skip(1) {
        let fields = line.split(',').collect::<Vec<_>>();
        runs.entry((fields[1].parse()?, fields[2].parse()?))
            .or_default()
            .push(fields[4].parse::<f64>()? / 1000.);
    }
    let high = runs.values().flatten().copied().fold(0f64, f64::max);
    let mut chart = ChartBuilder::on(area)
        .margin(20)
        .x_label_area_size(53)
        .y_label_area_size(75)
        .caption(title, ("sans-serif", 21))
        .build_cartesian_2d(-0.5f64..levels.len() as f64 - 0.5, 0.0..high * 1.18)?;
    chart
        .configure_mesh()
        .disable_x_mesh()
        .x_labels(levels.len() * 2)
        .x_label_formatter(&|&x| {
            let i = x.round();
            if (x - i).abs() < 0.01 && i >= 0. && (i as usize) < levels.len() {
                levels[i as usize].to_string()
            } else {
                String::new()
            }
        })
        .y_desc("Whole-step p95 (ms)")
        .x_desc("Environments assigned to native Rapier")
        .draw()?;
    for (j, (workers, color)) in [
        (1, RGBColor(73, 77, 88)),
        (2, RGBColor(16, 123, 147)),
        (4, RGBColor(189, 85, 41)),
    ]
    .into_iter()
    .enumerate()
    {
        for (i, &n) in levels.iter().enumerate() {
            let samples = runs
                .get_mut(&(n, workers))
                .ok_or("missing timing replicate")?;
            if samples.len() != 3 {
                return Err("expected three counterbalanced timing replicates".into());
            }
            samples.sort_by(f64::total_cmp);
            let x = i as f64 + 0.16 * (j as f64 - 1.);
            chart.draw_series(std::iter::once(PathElement::new(
                vec![(x, samples[0]), (x, samples[2])],
                color.stroke_width(2),
            )))?;
            for &v in &[samples[0], samples[2]] {
                chart.draw_series(std::iter::once(PathElement::new(
                    vec![(x - 0.042, v), (x + 0.042, v)],
                    color.stroke_width(2),
                )))?;
            }
            let point = chart.draw_series(std::iter::once(Circle::new(
                (x, samples[1]),
                5,
                color.filled(),
            )))?;
            if i == 0 && legend {
                point
                    .label(format!(
                        "{workers} worker{}",
                        if workers == 1 { "" } else { "s" }
                    ))
                    .legend(move |(x, y)| Circle::new((x + 8, y), 5, color.filled()));
            }
        }
    }
    if legend {
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .background_style(WHITE.mix(0.88))
            .border_style(RGBColor(190, 195, 199))
            .draw()?;
    }
    Ok(())
}
pub fn draw(dir: &Path) -> Result<()> {
    let p = dir.join("native-staging.svg");
    let area = SVGBackend::new(&p, (1120, 750)).into_drawing_area();
    area.fill(&WHITE)?;
    let area = area.titled(
        "Native CPU staging across 2,048 mixed physics environments",
        ("sans-serif", 28),
    )?;
    let (plots, foot) = area.split_vertically(665);
    let parts = plots.split_evenly((2, 1));
    panel(
        &parts[0],
        &dir.join("first/timing.csv"),
        "Bevy physics schedule with a dense fenced GPU readback",
        &[0, 4, 24, 96],
        true,
    )?;
    panel(
        &parts[1],
        &dir.join("scene/timing.csv"),
        "Ordinary SDF world including pose synchronization, IMUs, altimeters and sampled observations",
        &[24, 96],
        false,
    )?;
    foot.draw(&Text::new("RTX 5090 · four pinned host CPU cores · dots show median replicate p95 · bars span three order-balanced replicates", (55,24),("sans-serif",15).into_font().color(&RGBColor(57,66,77))))?;
    foot.present()?;
    Ok(())
}

/// Compare each GPU-first trial with the adjacent CPU-first trial at the same
/// workers, load, and repeat. Include every paired range and median fence wait.
pub fn draw_overlap(dir: &Path) -> Result<()> {
    let mut pairs = BTreeMap::<(usize, usize, usize), [f64; 2]>::new();
    let mut wait = BTreeMap::<(usize, usize, bool), Vec<f64>>::new();
    for (campaign, subdir) in ["scene", "reverse"].into_iter().enumerate() {
        for line in fs::read_to_string(dir.join(subdir).join("timing.csv"))?
            .lines()
            .skip(1)
        {
            let f = line.split(',').collect::<Vec<_>>();
            let key = (
                f[1].parse()?,
                f[2].parse()?,
                campaign * 6 + f[0].parse::<usize>()?,
            );
            pairs.entry(key).or_default()[usize::from(f[8].parse::<bool>()?)] = f[4].parse()?;
        }
        for line in fs::read_to_string(dir.join(subdir).join("steps.csv"))?
            .lines()
            .skip(1)
        {
            let f = line.split(',').collect::<Vec<_>>();
            if f[3].parse::<usize>()? > 30 {
                wait.entry((f[1].parse()?, f[2].parse()?, f[9].parse()?))
                    .or_default()
                    .push(f[12].parse::<f64>()? / 1000.);
            }
        }
    }
    let mut paired =
        String::from("native,workers,campaign,rep,cpu_first_p95_us,gpu_first_p95_us,ratio\n");
    use std::fmt::Write as _;
    for (&(n, w, rep), v) in &pairs {
        writeln!(
            paired,
            "{n},{w},{},{},{},{},{}",
            rep / 6,
            rep % 6,
            v[0],
            v[1],
            v[1] / v[0]
        )?;
    }
    fs::write(dir.join("matched.csv"), paired)?;
    let p = dir.join("native-overlap.svg");
    let canvas = SVGBackend::new(&p, (1140, 780)).into_drawing_area();
    canvas.fill(&WHITE)?;
    let root = canvas.titled(
        "Early GPU submission in an ordinary 2,048-environment Gazebo world",
        ("sans-serif", 27),
    )?;
    let (main, foot) = root.split_vertically(685);
    let rows = main.split_evenly((2, 1));
    let loads = [4, 24, 96];
    let colors = [
        RGBColor(73, 77, 88),
        RGBColor(16, 123, 147),
        RGBColor(189, 85, 41),
    ];
    let workers = [1, 2, 4];
    let ratios = pairs
        .values()
        .map(|v| (v[1] / v[0]).log2())
        .collect::<Vec<_>>();
    if ratios.len() != 108 || ratios.iter().any(|v| !v.is_finite()) {
        return Err("expected two campaigns with 108 finite matched pairs".into());
    }
    let lo = ratios.iter().copied().fold(0., f64::min).min(-0.1);
    let hi = ratios.iter().copied().fold(0., f64::max).max(0.1);
    let mut chart = ChartBuilder::on(&rows[0])
        .margin(15)
        .x_label_area_size(48)
        .y_label_area_size(80)
        .caption(
            "Change in whole-step p95 when GPU commands are submitted before native CPU work",
            ("sans-serif", 20),
        )
        .build_cartesian_2d(-0.5f64..2.5, lo * 1.15..hi * 1.15)?;
    let labels = |&x: &f64| {
        let i = x.round();
        if (i - x).abs() < 0.01 && (0.0..3.).contains(&i) {
            loads[i as usize].to_string()
        } else {
            String::new()
        }
    };
    chart
        .configure_mesh()
        .disable_x_mesh()
        .x_labels(6)
        .x_label_formatter(&labels)
        .y_label_formatter(&|v| format!("{:.2}×", 2f64.powf(*v)))
        .y_desc("GPU-first / CPU-first p95")
        .x_desc("Native worlds")
        .draw()?;
    chart.draw_series(std::iter::once(PathElement::new(
        [(-0.5, 0.), (2.5, 0.)],
        BLACK.mix(0.55),
    )))?;
    for (j, &cap) in workers.iter().enumerate() {
        let color = colors[j];
        for (i, &n) in loads.iter().enumerate() {
            let x = i as f64 + (j as f64 - 1.) * 0.17;
            let mut data = (0..12)
                .map(|rep| {
                    let v = pairs[&(n, cap, rep)];
                    (v[1] / v[0]).log2()
                })
                .collect::<Vec<_>>();
            data.sort_by(f64::total_cmp);
            chart.draw_series(std::iter::once(PathElement::new(
                [(x, data[0]), (x, data[11])],
                color.stroke_width(1),
            )))?;
            for (k, value) in data.iter().enumerate() {
                chart.draw_series(std::iter::once(Circle::new(
                    (x + (k as f64 - 5.5) * 0.007, *value),
                    3,
                    color.mix(0.48).filled(),
                )))?;
            }
            let dot = chart.draw_series(std::iter::once(Circle::new(
                (x, (data[5] + data[6]) / 2.),
                5,
                color.filled(),
            )))?;
            if i == 0 {
                dot.label(format!("{cap} worker{}", if cap == 1 { "" } else { "s" }))
                    .legend(move |(x, y)| Circle::new((x + 8, y), 5, color.filled()));
            }
        }
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.9))
        .border_style(RGBColor(190, 195, 199))
        .draw()?;
    let mut medians = BTreeMap::new();
    for (key, samples) in &mut wait {
        samples.sort_by(f64::total_cmp);
        medians.insert(*key, samples[samples.len() / 2]);
    }
    let high = medians.values().copied().fold(1., f64::max) * 1.16;
    let labels = (0..9)
        .map(|i| format!("{} / {}", loads[i / 3], workers[i % 3]))
        .collect::<Vec<_>>();
    let mut chart = ChartBuilder::on(&rows[1])
        .margin(15)
        .x_label_area_size(54)
        .y_label_area_size(80)
        .caption(
            "Residual host wait for device completion and readback mapping",
            ("sans-serif", 20),
        )
        .build_cartesian_2d(-0.5f64..8.5, 0.0..high)?;
    chart
        .configure_mesh()
        .disable_x_mesh()
        .x_labels(18)
        .x_label_formatter(&|x| {
            let i = x.round();
            if (*x - i).abs() < 0.01 && (0.0..9.).contains(&i) {
                labels[i as usize].clone()
            } else {
                String::new()
            }
        })
        .x_desc("Native worlds / CPU workers")
        .y_desc("Median fence wait (µs)")
        .draw()?;
    for (j, (early, color, title)) in [
        (false, RGBColor(139, 147, 157), "CPU first"),
        (true, RGBColor(20, 139, 154), "GPU first"),
    ]
    .into_iter()
    .enumerate()
    {
        chart
            .draw_series((0..9).map(|i| {
                let x = i as f64 + (j as f64 - 0.5) * 0.3;
                Rectangle::new(
                    [
                        (x - 0.14, 0.),
                        (x + 0.14, medians[&(loads[i / 3], workers[i % 3], early)]),
                    ],
                    color.filled(),
                )
            }))?
            .label(title)
            .legend(move |(x, y)| Rectangle::new([(x, y - 4), (x + 14, y + 4)], color.filled()));
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.9))
        .border_style(RGBColor(190, 195, 199))
        .draw()?;
    foot.draw(&Text::new("RTX 5090 · fixed four-core host affinity · 12 paired replicates per setting · values below 1× mean lower whole-step p95 with early submission", (60,26),("sans-serif",15).into_font().color(&RGBColor(57,66,77))))?;
    canvas.present()?;
    Ok(())
}
