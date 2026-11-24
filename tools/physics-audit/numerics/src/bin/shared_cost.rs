// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Counterbalanced matched Bevy runs, stratified by the registered physical mode.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
struct Row {
    case: String,
    rep: usize,
    registered: bool,
    modes: Vec<usize>,
    wall_ns: Vec<u64>,
    thread_cpu_ns: Vec<u64>,
    selected_fraction_normal_heat_beltwork: Vec<Option<[f64; 4]>>,
}
impl Row {
    fn read(value: serde_json::Value) -> Self {
        Self {
            case: value["case"].as_str().unwrap().to_owned(),
            rep: value["rep"].as_u64().unwrap() as usize,
            registered: value["registered"].as_bool().unwrap(),
            modes: serde_json::from_value(value["modes"].clone()).unwrap(),
            wall_ns: serde_json::from_value(value["wall_ns"].clone()).unwrap(),
            thread_cpu_ns: serde_json::from_value(value["thread_cpu_ns"].clone()).unwrap(),
            selected_fraction_normal_heat_beltwork: serde_json::from_value(
                value["selected_fraction_normal_heat_beltwork"].clone(),
            )
            .unwrap(),
        }
    }
}
fn quantile(mut v: Vec<f64>, q: f64) -> f64 {
    v.sort_by(f64::total_cmp);
    v[((v.len() as f64 * q).ceil() as usize).clamp(1, v.len()) - 1]
}
fn plot(out: &std::path::Path, samples: &[Value]) -> Result<(), Box<dyn Error>> {
    let file = out.join("shared-cost.svg");
    let page = SVGBackend::new(&file, (1050, 520)).into_drawing_area();
    page.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&page)
        .margin(25)
        .caption(
            "Matched local support steps in three scenes",
            ("sans-serif", 24),
        )
        .x_label_area_size(72)
        .y_label_area_size(80)
        .build_cartesian_2d(-0.6f64..2.6f64, 0.85f64..1.95f64)?;
    let labels = [
        "Yawing sole",
        "Sliding, different belts",
        "Arrested, common belts",
    ];
    chart
        .configure_mesh()
        .x_labels(3)
        .x_label_formatter(&|v| {
            if (*v - v.round()).abs() < 1e-8 {
                labels.get(v.round() as usize).unwrap_or(&"").to_string()
            } else {
                String::new()
            }
        })
        .x_desc("Observed contact mode")
        .y_desc("Median paired public-step wall time / ordinary")
        .draw()?;
    chart.draw_series([PathElement::new(
        vec![(-0.6, 1.), (2.6, 1.)],
        BLACK.mix(0.45),
    )])?;
    for (names, caption, offset, col) in [
        (
            &["before", "before-repeat", "before-final"][..],
            "Original finite-pressure proof",
            -0.19,
            RGBColor(169, 74, 40),
        ),
        (
            &["quadratic", "quadratic-repeat"][..],
            "Quadratic extrema",
            0.,
            RGBColor(59, 112, 169),
        ),
        (
            &["optimized", "optimized-repeat"][..],
            "Stationary pressure reuse",
            0.19,
            RGBColor(35, 130, 99),
        ),
    ] {
        let mut vals = vec![];
        for (i, (which, mode)) in [("yaw", 1), ("different_belts", 2), ("common_belts", 3)]
            .into_iter()
            .enumerate()
        {
            let x = i as f64 + offset;
            let mut v: Vec<_> = samples
                .iter()
                .filter(|s| {
                    names.contains(&s["run"].as_str().unwrap())
                        && s["case"] == which
                        && s["mode"] == mode
                })
                .map(|s| s["median_paired_wall_ratio"].as_f64().unwrap())
                .collect();
            assert_eq!(v.len(), names.len());
            v.sort_by(f64::total_cmp);
            chart.draw_series([PathElement::new(
                vec![(x, v[0]), (x, *v.last().unwrap())],
                col.stroke_width(2),
            )])?;
            for y in v {
                vals.push((x, y));
            }
        }
        chart
            .draw_series(vals.into_iter().map(|p| Circle::new(p, 5, col.filled())))?
            .label(caption)
            .legend(move |(x, y)| Circle::new((x + 10, y), 5, col.filled()));
    }
    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.9))
        .position(SeriesLabelPosition::UpperLeft)
        .draw()?;
    page.present()?;
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let out = PathBuf::from(std::env::args_os().nth(1).ok_or("supply phase directory")?);
    let mut physical = BTreeMap::new();
    let mut stats = vec![];
    let mut errors = [0f64; 4];
    let mut position = 0;
    for file in [
        "before",
        "before-repeat",
        "quadratic",
        "quadratic-repeat",
        "optimized",
        "optimized-repeat",
        "before-final",
    ] {
        let rows: Vec<serde_json::Value> =
            serde_json::from_slice(&fs::read(out.join(format!("{file}.json")))?)?;
        let rows = rows.into_iter().map(Row::read).collect::<Vec<_>>();
        assert_eq!(rows.len(), 192);
        type Samples = BTreeMap<(String, usize), Vec<[f64; 6]>>;
        let mut reps = Samples::new();
        for row in rows.iter().filter(|r| r.registered) {
            let ordinary = rows
                .iter()
                .find(|o| !o.registered && o.case == row.case && o.rep == row.rep)
                .unwrap();
            let (modes, trace) = physical
                .entry((row.case.clone(), row.rep))
                .or_insert_with(|| {
                    (
                        row.modes.clone(),
                        row.selected_fraction_normal_heat_beltwork.clone(),
                    )
                });
            assert_eq!(modes, &row.modes);
            for (a, b) in trace
                .iter()
                .zip(&row.selected_fraction_normal_heat_beltwork)
            {
                assert_eq!(a.is_some(), b.is_some());
                if let (Some(a), Some(b)) = (a, b) {
                    for i in 0..4 {
                        errors[i] = errors[i].max((a[i] - b[i]).abs());
                    }
                }
            }
            for phase in 0..4 {
                let ticks = row
                    .modes
                    .iter()
                    .enumerate()
                    .filter_map(|(i, m)| (*m == phase).then_some(i))
                    .collect::<Vec<_>>();
                if ticks.len() < 8 {
                    continue;
                }
                let get = |field: &Vec<u64>, p| {
                    quantile(ticks.iter().map(|i| field[*i] as f64 / 1000.).collect(), p)
                };
                reps.entry((row.case.clone(), phase)).or_default().push([
                    get(&row.wall_ns, 0.5),
                    get(&row.wall_ns, 0.95),
                    get(&ordinary.wall_ns, 0.5),
                    get(&ordinary.wall_ns, 0.95),
                    get(&row.thread_cpu_ns, 0.5),
                    get(&ordinary.thread_cpu_ns, 0.5),
                ]);
            }
        }
        for ((case, mode), reps) in reps {
            assert_eq!(reps.len(), 32);
            let middle = std::array::from_fn::<_, 6, _>(|i| {
                quantile(reps.iter().map(|r| r[i]).collect(), 0.5)
            });
            let ratio = quantile(reps.iter().map(|r| r[0] / r[2]).collect(), 0.5);
            stats.push(json!({"run":file,"case":case,"mode":mode,"reps":reps.len(),"median_rep_registered_p50_p95_ordinary_p50_p95_registered_cpu_ordinary_cpu_us":middle,"median_paired_wall_ratio":ratio}));
        }
        position += 1;
    }
    plot(&out, &stats)?;
    let data = json!({"runs":position,"repetitions_per_scene":32,"timings":stats,"max_across_builds_fraction_normal_dissipation_belt_work":errors});
    fs::write(
        out.join("checks/readback.json"),
        serde_json::to_vec_pretty(&data)?,
    )?;
    Ok(())
}
