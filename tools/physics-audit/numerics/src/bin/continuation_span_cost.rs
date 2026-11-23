// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read balanced six-way native step timings, including within-donor paired differences.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    io::{BufRead, BufReader},
    path::Path,
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn input(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn n(v: &Value) -> Result<usize> {
    v.as_u64()
        .and_then(|v| v.try_into().ok())
        .ok_or_else(|| "invalid timer integer".into())
}
fn stats(mut v: Vec<i64>) -> Value {
    v.sort_unstable();
    let n = v.len();
    let q = |p: usize| {
        v.get((p * n).div_ceil(100).saturating_sub(1))
            .map(|x| *x as f64 / 1000.)
    };
    json!({"n":n,"mean":(!v.is_empty()).then(||v.iter().map(|&v|v as f64/1000.).sum::<f64>()/n as f64),"p50":q(50),"p95":q(95),"p99":q(99),"min":q(0),"max":q(100)})
}
fn summarize(dir: &Path, source: &Path) -> Result<Value> {
    let design = input(&dir.join("design.json"))?;
    let fixtures = design["fixtures"]
        .as_array()
        .ok_or("missing planned sample")?
        .iter()
        .map(n)
        .collect::<Result<BTreeSet<_>>>()?;
    if fixtures.len() != 10
        || design["measured_repeats"] != 12
        || design["native_calls_per_tick"] != 8
    {
        return Err("unexpected span timing design".into());
    }
    let mut expected = BTreeMap::new();
    let mut clocks = BTreeMap::new();
    for index in &fixtures {
        let f = input(&source.join("cases").join(format!("case-{index:04}.json")))?;
        for tr in f["result"]["trials"]
            .as_array()
            .ok_or("missing archived trials")?
        {
            let s = n(&tr["stroke"])?;
            let budgets = tr["paths"]
                .as_array()
                .ok_or("span pathways")?
                .iter()
                .map(|p| n(&p["eight_pass_ticks"]))
                .collect::<Result<Vec<_>>>()?;
            expected.insert((*index, s), budgets);
        }
    }
    let mut slots = BTreeMap::<(usize, usize, usize), [u8; 6]>::new();
    for line in BufReader::new(fs::File::open(dir.join("span-cost.jsonl"))?).lines() {
        let v: Value = serde_json::from_str(&line?)?;
        let (i, s, r, k, p) = (
            n(&v["index"])?,
            n(&v["stroke"])?,
            n(&v["repeat"])?,
            n(&v["policy"])?,
            n(&v["position"])?,
        );
        if !(1..=12).contains(&r) || k >= 6 || p >= 6 {
            return Err("invalid measured timing key".into());
        }
        let planned = (i + s + (r - 1) % 6 + p) % 6;
        if k != if r > 6 { 5 - planned } else { planned }
            || n(&v["eight_pass_ticks"])?
                != *expected
                    .get(&(i, s))
                    .and_then(|b| b.get(k))
                    .ok_or("unknown measured donor")?
        {
            return Err("timed schedule differs from design or outcome".into());
        }
        let t = v["tick_ns"]
            .as_array()
            .ok_or("missing tick durations")?
            .iter()
            .map(|v| v.as_i64().filter(|&t| t > 0).ok_or("invalid tick duration"))
            .collect::<std::result::Result<Vec<_>, _>>()?;
        if t.len() != 26 || clocks.insert((i, s, r, k), t).is_some() {
            return Err("incomplete timing window or duplicate".into());
        }
        slots.entry((i, s, k)).or_default()[p] += 1;
    }
    if clocks.len() != expected.len() * 12 * 6 || slots.values().any(|v| v != &[2; 6]) {
        return Err("timing matrix not balanced or complete".into());
    }
    let mut policies = vec![];
    for (k, _) in [1, 2, 4, 8, 26, 0].iter().enumerate() {
        let mut ticks = vec![];
        let mut sums = vec![];
        let mut deltas = vec![];
        let mut admission = vec![];
        let mut ordinary = vec![];
        for (&(i, s, r, idx), v) in &clocks {
            if idx != k {
                continue;
            }
            ticks.extend_from_slice(&v[1..]);
            let cost = v[1..].iter().sum::<i64>();
            sums.push(cost);
            deltas.push(cost - clocks[&(i, s, r, 0)][1..].iter().sum::<i64>());
            let rich = expected[&(i, s)][k];
            admission.extend_from_slice(&v[1..rich]);
            ordinary.extend_from_slice(&v[rich..]);
        }
        policies.push(json!({"policy":k,"suffix_tick_us":stats(ticks),"rich_suffix_tick_us":stats(admission),"ordinary_suffix_tick_us":stats(ordinary),
            "suffix_sum_us":stats(sums),"paired_suffix_sum_minus_onset_us":stats(deltas)}));
    }
    Ok(
        json!({"cpu":dir.file_name().ok_or("placement name")?.to_string_lossy(),"measured_records":clocks.len(),"matched_donors":expected.len(),
        "recorded_tick_durations":clocks.len()*26,"policies":policies}),
    )
}
fn plot(data: &[Value], path: &Path) -> Result<()> {
    let root = SVGBackend::new(path, (950, 650)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root
        .titled("Measured cost of continuing relaxation", ("sans-serif", 24))?
        .split_evenly((2, 1));
    let names = ["1", "2", "4", "8", "26", "weight"];
    for (area, (field, label)) in areas.iter().zip([
        ("suffix_tick_us", "p95 individual suffix step (µs)"),
        ("suffix_sum_us", "p95 total 25-step suffix (µs)"),
    ]) {
        let max = data
            .iter()
            .flat_map(|d| d["policies"].as_array().unwrap())
            .map(|p| p[field]["p95"].as_f64().unwrap())
            .fold(0., f64::max)
            * 1.22;
        let mut chart = ChartBuilder::on(area)
            .margin(12)
            .x_label_area_size(45)
            .y_label_area_size(75)
            .build_cartesian_2d(-0.5f64..5.5, 0.0f64..max)?;
        chart
            .configure_mesh()
            .max_light_lines(0)
            .x_labels(6)
            .x_label_formatter(&|x| {
                let i = x.round() as isize;
                if (x - i as f64).abs() < 1e-6 && (0..6).contains(&i) {
                    names[i as usize].to_string()
                } else {
                    String::new()
                }
            })
            .x_desc("First intervals with eight passes, or causal weight release")
            .y_desc(label)
            .draw()?;
        for (i, d) in data.iter().enumerate() {
            let color = [RGBColor(23, 116, 104), RGBColor(177, 77, 28)][i % 2];
            let points = d["policies"]
                .as_array()
                .unwrap()
                .iter()
                .map(|p| {
                    (
                        p["policy"].as_f64().unwrap(),
                        p[field]["p95"].as_f64().unwrap(),
                    )
                })
                .collect::<Vec<_>>();
            chart
                .draw_series(LineSeries::new(points.clone(), color.stroke_width(2)))?
                .label(d["cpu"].as_str().unwrap())
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 18, y)], color.stroke_width(2))
                });
            chart.draw_series(
                points
                    .into_iter()
                    .map(|p| Circle::new(p, 4, color.filled())),
            )?;
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .background_style(WHITE.mix(0.9))
            .draw()?;
    }
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() < 3 {
        return Err("continuation_span_cost SPAN_DIR OUTPUT CPU_DIR [CPU_DIR...]".into());
    }
    let out = Path::new(&args[1]);
    fs::create_dir_all(out)?;
    let data = args[2..]
        .iter()
        .map(|d| summarize(Path::new(d), Path::new(&args[0])))
        .collect::<Result<Vec<_>>>()?;
    plot(&data, &out.join("span-cost.svg"))?;
    fs::write(
        out.join("span-cost.json"),
        serde_json::to_vec_pretty(
            &json!({"placements":data,"span_order":[1,2,4,8,26,null],"public_tick_us":500,"warm_repeats_per_policy":1,"measured_repeats_per_policy":12}),
        )?,
    )?;
    Ok(())
}
