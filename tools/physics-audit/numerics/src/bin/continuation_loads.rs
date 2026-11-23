// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare positive suffix loads at physical thresholds on the timing cohort, with both high-grid histories.
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, fs, path::Path, process::Command};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const LIMITS: [f64; 4] = [0., 1e-6, 1e-4, 1e-2];
fn input(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn n(x: &Value) -> Result<u64> {
    x.as_u64().ok_or_else(|| "missing native clock".into())
}
fn d(x: &Value) -> Result<f64> {
    x.as_f64()
        .filter(|v| v.is_finite())
        .ok_or_else(|| "missing measurement".into())
}
fn replay(bin: &Path, mode: &str, k: u64, stroke: u64, dir: &Path) -> Result<()> {
    let status = Command::new(bin)
        .arg(mode)
        .arg(k.to_string())
        .arg("--survey-stroke")
        .arg(stroke.to_string())
        .arg(dir)
        .output()?;
    if !status.status.success() {
        return Err(format!(
            "{mode} {k}/{stroke}: {}",
            String::from_utf8_lossy(&status.stderr)
        )
        .into());
    }
    Ok(())
}
fn distribution(mut xs: Vec<f64>) -> Value {
    if xs.is_empty() {
        return json!({"n":0,"sum":0.,"median":null,"p95":null,"max":null});
    }
    xs.sort_unstable_by(f64::total_cmp);
    json!({"n":xs.len(),"sum":xs.iter().sum::<f64>(),"median":xs[xs.len().div_ceil(2)-1],"p95":xs[(95*xs.len()).div_ceil(100)-1],"max":xs.last()})
}
fn main() -> Result<()> {
    let a: Vec<_> = env::args_os().skip(1).collect();
    if a.len() == 3 && a[0] == "--check" {
        return check_published(Path::new(&a[1]), Path::new(&a[2]));
    }
    if a.len() != 6 {
        return Err(
            "continuation_loads DRIVER RELAXATION PRIMARY TIMING_DESIGN NEW_SCRATCH OUT".into(),
        );
    }
    let bin = Path::new(&a[0]);
    let relaxed = Path::new(&a[1]);
    let primary = Path::new(&a[2]);
    let design = input(Path::new(&a[3]))?;
    let scratch = Path::new(&a[4]);
    let out = Path::new(&a[5]);
    fs::create_dir(scratch)?;
    fs::create_dir_all(out)?;
    for (name, src) in [("r8", relaxed), ("r1", primary)] {
        fs::create_dir(scratch.join(name))?;
        std::os::unix::fs::symlink(
            src.join("cases").canonicalize()?,
            scratch.join(name).join("cases"),
        )?;
    }
    let mut rows = Vec::new();
    let mut cuts = 0;
    let mut worst: Option<(f64, u64, u64, u64)> = None;
    for k in design["first_fixture_for_each_offset_and_history"]
        .as_array()
        .ok_or("timing design")?
    {
        let k = n(k)?;
        let archived = input(&relaxed.join("cases").join(format!("case-{k:04}.json")))?;
        let default = input(&primary.join("cases").join(format!("case-{k:04}.json")))?;
        for trial in archived["result"]["trials"].as_array().ok_or("donors")? {
            let stroke = n(&trial["stroke"])?;
            let cut = n(&trial["cut"]["tick"])?;
            let matched = default["result"]["trials"]
                .as_array()
                .ok_or("default donors")?
                .iter()
                .find(|v| v["stroke"] == stroke)
                .ok_or("missing default donor")?;
            if trial["cut"] != matched["cut"] {
                return Err("high-grid original donor mismatch".into());
            }
            replay(bin, "--continuation-replay", k, stroke, &scratch.join("r8"))?;
            replay(bin, "--survey-replay", k, stroke, &scratch.join("r1"))?;
            let files = [
                "r8/continuation/traces/onset.json",
                "r8/continuation/traces/relaxed.json",
                "r8/continuation/traces/fine.json",
                "r1/traces/fine.json",
            ];
            let mut traces = Vec::new();
            for path in files {
                let raw = input(&scratch.join(path))?;
                let f = raw["frames"].as_array().ok_or("no frames")?.clone();
                assert_eq!(f.len(), 27);
                traces.push(f);
            }
            assert_eq!(traces[0][1], traces[1][1]);
            for t in traces.iter().skip(1) {
                for b in ["foot", "upper", "pad"] {
                    assert_eq!(traces[0][0][b], t[0][b]);
                }
            }
            let onset = d(&traces[0][1]["normal_impulse_ns"])?;
            let before = rows.len();
            for i in 2..27 {
                let tick = cut + i as u64;
                let mut js = [0.; 4];
                for (idx, t) in traces.iter().enumerate() {
                    let f = &t[i];
                    assert_eq!(f["tick"], tick);
                    assert_eq!(f["time_us"], tick * 500);
                    js[idx] = d(&f["normal_impulse_ns"])?;
                    let c = f["corner_ns"].as_array().ok_or("quadrant reaction")?;
                    let mut sum = 0.;
                    for v in c {
                        sum += d(v)?;
                    }
                    if js[idx] < 0. || (sum - js[idx]).abs() > 1e-10 {
                        return Err("invalid native integral".into());
                    }
                }
                // Exclude after footprint loss, counted relative to the nominal 25 steps.
                if [0, 1, 2, 3]
                    .iter()
                    .any(|&l| traces[l][..=i].iter().any(|f| f["gap_m"].is_null()))
                {
                    continue;
                }
                rows.push(json!({"index":k,"stroke":stroke,"tick":tick,"after_cut":i,"normal_ns":js,"onset_normal_ns":onset}));
                if js[0] == 0.
                    && js[1] > 0.
                    && worst.is_none_or(|(v, idx, s, t)| {
                        js[1] > v || (js[1] == v && (k, stroke, tick) < (idx, s, t))
                    })
                {
                    worst = Some((js[1], k, stroke, tick));
                }
            }
            if rows.len() == before {
                eprintln!("no covered suffix for fixture {k} stroke {stroke}");
            }
            cuts += 1;
        }
    }
    let mut records = Vec::new();
    for limit in LIMITS {
        let mut counts = [0usize; 16];
        let mut extra = Vec::new();
        let mut fraction = Vec::new();
        let mut supported: [Vec<f64>; 3] = Default::default();
        let mut r8_deltas = Vec::new();
        let mut r1_deltas = Vec::new();
        for row in &rows {
            let v = &row["normal_ns"];
            let j = [d(&v[0])?, d(&v[1])?, d(&v[2])?, d(&v[3])?];
            let mask = j
                .iter()
                .enumerate()
                .map(|(i, &v)| usize::from(v > limit) << i)
                .sum::<usize>();
            counts[mask] += 1;
            if mask & 3 == 2 {
                extra.push(j[1]);
                let incoming = d(&row["onset_normal_ns"])?;
                if incoming > 0. {
                    fraction.push(j[1] / incoming);
                }
                supported[(mask >> 2).count_ones() as usize].push(j[1]);
                r8_deltas.push((j[1] - j[2]).abs() - (j[0] - j[2]).abs());
                r1_deltas.push((j[1] - j[3]).abs() - (j[0] - j[3]).abs());
            }
        }
        records.push(json!({"threshold_ns":limit,"joint_masks":counts,"continue_above_stop_below":distribution(extra),
            "fraction_of_initial_reaction":distribution(fraction),"extra_load_grouped_by_reference_votes":supported.map(distribution),
            "change_in_absolute_reaction_error_vs_fine_r8_ns":distribution(r8_deltas),"change_in_absolute_reaction_error_vs_fine_r1_ns":distribution(r1_deltas)}));
    }
    if let Some((_, index, stroke, tick)) = worst {
        let source = scratch.join("r8");
        replay(bin, "--continuation-replay", index, stroke, &source)?;
        let source = source.join("continuation");
        let mut manifest = input(&source.join("comparison.json"))?;
        fs::create_dir_all(out.join("traces"))?;
        for (lane, label) in manifest["lanes"]
            .as_array()
            .ok_or("replay lanes")?
            .clone()
            .iter()
            .zip(["stopping_peak", "continuing_peak", "fine_peak"])
        {
            let name = lane["file"].as_str().ok_or("source path")?;
            let trace = input(&source.join(name))?;
            let mut local_best = None;
            for s in trace["frames"].as_array().ok_or("source frames")? {
                let t = n(&s["tick"])?;
                let v = d(&s["normal_impulse_ns"])?;
                if t.abs_diff(tick) <= 2 && local_best.is_none_or(|(prev, _)| v > prev) {
                    local_best = Some((v, t));
                }
            }
            manifest["capture_ticks"][label] =
                json!(local_best.ok_or("missing neighboring sample")?.1);
            fs::copy(source.join(name), out.join(name))?;
        }
        manifest["capture_ticks"]["largest_extra_load"] = json!(tick);
        fs::write(
            out.join("comparison.json"),
            serde_json::to_vec_pretty(&manifest)?,
        )?;
        fs::copy(
            source.join("replay-case.json"),
            out.join("replay-case.json"),
        )?;
    }
    let measurement = json!({"input":"native step-contact/replay histories reproduced from archived donors and matched sample records","lanes":["stop","continue","4096_r8","4096_r1"],
        "native_cuts":cuts,"steps":rows.len(),"censored":25*cuts-rows.len(),"thresholds":records,"largest_continue_load_when_stop_unloaded":worst.map(|(j,k,s,t)|json!({"index":k,"stroke":s,"tick":t,"normal_ns":j})),"samples":rows});
    fs::write(
        out.join("suffix-loads.json"),
        serde_json::to_vec(&measurement)?,
    )?;
    plot(&out.join("suffix-loads.svg"), &measurement)?;
    check_published(Path::new(&a[3]).parent().ok_or("timing directory")?, out)?;
    fs::remove_dir_all(scratch)?;
    Ok(())
}
fn plot(path: &Path, data: &Value) -> Result<()> {
    let root = SVGBackend::new(path, (980, 470)).into_drawing_area();
    root.fill(&WHITE)?;
    let body = root.titled(
        "When does continued relaxation retain a reaction?",
        ("sans-serif", 25),
    )?;
    let (top, chart) = body.split_vertically(60);
    top.draw(&Text::new("Intervals where eight passes exceeded the threshold and the onset-only continuation did not.",(22,16),("sans-serif",17)))?;
    top.draw(&Text::new("Colors count the two empirical 4,096-call references that also exceeded the same threshold.",(22,40),("sans-serif",15)))?;
    let r = data["thresholds"].as_array().ok_or("thresholds")?;
    let ymax = r
        .iter()
        .map(|s| n(&s["continue_above_stop_below"]["n"]))
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .max()
        .unwrap_or(1)
        .max(1) as f64;
    let mut p = ChartBuilder::on(&chart)
        .margin(10)
        .x_label_area_size(68)
        .y_label_area_size(66)
        .build_cartesian_2d(0.0..4.0, 0.0..1.35 * ymax)?;
    p.configure_mesh()
        .disable_x_mesh()
        .max_light_lines(1)
        .x_labels(0)
        .y_desc("Public intervals")
        .x_desc("Positive scalar normal impulse threshold (N·s)")
        .draw()?;
    for (i, s) in r.iter().enumerate() {
        let votes = s["extra_load_grouped_by_reference_votes"]
            .as_array()
            .ok_or("counts")?;
        let mut h = 0.;
        let label = ["0", "0.000001", "0.0001", "0.01"][i];
        let x = i as f64 + 0.5;
        for (j, a) in votes.iter().enumerate() {
            let nn = n(&a["n"])? as f64;
            let color = [
                RGBColor(189, 104, 61),
                RGBColor(200, 179, 79),
                RGBColor(27, 129, 124),
            ][j];
            let series = p.draw_series([Rectangle::new(
                [(x - 0.23, h), (x + 0.23, h + nn)],
                color.filled(),
            )])?;
            if i == 0 {
                series
                    .label(["Neither reference", "One reference", "Both references"][j])
                    .legend(move |(x, y)| {
                        Rectangle::new([(x, y - 4), (x + 15, y + 4)], color.filled())
                    });
            }
            h += nn;
        }
        p.draw_series([Text::new(
            format!("{}", h as u64),
            (x - 0.04, h + 0.05 * ymax),
            ("sans-serif", 15),
        )])?;
        let (px, py) = p.plotting_area().map_coordinate(&(x, 0.));
        let text_style =
            ("sans-serif", 14)
                .into_text_style(&root)
                .pos(plotters::style::text_anchor::Pos::new(
                    plotters::style::text_anchor::HPos::Center,
                    plotters::style::text_anchor::VPos::Top,
                ));
        root.draw(&Text::new(label, (px, py + 8), text_style))?;
    }
    p.configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.9))
        .draw()?;
    root.present()?;
    Ok(())
}

fn check_published(timing_run: &Path, out: &Path) -> Result<()> {
    let measured = input(&out.join("suffix-loads.json"))?;
    let mut loads = BTreeMap::new();
    for s in measured["samples"]
        .as_array()
        .ok_or("missing loaded samples")?
    {
        let key = (n(&s["index"])?, n(&s["stroke"])?, n(&s["after_cut"])?);
        let j = &s["normal_ns"];
        assert!(
            loads
                .insert(key, [d(&j[0])?, d(&j[1])?, d(&j[2])?, d(&j[3])?])
                .is_none()
        );
    }
    let mut nchecks = 0;
    let mut censor = 0;
    for l in fs::read_to_string(timing_run.join("suffix-cost.jsonl"))?.lines() {
        let v: Value = serde_json::from_str(l)?;
        for t in v["runs"].as_array().ok_or("missing ordinary timed runs")? {
            let which = usize::from(t["suffix_passes"] == 8);
            for (i, flag) in t["loaded"]
                .as_array()
                .ok_or("load observations")?
                .iter()
                .enumerate()
                .skip(1)
            {
                let key = (n(&v["index"])?, n(&v["stroke"])?, i as u64 + 1);
                if let Some(j) = loads.get(&key) {
                    assert_eq!(j[which] > 0., flag.as_bool().ok_or("float load flag")?);
                    nchecks += 1;
                } else {
                    censor += 1;
                }
            }
        }
    }
    let mut counts_checked = 0;
    for (idx, limit) in LIMITS.into_iter().enumerate() {
        let row = &measured["thresholds"][idx];
        assert_eq!(row["threshold_ns"], limit);
        let mut total = 0;
        for bits in 0..16 {
            let count = loads
                .values()
                .filter(|js| (0..4).all(|k| (js[k] > limit) == (bits & (1 << k) != 0)))
                .count();
            assert_eq!(row["joint_masks"][bits], count);
            counts_checked += 1;
            total += count;
        }
        assert_eq!(total, loads.len());
        for votes in 0..3 {
            let vals = loads
                .values()
                .filter(|j| {
                    j[0] <= limit
                        && j[1] > limit
                        && usize::from(j[2] > limit) + usize::from(j[3] > limit) == votes
                })
                .map(|j| j[1]);
            let expected = distribution(vals.collect());
            let actual = &row["extra_load_grouped_by_reference_votes"][votes];
            assert_eq!(expected["n"], actual["n"]);
            for k in ["median", "p95", "max", "sum"] {
                if expected[k].is_null() {
                    assert!(actual[k].is_null());
                } else if (d(&expected[k])? - d(&actual[k])?).abs() > 1e-10 {
                    return Err("load-vote distribution mismatch".into());
                }
            }
        }
    }
    plot(&out.join("suffix-loads.svg"), &measured)?;
    fs::write(
        out.join("readback.json"),
        serde_json::to_vec_pretty(
            &json!({"ordinary_native_timed_load_masks_checked":nchecks,"timed_mask_observations_censored":censor,"unique_public_intervals":loads.len(),"threshold_pattern_counts_checked":counts_checked}),
        )?,
    )?;
    Ok(())
}
