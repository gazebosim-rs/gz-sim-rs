// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Match three-stroke motion, signed arrival shifts and finite-footprint status at every field coordinate.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn real(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|v| v.is_finite())
        .ok_or_else(|| "missing finite measurement".into())
}
fn n(v: &Value) -> Result<usize> {
    v.as_u64()
        .and_then(|x| x.try_into().ok())
        .ok_or_else(|| "missing fixture coordinate".into())
}
fn arr(v: &Value) -> Result<&Vec<Value>> {
    v.as_array().ok_or_else(|| "missing series".into())
}
fn vec(v: &Value) -> Result<DVec3> {
    let v: [f64; 3] = serde_json::from_value(v.clone())?;
    Ok(DVec3::from(v))
}
fn pick<'a>(p: &'a Value, key: &str) -> Result<&'a Value> {
    arr(p)?
        .iter()
        .find(|v| v["profile"] == key)
        .ok_or_else(|| format!("missing native lane {key}").into())
}
fn stats(mut v: Vec<f64>) -> Value {
    v.sort_by(f64::total_cmp);
    let n = v.len();
    let q = |p: usize| v.get((n * p).div_ceil(100).saturating_sub(1));
    json!({"n":n,"mean":(!v.is_empty()).then(||v.iter().sum::<f64>()/n as f64),"min":v.first(),"median":q(50),"p95":q(95),"max":v.last()})
}
struct Row {
    id: usize,
    x: usize,
    y: usize,
    holds: bool,
    final_mm: f64,
    rms_mm: f64,
    max_mm: f64,
    start_tick: usize,
    end_tick: usize,
    any_loss: [bool; 2],
    final_support: [bool; 2],
    time_ms: [[Option<f64>; 3]; 2],
    window_ns: [[Option<f64>; 3]; 2],
    thresholds: [Option<usize>; 4],
}
impl Row {
    fn times(&self, stroke: usize) -> Option<f64> {
        Some(self.time_ms[0][stroke]? - self.time_ms[1][stroke]?)
    }
    fn jumps(&self, stroke: usize) -> Option<f64> {
        Some(self.window_ns[0][stroke]? - self.window_ns[1][stroke]?)
    }
}
fn pair(doc: &Value, k: usize, deep: bool) -> Result<Option<Row>> {
    let d = &doc["result"];
    let tests = if deep { 150 } else { 250 };
    let speeds = if deep { 3 } else { 5 };
    let c = k * if deep { 71 } else { 117 } % tests;
    if d["cut"].is_null() {
        return Ok(None);
    }
    let f = &d["fixture"];
    let x = c / (2 * speeds);
    let y = (c / 2) % speeds;
    let axes_ok = n(&doc["case"])? == k
        && real(&f["offset_m"])? == (165 + 5 * x) as f64 / 1000.
        && real(&f["speed_scale"])? == (76 + (if deep { 24 } else { 12 }) * y) as f64 / 100.
        && f["switched"] == (c % 2 == 1);
    if !axes_ok {
        return Err("long field coordinate does not match native fixture".into());
    }
    let (a, b) = if deep {
        ("4096", "8192")
    } else {
        ("8r1_then512", "4096r1_then512")
    };
    let a = pick(&d["paths"], a)?;
    let b = pick(&d["paths"], b)?;
    let cut = n(&d["cut_tick"])?;
    let end = n(&d["end_tick"])?;
    let mut pos = [DVec3::ZERO; 2];
    let mut time = [[None; 3]; 2];
    let mut window = [[None; 3]; 2];
    let mut any_loss = [false; 2];
    let mut final_support = [false; 2];
    let mut spins = [DVec3::ZERO; 2];
    for (j, p) in [a, b].into_iter().enumerate() {
        let e = &p["end"];
        if n(&e["tick"])? != end || n(&e["time_us"])? != end * 500 {
            return Err("mismatched final public clock".into());
        }
        pos[j] = (vec(&e["foot"]["position"])? + vec(&e["upper"]["position"])?) * 0.5;
        spins[j] = vec(&e["foot"]["angular_velocity"])?;
        let ev = &p[if deep { "events" } else { "outcomes" }];
        any_loss[j] = !ev["first_disjoint_tick"].is_null();
        final_support[j] = !e["footprint_gap_m"].is_null();
        for (s, t) in arr(&ev["strokes"])?.iter().enumerate() {
            if s >= 3 || t["stroke"] != s + 1 {
                return Err("out-of-order actuator stroke".into());
            }
            if let Some(ns) = t["first_native_end_ns"].as_u64() {
                if t["released"] != true
                    || ns < (cut * 500_000) as u64
                    || ns > (end * 500_000) as u64
                {
                    return Err("invalid later native arrival".into());
                }
                time[j][s] = Some(ns as f64 * 1e-6);
                if t["window_public_ticks"] == 10 {
                    window[j][s] = Some(real(&t["sum_first_ten_loaded_tick_window_ns"])?);
                }
            }
        }
    }
    let comparison = if deep {
        &a["against"][2]
    } else {
        &a["common_suffix_onset_reference"]
    };
    let dist = pos[0].distance(pos[1]);
    if (dist - real(&comparison["final_com_m"])?).abs() > 1e-11
        || (spins[0].distance(spins[1]) - real(&comparison["final_spin_rad_s"])?).abs() > 1e-10
    {
        return Err("final native motion and archived trajectory comparison disagree".into());
    }
    let thresholds: [Option<usize>; 4] =
        serde_json::from_value(comparison["first_above_1um_100um_1mm_10mm_ticks"].clone())?;
    Ok(Some(Row {
        id: k,
        x,
        y,
        holds: c % 2 == 1,
        final_mm: 1000. * dist,
        rms_mm: 1000. * real(&comparison["rms_com_m"])?,
        max_mm: 1000. * real(&comparison["max_com_m"])?,
        start_tick: cut,
        end_tick: end,
        any_loss,
        final_support,
        time_ms: time,
        window_ns: window,
        thresholds,
    }))
}
fn plots(rows: &[Row], censored: &[usize], deep: bool, dir: &Path) -> Result<()> {
    let speeds = if deep { 3 } else { 5 };
    let total = 50 * speeds;
    let path = dir.join("three-stroke-field.svg");
    let root = SVGBackend::new(&path, (1380, 840)).into_drawing_area();
    root.fill(&WHITE)?;
    let caption = if deep {
        "Continuous 4,096 vs 8,192 native calls"
    } else {
        "Eight vs 4,096 calls at first impact, identical later 512-call grids"
    };
    root.draw(&Text::new(
        caption,
        (32, 37),
        ("sans-serif", 24).into_font(),
    ))?;
    root.draw(&Text::new(format!("{} of {total} paired terminal states • {} censored donors • gray = unpublished  •  × = missing donor or return",rows.len(),censored.len()),(33,68),("sans-serif",15).into_font()))?;
    let heights = [
        (115, "RMS COM separation (mm)"),
        (360, "Final COM separation (mm)"),
        (605, "Absolute third-return shift (ms)"),
    ];
    let lookup = rows
        .iter()
        .map(|r| ((r.x, r.y, r.holds), r))
        .collect::<BTreeMap<_, _>>();
    for (metric, (top, name)) in heights.into_iter().enumerate() {
        let value = |r: &Row| match metric {
            0 => Some(r.rms_mm),
            1 => Some(r.final_mm),
            _ => r.times(2).map(f64::abs),
        };
        let mut vals = rows.iter().filter_map(value).collect::<Vec<_>>();
        vals.sort_by(f64::total_cmp);
        let max = vals
            .get((vals.len() * 95).div_ceil(100).saturating_sub(1))
            .copied()
            .unwrap_or(1.)
            .max(1e-9);
        root.draw(&Text::new(
            format!("{name}   •   color saturates at p95 {max:.3}"),
            (32, top - 4),
            ("sans-serif", 17).into_font(),
        ))?;
        for (side, held) in [false, true].into_iter().enumerate() {
            let left = 90 + side as i32 * 670;
            root.draw(&Text::new(
                if held {
                    "Hold-eligible donor"
                } else {
                    "Always-dynamic donor"
                },
                (left, top + 19),
                ("sans-serif", 14).into_font(),
            ))?;
            for y in 0..speeds {
                for x in 0..25 {
                    let tx = left + x as i32 * 23;
                    let ty = top + 32 + y as i32 * 25;
                    let sample = lookup.get(&(x, y, held));
                    let v = sample.and_then(|r| value(r));
                    let color = v.map_or(RGBColor(221, 224, 225), |v| {
                        let t = (v / max).clamp(0., 1.);
                        RGBColor(
                            (241. - 205. * t) as u8,
                            (247. - 130. * t) as u8,
                            (243. - 126. * t) as u8,
                        )
                    });
                    root.draw(&Rectangle::new(
                        [(tx, ty), (tx + 22, ty + 24)],
                        color.filled(),
                    ))?;
                    let coord = 2 * speeds * x + 2 * y + usize::from(held);
                    let censored = censored
                        .iter()
                        .any(|&id| id * (if deep { 71 } else { 117 }) % total == coord);
                    if v.is_none() && (sample.is_some() || censored) {
                        root.draw(&Text::new(
                            "×",
                            (tx + 7, ty + 17),
                            ("sans-serif", 13).into_font(),
                        ))?;
                    }
                    if x == 0 {
                        root.draw(&Text::new(
                            format!("{}%", 76 + y * (if deep { 24 } else { 12 })),
                            (left - 48, ty + 17),
                            ("sans-serif", 12).into_font(),
                        ))?;
                    }
                    if y == speeds - 1 && x % 4 == 0 {
                        root.draw(&Text::new(
                            format!("{}", 165 + 5 * x),
                            (tx - 2, ty + 40),
                            ("sans-serif", 12).into_font(),
                        ))?;
                    }
                }
            }
            root.draw(&Text::new(
                "Support offset (mm)",
                (left + 200, top + 32 + speeds as i32 * 25 + 37),
                ("sans-serif", 13).into_font(),
            ))?;
        }
    }
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let args = env::args().skip(1).collect::<Vec<_>>();
    if args.len() != 3 {
        return Err("three_stroke_fields propagation|convergence DATA_DIR OUTPUT".into());
    }
    let deep = match args[0].as_str() {
        "propagation" => false,
        "convergence" => true,
        _ => return Err("unknown comparison".into()),
    };
    let total = if deep { 150 } else { 250 };
    let dir = Path::new(&args[1]);
    let out = Path::new(&args[2]);
    fs::create_dir_all(out)?;
    let mut rows = vec![];
    let mut pending = vec![];
    let mut censored = vec![];
    for k in 0..total {
        let p = dir.join("cases").join(format!("case-{k:03}.json"));
        if !p.exists() {
            pending.push(k);
            continue;
        }
        let d: Value = serde_json::from_slice(&fs::read(p)?)?;
        if d["format"]
            != if deep {
                "continuous-three-stroke-grid-v1"
            } else {
                "first-impact-propagation-v1"
            }
        {
            return Err("unexpected study version".into());
        }
        if let Some(row) = pair(&d, k, deep)? {
            rows.push(row);
        } else {
            censored.push(k);
        }
    }
    let worst = rows
        .iter()
        .max_by(|a, b| a.final_mm.total_cmp(&b.final_mm).then(b.id.cmp(&a.id)))
        .ok_or("no physical endpoints")?;
    let mut windows = vec![];
    for s in 0..3 {
        let dt = rows.iter().filter_map(|r| r.times(s)).collect::<Vec<_>>();
        let impulse = rows.iter().filter_map(|r| r.jumps(s)).collect::<Vec<_>>();
        let mut masks = [[0; 2]; 2];
        let mut by_dt = [Vec::new(), Vec::new()];
        for r in &rows {
            masks[usize::from(r.time_ms[0][s].is_some())]
                [usize::from(r.time_ms[1][s].is_some())] += 1;
            if let Some(dt) = r.times(s) {
                by_dt[usize::from(dt.abs() > 0.5)].push(r.final_mm);
            }
        }
        windows.push(json!({"stroke":s+1,"arrival_present_first_by_second":masks,"signed_time_delta_ms":stats(dt.clone()),
            "absolute_time_delta_ms":stats(dt.into_iter().map(f64::abs).collect()),"signed_complete_5ms_reaction_delta_ns":stats(impulse),
            "terminal_com_mm_for_paired_arrivals_within_or_beyond_one_tick":by_dt.map(stats)}));
    }
    let mut support = [[0; 2]; 2];
    let mut ever = [[0; 2]; 2];
    for r in &rows {
        support[usize::from(r.final_support[0])][usize::from(r.final_support[1])] += 1;
        ever[usize::from(r.any_loss[0])][usize::from(r.any_loss[1])] += 1;
    }
    let curve = [0, 1, 2, 3].map(|k| {
        stats(
            rows.iter()
                .filter_map(|r| r.thresholds[k].map(|tick| (tick - r.start_tick) as f64 * 0.5))
                .collect(),
        )
    });
    let result = json!({"comparison":args[0],"scheduled":total,"published":total-pending.len(),"missing_donors":censored,"unpublished":pending,"paired_terminal_states":rows.len(),"selection_final":pending.is_empty(),
        "terminal_com_mm":stats(rows.iter().map(|r|r.final_mm).collect()),"rms_com_mm":stats(rows.iter().map(|r|r.rms_mm).collect()),"peak_com_mm":stats(rows.iter().map(|r|r.max_mm).collect()),
        "ever_footprint_disjoint":ever,"supported_at_endpoint":support,"milliseconds_to_1um_100um_1mm_10mm":curve,"paired_strokes":windows,
        "selected_case":{"index":worst.id,"offset_mm":165+5*worst.x,"drive_percent":76+worst.y*if deep{24}else{12},"held":worst.holds,"cut":worst.start_tick,"end":worst.end_tick,
        "final_com_mm":worst.final_mm,"rms_com_mm":worst.rms_mm,"peak_com_mm":worst.max_mm,"event_times_ms":worst.time_ms,"first_5ms_reactions_ns":worst.window_ns}});
    fs::write(
        out.join("three-stroke-field.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    plots(&rows, &censored, deep, out)
}
