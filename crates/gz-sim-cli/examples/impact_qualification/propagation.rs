// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
const COUNT: usize = 250;
const NAMES: [&str; 8] = [
    "8r1_then512",
    "8r8_then512",
    "4096r1_then512",
    "4096r8_then512",
    "8r1_then64",
    "4096r1_then64",
    "continuous4096r1",
    "continuous4096r8",
];
fn onset(out: &Value, k: usize) -> Result<Option<u64>> {
    let stroke = array(&out["strokes"])?.get(k).ok_or("missing stroke")?;
    if uint(&stroke["stroke"])? != k as u64 + 1 {
        return Err("stroke label mismatch".into());
    }
    Ok(stroke["first_native_end_ns"].as_u64())
}
pub(super) fn summarize(dir: &Path) -> Result<()> {
    let mut stats: BTreeMap<(String, String, String), Series> = BTreeMap::new();
    let mut counts = BTreeMap::<String, usize>::new();
    let mut seen = BTreeSet::new();
    let mut censored = vec![];
    for file in fs::read_dir(dir.join("cases"))? {
        let f = file?.path();
        if f.extension().is_none_or(|e| e != "json") {
            continue;
        }
        let v = read(&f)?;
        let k = uint(&v["case"])? as usize;
        if v["format"] != "first-impact-propagation-v1" || k >= COUNT || !seen.insert(k) {
            return Err("invalid propagation case".into());
        }
        let r = &v["result"];
        let coord = k * 117 % COUNT;
        let offset = (165 + 5 * (coord / 10)) as f64 / 1000.;
        let speed = (76 + 12 * ((coord / 2) % 5)) as f64 / 100.;
        if (num(&r["fixture"]["offset_m"])? - offset).abs() > 1e-14
            || (num(&r["fixture"]["speed_scale"])? - speed).abs() > 1e-14
            || r["fixture"]["switched"] != (coord % 2 == 1)
        {
            return Err("physical lattice mismatch".into());
        }
        if r["cut"].is_null() {
            censored.push(k);
            continue;
        }
        let paths = array(&r["paths"])?;
        if paths.len() != NAMES.len() {
            return Err("missing control".into());
        }
        let advance = uint(&r["end_tick"])? - uint(&r["cut_tick"])?;
        let classes = [
            "all",
            if coord % 2 == 1 {
                "hold_donor"
            } else {
                "dynamic_donor"
            },
        ];
        for (i, p) in paths.iter().enumerate() {
            let name = NAMES[i];
            if p["profile"] != name {
                return Err("profile order mismatch".into());
            }
            let initial = [8, 8, 4096, 4096, 8, 4096, 4096, 4096][i];
            let follow = [512, 512, 512, 512, 64, 64, 4096, 4096][i];
            if uint(&p["native_calls"])? != initial + follow * (advance - 1) {
                return Err("public time/native budget mismatch".into());
            }
            let mut peers = vec![
                ("uniform4096r1", 6, "vs_uniform4096r1"),
                ("uniform4096r8", 7, "vs_uniform4096r8"),
            ];
            if i < 6 {
                peers.push((
                    "same_suffix",
                    if i < 4 { 2 } else { 5 },
                    "common_suffix_onset_reference",
                ));
            }
            for (peer, j, column) in peers {
                let q = &p[column];
                let other = &paths[j];
                let label = format!("{name}_{peer}");
                let center = |r: &Value| -> Result<[f64; 3]> {
                    let a = array(&r["end"]["foot"]["position"])?;
                    let b = array(&r["end"]["upper"]["position"])?;
                    if a.len() != 3 || b.len() != 3 {
                        return Err("invalid endpoint".into());
                    }
                    Ok([
                        (num(&a[0])? + num(&b[0])?) * 0.5,
                        (num(&a[1])? + num(&b[1])?) * 0.5,
                        (num(&a[2])? + num(&b[2])?) * 0.5,
                    ])
                };
                let x = center(p)?;
                let y = center(other)?;
                let expected = x
                    .into_iter()
                    .zip(y)
                    .map(|(a, b)| (a - b) * (a - b))
                    .sum::<f64>()
                    .sqrt();
                if (num(&q["final_com_m"])? - expected).abs() > 1e-10 {
                    return Err("saved endpoint distance disagrees with links".into());
                }
                for class in classes {
                    let mut push = |metric: String, v: f64| {
                        stats
                            .entry((class.into(), label.clone(), metric))
                            .or_default()
                            .push(v, k)
                    };
                    for field in [
                        "rms_com_m",
                        "max_com_m",
                        "final_com_m",
                        "final_spin_rad_s",
                        "max_public_reaction_difference_ns",
                    ] {
                        push(field.into(), num(&q[field])?);
                    }
                    for row in array(&q["curve"])? {
                        push(
                            format!("com_at_tick_{}", row["public_ticks_after_cut"]),
                            num(&row["com_m"])?,
                        );
                    }
                    for stroke in 1..3 {
                        let a = &p["outcomes"];
                        let b = &other["outcomes"];
                        let tag = format!("{class}_{label}_stroke{}", stroke + 1);
                        match (onset(a, stroke)?, onset(b, stroke)?) {
                            (Some(x), Some(y)) => {
                                *counts.entry(format!("{tag}_both")).or_default() += 1;
                                push(
                                    format!("stroke{}_abs_arrival_us", stroke + 1),
                                    x.abs_diff(y) as f64 * 0.001,
                                );
                                let a = &a["strokes"][stroke];
                                let b = &b["strokes"][stroke];
                                if a["window_public_ticks"] == 10 && b["window_public_ticks"] == 10
                                {
                                    push(
                                        format!("stroke{}_abs_5ms_reaction_ns", stroke + 1),
                                        (num(&a["sum_first_ten_loaded_tick_window_ns"])?
                                            - num(&b["sum_first_ten_loaded_tick_window_ns"])?)
                                        .abs(),
                                    );
                                }
                            }
                            (None, None) => {
                                *counts.entry(format!("{tag}_neither")).or_default() += 1
                            }
                            _ => *counts.entry(format!("{tag}_only_one")).or_default() += 1,
                        }
                    }
                }
            }
        }
    }
    let rows=stats.into_iter().map(|((group,comparison,metric),v)|json!({"group":group,"comparison":comparison,"metric":metric,"statistics":v.stats()})).collect::<Vec<_>>();
    write(
        &dir.join("summary.json"),
        &json!({"complete":seen.len()==COUNT,"cases":seen.len(),"censored_before_first_primary":censored,"rows":rows,"paired_return_counts":counts}),
    )
}
