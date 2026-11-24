// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct signed second-landing reactions and compare native row-order controls.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{collections::BTreeSet, env, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn check(valid: bool, message: &str) -> Result<()> {
    if valid { Ok(()) } else { Err(message.into()) }
}
fn read(path: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
fn number(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|n| n.is_finite())
        .ok_or_else(|| "missing finite value".into())
}
fn integer(v: &Value) -> Result<usize> {
    usize::try_from(v.as_u64().ok_or("missing count")?).map_err(Into::into)
}
fn vec3(v: &Value) -> Result<DVec3> {
    let a: [f64; 3] = serde_json::from_value(v.clone())?;
    let p = DVec3::from(a);
    check(p.is_finite(), "nonfinite vector")?;
    Ok(p)
}
fn values(v: &Value) -> Result<&[Value]> {
    v.as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "missing array".into())
}
fn com(frame: &Value) -> Result<DVec3> {
    Ok((vec3(&frame["foot"]["position"])? + vec3(&frame["upper"]["position"])?) * 0.5)
}
fn percentile(sorted: &[f64], frac: f64) -> Option<f64> {
    (!sorted.is_empty()).then(|| sorted[((sorted.len() as f64 * frac).ceil() as usize).max(1) - 1])
}
fn distribution<I: Iterator<Item = f64>>(v: I) -> Value {
    let mut v: Vec<_> = v.collect();
    v.sort_by(f64::total_cmp);
    json!({"count":v.len(),"min":v.first(),"median":percentile(&v,0.5),"p95":percentile(&v,0.95),"max":v.last()})
}
fn signed(r: &Value, n: &str, t: &str) -> Result<DVec3> {
    Ok(vec3(&r[n])? + vec3(&r[t])?)
}
fn signature(v: &Value) -> Result<Vec<usize>> {
    if v.is_null() {
        return Ok(Vec::new());
    }
    values(v)?
        .iter()
        .map(|p| {
            let local = vec3(&p["local_m"])?;
            let q = 2 * usize::from(local.x > 0.) + usize::from(local.y > 0.);
            check(
                p["quadrant"] == q,
                "point quadrant does not match native local coordinates",
            )?;
            Ok(q)
        })
        .collect()
}
fn coordinate(v: &Value) -> Result<(usize, bool)> {
    let k = integer(&v["case"])?;
    let slot = integer(&v["slot"])?;
    check(k < 250 && slot < 50, "coordinate out of study")?;
    let index = k * 117 % 250;
    let expected_offset = 165 + 5 * (index / 10);
    let expected_drive = 76 + 12 * ((index / 2) % 5);
    let band = slot / 10;
    let drive = slot / 2 % 5;
    let held = slot % 2 == 1;
    let f = &v["fixture"];
    check(
        (number(&f["offset_m"])? * 1000.).round() as usize == expected_offset
            && (number(&f["speed_scale"])? * 100.).round() as usize == expected_drive
            && f["switched"] == (index % 2 == 1)
            && expected_offset == 170 + 25 * band + 5 * usize::from(held)
            && expected_drive == 76 + 12 * drive
            && f["switched"] == ((band + drive) % 2 == 1)
            && v["held"] == held,
        "fixture diverged from the prespecified coordinate",
    )?;
    Ok((slot, held))
}
fn compare(a: &Value, b: &Value) -> Result<Value> {
    let x = &a["first_public"];
    let y = &b["first_public"];
    let j = signed(y, "normal_vector_ns", "tangent_vector_ns")?
        - signed(x, "normal_vector_ns", "tangent_vector_ns")?;
    let h = signed(y, "normal_moment_nms", "tangent_moment_nms")?
        - signed(x, "normal_moment_nms", "tangent_moment_nms")?;
    let order_a = signature(&a["first_paid_rows"])?;
    let order_b = signature(&b["first_paid_rows"])?;
    let coords = |record: &Value| -> Result<Vec<(usize, DVec3)>> {
        if record["first_paid_rows"].is_null() {
            return Ok(vec![]);
        }
        let mut rows = Vec::new();
        for p in values(&record["first_paid_rows"])? {
            rows.push((integer(&p["quadrant"])?, vec3(&p["local_m"])?));
        }
        rows.sort_by(|a, b| {
            a.0.cmp(&b.0)
                .then_with(|| a.1.x.total_cmp(&b.1.x))
                .then_with(|| a.1.y.total_cmp(&b.1.y))
                .then_with(|| a.1.z.total_cmp(&b.1.z))
        });
        Ok(rows)
    };
    let (c1, c2) = (coords(a)?, coords(b)?);
    let anchor =
        (c1.len() == c2.len() && !c1.is_empty() && c1.iter().zip(&c2).all(|(p, q)| p.0 == q.0))
            .then(|| {
                c1.iter()
                    .zip(&c2)
                    .map(|(p, q)| p.1.distance(q.1))
                    .fold(0., f64::max)
            });
    let delta_com = (com(&b["end"])? - com(&a["end"])?).length();
    let delta_spin = vec3(&b["end"]["foot"]["angular_velocity"])?
        .distance(vec3(&a["end"]["foot"]["angular_velocity"])?);
    let cop = if x["normal_weighted_point_local_m"].is_null()
        || y["normal_weighted_point_local_m"].is_null()
    {
        None
    } else {
        Some(
            vec3(&x["normal_weighted_point_local_m"])?
                .distance(vec3(&y["normal_weighted_point_local_m"])?),
        )
    };
    Ok(
        json!({"impulse_ns":j.length(),"delta_impulse_on_foot_ns":j.to_array(),"moment_nms":h.length(),"delta_moment_about_com_nms":h.to_array(),
             "signed_normal_ns":number(&y["normal_ns"])?-number(&x["normal_ns"])?,"cop_shift_mm":cop.map(|x|x*1000.),"com_5ms_mm":delta_com*1000.,
             "spin_5ms_rad_s":delta_spin,"same_order":order_a==order_b,"order_a":order_a,"order_b":order_b,"first_anchor_max_m":anchor,
             "first_native_a_ns":a["first_native_ns"],"first_native_b_ns":b["first_native_ns"]}),
    )
}
fn branch<'a>(data: &'a Value, name: &str) -> Result<&'a Value> {
    values(&data["branches"])?
        .iter()
        .find(|b| b["graph"] == name)
        .ok_or_else(|| format!("missing branch {name}").into())
}
const BASIC: [(&str, &str, &str); 2] = [("cache", "retained", "cold"), ("fresh", "cold", "fresh")];
const ORDERS: [(&str, &str, &str); 9] = [
    ("cache", "retained", "cold"),
    ("warm_hook", "retained", "warm_noop"),
    ("cold_hook", "cold", "cold_noop"),
    ("noop", "warm_noop", "cold_noop"),
    ("canonical", "warm_sorted", "cold_sorted"),
    ("exchanged", "warm_exchanged", "cold_exchanged"),
    ("warm_canonical_effect", "warm_noop", "warm_sorted"),
    ("cold_canonical_effect", "cold_noop", "cold_sorted"),
    ("warm_rank_effect", "warm_sorted", "warm_exchanged"),
];
fn verify_branch(dir: &Path, k: usize, source: &Value, b: &Value) -> Result<()> {
    let tag = b["graph"].as_str().ok_or("missing graph name")?;
    let t = read(&dir.join(format!("case-{k:03}/traces/{tag}4096.json")))?;
    let frames = values(&t["frames"])?;
    let cut = integer(&source["incoming"]["cut_tick"])?;
    check(
        frames.len() == 11 && frames[10] == b["end"] && b["grid"] == 4096,
        "missing or inconsistent export",
    )?;
    for (i, f) in frames.iter().enumerate() {
        check(
            f["tick"] == cut + i && f["time_us"] == (cut + i) * 500 && f["native_steps"] == 4096,
            "unexpected public tick or native budget",
        )?;
        let sums = values(&f["corner_ns"])?
            .iter()
            .map(number)
            .collect::<Result<Vec<_>>>()?;
        check(
            sums.len() == 4
                && sums.iter().all(|v| *v >= 0.)
                && (sums.iter().sum::<f64>() - number(&f["normal_impulse_ns"])?).abs() < 1e-8,
            "quadrant sums disagree with public total",
        )?;
    }
    let first = &b["first_public"];
    check(
        frames[1]["first_local_ns"] == b["first_native_ns"]
            && (number(&frames[1]["normal_impulse_ns"])? - number(&first["normal_ns"])?).abs()
                < 1e-9
            && vec3(&frames[1]["tangent_on_foot_ns"])?.distance(vec3(&first["tangent_vector_ns"])?)
                < 1e-9,
        "first paid receipt differs from public trace",
    )?;
    for (i, id) in ["foot", "upper", "pad"].into_iter().enumerate() {
        check(
            frames[0][id] == source["incoming"]["motion"]["links"][i],
            "incoming rigid state changed",
        )?;
    }
    check(
        number(&b["max_linear_balance_ns"])? < 1e-6
            && number(&b["max_angular_balance_nms"])? < 1e-8,
        "point reactions do not balance native momentum",
    )?;
    if !b["row_hook"].is_null() {
        let visits = integer(&b["row_hook"]["nonempty_manifolds_visited"])?;
        let alters = integer(&b["row_hook"]["ordered_manifolds_changed"])?;
        check(
            visits >= alters
                && (visits > 0 || b["first_native_ns"].is_null())
                && (!tag.contains("noop") || alters == 0),
            "invalid modifier visits",
        )?;
        let actual = signature(&b["first_paid_rows"])?;
        if tag.contains("sorted") || tag.contains("exchanged") {
            let ranks = actual
                .iter()
                .map(|&q| {
                    if tag.contains("exchanged") && q >= 2 {
                        q ^ 1
                    } else {
                        q
                    }
                })
                .collect::<Vec<_>>();
            check(
                ranks.is_sorted(),
                "first paid solve did not receive requested row order",
            )?;
        }
    }
    Ok(())
}
fn measure(dir: &Path, src: &Value, ordered: bool) -> Result<Value> {
    let (slot, held) = coordinate(src)?;
    let k = integer(&src["case"])?;
    let mut row =
        json!({"slot":slot,"held":held,"case":k,"status":src["status"],"fixture":src["fixture"]});
    if src["status"] == "censored" {
        row["reason"] = src["reason"].clone();
        return Ok(row);
    }
    check(
        src["status"] == "measured"
            && src["release_observed"] == true
            && src["uninterrupted_samples_checked"] == 11
            && integer(&src["second_tick"])? == integer(&src["incoming"]["cut_tick"])? + 1
            && number(&src["incoming"]["gap_m"])? > 0.,
        "invalid matched airborne cut",
    )?;
    for h in values(&src["incoming"]["cache"])? {
        check(
            [
                "absolute_normal_ns",
                "absolute_tangent_ns",
                "absolute_torsion_nms",
            ]
            .into_iter()
            .all(|key| h[key] == 0.)
                && h["discard_pending"] == false,
            "nonzero seed at the cut",
        )?;
    }
    let mut visits = json!({});
    for b in values(&src["branches"])? {
        verify_branch(dir, k, src, b)?;
        if !b["row_hook"].is_null() {
            visits[b["graph"].as_str().ok_or("missing tag")?] = b["row_hook"].clone();
        }
    }
    row["row_hook_counts"] = visits;
    if ordered {
        let other = read(&dir.parent().ok_or("missing sibling")?.join(format!(
            "multifidelity-secondary-cache-field/cases/case-{k:03}.json"
        )))?;
        check(
            src["incoming"] == other["incoming"] && src["second_tick"] == other["second_tick"],
            "new source no longer reproduces original incoming world",
        )?;
        for tag in ["retained", "cold"] {
            let x = branch(src, tag)?;
            let y = branch(&other, tag)?;
            check(
                x["first_public"] == y["first_public"]
                    && x["end"] == y["end"]
                    && x["first_paid_rows"] == y["first_paid_rows"],
                "original controls changed across binaries",
            )?;
        }
    }
    let contrasts = if ordered { &ORDERS[..] } else { &BASIC[..] };
    for &(key, a, b) in contrasts {
        row[key] = compare(branch(src, a)?, branch(src, b)?)?;
    }
    row["second_tick"] = src["second_tick"].clone();
    row["incoming_gap_m"] = src["incoming"]["gap_m"].clone();
    Ok(row)
}
fn summarize(rows: &[&Value], contrasts: &[(&str, &str, &str)]) -> Value {
    let mut s = json!({"measured":rows.len()});
    for &(label, _, _) in contrasts {
        let mut d = json!({});
        for metric in [
            "impulse_ns",
            "signed_normal_ns",
            "moment_nms",
            "cop_shift_mm",
            "com_5ms_mm",
            "spin_5ms_rad_s",
            "first_anchor_max_m",
        ] {
            d[metric] = distribution(rows.iter().filter_map(|r| r[label][metric].as_f64()));
        }
        d["same_order"] = json!(
            rows.iter()
                .filter(|r| r[label]["same_order"] == true)
                .count()
        );
        for (key, flag) in [("matching_rows", true), ("different_rows", false)] {
            d[key] = distribution(
                rows.iter()
                    .filter(|r| r[label]["same_order"] == flag)
                    .filter_map(|r| r[label]["impulse_ns"].as_f64()),
            );
        }
        s[label] = d;
    }
    s
}
fn draw(path: &Path, data: &[Value], ordered: bool) -> Result<()> {
    let fig = SVGBackend::new(path, (1180, 740)).into_drawing_area();
    fig.fill(&WHITE)?;
    let (head, body) = fig.split_vertically(84);
    head.draw(&Text::new(
        if ordered {
            "Does native row order account for the secondary impulse?"
        } else {
            "How sensitive is the second strike to the collision graph?"
        },
        (32, 35),
        ("sans-serif", 27).into_font(),
    ))?;
    head.draw(&Text::new(format!("{} of 50 fixture coordinates published  ·  4,096 native calls per public update  ·  both histories start at the same measured state",data.len()),(32,66),("sans-serif",17).into_font()))?;
    let cols = body.split_evenly((1, 2));
    for (chart, held) in cols.into_iter().zip([false, true]) {
        let cases: Vec<_> = data
            .iter()
            .filter(|v| v["held"] == held && v["status"] == "measured")
            .collect();
        let metrics: Vec<_> = if ordered {
            vec![
                ("cache", "Native", RGBColor(72, 77, 110)),
                ("canonical", "Same ordered quadrants", RGBColor(0, 139, 112)),
                ("exchanged", "Exchanged positive x", RGBColor(190, 84, 33)),
            ]
        } else {
            vec![
                ("cache", "Rebuilt vs retained", RGBColor(72, 77, 110)),
                ("fresh", "Fresh vs rebuilt", RGBColor(190, 84, 33)),
            ]
        };
        let mut c = ChartBuilder::on(&chart)
            .margin(20)
            .caption(
                format!(
                    "{} · {}/25 eligible",
                    if held {
                        "Held-out offsets"
                    } else {
                        "Calibration offsets"
                    },
                    cases.len()
                ),
                ("sans-serif", 22),
            )
            .x_label_area_size(60)
            .y_label_area_size(68)
            .build_cartesian_2d(0.5f64..25.5f64, (1e-12f64..10.).log_scale())?;
        c.configure_mesh()
            .x_desc("Five drive speeds at each of five support offsets")
            .y_desc("First public impulse-vector difference (N·s)")
            .y_label_formatter(&|y| format!("{y:.0e}"))
            .x_labels(6)
            .draw()?;
        for (j, (key, label, color)) in metrics.into_iter().enumerate() {
            c.draw_series(cases.iter().map(|v| {
                Circle::new(
                    (
                        integer(&v["slot"]).unwrap() as f64 / 2.0 - f64::from(u8::from(held)) * 0.5
                            + 1.
                            + 0.17 * (j as f64 - 1.),
                        v[key]["impulse_ns"].as_f64().unwrap().max(1.1e-12),
                    ),
                    4,
                    color.filled(),
                )
            }))?
            .label(label)
            .legend(move |(x, y)| Circle::new((x + 6, y), 4, color.filled()));
        }
        c.configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .border_style(BLACK.mix(0.15))
            .background_style(WHITE.mix(0.90))
            .draw()?;
    }
    fig.present()?;
    Ok(())
}
fn mechanism(dir: &Path) -> Result<()> {
    let src = read(&dir.join("mechanism.json"))?;
    let k = integer(&src["case"])?;
    let root = dir
        .parent()
        .and_then(Path::parent)
        .ok_or("missing original point study")?;
    let previous = read(&root.join(format!(
        "multifidelity-secondary-contact-points/case-{k:03}/continuous4096r1/points.json"
    )))?;
    check(
        src["selection"].is_string()
            && src["slot"].is_null()
            && src["held"].is_null()
            && src["status"] == "measured",
        "invalid independent mechanism source",
    )?;
    let study = dir.parent().ok_or("missing archive")?;
    for b in values(&src["branches"])? {
        verify_branch(study, k, &src, b)?;
    }
    for key in ["cold", "retained"] {
        let a = branch(&src, key)?;
        let b = values(&previous["branches"])?
            .iter()
            .find(|r| r["graph"] == key && r["grid"] == 4096)
            .ok_or("missing historic native point run")?;
        check(
            a["first_public"] == b["public"][0]["totals"],
            "historic point fields changed",
        )?;
    }
    let mut readback = json!({"case":k,"historical_point_integrals_checked":2,"native_lanes_checked":8,"uninterrupted_samples_checked":src["uninterrupted_samples_checked"]});
    for (key, a, b) in ORDERS {
        readback[key] = compare(branch(&src, a)?, branch(&src, b)?)?;
    }
    fs::write(
        dir.join("readback.json"),
        serde_json::to_vec_pretty(&readback)?,
    )?;
    Ok(())
}
fn main() -> Result<()> {
    let args: Vec<_> = env::args().skip(1).collect();
    if args.first().is_some_and(|s| s == "--mechanism") && args.len() == 2 {
        return mechanism(Path::new(&args[1]));
    }
    check(
        args.len() == 2,
        "secondary_cache_field STUDY_DIR OUTPUT_DIR",
    )?;
    let dir = Path::new(&args[0]);
    let out = Path::new(&args[1]);
    fs::create_dir_all(out)?;
    let mut data = Vec::new();
    let mut slots = BTreeSet::new();
    let mut format = None;
    let mut unmatched = Vec::new();
    for f in fs::read_dir(dir.join("cases"))? {
        let f = f?.path();
        if f.extension().is_none_or(|e| e != "json") {
            continue;
        }
        let v = read(&f)?;
        let tag = v["format"].as_str().ok_or("missing version")?.to_string();
        check(
            matches!(
                tag.as_str(),
                "second-landing-native-row-order-cohort-v1"
                    | "second-landing-native-cache-cohort-v1"
            ) && format.as_ref().is_none_or(|old| *old == tag),
            "incompatible field data",
        )?;
        let ordered = tag.contains("row-order");
        format = Some(tag);
        check(
            slots.insert(integer(&v["slot"])?),
            "duplicate original fixture",
        )?;
        let (slot, _) = coordinate(&v)?;
        let index = integer(&v["case"])?;
        if ordered
            && !dir
                .parent()
                .ok_or("missing sibling field")?
                .join(format!(
                    "multifidelity-secondary-cache-field/cases/case-{index:03}.json"
                ))
                .try_exists()?
        {
            unmatched.push(json!({"slot":slot,"case":index}));
        } else {
            data.push(measure(dir, &v, ordered)?);
        }
    }
    data.sort_by_key(|v| v["slot"].as_u64());
    let ordered = format.ok_or("no cases yet")?.contains("row-order");
    let contrasts = if ordered { &ORDERS[..] } else { &BASIC[..] };
    let mut summary = json!({"published":slots.len(),"joined":data.len(),"pending_original_case":unmatched,"pending":(0..50).filter(|s|!slots.contains(s)).collect::<Vec<_>>(),"ordered":ordered});
    for (key, held) in [("calibration", false), ("holdout", true)] {
        let rows: Vec<_> = data
            .iter()
            .filter(|r| r["held"] == held && r["status"] == "measured")
            .collect();
        summary[key] = summarize(&rows, contrasts);
        summary[key]["censored"] = json!(
            data.iter()
                .filter(|r| r["held"] == held && r["status"] == "censored")
                .map(|r| json!({"case":r["case"],"reason":r["reason"]}))
                .collect::<Vec<_>>()
        );
        if held {
            let worst = rows.iter().copied().max_by(|a, b| {
                a["cache"]["impulse_ns"]
                    .as_f64()
                    .unwrap()
                    .total_cmp(&b["cache"]["impulse_ns"].as_f64().unwrap())
                    .then_with(|| b["case"].as_u64().cmp(&a["case"].as_u64()))
            });
            summary["largest_observed_heldout"] = worst.cloned().unwrap_or(Value::Null);
        }
    }
    draw(&out.join("second-landing.svg"), &data, ordered)?;
    summary["cases"] = json!(data);
    fs::write(
        out.join("readback.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    Ok(())
}
