// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent matched-donor readback of first transfer and causal native timing.
use super::*;
use glam::DVec3;
type Rows = BTreeMap<(String, String, String), Series>;
pub(super) fn vector(v: &Value) -> Result<DVec3> {
    let [x, y, z] = array(v)? else {
        return Err("missing vector".into());
    };
    Ok(DVec3::new(num(x)?, num(y)?, num(z)?))
}
pub(super) fn fixture(dir: &Path, k: usize) -> Result<Value> {
    read(&dir.join("cases").join(format!("case-{k:04}.json")))
}
fn rows(s: Rows) -> Value {
    json!(s.into_iter().map(|((group,profile,metric),v)|json!({"group":group,"profile":profile,"metric":metric,"statistics":v.stats()})).collect::<Vec<_>>())
}
fn inc(c: &mut BTreeMap<String, usize>, k: impl Into<String>) {
    *c.entry(k.into()).or_default() += 1
}
pub(super) fn matched<'a>(d: &'a Value, t: &Value) -> Result<&'a Value> {
    let hit = array(&d["result"]["trials"])?
        .iter()
        .find(|v| v["stroke"] == t["stroke"])
        .ok_or("missing donor")?;
    if hit["cut"] != t["cut"] {
        return Err("cut mismatch".into());
    }
    Ok(hit)
}
fn put(s: &mut Rows, g: &str, p: &str, m: &str, v: f64, k: usize) {
    s.entry((g.into(), p.into(), m.into()))
        .or_default()
        .push(v, k)
}
fn window(r: &Value) -> Result<[f64; 2]> {
    let [a, b] = array(&r["elapsed_interval_s"])? else {
        return Err("missing transfer interval".into());
    };
    let (a, b) = (num(a)?, num(b)?);
    if a < 0. || b < a || b - a > 2e-13 {
        return Err("event interval out of bounds".into());
    }
    Ok([a, b])
}
pub(super) fn mechanics(dir: &Path, load: &Path) -> Result<()> {
    let mut s = Rows::new();
    let mut counts = BTreeMap::new();
    for k in 0..1250 {
        let f = fixture(dir, k)?;
        let donor = fixture(load, k)?;
        if f["format"] != "finite-first-contact-transfer-v1"
            || f["result"]["fixture"] != donor["result"]["fixture"]
        {
            return Err("unexpected transfer archive".into());
        }
        for t in array(&f["result"]["trials"])? {
            let prior = &matched(&donor, t)?["response"];
            let p = &t["response"];
            let geo = &p["geometry"];
            if *geo != prior["geometry"] {
                return Err("first physical transaction mismatch".into());
            }
            let key = 3 * k + uint(&t["stroke"])? as usize - 1;
            let Some(runs) = p["runs"].as_array() else {
                inc(&mut counts, "without_arc");
                continue;
            };
            if runs.len() != 3 {
                return Err("missing refined arcs".into());
            }
            let base = &runs[2];
            let hi = window(base)?[1];
            let mode = prior["load"]["mode"]
                .as_str()
                .ok_or("no initial force mode")?;
            let end = base["end"].as_str().ok_or("no terminal mode")?;
            let feature = geo["feature"]
                .as_str()
                .ok_or("no geometry")?
                .split('(')
                .next()
                .unwrap();
            inc(&mut counts, format!("{feature}_{mode}_{end}"));
            inc(&mut counts, "arcs");
            let last = (500000 - uint(&geo["hi_ns"])?) as f64 * 1e-9;
            if hi > last + 1e-13 {
                return Err("arc overshot public input boundary".into());
            }
            let hint = &p["linearized"];
            if !hint.is_null() {
                let expect = format!("Feature({})", hint["limit"].as_str().unwrap());
                if end.starts_with("Feature(") {
                    if expect == end {
                        inc(&mut counts, "linear_feature_agreement")
                    }
                    let error = (num(&hint["time_s"])? - hi).abs() * 1e9;
                    put(&mut s, "all", "linear", "time_error_ns", error, key);
                }
            }
            if end.starts_with("Feature(") {
                inc(&mut counts, "feature_changed");
                put(&mut s, "all", "128", "until_transfer_us", hi * 1e6, key)
            }
            let v0 = vector(&prior["outgoing_com_relative_m_s"])?;
            let w0 = vector(&prior["outgoing_spin_rad_s"])?;
            for (r, n) in runs.iter().zip([8, 32, 128]) {
                if r["subdivisions"] != n {
                    return Err("wrong bound-arc work".into());
                }
                let &t = window(r)?.first().unwrap();
                let v = vector(&r["velocity_m_s"])?;
                let w = vector(&r["spin_rad_s"])?;
                let j = vector(&r["linear_impulse_ns"])?;
                let l = vector(&r["angular_impulse_nms"])?;
                let dj = (2. * (v - v0 + t * 9.81 * DVec3::Z) - j).length();
                let dl = (0.05 * (w - w0) - l).length();
                if (dj - num(&r["linear_balance_ns"])?).abs() > 1e-12
                    || (dl - num(&r["angular_balance_nms"])?).abs() > 1e-12
                    || num(&r["friction_work_j"])? > 1e-10
                {
                    return Err("momentum or work mismatch".into());
                }
                let label = n.to_string();
                if r["end"] != base["end"] {
                    inc(&mut counts, format!("{label}_transition_disagreement"))
                }
                for (field, v) in [
                    ("time_change_vs128_ns", (hi - window(r)?[1]).abs() * 1e9),
                    (
                        "com_change_vs128_m",
                        vector(&r["com_m"])?.distance(vector(&base["com_m"])?),
                    ),
                    ("max_momentum_residual_ns", dj),
                    ("angular_residual_nms", dl),
                    ("abs_energy_balance_j", num(&r["energy_balance_j"])?.abs()),
                    (
                        "abs_normal_velocity_m_s",
                        num(&r["normal_speed_m_s"])?.abs(),
                    ),
                    ("abs_constraint_gap_m", num(&r["signed_gap_m"])?.abs()),
                ] {
                    for group in ["all", feature, mode] {
                        put(&mut s, group, &label, field, v, key)
                    }
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"rows":rows(s)}),
    )
}
pub(super) fn policy(
    dir: &Path,
    mechanics: &Path,
    primary: &Path,
    grid: &Path,
    relax: &Path,
) -> Result<()> {
    let mut s = Rows::new();
    let mut counts = BTreeMap::new();
    let mut worst = None;
    for k in 0..1250 {
        let f = fixture(dir, k)?;
        let arc = fixture(mechanics, k)?;
        let base = fixture(primary, k)?;
        let one = fixture(grid, k)?;
        let eight = fixture(relax, k)?;
        if f["format"] != "finite-retimed-transfer-v1"
            || [&arc, &base, &one, &eight]
                .into_iter()
                .any(|r| r["result"]["fixture"] != f["result"]["fixture"])
        {
            return Err("policy fixture mismatch".into());
        }
        for t in array(&f["result"]["trials"])? {
            let stroke = uint(&t["stroke"])?;
            let key = 3 * k + stroke as usize - 1;
            let a = &matched(&arc, t)?["response"];
            let b = matched(&base, t)?;
            let r1 = matched(&one, t)?;
            let r8 = matched(&eight, t)?;
            if a["geometry"] != t["geometry"] || b["geometry"] != t["geometry"] {
                return Err("physical first impact mismatch".into());
            }
            let paths = array(&t["paths"])?;
            if paths.len() != 8 || paths[2] != b["paths"][2] || paths[6] != b["paths"][3] {
                return Err("independent finite8/64 histories changed".into());
            }
            inc(&mut counts, "trials");
            for (refname, samples) in [
                (
                    "4096r1",
                    array(&r1["paths"])?
                        .iter()
                        .find(|p| p["method"] == "uniform" && p["calls"] == 4096)
                        .map(|p| &p["record"]["samples"]),
                ),
                (
                    "4096r8",
                    array(&r8["paths"])?
                        .iter()
                        .find(|p| p["profile"]["calls"] == 4096 && p["profile"]["relaxation"] == 8)
                        .map(|p| &p["samples"]),
                ),
            ] {
                let refs = array(samples.ok_or("missing fine independent trajectory")?)?;
                for (i, pair) in paths.chunks_exact(2).enumerate() {
                    let n = [4, 8, 16, 64][i];
                    let x = &pair[0];
                    let y = &pair[1];
                    if x["native_calls"] != n + 25 * 8 || y["native_calls"] != x["native_calls"] {
                        return Err("transfer budget changed".into());
                    }
                    let h = array(&y["impacts"])?.first().map(|p| &p["retimed"]);
                    let active = h.is_some_and(|h| !h.is_null());
                    if let Some(h) = h.filter(|p| !p.is_null()) {
                        let step = uint(&h["after_impact_ns"])?;
                        let pred = num(&a["linearized"]["time_s"])? * 1e9;
                        if h["limit"] != a["linearized"]["limit"]
                            || ((step as f64) - pred).abs() > 1.00005
                        {
                            return Err(format!(
                                "incorrect causal transfer estimate {k}/{stroke}/{n}"
                            )
                            .into());
                        }
                    } else if x["samples"] != y["samples"] || x["impacts"] != y["impacts"] {
                        return Err("unselected suffix changed".into());
                    }
                    let first = array(&x["impacts"])?.first();
                    let applied = array(&y["impacts"])?.first();
                    for metric in ["impulse_ns", "after", "normal", "local_ns", "feature"] {
                        if first.map(|p| &p[metric]) != applied.map(|p| &p[metric]) {
                            return Err("changed onset intervention".into());
                        }
                    }
                    let name = format!("retimed{n}_vs_{refname}");
                    if active && refname == "4096r1" {
                        inc(&mut counts, format!("retimed{n}"))
                    }
                    for (j, r) in refs.iter().enumerate() {
                        let (x, y) = (&x["samples"][j], &y["samples"][j]);
                        if x["i"] != y["i"] || y["i"] != r["i"] {
                            return Err("mismatched physical horizon".into());
                        }
                        let tick = uint(&r["i"])?;
                        let target = vector(&r["com_m"])?;
                        let old = vector(&x["com_m"])?.distance(target);
                        let new = vector(&y["com_m"])?.distance(target);
                        let change = new - old;
                        for group in [
                            "all",
                            if k % 5 == 4 { "holdout" } else { "calibration" },
                            if active { "retimed" } else { "unchanged" },
                        ] {
                            put(&mut s, group, &name, &format!("tick{tick}_com_m"), new, key);
                            put(
                                &mut s,
                                group,
                                &name,
                                &format!("tick{tick}_change_m"),
                                change,
                                key,
                            );
                            if tick == 26 {
                                let target = vector(&r["cumulative_tangent_ns"])?;
                                put(
                                    &mut s,
                                    group,
                                    &name,
                                    "tangent_error_ns",
                                    vector(&y["cumulative_tangent_ns"])?.distance(target),
                                    key,
                                );
                                put(
                                    &mut s,
                                    group,
                                    &name,
                                    "normal_error_ns",
                                    (num(&y["cumulative_normal_ns"])?
                                        - num(&r["cumulative_normal_ns"])?)
                                    .abs(),
                                    key,
                                );
                            }
                        }
                        if tick == 26 {
                            if change < 0. {
                                inc(&mut counts, format!("{name}_improved"))
                            }
                            if n == 8
                                && refname == "4096r1"
                                && k % 5 == 4
                                && worst.is_none_or(|(z, _)| change > z)
                            {
                                worst = Some((change, key))
                            }
                        }
                    }
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"worst_heldout_eight_change_and_case":worst,"rows":rows(s)}),
    )
}

pub(super) fn second(dir: &Path, arcs: &Path) -> Result<()> {
    let mut counts = BTreeMap::new();
    let mut s = Rows::new();
    let mut worst = None;
    for k in 0..1250 {
        let f = fixture(dir, k)?;
        let a = fixture(arcs, k)?;
        if f["format"] != "finite-second-contact-active-site-v2"
            || f["result"]["fixture"] != a["result"]["fixture"]
        {
            return Err("invalid matched secondary archive".into());
        }
        for t in array(&f["result"]["trials"])? {
            let p = &t["response"];
            let prior = &matched(&a, t)?["response"];
            let key = k * 3 + uint(&t["stroke"])? as usize - 1;
            let mut arc = prior["runs"][2].clone();
            if let Some(row) = arc.as_object_mut() {
                row.remove("subdivisions");
            }
            if p["geometry"] != prior["geometry"] || p["arc_end"] != arc {
                return Err("secondary contact did not reproduce measured arc".into());
            }
            inc(&mut counts, "cuts");
            let sg = &p["segment"];
            if sg.is_null() {
                continue;
            }
            inc(&mut counts, "finite_segments");
            let n = vector(&sg["normal"])?;
            let a = [vector(&sg["points_m"][0])?, vector(&sg["points_m"][1])?];
            let b = [vector(&sg["support_m"][0])?, vector(&sg["support_m"][1])?];
            if a[0].distance(a[1]) < 2e-10
                || n.dot(a[1] - a[0]).abs() > 4e-10
                || a.iter().zip(b).any(|(x, y)| x.distance(y) > 2e-10)
            {
                return Err("invalid incident line".into());
            }
            let v = vector(&p["arc_end"]["velocity_m_s"])?;
            let w = vector(&p["arc_end"]["spin_rad_s"])?;
            let center = vector(&p["arc_end"]["com_m"])?;
            let h = &p["impact"];
            if h.is_null() {
                inc(&mut counts, "unresolved_pressure");
                continue;
            }
            let q = uint(&h["loaded_end"])? as usize;
            if q >= 2 {
                return Err("invalid load site".into());
            }
            inc(&mut counts, "certified_endpoint");
            inc(
                &mut counts,
                if h["sticking"] == true {
                    "sticking"
                } else {
                    "sliding"
                },
            );
            let j = vector(&h["impulse_ns"])?;
            let v2 = v + 0.5 * j;
            let w2 = w + 20. * (a[q] - center).cross(j);
            let energy = v.length_squared() - v2.length_squared()
                + 0.025 * (w.length_squared() - w2.length_squared());
            if v2.distance(vector(&h["com_velocity_after"])?) > 1e-11
                || w2.distance(vector(&h["spin_after"])?) > 1e-11
                || (energy - num(&h["work_loss_j"])?).abs() > 2e-12
                || energy < -1e-12
            {
                return Err("physical endpoint momentum/work disagreement".into());
            }
            for i in 0..2 {
                let u = v2 + w2.cross(a[i] - center);
                if u.distance(vector(&h["velocity_after"][i])?) > 1e-11
                    || (i == q && u.dot(n).abs() > 1e-10)
                    || (i != q && u.dot(n) + 1e-12 < num(&h["opening_bound_m_s"])?)
                {
                    return Err("incorrect endpoint active set".into());
                }
            }
            put(
                &mut s,
                "all",
                "physical",
                "normal_impulse_ns",
                j.dot(n),
                key,
            );
            put(
                &mut s,
                "all",
                "physical",
                "opening_lower_bound_m_s",
                num(&h["opening_bound_m_s"])?,
                key,
            );
            for (r, (dur, pass)) in array(&p["native"])?.iter().zip([
                (1, 1),
                (1, 8),
                (128, 1),
                (128, 8),
                (2048, 1),
                (2048, 8),
            ]) {
                if r["duration_ns"] != dur
                    || r["relaxation"] != pass
                    || vector(&r["before_v"])?.distance(v) > 2e-12
                    || vector(&r["before_w"])?.distance(w) > 2e-12
                {
                    return Err("unmatched native reload".into());
                }
                let got = vector(&r["j_ns"])?;
                let mut sum = DVec3::ZERO;
                for z in array(&r["sites"])? {
                    sum += vector(&z["j_ns"])?;
                }
                if sum.distance(got) > 1e-11 {
                    return Err("missing site impulse".into());
                }
                let name = format!("native_{dur}ns_r{pass}");
                let error = got.distance(j);
                if dur == 1 && pass == 1 && worst.is_none_or(|(old, _)| error > old) {
                    worst = Some((error, key))
                }
                for (m, value) in [
                    ("impulse_error_ns", error),
                    ("normal_error_ns", (num(&r["normal_ns"])? - j.dot(n)).abs()),
                    ("spin_error_rad_s", vector(&r["after_w"])?.distance(w2)),
                    ("velocity_error_m_s", vector(&r["after_v"])?.distance(v2)),
                    (
                        "momentum_residual_ns",
                        (2. * (vector(&r["after_v"])? - v + DVec3::Z * (9.81 * dur as f64 * 1e-9))
                            - got)
                            .length(),
                    ),
                ] {
                    for g in [
                        "all",
                        if h["sticking"] == true {
                            "sticking"
                        } else {
                            "sliding"
                        },
                    ] {
                        put(&mut s, g, &name, m, value, key)
                    }
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"worst_native_1ns_and_case":worst,"rows":rows(s)}),
    )
}
// Condition the same mechanical endpoint on a narrower, independently integrated
// event bracket. Strings, flags, cardinalities, keys and discrete modes must match.
fn nearby(a: &Value, b: &Value) -> bool {
    if a == b {
        return true;
    }
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => x
            .as_f64()
            .zip(y.as_f64())
            .is_some_and(|(l, r)| (l - r).abs() < 1e-10),
        (Value::Array(x), Value::Array(y)) => {
            x.len() == y.len() && x.iter().zip(y).all(|(l, r)| nearby(l, r))
        }
        (Value::Object(x), Value::Object(y)) => {
            x.len() == y.len()
                && x.iter()
                    .all(|(k, l)| y.get(k).is_some_and(|r| nearby(l, r)))
        }
        _ => false,
    }
}
pub(super) fn registered(dir: &Path, mechanics: &Path, coarse: Option<&Path>) -> Result<()> {
    let mut counts = BTreeMap::new();
    let mut series = Rows::new();
    for k in 0..1250 {
        if !dir.join("cases").join(format!("case-{k:04}.json")).exists() {
            continue;
        }
        let d = fixture(dir, k)?;
        let base = fixture(mechanics, k)?;
        let older = coarse.map(|p| fixture(p, k)).transpose()?;
        let expected = if coarse.is_some() {
            "finite-second-registered-tight-v1"
        } else {
            "finite-second-registered-v1"
        };
        if d["format"] != expected || d["result"]["fixture"] != base["result"]["fixture"] {
            return Err("invalid registered source".into());
        }
        inc(&mut counts, "fixtures");
        for t in array(&d["result"]["trials"])? {
            let index = 3 * k + uint(&t["stroke"])? as usize - 1;
            let r = &t["response"];
            let ordinary = &matched(&base, t)?["response"];
            for field in [
                "geometry",
                "arc_end",
                "segment",
                "velocity_before",
                "impact",
            ] {
                if r[field] != ordinary[field]
                    && !(older.is_some() && nearby(&r[field], &ordinary[field]))
                {
                    return Err(
                        format!("different second incoming state for {index}: {field}").into(),
                    );
                }
            }
            inc(&mut counts, "donors");
            if r["segment"].is_null() {
                continue;
            }
            let predicted = &r["impact"];
            let runs = array(&r["registered"])?;
            let [two, eight, sixtyfour] = runs else {
                return Err("missing registered control".into());
            };
            inc(&mut counts, "incident_segments");
            if !predicted.is_null() {
                inc(&mut counts, "pressure_certified")
            }
            for (v, calls) in [(two, 2), (eight, 8), (sixtyfour, 64)] {
                if v["calls"] != calls
                    || v["projected"] != two["projected"]
                    || v["scheduled"] != two["scheduled"]
                {
                    return Err(
                        "call budget changed admission or the immediate pressure impulse".into(),
                    );
                }
            }
            let h = &two["projected"];
            if let Some(old) = &older {
                let orig = &matched(old, t)?["response"];
                if orig["geometry"] != r["geometry"] || !nearby(&orig["arc_end"], &r["arc_end"]) {
                    return Err("cold-start boundary mismatch".into());
                }
                let was = !orig["registered"][0]["projected"].is_null();
                if was && h.is_null() {
                    return Err("narrower incident root lost an existing certificate".into());
                }
                if !was && !h.is_null() {
                    inc(&mut counts, "new_after_bisection")
                }
                let a = &orig["arc_end"];
                let b = &r["arc_end"];
                for (metric, value) in [
                    (
                        "time_change_s",
                        (num(&a["elapsed_interval_s"][0])? - num(&b["elapsed_interval_s"][0])?)
                            .abs(),
                    ),
                    (
                        "foot_change_m",
                        vector(&a["foot_m"])?.distance(vector(&b["foot_m"])?),
                    ),
                    (
                        "spin_change_rad_s",
                        vector(&a["spin_rad_s"])?.distance(vector(&b["spin_rad_s"])?),
                    ),
                ] {
                    put(
                        &mut series,
                        "all",
                        "bracket_refinement",
                        metric,
                        value,
                        index,
                    )
                }
            }
            if h.is_null() {
                inc(
                    &mut counts,
                    if predicted.is_null() {
                        "ordinary_uncertified"
                    } else {
                        "ordinary_certified"
                    },
                );
                continue;
            }
            if predicted.is_null()
                || uint(&h["time_ns"])? != 0
                || two["first_end_ns"] != 0
                || eight["first_end_ns"] != 0
                || sixtyfour["first_end_ns"] != 0
            {
                return Err(format!("false or delayed projection on {index}").into());
            }
            let n = vector(&r["segment"]["normal"])?;
            let j = vector(&h["j_ns"])?;
            let a = uint(&predicted["loaded_end"])? as usize;
            let residual = vector(&predicted["impulse_ns"])?.distance(j);
            let errors = [
                (
                    "point_m",
                    vector(&r["segment"]["points_m"][a])?.distance(vector(&h["point"])?),
                    1e-10,
                ),
                ("normal", n.distance(vector(&h["normal"])?), 1e-10),
                ("impulse_ns", residual, 1e-10),
                (
                    "velocity_before_m_s",
                    vector(&r["velocity_before"][a])?.distance(vector(&h["before"])?),
                    1e-10,
                ),
                (
                    "velocity_after_m_s",
                    vector(&predicted["velocity_after"][a])?.distance(vector(&h["after"])?),
                    1e-10,
                ),
                (
                    "other_after_m_s",
                    vector(&predicted["velocity_after"][1 - a])?
                        .distance(vector(&h["opposite_after"])?),
                    1e-10,
                ),
                (
                    "work_j",
                    (num(&predicted["work_loss_j"])? - num(&h["work_loss_j"])?).abs(),
                    1e-10,
                ),
            ];
            if h["sticking"] != predicted["sticking"]
                || num(&h["opening_bound_m_s"])? <= 0.
                || n.dot(vector(&h["opposite_after"])?) < num(&h["opening_bound_m_s"])? - 1e-12
            {
                return Err("invalid opposite endpoint opening".into());
            }
            for (metric, err, lim) in errors {
                if err > lim {
                    return Err(format!("registered {index} {metric} error {err}").into());
                }
                put(&mut series, "all", "registered", metric, err, index)
            }
            inc(&mut counts, "projected");
            for u in runs {
                if num(&u["pair_load_ns"])? + 1e-12 < n.dot(j) {
                    return Err("public interval lost its projected load".into());
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"rows":rows(series)}),
    )
}
pub(super) fn line_sticking(dir: &Path, other: &Path) -> Result<()> {
    let mut counts = BTreeMap::new();
    let mut series = Rows::new();
    let mut largest = None;
    for k in 0..1250 {
        if !dir.join("cases").join(format!("case-{k:04}.json")).exists() {
            continue;
        }
        let d = fixture(dir, k)?;
        let base = fixture(other, k)?;
        if d["format"] != "finite-line-sticking-floor-v1"
            || d["result"]["fixture"] != base["result"]["fixture"]
        {
            return Err("invalid line input".into());
        }
        inc(&mut counts, "fixtures");
        for t in array(&d["result"]["trials"])? {
            let key = 3 * k + uint(&t["stroke"])? as usize - 1;
            let r = &t["response"];
            let b = &matched(&base, t)?["response"];
            if r["geometry"] != b["geometry"] || r["arc_end"] != b["arc_end"] {
                return Err("different arriving line pose".into());
            }
            if r["segment"].is_null() {
                continue;
            }
            let s = &r["segment"];
            if s["points"] != b["segment"]["points_m"] || s["normal"] != b["segment"]["normal"] {
                return Err("different line geometry".into());
            }
            let mode = r["classification"]
                .as_str()
                .ok_or("missing pressure classification")?;
            inc(&mut counts, mode);
            let h = &r["attained"];
            if mode != "Attainable" {
                continue;
            }
            if !b["impact"].is_null() {
                return Err("feasible full-stick line with a certified open end".into());
            }
            let mass = 2.;
            let inertia = 0.05;
            let end = &r["arc_end"];
            let c = vector(&end["com_m"])?;
            let spin = vector(&end["spin_rad_s"])?;
            let v = vector(&end["velocity_m_s"])?;
            let post = vector(&h["v"])?;
            let w = vector(&h["w"])?;
            let j = vector(&h["j_ns"])?;
            let moment = vector(&h["torque_nms"])?;
            let p = vector(&s["points"][0])?;
            let q = vector(&s["points"][1])?;
            let n = vector(&s["normal"])?;
            let x = (q - p).normalize();
            let y = n.cross(x).normalize();
            let lo = num(&h["first_range_ns"][0])?;
            let hi = num(&h["first_range_ns"][1])?;
            let width = hi - lo;
            if width <= 0. {
                return Err("infeasible longitudinal pressure range".into());
            }
            if largest.is_none_or(|(a, _)| width > a) {
                largest = Some((width, key))
            }
            let lost = 0.5
                * (mass * (v.length_squared() - post.length_squared())
                    + inertia * (spin.length_squared() - w.length_squared()));
            if lost < 0.
                || j.distance(mass * (post - v)) > 1e-10
                || moment.distance(inertia * (w - spin)) > 1e-10
                || (lost - num(&h["loss_j"])?).abs() > 1e-10
                || [p, q]
                    .into_iter()
                    .any(|at| (post + w.cross(at - c)).length() > 1e-10)
            {
                return Err("line energy or momentum mismatch".into());
            }
            for along in [lo, 0.5 * (lo + hi), hi] {
                let a = along * x + num(&h["transverse_ns"][0])? * y + num(&h["normal_ns"][0])? * n;
                let b = (num(&h["longitudinal_ns"])? - along) * x
                    + num(&h["transverse_ns"][1])? * y
                    + num(&h["normal_ns"][1])? * n;
                if [a, b].iter().any(|j| {
                    j.dot(n) < 0. || j.reject_from_normalized(n).length() > 0.5 * j.dot(n) + 1e-10
                }) || (a + b).distance(j) > 1e-10
                    || ((p - c).cross(a) + (q - c).cross(b)).distance(moment) > 1e-10
                {
                    return Err("equivalent pressure moments disagree".into());
                }
            }
            put(
                &mut series,
                "all",
                "attainable",
                "longitudinal_range_width_ns",
                width,
                key,
            );
            let runs = array(&r["native"])?;
            if runs.len() != 6 {
                return Err("missing cold native runs".into());
            }
            for (i, (ns, relax)) in [(1, 1), (1, 8), (128, 1), (128, 8), (2048, 1), (2048, 8)]
                .into_iter()
                .enumerate()
            {
                let t = &runs[i];
                if t["duration_ns"] != ns
                    || t["relaxation"] != relax
                    || vector(&t["before_v"])?.distance(v) > 1e-11
                    || vector(&t["before_w"])?.distance(spin) > 1e-11
                {
                    return Err("mismatched native initial twist".into());
                }
                let native_v = vector(&t["after_v"])?;
                let native_w = vector(&t["after_w"])?;
                let native_j = vector(&t["j_ns"])?;
                let native_energy = 0.5
                    * (mass * (native_v.length_squared() - post.length_squared())
                        + inertia * (native_w.length_squared() - w.length_squared()));
                for (m, val) in [
                    ("impulse_discrepancy_ns", native_j.distance(j)),
                    ("remaining_kinetic_energy_vs_floor_j", native_energy),
                    ("spin_discrepancy_rad_s", native_w.distance(w)),
                    ("velocity_discrepancy_m_s", native_v.distance(post)),
                ] {
                    put(&mut series, "all", &format!("{ns}ns_r{relax}"), m, val, key)
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"widest_internal_split_ns_and_case":largest,"rows":rows(series)}),
    )
}
