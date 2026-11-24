// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cross-check matched finite primary donors and multiple independent numeric references.
use super::*;
use glam::DVec3;
fn v(x: &Value) -> Result<DVec3> {
    let [a, b, c] = array(x)? else {
        return Err("missing xyz".into());
    };
    Ok(DVec3::new(num(a)?, num(b)?, num(c)?))
}
fn optional(dir: &Path, k: usize) -> Result<Option<Value>> {
    let p = dir.join("cases").join(format!("case-{k:04}.json"));
    if p.exists() {
        Ok(Some(read(&p)?))
    } else {
        Ok(None)
    }
}
fn trial(d: &Value, stroke: u64) -> Result<&Value> {
    array(&d["result"]["trials"])?
        .iter()
        .find(|r| r["stroke"].as_u64() == Some(stroke))
        .ok_or_else(|| "missing historical stroke".into())
}
fn count(m: &mut BTreeMap<String, u64>, s: impl Into<String>) {
    *m.entry(s.into()).or_default() += 1;
}
pub(super) fn summarize(dir: &Path, policy: &Path, grid: &Path, relaxed: &Path) -> Result<()> {
    let mut stats: BTreeMap<(String, String, String), Series> = BTreeMap::new();
    let mut counts = BTreeMap::new();
    let mut wins = BTreeMap::new();
    for k in 0..1250 {
        let Some(d) = optional(dir, k)? else {
            continue;
        };
        if d["format"] != "finite-deck-matched-primary-v1" || d["index"] != k {
            return Err("incorrect finite geometry archive".into());
        }
        let (candidate, one, eight) = (
            optional(policy, k)?,
            optional(grid, k)?,
            optional(relaxed, k)?,
        );
        for cmp in [candidate.as_ref(), one.as_ref(), eight.as_ref()]
            .into_iter()
            .flatten()
        {
            if cmp["result"]["fixture"] != d["result"]["fixture"] || cmp["index"] != k {
                return Err("fixture mismatch".into());
            }
        }
        count(&mut counts, "fixtures");
        for t in array(&d["result"]["trials"])? {
            let stroke = uint(&t["stroke"])?;
            let g = &t["geometry"];
            let feature = g["feature"]
                .as_str()
                .unwrap_or("unavailable")
                .split('(')
                .next()
                .unwrap();
            let key = 3 * k + stroke as usize - 1;
            let mut add = |name: &str, field: &str, val: f64| {
                for group in [
                    "all",
                    feature,
                    if k % 5 == 4 { "holdout" } else { "calibration" },
                ] {
                    stats
                        .entry((group.into(), name.into(), field.into()))
                        .or_default()
                        .push(val, key);
                }
            };
            count(
                &mut counts,
                format!(
                    "geometry_{}",
                    g["status"].as_str().ok_or("invalid finite status")?
                ),
            );
            count(&mut counts, format!("feature_{feature}"));
            let cand = candidate.as_ref().map(|d| trial(d, stroke)).transpose()?;
            let r1 = one.as_ref().map(|d| trial(d, stroke)).transpose()?;
            let r8 = eight.as_ref().map(|d| trial(d, stroke)).transpose()?;
            for t2 in [cand, r1, r8].into_iter().flatten() {
                if t2["cut"] != t["cut"] {
                    return Err(format!("unmatched cut {k}/{stroke}").into());
                }
            }
            if let Some(t2) = cand
                && t2["geometry"] != *g
            {
                return Err("independent finite geometry changed".into());
            }
            let target = if !g["instantaneous_impulse_ns"].is_null() {
                let lo = uint(&g["lo_ns"])?;
                let hi = uint(&g["hi_ns"])?;
                if hi < lo || hi - lo > 2 || hi > 500_000 {
                    return Err("invalid physical crossing".into());
                }
                Some((v(&g["instantaneous_impulse_ns"])?, v(&g["normal"])?, lo, hi))
            } else {
                None
            };
            let paths = array(&t["paths"])?;
            for (o, n) in paths.iter().zip([8, 64, 512, 4096, 16384]) {
                if o["calls"] != n {
                    return Err("native budget mismatch".into());
                }
                let f = &o["first"];
                let name = format!("native{n}");
                if let Some((j, normal, lo, hi)) = target
                    && !f.is_null()
                {
                    let end = uint(&f["end_ns"])?;
                    let mut sum = DVec3::ZERO;
                    for site in array(&f["sites"])? {
                        sum += v(&site["impulse_ns"])?;
                    }
                    let got = v(&f["impulse_ns"])?;
                    if sum.distance(got) > 1e-12 {
                        return Err("native point sum mismatch".into());
                    }
                    add(&name, "first_j_error_ns", got.distance(j));
                    add(
                        &name,
                        "first_jn_error_ns",
                        (num(&f["normal_ns"])? - normal.dot(j)).abs(),
                    );
                    add(
                        &name,
                        "time_outside_bracket_ns",
                        end.saturating_sub(hi).max(lo.saturating_sub(end)) as f64,
                    );
                    add(&name, "signed_end_from_upper_ns", end as f64 - hi as f64);
                    if end < lo {
                        count(&mut counts, format!("{name}_early"));
                    }
                }
            }
            if let Some(t2) = r8
                && let Some((j, normal, lo, hi)) = target
            {
                for o in array(&t2["paths"])? {
                    let p = &o["profile"];
                    if p["onset_only"] != false {
                        continue;
                    }
                    let f = &o["first"];
                    let Some(end) = f["end_ns"].as_u64() else {
                        continue;
                    };
                    let name = format!("native{}r{}", uint(&p["calls"])?, uint(&p["relaxation"])?);
                    add(
                        &name,
                        "first_jn_error_ns",
                        (num(&f["normal_ns"])? - normal.dot(j)).abs(),
                    );
                    add(
                        &name,
                        "time_outside_bracket_ns",
                        end.saturating_sub(hi).max(lo.saturating_sub(end)) as f64,
                    );
                }
            }
            let Some(cand) = cand else {
                continue;
            };
            let cp = array(&cand["paths"])?;
            if cp.len() != 6 || cp[0]["profile"] != "native8" {
                return Err("missing intervention".into());
            }
            for (path, budget) in cp.iter().zip([8, 2, 8, 64, 64, 8]) {
                let name = path["profile"].as_str().ok_or("missing name")?;
                if uint(&path["native_calls"])? != budget + 25 * 8 {
                    return Err("changed public budget".into());
                }
                if let Some(hit) = array(&path["impacts"])?.first().filter(|h| h["i"] == 1) {
                    count(&mut counts, format!("{name}_primary"));
                    if let Some((j, n, lo, hi)) = target {
                        let when = uint(&hit["local_ns"])?;
                        if (when < lo || when > hi) || hit["feature"] != g["feature"] {
                            return Err(
                                format!("incorrect finite feature or time {k}, {stroke}").into()
                            );
                        }
                        let jhat = v(&hit["impulse_ns"])?;
                        let vout = v(&hit["after"])?;
                        add(name, "first_j_error_ns", jhat.distance(j));
                        add(name, "normal_residual_m_s", vout.dot(n).abs());
                        if jhat.dot(n) <= 0. || num(&hit["dissipated_energy_j"])? < -1e-12 {
                            return Err("unphysical finite receipt".into());
                        }
                    }
                }
                for (kind, orig) in [("r1", r1), ("r8", r8)] {
                    let Some(o) = orig else {
                        continue;
                    };
                    let fine = array(&o["paths"])?
                        .iter()
                        .find(|p| {
                            if kind == "r1" {
                                p["method"] == "uniform" && p["calls"] == 4096
                            } else {
                                p["profile"]["calls"] == 4096 && p["profile"]["relaxation"] == 8
                            }
                        })
                        .ok_or("missing fine reference")?;
                    let record = if kind == "r1" { &fine["record"] } else { fine };
                    for (idx, (s, ref_s)) in array(&path["samples"])?
                        .iter()
                        .zip(array(&record["samples"])?)
                        .enumerate()
                    {
                        let tick = uint(&s["i"])?;
                        if ref_s["i"] != s["i"] {
                            return Err("mismatched public clock".into());
                        }
                        let d = v(&s["com_m"])?.distance(v(&ref_s["com_m"])?);
                        add(name, &format!("com_m_{kind}_tick{tick}"), d);
                        if tick == 26 {
                            add(
                                name,
                                &format!("normal_error_ns_{kind}"),
                                (num(&s["cumulative_normal_ns"])?
                                    - num(&ref_s["cumulative_normal_ns"])?)
                                .abs(),
                            );
                            add(
                                name,
                                &format!("tangent_error_ns_{kind}"),
                                v(&s["cumulative_tangent_ns"])?
                                    .distance(v(&ref_s["cumulative_tangent_ns"])?),
                            );
                            let other =
                                v(&cp[0]["samples"][idx]["com_m"])?.distance(v(&ref_s["com_m"])?);
                            add(name, &format!("change_vs_eight_m_{kind}"), d - other);
                            if d < other {
                                count(&mut wins, format!("{name}_{kind}"));
                            }
                        }
                    }
                }
            }
            // Same-driver ordinary eight and the independently observed eight
            // must agree in their public load and trajectory, allowing
            // representation-level roundoff in the alternate readback path.
            if let Some(o) = r8 {
                let entry = &array(&o["paths"])?[0];
                for (a, b) in array(&cp[0]["samples"])?
                    .iter()
                    .zip(array(&entry["samples"])?)
                {
                    for f in [
                        "com_m",
                        "spin_rad_s",
                        "velocity_m_s",
                        "cumulative_tangent_ns",
                    ] {
                        let d = v(&a[f])?.distance(v(&b[f])?);
                        if d > 1e-11 {
                            return Err(format!(
                                "ordinary independent readback {k}/{stroke}/{f}: {d}"
                            )
                            .into());
                        }
                    }
                    if (num(&a["cumulative_normal_ns"])? - num(&b["cumulative_normal_ns"])?).abs()
                        > 1e-11
                    {
                        return Err("ordinary reaction mismatch".into());
                    }
                }
            }
        }
    }
    let rows=stats.into_iter().map(|((cohort,profile,metric),x)|json!({"cohort":cohort,"profile":profile,"metric":metric,"statistics":x.stats()})).collect::<Vec<_>>();
    write(
        &policy.join("summary.json"),
        &json!({"counts":counts,"improved_over_native_eight_at_13ms":wins,"max_case_encoding":"3*fixture + stroke - 1","rows":rows}),
    )
}

pub(super) fn post_load(dir: &Path, policy: &Path) -> Result<()> {
    let mut rows: BTreeMap<(String, String), Series> = BTreeMap::new();
    let mut counts: BTreeMap<String, u64> = BTreeMap::new();
    let mut omitted = vec![];
    for k in 0..1250 {
        let Some(d) = optional(dir, k)? else { continue };
        let other = optional(policy, k)?.ok_or("missing independent primary solve")?;
        if d["format"] != "finite-deck-postimpact-v1"
            || d["index"] != k
            || d["result"]["fixture"] != other["result"]["fixture"]
        {
            return Err("incorrect postimpact donor".into());
        }
        count(&mut counts, "fixtures");
        for t in array(&d["result"]["trials"])? {
            let stroke = uint(&t["stroke"])?;
            let key = 3 * k + stroke as usize - 1;
            let p = trial(&other, stroke)?;
            let result = &t["response"];
            let geo = &result["geometry"];
            if t["cut"] != p["cut"] || geo != &p["geometry"] {
                return Err("physical starting point mismatch".into());
            }
            let l = &result["load"];
            if geo["instantaneous_impulse_ns"].is_null() {
                count(&mut counts, "no_single_impact");
                continue;
            }
            let feature = geo["feature"]
                .as_str()
                .ok_or("missing contact feature")?
                .split('(')
                .next()
                .unwrap();
            let admitted = result["transaction_geometry_admitted"]
                .as_bool()
                .ok_or("missing geometry admission")?;
            let projected = array(&p["paths"][2]["impacts"])?
                .first()
                .is_some_and(|e| e["i"] == 1);
            if !projected {
                omitted.push(json!({"fixture":k,"stroke":stroke,"feature":feature,"transaction_geometry_admitted":admitted}));
            }
            if l.is_null() {
                count(&mut counts, "unresolved_load");
                continue;
            }
            let mode = l["mode"].as_str().ok_or("missing load mode")?;
            count(&mut counts, mode);
            count(&mut counts, format!("{feature}_{mode}"));
            let n = v(&geo["normal"])?;
            let f = v(&l["force_n"])?;
            let vel = v(&l["material_velocity_m_s"])?;
            let force = num(&l["normal_n"])?;
            let free = num(&l["free_gap_acceleration_m_s2"])?;
            let constrained = num(&l["loaded_gap_acceleration_m_s2"])?;
            let dv = v(&l["normal_rate_s"])?;
            let a = v(&l["material_acceleration_m_s2"])?;
            if n.dot(vel).abs() > 1e-11
                || (n.dot(f) - force).abs() > 1e-11
                || (n.dot(a) + 2. * dv.dot(vel) - constrained).abs() > 2e-9
                || force < 0.
                || num(&l["friction_margin_n"])? < -1e-10
                || f.dot(vel) > 1e-10
                || (mode == "Opening" && (free <= 0. || f != DVec3::ZERO))
                || (mode != "Opening" && (free >= 0. || constrained.abs() > 1e-9))
            {
                return Err(format!("nonphysical loaded point {k}/{stroke}").into());
            }
            let mut add = |name: &str, value: f64| {
                for group in ["all", feature, mode] {
                    rows.entry((group.into(), name.into()))
                        .or_default()
                        .push(value, key)
                }
            };
            for name in [
                "normal_n",
                "geometric_acceleration_residual_m_s2",
                "free_gap_acceleration_m_s2",
                "friction_margin_n",
                "slip_power_w",
            ] {
                add(name, num(&l[name])?)
            }
            let ports = array(&l["vertex_motion"])?;
            if ports.len() != 8 {
                return Err("incomplete sole".into());
            }
            let mut next = f64::INFINITY;
            for corner in ports {
                let gap = num(&corner["face_gap_m"])?;
                let speed = num(&corner["normal_velocity_m_s"])?;
                if corner["inside_face_xy"] == true && gap > 1e-9 && speed < 0. {
                    next = next.min(-gap / speed)
                }
            }
            if next.is_finite() {
                add("linearized_competing_face_vertex_us", next * 1e6);
                if next < (500000 - uint(&geo["hi_ns"])?) as f64 * 1e-9 {
                    count(&mut counts, "competitor_predicted_inside_public_tick")
                }
            }
        }
    }
    let rows = rows
        .into_iter()
        .map(|((group, metric), v)| json!({"group":group,"metric":metric,"statistics":v.stats()}))
        .collect::<Vec<_>>();
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"declined_primary":omitted,"rows":rows}),
    )
}

pub(super) fn compare_revisions(before: &Path, after: &Path) -> Result<()> {
    let mut counts = BTreeMap::new();
    let mut differing = vec![];
    for k in 0..1250 {
        let a = optional(before, k)?.ok_or("missing original archive")?;
        let b = optional(after, k)?.ok_or("missing updated archive")?;
        if a["format"] != b["format"]
            || a["result"]["fixture"] != b["result"]["fixture"]
            || a["result"]["disjoint"] != b["result"]["disjoint"]
        {
            return Err("revision donor mismatch".into());
        }
        let (aa, bb) = (
            array(&a["result"]["trials"])?,
            array(&b["result"]["trials"])?,
        );
        if aa.len() != bb.len() {
            return Err("revision trial count mismatch".into());
        }
        for (x, y) in aa.iter().zip(bb) {
            if x["cut"] != y["cut"] || x["stroke"] != y["stroke"] {
                return Err("mismatched released cut".into());
            }
            let stroke = uint(&x["stroke"])?;
            if x["geometry"] != y["geometry"] {
                count(&mut counts, "geometry_changed")
            }
            for (u, v) in array(&x["paths"])?.iter().zip(array(&y["paths"])?) {
                let id = u["profile"].as_str().ok_or("missing profile")?;
                if u["profile"] != v["profile"] || u["native_calls"] != v["native_calls"] {
                    return Err("unexpected profile or budget change".into());
                }
                if u != v {
                    if id.starts_with("native") {
                        return Err("native control changed".into());
                    }
                    count(&mut counts, format!("{id}_changed"));
                    differing.push(json!({"fixture":k,"stroke":stroke,"profile":id,"before_hits":array(&u["impacts"])?.len(),"after_hits":array(&v["impacts"])?.len(),"first_tick_pose_changed":u["samples"][0]!=v["samples"][0],"endpoint_com_change_m":self::v(&u["samples"][3]["com_m"])?.distance(self::v(&v["samples"][3]["com_m"])?)}));
                } else {
                    count(&mut counts, format!("{id}_unchanged"))
                }
            }
        }
    }
    write(
        &after.join("revision-comparison.json"),
        &json!({"counts":counts,"changed":differing}),
    )
}
