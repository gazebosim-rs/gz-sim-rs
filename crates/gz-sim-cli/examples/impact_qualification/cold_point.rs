// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cross cold initial-point detection with optional native-call compaction.
use super::*;
fn center(r: &Value) -> Result<DVec3> {
    Ok(0.5 * (vec3(&r["foot"]["position"])? + vec3(&r["upper"]["position"])?))
}
pub(crate) fn summarize(dir: &Path, episodes: &Path, observed: &Path) -> Result<()> {
    let mut rows = Fields::new();
    let mut counts = BTreeMap::new();
    let mut largest = None;
    for k in 0..50 {
        let file = format!("case-{k:03}.json");
        let d = read(&dir.join("cases").join(&file))?;
        let e = read(&episodes.join("cases").join(&file))?;
        let o = read(&observed.join("cases").join(&file))?;
        if d["case"] != k
            || d["format"] != "cold-material-bearing-v1"
            || d["cut"] != e["cut"]
            || d["cut"] != o["cut"]
            || d["source"] != e["source"]
        {
            return Err("foreign restart donor".into());
        }
        let cut = uint(&e["cut"]["tick"])?;
        let refs = array(&e["references"])?;
        for (trial, n) in array(&d["trials"])?.iter().zip([1usize, 2, 8]) {
            if trial["prefix"] != n {
                return Err("wrong physical boundary".into());
            }
            let b = &trial["boundary"];
            let prefix = &refs[n - 1]["fine"];
            let drive = vec3(&e["drive"])?;
            if b["tick"] != cut + n as u64
                || b["native_steps"] != 0
                || center(b)?.distance(vec3(&prefix["com"])? + n as f64 * 0.0005 * drive) > 1e-12
            {
                return Err("mismatched physical cold boundary".into());
            }
            let paths = array(&trial["paths"])?;
            if paths.len() != 5
                || paths
                    .iter()
                    .any(|p| p.as_array().is_none_or(|a| a.len() != 26))
            {
                return Err("incomplete cold episode".into());
            }
            if n == 1 {
                let first = &paths[4][0];
                let r = &o["lanes"][0]["total"];
                if first["foot"] != r["public_foot"]
                    || first["upper"] != r["public_upper"]
                    || first["pad"] != r["public_pad"]
                    || vec3(&first["j"])?.distance(vec3(&r["j_ns"])?) > 2e-10
                {
                    return Err("8192 cold control differs".into());
                }
            }
            count(&mut counts, "restarts");
            let mut total = [DVec3::ZERO; 5];
            let id = 3 * k
                + match n {
                    1 => 0,
                    2 => 1,
                    _ => 2,
                };
            for h in 0..26 {
                let at = n + h + 1;
                for (i, p) in paths.iter().enumerate() {
                    let r = &p[h];
                    if r["tick"] != cut + at as u64
                        || r["time_us"] != (cut + at as u64) * 500
                        || r["drive"] != paths[0][h]["drive"]
                        || num(&r["other_pair_normal_ns"])? != 0.
                        || (i == 4 && r["native_steps"] != 8192)
                        || uint(&r["covered_ns"])? > 500000
                        || uint(&r["omitted"])? + uint(&r["native_steps"])?
                            > if i == 4 { 8192 } else { 8 }
                    {
                        return Err("different actuator or solver budget".into());
                    }
                    total[i] += vec3(&r["j"])?;
                }
                if h == 0 {
                    for (i, j, label) in [(0, 1, "fixed"), (2, 3, "compact")] {
                        let (a, b) = (&paths[i][0], &paths[j][0]);
                        let old = uint(&a["covered_ns"])?;
                        let new = uint(&b["covered_ns"])?;
                        if old == 500000 && new != 500000 {
                            return Err("lost established material path".into());
                        }
                        if old < 500000 && new == 500000 {
                            count(&mut counts, &format!("{label}_new_complete"));
                        }
                        if old == 500000 {
                            count(&mut counts, &format!("{label}_previous_complete"));
                        }
                        if new == 500000 {
                            count(&mut counts, &format!("{label}_complete"));
                        }
                        if a["native_steps"] != b["native_steps"] {
                            count(&mut counts, &format!("{label}_changed_calls"));
                        }
                        if a["foot"] == b["foot"]
                            && a["upper"] == b["upper"]
                            && a["pad"] == b["pad"]
                            && a["j"] == b["j"]
                        {
                            count(&mut counts, &format!("{label}_same_state_and_impulse"));
                        }
                    }
                }
                if ![0, 7, 24].contains(&h) {
                    continue;
                }
                let Some(r) = refs.get(at - 1) else {
                    continue;
                };
                let (a, b) = (&r["coarse"], &r["fine"]);
                if b["end"] != "original_clock"
                    || a["end"] != "original_clock"
                    || vec3(&a["j"])?.distance(vec3(&b["j"])?) > 1e-5
                    || vec3(&a["com"])?.distance(vec3(&b["com"])?) > 1e-7
                    || array(&a["events"])?.len() != array(&b["events"])?.len()
                {
                    continue;
                }
                let truth = vec3(&b["com"])? + at as f64 * 0.0005 * drive;
                let impulse = vec3(&b["j"])? - vec3(&prefix["j"])?;
                for (i, p) in paths.iter().enumerate() {
                    let label = format!(
                        "{}_after_{}_steps",
                        d["policies"][i].as_str().unwrap(),
                        h + 1
                    );
                    add(
                        &mut rows,
                        &label,
                        "com_error_m",
                        center(&p[h])?.distance(truth),
                        id,
                    );
                    add(
                        &mut rows,
                        &label,
                        "impulse_error_ns",
                        total[i].distance(impulse),
                        id,
                    );
                }
                if h == 0 {
                    let delta = center(&paths[2][h])?.distance(truth)
                        - center(&paths[3][h])?.distance(truth);
                    if largest.is_none_or(|(v, _, _)| delta > v) {
                        largest = Some((delta, k, n));
                    }
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("restarts")==Some(&150),"counts":counts,"largest_recovered_com_m_case_prefix":largest,"rows":values(rows)}),
    )
}

/// Read every first resumed actuator tick in the full three-stroke field.
pub(crate) fn field(dir: &Path, primary: &Path, episodes: &Path) -> Result<()> {
    let mut counts = BTreeMap::new();
    let mut fields = Fields::new();
    let mut largest = None;
    let mut worst = None;
    let mut nominal = BTreeMap::new();
    for k in 0..50 {
        let d = read(&episodes.join("cases").join(format!("case-{k:03}.json")))?;
        if nominal
            .insert(serde_json::to_string(&d["cut"])?, d)
            .is_some()
        {
            return Err("duplicate nominal physical cut".into());
        }
    }
    for k in 0..1250 {
        let file = format!("case-{k:04}.json");
        let d = read(&dir.join("cases").join(&file))?;
        let refn = read(&primary.join("cases").join(&file))?;
        if d["case"] != k
            || d["format"] != "cold-material-successor-field-v1"
            || d["fixture"] != refn["result"]["fixture"]
            || array(&d["trials"])?.len() != 3
        {
            return Err("cold field identity mismatch".into());
        }
        for (r, s) in array(&d["trials"])?
            .iter()
            .zip(array(&refn["result"]["trials"])?)
        {
            count(&mut counts, "starts");
            let status = r["status"].as_str().ok_or("missing outcome")?;
            count(&mut counts, status);
            let stroke = uint(&r["stroke"])? as usize;
            if r["stroke"] != s["stroke"] {
                return Err("changed stroke".into());
            }
            if r["paths"].is_null() {
                if !matches!(status, "unqualified primary" | "changed drive") {
                    return Err("missing qualified successor".into());
                }
                continue;
            }
            let paths = array(&r["paths"])?;
            let initial = &r["boundary"];
            let cut = uint(&s["cut"]["tick"])?;
            let base = &s["mechanics"][1];
            let drive = vec3(&s["deck_v"])?;
            if paths.len() != 5
                || initial["tick"] != cut + 1
                || center(initial)?.distance(vec3(&base["com"])? + 0.0005 * drive) > 1e-12
                || paths.iter().any(|p| {
                    p["tick"] != cut + 2
                        || p["time_us"] != (cut + 2) * 500
                        || p["drive"] != initial["pad"]["velocity"][2]
                        || p["other_pair_normal_ns"] != 0.
                })
            {
                return Err("different cold states/actuator clocks".into());
            }
            if let Some(o) = nominal
                .get(&serde_json::to_string(&s["cut"])?)
                .filter(|_| stroke == 1)
            {
                if o["cut"] != s["cut"] || o["trials"][0]["boundary"] != *initial {
                    return Err("nominal cold boundary differs".into());
                }
                for (i, p) in paths.iter().enumerate() {
                    if o["trials"][0]["paths"][i][0] != *p {
                        return Err("independent nominal solve differs".into());
                    }
                }
                count(&mut counts, "nominal_full_repeats");
            }
            let id = 3 * k + stroke - 1;
            let (a, b) = (&r["controls"][0], &r["controls"][1]);
            let verified = a["end"] == "original_clock"
                && b["end"] == "original_clock"
                && vec3(&a["com"])?.distance(vec3(&b["com"])?) <= 1e-7
                && vec3(&a["j"])?.distance(vec3(&b["j"])?) <= 1e-5
                && num(&b["arc_energy_defect"])?.abs() <= 1e-7
                && array(&a["events"])?.len() == array(&b["events"])?.len()
                && array(&a["events"])?
                    .iter()
                    .zip(array(&b["events"])?)
                    .all(|(x, y)| ["feature", "mode", "stuck"].iter().all(|k| x[k] == y[k]));
            if verified != (status == "qualified") {
                return Err("independent successor qualification differs".into());
            }
            for (i, p) in paths.iter().enumerate() {
                if uint(&p["covered_ns"])? > 500000
                    || (i == 4 && p["native_steps"] != 8192)
                    || (i < 4 && ![1, 8].contains(&uint(&p["native_steps"])?))
                {
                    return Err("invalid native accounting".into());
                }
                let label = [
                    "fixed_history",
                    "fixed_detect",
                    "compact_history",
                    "compact_detect",
                    "native8192",
                ][i];
                if p["covered_ns"] == 500000 {
                    count(&mut counts, &format!("{label}_whole"));
                }
                if verified {
                    let truth = vec3(&b["com"])? + 0.001 * drive;
                    add(
                        &mut fields,
                        label,
                        "com_error_m",
                        center(p)?.distance(truth),
                        id,
                    );
                    add(
                        &mut fields,
                        label,
                        "signed_j_error_ns",
                        vec3(&p["j"])?.distance(vec3(&b["j"])? - vec3(&base["j"])?),
                        id,
                    );
                }
            }
            for (label, i) in [("fixed", 0), ("compact", 2)] {
                let (old, new) = (&paths[i], &paths[i + 1]);
                let old_span = uint(&old["covered_ns"])?;
                let new_span = uint(&new["covered_ns"])?;
                if old_span == 500000 && new_span < 500000 {
                    return Err("lost complete contact".into());
                }
                if new_span > old_span {
                    count(&mut counts, &format!("{label}_increased_coverage"));
                }
                if old_span < 500000 && new_span == 500000 {
                    count(&mut counts, &format!("{label}_new_whole"));
                }
                if !verified {
                    continue;
                }
                let truth = vec3(&b["com"])? + 0.001 * drive;
                let delta = center(old)?.distance(truth) - center(new)?.distance(truth);
                count(
                    &mut counts,
                    &format!(
                        "{label}_{}",
                        if delta > 1e-12 {
                            "closer"
                        } else if delta < -1e-12 {
                            "farther"
                        } else {
                            "equal_to_1pm"
                        }
                    ),
                );
                add(&mut fields, label, "com_improvement_m", delta, id);
                if i == 2 {
                    if largest.is_none_or(|(v, _)| delta > v) {
                        largest = Some((delta, id));
                    }
                    if worst.is_none_or(|(v, _)| delta < v) {
                        worst = Some((delta, id));
                    }
                }
            }
        }
    }
    if counts.get("nominal_full_repeats") != Some(&50) {
        return Err("missing nominal physical repeats".into());
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("starts")==Some(&3750),"counts":counts,"largest_compact_improvement_m_case":largest,"least_compact_improvement_m_case":worst,"rows":values(fields)}),
    )
}
