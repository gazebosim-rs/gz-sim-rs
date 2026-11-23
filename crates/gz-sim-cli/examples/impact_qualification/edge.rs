// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check common physical targets, native work budgets and signed receipt errors.
use super::*;
fn vector(v: &Value) -> Result<[f64; 3]> {
    let [x, y, z] = array(v)? else {
        return Err("expected three components".into());
    };
    Ok([num(x)?, num(y)?, num(z)?])
}
pub(super) fn summarize(dir: &Path, baseline: Option<&Path>) -> Result<()> {
    let doc = read(&dir.join("study.json"))?;
    let rows = array(&doc["rows"])?;
    let archive = baseline.map(|p| read(&p.join("study.json"))).transpose()?;
    let old = archive.as_ref().map(|d| array(&d["rows"])).transpose()?;
    if old.is_some_and(|o| o.len() != rows.len()) || uint(&doc["certified"])? != rows.len() as u64 {
        return Err("different reference fixture count".into());
    }
    let mut seen = BTreeSet::new();
    let mut groups: BTreeMap<(&str, u64, &str, &str), Series> = BTreeMap::new();
    for (k, row) in rows.iter().enumerate() {
        let id = uint(&row["case"])? as usize;
        if !seen.insert(id) {
            return Err("duplicate case".into());
        }
        let [two, sixty_four, arc] = array(&row["profiles"])? else {
            return Err("missing profile".into());
        };
        if let Some(old) = old {
            let a = &old[k];
            if row["fixture"] != a["fixture"]
                || row["case"] != a["case"]
                || row["active_ticks"] != a["active_ticks"]
            {
                return Err(format!("independent native control changed: {id}").into());
            }
            for (current, archive) in [(two, &a["profiles"][0]), (sixty_four, &a["profiles"][2])] {
                let mut projected = current.clone();
                let obj = projected.as_object_mut().ok_or("invalid profile")?;
                obj.remove("profile");
                for sample in obj
                    .get_mut("samples")
                    .and_then(Value::as_array_mut)
                    .ok_or("invalid samples")?
                {
                    let fields = sample.as_object_mut().ok_or("invalid sample")?;
                    fields.remove("cumulative_native_calls");
                    fields.remove("followed_ticks");
                }
                if &projected != archive {
                    return Err(format!("independent sample changed: {id}").into());
                }
            }
        }
        let samples = array(&arc["samples"])?;
        let clocks: Vec<_> = [1, 2, 10, 40]
            .into_iter()
            .filter(|t| *t <= row["active_ticks"].as_u64().unwrap_or(0))
            .collect();
        if samples.len() != clocks.len() || clocks.is_empty() {
            return Err("missing physical checkpoints".into());
        }
        for (i, &tick) in clocks.iter().enumerate() {
            let followed = uint(&samples[i]["followed_ticks"])?;
            if followed > tick {
                return Err("impossible followed prefix".into());
            }
            let group = if followed == tick {
                "admitted_prefix"
            } else {
                "native_fallback"
            };
            let reference = vector(&samples[i]["reference_post_impulse_ns"])?;
            for (name, n, p) in [
                ("two", 2, two),
                ("sixty_four", 64, sixty_four),
                ("arc", 1, arc),
            ] {
                let r = &p["samples"][i];
                if p["profile"] != name
                    || uint(&r["tick"])? != tick
                    || uint(&r["cumulative_native_calls"])? != n * tick + u64::from(n == 1)
                    || vector(&r["reference_post_impulse_ns"])? != reference
                    || r["instantaneous_ns"] != samples[i]["instantaneous_ns"]
                {
                    return Err(
                        format!("unmatched work or physical target: {id}/{name}/{tick}").into(),
                    );
                }
                for (field, vector_field) in [
                    ("receipt_error_ns", "receipt_post_impulse_ns"),
                    ("momentum_error_ns", "momentum_post_impulse_ns"),
                ] {
                    let v = vector(&r[vector_field])?;
                    let dist = (v[0] - reference[0])
                        .hypot(v[1] - reference[1])
                        .hypot(v[2] - reference[2]);
                    if (dist - num(&r[field])?).abs() > 1e-13 * (1. + dist) {
                        return Err("inconsistent signed reaction".into());
                    }
                }
                for field in [
                    "com_error_m",
                    "spin_error_rad_s",
                    "receipt_error_ns",
                    "momentum_error_ns",
                ] {
                    let v = num(&r[field])?;
                    for cohort in ["all", group] {
                        groups
                            .entry((cohort, tick, name, field))
                            .or_default()
                            .push(v, id);
                    }
                }
            }
        }
    }
    let stats=groups.into_iter().map(|((cohort,tick,profile,field),x)|json!({ "cohort":cohort,"tick":tick,"profile":profile,"metric":field,"statistics":x.stats()})).collect::<Vec<_>>();
    write(
        &dir.join("summary.json"),
        &json!({"cases":seen.len(),"independent_native_control_archive":baseline,"rows":stats}),
    )
}
