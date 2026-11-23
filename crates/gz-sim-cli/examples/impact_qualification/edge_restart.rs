// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare two solver revisions against the same material integration and public clock.
use super::{transfer::vector, *};
use glam::DVec3;

fn center(row: &Value) -> Result<DVec3> {
    Ok(0.5 * (vector(&row["foot"]["position"])? + vector(&row["upper"]["position"])?))
}

// A rigid line can support several pressure distributions. Keep distances to
// each named continuation separate from errors against a unique reference.
fn pressure_comparisons(
    reference: &Value,
    step: usize,
    p: &Value,
    measured: DVec3,
) -> Result<Vec<Value>> {
    let Some(models) = reference["conditional_pressure_models"]["alternatives"].as_array() else {
        return Ok(vec![]);
    };
    let refs = array(&reference["references"])?;
    let stopped = &refs.last().ok_or("missing ambiguous line")?["fine"];
    let j_to_line = vector(&stopped["j"])? - vector(&refs[0]["fine"]["j"])?;
    let at = (step + 1) as f64 * 0.0005;
    let drive = vector(&reference["drive"])?;
    let mut rows = vec![];
    for model in models {
        let mut samples = Vec::new();
        for run in array(&model["continuation"])? {
            if let Some(f) = array(&run["frames"])?.iter().find(|f| {
                f["original_time_s"]
                    .as_f64()
                    .is_some_and(|t| (t - at).abs() < 1e-12)
            }) {
                samples.push((
                    vector(&f["com_m"])? + at * drive,
                    j_to_line + vector(&model["j_ns"])? + vector(&f["support_frame_j_ns"])?,
                ));
            }
        }
        if let [(coarse_pos, coarse_j), (fine_pos, fine_j)] = samples.as_slice() {
            rows.push(json!({"line_fractions":model["fractions"],"com_distance_m":center(p)?.distance(*fine_pos),
                "accumulated_impulse_distance_ns":measured.distance(*fine_j),"mesh_com_difference_m":coarse_pos.distance(*fine_pos),
                "mesh_impulse_difference_ns":coarse_j.distance(*fine_j)}));
        }
    }
    Ok(rows)
}

pub(super) fn compare(
    before: &Path,
    after: &Path,
    output: &Path,
    primary: Option<&Path>,
) -> Result<()> {
    let reference = read(&output.join("reference.json"))?;
    let case = uint(&reference["case"])?;
    let stroke = uint(&reference["stroke"])?;
    let dir = format!("case-{case:04}-stroke-{stroke}/traces");
    let frames = |base: &Path, lane| -> Result<Vec<Value>> {
        let d = read(&base.join(&dir).join(format!("lane-{lane}.json")))?;
        Ok(array(&d["frames"])?.to_vec())
    };
    let fine = frames(before, 4)?;
    if fine != frames(after, 4)? {
        return Err("continuous native control changed".into());
    }
    let matched = array(&reference["frames"])?;
    let mut results = BTreeMap::new();
    for (name, path) in [
        ("previous", frames(before, 3)?),
        ("revised", frames(after, 3)?),
        ("native8192", fine.clone()),
    ] {
        if path.len() != 27
            || fine.len() != path.len()
            || matched.len() > path.len()
            || matched.len() < 2
            || ["foot", "upper", "pad"]
                .iter()
                .any(|key| path[0][key] != matched[0][key])
        {
            return Err("incomplete or unmatched public trajectory".into());
        }
        let mut cumulative = DVec3::ZERO;
        let mut measured = DVec3::ZERO;
        let mut rows = vec![];
        for (i, p) in path.iter().enumerate().skip(1) {
            if p["tick"] != fine[i]["tick"]
                || p["time_us"] != fine[i]["time_us"]
                || p["drive"] != fine[i]["drive"]
            {
                return Err("different public clock or actuator command".into());
            }
            let mut row = json!({"elapsed_us":i*500,"com_distance_from_native8192_m":center(p)?.distance(center(&fine[i])?)});
            measured += vector(&p["j"])?;
            if let Some(m) = matched.get(i) {
                if p["tick"] != m["tick"]
                    || p["time_us"] != m["time_us"]
                    || reference["references"][i]["qualified"] != true
                {
                    return Err("unqualified trajectory reference".into());
                }
                cumulative += vector(&m["j"])?;
                row["com_error_m"] = json!(center(p)?.distance(center(m)?));
                row["step_impulse_error_ns"] = json!(vector(&p["j"])?.distance(vector(&m["j"])?));
                row["accumulated_impulse_error_ns"] = json!(measured.distance(cumulative));
                row["normal_impulse_error_ns"] =
                    json!((num(&p["normal_impulse_ns"])? - num(&m["normal_impulse_ns"])?).abs());
            } else {
                let comparisons = pressure_comparisons(&reference, i, p, measured)?;
                if !comparisons.is_empty() {
                    row["conditional_pressure_comparisons"] = json!(comparisons);
                }
            }
            rows.push(row);
        }
        results.insert(name, rows);
    }
    let receipt = read(&output.join("receipt.json"))?;
    let first = &frames(after, 3)?[1];
    if vector(&receipt["j_ns"])?.distance(vector(&first["j"])?) > 1e-11
        || receipt["native_calls"] != first["native_steps"]
    {
        return Err("authored scene and original resumed run disagree".into());
    }
    write(
        &output.join("trajectory.json"),
        &json!({"case":case,"stroke":stroke,"lanes":results}),
    )?;
    if let Some(primary) = primary {
        field(before, after, primary, output)?;
    }
    Ok(())
}

fn field(before: &Path, after: &Path, primary: &Path, output: &Path) -> Result<()> {
    let mut policies = BTreeMap::<&str, BTreeMap<&str, u32>>::new();
    let mut starts = 0;
    let mut regressions = vec![];
    let mut changes = vec![];
    for case in 0..1250 {
        let fname = format!("cases/case-{case:04}.json");
        let old = read(&before.join(&fname))?;
        let new = read(&after.join(&fname))?;
        let original = read(&primary.join(&fname))?;
        if old["case"] != case
            || new["case"] != case
            || old["fixture"] != new["fixture"]
            || array(&new["trials"])?.len() != 3
            || array(&old["trials"])?.len() != 3
        {
            return Err("incomplete restart census".into());
        }
        for i in 0..3 {
            let (a, b) = (&old["trials"][i], &new["trials"][i]);
            if a["stroke"] != i + 1
                || b["stroke"] != i + 1
                || a["status"] != b["status"]
                || a["boundary"] != b["boundary"]
                || a["controls"] != b["controls"]
                || a["paths"][4] != b["paths"][4]
            {
                return Err(format!(
                    "changed restart or controls at fixture {case}, stroke {}",
                    i + 1
                )
                .into());
            }
            starts += 1;
            if b["paths"].is_null() {
                continue;
            }
            for (lane, label) in [
                "fixed_history",
                "fixed_detect",
                "compact_history",
                "compact_detect",
            ]
            .into_iter()
            .enumerate()
            {
                let (x, y) = (&a["paths"][lane], &b["paths"][lane]);
                let counts = policies.entry(label).or_default();
                let (old_ns, new_ns) = (uint(&x["covered_ns"])?, uint(&y["covered_ns"])?);
                if x["tick"] != y["tick"]
                    || x["drive"] != y["drive"]
                    || x["time_us"] != y["time_us"]
                    || x["other_pair_normal_ns"] != 0.
                    || y["other_pair_normal_ns"] != 0.
                {
                    return Err("changed clock, drive or other pair reaction".into());
                }
                *counts.entry("executed").or_default() += 1;
                if old_ns == 500_000 {
                    *counts.entry("previous_complete").or_default() += 1;
                }
                if new_ns == 500_000 {
                    *counts.entry("revised_complete").or_default() += 1;
                }
                if old_ns == 500_000 && new_ns < old_ns {
                    regressions.push(json!({"case":case,"stroke":i+1,"policy":label,"covered_ns":[old_ns,new_ns]}));
                }
                if b["status"] != "qualified" {
                    continue;
                }
                let refn = &b["controls"][1];
                let base = &original["result"]["trials"][i];
                let truth = vector(&refn["com"])? + 0.001 * vector(&base["deck_v"])?;
                let j = vector(&refn["j"])? - vector(&base["mechanics"][1]["j"])?;
                let dc = center(y)?.distance(truth) - center(x)?.distance(truth);
                let dj = vector(&y["j"])?.distance(j) - vector(&x["j"])?.distance(j);
                for key in [
                    if dc < -1e-12 {
                        "com_improved"
                    } else if dc > 1e-12 {
                        "com_worsened"
                    } else {
                        "com_tied"
                    },
                    if dj < -1e-10 {
                        "impulse_improved"
                    } else if dj > 1e-10 {
                        "impulse_worsened"
                    } else {
                        "impulse_tied"
                    },
                ] {
                    *counts.entry(key).or_default() += 1;
                }
                if old_ns != new_ns || dc.abs() > 1e-12 || dj.abs() > 1e-10 {
                    changes.push(json!({"case":case,"stroke":i+1,"policy":label,"covered_ns":[old_ns,new_ns],"com_error_change_m":dc,"impulse_error_change_ns":dj}));
                }
            }
        }
    }
    write(
        &output.join("field.json"),
        &json!({"starts":starts,"policies":policies,"lost_complete":regressions,"changed":changes}),
    )?;
    if !regressions.is_empty() {
        return Err("previously complete material interval lost".into());
    }
    Ok(())
}
