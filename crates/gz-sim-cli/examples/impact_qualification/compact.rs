// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! An admitted tail must reproduce the separately run full-span two-call world.
//! Very short tails can require extra integer-nanosecond preflight partitions.
//! A rejected attempt must reproduce the original budget, including every receipt.
use super::*;
fn com(r: &Value) -> Result<DVec3> {
    Ok(0.5 * (vec3(&r["first"]["foot"]["position"])? + vec3(&r["first"]["upper"]["position"])?))
}
fn strip(v: &Value) -> Result<Value> {
    let mut r = v.clone();
    let d = r.as_object_mut().ok_or("missing native row")?;
    d.remove("compact");
    d.remove("profile");
    Ok(r)
}
pub(crate) fn summarize(dir: &Path, reference: &Path) -> Result<()> {
    let mut rows = Fields::new();
    let mut counts = BTreeMap::new();
    let mut selected = None;
    for k in 0..1250 {
        let file = format!("case-{k:04}.json");
        if !dir.join("cases").join(&file).exists() || !reference.join("cases").join(&file).exists()
        {
            continue;
        }
        let (a, b) = (fixture(dir, k)?, fixture(reference, k)?);
        if !((a["format"] == "certified-primary-tail-v1"
            && b["format"] == "subdivided-original-primary-v1")
            || (a["format"] == "directed-primary-tail-v1"
                && b["format"] == "directed-original-primary-v1"))
            || a["result"]["fixture"] != b["result"]["fixture"]
        {
            return Err("wrong compact field".into());
        }
        for t in array(&a["result"]["trials"])? {
            let q = matched(&b, t)?;
            let id = 3 * k + uint(&t["stroke"])? as usize - 1;
            count(&mut counts, "landings");
            let m = &q["mechanics"][1];
            let m0 = &q["mechanics"][0];
            let qualified = m["end"] == "original_clock"
                && m0["end"] == "original_clock"
                && vec3(&m["j"])?.distance(vec3(&m0["j"])?) < 1e-5
                && vec3(&m["com"])?.distance(vec3(&m0["com"])?) < 1e-7
                && num(&m["arc_energy_defect"])?.abs() < 1e-7
                && array(&m["events"])?.len() == array(&m0["events"])?.len()
                && array(&m["events"])?
                    .iter()
                    .zip(array(&m0["events"])?)
                    .all(|(x, y)| {
                        ["feature", "stuck", "mode"]
                            .iter()
                            .all(|key| x[key] == y[key])
                    });
            if qualified {
                count(&mut counts, "qualified")
            }
            for (i, n, ix) in [(0, 2, 1), (1, 8, 5), (2, 64, 7)] {
                let fixed = &t["paths"][2 * i];
                let compact = &t["paths"][2 * i + 1];
                let label = format!("compact{n}");
                if strip(fixed)? != strip(&q["paths"][ix])? || fixed["first"]["native_steps"] != n {
                    return Err(format!("native control changed {id}/{n}").into());
                }
                let accepted = !compact["compact"].is_null();
                let target = if accepted { &t["paths"][0] } else { fixed };
                let mut actual = strip(compact)?;
                let expected = strip(target)?;
                let calls = uint(&compact["first"]["native_steps"])?;
                if accepted {
                    let r = &compact["compact"];
                    let tail = uint(&r["duration_ns"])?;
                    let preflight = n - (n - 1).min(tail);
                    // A tail shorter than n-1 ns cannot have n-1 positive steps.
                    // Only extra preflight integrations can round the free-flight
                    // positions differently. Every twist, attitude, load, event,
                    // and time must still reproduce the two-call control exactly.
                    if preflight > 1 {
                        count(&mut counts, &format!("{label}_split_preflight"));
                        if fixed["hits"][0] != compact["hits"][0] {
                            return Err(
                                format!("split preflight changed first impact {id}/{n}").into()
                            );
                        }
                        for body in ["foot", "upper", "pad"] {
                            let pos = &expected["first"][body]["position"];
                            if vec3(&actual["first"][body]["position"])?.distance(vec3(pos)?)
                                > 2e-13
                            {
                                return Err(format!(
                                    "split preflight changed {body} position {id}/{n}"
                                )
                                .into());
                            }
                            actual["first"][body]["position"] = pos.clone();
                        }
                        actual["first"]["native_steps"] = expected["first"]["native_steps"].clone();
                    }
                    if calls != preflight + 1
                        || uint(&r["omitted"])? + calls != n
                        || uint(&r["start_ns"])? + uint(&r["duration_ns"])? != 500000
                        || compact["first"]["first_local_ns"] != r["start_ns"]
                        || array(&compact["spans"])?.len() != 1
                        || compact["spans"][0][0] != r["start_ns"]
                        || compact["spans"][0][1] != r["duration_ns"]
                    {
                        return Err("unaccounted compressed clock".into());
                    }
                    count(&mut counts, &format!("{label}_accepted"));
                }
                if actual != expected {
                    return Err(format!("compact did not reproduce chosen grid {id}/{n}").into());
                }
                add(&mut rows, &label, "native_calls", calls as f64, id);
                if !qualified {
                    continue;
                }
                let physical = vec3(&m["com"])? + 0.0005 * vec3(&q["deck_v"])?;
                let current = com(compact)?;
                let prev = com(fixed)?;
                let dc = current.distance(physical);
                let dj = vec3(&m["j"])?.distance(vec3(&compact["j"])?);
                add(&mut rows, &label, "com_error_m", dc, id);
                add(&mut rows, &label, "j_error_ns", dj, id);
                if accepted {
                    if array(&m["events"])?.len() != array(&compact["hits"])?.len()
                        || dc > 1e-8
                        || dj > 5e-6
                    {
                        return Err(format!("compact unproved {id}/{n}").into());
                    }
                    add(&mut rows, &label, "accepted_com_error_m", dc, id);
                    add(&mut rows, &label, "accepted_j_error_ns", dj, id);
                    let shift = current.distance(prev);
                    if n == 8 && selected.is_none_or(|(v, _)| shift > v) {
                        selected = Some((shift, id))
                    }
                }
            }
        }
        count(&mut counts, "fixtures");
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("fixtures")==Some(&1250),"counts":counts,"largest_eight_shift_m_case":selected,"rows":values(rows)}),
    )
}
