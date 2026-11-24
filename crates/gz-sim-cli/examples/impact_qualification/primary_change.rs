// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify the effect of subdividing a certificate at fixed native work.
use super::*;
fn com(r: &Value) -> Result<DVec3> {
    Ok(0.5 * (vec3(&r["first"]["foot"]["position"])? + vec3(&r["first"]["upper"]["position"])?))
}
fn complete(r: &Value) -> Result<bool> {
    let Some(first) = array(&r["hits"])?.first() else {
        return Ok(false);
    };
    let mut at = (num(&first["at_s"])? * 1e9).round() as u64;
    for s in array(&r["spans"])? {
        if uint(&s[0])? != at {
            return Ok(false);
        }
        at += uint(&s[1])?;
    }
    Ok(at == 500000)
}
pub(crate) fn summarize(dir: &Path, old: &Path) -> Result<()> {
    let mut counts = BTreeMap::new();
    let mut rows = Fields::new();
    let mut worst = None;
    let mut maximum = None;
    for k in 0..1250 {
        let f = format!("case-{k:04}.json");
        if !dir.join("cases").join(&f).exists() || !old.join("cases").join(&f).exists() {
            continue;
        }
        let (a, b) = (fixture(dir, k)?, fixture(old, k)?);
        for t in array(&a["result"]["trials"])? {
            let o = matched(&b, t)?;
            if t["cut"] != o["cut"]
                || t["deck_v"] != o["deck_v"]
                || t["mechanics"] != o["mechanics"]
                || t["geometry"] != o["geometry"]
            {
                return Err("different matched first landing".into());
            }
            count(&mut counts, "landings");
            let id = 3 * k + uint(&t["stroke"])? as usize - 1;
            for i in [0, 2, 3, 4, 8, 9] {
                if t["paths"][i] != o["paths"][i] {
                    return Err(
                        format!("changed native, spatial or endpoint control {id}/{i}").into(),
                    );
                }
            }
            let m = &t["mechanics"][1];
            let m0 = &t["mechanics"][0];
            if m["end"] != "original_clock" || m0["end"] != "original_clock" {
                continue;
            }
            if vec3(&m["com"])?.distance(vec3(&m0["com"])?) >= 1e-7
                || vec3(&m["j"])?.distance(vec3(&m0["j"])?) >= 1e-5
                || num(&m["arc_energy_defect"])?.abs() >= 1e-7
            {
                continue;
            }
            let (me, co) = (array(&m["events"])?, array(&m0["events"])?);
            if me.len() != co.len()
                || !me.iter().zip(co).all(|(x, y)| {
                    ["feature", "stuck", "mode"]
                        .iter()
                        .all(|key| x[key] == y[key])
                })
            {
                continue;
            }
            count(&mut counts, "qualified");
            let q = vec3(&m["com"])? + 0.0005 * vec3(&t["deck_v"])?;
            let j = vec3(&m["j"])?;
            for i in [1, 5, 6, 7] {
                let (x, y) = (&t["paths"][i], &o["paths"][i]);
                let name = x["profile"].as_str().ok_or("missing profile")?;
                if x["profile"] != y["profile"]
                    || x["first"]["native_steps"] != y["first"]["native_steps"]
                    || array(&x["hits"])?.first() != array(&y["hits"])?.first()
                {
                    return Err("changed incoming impact or outer budget".into());
                }
                let (a, b) = (com(x)?, com(y)?);
                let shift = a.distance(b);
                let delta = a.distance(q) - b.distance(q);
                let jdelta = vec3(&x["j"])?.distance(j) - vec3(&y["j"])?.distance(j);
                let rank = if delta < -1e-13 {
                    "closer"
                } else if delta > 1e-13 {
                    "farther"
                } else {
                    "tie"
                };
                count(&mut counts, &format!("{name}_{rank}"));
                let known = complete(y)?;
                let now = complete(x)?;
                if known && !now {
                    return Err(format!("lost complete interval {id} {name}").into());
                }
                if !known && now {
                    count(&mut counts, &format!("{name}_newly_complete"))
                }
                if x != y {
                    count(&mut counts, &format!("{name}_changed"))
                }
                if x["hits"] == y["hits"] && x["spans"] == y["spans"] {
                    close(shift, 1e-11)?;
                    close(vec3(&x["j"])?.distance(vec3(&y["j"])?), 1e-8)?;
                }
                add(&mut rows, name, "position_shift_m", shift, id);
                add(&mut rows, name, "signed_position_error_change_m", delta, id);
                add(&mut rows, name, "signed_vector_error_change_ns", jdelta, id);
                if i == 5 {
                    if maximum.is_none_or(|(v, _)| shift > v) {
                        maximum = Some((shift, id))
                    }
                    if worst.is_none_or(|(v, _)| delta > v) {
                        worst = Some((delta, id))
                    }
                }
            }
        }
        count(&mut counts, "fixtures");
    }
    write(
        &dir.join("paired-summary.json"),
        &json!({"complete":counts.get("fixtures")==Some(&1250),"counts":counts,"largest_eight_shift_m_case":maximum,"largest_eight_regression_m_case":worst,"rows":values(rows)}),
    )
}
