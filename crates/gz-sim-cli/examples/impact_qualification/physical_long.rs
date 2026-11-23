// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check a common cold re-import after candidate landing responses. The warm
//! histories supply independent first-boundary controls on the same SDF cut.
use super::*;
// The native duration converts integer nanoseconds by division; the reference
// field multiplies by 1e-9. Check discrete decisions exactly and bound all
// floating outputs, including individual event clocks and force integrals.
fn same_mechanics(a: &Value, b: &Value) -> bool {
    match (a, b) {
        (Value::Object(a), Value::Object(b)) => {
            a.len() == b.len()
                && a.iter()
                    .all(|(k, v)| b.get(k).is_some_and(|u| same_mechanics(v, u)))
        }
        (Value::Array(a), Value::Array(b)) => {
            a.len() == b.len() && a.iter().zip(b).all(|(x, y)| same_mechanics(x, y))
        }
        (Value::Number(a), Value::Number(b)) if a.is_f64() && b.is_f64() => {
            let (x, y) = (a.as_f64().unwrap(), b.as_f64().unwrap());
            (x - y).abs() <= 2e-12 * (1. + x.abs().max(y.abs()))
        }
        _ => a == b,
    }
}
pub(crate) fn summarize(dir: &Path, warm: &Path, clocked: &Path) -> Result<()> {
    let names = [
        "native8_cold512",
        "endpoint8_cold512",
        "native4096_cold512",
        "native8192_cold512",
        "mechanics_cold512",
    ];
    let mut rows = Rows::new();
    let mut counts = BTreeMap::<String, usize>::new();
    let mut largest = None;
    for k in 0..50 {
        if !file(dir, k).exists() || !file(warm, k).exists() {
            continue;
        }
        let j = 17 * k % 50;
        let donor = (0..1250)
            .find(|i| i * 613 % 1250 == 50 * (j / 2) + 24 + j % 2)
            .unwrap();
        let where_clock = clocked.join("cases").join(format!("case-{donor:04}.json"));
        if !where_clock.exists() {
            continue;
        }
        let archive = read(&file(dir, k))?;
        let w = read(&file(warm, k))?;
        let d = read(&where_clock)?;
        let t = &d["result"]["trials"][0];
        let mechanical = &t["response"];
        if archive["format"] != "cold-boundary-secondary-mechanics-v1"
            || archive["case"] != k
            || w["source"] != archive["source"]
            || t["cut"] != archive["cut"]
            || t["stroke"] != 1
            || d["format"] != "finite-clocked-endpoint-sequence-v1"
        {
            return Err("unmatched physical history".into());
        }
        cross_checks(&archive, &w)?;
        *counts.entry("cases".into()).or_default() += 1;
        if archive["offset_ns"] != w["offset_ns"] || archive["offset_ns"] != mechanical["offset_ns"]
        {
            return Err("different restart clocks".into());
        }
        for (key, i) in [("mechanics128", 1), ("mechanics", 2)] {
            if !same_mechanics(&archive[key], &mechanical["paths"][i]) {
                return Err(format!(
                    "independent mechanical arc reconstruction changed for {k} {key}"
                )
                .into());
            }
        }
        if archive["paths"].is_null() {
            let reason = if archive["mechanics"].is_null() {
                "no_secondary"
            } else if archive["unresolved_mesh"] == true {
                "mesh_disagreement"
            } else {
                archive["mechanics"]["end"]
                    .as_str()
                    .ok_or("missing refusal")?
            };
            *counts.entry(reason.into()).or_default() += 1;
            continue;
        }
        if w["initial"] != archive["initial"] {
            return Err("different secondary states".into());
        }
        let cut = uint(&archive["cut_tick"])?;
        let end = uint(&archive["end_tick"])?;
        let p = labels(
            &archive["paths"],
            &names,
            end - cut,
            &[8, 8, 4096, 8192, 0],
            &[512; 5],
        )?;
        let fine = &archive["mechanics"];
        let h = num(&fine["horizon_s"])?;
        close(h, (500_000 - uint(&archive["offset_ns"])?) as f64 * 1e-9)?;
        let v = vec3(&archive["initial"]["pad"]["velocity"])?;
        let shift = (num(&w["geometric_time_s"])? + h) * v;
        close(
            com(&p[4]["first"])?.distance(vec3(&fine["com_m"])? + shift),
            0.,
        )?;
        close(
            num(&p[4]["first"]["normal_impulse_ns"])?,
            num(&fine["normal_impulse_ns"])?,
        )?;
        *counts.entry("matched".into()).or_default() += 1;
        for (i, wi) in [(0, 0), (1, 1), (2, 3), (3, 5)] {
            let reference = &w["paths"][wi];
            if reference["first"] != p[i]["first"] {
                return Err(format!(
                    "native first-boundary result changed for case {k} {}",
                    names[i]
                )
                .into());
            }
            if i != 3 {
                endpoint(
                    &mut rows,
                    &format!("cold_vs_retained_{}", names[i]),
                    &p[i],
                    reference,
                    k,
                )?;
            }
        }
        for (i, a) in p.iter().enumerate() {
            if a["first"]["tick"] != cut + 1 || a["end"]["tick"] != end {
                return Err("changed actuator phase".into());
            }
            for (j, ix) in [0, 2, 3, 4].into_iter().enumerate() {
                let q = &a["comparisons"][j];
                let other = &p[ix];
                let label = format!("{}_vs_{}", names[i], names[ix]);
                close(
                    com(&a["end"])?.distance(com(&other["end"])?),
                    num(&q["final_com_m"])?,
                )?;
                endpoint(&mut rows, &label, a, other, k)?;
                store(
                    &mut rows,
                    &label,
                    "first_com_m",
                    com(&a["first"])?.distance(com(&other["first"])?),
                    k,
                );
                for key in [
                    "rms_com_m",
                    "max_com_m",
                    "rms_spin_rad_s",
                    "max_public_reaction_difference_ns",
                ] {
                    store(&mut rows, &label, key, num(&q[key])?, k)
                }
            }
        }
        let dist = num(&p[4]["comparisons"][0]["final_com_m"])?;
        if largest.is_none_or(|(v, _)| dist > v) {
            largest = Some((dist, k))
        }
        for (i, reference) in [(1, "4096"), (2, "8192")] {
            for key in ["rms_com_m", "final_com_m"] {
                let change =
                    num(&p[4]["comparisons"][i][key])? - num(&p[0]["comparisons"][i][key])?;
                let ranking = if change < -1e-12 {
                    "closer"
                } else if change > 1e-12 {
                    "farther"
                } else {
                    "tie"
                };
                *counts
                    .entry(format!("mechanics_vs_{reference}_{key}_{ranking}"))
                    .or_default() += 1;
                store(
                    &mut rows,
                    &format!("mechanics_minus_native8_vs_{reference}"),
                    key,
                    change,
                    k,
                )
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"complete":counts.get("cases")==Some(&50),"largest_causal_terminal_m_case":largest,"rows":rows_value(rows)}),
    )
}
