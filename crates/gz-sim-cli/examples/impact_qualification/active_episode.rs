// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare only contemporaneous public clocks of consecutive Bevy
//! updates. Stop each independent mechanical reference at its first unproved mode.
use super::*;
const LABELS: [&str; 5] = [
    "onset_budget",
    "retain_budget",
    "compact_active",
    "native8192",
    "compact_onset",
];
fn center(v: &Value) -> Result<DVec3> {
    Ok(0.5 * (vec3(&v["foot"]["position"])? + vec3(&v["upper"]["position"])?))
}
pub(crate) fn summarize(dir: &Path, whole: &Path, previous: Option<&Path>) -> Result<()> {
    let mut counts = BTreeMap::new();
    let mut fields = Fields::new();
    let mut selected = None;
    let mut lengths = vec![];
    for k in 0..50 {
        let raw = dir.join("cases").join(format!("case-{k:03}.json"));
        if !raw.exists() {
            continue;
        }
        let a = read(&raw)?;
        let control = read(&whole.join("cases").join(format!("case-{k:03}.json")))?;
        if a["format"] != "factorial-multi-command-material-point-v1"
            || a["case"] != k
            || a["cut"] != control["cut"]
            || a["source"] != control["source"]
            || a["profiles"] != json!(LABELS)
        {
            return Err("wrong episode donor".into());
        }
        count(&mut counts, "worlds");
        let refs = array(&a["references"])?;
        if refs.is_empty() || refs[0]["fine"] != control["mechanics"] {
            return Err("different mechanical first landing".into());
        }
        let paths = array(&a["paths"])?;
        if paths.len() != 5
            || paths
                .iter()
                .any(|p| p.as_array().is_none_or(|v| v.len() != 26))
            || paths[0][0] != paths[1][0]
            || paths[4][0] != paths[2][0]
        {
            return Err("different initial treatment or episode length".into());
        }
        if let Some(previous) = previous {
            let old = read(&previous.join("cases").join(format!("case-{k:03}.json")))?;
            if old["initial"] != a["initial"]
                || old["references"] != a["references"]
                || array(&old["paths"])? != &paths[..4]
            {
                return Err("the four established histories failed to repeat".into());
            }
        }
        let cut = uint(&a["cut"]["tick"])?;
        let mut total = [DVec3::ZERO; 5];
        let mut calls = [0; 5];
        let mut qualified = 0;
        for i in 0..26 {
            let clock = i + 1;
            let hours = clock as f64 * 0.0005;
            let g = format!("{clock:02}_ticks");
            let mut next = Vec::new();
            for (ix, row) in paths.iter().map(|p| &p[i]).enumerate() {
                let n = uint(&row["native_steps"])?;
                let om = uint(&row["omitted"])?;
                if row["tick"] != cut + clock as u64
                    || row["time_us"] != (cut + clock as u64) * 500
                    || row["drive"] != paths[0][i]["drive"]
                    || num(&row["other_pair_normal_ns"])? != 0.
                    || uint(&row["covered_ns"])? > 500000
                    || num(&row["normal_impulse_ns"])? < 0.
                {
                    return Err("unmatched episode clock or contact".into());
                }
                if ix == 3 {
                    if n != 8192 {
                        return Err("wrong native fine budget".into());
                    }
                } else if n + om != 1 && n + om != 8 {
                    return Err("unaccounted active native budget".into());
                }
                if (ix == 1 || ix == 2)
                    && i > 0
                    && paths[ix][i - 1]["point_at_end"] == true
                    && n + om != 8
                {
                    return Err("lost prior active budget".into());
                }
                if ix < 3 && row["point_at_end"] == true {
                    count(&mut counts, &format!("{}_at_end", LABELS[ix]));
                }
                if om > 0 && ix == 2 {
                    count(
                        &mut counts,
                        if i == 0 {
                            "compact_primary"
                        } else {
                            "compact_later"
                        },
                    );
                }
                if ix == 1 && i > 0 && n > 1 && paths[0][i]["native_steps"] == 1 {
                    count(&mut counts, "additional_refined_ticks");
                }
                total[ix] += vec3(&row["j"])?;
                calls[ix] += n;
                next.push(center(row)?);
                let target = center(&paths[3][i])?;
                add(
                    &mut fields,
                    &format!("{g}_{}", LABELS[ix]),
                    "vs_native8192_m",
                    next[ix].distance(target),
                    k,
                );
                add(
                    &mut fields,
                    &format!("{g}_{}", LABELS[ix]),
                    "cumulative_calls",
                    calls[ix] as f64,
                    k,
                );
            }
            if clock == 26 {
                for (x, y) in [(1, 0), (2, 0), (2, 1), (2, 4), (4, 0)] {
                    let delta = next[x].distance(next[3]) - next[y].distance(next[3]);
                    count(
                        &mut counts,
                        &format!("{}_vs_{}_13ms_native_{}", LABELS[x], LABELS[y], rank(delta)),
                    );
                }
            }
            if qualified != i {
                continue;
            }
            let Some(r) = refs.get(i) else { continue };
            let (a0, b) = (&r["coarse"], &r["fine"]);
            if r["at"] != clock {
                return Err("misaligned physical interval".into());
            }
            if a0["end"] != "original_clock" || b["end"] != "original_clock" {
                continue;
            }
            if paths[0][i]["drive"] != a["drive"][2] {
                return Err("physical control after actuator change".into());
            }
            let (ae, be) = (array(&a0["events"])?, array(&b["events"])?);
            if ae.len() != be.len()
                || !ae.iter().zip(be).all(|(x, y)| {
                    ["feature", "stuck", "mode"]
                        .iter()
                        .all(|key| x[key] == y[key])
                })
                || vec3(&a0["j"])?.distance(vec3(&b["j"])?) > 1e-5
                || vec3(&a0["com"])?.distance(vec3(&b["com"])?) > 1e-7
                || num(&b["arc_energy_defect"])?.abs() > 1e-7
            {
                continue;
            }
            qualified = clock;
            close((num(&b["elapsed"])? - hours).abs(), 1e-12)?;
            let exact = vec3(&b["com"])? + hours * vec3(&a["drive"])?;
            let force = vec3(&b["j"])?;
            for ix in 0..LABELS.len() {
                let label = format!("{g}_{}", LABELS[ix]);
                add(
                    &mut fields,
                    &label,
                    "vs_mechanics_com_m",
                    next[ix].distance(exact),
                    k,
                );
                add(
                    &mut fields,
                    &label,
                    "vs_mechanics_cumulative_j_ns",
                    total[ix].distance(force),
                    k,
                );
            }
            for (x, y) in [(1, 0), (2, 0), (2, 1), (2, 4), (4, 0)] {
                let d = next[x].distance(exact) - next[y].distance(exact);
                count(
                    &mut counts,
                    &format!("{g}_{}_vs_{}_{}", LABELS[x], LABELS[y], rank(d)),
                );
                if x == 2 && y == 0 && clock == 8 && selected.is_none_or(|(v, _)| d.abs() > v) {
                    selected = Some((d.abs(), k))
                }
            }
        }
        lengths.push(qualified);
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("worlds")==Some(&50),"counts":counts,"qualified_consecutive_physical_ticks":lengths,"largest_absolute_4ms_error_change_m_case":selected,"rows":values(fields)}),
    )
}
fn rank(d: f64) -> &'static str {
    if d < -1e-12 {
        "closer"
    } else if d > 1e-12 {
        "farther"
    } else {
        "tie"
    }
}
