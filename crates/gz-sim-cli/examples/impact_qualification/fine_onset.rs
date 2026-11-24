// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct first differences from actual common bin ends on three native grids.
use super::*;
const COMMON: usize = 8192;
#[derive(Clone, Copy)]
struct Sample {
    c: DVec3,
    v: DVec3,
    w: DVec3,
    j: DVec3,
    n: f64,
}
fn bins(path: &Path) -> Result<Vec<Vec<Sample>>> {
    let bytes = fs::read(path)?;
    if bytes.len() != 3 * (COMMON + 1) * 13 * 8 {
        return Err("incomplete binary observation file".into());
    }
    let mut out = vec![];
    for native in bytes.chunks_exact((COMMON + 1) * 13 * 8) {
        let mut rows = Vec::with_capacity(COMMON + 1);
        for raw in native.chunks_exact(13 * 8) {
            let f = |i: usize| f64::from_le_bytes(raw[8 * i..8 * i + 8].try_into().unwrap());
            let v = |i| DVec3::new(f(i), f(i + 1), f(i + 2));
            let row = Sample {
                c: v(0),
                v: v(3),
                w: v(6),
                j: v(9),
                n: f(12),
            };
            if ![row.c, row.v, row.w, row.j]
                .into_iter()
                .all(DVec3::is_finite)
                || !row.n.is_finite()
            {
                return Err("invalid observed value".into());
            }
            rows.push(row);
        }
        out.push(rows);
    }
    Ok(out)
}
pub(crate) fn summarize(dir: &Path, deep: &Path) -> Result<()> {
    let mut fields = Rows::new();
    let mut counts = BTreeMap::<String, usize>::new();
    let mut maximum = None;
    let mut selected = vec![];
    for k in 0..50 {
        let name = format!("case-{k:03}");
        let control = dir.join("controls").join(format!("{name}.json"));
        if !control.exists() {
            continue;
        }
        let (raw, refn, long) = (read(&file(dir, k))?, read(&control)?, read(&file(deep, k))?);
        let case = &long["result"];
        let geom = &raw["geometry"];
        let arrival = uint(&geom["hi_ns"])?;
        if raw["case"] != k
            || refn["case"] != k
            || raw["cut"] != case["cut"]
            || refn["geometry"] != *geom
            || refn["raw_cut"] != raw["cut"]
        {
            return Err("unmatched onset control".into());
        }
        *counts.entry("cases".into()).or_default() += 1;
        let series = bins(&dir.join("bins").join(format!("{name}.bin")))?;
        let mut firsts = vec![];
        let first_physical = vec3(&geom["instantaneous_impulse_ns"])?;
        for (i, grid) in [8192, 16384, 32768].into_iter().enumerate() {
            let samples = &series[i];
            let public = &raw["lanes"][i]["total"];
            let hit = &raw["lanes"][i]["first"];
            let group = grid.to_string();
            let (a, b) = (uint(&hit["start_ns"])?, uint(&hit["end_ns"])?);
            let initial = samples[0];
            let end = samples[COMMON];
            if uint(&public["calls"])? != grid
                || b <= a
                || case["paths"][i]["first"]["first_local_ns"] != b
            {
                return Err("bad native onset clock".into());
            }
            let mut j = DVec3::ZERO;
            let mut n = 0.;
            for s in array(&hit["sites"])? {
                j += vec3(&s["j_ns"])?;
                n += num(&s["normal_ns"])?;
                close(vec3(&s["normal"])?.length(), 1.)?;
            }
            close(j.distance(vec3(&hit["j_ns"])?), 0.)?;
            close(n, num(&hit["normal_ns"])?)?;
            close(end.j.distance(vec3(&public["j_ns"])?), 0.)?;
            close(end.n, num(&public["normal_ns"])?)?;
            close(initial.c.distance(vec3(&raw["cut"]["com_m"])?), 0.)?;
            close(
                initial.v.distance(vec3(&raw["cut"]["com_velocity_m_s"])?),
                0.,
            )?;
            let mut error = 0.0_f64;
            let mut last = 0.;
            let mut first_bin = None;
            for (t, s) in samples.iter().enumerate() {
                let ns = t as u64 * 500000 / COMMON as u64;
                if s.n < last || (ns < b && (s.n != 0. || s.j != DVec3::ZERO)) {
                    return Err("reaction before native onset".into());
                }
                if first_bin.is_none() && s.n > 0. {
                    first_bin = Some(t);
                }
                let g = -9.81 * DVec3::Z;
                let time = ns as f64 * 1e-9;
                error = error.max((2. * (s.v - initial.v - time * g) - s.j).length());
                last = s.n;
            }
            if first_bin != Some((b * COMMON as u64).div_ceil(500000) as usize) {
                return Err("nested common bin misses first reaction".into());
            }
            store(
                &mut fields,
                &group,
                "first_partial_j_error_ns",
                j.distance(first_physical),
                k,
            );
            store(
                &mut fields,
                &group,
                "first_paid_end_minus_geometry_ns",
                b as f64 - arrival as f64,
                k,
            );
            store(&mut fields, &group, "linear_momentum_residual_ns", error, k);
            firsts.push((b, j));
            let drive = vec3(&refn["deck_v"])?;
            for item in array(&refn["offsets_ns_and_bins"])? {
                let requested = uint(&item[0])?;
                let bin = uint(&item[1])?;
                let reference = &refn["samples"][bin.to_string()];
                let time = uint(&reference["ns"])? as f64 / 1e9;
                if reference["ns"] != bin * 500000 / COMMON as u64 {
                    return Err("wrong reference quadrature time".into());
                }
                let (fine, coarse) = (&reference["paths"][1], &reference["paths"][0]);
                if fine["end"] != "original_clock"
                    || coarse["end"] != "original_clock"
                    || vec3(&fine["j"])?.distance(vec3(&coarse["j"])?) > 1e-5
                    || vec3(&fine["com"])?.distance(vec3(&coarse["com"])?) > 1e-7
                {
                    return Err("unqualified mechanical onset control".into());
                }
                let s = samples[bin as usize];
                for (label, v) in [
                    ("com_m", s.c.distance(vec3(&fine["com"])? + time * drive)),
                    ("impulse_ns", s.j.distance(vec3(&fine["j"])?)),
                    (
                        "velocity_m_s",
                        s.v.distance(vec3(&fine["velocity"])? + drive),
                    ),
                    ("spin_rad_s", s.w.distance(vec3(&fine["omega"])?)),
                ] {
                    store(
                        &mut fields,
                        &format!("{grid}_after_{requested}ns"),
                        label,
                        v,
                        k,
                    );
                }
            }
        }
        for (pair, (i, j)) in [(0, 1), (1, 2), (0, 2)].into_iter().enumerate() {
            let label = format!("{}_vs_{}", [8192, 16384, 32768][i], [8192, 16384, 32768][j]);
            let prev = &raw["pairs"][pair];
            let (first, second) = (&series[i], &series[j]);
            let onset = firsts[i].0.min(firsts[j].0);
            let mut thresh = [None; 8];
            let mut pre = [0.0_f64; 3];
            let mut peak = [0.0_f64; 3];
            for (t, (a, b)) in first.iter().zip(second).enumerate() {
                let at = t as u64 * 500000 / COMMON as u64;
                let (d, v, j, w) = (
                    a.c.distance(b.c),
                    a.v.distance(b.v),
                    a.j.distance(b.j),
                    a.w.distance(b.w),
                );
                for (o, lim) in thresh.iter_mut().zip([
                    d > 1e-12,
                    d > 1e-9,
                    d > 1e-7,
                    d > 1e-6,
                    j > 1e-8,
                    j > 1e-3,
                    w > 1e-4,
                    v > 1e-4,
                ]) {
                    if o.is_none() && lim {
                        *o = Some(at);
                    }
                }
                for (k, z) in [d, v, j].into_iter().enumerate() {
                    peak[k] = peak[k].max(z);
                    if at < onset {
                        pre[k] = pre[k].max(z);
                    }
                }
            }
            if json!(thresh) != prev["first_common_bin_ns"] {
                return Err("earliest public divergence changed".into());
            }
            for m in 0..3 {
                close(pre[m], num(&prev["unloaded_peak_com_velocity_impulse"][m])?)?;
                close(peak[m], num(&prev["peak_com_velocity_impulse"][m])?)?;
            }
            for (key, x) in array(&raw["keys"])?.iter().zip(thresh) {
                let Some(x) = x else { continue };
                let key = key.as_str().ok_or("bad threshold")?;
                store(
                    &mut fields,
                    &label,
                    &format!("first_{key}_minus_geometric_ns"),
                    x as f64 - arrival as f64,
                    k,
                );
                *counts.entry(format!("{label}_{key}_crossed")).or_default() += 1;
            }
            for (key, x) in [
                ("peak_unloaded_com_m", pre[0]),
                ("peak_unloaded_velocity_m_s", pre[1]),
                ("peak_com_m", peak[0]),
                ("peak_impulse_ns", peak[2]),
                (
                    "first_partial_impulse_difference_ns",
                    firsts[i].1.distance(firsts[j].1),
                ),
            ] {
                store(&mut fields, &label, key, x, k);
            }
            if pair == 1 {
                let x = firsts[i].1.distance(firsts[j].1);
                if maximum.is_none_or(|(v, _)| x > v) {
                    maximum = Some((x, k));
                }
                selected.push(json!({"case":k,"source":raw["source"],"geometric_ns":arrival,"first_native_ns":[firsts[i].0,firsts[j].0],"first_common_bin_ns":thresh,"unloaded_com_m":pre[0],"first_partial_impulse_difference_ns":x,"terminal_com_m":case["paths"][1]["against"][2]["final_com_m"]}));
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("cases")==Some(&50),"counts":counts,"largest_first_partial_impulse_difference_ns_case":maximum,"cases":selected,"rows":rows_value(fields)}),
    )
}

/// Independently recover the first shared bin crossing in the next public
/// solve. Compare to both the cold fine histories and original-clock mechanics.
pub(crate) fn successor(dir: &Path, episodes: &Path, suffix: &Path) -> Result<()> {
    let mut fields = Rows::new();
    let mut counts = BTreeMap::<String, usize>::new();
    let mut cases = Vec::new();
    let mut max = None;
    for k in 0..50 {
        let name = format!("case-{k:03}");
        let doc = read(&file(dir, k))?;
        let refn = read(&file(episodes, k))?;
        let primary = &refn["references"][0]["fine"];
        let next = &refn["references"][1]["fine"];
        let control = &refn["references"][1]["coarse"];
        let cut = uint(&doc["cut"]["tick"])?;
        if doc["format"] != "physical-successor-onset-v1"
            || doc["case"] != k
            || doc["cut"] != refn["cut"]
            || doc["source"] != refn["source"]
            || doc["primary_mechanics"] != *primary
            || doc["clock_origin_ns"] != (cut + 1) * 500000
            || doc["boundary"]["tick"] != cut + 1
            || next["end"] != "original_clock"
            || control["end"] != "original_clock"
            || array(&next["events"])?.len() != array(&control["events"])?.len()
            || !array(&next["events"])?
                .iter()
                .zip(array(&control["events"])?)
                .all(|(x, y)| {
                    ["mode", "feature", "stuck"]
                        .iter()
                        .all(|key| x[key] == y[key])
                })
            || num(&next["arc_energy_defect"])?.abs() > 1e-7
            || vec3(&next["com"])?.distance(vec3(&control["com"])?) > 1e-7
            || vec3(&next["j"])?.distance(vec3(&control["j"])?) > 1e-5
        {
            return Err(format!("unqualified common landing {k}").into());
        }
        let common = bins(&dir.join("bins").join(format!("{name}.bin")))?;
        let drive = vec3(&refn["drive"])?;
        let truth = vec3(&next["com"])? + 0.001 * drive;
        let response = vec3(&next["j"])? - vec3(&primary["j"])?;
        let next_event = array(&next["events"])?
            .iter()
            .map(|v| num(&v["at_s"]))
            .collect::<Result<Vec<_>>>()?
            .into_iter()
            .find(|&s| s > 0.0005)
            .map(|s| (s - 0.0005) * 1e9);
        let mut threshold = None;
        for (i, n) in [8192, 16384, 32768].into_iter().enumerate() {
            let archive = suffix
                .join(&name)
                .join("traces")
                .join(format!("native{n}.json"));
            let r = &doc["lanes"][i];
            let total = &r["total"];
            let s = &common[i];
            let first = s[0];
            let last = s[COMMON];
            if first.c != common[0][0].c
                || first.v != common[0][0].v
                || first.w != common[0][0].w
                || first.n != 0.
                || first.j != DVec3::ZERO
                || first.c.distance(com(&doc["boundary"])?) > 2e-11
                || first.v.distance(vec3(&primary["velocity"])? + drive) > 2e-11
                || first.w.distance(vec3(&primary["omega"])?) > 2e-11
                || total["calls"] != n
            {
                return Err("unmatched incoming states".into());
            }
            close(last.j.distance(vec3(&total["j_ns"])?), 0.)?;
            close(last.n, num(&total["normal_ns"])?)?;
            close(
                last.c.distance(
                    0.5 * (vec3(&total["public_foot"]["position"])?
                        + vec3(&total["public_upper"]["position"])?),
                ),
                0.,
            )?;
            if archive.exists() {
                let trace = read(&archive)?;
                let a = &trace["frames"][1];
                if trace["frames"][0] != doc["boundary"]
                    || a["tick"] != cut + 2
                    || a["native_steps"] != n
                    || a["foot"] != total["public_foot"]
                    || a["upper"] != total["public_upper"]
                    || a["pad"] != total["public_pad"]
                    || a["first_local_ns"] != r["first"]["end_ns"]
                {
                    return Err(format!("separate native suffix differs {k}/{n}").into());
                }
                *counts
                    .entry("independent_suffix_endpoints".into())
                    .or_default() += 1;
            }
            let mut last_n = 0.;
            let mut residual = 0f64;
            for (j, row) in s.iter().enumerate() {
                if row.n < last_n {
                    return Err("decreasing integrated normal impulse".into());
                }
                last_n = row.n;
                let t = (j as u64 * 500000 / COMMON as u64) as f64 * 1e-9;
                residual =
                    residual.max((2. * (row.v - first.v + 9.81 * t * DVec3::Z) - row.j).length());
            }
            let label = format!("native{n}");
            store(&mut fields, &label, "max_momentum_residual_ns", residual, k);
            store(
                &mut fields,
                &label,
                "physical_end_com_error_m",
                truth.distance(last.c),
                k,
            );
            store(
                &mut fields,
                &label,
                "physical_incremental_impulse_error_ns",
                response.distance(last.j),
                k,
            );
        }
        for (p, (i, j)) in [(0, 1), (1, 2), (0, 2)].into_iter().enumerate() {
            let (a, b) = (&common[i], &common[j]);
            let mut first = [None; 8];
            for (l, (x, y)) in a.iter().zip(b).enumerate() {
                let d = x.c.distance(y.c);
                let j = x.j.distance(y.j);
                let w = x.w.distance(y.w);
                let v = x.v.distance(y.v);
                for (o, test) in first.iter_mut().zip([
                    d > 1e-12,
                    d > 1e-9,
                    d > 1e-7,
                    d > 1e-6,
                    j > 1e-8,
                    j > 1e-3,
                    w > 1e-4,
                    v > 1e-4,
                ]) {
                    if test && o.is_none() {
                        *o = Some(l as u64 * 500000 / COMMON as u64);
                    }
                }
            }
            if json!(first) != doc["pairs"][p]["first_common_bin_ns"] {
                return Err("different first threshold bin".into());
            }
            if p != 1 {
                continue;
            }
            let d = a[COMMON].c.distance(b[COMMON].c);
            store(&mut fields, "finest_pair", "end_com_m", d, k);
            for (idx, key) in array(&doc["keys"])?.iter().enumerate() {
                if let Some(t) = first[idx] {
                    let key = key.as_str().ok_or("missing threshold name")?;
                    store(
                        &mut fields,
                        "finest_pair",
                        &format!("first_{key}_ns"),
                        t as f64,
                        k,
                    );
                    *counts.entry(format!("{key}_crossed")).or_default() += 1;
                }
            }
            if max.is_none_or(|(v, _)| d > v) {
                max = Some((d, k));
            }
            if let Some(t) = first[1] {
                let when = if next_event.is_none_or(|at| (t as f64) < at) {
                    "before_next_material_impact"
                } else {
                    "at_or_after_next_material_impact"
                };
                *counts.entry(when.into()).or_default() += 1;
            }
            threshold = Some(first);
        }
        cases.push(json!({"case":k,"cut":cut,"next_material_impact_ns":next_event,"finest_first_bins_ns":threshold}));
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":cases.len()==50,"cases":cases,"counts":counts,"largest_finest_pair_end_m_case":max,"rows":rows_value(fields)}),
    )
}
