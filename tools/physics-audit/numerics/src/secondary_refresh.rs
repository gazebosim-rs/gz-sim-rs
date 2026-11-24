// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct alternative finite-box support-plane proofs and the resulting native responses.
use super::*;
#[path = "finite_box_normal.rs"]
mod geometry;
#[path = "secondary_refresh_plot.rs"]
mod plot;
use glam::DQuat;
const GRAPHS: [&str; 2] = ["retained", "cold"];
const METHODS: [&str; 6] = [
    "unmodified",
    "every",
    "first_unloaded",
    "first_stale",
    "while_stale",
    "first_travel",
];
const GRIDS: [u64; 3] = [8, 64, 4096];
const PASSES: [u64; 3] = [1, 8, 32];
fn witness(t: &Value, method: &str) -> Result<Option<f64>> {
    let r = &t["refresh"];
    let count = i(&r["requeries"])?;
    if ["first_unloaded", "first_stale", "first_travel"].contains(&method) {
        check(count <= 1, "multiple one-shot refreshes")?;
    }
    if !["first_stale", "while_stale", "first_travel"].contains(&method) {
        return Ok(None);
    }
    let stale = i(&r["certified_stale"])?;
    check(
        stale == count && stale <= i(&r["unloaded_tests"])?,
        "invalid stale native counts",
    )?;
    let w = &r["first_witness"];
    if stale == 0 {
        check(w.is_null(), "inactive witness has a plane")?;
        return Ok(None);
    }
    check(
        i(&w["unloaded_points"])? > 0,
        "no existing native points to refresh",
    )?;
    let p = vec3(&w["other_origin_in_first"])?;
    let q = DQuat::from_array(serde_json::from_value(
        w["other_rotation_in_first"].clone(),
    )?)
    .normalize();
    let a = vec3(&w["half_extents"][0])?;
    let b = vec3(&w["half_extents"][1])?;
    let vertex = |k: usize, h: DVec3| {
        h * DVec3::from_array(std::array::from_fn(|d| {
            if k & (1 << d) == 0 { -1. } else { 1. }
        }))
    };
    let gap = |axis: DVec3| -> Result<f64> {
        check(
            (axis.length() - 1.).abs() < 1e-6,
            "invalid native geometric direction",
        )?;
        let upper = (0..8)
            .map(|k| vertex(k, a).dot(axis))
            .fold(f64::NEG_INFINITY, f64::max);
        let lower = (0..8)
            .map(|k| (p + q * vertex(k, b)).dot(axis))
            .fold(f64::INFINITY, f64::min);
        Ok(lower - upper)
    };
    let old = gap(vec3(&w["cached_axis"])?)?;
    let new = gap(vec3(&w["stronger_axis"])?)?;
    let tolerance = n(&w["tolerance_m"])?;
    check(
        (old - n(&w["gap_m"][0])?).abs() < 2e-12
            && (new - n(&w["gap_m"][1])?).abs() < 2e-12
            && tolerance > 0.
            && new - old > tolerance - 2e-12,
        "recorded support plane failed independent convex-vertex proof",
    )?;
    Ok(Some(new - old))
}
fn source_row(
    src: &Value,
    natural: &Value,
    ordered: &Value,
    cache: &Value,
    travel: Option<&Value>,
) -> Result<Value> {
    for k in ["case", "slot", "held", "fixture", "incoming", "status"] {
        check(
            src[k] == natural[k] && src[k] == ordered[k],
            "different return or physical history",
        )?;
    }
    let mut r =
        json!({"case":src["case"],"slot":src["slot"],"held":src["held"],"status":src["status"]});
    if src["status"] != "measured" {
        r["reason"] = src["reason"].clone();
        return Ok(r);
    }
    check(
        src["cut_tick"] == natural["cut_tick"]
            && src["format"] == "secondary-unloaded-feature-witness-v2",
        "wrong checked plane format or actuator tick",
    )?;
    let mut data = BTreeMap::new();
    let mut proofs = 0;
    let methods = &METHODS[..if travel.is_some() { 6 } else { 5 }];
    let mut treatments = cases(&src["treatments"])?.iter().collect::<Vec<_>>();
    if let Some(exp) = travel {
        for key in [
            "case",
            "slot",
            "held",
            "fixture",
            "status",
            "incoming",
            "ordinary_onset",
            "cut_tick",
        ] {
            check(
                exp[key] == src[key],
                "travel threshold did not reproduce the common incoming state and ordinary onset",
            )?;
        }
        check(
            exp["format"] == "secondary-unloaded-surface-travel-v1",
            "wrong travel study format",
        )?;
        let mut duplicates = BTreeSet::new();
        let control = &src["ordinary_onset"][0]["first_positive"]["before"][2];
        let foot = &src["incoming"]["motion"]["links"][0];
        let g = vec3(&src["incoming"]["motion"]["gravity"])?.length();
        // Each local collider is centered on its own body COM in this authored fixture.
        let arms = [
            DVec3::new(0.18, 0.10, 0.06).length(),
            DVec3::new(0.33, 0.18, 0.06).length(),
        ];
        let speed = vec3(&foot["velocity"])?.distance(vec3(&control["velocity"])?)
            + vec3(&foot["angular_velocity"])?.length() * arms[0]
            + vec3(&control["angular_velocity"])?.length() * arms[1]
            + g * 0.0005;
        for t in cases(&exp["treatments"])? {
            let grid = i(&t["calls"])?;
            if t["refresh"]["method"] == "first_stale" {
                check(
                    treatments.iter().any(|x| *x == t)
                        && duplicates.insert((
                            t["graph"].as_str().ok_or("missing graph")?,
                            grid,
                            i(&t["passes"])?,
                        )),
                    "fixed-length call no longer matches archived treatment",
                )?;
            } else {
                check(
                    t["refresh"]["method"] == "first_travel" && GRIDS.contains(&grid),
                    "wrong travel treatment",
                )?;
                if let Some(tol) = t["refresh"]["first_witness"]["tolerance_m"].as_f64() {
                    let step = 500_000u64.div_ceil(grid) as f64 * 1e-9;
                    let expected =
                        (speed * step).max(128. * f64::EPSILON * (1. + arms[0] + arms[1]));
                    check(
                        (expected - tol).abs() < 2e-12,
                        "surface motion scale disagrees with independent incoming native state and geometry",
                    )?;
                }
                treatments.push(t);
            }
        }
        check(duplicates.len() == 18, "missing fixed-threshold duplicates")?;
    }
    for t in treatments {
        let key = (
            i(&t["calls"])?,
            i(&t["passes"])?,
            t["graph"].as_str().ok_or("graph")?,
            t["refresh"]["method"].as_str().ok_or("method")?,
        );
        check(
            GRIDS.contains(&key.0)
                && PASSES.contains(&key.1)
                && GRAPHS.contains(&key.2)
                && methods[1..].contains(&key.3),
            "invalid geometry variant",
        )?;
        proofs += usize::from(witness(t, key.3)?.is_some());
        check(
            data.insert(key, (response(t)?, t)).is_none(),
            "duplicate geometry simulation",
        )?;
    }
    check(
        data.len() == 18 * (methods.len() - 1),
        "incomplete geometry cohort case",
    )?;
    let ordinary_onset = cases(&src["ordinary_onset"])?;
    for t in cases(&natural["treatments"])? {
        let c = i(&t["calls"])?;
        let p = i(&t["passes"])?;
        let g = t["graph"].as_str().ok_or("missing natural graph")?;
        if !GRIDS.contains(&c) || !PASSES.contains(&p) {
            continue;
        }
        let a = response(t)?;
        let raw = if c == 8 && p == 8 {
            let own = ordinary_onset
                .iter()
                .find(|v| v["graph"] == g)
                .ok_or("missing ordinary material event")?;
            check(
                response(own)?.j == a.j
                    && own["first_ns"] == t["first_ns"]
                    && own["end_spin_rad_s"] == t["end_spin_rad_s"],
                "unmodified onset did not match archived natural source",
            )?;
            own
        } else {
            t
        };
        data.insert((c, p, g, "unmodified"), (a, raw));
    }
    check(
        data.len() == 18 * methods.len(),
        "missing unchanged geometry",
    )?;
    let check_fresh = cases(&cache["branches"])?
        .iter()
        .find(|v| v["graph"] == "fresh")
        .ok_or("missing original always-fresh experiment")?;
    let before = &check_fresh["first_public"];
    let control = data[&(4096, 1, "retained", "every")];
    check(
        (control.0.j - vec3(&before["normal_vector_ns"])? - vec3(&before["tangent_vector_ns"])?)
            .length()
            < 1e-9
            && control.1["first_ns"] == check_fresh["first_native_ns"],
        "old unconditional update changed",
    )?;
    let mut refs = cases(&ordered["treatments"])?
        .iter()
        .filter(|v| v["calls"] == 4096 && v["passes"] == 32)
        .map(|t| vec3(&t["world_impulse_ns"]))
        .collect::<Result<Vec<_>>>()?;
    check(refs.len() == 4, "missing four fine controls")?;
    refs.extend(GRAPHS.map(|g| data[&(4096, 32, g, "unmodified")].0.j));
    let fresh_refs = GRAPHS.map(|g| data[&(4096, 32, g, "every")].0.j);
    let mut cells = vec![];
    for c in GRIDS {
        for p in PASSES {
            for &method in methods {
                let s = GRAPHS.map(|g| data[&(c, p, g, method)]);
                let o = GRAPHS.map(|g| data[&(c, p, g, "unmodified")].0);
                let mut far = 0f64;
                let mut fresh_far = 0f64;
                let mut q = 0u64;
                let mut tests = 0u64;
                let mut angle = 0f64;
                let mut qualified = true;
                let mut per_graph = vec![];
                for (gi, (a, raw)) in s.iter().enumerate() {
                    for &target in &refs {
                        far = far.max(a.j.distance(target));
                    }
                    for &target in &fresh_refs {
                        fresh_far = fresh_far.max(a.j.distance(target));
                    }
                    q += raw["refresh"]["requeries"].as_u64().unwrap_or(0);
                    tests += raw["refresh"]["unloaded_tests"].as_u64().unwrap_or(0);
                    if c == 8 && p == 8 {
                        let e = geometry::event(&raw["first_positive"])?;
                        let pre = &e["before"];
                        let unique = n(&pre["near_normal_span_mrad"])? <= 1.
                            && n(&pre["signed_distance_m"])?.abs() >= 1e-9;
                        qualified &= unique;
                        angle = angle.max(n(&pre["native_angle_mrad"])?);
                        per_graph.push(json!({"graph":GRAPHS[gi],"time_ns":raw["first_ns"],"before":pre,"after":e["after"],"qualified":unique}));
                    }
                }
                let delta = [s[0].0.j.distance(o[0].j), s[1].0.j.distance(o[1].j)];
                cells.push(json!({"calls":c,"passes":p,"method":method,"history_spread_ns":s[0].0.j.distance(s[1].0.j),"cold_minus_warm_normal_ns":s[1].0.jz-s[0].0.jz,
      "max_distance_to_ordinary_six_ns":far,"max_distance_to_fully_fresh_two_ns":fresh_far,"max_to_all_eight_ns":far.max(fresh_far),"native_queries":q,"unloaded_tests":tests,
      "warm_change_ns":delta[0],"cold_change_ns":delta[1],"warm_change_from_first_stale_ns":s[0].0.j.distance(data[&(c,p,"retained","first_stale")].0.j),"mean_query_count":q as f64*0.5,"mean_unloaded_tests":tests as f64*0.5,
      "last_com_change_um":s[0].0.com.distance(o[0].com).max(s[1].0.com.distance(o[1].com))*1e6,
      "last_spin_change_rad_s":s[0].0.spin.distance(o[0].spin).max(s[1].0.spin.distance(o[1].spin)),
      "linear_balance_defect_ns":s[0].0.dp.max(s[1].0.dp),"max_normal_angle_mrad":(c==8&&p==8&&qualified).then_some(angle),"paid_geometry":per_graph}));
            }
        }
    }
    r["first_witnesses_independently_rebuilt"] = json!(proofs);
    r["cells"] = json!(cells);
    Ok(r)
}
pub(super) fn run(dir: &Path, out: &Path, travel: bool) -> Result<()> {
    fs::create_dir_all(out)?;
    let studies = dir.parent().ok_or("missing studies")?;
    let mut rows = vec![];
    let mut slots = BTreeSet::new();
    for entry in fs::read_dir(dir.join("cases"))? {
        let file = entry?.path();
        if file.extension().and_then(|s| s.to_str()) != Some("json") {
            continue;
        }
        let name = file.file_name().ok_or("missing name")?;
        let sibling = |s: &str| studies.join(s).join("cases").join(name);
        let nfile = sibling("multifidelity-secondary-natural");
        if !nfile.exists() {
            continue;
        }
        let exp = travel.then(|| read(&file)).transpose()?;
        let row = source_row(
            &read(&if travel {
                sibling("multifidelity-secondary-return-features")
            } else {
                file.clone()
            })?,
            &read(&nfile)?,
            &read(&sibling("multifidelity-secondary-relaxation"))?,
            &read(&sibling("multifidelity-secondary-cache-field"))?,
            exp.as_ref(),
        )?;
        check(slots.insert(i(&row["slot"])?), "duplicate source")?;
        rows.push(row);
    }
    rows.sort_by_key(|r| r["slot"].as_u64());
    let mut result = json!({"joined":rows.len(),"pending_slots":(0..50).filter(|s|!slots.contains(s)).collect::<Vec<_>>(),"first_witnesses_rebuilt":rows.iter().filter_map(|r|r["first_witnesses_independently_rebuilt"].as_u64()).sum::<u64>()});
    let methods = &METHODS[..if travel { 6 } else { 5 }];
    for (held, label) in [(false, "calibration"), (true, "holdout")] {
        let cohort = rows
            .iter()
            .filter(|r| r["held"] == held && r["status"] == "measured")
            .collect::<Vec<_>>();
        let mut fields = vec![];
        for (ci, c) in GRIDS.into_iter().enumerate() {
            for (pi, p) in PASSES.into_iter().enumerate() {
                for (mi, m) in methods.iter().enumerate() {
                    let k = (ci * PASSES.len() + pi) * methods.len() + mi;
                    let mut cell = json!({"calls":c,"passes":p,"method":m});
                    for key in [
                        "history_spread_ns",
                        "warm_change_ns",
                        "cold_change_ns",
                        "max_distance_to_ordinary_six_ns",
                        "max_distance_to_fully_fresh_two_ns",
                        "max_to_all_eight_ns",
                        "last_com_change_um",
                        "last_spin_change_rad_s",
                        "linear_balance_defect_ns",
                        "max_normal_angle_mrad",
                    ] {
                        cell[key] = distribution(
                            cohort
                                .iter()
                                .filter_map(|r| r["cells"][k][key].as_f64())
                                .collect(),
                        );
                    }
                    for key in ["mean_query_count", "mean_unloaded_tests"] {
                        let total: f64 = cohort
                            .iter()
                            .map(|r| r["cells"][k][key].as_f64().unwrap())
                            .sum();
                        cell[key] =
                            json!((!cohort.is_empty()).then(|| total / cohort.len() as f64));
                    }
                    fields.push(cell);
                }
            }
        }
        result[label] = json!({"measured":cohort.len(),"cells":fields});
    }
    let k = methods.len();
    let cal = &result["calibration"]["cells"];
    let base = &cal[k];
    if result["joined"] == 50 && !travel {
        let p95 = |a: &Value, key: &str| n(&a[key]["p95"]);
        // Intersect geometric validity for each comparison so classification cannot win by censoring.
        let mut eligible = vec![];
        for (i, m) in methods[1..].iter().enumerate() {
            let del = k + i + 1;
            let cell = &cal[del];
            let sample = rows
                .iter()
                .filter(|r| {
                    r["held"] == false
                        && r["status"] == "measured"
                        && r["cells"][k]["max_normal_angle_mrad"].is_number()
                        && r["cells"][del]["max_normal_angle_mrad"].is_number()
                })
                .collect::<Vec<_>>();
            if sample.is_empty() {
                continue;
            }
            let baseline = distribution(
                sample
                    .iter()
                    .map(|r| r["cells"][k]["max_normal_angle_mrad"].as_f64().unwrap())
                    .collect(),
            );
            let trial = distribution(
                sample
                    .iter()
                    .map(|r| r["cells"][del]["max_normal_angle_mrad"].as_f64().unwrap())
                    .collect(),
            );
            if n(&trial["p95"])? < n(&baseline["p95"])? - 1e-7
                && p95(cell, "history_spread_ns")? < p95(base, "history_spread_ns")? - 1e-8
                && p95(cell, "max_to_all_eight_ns")? <= p95(base, "max_to_all_eight_ns")? + 0.001
            {
                eligible.push((
                    *m,
                    n(&cell["mean_query_count"])?,
                    n(&cell["mean_unloaded_tests"])?,
                ));
            }
        }
        eligible.sort_by(|a, b| {
            a.1.total_cmp(&b.1)
                .then_with(|| a.2.total_cmp(&b.2))
                .then_with(|| a.0.cmp(b.0))
        });
        result["calibration_selection"] = json!(eligible.first().map(|x| x.0));
    }
    let chosen = k + if travel { 5 } else { 3 };
    let metric = if travel {
        "warm_change_from_first_stale_ns"
    } else {
        "warm_change_ns"
    };
    let worst = rows
        .iter()
        .filter(|r| r["held"] == true && r["status"] == "measured")
        .max_by(|a, b| {
            a["cells"][chosen][metric]
                .as_f64()
                .unwrap()
                .total_cmp(&b["cells"][chosen][metric].as_f64().unwrap())
                .then_with(|| b["case"].as_u64().cmp(&a["case"].as_u64()))
        });
    result["max_heldout_warm_witness_change"] = worst
        .map(|r| json!({"case":r["case"],"change_ns":r["cells"][chosen][metric]}))
        .unwrap_or(Value::Null);
    result["cases"] = json!(rows);
    fs::write(
        out.join("readback.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    plot::field(out, &result)?;
    Ok(())
}

pub(super) fn registered(dir: &Path, out: &Path) -> Result<()> {
    let base = dir
        .parent()
        .ok_or("missing studies")?
        .join("multifidelity-secondary-return-features/cases");
    let mut slots = BTreeSet::new();
    let mut counts = BTreeMap::<(bool, u64, u64, String), (u64, u64, u64)>::new();
    let mut measured = 0;
    for e in fs::read_dir(dir.join("cases"))? {
        let e = e?.path();
        if !e.extension().is_some_and(|x| x == "json") {
            continue;
        }
        let src = read(&e)?;
        let original = read(&base.join(e.file_name().unwrap()))?;
        for k in ["slot", "case", "held", "cut_tick"] {
            check(src[k] == original[k], "registered control changed source")?;
        }
        check(slots.insert(i(&src["slot"])?), "duplicate registered donor")?;
        let mut keys = BTreeSet::new();
        for r in cases(&src["matches"])? {
            let calls = i(&r["calls"])?;
            let passes = i(&r["passes"])?;
            let graph = r["graph"].as_str().ok_or("missing graph")?;
            check(
                GRIDS.contains(&calls)
                    && PASSES.contains(&passes)
                    && GRAPHS.contains(&graph)
                    && keys.insert((calls, passes, graph)),
                "duplicate or invalid registered coordinate",
            )?;
            let v = cases(&original["treatments"])?
                .iter()
                .find(|t| {
                    t["calls"] == calls
                        && t["passes"] == passes
                        && t["graph"] == graph
                        && t["refresh"]["method"] == "first_stale"
                })
                .ok_or("missing original candidate")?;
            for k in ["first_ns", "normal_ns", "world_impulse_ns", "refresh"] {
                check(
                    r[k] == v[k],
                    "registered scalar, vector, onset or geometry proof changed",
                )?;
            }
            witness(r, "first_stale")?;
            let t = counts
                .entry((src["held"].as_bool().unwrap(), calls, passes, graph.into()))
                .or_default();
            t.0 += 1;
            t.1 += i(&r["refresh"]["requeries"])?;
            t.2 += i(&r["refresh"]["unloaded_tests"])?;
            measured += 1;
        }
        check(keys.len() == 18, "incomplete registered geometry field")?;
    }
    let a = read(&dir.join("authored/authored-cases.json"))?;
    let mut ids = BTreeSet::new();
    let mut events = 0;
    let mut tests = 0;
    let mut queries = 0;
    let mut evidence = vec![];
    for c in cases(&a["cases"])? {
        check(
            ids.insert(i(&c["source"])?),
            "duplicate authored geometry fixture",
        )?;
        let mut prev = 0;
        for e in cases(&c["events"])? {
            let tick = i(&e["tick"])?;
            check(tick > prev && tick <= 660, "invalid authored clock")?;
            prev = tick;
            let t = json!({"refresh":{"requeries":e["queries"],"certified_stale":e["queries"],"unloaded_tests":e["tests"],"first_witness":e["first_witness"]}});
            if let Some(delta) = witness(&t, "first_stale")? {
                evidence.push(json!({"case":c["source"],"tick":tick,"margin_gain_m":delta,"normal_ns":e["normal_ns"]}));
            }
            events += 1;
            tests += i(&e["tests"])?;
            queries += i(&e["queries"])?;
        }
    }
    check(ids.len() == 144, "missing authored worlds")?;
    let cells=counts.into_iter().map(|((held,calls,passes,graph),(n,queries,checks))|json!({"held":held,"calls":calls,"passes":passes,"graph":graph,"cases":n,"queries":queries,"unloaded_checks":checks})).collect::<Vec<_>>();
    fs::create_dir_all(out)?;
    fs::write(
        out.join("registered.json"),
        serde_json::to_vec_pretty(
            &json!({"matched_native_intervals":measured,"pending_slots":(0..50).filter(|s|!slots.contains(s)).collect::<Vec<_>>(),"cells":cells,"authored":{"worlds":ids.len(),"public_updates":ids.len()*660,"nominated":events,"unloaded_checks":tests,"requeries":queries,"positive_proofs":evidence}}),
        )?,
    )?;
    Ok(())
}
