// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Natural solver order and uninstrumented return costs for the same physical donors.
use super::*;
const GRAPHS: [&str; 2] = ["retained", "cold"];
fn field_case(src: &Value, ordered: &Value, original: &Value) -> Result<Value> {
    let verified = analyze(ordered, original)?;
    for key in [
        "case", "slot", "held", "fixture", "status", "incoming", "cut_tick",
    ] {
        check(
            src[key] == ordered[key],
            "natural and ordered donations differ",
        )?;
    }
    check(
        src["format"] == "secondary-native-unmodified-relaxation-v1",
        "incorrect ordinary study",
    )?;
    let mut out =
        json!({"case":src["case"],"slot":src["slot"],"held":src["held"],"status":src["status"]});
    if src["status"] != "measured" {
        out["reason"] = src["reason"].clone();
        return Ok(out);
    }
    let mut all = BTreeMap::new();
    for t in cases(&src["treatments"])? {
        let calls = i(&t["calls"])?;
        let passes = i(&t["passes"])?;
        let graph = t["graph"].as_str().ok_or("missing original graph")?;
        check(
            N.contains(&calls)
                && R.contains(&passes)
                && GRAPHS.contains(&graph)
                && t["visited_rows"].is_null()
                && t["reordered_rows"].is_null(),
            "unmodified treatment has invalid grid or a hook",
        )?;
        let r = response(t)?;
        if calls == 4096 && passes == 1 {
            let archived = cases(&original["branches"])?
                .iter()
                .find(|x| x["graph"] == graph)
                .ok_or("no unmodified control")?;
            let p = &archived["first_public"];
            check(
                (r.j - vec3(&p["normal_vector_ns"])? - vec3(&p["tangent_vector_ns"])?).length()
                    < 1e-9
                    && (r.h - vec3(&p["normal_moment_nms"])? - vec3(&p["tangent_moment_nms"])?)
                        .length()
                        < 1e-9
                    && t["first_ns"] == archived["first_native_ns"],
                "unmodified fine-grid control did not reproduce",
            )?;
        }
        check(
            all.insert((calls, passes, graph), r).is_none(),
            "duplicate natural treatment",
        )?;
    }
    check(all.len() == 48, "incomplete unmodified factorial")?;
    let center = vec3(&verified["empirical_center_ns"])?;
    let mut fine = cases(&ordered["treatments"])?
        .iter()
        .filter(|t| t["calls"] == 4096 && t["passes"] == 32)
        .map(|t| vec3(&t["world_impulse_ns"]))
        .collect::<Result<Vec<_>>>()?;
    fine.extend(GRAPHS.map(|g| all[&(4096, 32, g)].j));
    let mut rows = vec![];
    for calls in N {
        for passes in R {
            let a = GRAPHS.map(|g| all[&(calls, passes, g)]);
            let corresponding = GRAPHS.map(|g| all[&(4096, 32, g)]);
            let dist = [
                a[0].j.distance(corresponding[0].j),
                a[1].j.distance(corresponding[1].j),
            ];
            let mut largest = 0f64;
            let mut shrink = 0;
            let mut grow = 0;
            for (gi, g) in GRAPHS.iter().enumerate() {
                let base = all[&(calls, 1, *g)].j;
                let changes = fine
                    .iter()
                    .map(|t| a[gi].j.distance(*t) - base.distance(*t))
                    .collect::<Vec<_>>();
                shrink += usize::from(changes.iter().all(|x| *x < -1e-7));
                grow += usize::from(changes.iter().all(|x| *x > 1e-7));
                for t in &fine {
                    largest = largest.max(a[gi].j.distance(*t));
                }
            }
            rows.push(json!({"calls":calls,"passes":passes,"graph_spread_ns":a[0].j.distance(a[1].j),
      "max_matched_fine_ns":dist[0].max(dist[1]),"warm_matched_fine_ns":dist[0],"cold_matched_fine_ns":dist[1],
      "max_distance_to_six_fine_ns":largest,"max_distance_to_ordered_center_ns":a[0].j.distance(center).max(a[1].j.distance(center)),
      "graphs_closer_to_all_six":shrink,"graphs_farther_from_all_six":grow,
      "max_com_to_matched_fine_um":a[0].com.distance(corresponding[0].com).max(a[1].com.distance(corresponding[1].com))*1e6,
      "max_spin_to_matched_fine_rad_s":a[0].spin.distance(corresponding[0].spin).max(a[1].spin.distance(corresponding[1].spin)),
      "max_linear_defect_ns":a[0].dp.max(a[1].dp),"max_angular_defect_nms":a[0].dh.max(a[1].dh)}));
        }
    }
    out["cells"] = json!(rows);
    Ok(out)
}
pub(super) fn run(dir: &Path, out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    let studies = dir.parent().ok_or("missing sibling archives")?;
    let mut rows = Vec::new();
    let mut slots = BTreeSet::new();
    for entry in fs::read_dir(dir.join("cases"))? {
        let path = entry?.path();
        if path.extension().and_then(|s| s.to_str()) != Some("json") {
            continue;
        }
        let name = path.file_name().ok_or("missing case file name")?;
        let original = read(
            &studies
                .join("multifidelity-secondary-order-field/cases")
                .join(name),
        )?;
        let ordered = studies
            .join("multifidelity-secondary-relaxation/cases")
            .join(name);
        if !ordered.exists() {
            continue;
        }
        let src = read(&path)?;
        let row = field_case(&src, &read(&ordered)?, &original)?;
        check(
            slots.insert(i(&row["slot"])?),
            "duplicate original physical cut",
        )?;
        rows.push(row);
    }
    rows.sort_by_key(|r| r["slot"].as_u64());
    let mut result = json!({"joined":rows.len(),"pending_slots":(0..50).filter(|s|!slots.contains(s)).collect::<Vec<_>>()});
    for (held, split) in [(false, "calibration"), (true, "holdout")] {
        let group = rows
            .iter()
            .filter(|r| r["held"] == held && r["status"] == "measured")
            .collect::<Vec<_>>();
        let mut cells = vec![];
        for (ni, calls) in N.into_iter().enumerate() {
            for (pi, passes) in R.into_iter().enumerate() {
                let k = ni * R.len() + pi;
                let mut x = json!({"calls":calls,"passes":passes});
                for m in [
                    "graph_spread_ns",
                    "max_matched_fine_ns",
                    "warm_matched_fine_ns",
                    "cold_matched_fine_ns",
                    "max_distance_to_six_fine_ns",
                    "max_distance_to_ordered_center_ns",
                    "max_com_to_matched_fine_um",
                    "max_spin_to_matched_fine_rad_s",
                    "max_linear_defect_ns",
                    "max_angular_defect_nms",
                ] {
                    x[m] = distribution(
                        group
                            .iter()
                            .map(|r| n(&r["cells"][k][m]))
                            .collect::<Result<Vec<_>>>()?,
                    );
                }
                for m in ["graphs_closer_to_all_six", "graphs_farther_from_all_six"] {
                    x[m] = json!(
                        group
                            .iter()
                            .map(|r| i(&r["cells"][k][m]))
                            .collect::<Result<Vec<_>>>()?
                            .into_iter()
                            .sum::<u64>()
                    );
                }
                cells.push(x);
            }
        }
        result[split] = json!({"measured":group.len(),"cells":cells});
    }
    let worst = rows
        .iter()
        .filter(|r| r["held"] == true && r["status"] == "measured")
        .max_by(|a, b| {
            a["cells"][5]["warm_matched_fine_ns"]
                .as_f64()
                .unwrap()
                .total_cmp(&b["cells"][5]["warm_matched_fine_ns"].as_f64().unwrap())
                .then_with(|| b["case"].as_u64().cmp(&a["case"].as_u64()))
        });
    result["largest_heldout_retained_8x32_change"] = worst
        .map(|r| json!({"case":r["case"],"vector_ns":r["cells"][5]["warm_matched_fine_ns"]}))
        .unwrap_or(Value::Null);
    draw(
        out,
        "ordinary-order.svg",
        "Secondary support reactions using the original contact order",
        &format!(
            "{} / 50 joined native fixtures  ·  held-out p95 (N·s)  ·  both actual histories, four grids and six relaxation budgets",
            rows.len()
        ),
        &result,
        [
            ("graph_spread_ns", "Rebuilt versus retained contact graph"),
            (
                "max_matched_fine_ns",
                "Largest change from each graph's own 4,096-call / 32-pass control",
            ),
        ],
    )?;
    result["cases"] = json!(rows);
    fs::write(
        out.join("readback.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    Ok(())
}
pub(super) fn cost(
    dir: &Path,
    out: &Path,
    features: bool,
    travel: bool,
    material: bool,
    events: bool,
) -> Result<()> {
    fs::create_dir_all(out)?;
    let mut result = json!({});
    for rep in [1, 2] {
        let mut data = BTreeMap::<(bool, u64, u64, String), Vec<f64>>::new();
        let mut cpu_data = BTreeMap::<(bool, u64, u64, String), Vec<f64>>::new();
        let mut slots = BTreeSet::new();
        let r = dir.join(format!("repeat-{rep}"));
        if !r.exists() {
            continue;
        }
        for file in fs::read_dir(r)? {
            let file = file?.path();
            if file.extension().and_then(|s| s.to_str()) != Some("json") {
                continue;
            }
            let src = read(&file)?;
            let slot = i(&src["slot"])?;
            let archive = read(
                &dir.parent()
                    .unwrap()
                    .join("multifidelity-secondary-order-field/cases")
                    .join(file.file_name().unwrap()),
            )?;
            check(
                src["repeat"] == rep
                    && slot % 6 == 1
                    && slot <= 49
                    && slots.insert(slot)
                    && archive["slot"] == slot
                    && archive["held"] == true
                    && archive["incoming"]["cut_tick"] == src["cut_tick"],
                "wrong timing source",
            )?;
            let mut keys = BTreeSet::new();
            let studies = dir.parent().ok_or("missing studies")?;
            let feature_source = studies
                .join("multifidelity-secondary-return-features/cases")
                .join(file.file_name().unwrap());
            let travel_source = studies
                .join("multifidelity-secondary-travel-features/cases")
                .join(file.file_name().unwrap());
            let travel_archive = (travel && travel_source.exists())
                .then(|| read(&travel_source))
                .transpose()?;
            let natural_source = studies
                .join("multifidelity-secondary-natural/cases")
                .join(file.file_name().unwrap());
            let reference = if features && !material && !events && feature_source.exists() {
                Some((read(&feature_source)?, read(&natural_source)?))
            } else {
                None
            };
            let mut controls = BTreeMap::new();
            for cell in cases(&src["cells"])? {
                let cold = cell["rebuilt"].as_bool().ok_or("invalid history")?;
                let calls = i(&cell["calls"])?;
                let passes = i(&cell["passes"])?;
                let hook = cell["hook"].as_str().ok_or("no treatment")?.to_string();
                let key = (cold, calls, passes, hook.clone());
                check(
                    [8, 64].contains(&calls)
                        && if travel || material || events {
                            passes == 8
                        } else if features {
                            [1, 8, 32].contains(&passes)
                        } else {
                            [1, 8, 16, 32].contains(&passes)
                        }
                        && if events {
                            ["ordinary", "subdivided", "directed", "retimed_directed"]
                                .contains(&hook.as_str())
                        } else if material {
                            [
                                "ordinary",
                                "point",
                                "certified_point",
                                "launch",
                                "certified_launch",
                            ]
                            .contains(&hook.as_str())
                        } else if travel {
                            ["ordinary", "first_unloaded", "first_stale", "first_travel"]
                                .contains(&hook.as_str())
                        } else if features {
                            [
                                "ordinary",
                                "every",
                                "first_unloaded",
                                "first_stale",
                                "while_stale",
                            ]
                            .contains(&hook.as_str())
                        } else {
                            ["ordinary", "no_op", "ordered"].contains(&hook.as_str())
                        }
                        && keys.insert(key.clone()),
                    "invalid native timing coordinate",
                )?;
                let times = cases(&cell["wall_ns"])?
                    .iter()
                    .map(i)
                    .collect::<Result<Vec<_>>>()?;
                check(
                    times.len() == 16 && times.iter().all(|ns| *ns > 0),
                    "missing timing repetitions",
                )?;
                if features {
                    let cpu = cases(&cell["thread_cpu_ns"])?
                        .iter()
                        .map(i)
                        .collect::<Result<Vec<_>>>()?;
                    check(
                        cpu.len() == 16 && cpu.iter().all(|x| *x > 0),
                        "missing native thread CPU clocks",
                    )?;
                    cpu_data
                        .entry(key.clone())
                        .or_default()
                        .extend(cpu.into_iter().map(|v| v as f64 * 1e-3));
                    if material || events {
                        let certified = hook.starts_with("certified_");
                        let folder = if events {
                            "multifidelity-secondary-event-bounds"
                        } else if certified {
                            "multifidelity-secondary-face-policy"
                        } else {
                            "multifidelity-secondary-face-controls"
                        };
                        let source = read(
                            &studies
                                .join(folder)
                                .join("cases")
                                .join(file.file_name().unwrap()),
                        )?;
                        let mode = if hook == "ordinary" {
                            "native"
                        } else {
                            hook.strip_prefix("certified_").unwrap_or(&hook)
                        };
                        let graph = if cold { "cold" } else { "retained" };
                        let matching = cases(&source["rows"])?
                            .iter()
                            .find(|r| {
                                r["graph"] == graph && r["calls"] == calls && r["mode"] == mode
                            })
                            .ok_or("missing native policy cost control")?;
                        check(
                            (n(&cell["normal_ns"])? - n(&matching["normal_ns"])?).abs() < 1e-10,
                            "timed material response changed",
                        )?;
                    }
                    if let Some((geom, native)) = &reference {
                        let from = if hook == "ordinary" {
                            native
                        } else if hook == "first_travel" {
                            travel_archive.as_ref().ok_or("missing travel source")?
                        } else {
                            geom
                        };
                        let actual = cases(&from["treatments"])?
                            .iter()
                            .find(|t| {
                                t["calls"] == calls
                                    && t["passes"] == passes
                                    && t["graph"] == if cold { "cold" } else { "retained" }
                                    && (hook == "ordinary" || t["refresh"]["method"] == hook)
                            })
                            .ok_or("missing measured cost treatment")?;
                        check(
                            (n(&cell["normal_ns"])? - n(&actual["normal_ns"])?).abs() < 1e-10,
                            "timed reaction did not reproduce physical source",
                        )?;
                        if hook != "ordinary" {
                            check(
                                cell["requeries"] == actual["refresh"]["requeries"],
                                "timed query count changed",
                            )?;
                        }
                    }
                } else if hook == "ordinary" {
                    check(
                        cell["visited_rows"].is_null(),
                        "ordinary unexpectedly hooked",
                    )?;
                } else {
                    check(
                        i(&cell["changed_rows"])? <= i(&cell["visited_rows"])?,
                        "invalid actual callback census",
                    )?;
                }
                controls.insert(key.clone(), n(&cell["normal_ns"])?);
                data.entry(key)
                    .or_default()
                    .extend(times.into_iter().map(|v| v as f64 * 1e-3));
            }
            check(
                keys.len()
                    == if events {
                        16
                    } else if material {
                        20
                    } else if travel {
                        16
                    } else if features {
                        60
                    } else {
                        48
                    },
                "incomplete timing factorial",
            )?;
            for (k, v) in &controls {
                if !features && k.3 == "ordinary" {
                    check(
                        controls[&(k.0, k.1, k.2, "no_op".into())] == *v,
                        "no-op did not reproduce unmodified reaction",
                    )?;
                }
            }
        }
        let rows=data.into_iter().map(|(key,v)|{
            let cpu=cpu_data.remove(&key);
            let (rebuilt,calls,passes,hook)=key;
            json!({"rebuilt":rebuilt,"calls":calls,"passes":passes,"hook":hook,"public_us":distribution(v),"thread_cpu_us":cpu.map(distribution)})
        }).collect::<Vec<_>>();
        result[format!("repeat_{rep}")] = json!({"complete_donors":slots.len(),"missing_slots":(1u64..50).step_by(6).filter(|s|!slots.contains(s)).collect::<Vec<_>>(),"cells":rows});
    }
    fs::write(out.join("cost.json"), serde_json::to_vec_pretty(&result)?)?;
    Ok(())
}
