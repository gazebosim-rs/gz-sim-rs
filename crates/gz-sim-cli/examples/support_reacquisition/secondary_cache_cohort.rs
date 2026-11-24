// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Crossover of retained, rebuilt and freshly queried geometry at the actual second landing.
use super::*;
fn version(order: bool) -> &'static str {
    if order {
        "second-landing-native-row-order-cohort-v1"
    } else {
        "second-landing-native-cache-cohort-v1"
    }
}
pub(super) fn coordinate(slot: usize) -> Result<(usize, Fixture, bool)> {
    if slot >= 50 {
        return Err("invalid second-landing field coordinate".into());
    }
    let held = slot % 2 == 1;
    let band = slot / 10;
    let drive = (slot / 2) % 5;
    let offset = 170 + 25 * band + 5 * usize::from(held);
    let k = (0..COUNT)
        .find(|&k| {
            let f = fixture(k);
            (f.offset_m * 1000.).round() as usize == offset
                && (f.speed_scale * 100.).round() as usize == 76 + 12 * drive
                && f.switched == ((band + drive) % 2 == 1)
        })
        .ok_or("missing fixture coordinate")?;
    Ok((k, fixture(k), held))
}
fn compact(record: &Value) -> Result<Value> {
    let first = &record["public"][0];
    let raw = first["first_positive"]
        .as_object()
        .ok_or("second landing absent on treatment first tick");
    // A changed numerical treatment can delay the first public reaction. Keep the
    // zero receipt and its missing point signature if that occurs.
    let rows = raw.ok().and_then(|r| r["solver_rows"].as_array());
    let signature=rows.map(|p|p.iter().map(|r|{
       let local=vec3(&r["local_foot"])?;
       Ok(json!({"quadrant":2*usize::from(local.x>0.)+usize::from(local.y>0.),"native_foot_first":r["foot_is_first"],"local_m":r["local_foot"]}))
   }).collect::<Result<Vec<_>>>()).transpose()?;
    let max = |field: &str| -> Result<f64> {
        record["public"]
            .as_array()
            .ok_or("missing dynamics")?
            .iter()
            .try_fold(0.0_f64, |v, x| Ok(v.max(scalar(&x[field])?)))
    };
    Ok(
        json!({"graph":record["graph"],"grid":record["grid"],"initial_rows":record["initial_rows"],"first_paid_rows":signature,
       "row_hook":first["row_hook"],"first_native_ns":first["first_positive"]["elapsed_ns"],"first_public":first["totals"],
       "max_linear_balance_ns":max("dp_residual_ns")?,"max_angular_balance_nms":max("dh_residual_nms")?,
       "max_native_linear_balance_ns":max("max_native_dp_ns")?,"max_native_angular_balance_nms":max("max_native_dh_nms")?}),
    )
}
fn probe(
    dir: &Path,
    slot: usize,
    index: usize,
    f: Fixture,
    held: bool,
    order: bool,
) -> Result<Value> {
    let root = dir
        .parent()
        .ok_or("output needs sibling native fixture archives")?;
    let source = load(&root.join(format!(
        "multifidelity-finite-propagation/cases/case-{index:03}.json"
    )))?;
    if source["case"] != index || source["result"]["fixture"] != serde_json::to_value(f)? {
        return Err("wrong source fixture".into());
    }
    let (cuts, _) = survey::cuts(f)?;
    let mut result = json!({"format":version(order),"slot":slot,"case":index,"held":held,"fixture":f,"status":"censored"});
    let Some(cut) = cuts[0] else {
        result["reason"] = json!("no primary cut");
        return Ok(result);
    };
    if snapshot(&cut) != source["result"]["cut"] {
        return Err("source primary state changed".into());
    }
    result["primary_tick"] = json!(cut.tick);
    let mut rig = restart(f, ADAPTIVE, &cut)?;
    let mut speed = rig.last_speed;
    let lane = &mut rig.case.lane;
    let ids = [lane.foot, lane.upper, lane.pad];
    let [foot, _, pad] = ids;
    let mut res = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let live = res
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("missing local engine")?;
    let start = sweep::DEPART_US[1] / STEP_US;
    let stop = sweep::DEPART_US[2] / STEP_US;
    let mut released = false;
    let mut last_loaded = None;
    let mut previous = None;
    let mut impacts = vec![];
    for tick in cut.tick + 1..=stop {
        let next = command(f, tick - 1);
        let state = propagate_one(
            live,
            ids,
            tick,
            b(4096, 1),
            (next != speed).then_some(next),
            None,
            &mut impacts,
        )?;
        speed = next;
        if tick > start && released && state.normal > 0. {
            result["second_tick"] = json!(tick);
            result["last_prior_load_tick"] = json!(last_loaded);
            result["release_observed"] = json!(true);
            let Some((incoming, copied)) = previous.take() else {
                result["reason"] =
                    json!("preceding public state was not an unloaded supported rising-pad cut");
                return Ok(result);
            };
            let incoming: Tick = incoming;
            if incoming.tick + 1 != tick {
                return Err("nonconsecutive second landing".into());
            }
            let parent: MultiFidelityBackend = match copied {
                Ok(parent) => parent,
                Err(reason) => {
                    result["reason"] = json!(format!("unqualified native restart: {reason}"));
                    return Ok(result);
                }
            };
            let snapshot = parent.airborne_restart_state(foot, pad)?;
            result["incoming"] = json!({"motion":motion(&snapshot),"cache":cache(&snapshot),"gap_m":incoming.gap,"cut_tick":incoming.tick,"last_unloaded_intervals":last_loaded.map(|last:u64|incoming.tick-last)});
            let future = continuation(live, ids, state, incoming.tick + 10, 4096, f)?;
            let held_end = 10usize;
            let mut observed = vec![incoming];
            observed.extend(future);
            if observed.len() != held_end + 1 {
                return Err("short uninterrupted control".into());
            }
            if let Ok(saved) = load(&root.join(format!(
                "multifidelity-primary-propagation/cases/case-{index:03}.json"
            ))) {
                let paths = saved["result"]["paths"]
                    .as_array()
                    .ok_or("archived source has no paths")?;
                let there = paths
                    .iter()
                    .find(|r| r["profile"] == "continuous4096r1")
                    .ok_or("missing 4096 source")?;
                let got = (tick - 1) * CLOCK_NS
                    + state
                        .first
                        .ok_or("positive native reaction missing its clock")?
                        .as_nanos() as u64;
                if there["outcomes"]["strokes"][1]["first_native_end_ns"] != got {
                    return Err("second onset disagrees with independent long run".into());
                }
                result["matched_long_return_ns"] = json!(got);
            }
            let folder = dir.join(format!("case-{index:03}"));
            fs::create_dir_all(folder.join("traces"))?;
            let mut branches = vec![];
            let graphs: &[Graph] = if order {
                &[
                    Graph::Retained,
                    Graph::Cold,
                    Graph::WarmNoop,
                    Graph::ColdNoop,
                    Graph::WarmSorted,
                    Graph::ColdSorted,
                    Graph::WarmExchanged,
                    Graph::ColdExchanged,
                ]
            } else {
                &[Graph::Retained, Graph::Cold, Graph::Fresh]
            };
            for &graph in graphs {
                let fork = parent.copy_airborne_landing(foot, pad, graph.cold())?;
                let row = branch((&folder, None), ids, incoming, fork, (4096, 4096), graph, f)?;
                let label = format!("{}4096", graph.name());
                let trace = load(&folder.join(format!("traces/{label}.json")))?;
                if matches!(graph, Graph::Retained) {
                    for (i, t) in observed.iter().enumerate() {
                        verify_raw(&trace["frames"][i], &t.as_json())?;
                    }
                    result["uninterrupted_samples_checked"] = json!(observed.len());
                }
                let mut compact = compact(&row)?;
                compact["end"] = trace["frames"][held_end].clone();
                branches.push(compact);
            }
            let entries = if order {
                [
                    ("warm_noop4096", "Retained geometry, original order"),
                    ("warm_sorted4096", "Retained geometry, ordered rows"),
                    ("cold_sorted4096", "Rebuilt geometry, same ordered rows"),
                ]
            } else {
                [
                    ("retained4096", "Retained sole geometry"),
                    ("cold4096", "Rebuilt collision graph"),
                    ("fresh4096", "Fresh sole features on first tick"),
                ]
            };
            let lanes=entries.map(|(label,name)|json!({"label":label,"name":name,"file":format!("traces/{label}.json")}));
            write_json(
                &folder.join("comparison.json"),
                &json!({"lanes":lanes,"case":index,"title":if order {"Controlled solver-row order at the second landing"} else {"Same incoming secondary landing with three contact geometries"},
         "capture_ticks":{"second_impact":tick,"after_five_ms":incoming.tick+10}}),
            )?;
            result["status"] = json!("measured");
            result["branches"] = json!(branches);
            return Ok(result);
        }
        if state.normal > 0. {
            last_loaded = Some(tick);
        }
        if tick > start && state.normal == 0. && state.gap.is_some_and(|g| g > 0.001) {
            released = true;
        }
        previous = if released
            && state.normal == 0.
            && state.extra == 0.
            && state.tangent == DVec3::ZERO
            && state.gap.is_some_and(|v| v > 0.)
            && speed > 0.
        {
            Some((state, live.copy_airborne_landing(foot, pad, false)))
        } else {
            None
        };
    }
    result["reason"] = json!(if released {
        "no second positive reaction on this stroke"
    } else {
        "no independently unloaded 1 mm gap on second stroke"
    });
    Ok(result)
}
pub(crate) fn survey(dir: &Path, start: usize, count: usize, order: bool) -> Result<()> {
    if count == 0 || start.checked_add(count).is_none_or(|v| v > 50) {
        return Err("expected a nonempty range of 50 fixture coordinates".into());
    }
    let cases = dir.join("cases");
    fs::create_dir_all(&cases)?;
    for slot in start..start + count {
        let (index, f, held) = coordinate(slot)?;
        let file = cases.join(format!("case-{index:03}.json"));
        if file.is_file() {
            let v = load(&file)?;
            if v["format"] != version(order)
                || v["slot"] != slot
                || v["case"] != index
                || v["held"] != held
            {
                return Err("invalid prior case".into());
            }
            continue;
        }
        let v = probe(dir, slot, index, f, held, order)?;
        let tmp = file.with_extension(format!("{}.pending", std::process::id()));
        write_json(&tmp, &v)?;
        fs::rename(tmp, file)?;
        eprintln!("slot {slot} source {index} {}", v["status"]);
    }
    Ok(())
}

/// Repeat the pre-field discovery donor without enrolling it twice in either split.
pub(crate) fn order_pilot(dir: &Path, index: usize) -> Result<()> {
    if index >= COUNT {
        return Err("invalid original source ID".into());
    }
    let mut v = probe(dir, 0, index, fixture(index), false, true)?;
    v["slot"] = Value::Null;
    v["held"] = Value::Null;
    v["selection"] = json!("mechanism control outside the 50-case analysis");
    write_json(&dir.join(format!("case-{index:03}/mechanism.json")), &v)
}
