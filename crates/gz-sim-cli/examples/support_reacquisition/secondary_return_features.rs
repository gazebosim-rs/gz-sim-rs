// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native finite-box refresh controls for previously unloaded support pairs.
use super::*;
#[path = "secondary_material_policy.rs"]
pub(crate) mod material_policy;
use gz_physics_multifidelity::{ContactIntervalReceipt, ReturnFeatureMode};
const TRAVEL_VERSION: &str = "secondary-unloaded-surface-travel-v1";
const VERSION: &str = "secondary-unloaded-feature-witness-v2";
#[derive(Clone, Copy)]
pub(super) enum Refresh {
    Unmodified,
    Every,
    First,
    FirstStale,
    FirstTravel,
    Registered,
    WhileStale,
}
impl Refresh {
    fn label(self) -> &'static str {
        match self {
            Self::Unmodified => "unmodified",
            Self::Every => "every",
            Self::First => "first_unloaded",
            Self::FirstStale | Self::Registered => "first_stale",
            Self::FirstTravel => "first_travel",
            Self::WhileStale => "while_stale",
        }
    }
}
pub(super) fn advance(
    world: &mut MultiFidelityBackend,
    pair: (BodyId, BodyId),
    grid: u32,
    it: ContactSolverIterations,
    kind: Refresh,
    obs: Option<&mut dyn FnMut(ContactSubstep<'_>)>,
) -> Result<(ContactIntervalReceipt, Value)> {
    let (r, counts) = if let Refresh::Unmodified = kind {
        let r = if let Some(observer) = obs {
            world.observe_contact_interval(DT, grid, it, observer)?
        } else {
            world.step_contact_interval(DT, grid, it)?
        };
        (r, json!({"method":kind.label(),"requeries":0}))
    } else if let Refresh::Every = kind {
        let out = world.advance_box_contacts(
            DT,
            grid,
            it,
            BoxContactActions {
                fresh_features: &[pair],
                ..Default::default()
            },
            obs,
        )?;
        (
            out.interval,
            json!({"method":kind.label(),"requeries":out.rebuilt_manifolds}),
        )
    } else {
        let mode = match kind {
            Refresh::First => ReturnFeatureMode::FirstUnloaded,
            Refresh::FirstStale | Refresh::Registered => ReturnFeatureMode::FirstStale,
            Refresh::FirstTravel => ReturnFeatureMode::FirstTravel,
            Refresh::WhileStale => ReturnFeatureMode::WhileStale,
            Refresh::Unmodified | Refresh::Every => unreachable!(),
        };
        let (r, c) = if matches!(kind, Refresh::Registered) {
            let r = world.advance_box_contacts(
                DT,
                grid,
                it,
                BoxContactActions {
                    returning_features: &[pair],
                    ..Default::default()
                },
                obs,
            )?;
            let [f] = r.returning_features.as_slice() else {
                return Err("missing named native geometry receipt".into());
            };
            if (f.foot, f.support) != pair || f.counts.requery != r.rebuilt_manifolds {
                return Err("incorrect named geometry receipt".into());
            }
            (r.interval, f.counts)
        } else {
            world.step_returning_box_features(pair, DT, grid, it, mode, obs)?
        };
        (
            r,
            json!({"method":kind.label(),"unloaded_tests":c.unloaded_tests,"certified_stale":c.stronger_plane,"requeries":c.requery,
                   "first_witness":c.first_witness.map(|w|json!({"other_origin_in_first":w.other_origin_in_first,"other_rotation_in_first":w.other_rotation_in_first,"half_extents":w.half_extents,
                    "cached_axis":w.cached_axis,"stronger_axis":w.stronger_axis,"gap_m":w.gap_m,"tolerance_m":w.tolerance_m,"unloaded_points":w.unloaded_points}))}),
        )
    };
    Ok((r, counts))
}
fn run(dir: &Path, index: usize, show: bool, travel: bool) -> Result<Value> {
    let studies = dir.parent().ok_or("missing studies")?;
    let original = load(&studies.join(format!(
        "multifidelity-secondary-order-field/cases/case-{index:03}.json"
    )))?;
    let f = fixture(index);
    if original["case"] != index || original["fixture"] != serde_json::to_value(f)? {
        return Err("wrong geometry source".into());
    }
    let mut output = json!({"case":index,"slot":original["slot"],"held":original["held"],"format":if travel {TRAVEL_VERSION} else {VERSION},"fixture":f,"status":original["status"]});
    if original["status"] != "measured" {
        output["reason"] = original["reason"].clone();
        return Ok(output);
    }
    let tick = original["incoming"]["cut_tick"]
        .as_u64()
        .ok_or("no qualified cut")?;
    let cut = survey::cuts(f)?.0[0].ok_or("missing primary landing")?;
    let archive = if show {
        Some(load(&dir.join(format!("cases/case-{index:03}.json")))?)
    } else {
        None
    };
    let mut done = false;
    observe_path_until(
        f,
        &cut,
        PATHS[6],
        ProjectionSpan::Native,
        tick,
        |parent, ids, incoming| {
            if incoming.tick != tick {
                return Ok(());
            }
            let [foot, _, pad] = ids;
            let state = parent.airborne_restart_state(foot, pad)?;
            if cache(&state) != original["incoming"]["cache"]
                || motion(&state) != original["incoming"]["motion"]
            {
                return Err("feature candidate changed incoming state".into());
            }
            output["incoming"] = original["incoming"].clone();
            output["cut_tick"] = json!(tick);
            if let Some(saved) = &archive {
                let natural = load(&studies.join(format!(
                    "multifidelity-secondary-natural/cases/case-{index:03}.json"
                )))?;
                let dest = dir.join(format!("case-{index:03}"));
                fs::create_dir_all(dest.join("traces"))?;
                let mut lanes = vec![];
                for (method, label, name) in [
                    (None, "ordinary", "Unchanged native contact geometry"),
                    (
                        Some(Refresh::FirstStale),
                        "witness",
                        "First discredited unloaded normal",
                    ),
                    (
                        if travel {
                            Some(Refresh::FirstTravel)
                        } else {
                            Some(Refresh::Every)
                        },
                        if travel { "travel" } else { "fresh" },
                        if travel {
                            "One native interval of incoming surface travel"
                        } else {
                            "Refresh all resident native box features"
                        },
                    ),
                ] {
                    let mut t = trial(parent, ids, incoming, f, Graph::Retained, (8, 8), method)?;
                    let prior = if method.is_some() { saved } else { &natural };
                    let reference = prior["treatments"]
                        .as_array()
                        .ok_or("missing feature archive")?
                        .iter()
                        .find(|v| {
                            v["graph"] == "retained"
                                && v["passes"] == 8
                                && v["calls"] == 8
                                && v["refresh"]["method"]
                                    == method.map(|m| json!(m.label())).unwrap_or(Value::Null)
                        })
                        .ok_or("no matched feature control")?;
                    if t.row != *reference {
                        return Err("feature replay changed archived receipt".into());
                    }
                    let mut frames = vec![incoming.as_json(), t.end.as_json()];
                    let mut hits = vec![];
                    for n in tick + 2..=tick + 10 {
                        let speed = command(f, n - 1);
                        let command = (speed != t.end.pad.velocity[2]).then_some(speed);
                        t.end = propagate_one(
                            &mut t.world,
                            ids,
                            n,
                            b(4096, 1),
                            command,
                            None,
                            &mut hits,
                        )?;
                        frames.push(t.end.as_json());
                    }
                    write_json(
                        &dest.join(format!("traces/{label}.json")),
                        &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
                    )?;
                    lanes.push(
                        json!({"label":label,"name":name,"file":format!("traces/{label}.json")}),
                    );
                }
                write_json(
                    &dest.join("comparison.json"),
                    &json!({"title":if travel {"A surface-travel threshold at the second support landing"} else {"Rechecking only a disproved finite-box contact normal"},"pair_totals_only":true,"lanes":lanes,"capture_ticks":{"first_return":tick+1,"after_five_ms":tick+10}}),
                )?;
            } else {
                let mut rows = Vec::new();
                for grid in [8, 64, 4096] {
                    for passes in [1, 8, 32] {
                        for graph in [Graph::Retained, Graph::Cold] {
                            let methods: &[Refresh] = if travel {
                                &[Refresh::FirstStale, Refresh::FirstTravel]
                            } else {
                                &[
                                    Refresh::Every,
                                    Refresh::First,
                                    Refresh::FirstStale,
                                    Refresh::WhileStale,
                                ]
                            };
                            for &method in methods {
                                rows.push(
                                    trial(
                                        parent,
                                        ids,
                                        incoming,
                                        f,
                                        graph,
                                        (grid, passes),
                                        Some(method),
                                    )?
                                    .row,
                                );
                            }
                        }
                    }
                }
                let mut ordinary = vec![];
                for graph in [Graph::Retained, Graph::Cold] {
                    ordinary.push(
                        trial(
                            parent,
                            ids,
                            incoming,
                            f,
                            graph,
                            (8, 8),
                            Some(Refresh::Unmodified),
                        )?
                        .row,
                    );
                }
                output["ordinary_onset"] = json!(ordinary);
                output["treatments"] = json!(rows);
            }
            done = true;
            Ok(())
        },
    )?;
    if !done {
        return Err("missing matched secondary update".into());
    }
    Ok(output)
}
pub(crate) fn field(dir: &Path, start: usize, count: usize, travel: bool) -> Result<()> {
    if count == 0 || start.checked_add(count).is_none_or(|n| n > 50) {
        return Err("invalid secondary feature coordinate range".into());
    }
    let dest = dir.join("cases");
    fs::create_dir_all(&dest)?;
    for slot in start..start + count {
        let (index, _, _) = cohort::coordinate(slot)?;
        let case = dest.join(format!("case-{index:03}.json"));
        if case.exists() {
            let r = load(&case)?;
            if r["format"] != (if travel { TRAVEL_VERSION } else { VERSION }) || r["slot"] != slot {
                return Err("incompatible feature case".into());
            }
            continue;
        }
        let result = run(dir, index, false, travel)?;
        let tmp = case.with_extension(format!("{}.pending", std::process::id()));
        write_json(&tmp, &result)?;
        fs::rename(tmp, case)?;
        eprintln!("checked finite-box witness at second return, source {index}, slot {slot}");
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, index: usize, travel: bool) -> Result<()> {
    run(dir, index, true, travel)?;
    Ok(())
}

/// Compare the complete unmerged production interface with the frozen geometry factorial.
pub(crate) fn registered_field(dir: &Path, start: usize, count: usize) -> Result<()> {
    if count == 0 || start.checked_add(count).is_none_or(|v| v > 50) {
        return Err("invalid return coordinate range".into());
    }
    let out = dir.join("cases");
    fs::create_dir_all(&out)?;
    let archive = dir
        .parent()
        .ok_or("missing studies")?
        .join("multifidelity-secondary-return-features/cases");
    for slot in start..start + count {
        let (id, f, held) = cohort::coordinate(slot)?;
        let name = format!("case-{id:03}.json");
        let dest = out.join(&name);
        if dest.exists() {
            let done = load(&dest)?;
            if done["format"] != "registered-native-return-plane-v1" || done["slot"] != slot {
                return Err("incompatible registered controls".into());
            }
            continue;
        }
        let src = load(&archive.join(&name))?;
        let tick = src["cut_tick"].as_u64().ok_or("no qualified donor")?;
        let cut = survey::cuts(f)?.0[0].ok_or("missing primary")?;
        let mut rows = vec![];
        observe_path_until(
            f,
            &cut,
            PATHS[6],
            ProjectionSpan::Native,
            tick,
            |parent, ids, incoming| {
                if incoming.tick != tick {
                    return Ok(());
                }
                let state = parent.airborne_restart_state(ids[0], ids[2])?;
                if motion(&state) != src["incoming"]["motion"]
                    || cache(&state) != src["incoming"]["cache"]
                {
                    return Err("changed production restart".into());
                }
                for grid in [8, 64, 4096] {
                    for passes in [1, 8, 32] {
                        for graph in [Graph::Retained, Graph::Cold] {
                            let t = trial(
                                parent,
                                ids,
                                incoming,
                                f,
                                graph,
                                (grid, passes),
                                Some(Refresh::Registered),
                            )?
                            .row;
                            let saved = src["treatments"]
                                .as_array()
                                .ok_or("missing archived outcomes")?
                                .iter()
                                .find(|v| {
                                    v["calls"] == grid
                                        && v["passes"] == passes
                                        && v["graph"] == t["graph"]
                                        && v["refresh"]["method"] == "first_stale"
                                })
                                .ok_or("missing archived row")?;
                            if t != *saved {
                                return Err(format!("production outcome differs at source {id}, grid {grid}, passes {passes}, graph {}",t["graph"]).into());
                            }
                            rows.push(json!({"calls":grid,"passes":passes,"graph":t["graph"],"normal_ns":t["normal_ns"],"first_ns":t["first_ns"],"world_impulse_ns":t["world_impulse_ns"],"refresh":t["refresh"]}));
                        }
                    }
                }
                Ok(())
            },
        )?;
        if rows.len() != 18 {
            return Err("production field omitted a branch".into());
        }
        let temp = dest.with_extension(format!("{}.pending", std::process::id()));
        write_json(
            &temp,
            &json!({"format":"registered-native-return-plane-v1","case":id,"slot":slot,"held":held,"cut_tick":tick,"matches":rows}),
        )?;
        fs::rename(temp, dest)?;
    }
    Ok(())
}

pub(crate) fn material_replay(dir: &Path, id: usize) -> Result<()> {
    let src = load(&dir.join(format!("cases/case-{id:03}.json")))?;
    let fixed = load(&dir.parent().ok_or("missing studies")?.join(format!(
        "multifidelity-secondary-return-features/cases/case-{id:03}.json"
    )))?;
    let f = fixture(id);
    let tick = src["cut_tick"].as_u64().ok_or("missing source tick")?;
    let start = tick * 500_000;
    let dest = dir.join(format!("case-{id:03}"));
    fs::create_dir_all(dest.join("traces"))?;
    let mut lanes = vec![];
    let geo = json!({"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF});
    let mut emit = |id: &str, name: &str, frames: Vec<Value>, analytic: bool| -> Result<()> {
        let file = format!("traces/{id}.json");
        write_json(&dest.join(&file), &json!({"geometry":geo,"frames":frames}))?;
        lanes.push(json!({"label":id,"name":name,"analytical":analytic,"file":file}));
        Ok(())
    };
    let cut = survey::cuts(f)?.0[0].ok_or("no primary source")?;
    observe_path_until(
        f,
        &cut,
        PATHS[6],
        ProjectionSpan::Native,
        tick,
        |parent, ids, entry| {
            if entry.tick != tick {
                return Ok(());
            }
            let [foot, _, pad] = ids;
            let state = parent.airborne_restart_state(foot, pad)?;
            if src["incoming"]["cache"] != cache(&state)
                || src["incoming"]["motion"] != motion(&state)
            {
                return Err("material replay lost matched incoming state".into());
            }
            for (registered, label, name) in [
                (false, "ordinary", "Ordinary native geometry"),
                (true, "registered", "Registered returning-pair requery"),
            ] {
                let mut w = parent.copy_airborne_landing(foot, pad, false)?;
                let speed = command(f, tick);
                if speed != entry.pad.velocity[2] {
                    w.set_world_velocity(pad, [0., 0., speed], [0.; 3])?;
                }
                let row = |b: [BodyState; 3], n: u64, ns: u64, j: f64| json!({"tick":n,"time_ns":start+ns,"time_us":(start+ns)/1000,"native_steps":u32::from(n>0),"foot":Body::from(b[0]),"upper":Body::from(b[1]),"pad":Body::from(b[2]),"normal_impulse_ns":j});
                let mut frames = vec![row(states(&w, ids)?, 0, 0, 0.)];
                let mut sum = 0.;
                let mut signed = DVec3::ZERO;
                let mut failed = false;
                let mut record = |s: ContactSubstep<'_>| {
                    let Ok(b) = super::super::sample_states(&s, ids) else {
                        failed = true;
                        return;
                    };
                    let mut load = 0.;
                    for p in s.contact_patches() {
                        if (p.a, p.b) != (foot.min(pad), foot.max(pad)) {
                            if p.points.iter().any(|x| x.normal_impulse > 0.) {
                                failed = true;
                            }
                            continue;
                        }
                        let sign = if p.a == foot { 1. } else { -1. };
                        for pt in p.points {
                            if pt.normal_impulse > 0. {
                                let Some(t) = pt.tangent_impulse_on_a else {
                                    failed = true;
                                    continue;
                                };
                                load += pt.normal_impulse;
                                signed += sign
                                    * (DVec3::from(p.normal_on_a) * pt.normal_impulse
                                        + DVec3::from(t));
                            }
                        }
                    }
                    sum += load;
                    frames.push(row(
                        b,
                        frames.len() as u64,
                        s.elapsed.as_nanos() as u64,
                        load,
                    ));
                };
                let it = ContactSolverIterations {
                    temporal: 4,
                    pgs: None,
                    relaxation: Some(8),
                };
                let method = if registered {
                    Refresh::Registered
                } else {
                    Refresh::Unmodified
                };
                let (receipt, _) = advance(&mut w, (foot, pad), 8, it, method, Some(&mut record))?;
                let raw = if registered {
                    fixed["treatments"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .find(|v| {
                            v["calls"] == 8
                                && v["passes"] == 8
                                && v["graph"] == "retained"
                                && v["refresh"]["method"] == "first_stale"
                        })
                        .unwrap()
                } else {
                    &fixed["ordinary_onset"][0]
                };
                let expected: DVec3 =
                    DVec3::from_array(serde_json::from_value(raw["world_impulse_ns"].clone())?);
                if failed
                    || frames.len() != 9
                    || receipt.native_steps != 8
                    || (sum - raw["normal_ns"].as_f64().unwrap()).abs() > 1e-12
                    || signed.distance(expected) > 1e-11
                {
                    return Err(
                        "registered-bin replay differs from original physical receipts".into(),
                    );
                }
                emit(label, name, frames, false)?;
            }
            Ok(())
        },
    )?;
    let mut frames = vec![];
    let mut prev = 0.;
    for i in 0..=8 {
        let ns = i * 62_500;
        let r = src["targets"]
            .as_array()
            .unwrap()
            .iter()
            .find(|x| x["time_ns"] == ns)
            .ok_or("mechanical frame missing")?;
        if r["qualified"] != true {
            return Err("unqualified mechanical frame".into());
        }
        let b = &r["model"]["end"];
        let load = b["total_normal_ns"].as_f64().unwrap();
        frames.push(json!({"tick":i,"time_ns":start+ns,"time_us":(start+ns)/1000,"native_steps":0,"foot":b["foot"],"upper":b["upper"],"pad":b["pad"],"normal_impulse_ns":load-prev}));
        prev = load;
    }
    emit("physical", "Piecewise rigid material contact", frames, true)?;
    if lanes.len() != 3 {
        return Err("no native second-landing donor".into());
    }
    let first = (src["material"]["time_s"].as_f64().unwrap() * 1e9 / 62_500.).ceil() as u64;
    write_json(
        &dest.join("comparison.json"),
        &json!({"title":"The material response of a measured second landing","pair_totals_only":true,"interval_samples":true,"sampled_ticks":true,"clock_origin_ns":start,"public_dt_ns":500_000,"lanes":lanes,"capture_ticks":{"first_material_reaction":first}}),
    )
}
