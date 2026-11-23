// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cross fresh/retained solver ordering with fixed-time relaxation at a measured support return.
use super::*;
#[path = "secondary_return_features.rs"]
pub(crate) mod features;
use features::Refresh;
const VERSION: &str = "secondary-native-relaxation-grid-v2";
const NATURAL_VERSION: &str = "secondary-native-unmodified-relaxation-v1";
const GRAPHS: [Graph; 4] = [
    Graph::WarmSorted,
    Graph::ColdSorted,
    Graph::WarmExchanged,
    Graph::ColdExchanged,
];
const GRIDS: [u32; 4] = [8, 64, 512, 4096];
const PASSES: [usize; 6] = [1, 2, 4, 8, 16, 32];
struct Trial {
    world: MultiFidelityBackend,
    end: Tick,
    row: Value,
}
fn trial(
    parent: &MultiFidelityBackend,
    ids: [BodyId; 3],
    incoming: &Tick,
    f: Fixture,
    graph: Graph,
    solver: (u32, usize),
    refresh: Option<Refresh>,
) -> Result<Trial> {
    let (grid, passes) = solver;
    let [foot, _, pad] = ids;
    let mut world = parent.copy_airborne_landing(foot, pad, graph.cold())?;
    let speed = command(f, incoming.tick);
    if speed != incoming.pad.velocity[2] {
        world.set_world_velocity(pad, [0., 0., speed], [0.; 3])?;
    }
    let mut observed = Interval::new(states(&world, ids)?);
    let mut err = None;
    let settings = ContactSolverIterations {
        temporal: 4,
        pgs: None,
        relaxation: Some(passes),
    };
    let mut accept = |s: ContactSubstep<'_>| {
        if err.is_none() {
            err = observed.observe(s, ids).err();
        }
    };
    let mut refreshed = None;
    let (receipt, counts) = if let Some(method) = refresh {
        let (receipt, counts) = features::advance(
            &mut world,
            (foot, pad),
            grid,
            settings,
            method,
            Some(&mut accept),
        )?;
        refreshed = Some(counts);
        (receipt, None)
    } else if let Some(order) = graph.order() {
        let (receipt, counts) =
            world.observe_landing_row_order((foot, pad), DT, grid, settings, order, &mut accept)?;
        (receipt, Some(counts))
    } else {
        (
            world.observe_contact_interval(DT, grid, settings, &mut accept)?,
            None,
        )
    };
    if let Some(err) = err {
        return Err(err);
    }
    let [root, upper, pad_state] = states(&world, ids)?;
    let mut normal = DVec3::ZERO;
    let mut tangent = DVec3::ZERO;
    let mut scalar_load = 0.;
    let mut first = None;
    for pair in receipt.pairs {
        if (pair.a, pair.b) != (foot.min(pad), foot.max(pad)) {
            if pair.normal_impulse > 0. {
                return Err("unselected reaction".into());
            }
            continue;
        }
        let sign = if pair.a == foot { 1. } else { -1. };
        normal += DVec3::from(pair.normal_impulse_on_a) * sign;
        tangent += DVec3::from(pair.tangent_impulse_on_a.ok_or("missing signed friction")?) * sign;
        scalar_load += pair.normal_impulse;
        first = pair.first_positive_end;
    }
    let end = Tick {
        foot: root.into(),
        upper: upper.into(),
        pad: pad_state.into(),
        tick: incoming.tick + 1,
        calls: grid,
        first,
        normal: scalar_load,
        tangent,
        extra: 0.,
        gap: supported_gap(root, pad_state),
    };
    let w = Bodies::from_states([root, upper, pad_state]);
    let j = normal + tangent;
    let h = observed.sums.torque_normal + observed.sums.torque_tangent;
    let dp = (w.momentum() - observed.entry.momentum() + DVec3::Z * (2. * 9.81 * DT.as_secs_f64())
        - j)
        .length();
    let dh = (w.angular_momentum() - observed.entry.angular_momentum() - h).length();
    if observed.calls != grid
        || observed.duration != CLOCK_NS
        || counts.is_some_and(|c| c.nonempty_manifolds_visited == 0) && end.normal > 0.
        || normal.distance(observed.sums.vector) > 1e-9
        || tangent.distance(observed.sums.tangent) > 1e-9
        || (scalar_load - observed.sums.normal).abs() > 1e-9
        || !dp.is_finite()
        || !dh.is_finite()
    {
        return Err(format!("native secondary balance: {dp} {dh}").into());
    }
    let mut row = json!({"graph":graph.name(),"calls":grid,"passes":passes,"first_ns":first.map(|t|t.as_nanos() as u64),"loaded_calls":observed.positive_calls,
                  "normal_ns":scalar_load,"normal_vector_ns":normal.to_array(),"tangent_vector_ns":tangent.to_array(),"world_impulse_ns":j.to_array(),
                  "quadrant_ns":observed.sums.quadrant,"moment_nms":h.to_array(),"end_com_m":w.com().to_array(),"end_spin_rad_s":root.angular_velocity,
                  "start_momentum_ns":observed.entry.momentum().to_array(),"end_momentum_ns":w.momentum().to_array(),
                  "start_angular_momentum_nms":observed.entry.angular_momentum().to_array(),"end_angular_momentum_nms":w.angular_momentum().to_array(),
                  "peak_native_linear_residual_ns":observed.max_dp,"peak_native_angular_residual_nms":observed.max_dh,
                  "linear_residual_ns":dp,"angular_residual_nms":dh,"visited_rows":counts.map(|c| c.nonempty_manifolds_visited),"reordered_rows":counts.map(|c| c.ordered_manifolds_changed)});
    if let Some(info) = refreshed {
        row["refresh"] = info;
    }
    if graph.order().is_none()
        && (grid == 4096 && matches!(passes, 1 | 32)
            || refresh.is_some() && grid == 8 && passes == 8)
    {
        row["first_positive"] = observed.first.unwrap_or(Value::Null);
    }
    Ok(Trial { world, end, row })
}
fn study(dir: &Path, index: usize, replay: bool, natural: bool) -> Result<Value> {
    let (version, graphs): (&str, &[Graph]) = if natural {
        (NATURAL_VERSION, &[Graph::Retained, Graph::Cold])
    } else {
        (VERSION, &GRAPHS)
    };
    let root = dir.parent().ok_or("missing sibling order field")?;
    let src = load(&root.join(format!(
        "multifidelity-secondary-order-field/cases/case-{index:03}.json"
    )))?;
    let f = fixture(index);
    if src["case"] != index
        || src["fixture"] != serde_json::to_value(f)?
        || src["format"] != "second-landing-native-row-order-cohort-v1"
    {
        return Err("wrong native ordering source".into());
    }
    let mut out = json!({"format":version,"case":index,"held":src["held"],"slot":src["slot"],"fixture":f,"status":src["status"]});
    if src["status"] == "censored" {
        out["reason"] = src["reason"].clone();
        return Ok(out);
    }
    let tick = src["incoming"]["cut_tick"]
        .as_u64()
        .ok_or("incomplete native source")?;
    let (cuts, _) = survey::cuts(f)?;
    let first = cuts[0].ok_or("missing primary release")?;
    let case = dir.join(format!("case-{index:03}"));
    let archived = if replay {
        Some(load(&dir.join(format!("cases/case-{index:03}.json")))?)
    } else {
        None
    };
    let mut rows = Vec::new();
    observe_path_until(
        f,
        &first,
        PATHS[6],
        ProjectionSpan::Native,
        tick,
        |parent, ids, incoming| {
            if incoming.tick != tick {
                return Ok(());
            }
            let [foot, _, pad] = ids;
            let state = parent.airborne_restart_state(foot, pad)?;
            if motion(&state) != src["incoming"]["motion"]
                || cache(&state) != src["incoming"]["cache"]
            {
                return Err("second-landing donor changed".into());
            }
            out["incoming"] = src["incoming"].clone();
            out["cut_tick"] = json!(tick);
            if let Some(previous) = &archived {
                fs::create_dir_all(case.join("traces"))?;
                let (title, treatments) = if natural {
                    (
                        "Ordinary solver order across contact budgets at a support return",
                        [
                            (
                                Graph::Retained,
                                8,
                                1,
                                "ordinary",
                                "Eight native calls, one pass",
                            ),
                            (
                                Graph::Retained,
                                8,
                                32,
                                "relaxed",
                                "Eight native calls, 32 passes",
                            ),
                            (
                                Graph::Retained,
                                4096,
                                32,
                                "fine",
                                "4,096 native calls, 32 passes",
                            ),
                        ],
                    )
                } else {
                    (
                        "Ordering and fixed-time convergence at a support return",
                        [
                            (
                                Graph::WarmSorted,
                                4096,
                                1,
                                "sorted",
                                "Retained ordered rows, one pass",
                            ),
                            (
                                Graph::WarmExchanged,
                                4096,
                                1,
                                "exchanged",
                                "Retained exchanged rows, one pass",
                            ),
                            (
                                Graph::WarmSorted,
                                4096,
                                32,
                                "relaxed",
                                "Retained ordered rows, 32 passes",
                            ),
                        ],
                    )
                };
                let mut lanes = vec![];
                for (graph, grid, passes, label, name) in treatments {
                    let mut t = trial(parent, ids, incoming, f, graph, (grid, passes), None)?;
                    let saved = previous["treatments"]
                        .as_array()
                        .ok_or("missing enrolled treatment")?
                        .iter()
                        .find(|r| {
                            r["graph"] == graph.name()
                                && r["calls"] == grid
                                && r["passes"] == passes
                        })
                        .ok_or("replay grid absent")?;
                    if &t.row != saved {
                        return Err("replay diverged from original factorial".into());
                    }
                    let mut frames = vec![incoming.as_json(), t.end.as_json()];
                    let mut impacts = vec![];
                    for n in tick + 2..=tick + 10 {
                        let velocity = command(f, n - 1);
                        let change = (velocity != t.end.pad.velocity[2]).then_some(velocity);
                        t.end = propagate_one(
                            &mut t.world,
                            ids,
                            n,
                            b(4096, 1),
                            change,
                            None,
                            &mut impacts,
                        )?;
                        frames.push(t.end.as_json());
                    }
                    write_json(
                        &case.join(format!("traces/{label}.json")),
                        &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
                    )?;
                    lanes.push(
                        json!({"label":label,"name":name,"file":format!("traces/{label}.json")}),
                    );
                }
                write_json(
                    &case.join("comparison.json"),
                    &json!({"title":title,"lanes":lanes,"pair_totals_only":true,"capture_ticks":{"return":tick+1,"after_five_ms":tick+10}}),
                )?;
            } else {
                for grid in GRIDS {
                    for passes in PASSES {
                        for &graph in graphs {
                            let t = trial(parent, ids, incoming, f, graph, (grid, passes), None)?;
                            if grid == 4096 && passes == 1 {
                                let r = src["branches"]
                                    .as_array()
                                    .ok_or("missing original controls")?
                                    .iter()
                                    .find(|b| b["graph"] == graph.name())
                                    .ok_or("missing matching graph")?;
                                if (scalar(&r["first_public"]["normal_ns"])?
                                    - scalar(&t.row["normal_ns"])?)
                                .abs()
                                    > 1e-10
                                    || r["first_native_ns"] != t.row["first_ns"]
                                    || vec3(&r["first_public"]["normal_vector_ns"])?
                                        .distance(vec3(&t.row["normal_vector_ns"])?)
                                        > 1e-10
                                    || vec3(&r["first_public"]["tangent_vector_ns"])?
                                        .distance(vec3(&t.row["tangent_vector_ns"])?)
                                        > 1e-10
                                {
                                    return Err(
                                        "original ordered first interval did not reproduce".into(),
                                    );
                                }
                            }
                            rows.push(t.row);
                        }
                    }
                }
            }
            Ok(())
        },
    )?;
    if !replay && rows.len() != GRIDS.len() * PASSES.len() * graphs.len() {
        return Err("short factorial".into());
    }
    out["treatments"] = json!(rows);
    Ok(out)
}
pub(crate) fn field(dir: &Path, start: usize, count: usize, natural: bool) -> Result<()> {
    let version = if natural { NATURAL_VERSION } else { VERSION };
    if count == 0 || start.checked_add(count).is_none_or(|n| n > 50) {
        return Err("invalid secondary stability coordinate range".into());
    }
    let dest = dir.join("cases");
    fs::create_dir_all(&dest)?;
    for slot in start..start + count {
        let (index, _, _) = cohort::coordinate(slot)?;
        let path = dest.join(format!("case-{index:03}.json"));
        if path.is_file() {
            let r = load(&path)?;
            if r["format"] != version || r["case"] != index || r["slot"] != slot {
                return Err("invalid earlier case".into());
            }
            continue;
        }
        let result = study(dir, index, false, natural)?;
        let tmp = path.with_extension(format!("{}.pending", std::process::id()));
        write_json(&tmp, &result)?;
        fs::rename(tmp, path)?;
        eprintln!(
            "fixed-time secondary case {index}, slot {slot}: {}",
            result["status"]
        );
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, index: usize, natural: bool) -> Result<()> {
    study(dir, index, true, natural)?;
    Ok(())
}
