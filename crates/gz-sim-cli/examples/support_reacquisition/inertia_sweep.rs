// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Test a whole-public-tick relative clearance envelope independently of the legacy arming threshold.
use super::*;
#[path = "inertia_compact.rs"]
pub(crate) mod compact;
#[path = "inertia_gravity.rs"]
pub(crate) mod gravity;
fn xml(r: &Physical, n: u32, sweep: bool) -> (Profile, String) {
    let (p, text) = released::configured(r, n, true);
    if sweep {
        (p,text.replace("</refine_rotation_timing>","</refine_rotation_timing><detect_separated_crossings>true</detect_separated_crossings>"))
    } else {
        (p, text)
    }
}
pub(crate) fn inspect(path: &Path) -> Result<()> {
    let outcomes: Value = serde_json::from_slice(&fs::read(path.join("single.json"))?)?;
    let mut rows = vec![];
    for row in outcomes["cases"].as_array().unwrap() {
        let id = row["id"].as_u64().unwrap() as usize;
        let failed = row["trials"].as_array().unwrap().iter().any(|r| {
            r["budget"] == 8
                && r["sweep"] == true
                && (r["native_calls"] == 8 || r["metrics"]["com_error_m"].as_f64().unwrap() > 10e-6)
        });
        if !failed && ![248, 249, 886].contains(&id) {
            continue;
        }
        let r = Physical::new(id);
        let (_, text) = xml(&r, 8, true);
        let (mut sim, ids) = sdf_features::load_finite_box(&text, false)?;
        let mut log = vec![];
        for tick in 0..8 {
            let e = backend(&sim);
            let c = gyro::Cut::bodies(e, ids, 0);
            let nominal = e.rigid_box_sweep(ids.foot, ids.pad, Duration::from_micros(500))?;
            let q = DQuat::from_array(c.foot.orientation);
            let mut w = q.conjugate() * DVec3::from(c.foot.angular_velocity);
            let mut orientation = q;
            let pq = DQuat::from_array(c.pad.orientation);
            let center = DVec3::from(c.foot.position) + q * ARM - DVec3::from(c.pad.position);
            let v = DVec3::from(c.foot.velocity) - DVec3::from(c.pad.velocity);
            let mut min_gap = f64::INFINITY;
            let mut min_edge = f64::INFINITY;
            let mut start_gap = 0.;
            let mut end_gap = 0.;
            let mut lowest = 0;
            for i in 0..=100 {
                let t = i as f64 * 5e-6;
                let mut at = f64::INFINITY;
                for (j, p) in corners().iter().enumerate() {
                    let x = pq.conjugate() * (center + t * v + orientation * (*p - ARM));
                    if x.z - PAD_HALF[2] < min_gap {
                        lowest = j;
                        min_gap = x.z - PAD_HALF[2];
                    }
                    at = at.min(x.z - PAD_HALF[2]);
                    min_edge = min_edge.min((PAD_HALF[0] - x.x.abs()).min(PAD_HALF[1] - x.y.abs()));
                }
                if i == 0 {
                    start_gap = at;
                }
                if i == 100 {
                    end_gap = at;
                    break;
                }
                (orientation, w) = advance_euler(r.body, orientation, w, 5e-6);
            }
            sim.step_exact(1)?;
            let scheduled = last_refinement(&sim)
                .filter(|x| x.sim_time == sim.elapsed())
                .map(|x| x.interval.native_steps);
            let reason = last_box_timing(&sim)
                .filter(|x| x.sim_time == sim.elapsed())
                .and_then(|x| x.timing.decline)
                .map(|x| format!("{x:?}"));
            log.push(json!({"tick":tick+1,"direct":format!("{nominal:?}"),"scheduled":scheduled,"timing_decline":reason,"start_gap_m":start_gap,"end_gap_m":end_gap,"free_min_gap_m":min_gap,"free_min_edge_m":min_edge,"lowest":lowest}));
        }
        rows.push(json!({"id":id,"arrival_ns":r.arrival*1e9,"ticks":log}));
    }
    write_json(&path.join("misses.json"), &json!({"cases":rows}))
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut isolated = vec![];
    // Original 320 plus 320 new, unseen triaxial donors from the continued sequence.
    for id in (0..320).chain(640..960) {
        let r = Physical::new(id);
        let c = r.certify();
        if c.reason.is_some() {
            continue;
        }
        let mut trials = vec![];
        for (n, on) in [4, 8, 16].into_iter().flat_map(|n| [(n, false), (n, true)]) {
            let (p, text) = xml(&r, n, on);
            let (row, frames, _) = scene::run(&r, false, p, &text)?;
            if id == 248 && n == 8 {
                let name = if on { "swept" } else { "baseline" };
                separated::single_rigid_trace(path, name, &frames)?;
                fs::write(path.join(format!("{name}.sdf")), text)?;
            }
            trials.push(json!({"budget":n,"sweep":on,"native_calls":row["native_calls"],"selected":row["ticks"].as_array().unwrap().iter().filter(|t|t["calls"]!=1).count(),
                "first_ns":row["first_positive_end_ns"],"early_ns":row["preimpact_measured_ns"],"metrics":row["metrics"]}));
        }
        if id == 248 {
            separated::single_rigid_trace(path, "analytic", &r.trace())?;
        }
        isolated.push(json!({"id":id,"arrival_ns":(r.arrival*1e9).round()as u64,"trials":trials}));
    }
    write_json(
        &path.join("single.json"),
        &json!({"cases":isolated,"view_id":248}),
    )?;
    let mut returns = vec![];
    for id in 0..DONORS {
        let r = donor(id);
        let h = History::solve(&r, RK_H);
        let n = ((h.end + 1e-13) * 2000.).floor() as u64;
        if h.events
            .iter()
            .filter(|e| e.time_s <= n as f64 * 0.0005)
            .count()
            < 2
        {
            continue;
        }
        let mut trials = vec![];
        for budget in [4, 8, 16] {
            for on in [false, true] {
                let (_, text) = xml(&r, budget, on);
                let (row, _) = run(&r, &h, &text, n)?;
                trials.push(json!({"budget":budget,"sweep":on,"trial":row}));
            }
        }
        returns.push(json!({"id":id,"events":h.events,"ticks":n,"trials":trials}));
    }
    write_json(&path.join("returns.json"), &json!({"cases":returns}))?;
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":"Finding a rotating strike between public samples","lanes":[
        {"label":"baseline","name":"Previous impact nomination","file":"traces/baseline.json"},
        {"label":"swept","name":"Whole-tick separated-box test","file":"traces/swept.json"},
        {"label":"analytic","name":"Independent Euler flight and impact","file":"traces/analytic.json","analytical":true}]}),
    )
}
pub(crate) fn cost(folder: &Path) -> Result<()> {
    fs::create_dir_all(folder)?;
    let data: Value = serde_json::from_slice(&fs::read(
        folder
            .parent()
            .ok_or("need timing subdirectory")?
            .join("returns.json"),
    )?)?;
    let mut all = vec![];
    for c in data["cases"].as_array().unwrap() {
        let id = c["id"].as_u64().unwrap() as usize;
        let n = c["ticks"].as_u64().unwrap();
        let r = donor(id);
        let mut fixtures = vec![];
        for on in [false, true] {
            let (_, text) = xml(&r, 8, on);
            let (mut sim, ids) = sdf_features::load_finite_box(&text, false)?;
            sim.step_exact(n)?;
            let end = state(backend(&sim), ids)?;
            clocked(&text, n, end)?;
            fixtures.push((on, text, end));
        }
        let mut times: [Vec<Vec<u64>>; 2] = Default::default();
        for repeat in 0..4 {
            for pos in 0..2 {
                let i = (pos + repeat + id) % 2;
                times[i].push(clocked(&fixtures[i].1, n, fixtures[i].2)?);
            }
        }
        for ((on, _, _), samples) in fixtures.into_iter().zip(times) {
            all.push(json!({"id":id,"sweep":on,"steps_ns":samples}));
        }
    }
    write_json(&folder.join("cost.json"), &json!({"rows":all}))
}
