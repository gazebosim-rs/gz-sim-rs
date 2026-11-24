// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the guarded rigid corner projection against independent manufactured impulses.
use super::*;
#[path = "inertia_point_return.rs"]
pub(crate) mod near_return;
#[path = "inertia_neighbors.rs"]
pub(crate) mod neighbors;
const MODES_POINT: [(u32, bool, bool); 8] = [
    (4, false, false),
    (4, true, false),
    (8, false, false),
    (8, true, false),
    (8, false, true),
    (8, true, true),
    (16, false, false),
    (16, true, false),
];
fn authored(i: &Impact, n: u32, project: bool, compact: bool) -> String {
    let mut text = configured(i, PROFILES[1]).replace(
        "<native_steps>8</native_steps>",
        &format!("<native_steps>{n}</native_steps>"),
    );
    if project {
        text = text.replace(
            "</detect_separated_crossings>",
            "</detect_separated_crossings><project_rigid_corner>true</project_rigid_corner>",
        );
    }
    if compact {
        text = text.replace(
            "</detect_separated_crossings>",
            "</detect_separated_crossings><compact_impact_budget>true</compact_impact_budget>",
        );
    }
    text
}
#[derive(Clone, Copy)]
pub(crate) enum Study {
    Projection,
    Departure,
}
impl Study {
    fn text(self, i: &Impact, n: u32, on: bool, compact: bool) -> String {
        match self {
            Self::Projection => authored(i, n, on, compact),
            Self::Departure => {
                let text = authored(i, n, true, compact);
                if on {
                    text.replace("</project_rigid_corner>","</project_rigid_corner><follow_rigid_departure>true</follow_rigid_departure>")
                } else {
                    text
                }
            }
        }
    }
}
pub(crate) fn write(path: &Path, study: Study) -> Result<()> {
    let departure = matches!(study, Study::Departure);
    let featured = if departure { 1353 } else { 1160 };
    fs::create_dir_all(path.join("traces"))?;
    let mut all = vec![];
    let mut declined = std::collections::BTreeMap::new();
    for id in (0..192).chain(1152..1600) {
        for mode in 0..4 {
            let i = manufactured(id, mode);
            if i.r.closing <= 0.05 || i.loss <= 0. || i.unique_margin <= 1e-8 {
                continue;
            }
            if let Some(reason) = i.r.certify().reason {
                *declined.entry(reason).or_insert(0usize) += 1;
                continue;
            }
            let mut rows = vec![];
            for (n, on, compact) in MODES_POINT {
                let text = study.text(&i, n, on, compact);
                let (row, frames) = run(&i, PROFILES[1], &text)?;
                if id == featured && mode == 3 && n == 8 && !compact {
                    let label = match (departure, on) {
                        (false, false) => "native",
                        (true, false) | (false, true) => "projected",
                        (true, true) => "departure",
                    };
                    separated::single_rigid_trace(path, label, &frames)?;
                    fs::write(path.join(format!("{label}.sdf")), text)?;
                    if on {
                        separated::single_rigid_trace(path, "analytic", &i.r.trace())?;
                    }
                }
                rows.push(json!({"budget":n,"project":on||departure,"follow":departure.then_some(on),"compact":compact,"run":row}));
            }
            all.push(json!({"reference":law(&i),"trials":rows}));
        }
    }
    write_json(
        &path.join("study.json"),
        &json!({"cases":all,"declines":declined,"view":[featured,3]}),
    )?;
    if departure {
        return write_json(
            &path.join("comparison.json"),
            &json!({"single_body":true,"title":"Leaving a loaded corner under positive outward acceleration","lanes":[{"label":"projected","name":"Projected impulse","file":"traces/projected.json"},{"label":"departure","name":"Certified post-impact flight","file":"traces/departure.json"},{"label":"analytic","name":"Independent Coulomb and Euler reference","file":"traces/analytic.json","analytical":true}]}),
        );
    }
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":"A single measured frictional corner impulse","lanes":[{"label":"native","name":"Native measured pulse","file":"traces/native.json"},{"label":"projected","name":"Guarded spatial point impulse","file":"traces/projected.json"},{"label":"analytic","name":"Independent Coulomb and Euler reference","file":"traces/analytic.json","analytical":true}]}),
    )
}
pub(crate) fn cost(path: &Path, study: Study) -> Result<()> {
    let departure = matches!(study, Study::Departure);
    fs::create_dir_all(path)?;
    let data: Value = serde_json::from_slice(&fs::read(
        path.parent()
            .ok_or("need cost subdirectory")?
            .join("study.json"),
    )?)?;
    let mut rows = vec![];
    for c in data["cases"].as_array().unwrap() {
        let r = &c["reference"];
        let id = r["id"].as_u64().unwrap() as usize;
        let m = MODES.iter().position(|s| r["mode"] == *s).unwrap();
        let i = manufactured(id, m);
        let mut runs = vec![];
        for (on, compact) in [(false, false), (true, false), (false, true), (true, true)] {
            let text = study.text(&i, 8, on, compact);
            let (mut sim, ids) = load(&text, false)?;
            sim.step_exact(8)?;
            let end = state(backend(&sim), ids)?;
            clocked(&text, 8, end)?;
            runs.push((on, compact, text, end));
        }
        let mut times: [Vec<Vec<u64>>; 4] = Default::default();
        for rep in 0..4 {
            for j in 0..4 {
                let k = (if rep % 2 == 0 { j } else { 3 - j } + id + m + rep) % 4;
                times[k].push(clocked(&runs[k].2, 8, runs[k].3)?);
            }
        }
        for ((on, compact, _, _), times) in runs.into_iter().zip(times) {
            rows.push(
                json!({"id":id,"mode":MODES[m],"project":on||departure,"follow":departure.then_some(on),"compact":compact,"steps_ns":times}),
            );
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":rows}))
}

// Compare both authored policies up through the first public landing. A negative
// free normal acceleration at tangency rules out force-free departure for any
// positive-duration interval, even when the isolated instantaneous law is unique.
pub(crate) fn refusals(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = Vec::new();
    for id in 0..1024 {
        for (mode,_) in MODES.iter().enumerate() {
            let i = manufactured(id, mode);
            let r = &i.r;
            if r.closing <= 0.05
                || i.loss <= 0.
                || i.unique_margin <= 1e-8
                || r.certify_branches(&[(0., r.pre)]).reason.is_some()
            {
                continue;
            }
            let w = r.body.omega(r.q, r.post.l);
            let alpha = r.body.omega(r.q, -w.cross(r.post.l));
            let lever = r.q * (corners()[r.corner] - ARM);
            let acceleration =
                r.n.dot(r.gravity + alpha.cross(lever) + w.cross(w.cross(lever)));
            if acceleration >= -0.5 {
                continue;
            }
            let ticks = (r.arrival / 0.0005).ceil() as u64;
            let mut states = Vec::new();
            let mut counts = Vec::new();
            let mut saw = false;
            let mut tried = Vec::new();
            for enabled in [false, true] {
                let xml = authored(&i, 8, enabled, false);
                let (mut sim, ids) = load(&xml, false)?;
                let mut mask = Vec::new();
                for _ in 0..ticks {
                    sim.step_exact(1)?;
                    if let Some(log) = last_box_timing(&sim).filter(|l| l.sim_time == sim.elapsed())
                    {
                        assert!(
                            log.interval.rigid_point_impacts.is_empty(),
                            "negative departure projected on {id}/{mode}"
                        );
                        if let Some(list) = &log.timing.point_attempts {
                            for (at, d) in list {
                                if *d
                                    == Some(gz_physics_multifidelity::RigidPointDecline::Departure)
                                {
                                    saw = true;
                                }
                                tried.push(
                                    json!({"cut_ns":at,"decline":d.map(|v|format!("{v:?}"))}),
                                );
                            }
                        }
                        mask.push((log.interval.native_steps, log.interval.clone()));
                    }
                }
                states.push(state(backend(&sim), ids)?);
                counts.push(mask);
            }
            assert_eq!(counts[0], counts[1]);
            assert!(states[0] == states[1], "changed declined state");
            rows.push(json!({"id":id,"mode":MODES[mode],"outward_acceleration":acceleration,"arrival_ns":(r.arrival*1e9).round() as u64,"tried":tried,"declined_departure":saw,"same_state":true,"same_interval":true}));
            if rows.len() == 96 {
                return write_json(&path.join("refusals.json"), &json!({"withdrawn":rows}));
            }
        }
    }
    Err("insufficient immediate nondeparture witnesses".into())
}
