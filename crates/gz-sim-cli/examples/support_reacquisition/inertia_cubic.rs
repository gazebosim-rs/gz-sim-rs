// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualification of conservative high-spin corner crossing times on regular SDF steps.
use super::*;
use gz_sim_scene::primary_contact::{last_box_timing, set_refined_observer};
use std::sync::{Arc, Mutex};

pub(super) fn configured(r: &Physical, calls: u32, cubic: bool) -> (Profile, String) {
    let (profile, mut text) = gyro::configured(r, calls, true);
    if cubic {
        text = text.replace(
            "</integrate_free_rotation>",
            "</integrate_free_rotation><refine_rotation_timing>true</refine_rotation_timing>",
        );
    }
    (profile, text)
}

fn verify_live(r: &Physical, text: &str) -> Result<Vec<Value>> {
    let (mut sim, ids) = sdf_features::load_finite_box(text, false)?;
    let cuts = Arc::new(Mutex::new(Vec::new()));
    let share = cuts.clone();
    set_refined_observer(&mut sim, move |_, s| {
        share.lock().unwrap().push(gyro::Cut {
            foot: s.state(ids.foot).unwrap(),
            pad: s.state(ids.pad).unwrap(),
            ns: s.elapsed.as_nanos() as u64,
        })
    })?;
    let mut out = vec![];
    for tick in 0..8 {
        let initial = gyro::Cut::bodies(backend(&sim), ids, 0);
        sim.step_exact(1)?;
        let measured = std::mem::take(&mut *cuts.lock().unwrap());
        let Some(interval) = last_box_timing(&sim).filter(|i| i.sim_time == sim.elapsed()) else {
            continue;
        };
        let t = &interval.timing;
        let mut witnesses = vec![];
        for w in &t.crossings {
            let cut = if w.measured_ns == 0 {
                initial
            } else {
                *measured
                    .iter()
                    .find(|c| c.ns == w.measured_ns)
                    .ok_or("missing measured native cut")?
            };
            let h = (w.latest_ns - w.measured_ns) as f64 * 1e-9 + 2e-9;
            let Some((corner, sec)) = gyro::Ballistic::new(r.body, cut).crossing(h)? else {
                return Err(format!("case {}: no Euler crossing in {w:?}", r.id).into());
            };
            let root = w.measured_ns as f64 + sec * 1e9;
            if usize::from(w.corner) != corner
                || root < w.earliest_ns as f64 - 0.001
                || root > w.latest_ns as f64 + 0.001
            {
                return Err(format!(
                    "case {}: measured independent root {corner}@{root} outside {w:?}",
                    r.id
                )
                .into());
            }
            witnesses.push(json!({"corner":corner,"cut_ns":w.measured_ns,"bracket_ns":[w.earliest_ns,w.latest_ns],"rk4_ns":root}));
        }
        out.push(json!({"tick":tick+1,"queries":t.cubic_queries,"uncertain_horizons":t.uncertain_horizons,
            "decline":t.decline.map(|x|format!("{x:?}")),"witnesses":witnesses}));
    }
    Ok(out)
}

pub(crate) fn write(path: &Path) -> Result<()> {
    adverse(path, false)?;
    fs::create_dir_all(path.join("traces"))?;
    let mut cases = vec![];
    for id in 0..CASES {
        let r = Physical::new(id);
        let proof = r.certify();
        if proof.reason.is_some() {
            continue;
        }
        let mut trials = vec![];
        for calls in [4, 8, 16, 32] {
            for cubic in [false, true] {
                let (p, xml) = configured(&r, calls, cubic);
                let (mut row, frames, _) = scene::run(&r, false, p, &xml)?;
                if cubic {
                    row["clocks"] = json!(verify_live(&r, &xml)?);
                }
                row["cubic"] = cubic.into();
                if id == 153 && calls == 8 {
                    let label = if cubic { "cubic" } else { "frozen" };
                    separated::single_rigid_trace(path, label, &frames)?;
                    fs::write(path.join(format!("{label}.sdf")), xml)?;
                }
                trials.push(row);
            }
        }
        if id == 153 {
            separated::single_rigid_trace(path, "analytic", &r.trace())?;
        }
        cases.push(json!({"id":id,"arrival_ns":(r.arrival*1e9).round()as u64,"reference_corner":r.corner,"trials":trials}));
    }
    write_json(
        &path.join("cubic.json"),
        &json!({"cases":cases,"view_id":153}),
    )?;
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":"Timing a spinning corner from live rigid-body state","lanes":[
        {"label":"frozen","name":"Frozen-spin timing envelope","file":"traces/frozen.json"},
        {"label":"cubic","name":"Cubic corner timing envelope","file":"traces/cubic.json"},
        {"label":"analytic","name":"Independent Euler flight and impact","file":"traces/analytic.json","analytical":true}]}),
    )
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    adverse(path, true)?;
    fs::create_dir_all(path)?;
    let mut cases = vec![];
    for id in 0..CASES {
        let r = Physical::new(id);
        if r.certify().reason.is_some() {
            continue;
        }
        let mut fixtures = vec![];
        for n in [4, 8, 16, 32] {
            for cubic in [false, true] {
                let (p, text) = configured(&r, n, cubic);
                let end = scene::run(&r, false, p, &text)?.2;
                scene::measure(&text, false, end)?;
                fixtures.push((n, cubic, text, end));
            }
        }
        let mut times: [Vec<[u64; 8]>; 8] = Default::default();
        for rep in 0..8 {
            for pos in 0..8 {
                let i = (pos + rep) % 8;
                let i = if id % 2 == 0 { i } else { 7 - i };
                times[i].push(scene::measure(&fixtures[i].2, false, fixtures[i].3)?);
            }
        }
        for ((n, cubic, _, _), raw) in fixtures.into_iter().zip(times) {
            cases.push(json!({"id":id,"calls":n,"cubic":cubic,"primary_tick":(r.arrival*2000.).ceil()as usize,"tick_ns":raw}));
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":cases}))
}

/// Construct known incoming intersections without assuming any particular post-impact
/// solution for nearly coplanar, exactly simultaneous or grazing contacts.
pub(super) fn adverse_fixture(family: usize, phase: usize) -> Physical {
    let mut r = Physical::new(8 * phase + family);
    let (roll, pitch, speed) = match family {
        0 => (0., 0., 0.9),        // coplanar face
        1 => (0., 0.08, 0.9),      // tied edge
        2 => (1e-13, 0.08, 0.9),   // order below roundoff enclosure
        3 => (1e-8, 0.08, 0.9),    // adjacent arrival a few nanoseconds later
        4 => (0.14, -0.06, 0.003), // closing below minimum certified speed
        5 => (0.14, -0.06, 1.3),   // well-separated positive control
        _ => unreachable!(),
    };
    r.q = r.pad_q * DQuat::from_rotation_x(roll) * DQuat::from_rotation_y(pitch);
    r.arrival = [
        1.2e-6, 31.25e-6, 62.5e-6, 125e-6, 247.13e-6, 381.62e-6, 437.5e-6, 499.998e-6,
    ][phase];
    let arms = corners().map(|v| r.q * (v - ARM));
    let ix = (0..4)
        .min_by(|&i, &j| arms[i].dot(r.n).total_cmp(&arms[j].dot(r.n)))
        .unwrap();
    r.corner = ix;
    r.closing = speed;
    r.at = r.pad_v * r.arrival - r.n * arms[ix].dot(r.n);
    r.pre = Branch {
        v: r.pad_v - r.n * speed,
        l: DVec3::ZERO,
    };
    r
}

fn nominated(r: &Physical, text: &str, calls: u32, cubic: bool) -> Result<(Value, u64)> {
    let (mut sim, ids) = sdf_features::load_finite_box(text, false)?;
    let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
    let e = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
    let pair = [(ids.foot, ids.pad)];
    let a = gz_physics_multifidelity::BoxContactActions {
        fresh_features: &pair,
        prospective_timing: &pair,
        guard_separated_flight: &pair,
        integrate_free_rotation: &pair,
        refine_rotation_timing: if cubic { &pair } else { &[] },
        ..Default::default()
    };
    let dt = Duration::from_micros(500);
    let now = Instant::now();
    let out = e.advance_box_contacts(
        dt,
        calls,
        gz_physics_multifidelity::ContactSolverIterations {
            temporal: 4,
            pgs: Some(1),
            relaxation: Some(16),
        },
        a,
        None,
    )?;
    let ns = now.elapsed().as_nanos() as u64;
    let clock = out.timing.unwrap();
    assert_eq!(out.interval.native_steps, calls);
    assert_eq!(clock.durations.iter().sum::<Duration>(), dt);
    assert!(clock.durations.iter().all(|d| !d.is_zero()));
    let first = clock.crossings.first().map(
        |w| json!({"corner":w.corner,"cut_ns":w.measured_ns,"lo":w.earliest_ns,"hi":w.latest_ns}),
    );
    let row = json!({"decline":clock.decline.map(|r|format!("{r:?}")),"decline_after":clock.decline_after,
        "queries":clock.cubic_queries,"uncertain_horizons":clock.uncertain_horizons,"witnesses":clock.crossings.len(),"first":first,
        "calls":calls,"durations_ns":clock.durations.iter().map(|x|x.as_nanos()as u64).collect::<Vec<_>>(),
        "load_ns":out.interval.pairs.iter().map(|p|p.normal_impulse).sum::<f64>(), "end":e.state(ids.foot).map(Body::from),
        "reference_arrival_ns":r.arrival*1e9});
    Ok((row, ns))
}

fn adverse(path: &Path, cost: bool) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut data = vec![];
    for family in 0..6 {
        for phase in 0..8 {
            let r = adverse_fixture(family, phase);
            for calls in [4, 8, 16] {
                let mut fixtures = vec![];
                for cubic in [false, true] {
                    let (_, xml) = configured(&r, calls, cubic);
                    let (row, _) = nominated(&r, &xml, calls, cubic)?;
                    // An exact or roundoff-tied face/edge must never get an isolated-corner witness.
                    if family <= 2 || family == 4 {
                        assert_eq!(row["witnesses"], 0, "family {family} phase {phase} {row}");
                    }
                    fixtures.push((xml, row));
                }
                let mut elapsed: [Vec<u64>; 2] = Default::default();
                if cost {
                    for rep in 0..16 {
                        for pos in 0..2 {
                            let i = (pos + rep + phase) % 2;
                            let (again, ns) = nominated(&r, &fixtures[i].0, calls, i == 1)?;
                            assert_eq!(again, fixtures[i].1);
                            elapsed[i].push(ns);
                        }
                    }
                }
                for (i, ((_, row), samples)) in fixtures.into_iter().zip(elapsed).enumerate() {
                    data.push(json!({"family":(["face","edge","subroundoff","near_edge","grazing","isolated"][family]),
                        "phase":phase,"calls":calls,"cubic":i==1,"clock":row,"elapsed_ns":samples}));
                }
            }
        }
    }
    write_json(
        &path.join(if cost {
            "adverse_cost.json"
        } else {
            "adverse.json"
        }),
        &json!({"rows":data}),
    )
}
