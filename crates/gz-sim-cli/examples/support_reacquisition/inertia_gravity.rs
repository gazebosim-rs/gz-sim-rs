// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reference accelerating COM translation independently from the rotating inertia axes.
use super::*;
#[path = "inertia_friction.rs"]
pub(crate) mod friction;
const GRIDS: [u32; 3] = [4, 8, 16];
fn fixture(id: usize, field: usize) -> Physical {
    let mut r = Physical::new(id);
    r.gravity = 9.81
        * match field {
            0 => -DVec3::Z,
            1 => r.pad_q * DVec3::new(0.48, -0.31, -0.84).normalize(),
            _ => r.pad_q * DVec3::new(-0.52, 0.8, -0.28).normalize(),
        };
    r
}
fn configured(r: &Physical, budget: u32, ballistic: bool, compact: bool) -> (Profile, String) {
    let (p, mut text) = xml(r, budget, true);
    if ballistic {
        text=text.replace("</integrate_free_rotation>","</integrate_free_rotation><integrate_ballistic_translation>true</integrate_ballistic_translation>");
    }
    if compact {
        text = text.replace(
            "</detect_separated_crossings>",
            "</detect_separated_crossings><compact_impact_budget>true</compact_impact_budget>",
        );
    }
    (p, text)
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;

    let mut rows = vec![];
    let mut excluded = std::collections::BTreeMap::new();
    let mut featured = None;
    for id in (0..192).chain(960..1152) {
        for field in 0..3 {
            let r = fixture(id, field);
            let cert = r.certify();
            if let Some(reason) = cert.reason {
                *excluded.entry(reason).or_insert(0usize) += 1;
                continue;
            }
            let mut trials = vec![];
            for n in GRIDS {
                for (on, compact) in [(false, false), (true, false), (true, true)] {
                    let (p, text) = configured(&r, n, on, compact);
                    let (v, _, _) = scene::run(&r, false, p, &text)?;
                    let f = v["first_positive_end_ns"].as_u64();
                    let calls = v["native_calls"].as_u64().unwrap();
                    let spans=v["ticks"].as_array().unwrap().iter().map(|t|json!({"tick":t["tick"],"calls":t["calls"],"ballistic":t["ballistic"],"guard":t["guarded"],"end_ns":t["native_end_ns"],"load":t["native_normal_ns"],"compact":t["compact"]})).collect::<Vec<_>>();
                    let check = if on && n == 8 {
                        Some(check_flights(&r, &text)?)
                    } else {
                        None
                    };
                    trials.push(json!({"budget":n,"ballistic":on,"compact":compact,"native_calls":calls,"first_ns":f,"early_ns":v["preimpact_measured_ns"],"metrics":v["metrics"],"spans":spans,"flight_check":check}));
                }
            }
            if featured.is_none()
                && id >= 960
                && field == 0
                && r.body.omega(r.q, r.pre.l).length() > 5.
            {
                featured = Some((id, field));
            }
            rows.push(json!({"id":id,"field":field,"gravity":r.gravity.to_array(),"arrival_ns":(r.arrival*1e9).round()as u64,"normal":r.n.to_array(),"j":r.j,"corner":r.corner,"certificate":cert,"trials":trials}));
        }
    }
    if let Some((id, field)) = featured {
        let r = fixture(id, field);
        for (name, on) in [("ordinary", false), ("ballistic", true)] {
            let (p, text) = configured(&r, 8, on, false);
            let (_, frames, _) = scene::run(&r, false, p, &text)?;
            separated::single_rigid_trace(path, name, &frames)?;
            fs::write(path.join(format!("{name}.sdf")), text)?;
        }
        separated::single_rigid_trace(path, "analytic", &r.trace())?;
        write_json(
            &path.join("comparison.json"),
            &json!({"single_body":true,"title":"Spinning free flight under gravity","lanes":[{"name":"Native gravity advection","label":"ordinary","file":"traces/ordinary.json"},{"name":"Ballistic COM in verified free flight","label":"ballistic","file":"traces/ballistic.json"},{"name":"Independent ballistic and Euler reference","label":"analytic","file":"traces/analytic.json","analytical":true}]}),
        )?;
    }
    write_json(
        &path.join("study.json"),
        &json!({"cases":rows,"declines":excluded,"view":featured}),
    )
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let data: Value = serde_json::from_slice(&fs::read(
        path.parent()
            .ok_or("need timings subdirectory")?
            .join("study.json"),
    )?)?;
    let mut rows = vec![];
    for c in data["cases"].as_array().unwrap() {
        let id = c["id"].as_u64().unwrap() as usize;
        let field = c["field"].as_u64().unwrap() as usize;
        let r = fixture(id, field);
        let mut controls = vec![];
        for (on, compact) in [(false, false), (true, false), (true, true)] {
            let (_, text) = configured(&r, 8, on, compact);
            let (mut s, ids) = sdf_features::load_finite_box(&text, false)?;
            s.step_exact(8)?;
            let end = state(backend(&s), ids)?;
            clocked(&text, 8, end)?;
            controls.push((on, compact, text, end));
        }
        let mut measurements: [Vec<Vec<u64>>; 3] = Default::default();
        for rep in 0..6 {
            for k in 0..3 {
                let i = (if rep % 2 == 0 { k } else { 2 - k } + id + rep) % 3;
                measurements[i].push(clocked(&controls[i].2, 8, controls[i].3)?);
            }
        }
        for ((on, compact, _, _), times) in controls.into_iter().zip(measurements) {
            rows.push(
                json!({"id":id,"field":field,"ballistic":on,"compact":compact,"steps_ns":times}),
            );
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":rows}))
}

/// Starting independently from every actual accepted boundary, check the
/// whole four-vertex flight and the COM integrated under the authored field.
fn check_flights(r: &Physical, text: &str) -> Result<Value> {
    let (mut sim, ids) = sdf_features::load_finite_box(text, false)?;
    let data = Arc::new(Mutex::new(Vec::new()));
    let shared = data.clone();
    set_refined_observer(&mut sim, move |_, s| {
        shared.lock().unwrap().push(gyro::Cut {
            ns: s.elapsed.as_nanos() as u64,
            foot: s.state(ids.foot).unwrap(),
            pad: s.state(ids.pad).unwrap(),
        });
    })?;
    let mut min = f64::INFINITY;
    let mut max_center = 0f64;
    let mut max_v = 0f64;
    let mut max_spin = 0f64;
    let mut count = 0;
    let com = |b: BodyState| DVec3::from(b.position) + DQuat::from_array(b.orientation) * ARM;
    for _ in 0..8 {
        let mut before = gyro::Cut::bodies(backend(&sim), ids, 0);
        sim.step_exact(1)?;
        let mut cuts = std::mem::take(&mut *data.lock().unwrap());
        if cuts.is_empty() {
            cuts.push(gyro::Cut::bodies(backend(&sim), ids, 500_000));
        }
        if let Some(flags) = last_box_flight(&sim)
            .filter(|g| g.sim_time == sim.elapsed())
            .and_then(|g| g.flight.ballistic_steps.as_ref())
        {
            assert_eq!(cuts.len(), flags.len());
            for (cut, &applied) in cuts.into_iter().zip(flags) {
                if applied {
                    let result = released::checked_accelerated(r.body, before, cut, r.gravity)?;
                    let lower = result["quadratic_lower_m"].as_f64().unwrap();
                    assert!(
                        lower > 0.,
                        "unverified ballistic {} {:?} {result}",
                        r.id,
                        r.gravity
                    );
                    min = min.min(lower);
                    let h = (cut.ns - before.ns) as f64 * 1e-9;
                    let v = DVec3::from(before.foot.velocity);
                    max_center = max_center.max(
                        com(cut.foot).distance(com(before.foot) + (v + r.gravity * (0.5 * h)) * h),
                    );
                    max_v = max_v.max(DVec3::from(cut.foot.velocity).distance(v + r.gravity * h));
                    max_spin = max_spin.max(result["end_spin_error_rad_s"].as_f64().unwrap());
                    count += 1;
                }
                before = cut;
            }
        }
    }
    assert!(count > 0 && max_center < 1e-12 && max_v < 1e-12);
    Ok(
        json!({"spans":count,"min_positive_gap_m":min,"max_center_error_m":max_center,"max_speed_error_m_s":max_v,"max_spin_error_rad_s":max_spin}),
    )
}
