// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify the measured departure of a spinning foot and its subsequent unconstrained motion.
use super::*;
use gz_sim_scene::primary_contact::{last_box_flight, set_refined_observer};
use std::sync::{Arc, Mutex};
pub(super) fn configured(r: &Physical, n: u32, release: bool) -> (Profile, String) {
    let (p, text) = pulse::xml(r, n, "measured");
    if release {
        (p,text.replace("</integrate_free_rotation>",
        "</integrate_free_rotation><integrate_released_rotation>true</integrate_released_rotation>"))
    } else {
        (p, text)
    }
}
pub(super) fn oracle(b: ReferenceBody, cut: gyro::Cut, end: gyro::Cut) -> Result<Value> {
    oracle_grid(b, cut, end, 1e-6, DVec3::ZERO)
}
pub(super) fn checked_departure(b: ReferenceBody, cut: gyro::Cut, end: gyro::Cut) -> Result<Value> {
    checked_accelerated(b, cut, end, DVec3::ZERO)
}
pub(super) fn checked_accelerated(
    b: ReferenceBody,
    cut: gyro::Cut,
    end: gyro::Cut,
    gravity: DVec3,
) -> Result<Value> {
    let mut result = Value::Null;
    for divisions in [1_u32, 4, 16, 64] {
        result = oracle_grid(b, cut, end, 1e-6 / f64::from(divisions), gravity)?;
        if result["quadratic_lower_m"].as_f64().unwrap() > 0. {
            break;
        }
    }
    Ok(result)
}
fn oracle_grid(
    b: ReferenceBody,
    cut: gyro::Cut,
    end: gyro::Cut,
    max_step: f64,
    gravity: DVec3,
) -> Result<Value> {
    let dt = (end.ns - cut.ns) as f64 * 1e-9;
    let q0 = DQuat::from_array(cut.foot.orientation);
    let mut q = q0;
    let mut w = q.conjugate() * DVec3::from(cut.foot.angular_velocity);
    let n = DQuat::from_array(cut.pad.orientation) * DVec3::Z;
    let central =
        DVec3::from(cut.foot.position) + q * ARM - DVec3::from(cut.pad.position) - n * PAD_HALF[2];
    let v = DVec3::from(cut.foot.velocity) - DVec3::from(cut.pad.velocity);
    let arms = corners().map(|p| p - ARM);
    let gap = |q: DQuat, t: f64, r: DVec3| (central + (v + 0.5 * t * gravity) * t + q * r).dot(n);
    let l = b.tensor * w;
    let omega = (w.dot(l) / b.lower).sqrt().min(l.length() / b.lower);
    let ratio = (b.upper - b.lower) / b.lower;
    let alpha = 0.5 * ratio * omega * omega;
    let jerk = (1. + ratio) * omega * alpha + 3. * omega * alpha + omega.powi(3);
    let count = (dt / max_step).ceil().max(1.) as usize;
    let h = dt / count as f64;
    let mut minimum = f64::INFINITY;
    let mut bounded = f64::INFINITY;
    for i in 0..count {
        let t = i as f64 * h;
        let (next, new_w) = advance_euler(b, q, w, h);
        for &r in &arms {
            let z = gap(q, t, r);
            let u = (v + t * gravity + (q * w).cross(q * r)).dot(n);
            let a =
                (gravity + b.accel(q, q * w).cross(q * r) + (q * w).cross((q * w).cross(q * r)))
                    .dot(n);
            // For every t in [0,h], the Taylor remainder is at least -J*h*t²/6.
            // Its t² dependence preserves an exactly attained start minimum on a
            // separating corner with a sub-picometer initial clearance.
            let a = a - jerk * r.length() * h / 3.;
            let at = if a > 0. {
                (-u / a).clamp(0., h)
            } else if u + 0.5 * a * h < 0. {
                h
            } else {
                0.
            };
            let low = z + at * (u + 0.5 * a * at);
            bounded = bounded.min(low);
            minimum = minimum.min(z).min(gap(next, t + h, r));
        }
        q = next;
        w = new_w;
    }
    let native_w = DVec3::from(end.foot.angular_velocity);
    let actual_q = DQuat::from_array(end.foot.orientation);
    Ok(
        json!({"rk_steps":count,"sample_min_m":minimum,"quadratic_lower_m":bounded,
        "end_attitude_error_rad":angular_distance(actual_q,q),"end_spin_error_rad_s":native_w.distance(q*w),
        "angular_momentum_drift_nms":b.momentum(actual_q,native_w).distance(q0*l)}),
    )
}

fn release_steps(r: &Physical, text: &str) -> Result<Vec<Value>> {
    let (mut sim, ids) = sdf_features::load_finite_box(text, false)?;
    let samples = Arc::new(Mutex::new(Vec::new()));
    let shared = samples.clone();
    set_refined_observer(&mut sim, move |_, s| {
        let cut = gyro::Cut {
            foot: s.state(ids.foot).unwrap(),
            pad: s.state(ids.pad).unwrap(),
            ns: s.elapsed.as_nanos() as u64,
        };
        shared.lock().unwrap().push(cut);
    })?;
    let mut out = vec![];
    for tick in 0..8u64 {
        let mut start = gyro::Cut::bodies(backend(&sim), ids, 0);
        sim.step_exact(1)?;
        let mut cuts = std::mem::take(&mut *samples.lock().unwrap());
        let Some(flight) = last_box_flight(&sim).filter(|f| f.sim_time == sim.elapsed()) else {
            continue;
        };
        let Some(released) = flight.flight.released.as_ref() else {
            continue;
        };
        if cuts.is_empty() {
            cuts.push(gyro::Cut::bodies(backend(&sim), ids, 500_000));
        }
        assert_eq!(cuts.len(), released.len());
        for (i, (cut, step)) in cuts.into_iter().zip(released).enumerate() {
            let matching = if step.integrated {
                let check = oracle(r.body, start, cut)?;
                let min = check["sample_min_m"].as_f64().unwrap();
                assert!(
                    min > 0. && min + 2e-12 >= step.minimum_gap_pm.unwrap() as f64 * 1e-12,
                    "case {}: {step:?} {check}",
                    r.id
                );
                assert!(
                    check["quadratic_lower_m"].as_f64().unwrap() > 0.,
                    "independent departure inconclusive on case {} {check}",
                    r.id
                );
                Some(check)
            } else {
                None
            };
            assert_eq!(flight.flight.guarded[i], step.integrated);
            out.push(json!({"tick":tick+1,"begin_ns":start.ns,"end_ns":cut.ns,"integrated":step.integrated,
                "native_subdivisions":flight.flight.rotation_slices.as_ref().unwrap()[i],"corner_spans":step.corner_spans,
                "min_gap_pm":step.minimum_gap_pm,"independent":matching}));
            start = cut;
        }
    }
    Ok(out)
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut cases = vec![];
    for id in 0..2 * CASES {
        let r = Physical::new(id);
        if r.certify().reason.is_some() {
            continue;
        }
        let mut trials = vec![];
        for calls in [4, 8, 16] {
            for release in [false, true] {
                let (p, text) = configured(&r, calls, release);
                let (row, frames, _) = scene::run(&r, false, p, &text)?;
                let ledger = pulse::ledger(&r, &text)?;
                let follow = if release {
                    release_steps(&r, &text)?
                } else {
                    vec![]
                };
                if id == 153 && calls == 8 {
                    let label = if release { "released" } else { "baseline" };
                    separated::single_rigid_trace(path, label, &frames)?;
                    fs::write(path.join(format!("{label}.sdf")), text)?;
                }
                trials.push(json!({"release":release,"calls":calls,"native_calls":row["native_calls"],"metrics":row["metrics"],
                    "first_ns":row["first_positive_end_ns"],"early_ns":row["preimpact_measured_ns"],"ledger":ledger,
                    "departure":follow}));
            }
        }
        if id == 153 {
            separated::single_rigid_trace(path, "analytic", &r.trace())?;
        }
        cases.push(json!({"id":id,"arrival_ns":(r.arrival*1e9).round()as u64,"trials":trials}));
    }
    write_json(
        &path.join("released.json"),
        &json!({"cases":cases,"view_id":153}),
    )?;
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":"Free rigid rotation after a measured lift-off","lanes":[
        {"label":"baseline","name":"Ordinary rotation after release","file":"traces/baseline.json"},
        {"label":"released","name":"Certified released rigid rotation","file":"traces/released.json"},
        {"label":"analytic","name":"Independent Euler flight and impact","file":"traces/analytic.json","analytical":true}]}),
    )
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for id in CASES..2 * CASES {
        let r = Physical::new(id);
        if r.certify().reason.is_some() {
            continue;
        }
        let mut fixtures = vec![];
        for release in [false, true] {
            let (p, text) = configured(&r, 8, release);
            let end = scene::run(&r, false, p, &text)?.2;
            scene::measure(&text, false, end)?;
            fixtures.push((release, text, end));
        }
        let mut times: [Vec<[u64; 8]>; 2] = Default::default();
        for rep in 0..16 {
            for pos in 0..2 {
                let i = (id + pos + rep) % 2;
                times[i].push(scene::measure(&fixtures[i].1, false, fixtures[i].2)?);
            }
        }
        for ((release, _, _), raw) in fixtures.into_iter().zip(times) {
            rows.push(json!({"id":id,"release":release,"impact_tick":(r.arrival*2000.).ceil()as u64,"tick_ns":raw}));
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":rows}))
}
