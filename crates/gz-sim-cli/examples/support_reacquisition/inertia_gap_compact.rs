// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Attempt a bounded terminal response before resolving every individual impact.
use super::*;
use std::{hint::black_box, time::Instant};
const GOALS: [(f64, f64); 4] = [(1e-3, 8e-9), (1e-4, 8e-9), (1e-5, 8e-9), (1e-4, 128e-9)];
fn goal(radius: f64, release: f64) -> limit::Goal {
    limit::Goal {
        vector_radius: radius,
        release_s: release,
        time_s: 4e-6,
    }
}
fn v(x: &Value) -> DVec3 {
    DVec3::from_array(serde_json::from_value(x.clone()).unwrap())
}
fn sum(s: &Sequence, lower: Crossing, upper: Crossing) -> Option<Value> {
    let p = &s.limit["proof"];
    let frozen = &p["frozen"];
    if !frozen["impulses"].is_array() {
        return None;
    }
    let last = s.finite.last().copied().unwrap_or(lower);
    let base = last.after();
    let cut = Flight {
        v: v(&frozen["velocity"]),
        l: v(&frozen["momentum"]),
        ..base
    };
    let tail = [0, 1].map(|k| v(&frozen["impulses"][k]));
    let propagated = s
        .limit
        .get("propagated")
        .cloned()
        .unwrap_or_else(|| uncertainty::Error::default().data());
    let radius = [0, 1].map(|k| {
        num(&propagated["impulses_ns"][k])
            + s.limit["radii"][k].as_f64().unwrap_or_else(|| {
                num(&p["world_vector_radii"][k])
                    + v(&p["world_vector_centers"][k]).distance(tail[k])
                    + 2e-12
            })
    });
    let offset = s.records.last().map_or(0., |row| {
        num(&row["offset_s"]) + num(&row["offset_correction_s"])
    });
    let mut js = tail;
    for h in &s.finite {
        js[h.surface] += h.law.impulse;
    }
    let arms = [lower.corner, upper.corner].map(|k| vertices()[k].length());
    let hcap = num(&p["remaining_time_cap_s"]);
    let total = num(&p["total_impulse_bound_ns"]);
    Some(
        json!({"offset_s":offset,"remaining_time_s":hcap,"first_gap":p["prior_gap_m"],"first_time_cap_s":p["initial_time_cap_s"],"first_speed":p["incoming_speed"],"sum":js.map(|j|j.to_array()),"tail":tail.map(|j|j.to_array()),"radius":radius,"v":cut.v.to_array(),"l":cut.l.to_array(),"com":cut.com.to_array(),"q":cut.q.to_array(),"propagated":propagated,"tracked_impulses":s.limit["tracked_impulses"],"pre_l":base.l.to_array(),"pre_v":base.v.to_array(),"arms_m":arms,"impulse_cap_ns":total,"rho":p["normal_speed_pair_ratio_cap"],"minimum_outward":p["both_release_accel_floor"],"prior_strikes":s.finite.len(),"free_after_s":p["free_after_s"]}),
    )
}
fn num(v: &Value) -> f64 {
    v.as_f64().unwrap()
}
fn state(x: &Value) -> Flight {
    Flight {
        q: DQuat::from_array(serde_json::from_value(x["q"].clone()).unwrap()),
        com: v(&x["com"]),
        v: v(&x["v"]),
        l: v(&x["l"]),
    }
}
fn compare(i: &Impact, a: &Value, b: &Value, extra: f64) -> Value {
    let stop = |p: &Value| {
        num(&p["offset_s"]) + num(&p["propagated"]["time_s"]) + num(&p["remaining_time_s"])
    };
    let after = stop(a).max(stop(b)) + extra;
    let x = |p: &Value| advance(&i.r, state(p), after - num(&p["offset_s"]));
    let xx = x(a);
    let yy = x(b);
    let bound = |p: &Value| {
        let e = &p["propagated"];
        let t = num(&p["remaining_time_s"]) + num(&e["time_s"]);
        let dt = after - num(&p["offset_s"]);
        let s = num(&p["impulse_cap_ns"]);
        let rr = [0, 1].map(|k| num(&p["arms_m"][k]));
        let radius = [0, 1].map(|k| num(&p["radius"][k]));
        let maxr = rr[0].max(rr[1]);
        let r = &i.r;
        let w = (v(&p["pre_l"]).length() + num(&e["momentum_nms"]) + maxr * s) / r.body.lower;
        let speed =
            num(&e["velocity_m_s"]) + v(&p["pre_v"]).length() + r.gravity.length() * t + s / M;
        let dv = num(&e["velocity_m_s"]) + radius.iter().sum::<f64>() / M + r.gravity.length() * t;
        let dl = num(&e["momentum_nms"])
            + rr[0] * radius[0]
            + rr[1] * radius[1]
            + maxr * (num(&e["angle_rad"]) + w * t) * s;
        let dq =
            (2.5 * w * dt).exp() * (num(&e["angle_rad"]) + 2. * w * t + dl / r.body.lower * dt);
        (
            num(&e["position_m"]) + 2. * speed * t + dv * dt,
            dl / r.body.lower + 8. * w * dq,
            dl,
            dv,
        )
    };
    let ba = bound(a);
    let bb = bound(b);
    let diffs = [0, 1].map(|k| v(&a["sum"][k]).distance(v(&b["sum"][k])));
    json!({"endpoint_s":after,"in_both_free_windows":after<=num(&a["offset_s"])-num(&a["propagated"]["time_s"])+num(&a["free_after_s"]) && after<=num(&b["offset_s"])-num(&b["propagated"]["time_s"])+num(&b["free_after_s"]),"impulse_difference_ns":diffs,"impulse_combined_radii_ns":([0,1].map(|k|num(&a["radius"][k])+num(&b["radius"][k]))),"momentum_difference_nms":xx.l.distance(yy.l),"momentum_bound_nms":ba.2+bb.2,"velocity_difference_m_s":xx.v.distance(yy.v),"velocity_bound_m_s":ba.3+bb.3,"com_difference_m":xx.com.distance(yy.com),"com_bound_m":ba.0+bb.0,"spin_difference_rad_s":i.r.body.omega(xx.q,xx.l).distance(i.r.body.omega(yy.q,yy.l)),"spin_bound_rad_s":ba.1+bb.1})
}
fn original(
    i: &Impact,
    g: &Gate,
    upper: Crossing,
    lower: Crossing,
    mu: [f64; 2],
    reference: &Sequence,
) -> Value {
    let mut stored = vec![];
    let mut complete = sum(reference, lower, upper);
    if let Some(ref mut p) = complete {
        p["free_after_s"] = reference.limit["extended"]["free_after_s"].clone();
    }
    for (radius, free) in GOALS {
        let start = Instant::now();
        let s = evaluate(i, g, upper, lower, mu, Some(goal(radius, free)));
        let elapsed = start.elapsed().as_nanos() as u64;
        let candidate = sum(&s, lower, upper);
        let matched = candidate
            .as_ref()
            .zip(complete.as_ref())
            .map(|(a, b)| compare(i, a, b, 1e-9));
        stored.push(json!({"radius_goal":radius,"release_goal_s":free,"work_ns":elapsed,"finite_events":s.finite.len(),"reason":s.reason,"candidate":candidate,"compare":matched}));
    }
    json!({"reference":complete,"goals":stored})
}
fn benchmark(i: &Impact, g: &Gate, upper: Crossing, lower: Crossing, mu: [f64; 2]) -> Value {
    let mut times = [vec![], vec![]];
    for rep in 0..9 {
        for n in 0..2 {
            let solver = (n + rep) % 2;
            let fast = (solver == 1).then_some(goal(1e-4, 8e-9));
            let start = Instant::now();
            let s = black_box(evaluate(black_box(i), g, upper, lower, mu, fast));
            let ns = start.elapsed().as_nanos() as u64;
            black_box(s.reason);
            if rep > 0 {
                times[solver].push(ns);
            }
        }
    }
    json!({"full_ns":times[0],"early_ns":times[1]})
}
pub(super) fn heldout_inputs() -> Vec<Impact> {
    let mut groups = [0; 4];
    let mut out = Vec::new();
    for id in 2600..3400 {
        let mode = (id / 4) % 4;
        if groups[mode] >= 24 {
            continue;
        }
        let i = manufactured(id, mode);
        if i.unique_margin > 1e-8
            && i.loss > 0.
            && i.r.closing > 0.05
            && i.r.certify().reason.is_none()
        {
            groups[mode] += 1;
            out.push(i);
            if groups == [24; 4] {
                break;
            }
        }
    }
    assert_eq!(groups, [24; 4]);
    out
}
pub(crate) fn write(path: &Path, cost: bool, heldout: bool) -> crate::Result<()> {
    fs::create_dir_all(path)?;
    let mut donors = vec![];
    let inputs = if heldout { heldout_inputs() } else { inputs() };
    let gates: &[u64] = if heldout {
        &[37000, 430000, 910000, 1350000]
    } else {
        &[20000, 750000, 1100000]
    };
    let materials = if heldout {
        [0.1, 0.45, 0.75]
    } else {
        [0., 0.35, 0.8]
    };
    let primary_inputs = inputs.len();
    let mut ordered_upper = 0;
    let mut attempts = 0;
    for i in inputs {
        let mut cases = vec![];
        for &delay in gates {
            let g = gate(&i, delay as f64 * 1e-9);
            for gate_mu in materials {
                attempts += 1;
                let mu = [i.mu, i.mu.min(gate_mu)];
                let Ok(u) = upper(&i, &g, mu[1]) else {
                    continue;
                };
                ordered_upper += 1;
                let Some(l) = follow(&i, faces(&i.r, &g), mu, u, WINDOW)
                    .next
                    .filter(|h| h.surface == 0)
                else {
                    continue;
                };
                let reference = sequence(&i, &g, u, l, mu);
                if !cost
                    && !heldout
                    && i.r.id == 1160
                    && i.mode == 3
                    && delay == 1100000
                    && gate_mu == 0.8
                {
                    let early = evaluate(&i, &g, u, l, mu, Some(goal(1e-4, 128e-9)));
                    terminal::compact_view(path, &i, &g, gate_mu, l, (&reference, &early))?;
                }
                let result = if cost {
                    benchmark(&i, &g, u, l, mu)
                } else {
                    original(&i, &g, u, l, mu, &reference)
                };
                cases.push(json!({"delay_ns":delay,"gate_mu":gate_mu,"reference_kind":reference.limit["kind"],"full_events":reference.finite.len(),"result":result}));
            }
        }
        if !cases.is_empty() {
            donors.push(json!({"id":i.r.id,"mode":MODES[i.mode],"cases":cases}));
        }
    }
    write_json(
        &path.join(if cost {
            "cost.json"
        } else if heldout {
            "heldout.json"
        } else {
            "compact.json"
        }),
        &json!({"primary_inputs":primary_inputs,"attempts":attempts,"ordered_upper":ordered_upper,"donors":donors}),
    )
}
