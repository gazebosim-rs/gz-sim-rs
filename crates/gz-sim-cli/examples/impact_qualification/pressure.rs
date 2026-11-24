// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent readback of frozen line pressures on matched secondary starts.
use super::transfer::{fixture, matched, vector};
use super::*;
use glam::DVec3;
type Rows = BTreeMap<(String, String, String), Series>;
fn bump(c: &mut BTreeMap<String, usize>, s: &str) {
    *c.entry(s.to_owned()).or_default() += 1
}
fn samples(map: Rows) -> Value {
    json!(map.into_iter().map(|((class,method,metric),series)|json!({"class":class,"method":method,"metric":metric,"statistics":series.stats()})).collect::<Vec<_>>())
}
fn close(x: f64, y: f64) -> Result<()> {
    if !x.is_finite() || !y.is_finite() || (x - y).abs() > 1e-10 * (1. + x.abs().max(y.abs())) {
        return Err(format!("line reconstruction {x}, expected {y}").into());
    }
    Ok(())
}
pub(super) fn wrench(dir: &Path, precise: &Path, registered: Option<&Path>) -> Result<()> {
    let mut rows = Rows::new();
    let mut counts = BTreeMap::new();
    let mut worst = None;
    for k in 0..1250 {
        if !dir.join("cases").join(format!("case-{k:04}.json")).exists() {
            continue;
        }
        let d = fixture(dir, k)?;
        let b = fixture(precise, k)?;
        let fresh = registered.map(|dir| fixture(dir,k)).transpose()?;
        if d["format"] != "finite-line-native-wrench-v1"
            || d["result"]["fixture"] != b["result"]["fixture"]
        {
            return Err("invalid line wrench input".into());
        }
        bump(&mut counts, "fixtures");
        for t in array(&d["result"]["trials"])? {
            let key = 3 * k + uint(&t["stroke"])? as usize - 1;
            let r = &t["response"];
            let src = &matched(&b, t)?["response"];
            if r["arc_end"] != src["arc_end"] || r["geometry"] != src["geometry"] {
                return Err("secondary motion changed".into());
            }
            if r["segment"].is_null() {
                continue;
            }
            let s = &r["segment"];
            if s["points"] != src["segment"]["points_m"] || s["normal"] != src["segment"]["normal"]
            {
                return Err("secondary line changed".into());
            }
            let h = fresh.as_ref().map(|a| matched(a,t)).transpose()?;
            let endpoint = if let Some(registered)=h{
                let a=&registered["response"];
                if a["arc_end"]!=r["arc_end"] || a["segment"]!=src["segment"] || a["registered"][1]["projected"].is_null(){return Err("wrench reference mismatch".into())}
                Some(vector(&a["registered"][1]["projected"]["j_ns"])?)
            } else if !src["impact"].is_null(){Some(vector(&src["impact"]["impulse_ns"])?)}else{None};
            let e = &r["arc_end"];
            let c = vector(&e["com_m"])?;
            let p = vector(&s["points"][0])?;
            let q = vector(&s["points"][1])?;
            let n = vector(&s["normal"])?;
            let axis = (q - p).normalize();
            let y = n.cross(axis).normalize();
            let velocity = vector(&e["velocity_m_s"])?;
            let spin = vector(&e["spin_rad_s"])?;
            let class = if src["impact"].is_null() {
                "ambiguous"
            } else {
                "certified"
            };
            bump(&mut counts, class);
            let runs = array(&r["runs"])?;
            if runs.len() != 9 {
                return Err("missing native wrench rows".into());
            }
            for (i, entry) in runs.iter().enumerate() {
                let dt = [1, 128, 2048][i / 3];
                let passes = [1, 8, 32][i % 3];
                let method = format!("{dt}ns_r{passes}");
                let raw = &entry["raw"];
                let f = &entry["frozen_line"];
                if entry["duration_ns"] != dt
                    || raw["duration_ns"] != dt
                    || raw["relaxation"] != passes
                    || entry["relaxation"] != passes
                {
                    return Err("incorrect native acquisition".into());
                }
                let mut metric = |m: &str, value: f64| {
                    for g in ["all", class] {
                        rows.entry((g.into(), method.clone(), m.into()))
                            .or_default()
                            .push(value, key)
                    }
                };
                let j = vector(&f["j_ns"])?;
                let torque = vector(&f["moment_nms"])?;
                let v = vector(&raw["after_v"])? + DVec3::Z * (9.81 * f64::from(dt) * 1e-9);
                let w = vector(&raw["after_w"])?;
                close(j.distance(vector(&raw["j_ns"])?), 0.)?;
                metric(
                    "native_linear_momentum_residual_ns",
                    (2. * (v - velocity)).distance(j),
                );
                close((0.05 * (w - spin)).distance(torque), 0.)?;
                close(vector(&f["post_v"])?.distance(v), 0.)?;
                close(vector(&f["post_w"])?.distance(w), 0.)?;
                let residual = torque - (p - c).cross(j);
                let couple = axis.dot(residual);
                close(couple, num(&f["couple_nms"])?)?;
                let j1 = residual.cross(axis) / (q - p).length();
                let j0 = j - j1;
                let u = [p, q].map(|x| v + w.cross(x - c));
                let pn = [j0.dot(n), j1.dot(n)];
                let vn = u.map(|x| x.dot(n));
                for (i, load) in [j0, j1].iter().enumerate() {
                    close(pn[i], num(&f["normal_ns"][i])?)?;
                    close(u[i].dot(n), num(&f["normal_speed_m_s"][i])?)?;
                    close(load.dot(y), num(&f["transverse_ns"][i])?)?;
                }
                let nw = pn[0] * vn[0] + pn[1] * vn[1];
                close(nw, num(&f["normal_work_j"])?)?;
                let tang = u.map(|x| x - n * x.dot(n));
                let work = -j0.dot(tang[0]) - j1.dot(tang[1]);
                close(work, num(&f["friction_after_j"])?)?;
                let mut excess = 0.;
                if pn.iter().all(|v| *v >= 0.) {
                    let limits = [
                        0.5 * (pn[0] * tang[0] + pn[1] * tang[1]).length(),
                        0.5 * (pn[0] * tang[0].length() + pn[1] * tang[1].length()),
                    ];
                    for (i, l) in limits.iter().enumerate() {
                        close(*l, num(&f["coulomb_dissipation_j"][i])?)?
                    }
                    excess = 0.0_f64.max(limits[0] - work).max(work - limits[1]);
                    metric("friction_work_infeasibility_j", excess);
                } else if !f["coulomb_dissipation_j"].is_null() {
                    return Err("tensile pressure used for dissipation bound".into());
                }
                let tension = 0.0_f64.max(-pn[0]).max(-pn[1]);
                let inward = 0.0_f64.max(-vn[0]).max(-vn[1]);
                metric("tension_ns", tension);
                metric("free_couple_nms", couple.abs());
                metric("post_closing_speed_m_s", inward);
                metric("normal_complementarity_abs_j", nw.abs());
                metric("additional_tangent_capacity_ns", num(&f["cone_slack_ns"])?);
                metric("post_frictional_energy_injection_j", (-work).max(0.));
                if let Some(endpoint)=endpoint {
                    let err = j.distance(endpoint);
                    metric("impulse_error_ns", err);
                }
                if dt == 1
                    && passes == 32
                    && worst.is_none_or(|(v, _)| excess > v)
                {
                    worst = Some((excess, key))
                }
                if dt == 1
                    && (tension > 1e-6
                        || couple.abs() > 1e-6
                        || inward > 1e-5
                        || nw.abs() > 1e-6
                        || excess > 1e-6
                        || num(&f["cone_slack_ns"])? > 1e-6)
                {
                    bump(
                        &mut counts,
                        &format!("above_reporting_threshold_{method}_{class}"),
                    );
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"largest_1ns_32pass_friction_work_deviation_j_and_case":worst,"rows":samples(rows)}),
    )
}
pub(super) fn closure(dir: &Path, precise: &Path) -> Result<()> {
    let mut counts = BTreeMap::new();
    let mut rows = Rows::new();
    let mut most = None;
    let mut first = None;
    for k in 0..1250 {
        if !dir.join("cases").join(format!("case-{k:04}.json")).exists() {
            continue;
        }
        let d = fixture(dir, k)?;
        let b = fixture(precise, k)?;
        if d["format"] != "continuous-line-closure-v1"
            || d["result"]["fixture"] != b["result"]["fixture"]
        {
            return Err("invalid closure input".into());
        }
        bump(&mut counts, "fixtures");
        for t in array(&d["result"]["trials"])? {
            let k = 3 * k + uint(&t["stroke"])? as usize - 1;
            let state = &t["response"];
            let r = &state["pressure"];
            let src = &matched(&b, t)?["response"];
            if state["geometry"] != src["geometry"] || state["arc_end"] != src["arc_end"] {
                return Err("separate closure experiment changed incoming geometry".into());
            }
            if r.is_null() {
                continue;
            }
            if r["pressure_bound"] != src["impact"]["loaded_end"] {
                return Err("simple pressure bound changed".into());
            }
            let old = !r["pressure_bound"].is_null();
            let class = if old {
                "previously_certified"
            } else {
                "ambiguous"
            };
            let candidates = array(&r["candidates"])?;
            if candidates.len() != 2 {
                return Err("missing alternative endpoint".into());
            }
            let open = candidates.iter().position(|s| s["status"] == "opens_other");
            let two_required = candidates
                .iter()
                .all(|c| c["status"] == "cannot_stop_other" || c["status"] == "penetrates_other");
            if two_required {
                bump(&mut counts, "both_normals_necessary")
            }
            if old {
                let id = uint(&r["pressure_bound"])? as usize;
                if Some(id) != open {
                    return Err("old pressure certificate lost its opening solution".into());
                }
                close(
                    vector(&src["impact"]["impulse_ns"])?
                        .distance(vector(&candidates[id]["j_ns"])?),
                    0.,
                )?;
            }
            bump(&mut counts, class);
            let re = array(&r["closure"])?;
            let mut visited = 0;
            let mut excluded = false;
            for (i, q) in re.iter().enumerate() {
                let budget = [1, 32, 512, 8192][i];
                let cost = uint(&q["visited"])?;
                if q["budget"] != budget || cost > budget || cost < visited || excluded {
                    return Err("non-monotone closure search".into());
                }
                visited = cost;
                excluded = q["excluded"] == true;
            }
            if excluded {
                let Some(i) = open else {
                    return Err("no opening law after excluding the closed line".into());
                };
                if !(candidates[1 - i]["status"] == "cannot_stop_other"
                    || candidates[1 - i]["status"] == "penetrates_other")
                {
                    return Err("unresolved alternative endpoint".into());
                }
                bump(&mut counts, &format!("excluded_{class}"));
                if !old {
                    first.get_or_insert(k);
                    if most.is_none_or(|(v, _)| visited > v) {
                        most = Some((visited, k))
                    }
                }
            }
            rows.entry((
                class.into(),
                if excluded { "excluded" } else { "unresolved" }.into(),
                "boxes".into(),
            ))
            .or_default()
            .push(visited as f64, k);
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"first_new_case":first,"largest_new_search_cost_and_case":most,"rows":samples(rows)}),
    )
}
pub(super) fn registered(dir: &Path, tight: &Path, closure: &Path) -> Result<()> {
    let mut counts = BTreeMap::new();
    let mut rows = Rows::new();
    for k in 0..1250 {
        let d = fixture(dir, k)?;
        let previous = fixture(tight, k)?;
        let independent = fixture(closure, k)?;
        if d["format"] != "finite-second-registered-work-v1"
            || d["result"]["fixture"] != previous["result"]["fixture"]
            || d["result"]["fixture"] != independent["result"]["fixture"]
        {
            return Err("invalid native work fixture".into());
        }
        bump(&mut counts, "fixtures");
        for t in array(&d["result"]["trials"])? {
            let key = 3 * k + uint(&t["stroke"])? as usize - 1;
            let r = &t["response"];
            let old = &matched(&previous, t)?["response"];
            let physical = &matched(&independent, t)?["response"];
            if r["arc_end"] != physical["arc_end"]
                || r["geometry"] != physical["geometry"]
                || [
                    "arc_end",
                    "segment",
                    "impact",
                    "velocity_before",
                    "geometry",
                ]
                .iter()
                .any(|s| r[*s] != old[*s])
            {
                return Err("registered work donor mismatch".into());
            }
            if r["segment"].is_null() {
                continue;
            }
            let p = &physical["pressure"];
            let candidates = array(&p["candidates"])?;
            let i = candidates
                .iter()
                .position(|c| c["status"] == "opens_other")
                .ok_or("no physical single site")?;
            let target = &candidates[i];
            let basic = !p["pressure_bound"].is_null();
            let group = if basic { "direct" } else { "distributed_work" };
            if !basic
                && array(&p["closure"])?.last().ok_or("missing work proof")?["excluded"] != true
            {
                return Err("missing independent distributed certificate".into());
            }
            let runs = array(&r["registered"])?;
            if runs.len() != 3 {
                return Err("missing call budgets".into());
            }
            let n = vector(&r["segment"]["normal"])?;
            let h = &runs[0]["projected"];
            if h.is_null() || h["time_ns"] != 0 || h["sticking"] != target["sticking"] {
                return Err(format!("unmatched registered impact {key}").into());
            }
            let boxes = uint(&h["work_boxes"])?;
            if (boxes == 0) != basic || boxes > 128 {
                return Err("used the wrong pressure proof".into());
            }
            let point = vector(&r["segment"]["points_m"][i])?;
            let j = vector(&h["j_ns"])?;
            let impulse_error = vector(&target["j_ns"])?.distance(j);
            let speed = vector(&h["opposite_after"])?.dot(n);
            close(impulse_error, 0.)?;
            close(point.distance(vector(&h["point"])?), 0.)?;
            close(speed, num(&target["opposite_m_s"])?)?;
            close(num(&target["work_loss_j"])?, num(&h["work_loss_j"])?)?;
            if num(&h["opening_bound_m_s"])? <= 0. || speed + 1e-12 < num(&h["opening_bound_m_s"])?
            {
                return Err("invalid opening bound".into());
            }
            for (ix, v) in runs.iter().enumerate() {
                if v["calls"] != [2, 8, 64][ix]
                    || v["projected"] != *h
                    || v["first_end_ns"] != 0
                    || num(&v["pair_load_ns"])? + 1e-12 < j.dot(n)
                {
                    return Err("budget or delivery changed the instantaneous result".into());
                }
                if basic {
                    let mut repeated = v.clone();
                    repeated["projected"]
                        .as_object_mut()
                        .unwrap()
                        .remove("work_boxes");
                    if repeated != old["registered"][ix] {
                        return Err("previously qualified native result changed".into());
                    }
                } else if !old["registered"][ix]["projected"].is_null() {
                    return Err("old native control already projected".into());
                }
            }
            bump(&mut counts, group);
            for (m, z) in [
                ("spatial_impulse_error_ns", impulse_error),
                ("visited_boxes", boxes as f64),
                ("opening_m_s", speed),
            ] {
                rows.entry((group.into(), "registered".into(), m.into()))
                    .or_default()
                    .push(z, key)
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"rows":samples(rows)}),
    )
}
