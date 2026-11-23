// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::transfer::{fixture, matched, vector as vec3};
use super::*;
use glam::{DQuat, DVec3};
#[path = "vertex.rs"]
pub(crate) mod vertex;

#[path = "internal_transfer.rs"]
pub(crate) mod internal;

#[path = "cold_point.rs"]
pub(crate) mod cold_point;
#[path = "compact.rs"]
pub(crate) mod compact;
#[path = "active_episode.rs"]
pub(crate) mod episode;
#[path = "primary_change.rs"]
pub(crate) mod primary_change;
#[path = "whole_primary.rs"]
pub(crate) mod whole_primary;
type Fields = BTreeMap<(String, String), Series>;
fn add(s: &mut Fields, group: &str, label: &str, v: f64, k: usize) {
    s.entry((group.into(), label.into()))
        .or_default()
        .push(v, k)
}
fn values(s: Fields) -> Value {
    json!(s.into_iter().map(|((group,metric),series)|json!({"group":group,"metric":metric,"statistics":series.stats()})).collect::<Vec<_>>())
}
fn count(c: &mut BTreeMap<String, usize>, k: &str) {
    *c.entry(k.into()).or_default() += 1
}
fn close(d: f64, tol: f64) -> Result<()> {
    if !d.is_finite() || d > tol {
        Err(format!("mechanical readback residual {d} exceeds {tol}").into())
    } else {
        Ok(())
    }
}
fn quat(v: &Value) -> Result<DQuat> {
    Ok(DQuat::from_array(serde_json::from_value(v.clone())?))
}
fn eq_events(a: &Value, b: &Value) -> Result<bool> {
    let (a, b) = (array(&a["events"])?, array(&b["events"])?);
    Ok(a.len() == b.len()
        && a.iter().zip(b).all(|(x, y)| {
            ["mode", "feature", "loaded_end", "sticking"]
                .iter()
                .all(|k| x[k] == y[k])
                && x["arc"]["end"] == y["arc"]["end"]
        }))
}
fn pair(s: &mut Fields, g: &str, a: &Value, b: &Value, k: usize) -> Result<()> {
    for key in ["com_m", "velocity_m_s", "spin_rad_s", "impulse_ns"] {
        add(s, g, key, vec3(&a[key])?.distance(vec3(&b[key])?), k)
    }
    let dq = quat(&a["orientation_xyzw"])? * quat(&b["orientation_xyzw"])?.inverse();
    add(
        s,
        g,
        "orientation_rad",
        2. * dq.xyz().length().atan2(dq.w.abs()),
        k,
    );
    Ok(())
}
pub(crate) fn summarize(dir: &Path, registered: &Path, previous: Option<&Path>) -> Result<()> {
    let mut fields = Fields::new();
    let mut counts = BTreeMap::new();
    for k in 0..1250 {
        let file = dir.join("cases").join(format!("case-{k:04}.json"));
        if !file.exists() {
            continue;
        }
        let d = read(&file)?;
        let label = d["format"].as_str().ok_or("no sequence format")?;
        if ![
            "finite-intermittent-endpoint-sequence-v1",
            "finite-implicit-endpoint-sequence-v2",
            "finite-clocked-endpoint-sequence-v1",
        ]
        .contains(&label)
        {
            return Err("unrecognized contact sequence".into());
        }
        let clocked = label.starts_with("finite-clocked");
        let refd = fixture(registered, k)?;
        let old = previous.map(|x| fixture(x, k)).transpose()?;
        count(&mut counts, "fixtures");
        for t in array(&d["result"]["trials"])? {
            let idx = 3 * k + uint(&t["stroke"])? as usize - 1;
            let r = &t["response"];
            let base = &matched(&refd, t)?["response"];
            if r["geometry"] != base["geometry"] || r["arc_end"] != base["arc_end"] {
                return Err("sequence incoming state changed".into());
            }
            let Some(paths) = r["paths"].as_array() else {
                count(&mut counts, "no_line");
                continue;
            };
            if paths.len() != 3 {
                return Err("missing mesh".into());
            }
            count(&mut counts, "lines");
            let fine = &paths[2];
            let start = &base["arc_end"];
            let cv = vec3(&start["velocity_m_s"])?;
            let spin = vec3(&start["spin_rad_s"])?;
            let end = fine["end"].as_str().ok_or("missing ending reason")?;
            count(&mut counts, &format!("end_{end}"));
            count(
                &mut counts,
                &format!("{}_impacts", array(&fine["events"])?.len()),
            );
            let mut launch = false;
            for (i, p) in paths.iter().enumerate() {
                let required = if clocked {
                    [32, 128, 512]
                } else {
                    [8, 32, 128]
                };
                if p["subdivisions"] != required[i] {
                    return Err("incorrect mesh label".into());
                }
                let mut j = DVec3::ZERO;
                let mut tau = DVec3::ZERO;
                let mut work = 0.;
                let mut c = vec3(&start["com_m"])?;
                let mut time = 0.;
                for (number, e) in array(&p["events"])?.iter().enumerate() {
                    close((num(&e["at_s"])? - time).abs(), 2e-15)?;
                    let n = vec3(&e["normal"])?;
                    let imp = vec3(&e["j_ns"])?;
                    if number == 0 {
                        close(
                            imp.distance(vec3(&base["registered"][1]["projected"]["j_ns"])?),
                            2e-11,
                        )?;
                    }
                    let point = vec3(&e["points_m"][uint(&e["loaded_end"])? as usize])?;
                    j += imp;
                    tau += (point - c).cross(imp);
                    work -= num(&e["loss_j"])?;
                    if imp.dot(n) <= 0. || num(&e["opposite_speed_m_s"])? <= 0. {
                        return Err("invalid open-line impulse".into());
                    }
                    if i == 2 && e["mode"] == "IncipientSlip" {
                        launch = true
                    }
                    let a = &e["arc"];
                    if a.is_null() {
                        continue;
                    }
                    time += num(&a["elapsed_interval_s"][0])?;
                    j += vec3(&a["linear_impulse_ns"])?;
                    tau += vec3(&a["angular_impulse_nms"])?;
                    work += num(&a["friction_work_j"])?;
                    c = vec3(&a["com_m"])?;
                }
                close(j.distance(vec3(&p["impulse_ns"])?), 5e-12)?;
                close(tau.distance(vec3(&p["angular_impulse_nms"])?), 5e-11)?;
                close((work - num(&p["contact_work_j"])?).abs(), 5e-12)?;
                close((time - num(&p["elapsed_s"])?).abs(), 1e-15)?;
                let linear = 2. * (vec3(&p["velocity_m_s"])? - cv + time * 9.81 * DVec3::Z);
                close(
                    (linear.distance(j) - num(&p["linear_defect_ns"])?).abs(),
                    5e-12,
                )?;
                close(
                    ((0.05 * (vec3(&p["spin_rad_s"])? - spin)).distance(tau)
                        - num(&p["angular_defect_nms"])?)
                    .abs(),
                    5e-12,
                )?;
                if p["end"] == "original_clock" {
                    close((time - num(&p["horizon_s"])?).abs(), 1e-14)?
                }
                if i < 2 {
                    if !eq_events(p, fine)? || p["end"] != fine["end"] {
                        count(&mut counts, &format!("event_mismatch_mesh_{i}"));
                    }
                    pair(&mut fields, &format!("mesh_{i}_vs_finest"), p, fine, idx)?;
                }
            }
            let class = if launch {
                "contains_incipient_slip"
            } else {
                "smooth_launches"
            };
            count(&mut counts, class);
            pair(&mut fields, class, &paths[1], fine, idx)?;
            for field in ["energy_defect_j", "linear_defect_ns", "angular_defect_nms"] {
                add(&mut fields, "finest", field, num(&fine[field])?.abs(), idx)
            }
            if let Some(old) = &old {
                let x = &matched(old, t)?["response"]["paths"][2];
                if x.is_null() {
                    return Err("missing prior sequence".into());
                }
                pair(
                    &mut fields,
                    &format!("previous_to_current_{class}"),
                    x,
                    fine,
                    idx,
                )?;
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("fixtures")==Some(&1250),"counts":counts,"rows":values(fields)}),
    )
}
pub(crate) fn policy(dir: &Path, mechanics: &Path) -> Result<()> {
    let mut s = Fields::new();
    let mut c = BTreeMap::new();
    let mut largest = None;
    let mut worst = None;
    let names = [
        "native2",
        "endpoint2",
        "retimed2",
        "native8",
        "endpoint8",
        "retimed8",
        "native4096",
        "native8192",
    ];
    for k in 0..1250 {
        if !dir.join("cases").join(format!("case-{k:04}.json")).exists()
            || !mechanics
                .join("cases")
                .join(format!("case-{k:04}.json"))
                .exists()
        {
            continue;
        }
        let d = fixture(dir, k)?;
        let m = fixture(mechanics, k)?;
        if d["format"] != "finite-secondary-interval-policy-v1"
            || m["format"] != "finite-clocked-endpoint-sequence-v1"
        {
            return Err("invalid policy or reference".into());
        }
        count(&mut c, "fixtures");
        for t in array(&d["result"]["trials"])? {
            let idx = 3 * k + uint(&t["stroke"])? as usize - 1;
            let z = &matched(&m, t)?["response"];
            if t["paths"].is_null() {
                if !z["paths"].is_null() {
                    return Err("laboratory cut differed".into());
                }
                continue;
            }
            let paths = array(&t["paths"])?;
            let fine = &z["paths"][2];
            let prev = &z["paths"][1];
            if paths.len() != names.len()
                || z["offset_ns"] != t["offset_ns"]
                || z["arc_end"] != t["prehistory"]["arc_end"]
            {
                return Err("unmatched policy start".into());
            }
            count(&mut c, "lines");
            let h = 500_000 - uint(&t["offset_ns"])?;
            close((num(&fine["horizon_s"])? - h as f64 * 1e-9).abs(), 1e-15)?;
            let center = |i: usize| -> Result<DVec3> {
                Ok(0.5
                    * (vec3(&paths[i]["first"]["foot"]["position"])?
                        + vec3(&paths[i]["first"]["upper"]["position"])?))
            };
            for (i, p) in paths.iter().enumerate() {
                if p["profile"] != names[i]
                    || uint(&p["first"]["native_steps"])?
                        != ([2, 2, 2, 8, 8, 8, 4096, 8192][i] as u64).min(h)
                {
                    return Err("policy budget changed".into());
                }
                if (i == 2 || i == 5) && !p["impulse"]["retimed"].is_null() {
                    count(&mut c, &format!("retimed_{}", names[i]));
                }
                if i == 2 || i == 5 {
                    for key in [
                        "j_ns",
                        "normal",
                        "energy_j",
                        "point_m",
                        "stick",
                        "work_boxes",
                        "opposite_after",
                    ] {
                        if p["impulse"][key] != paths[i - 1]["impulse"][key] {
                            return Err("retiming changed physical impact".into());
                        }
                    }
                }
                for (j, name) in [(6, "4096"), (7, "8192")] {
                    add(
                        &mut s,
                        &format!("{}_vs_{name}", names[i]),
                        "com_m",
                        center(i)?.distance(center(j)?),
                        idx,
                    );
                }
            }
            let delta = center(5)?.distance(center(4)?);
            if largest.is_none_or(|(v, _)| delta > v) {
                largest = Some((delta, idx))
            }
            if h < 8192 {
                count(&mut c, "native_reference_saturated")
            }
            if fine["end"] != "original_clock"
                || prev["end"] != "original_clock"
                || !eq_events(fine, prev)?
            {
                count(&mut c, "unresolved_reference");
                continue;
            }
            let target = vec3(&fine["com_m"])?
                + vec3(&t["initial"]["pad"]["velocity"])?
                    * (num(&t["geometric_time_s"])? + h as f64 * 1e-9);
            let j = vec3(&fine["impulse_ns"])?;
            let err = fine["energy_defect_j"]
                .as_f64()
                .ok_or("missing work balance")?
                .abs();
            if vec3(&prev["com_m"])?.distance(vec3(&fine["com_m"])?) > 1e-7
                || vec3(&prev["impulse_ns"])?.distance(j) > 1e-5
                || err > 1e-7
            {
                count(&mut c, "unresolved_accuracy");
                continue;
            }
            count(&mut c, "matched_physical");
            for (i, p) in paths.iter().enumerate() {
                let g = format!("{}_vs_mechanics", names[i]);
                add(&mut s, &g, "com_m", center(i)?.distance(target), idx);
                add(
                    &mut s,
                    &g,
                    "signed_impulse_error_ns",
                    vec3(&p["world_impulse_ns"])?.distance(j),
                    idx,
                );
                add(
                    &mut s,
                    &g,
                    "spin_rad_s",
                    vec3(&p["first"]["foot"]["angular_velocity"])?
                        .distance(vec3(&fine["spin_rad_s"])?),
                    idx,
                );
                add(
                    &mut s,
                    &g,
                    "scalar_normal_error_ns",
                    (num(&p["first"]["normal_impulse_ns"])? - num(&fine["normal_impulse_ns"])?)
                        .abs(),
                    idx,
                );
            }
            let increase = center(5)?.distance(target) - center(4)?.distance(target);
            if worst.is_none_or(|(v, _)| increase > v) {
                worst = Some((increase, idx))
            }
            count(
                &mut c,
                if increase < -1e-15 {
                    "retiming_closer"
                } else if increase > 1e-15 {
                    "retiming_farther"
                } else {
                    "retiming_tie"
                },
            );
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"counts":c,"largest_causal_com_m_case":largest,"largest_mechanical_com_regression_m_case":worst,"rows":values(s)}),
    )
}
