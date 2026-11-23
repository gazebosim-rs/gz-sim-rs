// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify the registered sliding vertex over the very same cold interval as
//! the endpoint and retimed native experiments, with a separate mesh reference.
use super::*;
pub(crate) fn summarize(dir: &Path, policy: &Path, mechanics: &Path) -> Result<()> {
    let mut fields = Fields::new();
    let mut counts = BTreeMap::new();
    let mut largest = None;
    let mut worst = None;
    for k in 0..1250 {
        let path = |d: &Path| d.join("cases").join(format!("case-{k:04}.json"));
        if ![dir, policy, mechanics]
            .into_iter()
            .all(|d| path(d).exists())
        {
            continue;
        }
        let (v, p, m) = (
            fixture(dir, k)?,
            fixture(policy, k)?,
            fixture(mechanics, k)?,
        );
        let sticking = v["format"] == "finite-secondary-sticking-point-v1";
        if (!sticking && v["format"] != "finite-secondary-sliding-vertex-v1")
            || p["format"] != "finite-secondary-interval-policy-v1"
            || m["format"] != "finite-clocked-endpoint-sequence-v1"
        {
            return Err("unmatched panel formats".into());
        }
        count(&mut counts, "fixtures");
        for t in array(&v["result"]["trials"])? {
            let id = 3 * k + uint(&t["stroke"])? as usize - 1;
            let ordinary = matched(&p, t)?;
            let mech = &matched(&m, t)?["response"];
            for key in ["initial", "offset_ns", "geometric_time_s", "prehistory"] {
                if t[key] != ordinary[key] {
                    return Err("different secondary initial conditions".into());
                }
            }
            if t["paths"].is_null() {
                if !ordinary["paths"].is_null() || !mech["paths"].is_null() {
                    return Err("missing cold donor".into());
                }
                continue;
            }
            count(&mut counts, "lines");
            let lanes = array(&t["paths"])?;
            if lanes.len() != 12 {
                return Err("missing vertex comparison".into());
            }
            let h = 500000 - uint(&t["offset_ns"])?;
            for (i, j) in [(0, 1), (2, 2), (4, 4), (6, 5)] {
                let mut a = lanes[i].clone();
                a.as_object_mut().unwrap().remove("spans");
                if a != ordinary["paths"][j] {
                    return Err(format!("field {id} baseline {i} changed").into());
                }
            }
            let fine = &mech["paths"][2];
            let prev = &mech["paths"][1];
            let reference = if fine["end"] == "original_clock"
                && prev["end"] == "original_clock"
                && eq_events(fine, prev)?
                && vec3(&fine["com_m"])?.distance(vec3(&prev["com_m"])?) <= 1e-7
                && vec3(&fine["impulse_ns"])?.distance(vec3(&prev["impulse_ns"])?) <= 1e-5
                && num(&fine["energy_defect_j"])?.abs() <= 1e-7
            {
                let target = vec3(&fine["com_m"])?
                    + vec3(&t["initial"]["pad"]["velocity"])?
                        * (num(&t["geometric_time_s"])? + h as f64 * 1e-9);
                Some((target, vec3(&fine["impulse_ns"])?))
            } else {
                None
            };
            if reference.is_some() {
                count(&mut counts, "matched_reference")
            }
            for a in lanes.chunks_exact(2) {
                let base = &a[0];
                let trial = &a[1];
                let name = trial["profile"].as_str().ok_or("measure missing label")?;
                if !array(&base["spans"])?.is_empty() || base["impulse"] != trial["impulse"] {
                    return Err("vertex following changed initial projection".into());
                }
                let calls = uint(&base["first"]["native_steps"])?;
                if calls != uint(&trial["first"]["native_steps"])?
                    || ![2, 8, 64].contains(&calls)
                    || trial["first"]["tick"] != base["first"]["tick"]
                {
                    return Err("native clock or work changed".into());
                }
                let spans = array(&trial["spans"])?;
                let mut end = 0;
                let mut span_ns = 0;
                let mut impulse = DVec3::ZERO;
                for s in spans {
                    let start = uint(&s["start_ns"])?;
                    let dt = uint(&s["duration_ns"])?;
                    if dt == 0
                        || start < end
                        || start + dt > h
                        || !matches!(uint(&s["subdivisions"])?, 2 | 4 | 8 | 16 | 32 | 64)
                    {
                        return Err(format!("invalid continuous support span at {id}").into());
                    }
                    if sticking {
                        let advance = (vec3(&s["to_m"])? - vec3(&s["from_m"])?)
                            - dt as f64 * 1e-9 * vec3(&t["initial"]["pad"]["velocity"])?;
                        if advance.length() > 2e-11
                            || num(&s["minimum_normal_n"])? <= 0.
                            || num(&s["minimum_cone_margin_n"])? <= 0.
                            || num(&s["normal_impulse_ns"])? <= 0.
                            || vec3(&s["normal_impulse_vector_ns"])?.length()
                                > num(&s["normal_impulse_ns"])? + 2e-13
                        {
                            return Err("sticking point slipped, lost pressure, or exceeded the integrated scalar normal".into());
                        }
                    } else if num(&s["minimum_inward_m_s2"])? <= 0.
                        || num(&s["minimum_slip_m_s"])? <= 0.
                        || num(&s["friction_work_j"])? > 0.
                    {
                        return Err("sliding point failed its force or slip guards".into());
                    }
                    let j = vec3(&s["impulse_ns"])?;
                    let n = vec3(&s["normal"])?;
                    if (n.length() - 1.).abs() > 1e-12 || j.dot(n) <= 0. {
                        return Err("noncompressive sustained force".into());
                    }
                    end = start + dt;
                    span_ns += dt;
                    impulse += j;
                }
                if spans.is_empty() {
                    if base["first"] != trial["first"]
                        || base["world_impulse_ns"] != trial["world_impulse_ns"]
                    {
                        return Err("refused sliding arc changed live native state".into());
                    }
                } else {
                    count(&mut counts, &format!("{name}_admitted"))
                }
                let center = |r: &Value| -> Result<DVec3> {
                    Ok((vec3(&r["first"]["foot"]["position"])?
                        + vec3(&r["first"]["upper"]["position"])?)
                        * 0.5)
                };
                let delta = center(base)?.distance(center(trial)?);
                add(&mut fields, name, "causal_com_m", delta, id);
                if !spans.is_empty() {
                    add(&mut fields, name, "registered_ns", span_ns as f64, id);
                    add(
                        &mut fields,
                        name,
                        "registered_spans",
                        spans.len() as f64,
                        id,
                    );
                    add(
                        &mut fields,
                        name,
                        "sustained_impulse_magnitude_ns",
                        impulse.length(),
                        id,
                    )
                }
                if matches!(name, "follow8" | "stick8") && largest.is_none_or(|(d, _)| delta > d) {
                    largest = Some((delta, id))
                }
                if let Some((center_ref, j)) = reference {
                    let improvement =
                        center(trial)?.distance(center_ref) - center(base)?.distance(center_ref);
                    if matches!(name, "follow8" | "stick8")
                        && worst.is_none_or(|(d, _)| improvement > d)
                    {
                        worst = Some((improvement, id))
                    }
                    let rank = if improvement < -1e-13 {
                        "closer"
                    } else if improvement > 1e-13 {
                        "farther"
                    } else {
                        "tie"
                    };
                    count(&mut counts, &format!("{name}_{rank}"));
                    for (label, row) in [(base["profile"].as_str().unwrap(), base), (name, trial)] {
                        add(
                            &mut fields,
                            label,
                            "com_error_m",
                            center(row)?.distance(center_ref),
                            id,
                        );
                        add(
                            &mut fields,
                            label,
                            "signed_impulse_error_ns",
                            vec3(&row["world_impulse_ns"])?.distance(j),
                            id,
                        );
                        add(
                            &mut fields,
                            label,
                            "spin_error_rad_s",
                            vec3(&row["first"]["foot"]["angular_velocity"])?
                                .distance(vec3(&fine["spin_rad_s"])?),
                            id,
                        );
                        if !spans.is_empty() {
                            add(
                                &mut fields,
                                &format!("{label}_admitted"),
                                "com_error_m",
                                center(row)?.distance(center_ref),
                                id,
                            );
                            add(
                                &mut fields,
                                &format!("{label}_admitted"),
                                "signed_impulse_error_ns",
                                vec3(&row["world_impulse_ns"])?.distance(j),
                                id,
                            )
                        }
                    }
                }
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("fixtures")==Some(&1250),"counts":counts,"largest_eight_causal_m_case":largest,"largest_eight_regression_m_case":worst,"rows":values(fields)}),
    )
}
