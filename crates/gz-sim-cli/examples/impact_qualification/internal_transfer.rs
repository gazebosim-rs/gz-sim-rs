// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct signed delivered impulses and internal changes of bearing on
//! the native clock. Only a continuous physical prefix can be compared eventwise.
use super::*;
fn center(row: &Value) -> Result<DVec3> {
    Ok(
        0.5 * (vec3(&row["first"]["foot"]["position"])?
            + vec3(&row["first"]["upper"]["position"])?),
    )
}
pub(crate) fn summarize(dir: &Path, sliding: &Path, sticking: &Path, clocked: &Path) -> Result<()> {
    field(dir, sliding, Some(sticking), clocked, false)
}
pub(crate) fn spatial(dir: &Path, transfer: &Path, clocked: &Path) -> Result<()> {
    field(dir, transfer, None, clocked, false)
}
pub(crate) fn launch(dir: &Path, spatial: &Path, clock: &Path) -> Result<()> {
    field(dir, spatial, None, clock, true)
}
fn verify_growth(g: &Value, dt: f64) -> Result<()> {
    if !g.is_null() {
        let dir = vec3(&g["direction"])?;
        let end = vec3(&g["end_material_velocity_m_s"])?;
        let a = num(&g["acceleration_m_s2"])?;
        let v = num(&g["initial_m_s"])?;
        if (dir.length() - 1.).abs() > 2e-12
            || a <= 0.
            || v < 0.
            || end.dot(dir) + 2e-10 * (1. + end.length()) < v + a * dt
        {
            return Err(
                "integrated material speed contradicts directional growth certificate".into(),
            );
        }
    }
    Ok(())
}
fn field(
    dir: &Path,
    baseline: &Path,
    sticking: Option<&Path>,
    clocked: &Path,
    launch: bool,
) -> Result<()> {
    let spatial = sticking.is_none();
    let mut counts = BTreeMap::new();
    let mut fields = Fields::new();
    let mut largest = None;
    let mut worst = None;
    for k in 0..1250 {
        let path = |d: &Path| d.join("cases").join(format!("case-{k:04}.json"));
        if ![dir, baseline, sticking.unwrap_or(baseline), clocked]
            .into_iter()
            .all(|d| path(d).exists())
        {
            continue;
        }
        let (a, s, v, m) = (
            fixture(dir, k)?,
            fixture(baseline, k)?,
            sticking
                .map(|s| fixture(s, k))
                .transpose()?
                .unwrap_or(Value::Null),
            fixture(clocked, k)?,
        );
        if a["format"]
            != if launch {
                "finite-secondary-skew-slip-launch-v1"
            } else if spatial {
                "finite-secondary-sliding-skew-edge-v1"
            } else {
                "finite-secondary-internal-transfer-v1"
            }
            || s["format"]
                != if launch {
                    "finite-secondary-sliding-skew-edge-v1"
                } else if spatial {
                    "finite-secondary-internal-transfer-v1"
                } else {
                    "finite-secondary-sliding-vertex-v1"
                }
            || (!spatial && v["format"] != "finite-secondary-sticking-point-v1")
            || m["format"] != "finite-clocked-endpoint-sequence-v1"
        {
            return Err("incompatible transfer campaign".into());
        }
        count(&mut counts, "fixtures");
        for t in array(&a["result"]["trials"])? {
            let id = 3 * k + uint(&t["stroke"])? as usize - 1;
            let (slide, stick, mech) = (
                matched(&s, t)?,
                if spatial {
                    &Value::Null
                } else {
                    matched(&v, t)?
                },
                &matched(&m, t)?["response"],
            );
            for key in ["offset_ns", "prehistory", "initial"] {
                if t[key] != slide[key] || (!spatial && t[key] != stick[key]) {
                    return Err("unequal cold input".into());
                }
            }
            if t["paths"].is_null() {
                continue;
            }
            count(&mut counts, "lines");
            let h = 500000 - uint(&t["offset_ns"])?;
            let (fine, coarse) = (&mech["paths"][2], &mech["paths"][1]);
            let reference = fine["end"] == "original_clock"
                && coarse["end"] == "original_clock"
                && eq_events(fine, coarse)?
                && vec3(&fine["com_m"])?.distance(vec3(&coarse["com_m"])?) <= 1e-7
                && vec3(&fine["impulse_ns"])?.distance(vec3(&coarse["impulse_ns"])?) <= 1e-5
                && num(&fine["energy_defect_j"])?.abs() <= 1e-7;
            let vel = vec3(&t["initial"]["pad"]["velocity"])?;
            let target =
                vec3(&fine["com_m"])? + vel * (num(&t["geometric_time_s"])? + h as f64 * 1e-9);
            let events = array(&fine["events"])?;
            let lanes = array(&t["paths"])?;
            if lanes.len() != 12 {
                return Err("incomplete transfer ladder".into());
            }
            if reference {
                count(&mut counts, "matched_reference")
            }
            for (pair, group) in lanes.chunks_exact(2).enumerate() {
                let (b, z) = (&group[0], &group[1]);
                let idx = 2 * pair + 1;
                let name = z["profile"].as_str().ok_or("no native policy")?;
                let calls = [2, 2, 8, 8, 64, 64][pair] as u64;
                if uint(&z["first"]["native_steps"])? != calls
                    || uint(&b["first"]["native_steps"])? != calls
                    || z["impulse"] != b["impulse"]
                    || (!spatial && !array(&b["transfers"])?.is_empty())
                {
                    return Err("initial impact or native budget changed".into());
                }
                let spans = array(&b["spans"])?;
                let has_stick = spans.iter().any(|s| !s["minimum_cone_margin_n"].is_null());
                let has_slide = spans.iter().any(|s| !s["minimum_slip_m_s"].is_null());
                if spatial {
                    if b != &slide["paths"][idx] {
                        return Err(format!("former transfer policy changed at {id} {name}").into());
                    }
                } else if !has_stick || !has_slide {
                    let other = if has_stick {
                        &stick["paths"][idx]
                    } else {
                        &slide["paths"][idx]
                    };
                    for key in ["spans", "first", "world_impulse_ns", "impulse"] {
                        if b[key] != other[key] {
                            return Err(
                                format!("single-mode repeat changed {id} {name} {key}").into()
                            );
                        }
                    }
                } else {
                    count(&mut counts, "mixed_baseline_intervals")
                }
                let transfers = array(&z["transfers"])?;
                let skew = array(&z["spans"])?
                    .iter()
                    .any(|s| s["sliding_edge"] == true)
                    || transfers.iter().any(|t| {
                        t["pieces"].as_array().is_some_and(|p| {
                            p.iter().any(|s| {
                                s["sticking"] == false
                                    && s["feature"]
                                        .as_str()
                                        .is_some_and(|f| f.starts_with("EdgePair"))
                            })
                        })
                    });
                let launched = array(&z["spans"])?
                    .iter()
                    .any(|s| !s["slip_growth"].is_null())
                    || transfers.iter().any(|t| {
                        t["pieces"]
                            .as_array()
                            .is_some_and(|p| p.iter().any(|s| !s["slip_growth"].is_null()))
                    });
                if launch && launched {
                    count(&mut counts, &format!("{name}_growth_admitted"));
                }
                if spatial && skew {
                    count(&mut counts, &format!("{name}_skew_admitted"));
                }
                if (launch && !launched)
                    || (!launch && spatial && !skew)
                    || (!spatial && transfers.is_empty())
                {
                    for key in ["first", "spans", "world_impulse_ns", "transfers"] {
                        if b[key] != z[key] {
                            return Err("declined transfer modified ordinary path".into());
                        }
                    }
                } else {
                    count(&mut counts, &format!("{name}_admitted"))
                }
                let mut work: Vec<_> = array(&z["spans"])?
                    .iter()
                    .map(|s| (s, false))
                    .chain(transfers.iter().map(|s| (s, true)))
                    .collect();
                work.sort_by_key(|(s, _)| s["start_ns"].as_u64().unwrap());
                let mut stop = 0;
                let mut certified = 0;
                let mut prefix = true;
                let mut event = 1;
                for (s, split) in work {
                    let at = uint(&s["start_ns"])?;
                    let dt = uint(&s["duration_ns"])?;
                    if at < stop || dt == 0 || at + dt > h {
                        return Err("overlapping or invalid span".into());
                    }
                    prefix &= at == stop;
                    stop = at + dt;
                    certified += dt;
                    if !split {
                        if s["sliding_edge"] == true {
                            verify_growth(&s["slip_growth"], dt as f64 * 1e-9)?;
                            let effort = num(&s["normal_impulse_ns"])?;
                            if !s["feature"]
                                .as_str()
                                .is_some_and(|f| f.starts_with("EdgePair"))
                                || (num(&s["minimum_slip_m_s"])? <= 0.
                                    && s["slip_growth"].is_null())
                                || num(&s["minimum_inward_m_s2"])? <= 0.
                                || num(&s["friction_work_j"])? >= 0.
                                || effort <= 0.
                                || vec3(&s["normal_impulse_vector_ns"])?.length() > effort + 2e-12
                            {
                                return Err("invalid sliding skew receipt".into());
                            }
                        }
                        continue;
                    }
                    let mut total_j = DVec3::ZERO;
                    let mut total_n = DVec3::ZERO;
                    let mut scalar = 0.;
                    let mut time = 0.;
                    let parts = array(&s["pieces"])?;
                    if !(2..=5).contains(&parts.len()) {
                        return Err("invalid number of internal pieces".into());
                    }
                    for (i, p) in parts.iter().enumerate() {
                        close((num(&p["offset_s"])? - time).abs(), 2e-16)?;
                        time += num(&p["duration_s"])?;
                        verify_growth(&p["slip_growth"], num(&p["duration_s"])?)?;
                        total_j += vec3(&p["j_ns"])?;
                        total_n += vec3(&p["normal_vector_ns"])?;
                        scalar += num(&p["normal_ns"])?;
                        if num(&p["duration_s"])? <= 0.
                            || num(&p["normal_ns"])? <= 0.
                            || num(&p["work_j"])? > 1e-14
                        {
                            return Err("invalid sustained component".into());
                        }
                        let e = &p["impact"];
                        if e.is_null() {
                            if i + 1 != parts.len() {
                                return Err("missing intermediate impulse".into());
                            }
                            continue;
                        }
                        let at_s = num(&e["bracket_s"][0])?;
                        let upper = num(&e["bracket_s"][1])?;
                        close((time - at_s).abs(), 2e-16)?;
                        if upper < at_s
                            || upper - at_s > 2.1e-17
                            || num(&e["opening_m_s"])? <= 0.
                            || num(&e["lost_j"])? < 0.
                            || uint(&e["work_boxes"])? > 128
                        {
                            return Err("unqualified internal impulse".into());
                        }
                        let j = vec3(&e["j_ns"])?;
                        let n = vec3(&e["normal"])?;
                        let amount = j.dot(n);
                        if amount <= 0. {
                            return Err("tensile internal impact".into());
                        }
                        total_j += j;
                        total_n += amount * n;
                        scalar += amount;
                        count(&mut counts, &format!("{name}_internal_impacts"));
                        if prefix {
                            let truth = events
                                .get(event)
                                .ok_or("extra internal impact on continuous prefix")?;
                            let time = at as f64 * 1e-9 + at_s;
                            let difference = (time - num(&truth["at_s"])?).abs();
                            let dj = j.distance(vec3(&truth["j_ns"])?);
                            let target_point =
                                vec3(&truth["points_m"][uint(&truth["loaded_end"])? as usize])?
                                    + vel * (num(&t["geometric_time_s"])? + time);
                            let dp = vec3(&e["point_m"])?.distance(target_point);
                            if difference > 2e-9
                                || dj > 2e-6
                                || dp > 2e-8
                                || truth["sticking"] != e["stick"]
                            {
                                return Err(format!("physical prefix event mismatch {id} {name} t={difference:e} j={dj:e} site={dp:e}").into());
                            }
                            for (key, v) in [
                                ("prefix_event_time_s", difference),
                                ("prefix_event_impulse_ns", dj),
                                ("prefix_event_position_m", dp),
                            ] {
                                add(&mut fields, name, key, v, id)
                            }
                            event += 1;
                        }
                    }
                    close((time - dt as f64 * 1e-9).abs(), 2e-15)?;
                    close(total_j.distance(vec3(&s["j_ns"])?), 3e-12)?;
                    close(total_n.distance(vec3(&s["normal_vector_ns"])?), 3e-12)?;
                    close((scalar - num(&s["normal_ns"])?).abs(), 3e-12)?;
                }
                if !transfers.is_empty() {
                    add(
                        &mut fields,
                        name,
                        "covered_fraction",
                        certified as f64 / h as f64,
                        id,
                    )
                }
                let d = center(z)?.distance(center(b)?);
                if name
                    == if launch {
                        "launch8"
                    } else if spatial {
                        "spatial8"
                    } else {
                        "transfer8"
                    }
                    && largest.is_none_or(|(v, _)| d > v)
                {
                    largest = Some((d, id))
                }
                if !reference {
                    continue;
                }
                let je = vec3(&fine["impulse_ns"])?;
                for row in [b, z] {
                    let label = row["profile"].as_str().unwrap();
                    let err = center(row)?.distance(target);
                    let dj = vec3(&row["world_impulse_ns"])?.distance(je);
                    add(&mut fields, label, "com_error_m", err, id);
                    add(&mut fields, label, "impulse_error_ns", dj, id);
                    if !transfers.is_empty() {
                        add(
                            &mut fields,
                            &format!("{label}_admitted"),
                            "com_error_m",
                            err,
                            id,
                        );
                        add(
                            &mut fields,
                            &format!("{label}_admitted"),
                            "impulse_error_ns",
                            dj,
                            id,
                        )
                    }
                }
                if prefix && stop == h {
                    let delta = center(z)?.distance(target);
                    let dj = vec3(&z["world_impulse_ns"])?.distance(je);
                    if delta > 1e-8 || dj > 5e-6 || event != events.len() {
                        return Err(format!(
                            "continuous physical interval diverged {id} {name}: {delta:e} {dj:e}"
                        )
                        .into());
                    }
                    count(&mut counts, &format!("{name}_whole_interval"));
                    add(&mut fields, name, "whole_interval_j_error_ns", dj, id);
                }
                let change = center(z)?.distance(target) - center(b)?.distance(target);
                if name
                    == if launch {
                        "launch8"
                    } else if spatial {
                        "spatial8"
                    } else {
                        "transfer8"
                    }
                    && worst.is_none_or(|(v, _)| change > v)
                {
                    worst = Some((change, id))
                }
                let rank = if change < -1e-13 {
                    "closer"
                } else if change > 1e-13 {
                    "farther"
                } else {
                    "tie"
                };
                count(&mut counts, &format!("{name}_{rank}"));
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("fixtures")==Some(&1250),"counts":counts,"largest_eight_causal_m_case":largest,"largest_eight_regression_m_case":worst,"rows":values(fields)}),
    )
}
