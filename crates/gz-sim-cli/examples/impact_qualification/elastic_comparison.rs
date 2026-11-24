// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Readback of the fixture 80 material experiment at the original public clocks.
use super::{transfer::vector, *};
use glam::DVec3;

fn sample_at(rows: &[Value], at: f64) -> Result<&Value> {
    let mut matches = rows.iter().filter(|r| {
        r["original_time_s"]
            .as_f64()
            .is_some_and(|t| (t - at).abs() < 1e-12)
    });
    let row = matches.next().ok_or("missing common sample time")?;
    if matches.next().is_some() {
        return Err("duplicate clock in profile".into());
    }
    Ok(row)
}

fn native_center(row: &Value) -> Result<DVec3> {
    Ok(0.5 * (vector(&row["foot"]["position"])? + vector(&row["upper"]["position"])?))
}

pub(super) fn compare(edge_roll: &Path, directory: &Path) -> Result<()> {
    let material = read(&directory.join("reference.json"))?;
    let original = read(&edge_roll.join("reference.json"))?;
    if original["case"] != 80 || original["stroke"] != 2 {
        return Err("this comparison uses the fixture 80 two-link mass and gravity".into());
    }
    let drive = vector(&original["drive"])?;
    let references = array(&original["references"])?;
    let first = &references.first().ok_or("missing public restart")?["fine"];
    let line = &references.last().ok_or("missing line arrival")?["fine"];
    let restart_time = num(&first["elapsed"])?;
    let start = num(&material["start_s"])?;
    if line["end"] != "unproved_line" || (start - num(&line["elapsed"])?).abs() > 1e-12 {
        return Err("material run must begin at the same line arrival".into());
    }
    let arrival_j = vector(&line["j"])? - vector(&first["j"])?;
    let incoming = vector(&line["velocity"])?;
    let profiles = array(&material["profiles"])?;
    for profile in profiles {
        let initial = sample_at(array(&profile["frames"])?, start)?;
        for (material_key, source_key) in [
            ("com_m", "com"),
            ("foot_m", "foot"),
            ("v_m_s", "velocity"),
            ("spin_rad_s", "omega"),
        ] {
            if vector(&initial[material_key])?.distance(vector(&line[source_key])?) > 1e-12 {
                return Err("material profile starts from a different incoming state".into());
            }
        }
        if initial["orientation_xyzw"] != line["orientation"]
            || vector(&initial["support_frame_j_ns"])? != DVec3::ZERO
        {
            return Err(
                "material arrival changed the foot orientation or applied an impulse".into(),
            );
        }
    }
    let controls = &original["conditional_pressure_models"]["alternatives"];
    let traces = edge_roll.join("raw/case-0080-stroke-2/traces");
    let revised = read(&traces.join("lane-3.json"))?;
    let continuous = read(&traces.join("lane-4.json"))?;
    let revised = array(&revised["frames"])?;
    let continuous = array(&continuous["frames"])?;
    let initial = original["frames"]
        .get(0)
        .ok_or("missing matched initial frame")?;
    if profiles.len() < 2
        || revised.len() != continuous.len()
        || revised.is_empty()
        || ["foot", "upper", "pad", "time_us", "tick"]
            .iter()
            .any(|key| revised[0][key] != initial[key] || continuous[0][key] != initial[key])
    {
        return Err("incomplete comparison or unmatched restart".into());
    }
    let restart_us = uint(&initial["time_us"])?;
    let mut measured = [DVec3::ZERO; 2];
    let mut rows = vec![];
    for (r, c) in revised.iter().zip(continuous).skip(1) {
        if r["time_us"] != c["time_us"]
            || r["tick"] != c["tick"]
            || r["drive"] != c["drive"]
            || num(&r["drive"])? != drive.z
        {
            return Err("mismatched controller clocks or deck speed".into());
        }
        let elapsed_us = uint(&r["time_us"])?
            .checked_sub(restart_us)
            .ok_or("reversed clock")?;
        let at = restart_time + elapsed_us as f64 * 1e-6;
        measured[0] += vector(&r["j"])?;
        measured[1] += vector(&c["j"])?;
        if at < start {
            continue;
        }
        let mut materials = vec![];
        let mut separations = vec![];
        for (index, p) in profiles.iter().enumerate() {
            let f = sample_at(array(&p["frames"])?, at)?;
            let center = vector(&f["com_m"])?;
            let impulse = vector(&f["support_frame_j_ns"])?;
            // Independently reconstruct the impulse from the change in velocity
            // of the two 1 kg links and the gravitational impulse since arrival.
            let balance = 2. * (vector(&f["v_m_s"])? - incoming + 9.81 * (at - start) * DVec3::Z);
            let residual = balance.distance(impulse);
            if residual > 1e-8 {
                return Err(
                    format!("material impulse and momentum disagree by {residual} N s").into(),
                );
            }
            let world_center = center + at * drive;
            let mut rigid = vec![];
            for scenario in array(controls)? {
                let finest = array(&scenario["continuation"])?
                    .last()
                    .ok_or("missing rigid control")?;
                // A candidate with a negative pressure stopped at arrival.
                if array(&finest["frames"])?.is_empty() {
                    continue;
                }
                let rf = sample_at(array(&finest["frames"])?, at)?;
                let rigid_j = vector(&scenario["j_ns"])? + vector(&rf["support_frame_j_ns"])?;
                rigid.push(json!({"fractions":scenario["fractions"],
                    "com_distance_m":center.distance(vector(&rf["com_m"])?),
                    "impulse_distance_ns":impulse.distance(rigid_j)}));
            }
            for previous in &profiles[..index] {
                let pf = sample_at(array(&previous["frames"])?, at)?;
                separations.push(json!({"profiles":[previous["name"],p["name"]],
                    "com_distance_m":center.distance(vector(&pf["com_m"])?),
                    "impulse_distance_ns":impulse.distance(vector(&pf["support_frame_j_ns"])?) }));
            }
            let native = |frame: &Value, accumulated: DVec3| -> Result<Value> {
                Ok(
                    json!({"com_distance_m":world_center.distance(native_center(frame)?),
                    "accumulated_impulse_distance_ns":(arrival_j+impulse).distance(accumulated)}),
                )
            };
            materials.push(
                json!({"name":p["name"],"normal_force_n":num(&f["normal_force_n"])?,
                "momentum_residual_ns":residual,
                "revised":native(r,measured[0])?,"continuous_native":native(c,measured[1])?,
                "rigid_pressure_controls":rigid}),
            );
        }
        rows.push(
            json!({"elapsed_since_restart_us":elapsed_us,"original_time_s":at,
            "materials":materials,"pairwise_material_separations":separations}),
        );
    }
    if rows.is_empty() {
        return Err("no overlapping material and native clocks".into());
    }
    write(
        &directory.join("comparison.json"),
        &json!({"case":80,"stroke":2,"frames":rows}),
    )
}
