// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Evaluate the continuous mechanical control at the same clocks as the cold replay.
use super::super::super::finite::{geometry, kinematics, load::complete};
use super::*;
#[path = "corner_arrival.rs"]
pub(crate) mod corner_arrival;
#[path = "free_landing.rs"]
pub(crate) mod free_landing;
#[path = "line_pressure_alternatives.rs"]
mod pressure_alternatives;

pub(crate) fn write(dir: &Path, case: usize, stroke: usize) -> Result<()> {
    if case >= 1250 || !(1..=3).contains(&stroke) {
        return Err("invalid field landing".into());
    }
    let fixture = survey::fixture(case);
    let cut = survey::cuts(fixture)?.0[stroke - 1].ok_or("missing primary cut")?;
    let (flight, _) = kinematics(fixture, &cut)?;
    let geometry = geometry(flight)?;
    let drive = command(fixture, cut.tick) * DVec3::Z;
    let mut references = Vec::new();
    let mut line_alternatives = Value::Null;
    let mut coarse_alternatives = Value::Null;
    let ticks_on_same_drive = (1..=27)
        .take_while(|at| command(fixture, cut.tick + at - 1) == drive.z)
        .count() as u64;
    let horizon = ticks_on_same_drive as f64 * DT.as_secs_f64();
    for at in 1..=ticks_on_same_drive {
        let h = DT.as_secs_f64() * at as f64;
        let mut mesh = 128;
        let (mut coarse, mut coarse_line) =
            complete::at_with_line_diagnostics(flight, &geometry, h, mesh)?;
        let (mut fine, mut unresolved_line) =
            complete::at_with_line_diagnostics(flight, &geometry, h, mesh * 4)?;
        let mut qualified = handoff::qualified(&coarse, &fine)?;
        while !qualified && mesh < 8192 {
            mesh *= 4;
            (coarse, coarse_line) = (fine, unresolved_line);
            (fine, unresolved_line) =
                complete::at_with_line_diagnostics(flight, &geometry, h, mesh * 4)?;
            qualified = handoff::qualified(&coarse, &fine)?;
        }
        if !qualified && fine["end"] == "unproved_line" && !unresolved_line.is_null() {
            line_alternatives =
                pressure_alternatives::study(flight, &fine, &unresolved_line, horizon)?;
        }
        if !qualified && coarse["end"] == "unproved_line" && !coarse_line.is_null() {
            coarse_alternatives =
                pressure_alternatives::study(flight, &coarse, &coarse_line, horizon)?;
        }
        references.push(json!({"at":at,"meshes":[mesh, mesh*4],"unresolved_line":unresolved_line,"coarse":coarse,"fine":fine,"qualified":qualified}));
        if !qualified {
            break;
        }
    }
    let doc = json!({"drive":drive.to_array(),"references":references});
    let ticks = handoff::prefix(fixture, &cut, &doc)?;
    let mut frames = Vec::new();
    for (i, tick) in ticks.into_iter().enumerate() {
        let mut row = tick.as_json();
        if i == 0 {
            row["j"] = json!([0., 0., 0.]);
            row["normal_impulse_ns"] = json!(0.);
            row["tangent_on_foot_ns"] = json!([0., 0., 0.]);
            row["first_local_ns"] = Value::Null;
        } else {
            let j = |i: usize| -> Result<DVec3> {
                Ok(DVec3::from_array(serde_json::from_value(
                    doc["references"][i]["fine"]["j"].clone(),
                )?))
            };
            row["j"] = json!((j(i)? - j(i - 1)?).to_array());
        }
        frames.push(row);
    }
    fs::create_dir_all(dir)?;
    write_json(
        &dir.join("reference.json"),
        &json!({
            "case":case, "stroke":stroke, "cut":snapshot(&cut), "drive":drive.to_array(),
            "references":doc["references"], "frames":frames, "conditional_pressure_models":line_alternatives, "coarse_conditional_pressure_models":coarse_alternatives,
            "geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF}
        }),
    )
}

/// Keep impulsive and sustained reactions separate, with event times relative
/// to the public restart and contact points in the simulator's world frame.
pub(crate) fn observe(dir: &Path, sdf: &Path, steps: u32) -> Result<()> {
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        &fs::read_to_string(sdf)?,
        sdf.to_str().ok_or("invalid SDF path")?,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    fs::create_dir_all(dir)?;
    let samples = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    if steps > 1 {
        let sink = samples.clone();
        gz_sim_scene::primary_contact::set_refined_observer(&mut sim, move |_, substep| {
            sink.lock().expect("contact observer").push(json!({
                "end_ns":substep.elapsed.as_nanos() as u64,
                "duration_ns":substep.duration.as_nanos() as u64,
                "material":substep.point_span.is_some(),
            }));
        })?;
    }
    let mut rows = Vec::new();
    for step in 1..=steps {
        let start = sim.elapsed();
        sim.step_exact(1)?;
        let receipt = gz_sim_scene::primary_contact::last_refinement(&sim)
            .filter(|r| r.start_time == start && r.sim_time == sim.elapsed());
        if steps == 1 {
            return write_json(
                &dir.join("receipt.json"),
                &record(receipt.ok_or("no refinement")?)?,
            );
        }
        let mut row = match receipt {
            Some(r) => record(r)?,
            None => json!({"native_calls":1,"events":[]}),
        };
        let bins = std::mem::take(&mut *samples.lock().expect("contact observer"));
        if let Some(r) = receipt
            && (bins.len() != r.interval.native_steps as usize
                || bins
                    .last()
                    .is_none_or(|b| b["end_ns"] != (sim.elapsed() - start).as_nanos() as u64))
        {
            return Err("native observations do not cover the committed public interval".into());
        }
        row["native_intervals"] = json!(bins);
        row["refined"] = json!(receipt.is_some());
        row["step"] = json!(step);
        row["tick_start_s"] = json!(start.as_secs_f64());
        rows.push(row);
    }
    write_json(&dir.join("receipts.json"), &json!({"steps":rows}))
}

fn record(r: &gz_sim_scene::primary_contact::Refinement) -> Result<Value> {
    let [pair] = r.requested_pairs.as_slice() else {
        return Err("expected one requested contact pair".into());
    };
    let mut events = vec![];
    let mut spans = vec![];
    for transfer in &r.interval.point_transfers {
        if (transfer.foot, transfer.support) != (pair.foot, pair.support) {
            return Err("unexpected transfer pair".into());
        }
        for arc in &transfer.intervals {
            let start = transfer.start.as_secs_f64();
            if let Some(hit) = arc.impact {
                events.push(json!({
                    "bracket_s":hit.local_bracket.map(|t| start+t),
                    "point_m":hit.point, "normal":hit.normal, "j_ns":hit.impulse_on_foot,
                    "angular_impulse_nms":hit.angular_impulse_about_com,
                    "sticking_impulse":hit.sticking, "lost_energy_j":hit.lost_energy,
                    "opposite_opening_m_s":hit.opposite_opening_speed
                }));
            }
            spans.push(json!({
                "start_s":start+arc.offset,"duration_s":arc.duration,
                "feature":format!("{:?}",arc.feature),"sticking":arc.sticking,
                "end_material_velocity_m_s":arc.end_material_velocity,
                "normal_impulse_ns":arc.normal_impulse,"friction_work_j":arc.friction_work,
                "j_ns":arc.impulse_on_foot, "slip_growth":arc.slip_growth.map(|s|json!({
                    "direction":s.direction.to_array(),"initial_m_s":s.initial_component,"lower_m_s2":s.acceleration
                }))
            }));
        }
    }
    let mut finite_events = vec![];
    for hit in &r.interval.finite_point_impacts {
        let h = hit.resultant;
        finite_events.push(json!({"elapsed_s":h.elapsed.as_secs_f64(),
            "feature":format!("{:?}",hit.feature),"j_ns":h.impulse_on_foot,"point_m":h.point,
            "normal":h.normal,"sticking_impulse":h.sticking,"lost_energy_j":h.dissipated_energy}));
    }
    Ok(json!({
        "tick_duration_s":(r.sim_time-r.start_time).as_secs_f64(),"native_calls":r.interval.native_steps,
        "events":events,"finite_events":finite_events,"transfer_spans":spans,"normal_impulse_ns":pair.normal_impulse,
        "j_ns":(DVec3::from(pair.normal_impulse_on_foot)+DVec3::from(pair.tangent_impulse_on_foot.ok_or("missing tangent impulse")?)).to_array()
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_refined_roll_precedes_two_different_admissible_line_trajectories() -> Result<()> {
        let dir = tempfile::tempdir()?;
        write(dir.path(), 80, 2)?;
        let data: Value =
            serde_json::from_reader(fs::File::open(dir.path().join("reference.json"))?)?;
        let refs = data["references"].as_array().unwrap();
        let resolved = &refs[19];
        assert_eq!(resolved["qualified"], true);
        assert_eq!(resolved["fine"]["events"][4]["feature"], "SupportFace(2)");
        assert_eq!(resolved["fine"]["events"][4]["mode"], "Sliding");
        let at = |r: &Value| r["events"][4]["at_s"].as_f64().unwrap();
        assert!((at(&resolved["fine"]) - at(&resolved["coarse"])).abs() < 1e-11);
        assert!((at(&resolved["fine"]) - 0.009742436127).abs() < 1e-11);
        assert_eq!(data["frames"].as_array().unwrap().len(), 20);
        let vec = |v: &Value| DVec3::from_array(serde_json::from_value(v.clone()).unwrap());
        let study = &data["conditional_pressure_models"];
        let cases = study["alternatives"].as_array().unwrap();
        assert_eq!(cases.len(), 3);
        let mut terminal = vec![];
        for (i, case) in cases.iter().enumerate() {
            let j = vec(&case["j_ns"]);
            let coarse_j =
                vec(&data["coarse_conditional_pressure_models"]["alternatives"][i]["j_ns"]);
            assert!(j.distance(coarse_j) < 1e-11);
            assert!(
                case["normal_ns"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|p| p.as_f64().unwrap() > 0.)
            );
            assert!(
                case["friction_law_residual_ns"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|p| p.as_f64().unwrap() < 2e-15)
            );
            let mesh = &case["continuation"];
            if case["fractions"] == json!([0., 0.5]) {
                assert!(
                    mesh.as_array()
                        .unwrap()
                        .iter()
                        .all(|m| m["end"] == "no two compressive sliding pressures")
                );
                continue;
            }
            let ends = mesh
                .as_array()
                .unwrap()
                .iter()
                .map(|m| {
                    assert_eq!(m["end"], "original_clock");
                    assert_eq!(m["last_time_s"], study["horizon_s"]);
                    assert!(m["min_pressure_n"].as_f64().unwrap() > 4.);
                    assert!(m["min_pressure_n"].as_f64().unwrap().is_finite());
                    assert!(m["minimum_slip_m_s"].as_f64().unwrap() > 1e-3);
                    assert!(m["max_gap_m"].as_f64().unwrap() < 1e-10);
                    let last = m["frames"].as_array().unwrap().last().unwrap();
                    vec(&last["com_m"])
                })
                .collect::<Vec<_>>();
            assert!(ends[0].distance(ends[1]) < 2e-11);
            terminal.push(ends[1]);
        }
        let difference = terminal[0].distance(terminal[1]);
        assert!((2.7e-6..2.9e-6).contains(&difference));
        assert!(vec(&cases[0]["j_ns"]).distance(vec(&cases[2]["j_ns"])) > 4e-5);
        Ok(())
    }
}
