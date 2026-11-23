// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Prescribed clock-phase shifts of the very same material landing.
use super::*;
const PHASES: u64 = 64;
pub(super) fn run(studies: &Path, out: &Path) -> Result<()> {
    let inputs = studies.join("multifidelity-secondary-return-features/cases");
    let dir = out.join("cases");
    fs::create_dir_all(&dir)?;
    for entry in fs::read_dir(inputs)? {
        let path = entry?.path();
        if path.extension().is_none_or(|s| s != "json") {
            continue;
        }
        let donor = read(&path)?;
        let (input, deck) = flight(&donor)?;
        let (_, Some(original)) = onset(input)? else {
            return Err("missing original material point".into());
        };
        let origin_ns = original.t * 1e9;
        let mut rows = vec![];
        for k in 0..=PHASES {
            let shift = if k == PHASES {
                0.
            } else {
                (origin_ns.fract() - (k as f64 + 0.5) / PHASES as f64) * 1e-9
            };
            let f = FiniteBoxFlight {
                moving: input.at(shift),
                center_of_mass: input.com_at(shift),
                relative_velocity: input.relative_velocity + input.acceleration * shift,
                ..input
            };
            let (metadata, Some(o)) = onset(f)? else {
                return Err("shifted first impact unresolved".into());
            };
            let FiniteBoxArrival::Crossing(hit) = f.first_crossing(Duration::from_micros(500))?
            else {
                return Err("shifted native bracket missing".into());
            };
            let state_mismatch = o
                .flight
                .center_of_mass
                .distance(original.flight.center_of_mass);
            let impulse_mismatch = o.impulse.distance(original.impulse);
            let phase_error = (o.t + shift - original.t).abs() * 1e9;
            if o.site.feature != original.site.feature
                || state_mismatch > 1e-12
                || impulse_mismatch > 1e-10
                || phase_error > 1e-4
            {
                return Err(format!(
                    "inconsistent phase translation {:?} {k} {phase_error} {impulse_mismatch}",
                    donor["case"]
                )
                .into());
            }
            let until = hit.latest.as_secs_f64();
            let edge = f.isolated_skew_edges(hit);
            let dynamic = f.single_skew_impact_in_bracket(hit, MAT)?;
            let staged=f.fractional_point_path(hit,MAT)?.map(|p|{
                let end=p.follow.last().unwrap().motion;
                json!({"first_s":p.first_time,"bracket_s":p.crossing_bracket,"feature":format!("{:?}",p.first_site.feature),"first_impulse_ns":p.first.impulse.to_array(),"total_ns":p.impulse().to_array(),"com_m":(end.center+deck*hit.latest.as_secs_f64()).to_array(),"angular_rad_s":end.angular_velocity.to_array(),
                    "events":p.follow.iter().filter_map(|s|s.impact.map(|e|json!({"time_s":p.first_time+s.start+s.motion.bracket[0],"j_ns":e.law.impulse.to_array()}))).collect::<Vec<_>>()})
            });
            let face = f.isolated_face_vertex(hit);
            let speed = f.relative_velocity.length()
                + f.acceleration.length() * until
                + f.angular_velocity.length()
                    * (f.moving.center.distance(f.center_of_mass) + f.moving.half_extent.length());
            let committed = edge
                .and_then(|w| w.measure(f.at(until), f.support, speed * 1e-9))
                .or_else(|| face.and_then(|w| w.measure(f.at(until), f.support, speed * 1e-9)));
            let low = trace(f, o, deck, until, 16)?;
            let high = trace(f, o, deck, until, 128)?;
            let complete = low["status"] == "complete" && high["status"] == "complete";
            let a = &low["end"];
            let b = &high["end"];
            let match_events = low["events"]
                .as_array()
                .zip(high["events"].as_array())
                .is_some_and(|(a, b)| {
                    a.len() == b.len()
                        && a.iter().zip(b).all(|(x, y)| {
                            (x["time_s"].as_f64().unwrap() - y["time_s"].as_f64().unwrap()).abs()
                                < 2e-14
                        })
                });
            let agreed = complete
                && match_events
                && v(&a["total_impulse_ns"])?.distance(v(&b["total_impulse_ns"])?) < 1e-7;
            let (excess, measured, events) = if agreed {
                (
                    Some((v(&b["total_impulse_ns"])? - o.impulse).length()),
                    b["total_impulse_ns"].clone(),
                    high["events"].clone(),
                )
            } else {
                (None, Value::Null, high["events"].clone())
            };
            rows.push(json!({"staged":staged,"end_reference_com_m":high["end"]["com_m"],"end_reference_spin_rad_s":high["end"]["spin_rad_s"],"phase":(k<PHASES).then_some(k),"shift_s":shift,"onset_ns":o.t*1e9,"upper_ns":hit.latest.as_nanos() as u64,"initial_point_m":o.site.point.to_array(),"initial_impulse_ns":o.impulse.to_array(),"phase_translation_error_ns":phase_error,"initial_impulse_translation_error_ns":impulse_mismatch,"center_translation_error_m":state_mismatch,"feature":metadata["feature"],"first_contact_unique":hit.points.is_some(),"face_certified":face.is_some(),"edge_certified":edge.is_some(),"single_outgoing_skew":dynamic.is_some(),"integer_geometry_rechecked":committed.is_some(),"end_status":high["status"],"both_meshes_agree":agreed,"events":events,"end_impulse_ns":measured,"first_only_discrepancy_ns":excess}));
        }
        write(
            &dir.join(path.file_name().unwrap()),
            &json!({"case":donor["case"],"slot":donor["slot"],"held":donor["held"],"cut_tick":donor["cut_tick"],"rows":rows}),
        )?;
    }
    Ok(())
}
