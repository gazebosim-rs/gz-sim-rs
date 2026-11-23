// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Start with the original pre-primary rigid donor. Continue on the physical
//! clock through its first flight, point, sustained arcs and later endpoints.
use super::*;
use gz_physics::{FiniteArcEnd, FiniteFeatureArc, FiniteFeatureTrack};
fn vector(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(v.clone())?))
}
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer) fn at(
    f: FiniteBoxFlight,
    geo: &Value,
    total: f64,
    n: u32,
) -> Result<Value> {
    Ok(at_with_line_diagnostics(f, geo, total, n)?.0)
}

/// Keep the failed distributed-pressure check available to a reference that
/// stopped at a line impact. Historical callers consume the trajectory alone.
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer) fn at_with_line_diagnostics(
    f: FiniteBoxFlight,
    geo: &Value,
    total: f64,
    n: u32,
) -> Result<(Value, Value)> {
    at_with_feature_history(f, geo, total, n, None)
}

/// Optionally retain the individual arcs, including their state immediately
/// before the next impact. The first impulse and arc precede this sequence.
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer) fn at_with_feature_history(
    f: FiniteBoxFlight,
    geo: &Value,
    total: f64,
    n: u32,
    history: Option<&mut Value>,
) -> Result<(Value, Value)> {
    let mut unresolved_line = Value::Null;
    let Some(d) = departure(f, geo)? else {
        return Ok((json!({"end":"no_unique_primary"}), unresolved_line));
    };
    let first = d.hit.latest.as_secs_f64();
    if total < first {
        return Ok((
            json!({"end":"flight","com":f.com_at(total).to_array(),"velocity":(f.relative_velocity+f.acceleration*total).to_array(),"omega":f.angular_velocity.to_array(),"orientation":f.at(total).orientation.to_array(),"foot":f.at(total).center.to_array(),"j":[0.,0.,0.],"jn":0.,"normal_vector":[0.,0.,0.],"elapsed":total}),
            unresolved_line,
        ));
    }
    let jp = vector(&geo["instantaneous_impulse_ns"])?;
    let n0 = d.hit.normal;
    let mut j = jp;
    let mut normal = jp.dot(n0);
    let mut normal_vector = normal * n0;
    let mut angular = (d.site.point - d.after.center_of_mass).cross(jp);
    let mut work = -geo["lost_energy_j"]
        .as_f64()
        .ok_or("missing primary dissipation")?;
    let arc = FiniteFeatureArc::new(
        d.after,
        d.site,
        MATERIAL,
        geo["sticking"].as_bool().ok_or("unknown primary mode")?,
        d.allowance + 1e-12,
    )?;
    let Some(arc) = arc else {
        return Ok((json!({"end":"untracked_primary"}), unresolved_line));
    };
    let a = arc.measure_to(total - first, n, 1e-17)?;
    j += a.impulse;
    normal += a.normal_impulse;
    normal_vector += a.normal_impulse_vector;
    angular += a.angular_impulse;
    work += a.friction_work;
    let mut com = a.center;
    let mut foot = a.moving.center;
    let mut q = a.moving.orientation;
    let mut v = a.velocity;
    let mut w = a.angular_velocity;
    let mut events = vec![
        json!({"at_s":first,"j":jp.to_array(),"point":d.site.point.to_array(),"normal":n0.to_array(),"feature":geo["feature"],"stuck":geo["sticking"]}),
    ];
    let mut closure_checked_energy = a.energy_defect;
    let mut elapsed = first + a.bracket[0];
    let end = match a.end {
        FiniteArcEnd::Horizon => "original_clock".to_owned(),
        FiniteArcEnd::Feature(limit) => {
            let track = FiniteFeatureTrack::new(d.after, d.site)?;
            let Some(s) = track.incident_segment(a.moving, a.site, limit, 1e-10) else {
                return Ok((
                    json!({"end":"unidentified_secondary","elapsed":elapsed}),
                    unresolved_line,
                ));
            };
            let extra =
                transfer::second::after::sequence::path(f, a, s, (total - elapsed).max(0.), n)?;
            if let Some(history) = history {
                *history = json!({"start_s":elapsed,"preceding_impulse_ns":j.to_array(),"events":extra["events"]});
            }
            unresolved_line = extra["unresolved_line"].clone();
            for e in extra["events"].as_array().ok_or("no event history")? {
                events.push(json!({"at_s":elapsed+e["at_s"].as_f64().ok_or("no event clock")?,"j":e["j_ns"],"point":e["points_m"][e["loaded_end"].as_u64().ok_or("no loaded end")? as usize],"normal":e["normal"],"feature":e["feature"],"mode":e["mode"],"stuck":e["sticking"]}));
            }
            j += vector(&extra["impulse_ns"])?;
            normal += extra["normal_impulse_ns"]
                .as_f64()
                .ok_or("missing normal")?;
            normal_vector += vector(&extra["normal_impulse_vector_ns"])?;
            angular += vector(&extra["angular_impulse_nms"])?;
            work += extra["contact_work_j"].as_f64().ok_or("missing work")?;
            com = vector(&extra["com_m"])?;
            foot = vector(&extra["foot_m"])?;
            q = DQuat::from_array(serde_json::from_value(extra["orientation_xyzw"].clone())?);
            v = vector(&extra["velocity_m_s"])?;
            w = vector(&extra["spin_rad_s"])?;
            closure_checked_energy += extra["energy_defect_j"]
                .as_f64()
                .ok_or("no energy balance")?;
            elapsed += extra["elapsed_s"].as_f64().ok_or("no endpoint clock")?;
            extra["end"].as_str().ok_or("unknown endpoint")?.to_owned()
        }
        e => format!("{e:?}"),
    };
    let energy = |v: DVec3, w: DVec3| {
        0.5 * (MATERIAL.mass * v.length_squared() + MATERIAL.inertia * w.length_squared())
    };
    Ok((
        json!({"end":end,"elapsed":elapsed,"events":events,"com":com.to_array(),"foot":foot.to_array(),"orientation":q.to_array(),"velocity":v.to_array(),"omega":w.to_array(),"j":j.to_array(),"jn":normal,"normal_vector":normal_vector.to_array(),"angular":angular.to_array(),"work_j":work,
        "linear_defect":(MATERIAL.mass*(v-f.relative_velocity-elapsed*f.acceleration)-j).length(),"angular_defect":(MATERIAL.inertia*(w-f.angular_velocity)-angular).length(),"arc_energy_defect":closure_checked_energy,"energy_defect":energy(v,w)-energy(f.relative_velocity,f.angular_velocity)-MATERIAL.mass*f.acceleration.dot(com-f.center_of_mass)-work}),
        unresolved_line,
    ))
}
