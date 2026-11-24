// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Piecewise rigid motion while the pressure law identifies one carrying
//! endpoint. Stop at the first unproved contact, including genuine two-site loads.
use super::*;
const EVENT_LIMIT: usize = 32;

pub(in crate::touchdown::contact_controls::rebound::weak_study::outer) fn path(
    original: FiniteBoxFlight,
    mut e: FiniteArcResult,
    mut s: FiniteContactSegment,
    horizon: f64,
    n: u32,
) -> Result<Value> {
    let mut time = 0.;
    let mut events = Vec::new();
    let mut j = DVec3::ZERO;
    let mut normal = 0.;
    let mut normal_vector = DVec3::ZERO;
    let mut torque = DVec3::ZERO;
    let mut work = 0.;
    let mut shift = DVec3::ZERO;
    let initial = e;
    let energy = |v: DVec3, w: DVec3| {
        0.5 * (MATERIAL.mass * v.length_squared() + MATERIAL.inertia * w.length_squared())
    };
    let mut failed = Value::Null;
    let end = loop {
        if events.len() >= EVENT_LIMIT {
            break "event_budget";
        }
        let Some((h, boxes)) = outgoing(e, s)? else {
            failed = closure::response(e, s)?;
            failed["line_points_m"] = json!(s.moving_points.map(|p|p.to_array()));
            failed["normal"] = json!(s.normal.to_array());
            break "unproved_line";
        };
        let arm = s.moving_points[h.loaded_end] - e.center;
        let spin = arm.cross(h.impulse);
        j += h.impulse;
        normal += h.impulse.dot(s.normal);
        normal_vector += s.normal * h.impulse.dot(s.normal);
        torque += spin;
        work -= h.lost_energy;
        let f = FiniteBoxFlight {
            moving: e.moving,
            center_of_mass: e.center,
            relative_velocity: e.velocity + h.impulse / MATERIAL.mass,
            angular_velocity: e.angular_velocity + spin / MATERIAL.inertia,
            ..original
        };
        // Save the instantaneous post-impact state even when no subsequent
        // sustained path is admitted. The geometric start has not advanced.
        e.velocity = f.relative_velocity;
        e.angular_velocity = f.angular_velocity;
        let mut row = json!({"at_s":time,"points_m":s.moving_points.map(|p|p.to_array()),"loaded_end":h.loaded_end,"normal":s.normal.to_array(),
            "sticking":h.sticking,"j_ns":h.impulse.to_array(),"loss_j":h.lost_energy,"work_boxes":boxes,
            "opposite_speed_m_s":h.velocity_after[1-h.loaded_end].dot(s.normal)});
        let Some((track, sample)) = FiniteFeatureTrack::outgoing_endpoint(f, s, h.loaded_end)
        else {
            events.push(row);
            break "unproved_outgoing_feature";
        };
        row["feature"] = json!(format!("{:?}", sample.site.feature));
        let Some(load) = f.first_point_load(sample.site, MATERIAL, h.sticking)? else {
            events.push(row);
            break "unproved_load";
        };
        row["mode"] = json!(format!("{:?}", load.mode));
        row["force_n"] = json!(load.force.to_array());
        let Some(arc) =
            FiniteFeatureArc::after_line_endpoint(f, s, h.loaded_end, MATERIAL, h.sticking, 1e-10)?
        else {
            events.push(row);
            break "untracked_load";
        };
        let a = arc.measure_to(horizon - time, n, 1e-17)?;
        row["arc"] = record(a, f);
        events.push(row);
        j += a.impulse;
        normal += a.normal_impulse;
        normal_vector += a.normal_impulse_vector;
        torque += a.angular_impulse;
        work += a.friction_work;
        shift += a.initial_translation * sample.site.normal;
        let prev = time;
        time += a.bracket[0];
        e = a;
        match a.end {
            FiniteArcEnd::Horizon => break "original_clock",
            FiniteArcEnd::LoadMode => break "load_mode_transition",
            FiniteArcEnd::Unresolved => break "integration_unresolved",
            FiniteArcEnd::Feature(limit) => {
                if time <= prev + 16. * f64::EPSILON * horizon {
                    break "indistinguishable_events";
                }
                let Some(line) = track.incident_segment(a.moving, a.site, limit, 1e-10) else {
                    break "unproved_next_line";
                };
                s = line;
            }
        }
    };
    let dv = MATERIAL.mass * (e.velocity - initial.velocity - time * original.acceleration);
    let dw = MATERIAL.inertia * (e.angular_velocity - initial.angular_velocity);
    let gravity = MATERIAL.mass * original.acceleration.dot(e.center - initial.center - shift);
    let balance = energy(e.velocity, e.angular_velocity)
        - energy(initial.velocity, initial.angular_velocity)
        - gravity
        - work;
    Ok(
        json!({"subdivisions":n,"events":events,"end":end,"elapsed_s":time,"horizon_s":horizon,
        "com_m":e.center.to_array(),"foot_m":e.moving.center.to_array(),"orientation_xyzw":e.moving.orientation.to_array(),"velocity_m_s":e.velocity.to_array(),"spin_rad_s":e.angular_velocity.to_array(),
        "impulse_ns":j.to_array(),"normal_impulse_ns":normal,"normal_impulse_vector_ns":normal_vector.to_array(),"angular_impulse_nms":torque.to_array(),"contact_work_j":work,"linear_defect_ns":dv.distance(j),"angular_defect_nms":dw.distance(torque),"energy_defect_j":balance,"unresolved_line":failed}),
    )
}
fn measure(f: FiniteBoxFlight, clocked: bool) -> Result<Value> {
    let (mut out, start) = locate(f, true)?;
    let Some((e, s)) = start else { return Ok(out) };
    let t = out["geometry"]["hi_ns"].as_u64().ok_or("missing clock")? as f64 * 1e-9 + e.bracket[0];
    let offset = (t * 1e9).round() as u64;
    let horizon = if clocked {
        (STEP_US * 1000 - offset) as f64 * 1e-9
    } else {
        STEP_US as f64 * 1e-6 - t
    };
    out["offset_ns"] = json!(offset);
    let mut paths = vec![];
    for n in if clocked {
        [32, 128, 512]
    } else {
        [8, 32, 128]
    } {
        paths.push(path(f, e, s, horizon, n)?);
    }
    out["paths"] = json!(paths);
    Ok(out)
}
pub(crate) fn write(dir: &Path, first: usize, count: usize, clocked: bool) -> Result<()> {
    survey::write_cases(
        dir,
        first,
        count,
        if clocked {
            "finite-clocked-endpoint-sequence-v1"
        } else {
            "finite-implicit-endpoint-sequence-v2"
        },
        |f| {
            let (cuts, disjoint) = survey::cuts(f)?;
            let mut trials = vec![];
            for (i, c) in cuts.into_iter().enumerate() {
                if let Some(c) = c {
                    let (flight, defect) = kinematics(f, &c)?;
                    if defect > 1e-8 {
                        return Err("invalid original twist".into());
                    }
                    trials
                        .push(json!({"stroke":i+1,"cut":snapshot(&c),"response":measure(flight,clocked)?}))
                }
            }
            Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
        },
    )
}
