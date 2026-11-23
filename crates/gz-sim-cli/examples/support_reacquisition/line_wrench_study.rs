// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct the equivalent continuous-line wrench from a regular native
//! contact solve at matched incoming states. Each duration is its own reload.
use super::*;
use gz_physics::{FiniteArcResult, LineWrench};
fn vector(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(v.clone())?))
}
fn audit(
    s: FiniteContactSegment,
    e: FiniteArcResult,
    j: DVec3,
    post: DVec3,
    spin: DVec3,
) -> Result<Value> {
    let torque = MATERIAL.inertia * (spin - e.angular_velocity);
    let speeds = s.moving_points.map(|p| post + spin.cross(p - e.center));
    let LineWrench {
        normal,
        transverse,
        longitudinal_total,
        free_couple,
        required_cone_slack,
        normal_speed,
        normal_work,
        slip_dissipation,
        coulomb_dissipation,
    } = s.audit_wrench(e.center, j, torque, speeds, MATERIAL.friction)?;
    Ok(
        json!({"normal_ns":normal,"transverse_ns":transverse,"longitudinal_ns":longitudinal_total,
        "couple_nms":free_couple,"cone_slack_ns":required_cone_slack,"normal_speed_m_s":normal_speed,"normal_work_j":normal_work,"friction_after_j":slip_dissipation,
        "coulomb_dissipation_j":coulomb_dissipation,"post_v":post.to_array(),"post_w":spin.to_array(),"j_ns":j.to_array(),"moment_nms":torque.to_array()}),
    )
}
fn measure_wrench(f: FiniteBoxFlight) -> Result<Value> {
    let (mut result, hit) = locate(f, true)?;
    let Some((e, s)) = hit else { return Ok(result) };
    let goals = s.single_loaded_endpoint(e.center, e.velocity, e.angular_velocity, MATERIAL)?;
    result["segment"] =
        json!({"points":s.moving_points.map(|p|p.to_array()),"normal":s.normal.to_array()});
    if let Some(h) = goals {
        result["endpoint"] = audit(
            s,
            e,
            h.impulse,
            e.velocity + h.impulse / MATERIAL.mass,
            e.angular_velocity
                + (s.moving_points[h.loaded_end] - e.center).cross(h.impulse) / MATERIAL.inertia,
        )?;
    }
    let xml = sdf(e, f.support)?;
    let mut runs = vec![];
    for ns in [1, 128, 2048] {
        for r in [1, 8, 32] {
            let raw = cold(&xml, Duration::from_nanos(ns), r, e)
                .map_err(|err| format!("cold {ns} ns, {r} relaxation passes: {err}"))?;
            let j = vector(&raw["j_ns"])?;
            let v = vector(&raw["after_v"])?;
            let w = vector(&raw["after_w"])?;
            // Remove the known gravity impulse before comparing an integrated
            // native interval to an instantaneous rigid-line impact. Show the
            // uncorrected native state alongside this frozen-geometry diagnostic.
            let v_contact = v + DVec3::Z * (9.81 * ns as f64 * 1e-9);
            let row = audit(s, e, j, v_contact, w)?;
            runs.push(json!({"duration_ns":ns,"relaxation":r,"raw":raw,"frozen_line":row}));
        }
    }
    result["runs"] = json!(runs);
    Ok(result)
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(dir, first, count, "finite-line-native-wrench-v1", |f| {
        let (cuts, disjoint) = survey::cuts(f)?;
        let mut trials = vec![];
        for (i, c) in cuts.into_iter().enumerate() {
            if let Some(c) = c {
                let g = kinematics(f, &c)?;
                if g.1 > 1e-8 {
                    return Err("missing matched incoming state".into());
                }
                trials
                    .push(json!({"cut":snapshot(&c),"stroke":i+1,"response":measure_wrench(g.0)?}))
            }
        }
        Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
    })
}
