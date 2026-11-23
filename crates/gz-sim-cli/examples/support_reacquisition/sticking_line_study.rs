// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Test if the angular-momentum lower bound can be attained by compressive
//! non-slipping pressure on the actual line at the next impact.
use super::*;
fn measure_line(f: FiniteBoxFlight) -> Result<Value> {
    let (mut out, hit) = locate(f, true)?;
    let Some((end, segment)) = hit else {
        return Ok(out);
    };
    let response = segment.maximum_dissipation_stick(
        end.center,
        end.velocity,
        end.angular_velocity,
        MATERIAL.mass,
        glam::DMat3::from_diagonal(DVec3::splat(MATERIAL.inertia)),
        MATERIAL.friction,
    )?;
    use gz_physics::StickingLine::*;
    out["segment"] = json!({"points":segment.moving_points.map(|p|p.to_array()),"normal":segment.normal.to_array()});
    let mode = match response {
        NoApproach => "NoApproach",
        Tensile => "Tensile",
        OutsideFriction => "OutsideFriction",
        Marginal => "Marginal",
        Attainable(_) => "Attainable",
    };
    out["classification"] = json!(mode);
    if let Attainable(h) = response {
        if segment
            .single_loaded_endpoint(end.center, end.velocity, end.angular_velocity, MATERIAL)?
            .is_some()
        {
            return Err("full stick feasible despite forced endpoint opening".into());
        }
        let j = h.equivalent;
        out["attained"] = json!({"v":h.velocity.to_array(),"w":h.angular_velocity.to_array(),"j_ns":h.impulse.to_array(),"torque_nms":h.angular_impulse.to_array(),
            "loss_j":h.dissipated_energy,"longitudinal_ns":j.longitudinal_total,"transverse_ns":j.transverse,"normal_ns":j.normal,"first_range_ns":j.first_longitudinal_range});
        let xml = sdf(end, f.support)?;
        let mut native = vec![];
        for (ns, r) in [(1, 1), (1, 8), (128, 1), (128, 8), (2048, 1), (2048, 8)] {
            native.push(cold(&xml, Duration::from_nanos(ns), r, end)?)
        }
        out["native"] = json!(native);
    }
    Ok(out)
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(dir, first, count, "finite-line-sticking-floor-v1", |f| {
        let (cuts, disjoint) = survey::cuts(f)?;
        let mut rows = vec![];
        for (i, cut) in cuts.into_iter().enumerate() {
            if let Some(c) = cut {
                let (flight, defect) = kinematics(f, &c)?;
                if defect > 1e-8 {
                    return Err("lost incoming rigid twist".into());
                }
                rows.push(json!({"stroke":i+1,"cut":snapshot(&c),"response":measure_line(flight)?}))
            }
        }
        Ok(json!({"fixture":f,"disjoint":disjoint,"trials":rows}))
    })
}
