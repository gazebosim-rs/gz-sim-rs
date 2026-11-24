// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Follow the outgoing pressure-bearing point for the remainder of the original
//! actuator tick. Keep declined geometry, load modes and mesh dependence visible.
#[path = "contact_sequence.rs"]
pub(crate) mod sequence;
use super::*;
use gz_physics::{EdgeMobility, FiniteArcResult, generalized_segment_work_impact};

fn outgoing(
    e: FiniteArcResult,
    seg: FiniteContactSegment,
) -> Result<Option<(gz_physics::SegmentPointImpact, u32)>> {
    let axes = [DVec3::X, DVec3::Y, DVec3::Z];
    let r = seg.moving_points.map(|p| p - e.center);
    let w: EdgeMobility = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            std::array::from_fn(|a| {
                std::array::from_fn(|b| {
                    axes[a].dot(
                        axes[b] / MATERIAL.mass
                            + r[j].cross(axes[b]).cross(r[i]) / MATERIAL.inertia,
                    )
                })
            })
        })
    });
    let before = r.map(|p| e.velocity + e.angular_velocity.cross(p));
    Ok(generalized_segment_work_impact(
        w,
        before,
        seg.normal,
        r[1] - r[0],
        MATERIAL.friction,
        128,
    )?)
}
fn measure(f: FiniteBoxFlight) -> Result<Value> {
    let (mut out, located) = locate(f, true)?;
    let Some((e, seg)) = located else {
        return Ok(out);
    };
    let (h, boxes) = outgoing(e, seg)?.ok_or("unresolved secondary impulse")?;
    let r = seg.moving_points[h.loaded_end] - e.center;
    let after = FiniteBoxFlight {
        moving: e.moving,
        center_of_mass: e.center,
        relative_velocity: e.velocity + h.impulse / MATERIAL.mass,
        angular_velocity: e.angular_velocity + r.cross(h.impulse) / MATERIAL.inertia,
        ..f
    };
    let elapsed = out["geometry"]["hi_ns"]
        .as_u64()
        .ok_or("no primary clock")? as f64
        * 1e-9
        + e.bracket[0];
    let remaining = STEP_US as f64 * 1e-6 - elapsed;
    if remaining <= 0. {
        return Err("secondary beyond original clock".into());
    }
    out["secondary"] = json!({"point_m":seg.moving_points[h.loaded_end].to_array(),"normal":seg.normal.to_array(),"loaded_end":h.loaded_end,"impulse_ns":h.impulse.to_array(),
        "sticking":h.sticking,"work_boxes":boxes,"remaining_s":remaining,"other_speed_m_s":h.velocity_after[1-h.loaded_end].dot(seg.normal)});
    let Some((_, s)) = FiniteFeatureTrack::outgoing_endpoint(after, seg, h.loaded_end) else {
        return Ok(out);
    };
    out["outgoing"] = json!({"feature":format!("{:?}",s.site.feature),"normal":s.site.normal.to_array(),"point_m":s.site.point.to_array(),
        "margins":s.margins().iter().map(|v|json!({"limit":format!("{:?}",v.limit),"distance_m":v.distance,"rate_m_s":v.speed})).collect::<Vec<_>>()});
    let Some(l) = after.first_point_load(s.site, MATERIAL, h.sticking)? else {
        return Ok(out);
    };
    out["load"] = json!({"mode":format!("{:?}",l.mode),"force_n":l.force.to_array(),"free_m_s2":l.free_normal_acceleration,"constrained_m_s2":l.constrained_normal_acceleration});
    let Some(arc) = FiniteFeatureArc::after_line_endpoint(
        after,
        seg,
        h.loaded_end,
        MATERIAL,
        h.sticking,
        1e-10,
    )?
    else {
        return Ok(out);
    };
    let mut rows = vec![];
    for n in [8, 32, 128] {
        let a = arc.measure_to(remaining, n, 1e-16)?;
        let mut row = record(a, after);
        row["subdivisions"] = json!(n);
        rows.push(row);
    }
    out["arc"] = json!(rows);
    Ok(out)
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(dir, first, count, "finite-after-secondary-point-v1", |f| {
        let (cuts, disjoint) = survey::cuts(f)?;
        let mut trials = vec![];
        for (i, c) in cuts.into_iter().enumerate() {
            if let Some(c) = c {
                let (flight, defect) = kinematics(f, &c)?;
                if defect > 1e-8 {
                    return Err("invalid incoming twist".into());
                }
                trials.push(json!({"stroke":i+1,"cut":snapshot(&c),"response":measure(flight)?}));
            }
        }
        Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
    })
}
