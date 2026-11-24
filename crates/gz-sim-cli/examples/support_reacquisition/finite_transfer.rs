// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reintegrate the first bound spatial contact at three different mesh sizes.
use super::*;
#[path = "second_point.rs"]
pub(crate) mod second;
use gz_physics::{FiniteArcResult, FiniteFeatureArc, FiniteFeatureTrack};

fn record(s: FiniteArcResult, f: FiniteBoxFlight) -> Value {
    let t = s.bracket[0];
    let a = f.acceleration;
    let linear = MATERIAL.mass * (s.velocity - f.relative_velocity - t * a) - s.impulse;
    let angular = MATERIAL.inertia * (s.angular_velocity - f.angular_velocity) - s.angular_impulse;
    json!({"end":format!("{:?}",s.end),"elapsed_interval_s":s.bracket,"com_m":s.center.to_array(),"foot_m":s.moving.center.to_array(),
        "orientation_xyzw":s.moving.orientation.to_array(),"velocity_m_s":s.velocity.to_array(),"spin_rad_s":s.angular_velocity.to_array(),
        "point_m":s.site.site.point.to_array(),"normal":s.site.site.normal.to_array(),"signed_gap_m":s.site.gap,"normal_speed_m_s":s.normal_velocity_defect,
        "linear_impulse_ns":s.impulse.to_array(),"angular_impulse_nms":s.angular_impulse.to_array(),"linear_balance_ns":linear.length(),"angular_balance_nms":angular.length(),
        "friction_work_j":s.friction_work,"energy_balance_j":s.energy_defect,"initial_closure_m":s.initial_translation})
}
fn measure(f: FiniteBoxFlight) -> Result<Value> {
    let geo = geometry(f)?;
    let Some(d) = departure(f, &geo)? else {
        return Ok(json!({"geometry":geo}));
    };
    let s = FiniteFeatureTrack::new(d.after, d.site)?
        .sample(
            d.after.moving,
            d.after.center_of_mass,
            d.after.relative_velocity,
            d.after.angular_velocity,
        )
        .ok_or("missing initial finite feature")?;
    let hint = s
        .approaching_limit()
        .map(|(l, t)| json!({"limit":format!("{l:?}"),"time_s":t}));
    let mut r = json!({"geometry":geo,"initial_margins":s.margins().iter().map(|x|json!({"limit":format!("{:?}",x.limit),"m":x.distance,"m_s":x.speed})).collect::<Vec<_>>(),"linearized":hint});
    let Some(arc) = FiniteFeatureArc::new(
        d.after,
        d.site,
        MATERIAL,
        geo["sticking"].as_bool().unwrap(),
        d.allowance + 1e-12,
    )?
    else {
        return Ok(r);
    };
    let t = (Duration::from_micros(STEP_US) - d.hit.latest).as_secs_f64();
    let mut runs = vec![];
    for n in [8, 32, 128] {
        let end = arc.measure(t, n)?;
        let mut z = record(end, d.after);
        z["subdivisions"] = json!(n);
        runs.push(z);
    }
    r["runs"] = json!(runs);
    Ok(r)
}
fn collect(f: Fixture) -> Result<Value> {
    let (cuts, disjoint) = survey::cuts(f)?;
    let mut trials = vec![];
    for (i, cut) in cuts.into_iter().enumerate() {
        if let Some(c) = cut {
            let (flight, defect) = kinematics(f, &c)?;
            if defect > 1e-8 {
                return Err("mismatched locked state".into());
            }
            trials.push(json!({"stroke":i+1,"cut":snapshot(&c),"response":measure(flight)?}));
        }
    }
    Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(
        dir,
        first,
        count,
        "finite-first-contact-transfer-v1",
        collect,
    )
}
