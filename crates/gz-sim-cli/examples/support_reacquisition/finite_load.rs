// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure whether the actual first point opens or continues to carry load.
//! The two pinned material features also give a geometric finite-difference
//! control for the free normal acceleration, independent of the force law.
use super::*;
#[path = "finite_transfer.rs"]
pub(crate) mod transfer;
use gz_physics::{FiniteLoadMode, FinitePointSite, SeparatingFeature, SphericalPointMaterial};
#[path="complete_primary.rs"]
pub(crate) mod complete;
const FORMAT: &str = "finite-deck-postimpact-v1";
pub(in crate::touchdown::contact_controls::rebound::weak_study::outer) const MATERIAL: SphericalPointMaterial = SphericalPointMaterial {
    mass: 2.,
    inertia: 0.05,
    friction: 0.5,
};

#[derive(Clone, Copy)]
struct Departure {
    after: FiniteBoxFlight,
    hit: gz_physics::BoxCrossing,
    site: FinitePointSite,
    allowance: f64,
}
fn departure(f: FiniteBoxFlight, geo: &Value) -> Result<Option<Departure>> {
    let FiniteBoxArrival::Crossing(hit) = f.first_crossing(Duration::from_micros(STEP_US))? else {
        return Ok(None);
    };
    let Some([point, _]) = hit.points else {
        return Ok(None);
    };
    let Some(impulse) = geo["instantaneous_impulse_ns"].as_array() else {
        return Ok(None);
    };
    let j = DVec3::from_array(std::array::from_fn(|i| impulse[i].as_f64().unwrap()));
    let t = hit.latest.as_secs_f64();
    let center = f.com_at(t);
    let r = point - center;
    let incoming = f.relative_velocity + t * f.acceleration;
    let v = incoming + j / MATERIAL.mass;
    let w = f.angular_velocity + r.cross(j) / MATERIAL.inertia;
    let moving = f.at(t);
    let after = FiniteBoxFlight {
        moving,
        center_of_mass: center,
        relative_velocity: v,
        angular_velocity: w,
        ..f
    };
    let site = FinitePointSite {
        point,
        normal: hit.normal,
        feature: hit.feature,
    };
    let dur = (hit.latest - hit.earliest).as_secs_f64();
    let speed = f.relative_velocity.length()
        + f.acceleration.length() * t
        + f.angular_velocity.length()
            * (f.moving.center.distance(f.center_of_mass) + f.moving.half_extent.length());
    let allowance = 1.000001 * dur * speed;
    Ok(Some(Departure {
        after,
        hit,
        site,
        allowance,
    }))
}
fn measured(f: FiniteBoxFlight) -> Result<Value> {
    let geo = geometry(f)?;
    let Some(Departure {
        after,
        hit,
        site,
        allowance,
    }) = departure(f, &geo)?
    else {
        return Ok(json!({"geometry":geo}));
    };
    let center = after.center_of_mass;
    let v = after.relative_velocity;
    let w = after.angular_velocity;
    let moving = after.moving;
    let r = site.point - center;
    let accepted = after.contact_geometry(allowance).is_some();
    let mut evidence = json!({"geometry":geo,"preflight_spatial_allowance_m":allowance,"transaction_geometry_admitted":accepted,"outgoing_com_relative_m_s":v.to_array(),"outgoing_spin_rad_s":w.to_array(),"com_at_contact_m":center.to_array()});
    let Some(load) = after.first_point_load(site, MATERIAL, geo["sticking"].as_bool().unwrap())?
    else {
        return Ok(evidence);
    };
    let n = hit.normal;
    let axes = [DVec3::X, DVec3::Y, DVec3::Z];
    let gap = |s: f64| {
        // The initially coincident material markers each lie on both extended
        // feature planes for all later times. For two lines use their common
        // normal. They need not subsequently coincide or remain inside the box.
        let delta = DQuat::from_scaled_axis(w * s);
        let d = (v + 0.5 * s * after.acceleration) * s + (delta * r - r);
        let n1 = match hit.feature {
            SeparatingFeature::SupportFace(_) => n,
            SeparatingFeature::MovingFace(_) => delta * n,
            SeparatingFeature::EdgePair(i, k) => {
                let a = delta * (moving.orientation * axes[i]);
                let b = f.support.orientation * axes[k];
                let nn = a.cross(b).normalize();
                nn * n.dot(nn).signum()
            }
        };
        n1.dot(d)
    };
    let accel = |h: f64| (gap(h) + gap(-h)) / (h * h);
    let h = 12e-6;
    let residual = ((4. * accel(h) - accel(2. * h)) / 3. - load.free_normal_acceleration).abs();
    let post = load.relative_point_velocity;
    let force = load.force;
    let jn = force.dot(n);
    let force_t = force - n * jn;
    let ap = f.acceleration
        + w.cross(w.cross(r))
        + force / MATERIAL.mass
        + r.cross(force).cross(r) / MATERIAL.inertia;
    let mut vertex_motion = vec![];
    // Diagnostic normal heights in the support frame; useful for detecting a
    // competing sole vertex on the upper face, even for a skew-edge impact.
    let z = f.support.orientation * DVec3::Z;
    let top = f.support.center + z * f.support.half_extent.z;
    for i in 0..8 {
        let local = moving.half_extent
            * DVec3::new(
                if i & 1 == 0 { -1. } else { 1. },
                if i & 2 == 0 { -1. } else { 1. },
                if i & 4 == 0 { -1. } else { 1. },
            );
        let x = moving.center + moving.orientation * local;
        let projected = f.support.orientation.inverse() * (x - f.support.center);
        vertex_motion.push(json!({"corner":i,"face_gap_m":(x-top).dot(z),"normal_velocity_m_s":z.dot(v+w.cross(x-center)),"inside_face_xy":projected.x.abs()<=f.support.half_extent.x && projected.y.abs()<=f.support.half_extent.y}));
    }
    evidence["load"] = json!({"mode":format!("{:?}",load.mode),"force_n":force.to_array(),"normal_n":jn,"friction_margin_n":MATERIAL.friction*jn-force_t.length(),"slip_power_w":force.dot(post),"material_velocity_m_s":post.to_array(),"normal_rate_s":load.normal_rate.to_array(),"free_gap_acceleration_m_s2":load.free_normal_acceleration,"geometric_acceleration_residual_m_s2":residual,"loaded_gap_acceleration_m_s2":load.constrained_normal_acceleration,"material_acceleration_m_s2":ap.to_array(),"vertex_motion":vertex_motion});
    if load.mode != FiniteLoadMode::Opening
        && (load.constrained_normal_acceleration.abs() > 1e-8 || force.dot(post) > 1e-8)
    {
        return Err("invalid postimpact constraint".into());
    }
    Ok(evidence)
}
fn collect(f: Fixture) -> Result<Value> {
    let (cuts, disjoint) = survey::cuts(f)?;
    let mut trials = vec![];
    for (i, cut) in cuts.into_iter().enumerate() {
        if let Some(c) = cut {
            let (flight, defect) = kinematics(f, &c)?;
            if defect > 1e-8 {
                return Err("inconsistent rigid donor".into());
            }
            trials.push(json!({"stroke":i+1,"cut":snapshot(&c),"response":measured(flight)?}));
        }
    }
    Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(dir, first, count, FORMAT, collect)
}
