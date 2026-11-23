// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Geometric first arrival on the actual finite deck from matched native donors.
use super::*;
#[path = "finite_load.rs"]
pub(crate) mod load;
#[path = "finite_policy.rs"]
pub(crate) mod policy;
use gz_physics::{DynamicBackend, FiniteBoxArrival, FiniteBoxFlight, FlightBox, PointImpact};
use gz_physics_multifidelity::ContactSolverIterations;
const GRID: [u32; 5] = [8, 64, 512, 4096, 16384];
const TAG: &str = "finite-deck-matched-primary-v1";
pub(super) fn kinematics(f: Fixture, cut: &ReboundFrame) -> Result<(FiniteBoxFlight, f64)> {
    let [foot, upper, pad] = [cut.foot, cut.upper, cut.pad];
    let q = DQuat::from_array(foot.orientation);
    let u = DQuat::from_array(upper.orientation);
    let box_at = |b: Body, half: [f64; 3]| FlightBox {
        center: b.position.into(),
        half_extent: half.into(),
        orientation: DQuat::from_array(b.orientation),
    };
    let center = com(cut);
    let v = velocity(cut);
    let w = DVec3::from(foot.angular_velocity);
    let defect = [foot, upper]
        .into_iter()
        .map(|b| {
            (v + w.cross(DVec3::from(b.position) - center))
                .distance(b.velocity.into())
                .max(w.distance(b.angular_velocity.into()))
        })
        .fold(0., f64::max);
    // Both half-masses have the authored diagonal (0.0201,0.0201,0.025).
    // Parallel-axis contributions at ±0.07 m make the locked aggregate
    // spherical, I=0.05 kg m². Verify the actual rigid donor configuration.
    if q.dot(u).abs() < 1. - 1e-12
        || (q * DVec3::Z * 0.14).distance(DVec3::from(upper.position) - DVec3::from(foot.position))
            > 1e-9
        || DVec3::from(pad.angular_velocity).length() > 0.
    {
        return Err("invalid locked donor".into());
    }
    Ok((
        FiniteBoxFlight {
            moving: box_at(foot, FOOT_HALF),
            support: box_at(pad, PAD_HALF),
            center_of_mass: center,
            relative_velocity: v - DVec3::Z * command(f, cut.tick),
            angular_velocity: w,
            acceleration: DVec3::Z * (-9.81),
        },
        defect,
    ))
}
pub(super) fn geometry(f: FiniteBoxFlight) -> Result<Value> {
    let dt = Duration::from_micros(STEP_US);
    match f.first_crossing(dt)? {
        FiniteBoxArrival::InitiallyTouching => Ok(json!({"status":"initially_touching"})),
        FiniteBoxArrival::Separated => Ok(json!({"status":"separated"})),
        FiniteBoxArrival::Unresolved { earliest, latest } => Ok(
            json!({"status":"unresolved","lo_ns":earliest.as_nanos()as u64,"hi_ns":latest.as_nanos()as u64}),
        ),
        FiniteBoxArrival::Crossing(hit) => {
            let t = hit.latest.as_secs_f64();
            let mut entry = json!({"status":"crossing","lo_ns":hit.earliest.as_nanos()as u64,"hi_ns":hit.latest.as_nanos()as u64,
                "feature":format!("{:?}",hit.feature),"normal":hit.normal.to_array()});
            if let Some([point, support]) = hit.points {
                let n = hit.normal;
                let a = n.any_orthonormal_vector();
                let basis = [a, n.cross(a), n];
                let r = point - f.com_at(t);
                let v = f.relative_velocity + t * f.acceleration + f.angular_velocity.cross(r);
                let inv = std::array::from_fn(|i| {
                    std::array::from_fn(|j| {
                        basis[i].dot(0.5 * basis[j] + 20. * r.cross(basis[j]).cross(r))
                    })
                });
                let law = PointImpact {
                    inverse_mass: inv,
                    relative_velocity: basis.map(|b| b.dot(v)),
                    friction: 0.5,
                };
                entry["point"] = json!(point.to_array());
                entry["site_gap_m"] = json!((point - support).dot(n));
                entry["normal_speed_m_s"] = json!(n.dot(v));
                if let Ok(solution) = law.solve() {
                    let [x, y, z] = solution.impulse;
                    entry["instantaneous_impulse_ns"] =
                        json!((x * basis[0] + y * basis[1] + z * n).to_array());
                    entry["sticking"] = json!(solution.sticking);
                    entry["lost_energy_j"] = json!(solution.dissipated_energy);
                }
            }
            Ok(entry)
        }
    }
}
fn observed(f: Fixture, cut: &ReboundFrame, calls: u32) -> Result<Value> {
    let mut r = restart(f, ADAPTIVE, cut)?;
    let lane = &mut r.case.lane;
    if lane.tier()? != Dynamic {
        return Err("expected dynamic cut".into());
    }
    let mut res = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let engine = res
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("native solver")?;
    let (a, b) = (lane.foot, lane.pad);
    let speed = command(f, cut.tick);
    if speed != r.last_speed {
        engine.set_world_velocity(b, [0., 0., speed], [0.; 3])?;
    }
    let mut first = None;
    let mut preloaded = DVec3::ZERO;
    let mut preloaded_normal = 0.;
    let mut extra = 0.;
    let mut substep = 0;
    let mut missing_tangent = false;
    let receipt=engine.observe_contact_interval(Duration::from_micros(STEP_US),calls,ContactSolverIterations{
        temporal:4,pgs:None,relaxation:Some(1)},|s|{
        substep+=1;
        let mut j=DVec3::ZERO;
        let mut normal=0.;
        let mut sites=vec![];
        for patch in s.contact_patches(){
            if (patch.a,patch.b)!=(a.min(b),a.max(b)){extra+=patch.points.iter().map(|p|p.normal_impulse).sum::<f64>();continue;}
            let sign=if patch.a==a{1.}else{-1.};
            for p in patch.points{
                if p.normal_impulse<=0.{continue;}
                let Some(tangent)=p.tangent_impulse_on_a else {missing_tangent=true;continue;};
                let impulse=sign*(DVec3::from(patch.normal_on_a)*p.normal_impulse+DVec3::from(tangent));
                if first.is_none(){
                    let lever=if patch.a==a{p.lever_a}else{p.lever_b};
                    sites.push(json!({"frozen_lever_m":lever,"impulse_ns":impulse.to_array()}));
                }
                j+=impulse;normal+=p.normal_impulse;
            }
        }
        if normal>0. && first.is_none(){
            first=Some(json!({"substep":substep,"end_ns":s.elapsed.as_nanos()as u64,"dt_ns":s.duration.as_nanos()as u64,
                "sites":sites,"impulse_ns":j.to_array(),"normal_ns":normal}));
        }
        preloaded+=j;preloaded_normal+=normal;
    })?;
    if missing_tangent || substep != calls || receipt.native_steps != calls {
        return Err("incomplete signed native path".into());
    }
    let pair = receipt
        .pairs
        .iter()
        .find(|p| (p.a, p.b) == (a.min(b), a.max(b)));
    if (preloaded_normal - pair.map_or(0., |p| p.normal_impulse)).abs() > 1e-11
        || first.as_ref().map(|v| v["end_ns"].as_u64().unwrap())
            != pair
                .and_then(|p| p.first_positive_end)
                .map(|d| d.as_nanos() as u64)
    {
        return Err("public solver receipt disagrees with native readback".into());
    }
    Ok(
        json!({"calls":calls,"first":first,"total_ns":preloaded.to_array(),"total_normal_ns":preloaded_normal,"other_normal_ns":extra}),
    )
}
fn collect(f: Fixture) -> Result<Value> {
    let (cuts, disjoint) = survey::cuts(f)?;
    let mut trials = vec![];
    for (i, cut) in cuts.into_iter().enumerate() {
        let Some(cut) = cut else {
            continue;
        };
        let (flight, defect) = kinematics(f, &cut)?;
        let physical = if defect <= 1e-8 {
            geometry(flight)?
        } else {
            json!({"status":"inconsistent_rigid_twist"})
        };
        let mut paths = vec![];
        for n in GRID {
            paths.push(observed(f, &cut, n)?);
        }
        trials.push(json!({"stroke":i+1,"cut":snapshot(&cut),"rigid_twist_residual":defect,"geometry":physical,"paths":paths}));
    }
    Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(dir, first, count, TAG, collect)
}
