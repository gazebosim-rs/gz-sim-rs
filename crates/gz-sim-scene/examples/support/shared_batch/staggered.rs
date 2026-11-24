// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently park four feet after four distinct physical arrests.
use super::*;
use glam::DQuat;
use gz_physics::DynamicBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{
    fidelity_zone::{self, FidelityZone},
    model_fidelity, queue_link_world_velocity, support_loss_promotion,
};
use std::time::Duration;
fn wire(v: BodyState) -> Value {
    json!({"position":v.position,"orientation":v.orientation,"velocity":v.velocity,"angular_velocity":v.angular_velocity})
}
fn run_lane(out: &Path, name: &str) -> Result<Value> {
    let mut r = Rig::new(&[0, 1, 2, 3], name != "ordinary", 0.72, false)?;
    let tiered = name == "tiered";
    if tiered {
        for &(id, _, _) in &r.ids {
            fidelity_zone::configure_with_eligibility_dwell(
                &mut r.sim,
                id,
                vec![FidelityZone {
                    center: [99., 0., 0.],
                    promote_radius: 0.2,
                    demote_radius: 1.,
                }],
                Duration::ZERO,
                Duration::from_millis(2),
            )?;
        }
    }
    let target = bind(&r.sim, "default::toe_0::pad");
    let links = [
        "rocker_0::foot",
        "rocker_0::upper",
        "lift_0::pad",
        "toe_0::pad",
    ]
    .map(|s| bind(&r.sim, &format!("default::{s}")).1.unwrap());
    let mut trace = vec![];
    let mut data = vec![];
    let mut previous = [false; 4];
    let mut changes = vec![];
    for tick in 0..=345 {
        if tick == 270 {
            let n = DQuat::from_array(
                r.sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .state(links[3])
                    .unwrap()
                    .orientation,
            ) * DVec3::Z;
            r.sim.set_paused(true);
            let t0 = r.sim.elapsed();
            queue_link_world_velocity(&mut r.sim, target.0, (-1.8 * n).to_array(), [0.; 3])?;
            r.sim.tick()?;
            if r.sim.elapsed() != t0 {
                return Err("paused move consumed time".into());
            }
            r.sim.set_paused(false);
        }
        if tick > 0 {
            r.sim.step_exact(1)?;
        }
        let held: [bool; 4] =
            std::array::from_fn(|i| model_fidelity(&r.sim, r.ids[i].0).unwrap() == Prescribed);
        for k in 0..4 {
            if previous[k] != held[k] {
                changes.push(json!({"station":k,"tick":tick,"prescribed":held[k]}));
            }
        }
        previous = held;
        let e = r.sim.world().resource::<PhysicsBackend>();
        let b = e.downcast_ref::<MultiFidelityBackend>().unwrap();
        let states = links.map(|id| b.state(id).unwrap());
        let receipt = last_refinement(&r.sim).filter(|s| s.iteration == tick);
        let calls = if tick == 0 {
            0
        } else {
            receipt.map_or(1, |s| s.interval.native_steps)
        };
        let pair = |a, z, pad| (a == links[0] && z == pad) || (a == pad && z == links[0]);
        let j = [links[2], links[3]].map(|pad| {
            if tick == 0 {
                return 0.;
            }
            if let Some(s) = receipt {
                s.interval
                    .pairs
                    .iter()
                    .filter(|q| pair(q.a, q.b, pad))
                    .map(|q| q.normal_impulse)
                    .sum::<f64>()
            } else {
                b.rigid_contact_observations()
                    .iter()
                    .filter(|q| pair(q.a, q.b, pad))
                    .map(|q| q.normal_impulse)
                    .sum()
            }
        });
        let p = states[2];
        let f = states[0];
        let pr = DQuat::from_array(p.orientation);
        let fr = DQuat::from_array(f.orientation);
        let normal = pr * DVec3::Z;
        let gap = [-0.18, 0.18]
            .into_iter()
            .flat_map(|x| [-0.1, 0.1].map(move |y| DVec3::new(x, y, -0.06)))
            .map(|c| {
                (DVec3::from(f.position) + fr * c - DVec3::from(p.position)).dot(normal) - 0.06
            })
            .fold(f64::INFINITY, f64::min);
        trace.push(json!({"tick":tick,"time_us":tick*500,"native_steps":calls,"solver_iterations":if calls==0{0}else{4},"selected":calls>1,"weak_start":false,"secondary_start":false,"normal_impulse_ns":j[0]+j[1],"support_normal_ns":j,"gap_m":gap,"foot":wire(states[0]),"upper":wire(states[1]),"pad":wire(states[2]),"opponent":wire(states[3])}));
        let wake = support_loss_promotion::last_promotion(&r.sim, r.ids[0].0)
            .filter(|w| w.iteration == tick);
        if tick == 270
            && tiered
            && (held != [false, true, true, true] || wake.is_none_or(|w| w.support != links[3]))
        {
            return Err("incorrect independent toe wake".into());
        }
        data.push(json!({"tick":tick,"prescribed":held,"registered":r.loads(tick).into_iter().map(|v|v.is_some()).collect::<Vec<_>>(),"positions":r.states().into_iter().map(|v|v.position).collect::<Vec<_>>(),"station0_ns":j}));
    }
    if tiered && (changes.len() != 5 || previous != [false, true, true, true]) {
        return Err(format!("unexpected tier history: {changes:?}").into());
    }
    let geometry = json!({"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.121,0.16,0.06],"opponent_half_extents":[0.126,0.15,0.06]});
    fs::write(
        out.join(format!("traces/{name}.json")),
        serde_json::to_vec(&json!({"geometry":geometry,"frames":trace}))?,
    )?;
    Ok(json!({"transitions":changes,"samples":data}))
}
pub(super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut cases = serde_json::Map::new();
    let mut lanes = vec![];
    for (name, label) in [
        ("ordinary", "Four feet, ordinary dynamic"),
        ("tiered", "Independent tier decisions"),
        ("dynamic", "Four feet, registered dynamic"),
    ] {
        cases.insert(name.into(), run_lane(out, name)?);
        lanes.push(json!({"file":format!("traces/{name}.json"),"label":name,"name":label}));
    }
    let first = cases["tiered"]["transitions"][0]["tick"].as_u64().unwrap();
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"Station 0 parks as three neighbors keep sliding","pair_totals_only":true,"capture_ticks":{"first_independent_park":first,"all_stations_parked":265,"toe_pad_wake":270},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("field.json"), serde_json::to_vec(&cases)?)?;
    Ok(())
}
