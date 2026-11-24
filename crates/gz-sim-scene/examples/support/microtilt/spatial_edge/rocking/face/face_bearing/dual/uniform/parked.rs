// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay a real tier switch and a paused pad command against an uninterrupted dynamic assembly.
use super::*;
use gz_physics_multifidelity::ArticulationTier::{Dynamic, Prescribed};
use gz_sim_core::EntityId;
use gz_sim_scene::{model_fidelity, set_model_fidelity, support_loss_promotion};
const SOURCE: &str =
    include_str!("../../../../../../../../../../../examples/multifidelity_parked_shared_tier.sdf");
fn model(sim: &Simulation, name: &str) -> u64 {
    #[allow(deprecated)]
    sim.world()
        .iter_entities()
        .find(|e| e.get::<gz_sim_scene::Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn observe(sim: &Simulation, ids: &Reading, toe: BodyId, tick: u64) -> (u32, [f64; 2]) {
    if tick == 0 {
        return (0, [0.; 2]);
    }
    let same = |a, b, pad| (a == ids.foot && b == pad) || (a == pad && b == ids.foot);
    if let Some(r) = last_refinement(sim).filter(|r| r.iteration == tick) {
        let j = [ids.pad, toe].map(|pad| {
            r.interval
                .pairs
                .iter()
                .filter(|p| same(p.a, p.b, pad))
                .map(|p| p.normal_impulse)
                .sum()
        });
        (r.interval.native_steps, j)
    } else {
        let actual = engine(sim).rigid_contact_observations();
        let j = [ids.pad, toe].map(|pad| {
            actual
                .iter()
                .filter(|p| same(p.a, p.b, pad))
                .map(|p| p.normal_impulse)
                .sum()
        });
        (1, j)
    }
}
fn ordinary() -> String {
    let begin = SOURCE
        .find("<plugin name=\"gz::sim::systems::PrimaryContactRefinement\"")
        .unwrap();
    let end = begin + SOURCE[begin..].find("</plugin>").unwrap() + 9;
    let mut s = SOURCE.to_string();
    s.replace_range(begin..end, "");
    s
}
fn history(src: &str, switch: bool) -> Result<(Vec<Value>, Vec<Value>)> {
    let mut sim = load(src, true)?;
    let ids = ids(&sim)?;
    let rocker = model(&sim, "default::rocker");
    let toe = bind(&sim, "default::toe::pad")?;
    let pad_n = DQuat::from_array(engine(&sim).state(toe.1).ok_or("missing live toe")?.orientation) * DVec3::Z;
    let mut frames = vec![];
    let mut samples = vec![];
    for tick in 0..=197 {
        if switch {
            match tick {
                17 | 41 => set_model_fidelity(&mut sim, rocker, Prescribed)?,
                33 => set_model_fidelity(&mut sim, rocker, Dynamic)?,
                _ => {}
            }
        }
        if tick == 57 {
            sim.set_paused(true);
            let time = sim.elapsed();
            gz_sim_scene::queue_link_world_velocity(
                &mut sim,
                toe.0,
                (-1.8 * pad_n).to_array(),
                [0.; 3],
            )?;
            sim.tick()?;
            if sim.elapsed() != time || switch && model_fidelity(&sim, rocker)? != Prescribed {
                return Err("pause consumed a step or changed tier".into());
            }
            sim.set_paused(false);
        }
        if tick > 0 {
            sim.step_exact(1)?;
        }
        let tier = model_fidelity(&sim, rocker)?;
        let (calls, j) = observe(&sim, &ids, toe.1, tick);
        let b = engine(&sim);
        let (f, u, p) = departure::ingest(b, &ids);
        let mut record = row(tick, calls, j[0] + j[1], (f, u.unwrap(), p));
        record["opponent"] = wire(b.state(toe.1).ok_or("missing live toe")?);
        record["support_normal_ns"] = json!(j);
        frames.push(record);
        let woke =
            support_loss_promotion::last_promotion(&sim, rocker).filter(|p| p.iteration == tick);
        if tick == 57
            && switch
            && (tier != Dynamic
                || woke.is_none_or(|ev| {
                    ev.link != ids.foot
                        || ev.support != toe.1
                        || ev.start_time != Duration::from_micros(56 * 500)
                }))
        {
            return Err("support did not promote at the loss boundary".into());
        }
        samples.push(json!({"tick":tick,"prescribed":tier==Prescribed,"solver_calls":calls,"pair_ns":j,"wake":woke.map(|ev|format!("{:?}",ev.reason))}));
    }
    Ok((frames, samples))
}
fn geometry() -> Value {
    json!({"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.121,0.16,0.06],"opponent_half_extents":[0.126,0.15,0.06]})
}
pub(in super::super::super::super::super::super::super::super::super) fn run(
    out: &Path,
) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut reports = serde_json::Map::new();
    let mut lanes = vec![];
    for (tag, name, src, switched) in [
        ("ordinary", "Ordinary always dynamic", ordinary(), false),
        ("tiered", "Two measured tier holds", SOURCE.to_owned(), true),
        (
            "dynamic",
            "Registered always dynamic",
            SOURCE.to_owned(),
            false,
        ),
    ] {
        let (frames, samples) = history(&src, switched)?;
        fs::write(
            out.join(format!("traces/{tag}.json")),
            serde_json::to_vec(&json!({"geometry":geometry(),"frames":frames}))?,
        )?;
        reports.insert(tag.to_string(), json!(samples));
        lanes.push(json!({"file":format!("traces/{tag}.json"),"label":tag,"name":name}));
    }
    fs::write(out.join("parked.sdf"), SOURCE)?;
    fs::write(out.join("public.json"), serde_json::to_vec(&reports)?)?;
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"A parked foot resumes dynamics as the toe pad drops","pair_totals_only":true,"capture_ticks":{"parked":24,"resumed_stationary":33,"toe_pad_wake":57,"heel_carries_load":175},"lanes":lanes}),
        )?,
    )?;
    Ok(())
}
#[path = "parked/rearming.rs"]
pub(crate) mod rearming;
