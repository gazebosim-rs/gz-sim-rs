// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Observe the ordinary scene policy at the actual prepare/solve boundary.
use super::*;
use bevy_ecs::prelude::{IntoScheduleConfigs, Res, ResMut, Resource};
use gz_sim_core::{Phase, UpdateInfo};
use gz_sim_scene::{model_fidelity, support_loss_promotion};
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub(super) enum Mode {
    Running,
    Paused,
    Intact,
    Unloaded,
}

const SETTLE: Duration = Duration::from_millis(1200);
const PAUSE_TICKS: u64 = 3;
const UNUSED: &str = r#"<model name="unused"><static>true</static><pose>10 0 -0.06 0 0 0</pose><link name="pad"><collision name="c"><geometry><box><size>0.15 0.32 0.12</size></box></geometry></collision></link></model>"#;

fn since_settle(clock: Duration) -> f64 {
    clock.saturating_sub(SETTLE).as_secs_f64()
}
fn tier_name(tier: ArticulationTier) -> &'static str {
    match tier {
        ArticulationTier::Dynamic => "dynamic",
        ArticulationTier::Prescribed => "prescribed",
    }
}

#[derive(Clone, Serialize)]
struct Snapshot {
    label: &'static str,
    tier: &'static str,
    time_s: f64,
    start_time_s: f64,
    end_time_s: f64,
    dt_s: f64,
    iteration: u64,
    epoch: u64,
    paused: bool,
    support_b_present: bool,
    sample: Sample,
    cache: [Option<Value>; 2],
}

#[derive(Resource)]
struct Probe {
    foot: BodyId,
    upper: BodyId,
    pads: [BodyId; 2],
    iteration: Option<u64>,
    observations: Vec<Snapshot>,
}

impl Probe {
    fn snapshot(
        &self,
        physics: &PhysicsBackend,
        label: &'static str,
        clock: Duration,
        info: &UpdateInfo,
        paused: bool,
    ) -> Result<Snapshot> {
        let solver = physics
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("local solver missing")?;
        let mut cache = [None, None];
        for (i, pad) in self.pads.iter().copied().enumerate() {
            if physics.state(pad).is_some() {
                let c = solver.contact_warmstart(self.foot, pad)?;
                cache[i] = Some(
                    json!({"normal_ns":c.normal_ns,"points":c.points,"tangent_on_foot_ns":c.tangent_on_a_ns}),
                );
            }
        }
        Ok(Snapshot {
            label,
            tier: tier_name(solver.articulation_tier(self.foot)?),
            time_s: since_settle(clock),
            start_time_s: since_settle(info.start_time),
            end_time_s: since_settle(info.sim_time),
            dt_s: info.dt.as_secs_f64(),
            iteration: info.iteration,
            epoch: info.epoch,
            paused,
            support_b_present: physics.state(self.pads[1]).is_some(),
            sample: sample_world(physics, self.foot, self.upper, self.pads)?,
            cache,
        })
    }
    fn capture(
        &mut self,
        physics: &PhysicsBackend,
        info: &UpdateInfo,
        label: &'static str,
        clock: Duration,
    ) {
        if self.iteration == Some(info.iteration) && !info.dt.is_zero() {
            let snap = self
                .snapshot(physics, label, clock, info, info.paused)
                .expect("valid measured lane");
            self.observations.push(snap);
        }
    }
}
fn before_prepare(physics: Res<PhysicsBackend>, info: Res<UpdateInfo>, mut probe: ResMut<Probe>) {
    probe.capture(&physics, &info, "before_prepare", info.start_time);
}
fn after_prepare(physics: Res<PhysicsBackend>, info: Res<UpdateInfo>, mut probe: ResMut<Probe>) {
    probe.capture(
        &physics,
        &info,
        "after_prepare_before_solve",
        info.start_time,
    );
}
fn after_solve(physics: Res<PhysicsBackend>, info: Res<UpdateInfo>, mut probe: ResMut<Probe>) {
    probe.capture(&physics, &info, "after_solve", info.sim_time);
}
fn install(lane: &mut Lane) {
    lane.sim.world_mut().insert_resource(Probe {
        foot: lane.foot,
        upper: lane.upper,
        pads: lane.pads,
        iteration: None,
        observations: Vec::new(),
    });
    lane.sim.add_systems(Phase::PreStep, before_prepare);
    lane.sim.add_systems(
        Phase::PhysicsPrepare,
        after_prepare.after(support_loss_promotion::BeforeSolve),
    );
    lane.sim.add_systems(Phase::PhysicsCommit, after_solve);
}
fn pair_snapshot(a: Snapshot, b: Snapshot) -> Value {
    let mut observation = json!(a);
    observation["reference"] = json!(b.sample);
    observation["reference_tier"] = json!(b.tier);
    observation["reference_cache"] = json!(b.cache);
    observation
}
fn boundary(a: &Lane, b: &Lane, label: &'static str) -> Result<Value> {
    let read = |lane: &Lane| {
        let info = lane.sim.committed_update_info()?;
        lane.sim.world().resource::<Probe>().snapshot(
            lane.sim.world().resource::<PhysicsBackend>(),
            label,
            lane.sim.elapsed(),
            &info,
            lane.sim.is_paused(),
        )
    };
    Ok(pair_snapshot(read(a)?, read(b)?))
}
fn receipt(lane: &Lane) -> Option<Value> {
    support_loss_promotion::last_promotion(&lane.sim,lane.model).map(|p|json!({
        "model":p.model,"link":p.link.raw(),"support":p.support.raw(),"reason":format!("{:?}",p.reason),
        "epoch":p.epoch,"iteration":p.iteration,"start_time_s":since_settle(p.start_time),"sim_time_s":since_settle(p.sim_time),
        "absolute_start_time_ns":p.start_time.as_nanos(),"absolute_sim_time_ns":p.sim_time.as_nanos()
    }))
}
fn remove_unloaded(lane: &mut Lane) -> Result<Value> {
    let id = |name: &str| -> Result<u64> {
        lane.sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .and_then(|e| e.get::<EntityId>())
            .map(|n| n.get())
            .ok_or_else(|| format!("missing {name}").into())
    };
    let model = id("default::unused")?;
    let pad =
        entity_by_id(lane.sim.world(), id("default::unused::pad")?).ok_or("missing spare pad")?;
    let pad = lane
        .sim
        .world()
        .get::<BodyHandle>(pad)
        .ok_or("missing spare pad handle")?
        .id();
    let cache = lane.solver().contact_warmstart(lane.foot, pad)?;
    let captured = lane
        .solver()
        .held_support_pair_status(lane.foot, pad)
        .is_some();
    remove_model(&mut lane.sim, model)?;
    Ok(
        json!({"support":pad.raw(),"captured":captured,"normal_ns":cache.normal_ns,"points":cache.points}),
    )
}

pub(super) fn run(mode: Mode) -> Result<(Value, Recording)> {
    let source = if mode == Mode::Unloaded {
        SDF.replace("</world>", &format!("{UNUSED}</world>"))
    } else {
        SDF.to_owned()
    };
    let mut a = Lane::from_sdf(&source)?;
    let mut b = Lane::from_sdf(&source)?;
    for lane in [&mut a, &mut b] {
        install(lane);
        lane.sim.set_paused(false);
    }
    let dt = a.sim.step_duration().as_secs_f64();
    let end = LOSS + POST_STEPS - 1;
    let removes_loaded = matches!(mode, Mode::Running | Mode::Paused);
    let mut frames = Vec::new();
    let mut boundaries = Vec::new();
    let mut spare = Value::Null;
    let mut demotion = Value::Null;
    let mut loss = Value::Null;
    let mut first_dynamic = None;
    let mut committed_receipt = None;
    let mut peak = [0_f64; 4];
    let mut held = [0.; 2];
    for tick in 0..=end {
        if tick == LOSS {
            let next = a.sim.committed_update_info()?.iteration + 1;
            for lane in [&mut a, &mut b] {
                lane.sim.world_mut().resource_mut::<Probe>().iteration = Some(next);
                if mode == Mode::Paused {
                    lane.sim.set_paused(true);
                }
            }
            boundaries.push(boundary(&a, &b, "before_edit")?);
            let old = a.cache()?;
            if removes_loaded {
                a.edit()?;
                b.edit()?;
            } else if mode == Mode::Unloaded {
                spare = json!({"switched":remove_unloaded(&mut a)?,"reference":remove_unloaded(&mut b)?});
            }
            boundaries.push(boundary(&a, &b, "after_edit")?);
            loss = json!({"switched_before":old,"switched_after":a.cache()?,"receipt_after_edit":receipt(&a)});
            if mode == Mode::Paused {
                for label in ["paused_tick_1", "paused_tick_2", "paused_tick_3"] {
                    a.sim.tick()?;
                    b.sim.tick()?;
                    let mut paused = boundary(&a, &b, label)?;
                    paused["receipt"] = json!(receipt(&a));
                    boundaries.push(paused);
                }
                for lane in [&mut a, &mut b] {
                    lane.sim.set_paused(false);
                }
            }
        }
        let torque = load_at(tick);
        if torque != 0. {
            a.pitch(torque)?;
            b.pitch(torque)?;
        }
        if tick > 0 {
            a.sim.tick()?;
            b.sim.tick()?;
        }
        let switched = a.sample()?;
        let reference = b.sample()?;
        let solved_tier = model_fidelity(&a.sim, a.model)?;
        if tick <= DEMOTE && switched != reference {
            return Err("pre-demotion controls disagree".into());
        }
        if tick == DEMOTE {
            let before = a.cache()?;
            set_model_fidelity(&mut a.sim, a.model, ArticulationTier::Prescribed)?;
            let same = a.sample()?;
            if same.foot != switched.foot || same.upper != switched.upper {
                return Err("demotion moved the foot".into());
            }
            demotion = json!({"switched_before":before,"switched_after":a.cache()?,"held_pairs":a.held_pairs()});
        }
        if tick > DEMOTE && solved_tier == ArticulationTier::Dynamic {
            first_dynamic.get_or_insert(tick);
        }
        let observed = receipt(&a);
        if committed_receipt.is_some() && observed != committed_receipt {
            return Err("automatic receipt changed after commit".into());
        }
        if observed.is_some() {
            committed_receipt = observed;
        }
        for (i, error) in pose_error(switched, reference).into_iter().enumerate() {
            peak[i] = peak[i].max(error);
        }
        let prescribed = solved_tier == ArticulationTier::Prescribed;
        if prescribed {
            for (i, j) in switched.normal_impulses_ns.into_iter().enumerate() {
                held[i] += j;
            }
        }
        frames.push(Frame {
            tick,
            time_s: tick as f64 * dt,
            prescribed,
            support_b_present: a.both,
            upper_torque_y_nm: torque,
            switched,
            reference,
        });
    }
    let sa = &a.sim.world().resource::<Probe>().observations;
    let sb = &b.sim.world().resource::<Probe>().observations;
    if sa.len() != 3 || sb.len() != 3 {
        return Err(
            "expected exactly one pre-prepare, pre-solve and post-solve sample per lane".into(),
        );
    }
    let observed = |i: usize| pair_snapshot(sa[i].clone(), sb[i].clone());
    let mut first = Value::Null;
    if let Some(tick) = first_dynamic {
        let current = &frames[tick as usize];
        let previous = &frames[tick as usize - 1];
        first = json!({"switched_normal_ns":current.switched.normal_impulses_ns,"reference_normal_ns":current.reference.normal_impulses_ns,
            "switched_mean_normal_n":current.switched.normal_impulses_ns.map(|v|v/dt),"reference_mean_normal_n":current.reference.normal_impulses_ns.map(|v|v/dt),
            "switched_total_ground_impulse_world_ns":total_ground_impulse(previous.switched,current.switched,dt),
            "reference_total_ground_impulse_world_ns":total_ground_impulse(previous.reference,current.reference,dt)});
    }
    let summary = json!({"mode":format!("{mode:?}"),"dt_s":dt,"settle_s":SETTLE.as_secs_f64(),"demotion_tick":DEMOTE,
        "loss_tick":removes_loaded.then_some(LOSS),"promotion_tick":first_dynamic,"end_tick":end,
        "load_transfer_tick":LEFT_TORQUE_START,"torque_start_tick":RIGHT_TORQUE_START,"torque_end_tick":TORQUE_END,
        "model_id":a.model,"foot_body_id":a.foot.raw(),"support_body_ids":a.pads.map(BodyId::raw),
        "demotion":demotion,"loss":loss,"unloaded_edit":spare,"promotion":{"first_resumed":first},
        "policy":{"default_enabled":true,"pause_ticks":if mode==Mode::Paused{PAUSE_TICKS}else{0},"boundary":boundaries,
            "before_prepare":observed(0),"prepare":observed(1),"commit":observed(2),"receipt":committed_receipt},
        "held_normal_impulses_ns":held,"peak_pose_error":{"foot_center_m":peak[0],"upper_center_m":peak[1],
            "foot_angle_rad":peak[2],"upper_angle_rad":peak[3]}});
    Ok((
        summary,
        Recording {
            dt_s: dt,
            geometry: geometry(),
            frames,
        },
    ))
}

/// Compare equal clock intervals. The delayed control also runs eight later native
/// steps, excluded from the common-horizon maximum.
pub(super) fn manual_controls(automatic: &Recording) -> Result<Value> {
    let mut results = Vec::new();
    for delay in [0, 8] {
        let (summary, manual) = compare(false, delay)?;
        let mut peak = [0_f64; 4];
        let mut equal = 0;
        for (a, m) in automatic.frames.iter().zip(&manual.frames) {
            if a.tick != m.tick || a.reference != m.reference {
                return Err(
                    "reference or clock differs between automatic and manual trials".into(),
                );
            }
            equal += usize::from(a.switched == m.switched && a.prescribed == m.prescribed);
            for (i, v) in pose_error(m.switched, m.reference).into_iter().enumerate() {
                peak[i] = peak[i].max(v);
            }
        }
        let first = &manual.frames[LOSS as usize];
        results.push(json!({"delay_steps":delay,"promotion_tick":summary["promotion_tick"],
            "matched_reference_samples":automatic.frames.len(),"identical_switched_samples":equal,
            "first_step_after_edit_normal_ns":first.switched.normal_impulses_ns,
            "first_resumed":summary["promotion"]["first_resumed"],"common_end_tick":automatic.frames.last().unwrap().tick,
            "common_horizon_peak_error":{"foot_center_m":peak[0],"upper_center_m":peak[1],"foot_angle_rad":peak[2],"upper_angle_rad":peak[3]}}));
    }
    Ok(json!(results))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loaded_removal_promotes_before_the_first_running_or_resumed_solve() -> Result<()> {
        let (_, manual) = compare(false, 0)?;
        let mut active = None;
        for mode in [Mode::Running, Mode::Paused] {
            let (s, record) = run(mode)?;
            assert_eq!(s["promotion_tick"], LOSS);
            let p = &s["policy"];
            let snapshots = p["boundary"].as_array().unwrap();
            assert!(snapshots[0]["support_b_present"].as_bool().unwrap());
            let after = &snapshots[1];
            assert!(!after["support_b_present"].as_bool().unwrap());
            assert_eq!(after["tier"], "prescribed");
            assert!(s["loss"]["receipt_after_edit"].is_null());
            assert_eq!(p["before_prepare"]["tier"], "prescribed");
            assert_eq!(p["prepare"]["tier"], "dynamic");
            assert_eq!(p["commit"]["tier"], "dynamic");
            assert_eq!(p["prepare"]["sample"], after["sample"]);
            assert_eq!(p["prepare"]["cache"], after["cache"]);
            assert!(p["prepare"]["cache"][0]["normal_ns"].as_f64().unwrap() > 0.);
            assert!(p["prepare"]["cache"][1].is_null());
            assert_eq!(p["receipt"]["model"], s["model_id"]);
            assert_eq!(p["receipt"]["link"], s["foot_body_id"]);
            assert_ne!(s["foot_body_id"], s["support_body_ids"][1]);
            assert_eq!(p["receipt"]["support"], s["support_body_ids"][1]);
            assert_eq!(p["receipt"]["reason"], "Retired");
            assert_eq!(p["receipt"]["iteration"], p["commit"]["iteration"]);
            assert_eq!(p["receipt"]["start_time_s"], after["time_s"]);
            assert_eq!(p["receipt"]["sim_time_s"], p["commit"]["time_s"]);
            assert_eq!(p["prepare"]["time_s"], after["time_s"]);
            assert!(
                p["commit"]["time_s"].as_f64().unwrap() > p["prepare"]["time_s"].as_f64().unwrap()
            );
            let first = &record.frames[LOSS as usize];
            assert!(first.switched.normal_impulses_ns[0] > 0.);
            assert_eq!(first.switched.normal_impulses_ns[1], 0.);
            assert_eq!(s["held_normal_impulses_ns"], json!([0., 0.]));
            assert_eq!(record.frames.len(), manual.frames.len());
            assert!(
                record
                    .frames
                    .iter()
                    .zip(&manual.frames)
                    .all(|(f, m)| f.switched == m.switched
                        && f.reference == m.reference
                        && f.prescribed == m.prescribed
                        && f.support_b_present == m.support_b_present)
            );
            if mode == Mode::Paused {
                assert_eq!(snapshots.len() as u64, PAUSE_TICKS + 2);
                for paused in &snapshots[2..] {
                    assert_eq!(paused["time_s"], after["time_s"]);
                    assert_eq!(paused["iteration"], after["iteration"]);
                    assert_eq!(paused["dt_s"], 0.);
                    assert_eq!(paused["tier"], "prescribed");
                    assert!(paused["paused"].as_bool().unwrap() && paused["receipt"].is_null());
                    assert_eq!(paused["sample"], after["sample"]);
                    assert_eq!(paused["reference"], after["reference"]);
                    assert_eq!(paused["cache"], after["cache"]);
                }
                assert!(active.as_ref() == Some(&p["receipt"]));
            } else {
                active = Some(p["receipt"].clone());
            }
        }
        Ok(())
    }

    #[test]
    fn removing_an_unloaded_pad_and_leaving_both_loaded_pads_intact_preserve_prescribed_motion()
    -> Result<()> {
        for mode in [Mode::Intact, Mode::Unloaded] {
            let (s, record) = run(mode)?;
            assert!(s["promotion_tick"].is_null() && s["policy"]["receipt"].is_null());
            assert!(
                record
                    .frames
                    .iter()
                    .skip(DEMOTE as usize + 1)
                    .all(|f| f.prescribed && f.switched.normal_impulses_ns == [0.; 2])
            );
            if mode == Mode::Unloaded {
                assert_eq!(s["unloaded_edit"]["switched"]["normal_ns"], 0.);
                assert_eq!(s["unloaded_edit"]["switched"]["points"], 0);
                assert_eq!(s["unloaded_edit"]["switched"]["captured"], false);
            }
        }
        Ok(())
    }
}
