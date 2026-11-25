// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the shipped support journals with full dynamics on the same Bevy clock.
#[path = "support_edge_course/matched.rs"]
mod matched;

use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::{ContactWarmstart, MultiFidelityBackend, SupportExpiry};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, Name, load_local_replay_world, model_fidelity, remove_model,
    set_model_fidelity,
    support_loss_promotion::{SupportLossReason, last_promotion},
};
use parry3d_f64::{
    math::{Pose, Rotation, Vector},
    query,
    shape::Cuboid,
};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{error::Error, fs, path::PathBuf, time::Instant};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

const EDGE: &str = include_str!("../../../examples/multifidelity_rocking_foot.sdf");
const TWO: &str = include_str!("../../../examples/multifidelity_two_support_foot.sdf");
const EDGE_JOURNAL: &str = include_str!("../../../examples/replay/course_foot_edge.json");
const PAUSE_JOURNAL: &str = include_str!("../../../examples/replay/course_paused_support.json");

#[derive(Deserialize)]
struct Journal {
    seed: String,
    commands: Vec<Command>,
}
#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
enum Command {
    Step { count: u64 },
    SetTier { model: String, tier: Tier },
    RemoveModel { model: String },
    PausedTick,
}
#[derive(Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum Tier {
    Prescribed,
    Dynamic,
}

#[derive(Serialize, PartialEq)]
struct PointSeed {
    features: [u32; 2],
    local_anchors: [[f64; 3]; 2],
    normal_ns: f64,
    tangent_basis_ns: [f64; 2],
    tangent_world_ns: [f64; 3],
    twist_nms: f64,
}
type PointCache = std::collections::BTreeMap<[(u32, u32); 2], Vec<PointSeed>>;

/// Map the two distinct measured load totals to their collider pairs before either pad is removed.
fn pair_key(cache: &PointCache, normal_ns: f64) -> Result<[(u32, u32); 2]> {
    let mut matching = cache
        .iter()
        .filter(|(_, points)| points.iter().map(|p| p.normal_ns).sum::<f64>() == normal_ns);
    let key = *matching.next().ok_or("missing loaded pair")?.0;
    if matching.next().is_some() {
        return Err("ambiguous pair impulse".into());
    }
    Ok(key)
}

struct Part {
    body: BodyId,
    shape: Cuboid,
}
struct Lane {
    sim: Simulation,
    foot: Part,
    upper: BodyId,
    supports: Vec<Part>,
    model: u64,
}

fn id(sim: &Simulation, name: &str) -> Result<u64> {
    Ok(sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .and_then(|e| e.get::<EntityId>())
        .ok_or_else(|| format!("missing {name}"))?
        .get())
}

impl Lane {
    fn new(xml: &str, seed: &str, pads: &[&str]) -> Result<Self> {
        let mut sim =
            load_local_replay_world(xml, "course.sdf", "default", Sha256::digest(seed).into())?;
        sim.set_paused(true);
        let parsed = sdformat::load_str(xml, "course.sdf", &sdformat::NoResources)?;
        let part = |name: &str| -> Result<Part> {
            let scoped = format!("default::{name}");
            let e = gz_sim_core::entity_by_id(sim.world(), id(&sim, &scoped)?)
                .ok_or("missing public link")?;
            let body = sim.world().get::<BodyHandle>(e).ok_or("missing body")?.id();
            let link = parsed.worlds[0]
                .models
                .iter()
                .flat_map(|m| &m.links)
                .find(|l| l.scoped_name == scoped)
                .ok_or("missing SDF link")?;
            let [collision] = link.collisions.as_slice() else {
                return Err("expected one course collider".into());
            };
            let sdformat::Geometry::Box { size } = collision.geometry else {
                return Err("expected a box".into());
            };
            if collision.pose.transform != sdformat::Transform::default()
                || !collision.pose.relative_to.is_empty()
            {
                return Err("expected box centered in the link frame".into());
            }
            Ok(Part {
                body,
                shape: Cuboid::new(Vector::from_array(size) * 0.5),
            })
        };
        Ok(Self {
            foot: part("rocker::foot")?,
            upper: part("rocker::upper")?.body,
            supports: pads.iter().map(|&p| part(p)).collect::<Result<_>>()?,
            model: id(&sim, "default::rocker")?,
            sim,
        })
    }
    fn solver(&self) -> &MultiFidelityBackend {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref()
            .expect("loaded local solver")
    }
    fn state(&self, body: BodyId) -> Option<BodyState> {
        self.sim.world().resource::<PhysicsBackend>().state(body)
    }
    fn bodies(&self) -> [BodyState; 2] {
        [self.foot.body, self.upper].map(|b| self.state(b).expect("robot links remain present"))
    }
    fn caches(&self) -> Result<Vec<Option<ContactWarmstart>>> {
        self.supports
            .iter()
            .map(|p| {
                self.state(p.body)
                    .map(|_| self.solver().contact_warmstart(self.foot.body, p.body))
                    .transpose()
                    .map_err(Into::into)
            })
            .collect()
    }
    fn point_seeds(&self) -> PointCache {
        let mut pairs = std::collections::BTreeMap::new();
        for p in self.solver().stored_contact_points() {
            let seed = PointSeed {
                features: p.features,
                local_anchors: p.local_anchors,
                normal_ns: p.normal_ns,
                tangent_basis_ns: p.tangent_basis_ns,
                tangent_world_ns: p.tangent_world_ns,
                twist_nms: p.twist_nms,
            };
            pairs.entry(p.colliders).or_insert_with(Vec::new).push(seed);
        }
        pairs
    }
    fn reactions(&self) -> Vec<f64> {
        let contacts = self.solver().rigid_contact_observations();
        self.supports
            .iter()
            .map(|p| {
                contacts
                    .iter()
                    .filter(|c| {
                        (c.a, c.b) == (self.foot.body, p.body)
                            || (c.b, c.a) == (self.foot.body, p.body)
                    })
                    .map(|c| c.normal_impulse)
                    .sum()
            })
            .collect()
    }
    fn penetration(&self) -> Result<f64> {
        let pose = |s: BodyState| {
            Pose::from_parts(
                Vector::from_array(s.position),
                Rotation::from_array(s.orientation),
            )
        };
        let foot = pose(self.state(self.foot.body).ok_or("missing foot")?);
        let mut depth = 0.0_f64;
        for p in &self.supports {
            if let Some(s) = self.state(p.body)
                && let Some(c) = query::contact(&foot, &self.foot.shape, &pose(s), &p.shape, 0.)?
            {
                depth = depth.max(-c.dist);
            }
        }
        Ok(depth)
    }
}
fn cache_json(caches: &[Option<ContactWarmstart>]) -> Value {
    json!(caches.iter().map(|c| c.map(|c| json!({"points":c.points,"normal_ns":c.normal_ns,"tangent_ns":c.tangent_on_a_ns}))).collect::<Vec<_>>())
}
fn errors(a: [BodyState; 2], b: [BodyState; 2]) -> ([f64; 2], [f64; 2]) {
    let position = std::array::from_fn(|i| {
        DVec3::from_array(a[i].position).distance(DVec3::from_array(b[i].position))
    });
    let angle = std::array::from_fn(|i| {
        let delta =
            DQuat::from_array(a[i].orientation).conjugate() * DQuat::from_array(b[i].orientation);
        2. * delta.xyz().length().atan2(delta.w.abs())
    });
    (position, angle)
}

struct Timings {
    candidate: Vec<u128>,
    reference: Vec<u128>,
}
impl Timings {
    fn new() -> Self {
        Self {
            candidate: Vec::new(),
            reference: Vec::new(),
        }
    }
    fn step(&mut self, lanes: &mut [Lane; 2], candidate_first: bool) -> Result<()> {
        for i in if candidate_first { [0, 1] } else { [1, 0] } {
            let start = Instant::now();
            lanes[i].sim.step_exact(1)?;
            let ns = start.elapsed().as_nanos();
            if i == 0 {
                self.candidate.push(ns)
            } else {
                self.reference.push(ns)
            }
        }
        Ok(())
    }
    fn report(mut self) -> Value {
        let percentile = |samples: &mut Vec<u128>| {
            samples.sort_unstable();
            samples[(samples.len() * 95).div_ceil(100) - 1]
        };
        json!({"samples_per_lane":self.candidate.len(),"candidate_p95_ns":percentile(&mut self.candidate),
               "dynamic_p95_ns":percentile(&mut self.reference)})
    }
}

fn compare(
    xml: &str,
    source: &str,
    pads: &[&str],
    timings: &mut Timings,
    reverse: bool,
) -> Result<Value> {
    let journal: Journal = serde_json::from_str(source)?;
    let mut lanes = [
        Lane::new(xml, &journal.seed, pads)?,
        Lane::new(xml, &journal.seed, pads)?,
    ];
    let is_edge = pads.len() == 1;
    let mut tick = 0;
    let mut parked = false;
    let mut resumed = false;
    let mut matched = None;
    let mut expired = None;
    let mut crossed = None;
    let mut first_unequal = None;
    let mut first_threshold = None;
    let mut peak_position = [0.0_f64; 2];
    let mut peak_angle = [0.0_f64; 2];
    let mut peak_penetration = [0.0_f64; 2];
    let mut cache_at_boundary = Value::Null;
    let mut first_impulse = Value::Null;
    let mut demotion_tick = None;
    let mut retired_pair = None;
    let mut pause_ticks = 0;
    for command in journal.commands {
        match command {
            Command::SetTier { model, tier } => {
                let [a, b] = &mut lanes;
                let model = id(&a.sim, &model)?;
                if tier == Tier::Prescribed {
                    if a.bodies() != b.bodies() || a.caches()? != b.caches()? {
                        return Err("reference and candidate differ before demotion".into());
                    }
                    demotion_tick = Some(tick);
                    set_model_fidelity(&mut a.sim, model, ArticulationTier::Prescribed)?;
                    parked = true;
                } else {
                    let caches = a.caches()?;
                    let before = a.bodies();
                    let status = a
                        .solver()
                        .held_support_pair_status(a.foot.body, a.supports[0].body)
                        .ok_or("missing held ledge")?;
                    if status.expiry != Some(SupportExpiry::AnchorOutsideFace)
                        || caches[0].is_none_or(|c| c.normal_ns <= 0.)
                    {
                        return Err("expected expired toe contact with an old normal seed".into());
                    }
                    set_model_fidelity(&mut a.sim, model, ArticulationTier::Dynamic)?;
                    let after = a.caches()?;
                    if before != a.bodies() || after[0] != Some(ContactWarmstart::default()) {
                        return Err("edge promotion must clear the expired patch and keep the pose and twist".into());
                    }
                    matched = Some(matched::Restart::new(a, &journal.seed)?);
                    cache_at_boundary = json!({"before":cache_json(&caches),"after":cache_json(&after),"expiry":"AnchorOutsideFace"});
                }
            }
            Command::RemoveModel { model } => {
                if lanes[0].bodies() != lanes[1].bodies()
                    || lanes[0].caches()? != lanes[1].caches()?
                {
                    return Err("paused removal must start from matched states and seeds".into());
                }
                let old_points = lanes[0].point_seeds();
                let before = lanes[0].caches()?;
                if before.iter().any(|c| c.is_none_or(|v| v.normal_ns <= 0.)) {
                    return Err("both pads must bear load".into());
                }
                for lane in &mut lanes {
                    let model = id(&lane.sim, &model)?;
                    remove_model(&mut lane.sim, model)?;
                }
                let after = lanes[0].caches()?;
                let physical_retired = lanes[0]
                    .solver()
                    .contact_warmstart(lanes[0].foot.body, lanes[0].supports[1].body);
                if after[0] != before[0]
                    || !matches!(physical_retired, Err(gz_physics::PhysicsError::MissingBody))
                    || after[1].is_some()
                    || lanes[0]
                        .solver()
                        .held_support_pair_status(lanes[0].foot.body, lanes[0].supports[1].body)
                        .is_some()
                {
                    return Err(
                        "support retirement damaged the surviving seed or retained the removed pad"
                            .into(),
                    );
                }
                let left = pair_key(
                    &old_points,
                    before[0].ok_or("missing left cache")?.normal_ns,
                )?;
                let right = pair_key(
                    &old_points,
                    before[1].ok_or("missing right cache")?.normal_ns,
                )?;
                let points = lanes[0].point_seeds();
                let survivor = points.get(&left).ok_or("surviving manifold missing")?;
                if left == right || survivor.len() != 4 || old_points.get(&left) != Some(survivor) {
                    return Err("surviving point seeds changed at removal".into());
                }
                retired_pair = Some(right);
                cache_at_boundary = json!({"before":cache_json(&before),"after":cache_json(&after),"surviving_points":survivor,
                    "retired_pair_still_in_last_narrow_phase_snapshot":points.contains_key(&right)});
            }
            Command::PausedTick => {
                for lane in &mut lanes {
                    let before = (lane.sim.elapsed(), lane.bodies(), lane.caches()?);
                    if !lane.sim.tick()?.dt.is_zero()
                        || before != (lane.sim.elapsed(), lane.bodies(), lane.caches()?)
                    {
                        return Err("paused tick changed physical state".into());
                    }
                }
                if model_fidelity(&lanes[0].sim, lanes[0].model)? != ArticulationTier::Prescribed
                    || last_promotion(&lanes[0].sim, lanes[0].model).is_some()
                {
                    return Err("premature promotion during pause".into());
                }
                pause_ticks += 1;
            }
            Command::Step { count } => {
                for _ in 0..count {
                    tick += 1;
                    timings.step(&mut lanes, (tick % 2 == 0) ^ reverse)?;
                    let [a, b] = &lanes;
                    if let Some(restart) = &mut matched {
                        restart.advance(a)?;
                    }
                    let states = a.bodies();
                    let reference = b.bodies();
                    if demotion_tick.is_none() && states != reference {
                        return Err("dynamic initialization disagrees".into());
                    }
                    let tier = model_fidelity(&a.sim, a.model)?;
                    if parked && tier == ArticulationTier::Prescribed && is_edge {
                        let toe = DVec3::from_array(states[0].position)
                            + DQuat::from_array(states[0].orientation)
                                * DVec3::new(0.08, -0.10, -0.06);
                        if toe.x > 0. {
                            crossed.get_or_insert(tick);
                        }
                        if a.solver()
                            .held_support_pair_status(a.foot.body, a.supports[0].body)
                            .is_some_and(|s| s.expiry == Some(SupportExpiry::AnchorOutsideFace))
                        {
                            expired.get_or_insert(tick);
                        }
                    }
                    if parked && tier == ArticulationTier::Dynamic && !resumed {
                        let actual = a.reactions();
                        let full = b.reactions();
                        if full[0] <= 0. || (actual[0] / full[0] - 1.).abs() > 0.05 {
                            return Err(format!(
                                "resumed support reaction outside 5% bound: {actual:?} {full:?}"
                            )
                            .into());
                        }
                        if is_edge {
                            if crossed != Some(tick - 1) || expired != crossed {
                                return Err("edge crossing and seed expiry must occur on the preceding step".into());
                            }
                        } else {
                            if retired_pair.is_some_and(|key| a.point_seeds().contains_key(&key)) {
                                return Err(
                                    "retired native manifold survived the first resumed solve"
                                        .into(),
                                );
                            }
                            let event = last_promotion(&a.sim, a.model)
                                .ok_or("absent support-loss promotion")?;
                            if Some(tick - 1) != demotion_tick
                                || event.iteration != tick
                                || event.support != a.supports[1].body
                                || event.reason != SupportLossReason::Retired
                            {
                                return Err("missing automatic first-solve promotion".into());
                            }
                        }
                        first_impulse = json!({"tick":tick,"switched_ns":actual,"dynamic_ns":full});
                        resumed = true;
                    }
                    if resumed && !is_edge && (a.reactions()[1] != 0. || b.reactions()[1] != 0.) {
                        return Err("removed pad delivered an impulse".into());
                    }
                    let (dp, da) = errors(states, reference);
                    if dp.iter().any(|&v| v > 0.)
                        || states
                            .iter()
                            .zip(reference)
                            .any(|(s, r)| s.orientation != r.orientation)
                    {
                        first_unequal.get_or_insert(tick);
                    }
                    if dp.iter().any(|&v| v > 1e-6) || da.iter().any(|&v| v > 1e-5) {
                        first_threshold.get_or_insert(tick);
                    }
                    for i in 0..2 {
                        peak_position[i] = peak_position[i].max(dp[i]);
                        peak_angle[i] = peak_angle[i].max(da[i]);
                        peak_penetration[i] = peak_penetration[i].max(lanes[i].penetration()?);
                    }
                }
            }
        }
    }
    let bound = if is_edge { 1e-4 } else { 1e-3 };
    if !resumed || peak_position[0] > bound || !is_edge && pause_ticks != 3 {
        return Err(format!("foot trajectory missed bound: {peak_position:?}, resumed={resumed}, pause={pause_ticks}").into());
    }
    Ok(
        json!({"steps":tick,"dt_ns":lanes[0].sim.step_duration().as_nanos(),"support_order":pads,"demotion_tick":demotion_tick,
        "matched_dynamic_wake":matched.map(matched::Restart::report).transpose()?, "paused_updates":pause_ticks,"toe_crossing_tick":crossed,"edge_seed_expiry_tick":expired,"cache_at_edit_or_promotion":cache_at_boundary,
        "first_resumed_impulse":first_impulse,"first_unequal_pose_tick":first_unequal,"first_pose_difference_over_1um_or_10urad_tick":first_threshold,
        "peak_center_error_m_foot_upper":peak_position,"peak_angle_error_rad_foot_upper":peak_angle,"peak_box_penetration_m_switched_dynamic":peak_penetration}),
    )
}

fn main() -> Result<()> {
    let output = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: support_edge_course OUTPUT.json")?,
    );
    let mut report = json!({});
    for (name, xml, journal, pads) in [
        ("edge", EDGE, EDGE_JOURNAL, &["ledge::ground"][..]),
        (
            "paused_removal",
            TWO,
            PAUSE_JOURNAL,
            &["left::pad", "right::pad"][..],
        ),
    ] {
        let mut timings = Timings::new();
        let mut receipt = None;
        for repeat in 0..4 {
            let result = compare(xml, journal, pads, &mut timings, repeat % 2 != 0)?;
            if receipt.as_ref().is_some_and(|old| old != &result) {
                return Err("repeated physics results differ".into());
            }
            receipt = Some(result);
        }
        report[name] = receipt.ok_or("missing support results")?;
        report[name]["step_cost"] = timings.report();
    }
    fs::write(output, serde_json::to_vec_pretty(&report)?)?;
    Ok(())
}
