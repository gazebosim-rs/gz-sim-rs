// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare an automatically promoted floor-supported arm with its independent native reference.
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, JointState, PhysicsBackend, PhysicsError};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, FidelityError, JointSample, Name, PhysicsEngine,
    contact_promotion, load_world_with_sdf_step, model_fidelity, queue_link_world_velocity,
    set_model_fidelity,
};
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_sliding_payload.sdf");
const LINKS: [&str; 3] = ["arm::base", "arm::tip", "ram::head"];
const HIT_Y: f64 = 0.71;
const MISS_Y: f64 = 1.15;
const GRAZE_Y: f64 = 0.915;
const GRAZE_DT_US: u64 = 250;

#[derive(Parser)]
struct Args {
    /// Directory for trajectory.json, matrix.json and summary.json.
    output: PathBuf,
    /// Record every state of the 0.25 ms grazing strike instead of the full sweep.
    #[arg(long)]
    grazing: bool,
    /// Earlier grazing output for a matched before-and-after error report.
    #[arg(long, requires = "grazing")]
    baseline: Option<PathBuf>,
}

struct Lane {
    sim: Simulation,
    arm: u64,
    elbow: u64,
    ram: u64,
    bodies: [BodyId; 3],
    floor: BodyId,
    launch_velocity: [f64; 3],
}

impl Lane {
    fn new(dt_us: u64, offset: f64, pitch_rad: f64) -> Result<Self> {
        let rotation = DQuat::from_rotation_y(pitch_rad);
        // Apply the same rigid transform to each model origin, gravity and launch.
        // Child poses and the hinge axis are already relative to their model.
        let world_pose = |point: DVec3| {
            let [x, y, z] = (rotation * point).to_array();
            format!("<pose>{x} {y} {z} 0 {pitch_rad} 0</pose>")
        };
        let [gx, gy, gz] = (rotation * DVec3::new(0., 0., -9.81)).to_array();
        let source = SDF
            .replace(">0.001<", &format!(">{}<", dt_us as f64 * 1e-6))
            .replace(
                "<gravity>0 0 -9.81</gravity>",
                &format!("<gravity>{gx} {gy} {gz}</gravity>"),
            )
            .replace(
                "<pose>0 0 0.24 0 0 0</pose>",
                &world_pose(DVec3::new(0., 0., 0.24)),
            )
            .replace(
                "<pose>0 0 -0.1 0 0 0</pose>",
                &world_pose(DVec3::new(0., 0., -0.1)),
            )
            .replace(
                "<pose>-1.1 0.71 0.17 0 0 0</pose>",
                &world_pose(DVec3::new(-1.1, offset, 0.17)),
            );
        let mut sim = load_world_with_sdf_step(
            &source,
            "sliding_payload.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        sim.step_exact(2_000_000 / dt_us)?;
        let id = |name: &str| -> Result<u64> {
            let name = format!("default::{name}");
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .and_then(|e| e.get::<EntityId>())
                .map(|v| v.get())
                .ok_or_else(|| format!("missing {name}").into())
        };
        let handle = |id| -> Result<BodyId> {
            sim.world()
                .get::<BodyHandle>(entity_by_id(sim.world(), id).ok_or("missing entity")?)
                .map(BodyHandle::id)
                .ok_or_else(|| "missing body".into())
        };
        Ok(Self {
            arm: id("arm")?,
            elbow: id("arm::elbow")?,
            ram: id(LINKS[2])?,
            bodies: [
                handle(id(LINKS[0])?)?,
                handle(id(LINKS[1])?)?,
                handle(id(LINKS[2])?)?,
            ],
            floor: handle(id("floor::ground")?)?,
            launch_velocity: (rotation * DVec3::X * 4.).to_array(),
            sim,
        })
    }

    fn launch(&mut self, automatic: bool) -> Result<()> {
        if automatic {
            set_model_fidelity(&mut self.sim, self.arm, ArticulationTier::Prescribed)?;
            contact_promotion::configure(&mut self.sim, self.arm, vec![self.ram])?;
        }
        queue_link_world_velocity(&mut self.sim, self.ram, self.launch_velocity, [0.; 3])?;
        Ok(())
    }

    /// Retained native contact seed, independent of the last committed reaction.
    fn arm_floor_cache(&self) -> Result<Value> {
        let backend = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing native backend")?;
        let mut cache = Vec::with_capacity(2);
        for link in &self.bodies[..2] {
            let data = backend.contact_warmstart(*link, self.floor)?;
            cache.push(json!({"points":data.points,"normal_ns":data.normal_ns,"tangent_on_arm_ns":data.tangent_on_a_ns}));
        }
        Ok(json!(cache))
    }

    fn sample(&self) -> Result<Sample> {
        let physics = self.sim.world().resource::<PhysicsBackend>();
        let [base, tip, ram] = self.bodies;
        let state = |id| physics.state(id).ok_or("missing body state");
        let contacts = physics
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing native backend")?
            .rigid_contact_observations();
        let impulse = |a, b| {
            contacts
                .iter()
                .filter(|c| (c.a == a && c.b == b) || (c.a == b && c.b == a))
                .map(|c| c.normal_impulse)
                .sum()
        };
        let joint = self
            .sim
            .world()
            .get::<JointSample>(
                entity_by_id(self.sim.world(), self.elbow).ok_or("missing elbow entity")?,
            )
            .ok_or("missing elbow sample")?
            .state;
        Ok(Sample {
            bodies: [state(base)?, state(tip)?, state(ram)?],
            joint,
            tier: model_fidelity(&self.sim, self.arm)?,
            impact: impulse(tip, ram),
            arm_floor_impulses: [impulse(base, self.floor), impulse(tip, self.floor)],
            floor_impulse: impulse(self.floor, ram),
        })
    }
}

struct Sample {
    bodies: [BodyState; 3],
    joint: JointState,
    tier: ArticulationTier,
    impact: f64,
    arm_floor_impulses: [f64; 2],
    floor_impulse: f64,
}

impl Sample {
    /// Smallest Y gap between the payload and either arm box at this sample.
    /// Positive values certify separation even when the swept X extents overlap.
    fn transverse_gap(&self, extents: &[DVec3; 3]) -> f64 {
        let interval = |i: usize| {
            let b = self.bodies[i];
            let local_y = DQuat::from_array(b.orientation).conjugate() * DVec3::Y;
            let radius = local_y.abs().dot(extents[i]);
            (b.position[1] - radius, b.position[1] + radius)
        };
        let (lo, hi) = interval(2);
        [0, 1]
            .into_iter()
            .map(|i| {
                let (arm_lo, arm_hi) = interval(i);
                (lo - arm_hi).max(arm_lo - hi)
            })
            .fold(f64::INFINITY, f64::min)
    }

    fn tier(&self) -> &'static str {
        if self.tier == ArticulationTier::Dynamic {
            "dynamic"
        } else {
            "prescribed"
        }
    }

    fn rows(&self) -> Value {
        Value::Array(
            self.bodies
                .iter()
                .zip(LINKS)
                .map(|(s, name)| {
                    json!({
                        "name":name,"position":s.position,"orientation":s.orientation,
                        "velocity":s.velocity,"angular_velocity":s.angular_velocity,
                    })
                })
                .collect(),
        )
    }
}

#[derive(Default)]
struct Stats {
    first_promotion: Option<u64>,
    first_contact: Option<u64>,
    last_contact: Option<u64>,
    dynamic_ticks: u64,
    support_ticks: u64,
    peak_impulse: f64,
    total_impulse: f64,
}

impl Stats {
    fn observe(&mut self, s: &Sample, tick: u64) {
        if matches!(s.tier, ArticulationTier::Dynamic) {
            self.first_promotion.get_or_insert(tick);
            self.dynamic_ticks += u64::from(tick > 0);
        }
        if s.impact > 0. {
            self.first_contact.get_or_insert(tick);
            self.last_contact = Some(tick);
            self.peak_impulse = self.peak_impulse.max(s.impact);
            self.total_impulse += s.impact;
        }
        self.support_ticks += u64::from(tick > 0 && s.floor_impulse > 0.);
    }
}

#[derive(Default, serde::Serialize)]
struct Errors {
    position_m: f64,
    orientation_rad: f64,
    linear_velocity_m_s: f64,
    angular_velocity_rad_s: f64,
    hinge_angle_rad: f64,
    hinge_rate_rad_s: f64,
    impulse_ns: f64,
}

impl Errors {
    fn observe(&mut self, a: &Sample, b: &Sample) {
        for (a, b) in a.bodies.iter().zip(&b.bodies) {
            self.position_m = self
                .position_m
                .max(DVec3::from(a.position).distance(b.position.into()));
            self.linear_velocity_m_s = self
                .linear_velocity_m_s
                .max(DVec3::from(a.velocity).distance(b.velocity.into()));
            self.angular_velocity_rad_s = self
                .angular_velocity_rad_s
                .max(DVec3::from(a.angular_velocity).distance(b.angular_velocity.into()));
            let q = DQuat::from_array(a.orientation).conjugate() * DQuat::from_array(b.orientation);
            self.orientation_rad = self
                .orientation_rad
                .max(2. * q.xyz().length().atan2(q.w.abs()));
        }
        self.hinge_angle_rad = self
            .hinge_angle_rad
            .max((a.joint.position - b.joint.position).abs());
        self.hinge_rate_rad_s = self
            .hinge_rate_rad_s
            .max((a.joint.velocity - b.joint.velocity).abs());
        self.impulse_ns = self.impulse_ns.max((a.impact - b.impact).abs());
    }
}

struct Comparison {
    summary: Value,
    samples: Vec<Value>,
}

fn box_half_extents() -> Result<[DVec3; 3]> {
    let doc = sdformat::load_str(SDF, "sliding_payload.sdf", &sdformat::NoResources)?;
    let mut half = [DVec3::ZERO; 3];
    for (i, name) in LINKS.iter().enumerate() {
        let (model_name, link_name) = name.split_once("::").ok_or("unscoped body")?;
        let link = doc.worlds[0]
            .models
            .iter()
            .find(|m| m.name == model_name)
            .and_then(|m| m.links.iter().find(|l| l.name == link_name))
            .ok_or("missing authored link")?;
        let [collider] = link.collisions.as_slice() else {
            return Err("expected one box per link".into());
        };
        let sdformat::Geometry::Box { size } = &collider.geometry else {
            return Err("expected box collider".into());
        };
        if collider.pose != sdformat::Pose::default() {
            return Err("clearance measurement requires centered colliders".into());
        }
        half[i] = DVec3::from(*size) * 0.5;
    }
    Ok(half)
}

fn compare(dt_us: u64, offset: f64, record: bool) -> Result<Comparison> {
    compare_in_frame(dt_us, offset, record, 0.)
}

fn compare_in_frame(dt_us: u64, offset: f64, record: bool, pitch_rad: f64) -> Result<Comparison> {
    let mut automatic = Lane::new(dt_us, offset, pitch_rad)?;
    let mut reference = Lane::new(dt_us, offset, pitch_rad)?;
    let settled = automatic.sample()?;
    let settled_cache = if record {
        Some(automatic.arm_floor_cache()?)
    } else {
        None
    };
    automatic.launch(true)?;
    reference.launch(false)?;
    let extents = box_half_extents()?;
    let mut lateral_clearance = f64::INFINITY;
    let steps = 500_000 / dt_us;
    let mut samples = Vec::new();
    let mut auto = Stats::default();
    let mut control = Stats::default();
    let mut errors = Errors::default();
    let mut before_contact = Errors::default();
    let mut peak = (0., 0, LINKS[0]);
    let mut final_response = Value::Null;
    for tick in 0..=steps {
        if tick > 0 {
            automatic.sim.step_exact(1)?;
            reference.sim.step_exact(1)?;
        }
        let a = automatic.sample()?;
        let r = reference.sample()?;
        if r.tier != ArticulationTier::Dynamic {
            return Err("uncontrolled arm changed tier".into());
        }
        lateral_clearance = lateral_clearance.min(a.transverse_gap(&extents));
        auto.observe(&a, tick);
        control.observe(&r, tick);
        if record {
            for ((body, control), name) in a.bodies.iter().zip(&r.bodies).zip(LINKS) {
                let distance = DVec3::from(body.position).distance(control.position.into());
                if distance > peak.0 {
                    peak = (distance, tick, name);
                }
            }
        }
        errors.observe(&a, &r);
        if auto.first_contact.is_none() && control.first_contact.is_none() {
            before_contact.observe(&a, &r);
        }
        if tick == steps {
            final_response = json!({
                "automatic":a.rows(),"reference":r.rows(),
                "hinge_angle_rad":a.joint.position,"reference_hinge_angle_rad":r.joint.position,
                "hinge_rate_rad_s":a.joint.velocity,"reference_hinge_rate_rad_s":r.joint.velocity,
            });
        }
        if record {
            samples.push(json!({
                "tick":tick, "tier":a.tier(),"automatic":a.rows(),"reference":r.rows(),
                "automatic_impulse_ns":a.impact,"reference_impulse_ns":r.impact,
                "automatic_floor_impulse_ns":a.floor_impulse, "reference_floor_impulse_ns":r.floor_impulse,
                "automatic_arm_floor_impulses_ns":a.arm_floor_impulses, "reference_arm_floor_impulses_ns":r.arm_floor_impulses,
                "automatic_arm_floor_cache":automatic.arm_floor_cache()?, "reference_arm_floor_cache":reference.arm_floor_cache()?,
                "hinge_rate_rad_s":a.joint.velocity,"reference_hinge_rate_rad_s":r.joint.velocity,
                "hinge_angle_rad":a.joint.position,"reference_hinge_angle_rad":r.joint.position,
            }));
        }
    }
    let event = contact_promotion::last_promotion(&automatic.sim, automatic.arm);
    if event.map(|e| e.iteration) != auto.first_promotion.map(|tick| 2_000_000 / dt_us + tick) {
        return Err("promotion receipt clock disagrees with observed tier switch".into());
    }
    if let Some(e) = event
        && (e.model != automatic.arm
            || e.obstacle != automatic.ram
            || e.epoch != automatic.sim.epoch()
            || e.sim_time.as_micros() != u128::from(e.iteration * dt_us)
            || e.start_time + automatic.sim.step_duration() != e.sim_time)
    {
        return Err("promotion receipt has wrong scene owner or step interval".into());
    }
    if settled.floor_impulse <= 0.
        || auto.first_contact != control.first_contact
        || auto
            .first_contact
            .is_some_and(|impact| auto.first_promotion.is_none_or(|p| p > impact))
    {
        return Err("support, promotion timing or native contact disagrees with control".into());
    }
    // The three overlapping lanes in this authored sweep must produce a real strike.
    // At the larger clearances the reference must remain unloaded.
    if control.first_contact.is_some() != (offset <= GRAZE_Y) {
        return Err("the reference no longer realizes the authored hit/miss scenario".into());
    }
    Ok(Comparison {
        summary: json!({
            "dt_s":dt_us as f64*1e-6,"world_pitch_rad":pitch_rad,"offset_y_m":offset,"steps":steps,"settle_s":2.0,
            "first_promotion_tick":auto.first_promotion,"first_contact_tick":auto.first_contact,
            "reference_first_contact_tick":control.first_contact,
            "last_contact_tick":auto.last_contact,"reference_last_contact_tick":control.last_contact,
            "dynamic_ticks":auto.dynamic_ticks,"minimum_y_gap_m":lateral_clearance,
            "reference_dynamic_ticks":control.dynamic_ticks,"payload_floor_impulse_ticks":auto.support_ticks,
            "peak_normal_impulse_ns":auto.peak_impulse,"reference_peak_normal_impulse_ns":control.peak_impulse,
            "summed_normal_impulse_ns":auto.total_impulse,"reference_summed_normal_impulse_ns":control.total_impulse,
            "max_reference_errors":errors,"max_precontact_reference_errors":before_contact,
            "promotion_receipt":event.map(|e| json!({"model":e.model,"obstacle":e.obstacle,"epoch":e.epoch,"iteration":e.iteration,"start_us":e.start_time.as_micros(),"end_us":e.sim_time.as_micros()})),
            "settled_bodies":settled.rows(),"settled_payload_floor_impulse_ns":settled.floor_impulse,
            "settled_arm_floor_impulses_ns":settled.arm_floor_impulses, "settled_arm_floor_cache":settled_cache,
            "peak_position_error":record.then(||json!({"error_m":peak.0,"tick":peak.1,"body":peak.2})),
            "final_response":final_response,
        }),
        samples,
    })
}

/// At coarse timesteps the authored arm may retain internal motion even after settling.
/// Check admission in that physical state and preserve the backend's reason in the report.
fn coarse_admission() -> Result<Value> {
    let mut lane = Lane::new(2000, HIT_Y, 0.)?;
    let state = lane.sample()?;
    let mut first_reason = None;
    let mut first_tier = None;
    let mut admitted_after = None;
    // Give the always-dynamic solver one more physical second. Retry the public
    // transition at the newly committed state, preserving its native joint rates.
    for extra_ticks in 0..=500 {
        if extra_ticks > 0 {
            lane.sim.step_exact(1)?;
        }
        match set_model_fidelity(&mut lane.sim, lane.arm, ArticulationTier::Prescribed) {
            Ok(()) => admitted_after = Some(extra_ticks),
            Err(FidelityError::Physics(PhysicsError::TransitionDeferred { reason })) => {
                first_reason.get_or_insert(reason);
            }
            Err(err) => return Err(err.into()),
        }
        first_tier.get_or_insert(lane.sample()?.tier());
        if admitted_after.is_some() {
            break;
        }
    }
    let mut post_retry = Value::Null;
    if let Some(extra_ticks) = admitted_after {
        let mut reference = Lane::new(2000, HIT_Y, 0.)?;
        reference.sim.step_exact(extra_ticks)?;
        contact_promotion::configure(&mut lane.sim, lane.arm, vec![lane.ram])?;
        lane.launch(false)?;
        reference.launch(false)?;
        let (mut auto, mut control) = (Stats::default(), Stats::default());
        for tick in 1..=250 {
            lane.sim.step_exact(1)?;
            reference.sim.step_exact(1)?;
            auto.observe(&lane.sample()?, tick);
            control.observe(&reference.sample()?, tick);
        }
        if auto.first_contact != control.first_contact
            || !matches!((auto.first_promotion, auto.first_contact), (Some(promotion), Some(impact)) if promotion <= impact)
        {
            return Err("coarse admitted strike disagrees with dynamic reference".into());
        }
        post_retry = json!({"first_promotion_tick":auto.first_promotion,"first_contact_tick":auto.first_contact,
            "reference_first_contact_tick":control.first_contact,"summed_normal_impulse_ns":auto.total_impulse,
            "reference_summed_normal_impulse_ns":control.total_impulse});
    }
    Ok(
        json!({"dt_s":0.002,"settle_s":2.,"deferred_reason":first_reason,
        "hinge_rate_rad_s":state.joint.velocity,"tier":first_tier,
        "retry_budget_ticks":500,"extra_ticks_to_admission":admitted_after,"post_retry":post_retry}),
    )
}

/// Replay the committed pre-promotion state, then inspect the switch at the tick
/// boundary without advancing physics. The ordinary automatic run is an independent
/// check on the branch: its first resumed step must reproduce every sampled body.
fn grazing_switch_probe(run: &Comparison) -> Result<Value> {
    let promotion = run.summary["first_promotion_tick"]
        .as_u64()
        .ok_or("grazing pass never promoted")?;
    let before_tick = promotion
        .checked_sub(1)
        .ok_or("expected prescribed interval")?;
    let mut lane = Lane::new(GRAZE_DT_US, GRAZE_Y, 0.)?;
    lane.launch(true)?;
    lane.sim.step_exact(before_tick)?;
    let before = lane.sample()?;
    let before_cache = lane.arm_floor_cache()?;
    if before.tier != ArticulationTier::Prescribed
        || before.rows() != run.samples[before_tick as usize]["automatic"]
    {
        return Err("boundary probe diverged before promotion".into());
    }
    set_model_fidelity(&mut lane.sim, lane.arm, ArticulationTier::Dynamic)?;
    let after = lane.sample()?;
    let restored = lane.arm_floor_cache()?;
    if before.rows() != after.rows() || before.joint != after.joint {
        return Err("switch changed a pose or velocity before a solve".into());
    }
    lane.sim.step_exact(1)?;
    let resumed = lane.sample()?;
    let committed = &run.samples[promotion as usize];
    if resumed.rows() != committed["automatic"]
        || json!(resumed.arm_floor_impulses) != committed["automatic_arm_floor_impulses_ns"]
        || lane.arm_floor_cache()? != committed["automatic_arm_floor_cache"]
    {
        return Err("first resumed state disagrees with normally scheduled promotion".into());
    }
    Ok(json!({
        "last_prescribed_tick":before_tick, "first_resumed_tick":promotion,
        "before_switch_cache":before_cache,"after_switch_before_solve_cache":restored,
        "before_switch_floor_impulses_ns":before.arm_floor_impulses,"after_switch_before_solve_floor_impulses_ns":after.arm_floor_impulses,
        "first_resumed_floor_impulses_ns":resumed.arm_floor_impulses,
        "first_resumed_matches_automatic":true
    }))
}

/// Only attribute differences to promotion when the complete always-dynamic
/// trajectory, contact history and sample clocks agree with the earlier run.
fn compare_grazing_runs(before: &std::path::Path, run: &Comparison) -> Result<Value> {
    let prior: Value = serde_json::from_reader(fs::File::open(before.join("trajectory.json"))?)?;
    let summary: Value = serde_json::from_reader(fs::File::open(before.join("summary.json"))?)?;
    let samples = prior["samples"]
        .as_array()
        .ok_or("missing baseline samples")?;
    if prior["dt_s"] != run.summary["dt_s"]
        || prior["settle_s"] != run.summary["settle_s"]
        || summary["offset_y_m"] != run.summary["offset_y_m"]
        || samples.len() != run.samples.len()
    {
        return Err("baseline and updated trials have different conditions".into());
    }
    for (old, new) in samples.iter().zip(&run.samples) {
        let fields = new.as_object().ok_or("invalid sample")?;
        for key in fields
            .keys()
            .filter(|k| k.starts_with("reference") || *k == "tick")
        {
            if old.get(key) != new.get(key) {
                return Err(
                    format!("baseline reference disagrees at {:?} ({key})", new["tick"]).into(),
                );
            }
        }
    }
    let error = |s: &Value| {
        s["max_reference_errors"]["position_m"]
            .as_f64()
            .ok_or("missing position error")
    };
    let (old, new) = (error(&summary)?, error(&run.summary)?);
    let reduction = (old > 0.).then(|| (old - new) / old * 100.);
    Ok(json!({
        "dt_s":run.summary["dt_s"],"offset_y_m":run.summary["offset_y_m"],
        "matched_reference_samples":samples.len(),
        "baseline_peak":summary["peak_position_error"],"updated_peak":run.summary["peak_position_error"],
        "position_error_reduction_m":old-new,"position_error_reduction_percent":reduction,
        "baseline_first_promotion_tick":summary["first_promotion_tick"],"updated_first_promotion_tick":run.summary["first_promotion_tick"],
        "baseline_first_contact_tick":summary["first_contact_tick"],"updated_first_contact_tick":run.summary["first_contact_tick"],
        "baseline_max_reference_errors":summary["max_reference_errors"],"updated_max_reference_errors":run.summary["max_reference_errors"]
    }))
}

fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir_all(&args.output)?;
    let write = |name: &str, value: &Value| -> Result<()> {
        fs::write(args.output.join(name), serde_json::to_vec(value)?)?;
        Ok(())
    };
    if args.grazing {
        let mut run = compare(GRAZE_DT_US, GRAZE_Y, true)?;
        run.summary["switch_probe"] = grazing_switch_probe(&run)?;
        if let Some(prior) = args.baseline {
            write("comparison.json", &compare_grazing_runs(&prior, &run)?)?;
        }
        write("summary.json", &run.summary)?;
        write(
            "trajectory.json",
            &json!({"dt_s":run.summary["dt_s"], "settle_s":run.summary["settle_s"], "samples":run.samples}),
        )?;
        println!("{}", serde_json::to_string_pretty(&run.summary)?);
        return Ok(());
    }
    let mut matrix = Vec::new();
    let mut hit = None;
    let mut miss = None;
    for dt_us in [1000, 500, 250] {
        for offset in [HIT_Y, 0.85, GRAZE_Y, 0.93, 1., MISS_Y, 1.65] {
            let canonical = dt_us == 1000 && [HIT_Y, MISS_Y].contains(&offset);
            let result = compare(dt_us, offset, canonical)
                .map_err(|err| format!("step {dt_us} µs; offset {offset} m: {err}"))?;
            eprintln!(
                "dt {dt_us} y {offset} promotion={} impact={} reference={}",
                result.summary["first_promotion_tick"],
                result.summary["first_contact_tick"],
                result.summary["reference_first_contact_tick"]
            );
            matrix.push(result.summary.clone());
            if canonical {
                if offset == HIT_Y {
                    hit = Some(result);
                } else {
                    miss = Some(result);
                }
            }
        }
    }
    let Comparison {
        mut summary,
        mut samples,
    } = hit.ok_or("missing strike")?;
    let miss = miss.ok_or("missing near miss")?;
    summary["near_miss"] = miss.summary;
    summary["inclined"] = json!({
        "hit": compare_in_frame(1000, HIT_Y, false, 0.25)?.summary,
        "near_miss": compare_in_frame(1000, MISS_Y, false, 0.25)?.summary,
    });
    summary["coarse_step_admission"] = coarse_admission()?;
    for (s, mut m) in samples.iter_mut().zip(miss.samples) {
        for (dest, source) in [
            ("near_miss", "automatic"),
            ("near_miss_tier", "tier"),
            ("near_miss_impulse_ns", "automatic_impulse_ns"),
            ("near_miss_hinge_rate_rad_s", "hinge_rate_rad_s"),
        ] {
            s[dest] = m[source].take();
        }
    }
    write("summary.json", &summary)?;
    write("matrix.json", &json!(matrix))?;
    write(
        "trajectory.json",
        &json!({"settle_s":2.,"dt_s":0.001,"samples":samples}),
    )?;
    println!("{}", serde_json::to_string_pretty(&summary)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn settled_floor_strike_matches_reference_and_close_miss_stays_prescribed() -> Result<()> {
        let strike = compare(1000, HIT_Y, false)?.summary;
        let promotion = strike["first_promotion_tick"]
            .as_u64()
            .ok_or("missing promotion")?;
        let impact = strike["first_contact_tick"]
            .as_u64()
            .ok_or("missing impact")?;
        assert!(promotion > 75 && promotion <= impact);
        // In a floor-supported strike, finite contact and friction error accumulate.
        // Use a physical length and fractional impulse tolerance and export actual residuals.
        let impulse = strike["summed_normal_impulse_ns"].as_f64().unwrap();
        let reference = strike["reference_summed_normal_impulse_ns"]
            .as_f64()
            .unwrap();
        assert!(reference > 0.1 && (impulse - reference).abs() < 0.01 * reference);
        assert!(
            strike["max_reference_errors"]["position_m"]
                .as_f64()
                .unwrap()
                < 0.005
        );
        let miss = compare(1000, MISS_Y, false)?.summary;
        assert_eq!(miss["first_contact_tick"], Value::Null);
        assert!(miss["minimum_y_gap_m"].as_f64().unwrap() > 0.22);
        assert_eq!(miss["dynamic_ticks"], 0);
        assert_eq!(miss["promotion_receipt"], Value::Null);
        Ok(())
    }
    #[test]
    fn grazing_strike_resumes_loaded_floor_patches_before_the_first_dynamic_solve() -> Result<()> {
        let run = compare(GRAZE_DT_US, GRAZE_Y, true)?;
        let boundary = grazing_switch_probe(&run)?;
        let settled = &run.summary["settled_arm_floor_cache"];
        for seed in settled.as_array().ok_or("missing settled support cache")? {
            assert!(seed["points"].as_u64().unwrap() > 0);
            assert!(seed["normal_ns"].as_f64().unwrap() > 0.);
        }
        assert_eq!(boundary["after_switch_before_solve_cache"], *settled);
        assert_eq!(
            boundary["after_switch_before_solve_floor_impulses_ns"],
            json!([0., 0.])
        );
        // The same native fixture had 3.175 mm center error when support seeds were discarded.
        // Bound the complete 500 ms trajectory, including the off-center impact.
        assert!(
            run.summary["max_reference_errors"]["position_m"]
                .as_f64()
                .unwrap()
                < 0.0001
        );
        Ok(())
    }

    #[test]
    fn coarse_step_recovers_by_retrying_native_admission() -> Result<()> {
        let coarse = coarse_admission()?;
        assert!(coarse["extra_ticks_to_admission"].as_u64().is_some());
        assert!(
            coarse["post_retry"]["reference_summed_normal_impulse_ns"]
                .as_f64()
                .unwrap()
                > 0.1
        );
        Ok(())
    }
    #[test]
    fn rotating_the_floor_gravity_and_launch_preserves_promotion_and_contact_ticks() -> Result<()> {
        for offset in [HIT_Y, MISS_Y] {
            let level = compare(1000, offset, false)?.summary;
            let pitched = compare_in_frame(1000, offset, false, 0.25)?.summary;
            for key in [
                "first_promotion_tick",
                "first_contact_tick",
                "reference_first_contact_tick",
                "dynamic_ticks",
            ] {
                assert_eq!(level[key], pitched[key], "offset {offset}, {key}");
            }
        }
        Ok(())
    }
}
