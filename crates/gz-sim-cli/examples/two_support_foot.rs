// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Transfer load between two pads, retire one while prescribed, and measure the native handoff.
#[path = "two_support_foot/auto_loss.rs"]
mod auto_loss;

use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend, PhysicsError};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step,
    queue_link_world_wrench, remove_model, set_model_fidelity,
};
use serde::Serialize;
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    io::{BufWriter, Write},
    path::PathBuf,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_two_support_foot.sdf");
const DEMOTE: u64 = 601;
const LOSS: u64 = 621;
const POST_STEPS: u64 = 600;
const LEFT_TORQUE_START: u64 = 96;
const RIGHT_TORQUE_START: u64 = 21;
const TORQUE_END: u64 = 170;
#[derive(Parser)]
struct Args {
    /// Output directory for trajectory.json and summary.json.
    output: PathBuf,
    /// Keep the right pad throughout the trial for a matched positive control.
    #[arg(long)]
    keep_both: bool,
    /// Native advancing steps after the pad edit and before the first resumed solve.
    #[arg(long, default_value_t = 8)]
    delay_steps: u64,
    /// Measure the automatic pre-solve policy under a specific edit and pause mode.
    #[arg(long, value_enum, conflicts_with_all=["keep_both", "delay_steps"])]
    auto_loss: Option<auto_loss::Mode>,
}
struct Lane {
    sim: Simulation,
    model: u64,
    upper_link: u64,
    right_model: u64,
    foot: BodyId,
    upper: BodyId,
    pads: [BodyId; 2],
    both: bool,
}
impl Lane {
    fn new() -> Result<Self> {
        Self::from_sdf(SDF)
    }
    fn from_sdf(source: &str) -> Result<Self> {
        let mut sim = load_world_with_sdf_step(
            source,
            "two_support_foot.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        sim.step_exact(2400)?;
        let id = |name: &str| -> Result<u64> {
            let name = format!("default::{name}");
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .and_then(|e| e.get::<EntityId>())
                .map(|id| id.get())
                .ok_or_else(|| format!("missing {name}").into())
        };
        let body = |name: &str| -> Result<BodyId> {
            let e = entity_by_id(sim.world(), id(name)?).ok_or("missing ECS entity")?;
            Ok(sim
                .world()
                .get::<BodyHandle>(e)
                .ok_or("missing body handle")?
                .id())
        };
        Ok(Self {
            model: id("rocker")?,
            upper_link: id("rocker::upper")?,
            right_model: id("right")?,
            foot: body("rocker::foot")?,
            upper: body("rocker::upper")?,
            pads: [body("left::pad")?, body("right::pad")?],
            both: true,
            sim,
        })
    }
    fn solver(&self) -> &MultiFidelityBackend {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref()
            .expect("local multifidelity scene")
    }
    fn cache(&self) -> Result<[Value; 2]> {
        let read = |pad| -> Result<Value> {
            match self.solver().contact_warmstart(self.foot, pad) {
                Ok(c) => Ok(
                    json!({"points":c.points,"normal_ns":c.normal_ns,"tangent_on_foot_ns":c.tangent_on_a_ns}),
                ),
                Err(PhysicsError::MissingBody) if !self.both && pad == self.pads[1] => {
                    Ok(Value::Null)
                }
                Err(e) => Err(e.into()),
            }
        };
        Ok([read(self.pads[0])?, read(self.pads[1])?])
    }
    fn held_pairs(&self) -> [Option<Value>; 2] {
        self.pads.map(|pad| {
            self.solver()
                .held_support_pair_status(self.foot, pad)
                .map(|s| {
                    json!({
            "expiry":s.expiry.map(|e| format!("{e:?}")), "edge_anchors":s.edge_anchors,
            "max_face_overhang_m":s.max_face_overhang_m})
                })
        })
    }

    fn edit(&mut self) -> Result<()> {
        remove_model(&mut self.sim, self.right_model)?;
        self.both = false;
        Ok(())
    }
    fn pitch(&mut self, torque: f64) -> Result<()> {
        queue_link_world_wrench(&mut self.sim, self.upper_link, [0.; 3], [0., torque, 0.])?;
        Ok(())
    }
    fn sample(&self) -> Result<Sample> {
        let physics = self.sim.world().resource::<PhysicsBackend>();
        sample_world(physics, self.foot, self.upper, self.pads)
    }
}

fn sample_world(
    physics: &PhysicsBackend,
    foot: BodyId,
    upper: BodyId,
    pads: [BodyId; 2],
) -> Result<Sample> {
    let mut impulses = [0.; 2];
    let mut vectors = [DVec3::ZERO; 2];
    for patch in physics
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or("missing native solver")?
        .rigid_contact_patches()
    {
        for (i, pad) in pads.iter().copied().enumerate() {
            let sign = if (patch.a, patch.b) == (foot, pad) {
                1.
            } else if (patch.a, patch.b) == (pad, foot) {
                -1.
            } else {
                continue;
            };
            let impulse = patch.points.iter().map(|p| p.normal_impulse).sum::<f64>();
            impulses[i] += impulse;
            vectors[i] += sign * DVec3::from(patch.normal_on_a) * impulse;
        }
    }
    Ok(Sample {
        foot: physics.state(foot).ok_or("missing foot")?.into(),
        upper: physics.state(upper).ok_or("missing upper")?.into(),
        normal_impulses_ns: impulses,
        normal_vectors_on_foot_ns: vectors.map(|v| v.to_array()),
    })
}

#[derive(Serialize, Clone, Copy, PartialEq)]
struct Body {
    position: [f64; 3],
    orientation: [f64; 4],
    velocity: [f64; 3],
    angular_velocity: [f64; 3],
}
impl From<BodyState> for Body {
    fn from(s: BodyState) -> Self {
        Self {
            position: s.position,
            orientation: s.orientation,
            velocity: s.velocity,
            angular_velocity: s.angular_velocity,
        }
    }
}
#[derive(Serialize, Clone, Copy, PartialEq)]
struct Sample {
    foot: Body,
    upper: Body,
    normal_impulses_ns: [f64; 2],
    normal_vectors_on_foot_ns: [[f64; 3]; 2],
}
#[derive(Serialize)]
struct Frame {
    tick: u64,
    time_s: f64,
    prescribed: bool,
    support_b_present: bool,
    upper_torque_y_nm: f64,
    switched: Sample,
    reference: Sample,
}
#[derive(Serialize)]
struct Recording {
    dt_s: f64,
    geometry: Value,
    frames: Vec<Frame>,
}
fn pose_error(a: Sample, b: Sample) -> [f64; 4] {
    let dp = |p: Body, q: Body| DVec3::from(p.position).distance(q.position.into());
    let angle = |p: Body, q: Body| {
        let d = DQuat::from_array(p.orientation).conjugate() * DQuat::from_array(q.orientation);
        2. * d.xyz().length().atan2(d.w.abs())
    };
    [
        dp(a.foot, b.foot),
        dp(a.upper, b.upper),
        angle(a.foot, b.foot),
        angle(a.upper, b.upper),
    ]
}
/// Each link has mass 1 kg. At promotion the whole assembly has only gravity
/// and pad contact as external impulses. After removal the measured momentum
/// balance therefore also gives the complete reaction on the surviving pad.
fn total_ground_impulse(previous: Sample, current: Sample, dt: f64) -> [f64; 3] {
    let momentum = |s: Sample| DVec3::from(s.foot.velocity) + DVec3::from(s.upper.velocity);
    (momentum(current) - momentum(previous) - 2. * dt * DVec3::new(-0.4, 0., -9.81)).to_array()
}

fn load_at(tick: u64) -> f64 {
    match tick {
        RIGHT_TORQUE_START..LEFT_TORQUE_START => 0.70,
        LEFT_TORQUE_START..=TORQUE_END => -0.70,
        _ => 0.,
    }
}
fn compare(keep_both: bool, delay: u64) -> Result<(Value, Recording)> {
    let mut switched = Lane::new()?;
    let mut reference = Lane::new()?;
    // This control deliberately holds the foot for a caller-selected delay.
    gz_sim_scene::support_loss_promotion::set_enabled(&mut switched.sim, switched.model, false)?;
    let dt = reference.sim.step_duration().as_secs_f64();
    let promotion_tick = LOSS.checked_add(delay).ok_or("delay too long")?;
    let end = promotion_tick
        .checked_add(POST_STEPS - 1)
        .ok_or("delay too long")?;
    let mut frames: Vec<Frame> = Vec::new();
    let mut demotion = Value::Null;
    let mut loss = Value::Null;
    let mut promotion = Value::Null;
    let mut peak = [0_f64; 4];
    let mut held = [0.; 2];
    let mut post_lost = [0.; 2];
    for tick in 0..=end {
        if tick == LOSS && !keep_both {
            let a = switched.cache()?;
            let b = reference.cache()?;
            let pairs = switched.held_pairs();
            switched.edit()?;
            reference.edit()?;
            loss = json!({"switched_before":a,"reference_before":b,"switched_after":switched.cache()?,"reference_after":reference.cache()?,
                "held_pairs_before":pairs,"held_pairs_after":switched.held_pairs()});
        }
        let torque = load_at(tick);
        if torque != 0. {
            switched.pitch(torque)?;
            reference.pitch(torque)?;
        }
        if tick == promotion_tick {
            let before = switched.cache()?;
            let pairs = switched.held_pairs();
            let original = switched.sample()?;
            let ref_before = reference.sample()?;
            set_model_fidelity(&mut switched.sim, switched.model, ArticulationTier::Dynamic)?;
            let after = switched.sample()?;
            if original.foot != after.foot || original.upper != after.upper {
                return Err("promotion changed a pose or twist".into());
            }
            promotion = json!({"switched_before":before,"switched_after":switched.cache()?,"reference_cache":reference.cache()?,
                "held_pairs_before":pairs,"pose_error_before_first_resumed_solve":pose_error(original,ref_before),"poses_and_twists_continuous":true});
        }
        if tick > 0 {
            switched.sim.step_exact(1)?;
            reference.sim.step_exact(1)?;
        }
        let a = switched.sample()?;
        let b = reference.sample()?;
        if tick <= DEMOTE && a != b {
            return Err("pre-demotion lanes disagree".into());
        }
        let prescribed = tick > DEMOTE && tick < promotion_tick;
        if prescribed {
            for (i, value) in a.normal_impulses_ns.into_iter().enumerate() {
                held[i] += value;
            }
        }
        if tick >= LOSS && !keep_both {
            post_lost[0] += a.normal_impulses_ns[1];
            post_lost[1] += b.normal_impulses_ns[1];
        }
        if tick == DEMOTE {
            let before = switched.cache()?;
            set_model_fidelity(
                &mut switched.sim,
                switched.model,
                ArticulationTier::Prescribed,
            )?;
            let after = switched.sample()?;
            if a.foot != after.foot || a.upper != after.upper {
                return Err("demotion changed a pose or twist".into());
            }
            demotion = json!({"switched_before":before,"switched_after":switched.cache()?,"reference_cache":reference.cache()?,"held_pairs":switched.held_pairs(),"normal_impulses_ns":a.normal_impulses_ns,"poses_and_twists_continuous":true});
        }
        if tick == promotion_tick {
            let previous = frames.last().ok_or("missing boundary sample")?;
            promotion["first_resumed"] = json!({"switched_normal_ns":a.normal_impulses_ns,"reference_normal_ns":b.normal_impulses_ns,
                "switched_mean_normal_n":a.normal_impulses_ns.map(|p|p/dt),"reference_mean_normal_n":b.normal_impulses_ns.map(|p|p/dt),
                "switched_total_ground_impulse_world_ns":total_ground_impulse(previous.switched,a,dt),
                "reference_total_ground_impulse_world_ns":total_ground_impulse(previous.reference,b,dt),
                "switched_cache":switched.cache()?,"reference_cache":reference.cache()?,"switched_pitch_rate_rad_s":a.foot.angular_velocity[1],"reference_pitch_rate_rad_s":b.foot.angular_velocity[1]});
        }
        let e = pose_error(a, b);
        for (i, error) in e.into_iter().enumerate() {
            peak[i] = peak[i].max(error);
        }
        frames.push(Frame {
            tick,
            time_s: tick as f64 * dt,
            prescribed,
            support_b_present: switched.both,
            upper_torque_y_nm: torque,
            switched: a,
            reference: b,
        });
    }
    let mean_load = |start: u64, end: u64| {
        let mut forces = [0.; 2];
        for frame in &frames[start as usize..=end as usize] {
            for (i, f) in forces.iter_mut().enumerate() {
                *f += frame.reference.normal_impulses_ns[i] / dt / (end + 1 - start) as f64;
            }
        }
        forces
    };
    let summary = json!({"dt_s":dt,"settle_s":1.2,"demotion_tick":DEMOTE,"loss_tick":if keep_both{None}else{Some(LOSS)},"promotion_tick":promotion_tick,"end_tick":end,"keep_both":keep_both,"delay_steps":delay,
        "edit_boundary_time_s":if keep_both{None}else{Some((LOSS-1)as f64*dt)},
        "promotion_boundary_time_s":(promotion_tick-1)as f64*dt,"first_resumed_time_s":promotion_tick as f64*dt,
        "torque_start_tick":RIGHT_TORQUE_START,"load_transfer_tick":LEFT_TORQUE_START,"torque_end_tick":TORQUE_END,"torque_magnitude_nm":0.70,"supports":["left","right"],
        "mean_reference_loads_n":{"positive_torque_ticks_70_to_95":mean_load(70,95),"negative_torque_ticks_145_to_170":mean_load(145,170)},
        "demotion":demotion,"loss":loss,"promotion":promotion,"held_normal_impulses_ns":held,"lost_pad_total_normal_ns_after_removal_switched_reference":post_lost,
        "peak_pose_error":{"foot_center_m":peak[0],"upper_center_m":peak[1],"foot_angle_rad":peak[2],"upper_angle_rad":peak[3]}});
    let geometry = geometry();
    Ok((
        summary,
        Recording {
            dt_s: dt,
            geometry,
            frames,
        },
    ))
}
fn geometry() -> Value {
    json!({"support_a_center":[-0.125,0.,-0.06],"support_b_center":[0.125,0.,-0.06],"support_a_half_extents":[0.075,0.16,0.06],"support_b_half_extents":[0.075,0.16,0.06],"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06]})
}

fn main() -> Result<()> {
    let args = Args::parse();
    let (mut summary, record) = match args.auto_loss {
        Some(mode) => auto_loss::run(mode)?,
        None => compare(args.keep_both, args.delay_steps)?,
    };
    if matches!(args.auto_loss, Some(auto_loss::Mode::Running)) {
        summary["manual_controls"] = auto_loss::manual_controls(&record)?;
    }
    fs::create_dir_all(&args.output)?;
    let mut out = BufWriter::new(fs::File::create(args.output.join("trajectory.json"))?);
    serde_json::to_writer(&mut out, &record)?;
    out.flush()?;
    fs::write(
        args.output.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&summary)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn removing_one_loaded_pad_preserves_the_other_and_loads_it_on_the_first_solve() -> Result<()> {
        let (s, recording) = compare(false, 8)?;
        let mean = &s["mean_reference_loads_n"];
        let plus = &mean["positive_torque_ticks_70_to_95"];
        let minus = &mean["negative_torque_ticks_145_to_170"];
        assert!(plus[1].as_f64().unwrap() - plus[0].as_f64().unwrap() > 3.);
        assert!(minus[0].as_f64().unwrap() - minus[1].as_f64().unwrap() > 3.);
        let d = &s["demotion"];
        for i in 0..2 {
            assert!(d["switched_before"][i]["normal_ns"].as_f64().unwrap() > 0.);
            assert!(
                d["switched_before"][i]["tangent_on_foot_ns"][0]
                    .as_f64()
                    .unwrap()
                    .abs()
                    > 1e-6
            );
            assert_eq!(d["switched_after"][i], d["switched_before"][i]);
            assert!(d["held_pairs"][i].is_object() && d["held_pairs"][i]["expiry"].is_null());
        }
        assert_eq!(s["held_normal_impulses_ns"], json!([0., 0.]));
        let loss = &s["loss"];
        assert_eq!(loss["switched_before"][0], loss["switched_after"][0]);
        assert!(loss["switched_after"][1].is_null());
        assert_eq!(loss["held_pairs_before"][0], loss["held_pairs_after"][0]);
        assert!(loss["held_pairs_after"][1].is_null());
        let p = &s["promotion"];
        assert!(p["switched_before"][0]["normal_ns"].as_f64().unwrap() > 0.);
        assert_eq!(p["switched_before"], p["switched_after"]);
        assert!(p["switched_after"][1].is_null());
        assert_eq!(
            s["lost_pad_total_normal_ns_after_removal_switched_reference"],
            json!([0., 0.])
        );
        let first = &recording.frames[(LOSS + 8) as usize];
        for sample in [first.switched, first.reference] {
            assert!(sample.normal_impulses_ns[0] > 0. && sample.normal_impulses_ns[1] == 0.);
            assert!(sample.foot.angular_velocity[1] > 1e-3);
        }
        assert!(
            (first.switched.normal_impulses_ns[0] - first.reference.normal_impulses_ns[0]).abs()
                / recording.dt_s
                < 1.
        );
        Ok(())
    }

    #[test]
    fn two_unchanged_pads_retain_both_caches_and_resolve_individual_reactions() -> Result<()> {
        let (s, recording) = compare(true, 8)?;
        let p = &s["promotion"];
        let first = &recording.frames[(LOSS + 8) as usize];
        for i in 0..2 {
            assert!(
                p["held_pairs_before"][i].is_object()
                    && p["held_pairs_before"][i]["expiry"].is_null()
            );
            assert!(p["switched_after"][i]["normal_ns"].as_f64().unwrap() > 0.);
            assert_eq!(p["switched_before"][i], p["switched_after"][i]);
            assert!(
                (first.switched.normal_impulses_ns[i] / first.reference.normal_impulses_ns[i] - 1.)
                    .abs()
                    < 0.05
            );
        }
        Ok(())
    }
}
