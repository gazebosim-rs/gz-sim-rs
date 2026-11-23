// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Follow an originally loaded toe corner as it slides and pitches across a finite ledge.
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::{ContactWarmstart, MultiFidelityBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, set_model_fidelity,
};
use serde::Serialize;
use serde_json::{Value, json};
use std::{error::Error, fs, io::Write, path::PathBuf};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_rocking_foot.sdf");
const DEMOTE: u64 = 9;
const POST_STEPS: u64 = 3200;
const DEFAULT_HOLD: u64 = 4000;
const TOE: DVec3 = DVec3::new(0.08, -0.10, -0.06);

#[derive(Parser)]
struct Args {
    /// Output directory for summary.json and trajectory.json.
    output: PathBuf,
    /// Number of native steps between demotion and promotion. Zero tests a round trip.
    #[arg(long, default_value_t = DEFAULT_HOLD)]
    hold_steps: u64,
}

struct Lane {
    sim: Simulation,
    model: u64,
    foot: BodyId,
    upper: BodyId,
    ledge: BodyId,
}

impl Lane {
    fn new() -> Result<Self> {
        let sim = load_world_with_sdf_step(
            SDF,
            "rocking_foot.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
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
            let entity = entity_by_id(sim.world(), id(name)?).ok_or("missing entity")?;
            Ok(sim
                .world()
                .get::<BodyHandle>(entity)
                .ok_or("missing body")?
                .id())
        };
        Ok(Self {
            model: id("rocker")?,
            foot: body("rocker::foot")?,
            upper: body("rocker::upper")?,
            ledge: body("ledge::ground")?,
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

    fn seed(&self) -> Result<ContactWarmstart> {
        Ok(self.solver().contact_warmstart(self.foot, self.ledge)?)
    }

    fn held_status(&self) -> Option<Value> {
        self.solver().held_support_status(self.foot).map(|s| {
            json!({
                "expiry":s.expiry.map(|r| format!("{r:?}")), "edge_anchors":s.edge_anchors,
                "max_face_overhang_m":s.max_face_overhang_m,
            })
        })
    }

    fn sample(&self) -> Result<Sample> {
        let physics = self.sim.world().resource::<PhysicsBackend>();
        let foot = physics.state(self.foot).ok_or("missing foot")?;
        let upper = physics.state(self.upper).ok_or("missing upper link")?;
        let mut normal_impulse_ns = 0.;
        let mut normal_vector_ns = DVec3::ZERO;
        for patch in self.solver().rigid_contact_patches() {
            let sign = if (patch.a, patch.b) == (self.foot, self.ledge) {
                1.
            } else if (patch.b, patch.a) == (self.foot, self.ledge) {
                -1.
            } else {
                continue;
            };
            let impulse = patch.points.iter().map(|p| p.normal_impulse).sum::<f64>();
            normal_impulse_ns += impulse;
            normal_vector_ns += sign * DVec3::from(patch.normal_on_a) * impulse;
        }
        let toe = DVec3::from(foot.position) + DQuat::from_array(foot.orientation) * TOE;
        Ok(Sample {
            foot: foot.into(),
            upper: upper.into(),
            edge_distance_m: toe.x,
            normal_impulse_ns,
            normal_vector_ns: normal_vector_ns.to_array(),
        })
    }

    fn loaded_points(&self) -> Vec<Value> {
        let mut points = Vec::new();
        let state = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .state(self.foot)
            .unwrap();
        let rotation = DQuat::from_array(state.orientation);
        for patch in self.solver().rigid_contact_patches() {
            let first = patch.a == self.foot && patch.b == self.ledge;
            if !first && !(patch.b == self.foot && patch.a == self.ledge) {
                continue;
            }
            for p in patch.points.into_iter().filter(|p| p.normal_impulse > 0.) {
                let lever = DVec3::from(if first { p.lever_a } else { p.lever_b });
                points.push(json!({"foot_local_m":(rotation.conjugate()*lever).to_array(),
                    "world_m":(DVec3::from(state.position)+lever).to_array(),"normal_impulse_ns":p.normal_impulse}));
            }
        }
        points
    }
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
    edge_distance_m: f64,
    normal_impulse_ns: f64,
    normal_vector_ns: [f64; 3],
}
#[derive(Serialize)]
struct Frame {
    tick: u64,
    time_s: f64,
    prescribed: bool,
    switched: Sample,
    reference: Sample,
}
#[derive(Serialize, Default, Clone, Copy)]
struct Errors {
    foot_center_m: f64,
    upper_center_m: f64,
    foot_angle_rad: f64,
    upper_angle_rad: f64,
}
impl Errors {
    fn at(a: Sample, b: Sample) -> Self {
        let position = |x: Body, y: Body| DVec3::from(x.position).distance(y.position.into());
        let angle = |x: Body, y: Body| {
            let q = DQuat::from_array(x.orientation).conjugate() * DQuat::from_array(y.orientation);
            2. * q.xyz().length().atan2(q.w.abs())
        };
        Self {
            foot_center_m: position(a.foot, b.foot),
            upper_center_m: position(a.upper, b.upper),
            foot_angle_rad: angle(a.foot, b.foot),
            upper_angle_rad: angle(a.upper, b.upper),
        }
    }
    fn include(&mut self, e: Self) {
        self.foot_center_m = self.foot_center_m.max(e.foot_center_m);
        self.upper_center_m = self.upper_center_m.max(e.upper_center_m);
        self.foot_angle_rad = self.foot_angle_rad.max(e.foot_angle_rad);
        self.upper_angle_rad = self.upper_angle_rad.max(e.upper_angle_rad);
    }
}
fn seed(s: ContactWarmstart) -> Value {
    json!({"points":s.points,"normal_ns":s.normal_ns,"tangent_on_foot_ns":s.tangent_on_a_ns})
}
fn compare(hold: u64) -> Result<(Value, Vec<Frame>)> {
    let mut switched = Lane::new()?;
    let mut reference = Lane::new()?;
    let dt = switched.sim.step_duration().as_secs_f64();
    let promote = DEMOTE
        .checked_add(hold)
        .and_then(|t| t.checked_add(1))
        .ok_or("hold too long")?;
    let end = promote
        .checked_add(POST_STEPS - 1)
        .ok_or("trial too long")?;
    let mut frames = Vec::new();
    let mut crossing = None;
    let mut expired = None;
    let mut held_load = 0_f64;
    let mut peak = Errors::default();
    let mut peak_post = Errors::default();
    let mut demotion = Value::Null;
    let mut promotion = Value::Null;
    for tick in 0..=end {
        // A frame describes the step just completed. Tier switches leave poses, twists, and
        // the clock alone. Capture both sides of each boundary BEFORE advancing either lane.
        if tick == promote {
            let before = switched.sample()?;
            let ref_before = reference.sample()?;
            let cached = switched.seed()?;
            let status = switched.held_status();
            set_model_fidelity(&mut switched.sim, switched.model, ArticulationTier::Dynamic)?;
            let after = switched.sample()?;
            if before.foot != after.foot || before.upper != after.upper {
                return Err("promotion changed a boundary state".into());
            }
            promotion = json!({"before_cache":seed(cached),"after_cache":seed(switched.seed()?),
                "held_status":status,"pose_error_before_first_resumed_step":Errors::at(before,ref_before),
                "positions_and_twists_continuous":true});
        }
        if tick > 0 {
            switched.sim.step_exact(1)?;
            reference.sim.step_exact(1)?;
        }
        let a = switched.sample()?;
        let b = reference.sample()?;
        if tick <= DEMOTE && a != b {
            return Err("pre-demotion controls disagree".into());
        }
        if tick == DEMOTE {
            let before = switched.seed()?;
            let points = switched.loaded_points();
            set_model_fidelity(
                &mut switched.sim,
                switched.model,
                ArticulationTier::Prescribed,
            )?;
            let after = switched.sample()?;
            if a.foot != after.foot || a.upper != after.upper {
                return Err("demotion changed a boundary state".into());
            }
            demotion = json!({"before_cache":seed(before),"after_cache":seed(switched.seed()?),
                "normal_impulse_ns":a.normal_impulse_ns,"positively_loaded_points":points,"held_status":switched.held_status(),
                "pitch_rate_rad_s":a.foot.angular_velocity[1],"positions_and_twists_continuous":true});
        }
        let prescribed = tick > DEMOTE && tick < promote;
        if prescribed {
            held_load += a.normal_impulse_ns;
            if a.edge_distance_m > 0. && crossing.is_none() {
                crossing = Some(tick);
            }
            if expired.is_none()
                && switched
                    .solver()
                    .held_support_status(switched.foot)
                    .is_some_and(|s| s.expiry.is_some())
            {
                expired = Some(tick);
            }
        }
        if tick == promote {
            promotion["first_resumed_step"] = json!({"switched_normal_impulse_ns":a.normal_impulse_ns,
                "reference_normal_impulse_ns":b.normal_impulse_ns,
                "normal_impulse_difference_ns":a.normal_impulse_ns-b.normal_impulse_ns,
                "switched_mean_normal_force_n":a.normal_impulse_ns/dt,
                "reference_mean_normal_force_n":b.normal_impulse_ns/dt,
                "freshly_solved_cache":seed(switched.seed()?),
                "loaded_points":switched.loaded_points(),"reference_loaded_points":reference.loaded_points()});
        }
        let error = Errors::at(a, b);
        peak.include(error);
        if tick >= promote {
            peak_post.include(error);
        }
        frames.push(Frame {
            tick,
            time_s: tick as f64 * dt,
            prescribed,
            switched: a,
            reference: b,
        });
    }
    let last = frames.last().unwrap();
    let impulse_difference = frames
        .iter()
        .skip(promote as usize)
        .map(|f| (f.switched.normal_impulse_ns - f.reference.normal_impulse_ns).abs())
        .fold(0_f64, f64::max);
    let summary = json!({"dt_s":dt,"demotion_tick":DEMOTE,"hold_steps":hold,"crossing_tick":crossing,
        "expiry_tick":expired,"promotion_tick":promote,"end_tick":end,"witness_local_m":TOE.to_array(),
        "demotion":demotion,"promotion":promotion,"held_normal_impulse_ns":held_load,
        "peak_pose_error":peak,"peak_post_promotion_pose_error":peak_post,"end_pose_error":Errors::at(last.switched,last.reference),
        "peak_resumed_normal_impulse_error_ns":impulse_difference,
        "reference_crossing_tick":frames.iter().find(|f|f.reference.edge_distance_m>0.).map(|f|f.tick),
        "reference_last_loaded_tick":frames.iter().rfind(|f|f.reference.normal_impulse_ns>0.).map(|f|f.tick)});
    Ok((summary, frames))
}
fn main() -> Result<()> {
    let args = Args::parse();
    let (summary, frames) = compare(args.hold_steps)?;
    fs::create_dir_all(&args.output)?;
    let geometry = json!({"ledge_center":[-0.2,0.,-0.08],"ledge_half_extents":[0.2,0.25,0.08],
        "foot_half_extents":[0.08,0.10,0.06],"upper_half_extents":[0.05,0.06,0.06],
        "witness_local_m":TOE.to_array()});
    #[derive(Serialize)]
    struct Recording {
        dt_s: f64,
        geometry: Value,
        frames: Vec<Frame>,
    }
    let recording = Recording {
        dt_s: summary["dt_s"].as_f64().unwrap(),
        geometry,
        frames,
    };
    let mut trajectory =
        std::io::BufWriter::new(fs::File::create(args.output.join("trajectory.json"))?);
    serde_json::to_writer(&mut trajectory, &recording)?;
    trajectory.flush()?;
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
    fn material_toe_crossing_expires_only_the_old_impulses_before_the_next_solve() -> Result<()> {
        // Request promotion immediately after the first step whose toe is outside the ledge.
        let (summary, frames) = compare(2286 - DEMOTE)?;
        let demotion = &summary["demotion"];
        assert_eq!(
            demotion["positively_loaded_points"]
                .as_array()
                .unwrap()
                .len(),
            4
        );
        assert_eq!(demotion["held_status"]["edge_anchors"], 2);
        assert!(demotion["pitch_rate_rad_s"].as_f64().unwrap() > 1e-4);
        let crossing = summary["crossing_tick"].as_u64().unwrap();
        assert_eq!(summary["expiry_tick"], crossing);
        assert_eq!(summary["promotion_tick"], crossing + 1);
        assert!(frames[crossing as usize - 1].switched.edge_distance_m <= 0.);
        assert!(
            frames[crossing as usize].prescribed
                && frames[crossing as usize].switched.edge_distance_m > 0.
        );
        assert!(frames.iter().all(|f| f.reference.edge_distance_m < 0.));
        assert_eq!(summary["held_normal_impulse_ns"], 0.);
        let promotion = &summary["promotion"];
        assert_eq!(promotion["held_status"]["expiry"], "AnchorOutsideFace");
        assert!(promotion["before_cache"]["normal_ns"].as_f64().unwrap() > 0.);
        assert!(
            promotion["before_cache"]["tangent_on_foot_ns"][0]
                .as_f64()
                .unwrap()
                < 0.
        );
        assert_eq!(promotion["after_cache"], seed(ContactWarmstart::default()));
        let first = &frames[(crossing + 1) as usize];
        assert!(first.switched.normal_impulse_ns > 0. && first.reference.normal_impulse_ns > 0.);
        assert!(
            (first.switched.normal_impulse_ns / first.reference.normal_impulse_ns - 1.).abs()
                < 0.05
        );
        assert!(
            summary["peak_pose_error"]["foot_center_m"]
                .as_f64()
                .unwrap()
                < 0.0001
        );
        Ok(())
    }

    #[test]
    fn supported_inboard_toe_resumes_with_the_same_valid_cache() -> Result<()> {
        let (summary, _) = compare(1600)?;
        assert!(summary["crossing_tick"].is_null() && summary["expiry_tick"].is_null());
        let promotion = &summary["promotion"];
        assert!(
            promotion["held_status"]["max_face_overhang_m"]
                .as_f64()
                .unwrap()
                < 0.
        );
        assert!(promotion["before_cache"]["normal_ns"].as_f64().unwrap() > 0.);
        assert_eq!(promotion["before_cache"], promotion["after_cache"]);
        assert_eq!(summary["held_normal_impulse_ns"], 0.);
        Ok(())
    }
}
