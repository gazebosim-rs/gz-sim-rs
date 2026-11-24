// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Matched airborne starts, post-impact pressure and measured admission on a parked support.
#[path = "support/settle_angular.rs"]
mod angular;
#[path = "support/settle_microtilt.rs"]
mod microtilt;
#[path = "support/settle_departure.rs"]
mod departure;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, DynamicBackend, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions,
    load_world_with_runtime, primary_contact::last_refinement, queue_link_world_velocity,
};
use rapier3d_f64::{
    parry::{
        query::{self, ContactManifold},
        shape::Cuboid,
    },
    prelude::{Pose, Rotation, Vector},
};
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_primary_settling.sdf");
const DT: f64 = 0.0005;
const PARK: u64 = 160;
const LAST: u64 = 2560;
const POLICIES: [&str; 4] = ["ordinary", "relax8", "relax32", "refresh8"];
fn xml(edge: bool, mu: f64, rigid: bool, policy: &str) -> String {
    let mut s = SDF.replace(
        "<mu>0.2</mu><mu2>0.2</mu2>",
        &format!("<mu>{mu}</mu><mu2>{mu}</mu2>"),
    );
    if !edge {
        s = s
            .replace("<pose>0.18 0 -0.06", "<pose>0 0 -0.06")
            .replace("0.074 0 0.035", "0.074 0 0");
    }
    if policy == "ordinary" {
        let a = s
            .find("    <plugin name=\"gz::sim::systems::PrimaryContactRefinement\"")
            .unwrap();
        let b = a + s[a..].find("</plugin>").unwrap() + 9;
        s.replace_range(a..b, "");
    }
    if policy == "relax32" {
        s = s.replace("<relaxation_iterations>8", "<relaxation_iterations>32");
    }
    if policy == "refresh8" {
        s = s.replace(
            "</support>",
            "</support><refresh_box_features>true</refresh_box_features>",
        );
    }
    if rigid {
        let a = s
            .find("      <plugin name=\"gz::sim::systems::SwitchableArticulation\"")
            .unwrap();
        let b = a + s[a..].find("</plugin>").unwrap() + 9;
        s.replace_range(a..b, "");
        let a = s.find("      <link name=\"upper\">").unwrap();
        let b = a + s[a..].find("</joint>").unwrap() + 8;
        s.replace_range(a..b, "");
        s = s
            .replace(
                "<mass>1</mass>",
                "<pose>0 0 0.07 0 0 0</pose><mass>2</mass>",
            )
            .replace("0.0201</", "0.05</")
            .replace("0.025</", "0.05</");
    }
    s
}
fn bind(s: &Simulation, name: &str) -> Result<(u64, BodyId)> {
    #[allow(deprecated)]
    s.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .and_then(|e| Some((e.get::<EntityId>()?.get(), e.get::<BodyHandle>()?.id())))
        .ok_or_else(|| name.to_owned().into())
}
fn wire(b: BodyState) -> Value {
    json!({"position":b.position,"velocity":b.velocity,"orientation":b.orientation,"angular_velocity":b.angular_velocity})
}
#[derive(Clone, Copy)]
struct Metric {
    com: DVec3,
    q: DQuat,
    spin: DVec3,
    v: DVec3,
    linear: f64,
    angular: f64,
    energy: f64,
    weight: f64,
    cop: f64,
    points: usize,
    mismatch: Option<f64>,
    accept: bool,
}
impl Metric {
    fn describe(self) -> Value {
        json!({"com_m":self.com.to_array(),"orientation":self.q.to_array(),"com_velocity_m_s":self.v.to_array(),"spin_rad_s":self.spin.to_array(),
        "max_link_speed_m_s":self.linear,"max_link_spin_rad_s":self.angular,"kinetic_energy_j":self.energy,"normal_load_weight_ratio":self.weight,
        "projected_normal_center_of_pressure_offset_m":self.cop,"loaded_points":self.points,"fresh_normal_mismatch_rad":self.mismatch,"eligible":self.accept})
    }
}
struct Config {
    edge: bool,
    mu: f64,
    rigid: bool,
    per_point: bool,
    policy: &'static str,
}
#[derive(Clone, Copy)]
struct Reading {
    foot: BodyId,
    upper: Option<BodyId>,
    pad: BodyId,
}
fn metric(
    s: &MultiFidelityBackend,
    ids: &Reading,
    states: (BodyState, Option<BodyState>, BodyState),
    eligible: bool,
) -> Metric {
    let (f, u, p) = states;
    let fv = DVec3::from(f.velocity);
    let spin = DVec3::from(f.angular_velocity);
    let q = DQuat::from_array(f.orientation);
    let pos = DVec3::from(f.position);
    let (com, v, lin, ang, energy, foot_com) = if let Some(u) = u {
        let uv = DVec3::from(u.velocity);
        let w = DVec3::from(u.angular_velocity);
        let local = q.conjugate() * spin;
        let local_u = DQuat::from_array(u.orientation).conjugate() * w;
        let moment = DVec3::new(0.0201, 0.0201, 0.025);
        (
            (pos + DVec3::from(u.position)) * 0.5,
            (fv + uv) * 0.5,
            fv.length().max(uv.length()),
            spin.length().max(w.length()),
            0.5 * (fv.length_squared()
                + uv.length_squared()
                + (local * local + local_u * local_u).dot(moment)),
            pos,
        )
    } else {
        let center = pos + q * DVec3::new(0., 0., 0.07);
        (
            center,
            fv,
            fv.length(),
            spin.length(),
            fv.length_squared() + 0.025 * spin.length_squared(),
            center,
        )
    };
    let pose = |v: BodyState| {
        Pose::from_parts(
            Vector::from_array(v.position),
            Rotation::from_array(v.orientation),
        )
    };
    let pp = pose(p);
    let fp = pose(f);
    let mut fresh = ContactManifold::<(), ()>::default();
    query::details::contact_manifold_cuboid_cuboid(
        &pp.inv_mul(&fp),
        &Cuboid::new(Vector::new(0.33, 0.18, 0.06)),
        &Cuboid::new(Vector::new(0.18, 0.1, 0.06)),
        0.002,
        &mut fresh,
    );
    let fresh_normal = (!fresh.points.is_empty())
        .then(|| DQuat::from_array(p.orientation) * DVec3::from_array(fresh.local_n1.to_array()));
    let mut total = 0.;
    let mut cop = DVec3::ZERO;
    let mut count = 0;
    let mut disagreement: Option<f64> = None;
    for patch in s
        .rigid_contact_patches()
        .into_iter()
        .filter(|c| (c.a, c.b) == (ids.foot.min(ids.pad), ids.foot.max(ids.pad)))
    {
        let left = patch.a == ids.foot;
        let normal = DVec3::from(patch.normal_on_a) * if left { 1. } else { -1. };
        for point in patch.points.into_iter().filter(|p| p.normal_impulse > 0.) {
            let j = normal.z * point.normal_impulse;
            total += j;
            let r = if left { point.lever_a } else { point.lever_b };
            cop += (foot_com + DVec3::from(r) - com) * j;
            count += 1;
            if let Some(n) = fresh_normal {
                let angle = normal.cross(n).length().atan2(normal.dot(n));
                disagreement = Some(disagreement.unwrap_or(0.).max(angle));
            }
        }
    }
    Metric {
        com,
        q,
        spin,
        v,
        linear: lin,
        angular: ang,
        energy,
        weight: total / (2. * 9.81 * DT),
        cop: if total > 0. {
            cop.truncate().length() / total
        } else {
            0.
        },
        points: count,
        mismatch: disagreement,
        accept: eligible,
    }
}
fn trial(c: Config) -> Result<(Value, Vec<Value>)> {
    let scene = xml(c.edge, c.mu, c.rigid, c.policy);
    let mut sim = load_world_with_runtime(
        &scene,
        "settling.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_per_point_friction: c.per_point,
            ..Default::default()
        },
    )?;
    let (pad_link, pad) = bind(&sim, "default::lift::pad")?;
    let ids = Reading {
        foot: bind(&sim, "default::rocker::foot")?.1,
        upper: bind(&sim, "default::rocker::upper").ok().map(|i| i.1),
        pad,
    };
    let mut early = vec![];
    let mut first = None;
    let mut starts = vec![];
    let mut blocked: BTreeMap<String, u64> = BTreeMap::new();
    let mut metrics = Vec::with_capacity((LAST + 1) as usize);
    let mut trace = vec![];
    let dt = sim.step_duration();
    for tick in 0..=LAST {
        if tick > 0 {
            let boundary = tick - 1;
            if (40..=PARK).contains(&boundary) {
                let cmd = if boundary == PARK {
                    0.
                } else {
                    0.30 * (1. - ((boundary - 40) as f64 + 0.5) / 120.)
                };
                queue_link_world_velocity(&mut sim, pad_link, [0., 0., cmd], [0.; 3])?;
            }
            sim.step_exact(1)?;
        }
        let n = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let f = n.state(ids.foot).unwrap();
        let u = ids.upper.and_then(|id| n.state(id));
        let p = n.state(pad).unwrap();
        let key = (ids.foot.min(pad), ids.foot.max(pad));
        let receipt = last_refinement(&sim).filter(|e| e.iteration == tick);
        let impulse = if let Some(r) = receipt {
            starts.push(tick);
            r.interval
                .pairs
                .iter()
                .filter(|p| (p.a, p.b) == key)
                .map(|x| x.normal_impulse)
                .sum::<f64>()
        } else {
            n.rigid_contact_observations()
                .iter()
                .filter(|x| (x.a, x.b) == key)
                .map(|p| p.normal_impulse)
                .sum()
        };
        if first.is_none() && impulse > 0. {
            first = Some(tick);
        }
        if tick <= 45 {
            early.push(
                json!({"foot":wire(f),"upper":u.map(wire),"pad":wire(p),"positive":impulse>0.}),
            );
        }
        let applicable = tick > PARK && !c.rigid;
        let admitted = applicable.then(|| n.validate_articulation_demotion(ids.foot, dt));
        if let Some(Err(ref why)) = admitted {
            *blocked.entry(why.to_string()).or_default() += 1;
        }
        let allowed = admitted.is_some_and(|r| r.is_ok());
        let m = metric(n, &ids, (f, u, p), allowed);
        metrics.push(m);
        if c.edge
            && !c.rigid
            && !c.per_point
            && c.mu == 0.2
            && matches!(c.policy, "ordinary" | "relax8" | "refresh8")
            && (tick <= 35 || tick <= 1200 && tick % 10 == 0)
        {
            let normal = q_up(f);
            trace.push(json!({"tick":tick,"time_us":tick*500,"native_steps":receipt.map_or(u32::from(tick!=0),|r|r.interval.native_steps),"solver_iterations":4,
                "selected":receipt.is_some(),"weak_start":false,"secondary_start":false,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),
                "normal_impulse_ns":impulse,"gap_m":f.position[2]-p.position[2]-0.06-normal.abs().dot(DVec3::new(0.18,0.10,0.06))}));
        }
    }
    let first = first.ok_or("no impact")?;
    let cut = first
        .min(*starts.first().unwrap_or(&first))
        .checked_sub(1)
        .ok_or("missing incoming sample")?;
    let parked = &metrics[PARK as usize + 1..];
    let first_quiet = |physical: bool| {
        parked
            .windows(40)
            .position(|ws| {
                ws.iter().all(|m| {
                    if physical {
                        m.linear <= 0.001 && m.angular <= 0.001
                    } else {
                        m.accept
                    }
                })
            })
            .map(|i| PARK + i as u64 + 1)
    };
    let first_admitted = parked
        .iter()
        .position(|m| m.accept)
        .map(|i| PARK + i as u64 + 1);
    let mut samples = vec![];
    for age in [0_u64, 40, 120, 240, 480, 960, 1600, 2320] {
        let tick = PARK + age + 1;
        let m = metrics[tick as usize];
        let later = &metrics[tick as usize + 40];
        samples.push(json!({"tick":tick,"park_age_ms":age as f64*0.5,"sample":m.describe(),
            "observed_20ms_com_move_m":(later.com-m.com).length(),"observed_20ms_rotation_rad":quat_angle(m.q,later.q)}));
    }
    let peak = |f: fn(&Metric) -> f64| parked.iter().map(f).fold(0., f64::max);
    let result = json!({"placement":if c.edge{"edge"}else{"center"},"mu":c.mu,"rigid":c.rigid,"per_point":c.per_point,"policy":c.policy,
        "first_positive_tick":first,"first_branch_cut":cut,"first_branch_state":early[cut as usize],
        "refined_ticks":starts,"first_gate_tick":first_admitted,"first_20ms_gate_tick":first_quiet(false),"first_20ms_motion_tick":first_quiet(true),
        "gate_accepted_ticks":parked.iter().filter(|m|m.accept).count(),"gate_failure_reasons":blocked,
        "peak_parked_spin_rad_s":peak(|m|m.angular),"peak_parked_energy_j":peak(|m|m.energy),"peak_parked_normal_mismatch_rad":peak(|m|m.mismatch.unwrap_or(0.)),
        "samples":samples,"final":metrics.last().unwrap().describe()});
    Ok((result, trace))
}
fn q_up(b: BodyState) -> DVec3 {
    DQuat::from_array(b.orientation).conjugate() * DVec3::Z
}
fn quat_angle(a: DQuat, b: DQuat) -> f64 {
    let d = a.conjugate() * b;
    2. * DVec3::new(d.x, d.y, d.z).length().atan2(d.w.abs())
}
fn main() -> Result<()> {
    let out = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: local_primary_settling OUT")?,
    );
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--sliding-law")) {
        return microtilt::sliding_law(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--native-opening-edge")) {
        return microtilt::spatial_edge::opening::native::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--shared-sweep-boundary")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::sweep::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--belt-branches")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::sliding::capture::branches::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--belt-capture")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::sliding::capture::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--shared-tier-rearming")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::uniform::parked::rearming::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--parked-shared-tier")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::uniform::parked::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--comoving-arrest")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::uniform::arrest::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--uniform-shared-slide")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::uniform::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--sliding-support-capacity")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::sliding::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--spatial-two-support")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::spatial::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--native-two-support-load")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::run(&out,true);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--two-support-load")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::dual::run(&out,false);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--face-bearing")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::run(&out,false,false);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--native-face-bearing")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::run(&out,true,false);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--native-deck-edge-bearing")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::run(&out,true,true);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--deck-edge-bearing")) {
        return microtilt::spatial_edge::opening::rocking::face::bearing::run(&out,false,true);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--rocking-limit")) {
        return microtilt::spatial_edge::opening::rocking::face::train::run(&out,false,false);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--native-rocking-limit")) {
        return microtilt::spatial_edge::opening::rocking::face::train::run(&out,true,false);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--rocking-limit-cost")) {
        return microtilt::spatial_edge::opening::rocking::face::train::run(&out,true,true);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--sole-transfer-cost")) {
        return microtilt::spatial_edge::opening::rocking::face::transfer::run(&out,true,true);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--sole-transfer")) {
        return microtilt::spatial_edge::opening::rocking::face::transfer::run(&out,false,false);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--native-sole-transfer")) {
        return microtilt::spatial_edge::opening::rocking::face::transfer::run(&out,true,false);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--native-face-pressure")) {
        return microtilt::spatial_edge::opening::rocking::face::run_registered(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--face-pressure")) {
        return microtilt::spatial_edge::opening::rocking::face::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--native-two-bearing")) {
        return microtilt::spatial_edge::opening::rocking::two_bearing::registered::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--two-bearing")) {
        return microtilt::spatial_edge::opening::rocking::two_bearing::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--normal-recontact")) {
        return microtilt::spatial_edge::opening::rocking::transaction::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--rocking-recontact")) {
        return microtilt::spatial_edge::opening::rocking::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--opening-edge")) {
        return microtilt::spatial_edge::opening::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--authored-mixed-edge")) {
        return microtilt::spatial_edge::mixed::authored::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--native-mixed-edge")) {
        return microtilt::spatial_edge::mixed::native::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--mixed-edge")) {
        return microtilt::spatial_edge::mixed::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--authored-edge")) {
        return microtilt::spatial_edge::authored::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--native-edge")) {
        return microtilt::spatial_edge::transaction::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--spatial-edge")) {
        return microtilt::spatial_edge::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--microtilt-guard")) {
        return microtilt::guard::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--microtilt")) {
        return microtilt::run(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--angular")) {
        return angular::run_all(&out);
    }
    if std::env::args_os().nth(2).as_deref() == Some(std::ffi::OsStr::new("--departures")) {
        return departure::execute(&out);
    }
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    let geometry = json!({"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.33,0.18,0.06]});
    for edge in [false, true] {
        for rigid in [false, true] {
            for mu in [0., 0.2, 0.8] {
                for per_point in [false, true] {
                    for policy in POLICIES {
                        let (row, trace) = trial(Config {
                            edge,
                            rigid,
                            mu,
                            per_point,
                            policy,
                        })?;
                        rows.push(row);
                        if !trace.is_empty() {
                            fs::write(
                                out.join(format!("traces/{policy}.json")),
                                serde_json::to_vec(&json!({"geometry":geometry,"frames":trace}))?,
                            )?;
                        }
                    }
                }
            }
        }
    }
    let lanes = [
        ("ordinary", "Ordinary solve"),
        ("relax8", "8 relaxation passes"),
        ("refresh8", "8 passes + fresh geometry"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"Matched edge landing · braking and settling","pair_totals_only":true,"sampled_ticks":true,"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("study.json"), serde_json::to_vec(&rows)?)?;
    Ok(())
}
