// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a native fixed joint with a single equal-inertia body using the real SDF visuals.
#[path = "live_welded_foot/branch.rs"]
mod branch;
#[path = "live_welded_foot/branch_view.rs"]
mod branch_view;
mod soft_replay;
#[path = "live_welded_foot/view.rs"]
mod view;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, VisualGeometry, WorldPose, load_world_with_sdf_step,
    queue_link_world_wrench,
};
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const DT: f64 = 0.0005;
const END: usize = 360;
const KICK_START: usize = 40;
const KICK_END: usize = 60;
const FOOT: [f64; 3] = [0.36, 0.2, 0.12];
const UPPER: [f64; 3] = [0.10, 0.12, 0.12];
const SEP: f64 = 0.14;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    output: PathBuf,
    /// Restart both rigid representations from the incoming boundary saved in this earlier campaign.
    #[arg(long)]
    branch_from: Option<PathBuf>,
    /// Qualify a branch against pre-change cold replays captured with --branch-from alone.
    #[arg(long, requires = "branch_from")]
    compare_before: Option<PathBuf>,
}
#[derive(Clone, Deserialize, Serialize)]
struct Visual {
    p: [f64; 3],
    q: [f64; 4],
    size: [f64; 3],
}
#[derive(Clone, Deserialize, Serialize)]
struct Sample {
    tick: usize,
    com: [f64; 3],
    p_kg_m_s: [f64; 3],
    angular_about_com_nms: [f64; 3],
    foot: Visual,
    upper: Visual,
    normal_impulse_ns: f64,
    upper_normal_impulse_ns: f64,
    external_angular_impulse_nms: [f64; 3],
    external_linear_impulse_ns: [f64; 3],
}
#[derive(Clone, Deserialize, Serialize)]
struct Trial {
    name: String,
    ground: bool,
    welded: bool,
    frames: Vec<Sample>,
}
fn density_box(size: [f64; 3]) -> DVec3 {
    let [x, y, z] = size;
    DVec3::new(y * y + z * z, x * x + z * z, x * x + y * y) / 12.
}
fn inertia_total() -> DVec3 {
    density_box(FOOT) + density_box(UPPER) + (SEP * SEP / 2.) * DVec3::new(1., 1., 0.)
}
fn make_link(
    name: &str,
    size: [f64; 3],
    height: f64,
    inertial_offset: f64,
    mass: f64,
    inertia: DVec3,
    upper_visual: bool,
) -> String {
    let size = format!("{} {} {}", size[0], size[1], size[2]);
    let second = if upper_visual {
        r#"<visual name="upper"><pose>0 0 0.14 0 0 0</pose><geometry><box><size>0.1 0.12 0.12</size></box></geometry></visual>"#
    } else {
        ""
    };
    format!(
        r#"<link name="{name}"><pose>0 0 {height} 0 0 0</pose><inertial><pose>0 0 {inertial_offset} 0 0 0</pose><mass>{mass}</mass><inertia><ixx>{}</ixx><iyy>{}</iyy><izz>{}</izz></inertia></inertial>
    <collision name="shape"><geometry><box><size>{size}</size></box></geometry></collision><visual name="{name}"><geometry><box><size>{size}</size></box></geometry></visual>{second}</link>"#,
        inertia.x, inertia.y, inertia.z
    )
}
fn scene(welded: bool, ground: bool, factor: u32) -> String {
    scene_at(welded, ground, factor, None)
}
fn scene_at(welded: bool, ground: bool, factor: u32, restart: Option<&Sample>) -> String {
    let dt = DT / f64::from(factor);
    let body = if welded {
        format!(
            "{}{}{}",
            make_link("foot", FOOT, 0., 0., 1., density_box(FOOT), false),
            make_link("upper", UPPER, SEP, 0., 1., density_box(UPPER), false),
            r#"<joint name="weld" type="fixed"><parent>foot</parent><child>upper</child></joint>"#
        )
    } else {
        make_link("foot", FOOT, 0., SEP / 2., 2., inertia_total(), true)
    };
    let (height, launch, angular) = if ground {
        (0.095, "0 0 0", "0 0 0")
    } else {
        (0.60, "0.07 -0.03 -0.2", "0 1 0")
    };
    let (model_pose, linear) = if let Some(s) = restart {
        let [x, y, z] = s.foot.p;
        let [u, v, w] = s.p_kg_m_s.map(|p| p / 2.);
        (format!("{x} {y} {z} 0 0 0"), format!("{u} {v} {w}"))
    } else {
        (format!("0 0 {height} 0 0 0"), launch.to_owned())
    };
    let floor = if ground {
        r#"<model name="floor"><static>true</static><pose>0 0 -0.025 0 0 0</pose><link name="base">
       <collision name="shape"><geometry><box><size>4 4 0.05</size></box></geometry></collision></link></model>"#
    } else {
        ""
    };
    format!(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity><physics name="clock" type="ignored"><max_step_size>{dt}</max_step_size></physics>
      <model name="rig"><pose>{model_pose}</pose><plugin name="gz::sim::systems::InitialVelocity" filename="gz-sim-initial-velocity-system"><linear_velocity>{linear}</linear_velocity><angular_velocity>{angular}</angular_velocity></plugin>{body}</model>{floor}</world></sdf>"#
    )
}
#[allow(deprecated)]
fn id(sim: &Simulation, name: &str) -> Result<u64> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .and_then(|e| e.get::<EntityId>().map(EntityId::get))
        .ok_or_else(|| format!("missing {name}").into())
}
fn native_id(sim: &Simulation, id: u64) -> Result<gz_physics::BodyId> {
    let e = entity_by_id(sim.world(), id).ok_or("missing entity")?;
    sim.world()
        .get::<BodyHandle>(e)
        .map(BodyHandle::id)
        .ok_or_else(|| "missing native body ID".into())
}
fn read(sim: &Simulation, id: gz_physics::BodyId) -> Result<BodyState> {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(id)
        .ok_or_else(|| "missing native link".into())
}
fn visual(sim: &Simulation, id: u64) -> Result<Visual> {
    let e = entity_by_id(sim.world(), id).ok_or("missing visual")?;
    let p = sim
        .world()
        .get::<WorldPose>(e)
        .ok_or("missing published visual pose")?;
    let sdformat::Geometry::Box { size } = sim
        .world()
        .get::<VisualGeometry>(e)
        .ok_or("missing SDF shape")?
        .0
        .as_ref()
    else {
        return Err("expected box visual".into());
    };
    Ok(Visual {
        p: p.translation,
        q: p.rotation,
        size: *size,
    })
}
fn local_momentum(b: BodyState, inertia: DVec3) -> DVec3 {
    let q = DQuat::from_array(b.orientation);
    q * (inertia * (q.conjugate() * DVec3::from(b.angular_velocity)))
}
fn run_from(welded: bool, ground: bool, xml: &str, factor: u32, start: usize) -> Result<Trial> {
    let mut sim = load_world_with_sdf_step(
        xml,
        "welded-foot.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let lower = id(&sim, "default::rig::foot")?;
    let foot = native_id(&sim, lower)?;
    let upper = if welded {
        Some(id(&sim, "default::rig::upper")?)
    } else {
        None
    };
    let top = upper.map(|id| native_id(&sim, id)).transpose()?;
    let floor = if ground {
        Some(native_id(&sim, id(&sim, "default::floor::base")?)?)
    } else {
        None
    };
    let fv = id(&sim, "default::rig::foot::foot")?;
    let uv = id(
        &sim,
        if welded {
            "default::rig::upper::upper"
        } else {
            "default::rig::foot::upper"
        },
    )?;
    let mut frames = Vec::with_capacity(END + 1);
    let (mut total_force, mut total_torque) = (DVec3::ZERO, DVec3::ZERO);
    for tick in start..=END {
        let (mut normal, mut upper_normal) = (0., 0.);
        for _ in 0..if tick > start { factor } else { 0 } {
            if !ground && (KICK_START..KICK_END).contains(&tick) {
                // The material-point force acts at the upper link's COM. Transfer the same
                // wrench to the consolidated body's COM using the current orientation.
                let f = read(&sim, foot)?;
                let offset = DQuat::from_array(f.orientation) * DVec3::Z * (SEP / 2.);
                let force = DVec3::new(12., 0., -1.);
                let moment = offset.cross(force);
                queue_link_world_wrench(
                    &mut sim,
                    upper.unwrap_or(lower),
                    force.into(),
                    if welded { [0.; 3] } else { moment.into() },
                )?;
                total_force += (DT / f64::from(factor)) * force;
                total_torque += (DT / f64::from(factor)) * moment;
            }
            if let Err(e) = sim.step_exact(1) {
                return Err(format!(
                    "welded={welded} ground={ground} tick={tick}: {}",
                    sim.fault().map_or_else(|| e.to_string(), |f| f.to_string())
                )
                .into());
            }
            if let Some(floor) = floor {
                let native = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .ok_or("wrong backend")?;
                for row in native
                    .rigid_contact_observations()
                    .into_iter()
                    .filter(|r| r.active)
                {
                    let other = if row.a == floor {
                        row.b
                    } else if row.b == floor {
                        row.a
                    } else {
                        continue;
                    };
                    if other == foot {
                        normal += row.normal_impulse;
                    } else if Some(other) == top {
                        upper_normal += row.normal_impulse;
                    }
                }
            }
        }
        let b = read(&sim, foot)?;
        let q = DQuat::from_array(b.orientation);
        let (com, p, h) = if let Some(id) = top {
            let u = read(&sim, id)?;
            let com = (DVec3::from(b.position) + DVec3::from(u.position)) / 2.;
            let v = DVec3::from(b.velocity);
            let w = DVec3::from(u.velocity);
            (
                com,
                v + w,
                local_momentum(b, density_box(FOOT))
                    + local_momentum(u, density_box(UPPER))
                    + (DVec3::from(b.position) - com).cross(v)
                    + (DVec3::from(u.position) - com).cross(w),
            )
        } else {
            (
                DVec3::from(b.position) + q * DVec3::Z * SEP / 2.,
                DVec3::from(b.velocity) * 2.,
                local_momentum(b, inertia_total()),
            )
        };
        frames.push(Sample {
            tick,
            com: com.into(),
            p_kg_m_s: p.into(),
            angular_about_com_nms: h.into(),
            foot: visual(&sim, fv)?,
            upper: visual(&sim, uv)?,
            normal_impulse_ns: normal,
            upper_normal_impulse_ns: upper_normal,
            external_angular_impulse_nms: total_torque.into(),
            external_linear_impulse_ns: total_force.into(),
        });
    }
    Ok(Trial {
        name: format!(
            "{} {}",
            if ground { "Ground" } else { "Flight" },
            if welded { "fixed joint" } else { "single body" }
        ),
        welded,
        ground,
        frames,
    })
}
fn run(welded: bool, ground: bool, xml: &str, factor: u32) -> Result<Trial> {
    run_from(welded, ground, xml, factor, 0)
}
fn delta(a: [f64; 3], b: [f64; 3]) -> f64 {
    DVec3::from(a).distance(b.into())
}
fn orientation(a: [f64; 4], b: [f64; 4]) -> f64 {
    (DQuat::from_array(a) * DQuat::from_array(b).inverse())
        .to_scaled_axis()
        .length()
}
// Recover the complete contact impulse from the change in total body momentum.
// Gravity contributes to linear momentum and has no moment about the aggregate COM.
fn momentum_jump(prev: &Sample, next: &Sample) -> (DVec3, DVec3) {
    let elapsed = DT * (next.tick - prev.tick) as f64;
    (
        DVec3::from(next.p_kg_m_s) - DVec3::from(prev.p_kg_m_s) + 2. * 9.81 * elapsed * DVec3::Z,
        DVec3::from(next.angular_about_com_nms) - DVec3::from(prev.angular_about_com_nms),
    )
}
fn ground_boundary(w: &Trial, s: &Trial) -> Result<serde_json::Value> {
    let threshold = 1e-7;
    let Some(tick) = (1..w.frames.len()).find(|&n| {
        let (a, b) = (&w.frames[n], &s.frames[n]);
        if a.normal_impulse_ns == 0. && b.normal_impulse_ns == 0. {
            return false;
        }
        let (jp, hp) = momentum_jump(&w.frames[n - 1], a);
        let (jq, hq) = momentum_jump(&s.frames[n - 1], b);
        jp.distance(jq) > threshold || hp.distance(hq) > threshold
    }) else {
        return Ok(serde_json::Value::Null);
    };
    let capture = |history: &Trial| {
        let (a, b) = (&history.frames[tick - 1], &history.frames[tick]);
        let (linear, angular) = momentum_jump(a, b);
        json!({"incoming":a,"after_solve":b,"contact_linear_impulse_world_ns":linear.to_array(),
            "contact_angular_impulse_about_com_world_nms":angular.to_array()})
    };
    let a = &w.frames[tick - 1];
    let b = &s.frames[tick - 1];
    Ok(
        json!({"incoming_tick":tick-1,"first_divergent_contact_tick":tick,
       "incoming_time_s":(tick-1)as f64*DT,"first_divergent_contact_time_s":tick as f64*DT,
       "linear_threshold_ns":threshold,"angular_threshold_nms":threshold,
       "incoming_com_position_difference_m":delta(a.com,b.com),
       "incoming_linear_momentum_difference_ns":delta(a.p_kg_m_s,b.p_kg_m_s),
       "incoming_angular_momentum_difference_nms":delta(a.angular_about_com_nms,b.angular_about_com_nms),
       "incoming_foot_pose_difference_m":delta(a.foot.p,b.foot.p),"incoming_upper_pose_difference_m":delta(a.upper.p,b.upper.p),
       "welded":capture(w),"consolidated":capture(s)}),
    )
}
fn checks(w: &Trial, s: &Trial) -> Result<serde_json::Value> {
    let (
        mut foot,
        mut upper,
        mut com,
        mut p,
        mut h,
        mut rotation,
        mut weld_length,
        mut weld_rotation,
        mut balance_h,
        mut balance_p,
    ) = (
        0_f64, 0_f64, 0_f64, 0_f64, 0_f64, 0_f64, 0_f64, 0_f64, 0_f64, 0_f64,
    );
    let mut j = [0.; 2];
    let mut first_impulses = [0.; 2];
    let mut first = [None; 2];
    let mut max_upper = 0_f64;
    for (a, b) in w.frames.iter().zip(&s.frames) {
        if a.tick != b.tick {
            return Err("welded clocks differ".into());
        }
        foot = foot.max(delta(a.foot.p, b.foot.p));
        upper = upper.max(delta(a.upper.p, b.upper.p));
        com = com.max(delta(a.com, b.com));
        p = p.max(delta(a.p_kg_m_s, b.p_kg_m_s));
        h = h.max(delta(a.angular_about_com_nms, b.angular_about_com_nms));
        rotation = rotation
            .max(orientation(a.foot.q, b.foot.q))
            .max(orientation(a.upper.q, b.upper.q));
        let offset = DQuat::from_array(a.foot.q).conjugate()
            * (DVec3::from(a.upper.p) - DVec3::from(a.foot.p));
        weld_length = weld_length.max(offset.distance(SEP * DVec3::Z));
        weld_rotation = weld_rotation.max(orientation(a.foot.q, a.upper.q));
        for (i, f) in [a, b].into_iter().enumerate() {
            if f.normal_impulse_ns > 0. && first[i].is_none() {
                first[i] = Some(f.tick);
                first_impulses[i] = f.normal_impulse_ns;
            }
            j[i] += f.normal_impulse_ns;
            max_upper = max_upper.max(f.upper_normal_impulse_ns);
            let initial = if i == 0 { &w.frames[0] } else { &s.frames[0] };
            if !w.ground {
                let dp = DVec3::from(initial.p_kg_m_s) + DVec3::from(f.external_linear_impulse_ns)
                    - 2. * 9.81 * (DT * f.tick as f64) * DVec3::Z;
                let dh = DVec3::from(initial.angular_about_com_nms)
                    + DVec3::from(f.external_angular_impulse_nms);
                balance_p = balance_p.max(dp.distance(f.p_kg_m_s.into()));
                balance_h = balance_h.max(dh.distance(f.angular_about_com_nms.into()));
            }
        }
    }
    if weld_length > 1e-8 || weld_rotation > 1e-8 {
        return Err("native fixed joint failed to preserve rigid geometry".into());
    }
    if !w.ground
        && (com > 5e-6
            || foot > 5e-6
            || upper > 5e-6
            || rotation > 1e-5
            || balance_h > 1e-6
            || balance_p > 1e-5)
    {
        return Err("fixed-joint free-flight comparison exceeded the fixture tolerances".into());
    }
    if w.ground && (first.iter().any(Option::is_none) || max_upper != 0.) {
        return Err(format!(
            "missing bottom-only rigid ground reaction: first {first:?} upper {max_upper}"
        )
        .into());
    }
    let contact_boundary = w.ground.then(|| ground_boundary(w, s)).transpose()?;
    Ok(
        json!({"first_divergent_contact":contact_boundary,"mode":if w.ground{"native rigid ground"}else{"flight plus upper-link shove"},"matched_clocks":w.frames.len(),
       "max_foot_position_error_m":foot,"max_upper_position_error_m":upper,"max_com_error_m":com,"max_visual_rotation_error_rad":rotation,"max_linear_momentum_error_ns":p,
       "max_angular_momentum_error_nms":h,"max_fixed_relative_translation_error_m":weld_length,"max_fixed_relative_rotation_error_rad":weld_rotation,
       "free_flight_linear_budget_error_ns":(!w.ground).then_some(balance_p),"free_flight_angular_budget_error_nms":(!w.ground).then_some(balance_h),
       "ground_first_impulse_ticks":first,"first_loaded_window_normal_impulses_ns":first_impulses,"ground_total_normal_impulses_ns":j,"max_upper_ground_impulse_ns":max_upper}),
    )
}
fn main() -> Result<()> {
    let args = Args::parse();
    if args.output.exists() {
        return Err("choose new output directory".into());
    }
    let parent = args
        .output
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::Builder::new()
        .prefix(".weld-")
        .tempdir_in(parent)?;
    if let Some(prior) = args.branch_from {
        let passed = branch::campaign(&prior, args.compare_before.as_deref(), stage.path())?;
        fs::rename(stage.path(), args.output)?;
        if !passed {
            return Err("welded impact qualification failed; see qualification.json".into());
        }
        return Ok(());
    }
    let mut trials = vec![];
    let mut reports = vec![];
    for ground in [false, true] {
        for welded in [true, false] {
            let xml = scene(welded, ground, 1);
            fs::write(
                stage.path().join(format!(
                    "{}-{}.sdf",
                    if ground { "ground" } else { "flight" },
                    if welded { "welded" } else { "single" }
                )),
                &xml,
            )?;
            trials.push(run(welded, ground, &xml, 1)?);
        }
        let n = trials.len();
        reports.push(checks(&trials[n - 2], &trials[n - 1])?);
    }
    let mut convergence = vec![];
    for factor in [1, 2, 4, 8] {
        let mut metrics = if factor == 1 {
            reports[1].clone()
        } else {
            let a = run(true, true, &scene(true, true, factor), factor)?;
            let b = run(false, true, &scene(false, true, factor), factor)?;
            checks(&a, &b)?
        };
        metrics["native_step_us"] = json!(500. / f64::from(factor));
        convergence.push(metrics);
    }
    fs::write(
        stage.path().join("ground_convergence.json"),
        serde_json::to_vec_pretty(&convergence)?,
    )?;
    fs::write(
        stage.path().join("checks.json"),
        serde_json::to_vec_pretty(&reports)?,
    )?;
    fs::write(
        stage.path().join("links.json"),
        serde_json::to_vec(&trials)?,
    )?;
    view::write(stage.path(), &trials)?;
    fs::rename(stage.path(), args.output)?;
    Ok(())
}
