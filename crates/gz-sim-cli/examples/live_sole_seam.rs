// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Distribute one rigid foot's load between two panels that open and close at a seam.
mod soft_replay;
#[path = "live_sole_seam/view.rs"]
mod view;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_sim_core::EntityId;
use gz_sim_scene::{
    Name, PhysicsEngine,
    damped_sole::{last_step, panel_steps},
    load_world_with_sdf_step, queue_link_world_velocity,
};
use serde::Serialize;
use serde_json::json;
use std::{error::Error, fs, path::PathBuf, time::Instant};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const DT: f64 = 0.0005;
const SETTLE: usize = 100;
const LOWER_END: usize = SETTLE + 40;
const RETURN: usize = LOWER_END + 30;
const RISE_END: usize = RETURN + 40;
const END: usize = RISE_END + 400;
const MASS: f64 = 2.;
const LEFT_K: f64 = 4e6;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    output: PathBuf,
    #[arg(long, default_value_t = 32)]
    substeps: u32,
    #[arg(long, default_value_t = 256)]
    fine_substeps: u32,
    /// Estimate separate per-panel errors and refine the coarse run.
    #[arg(long)]
    adaptive: bool,
    #[arg(long, default_value_t = 5)]
    repetitions: usize,
}
#[derive(Serialize)]
struct MaterialRun {
    name: &'static str,
    right_stiffness_n_m3: f64,
    samples: Vec<Sample>,
}
#[derive(Serialize, Default)]
struct Sample {
    tick: usize,
    time_s: f64,
    center_m: [f64; 3],
    orientation_xyzw: [f64; 4],
    velocity_m_s: [f64; 3],
    spin_rad_s: [f64; 3],
    panels: [Reaction; 2],
    support_margin_m: Option<f64>,
    selected_substeps: Option<u32>,
}
#[derive(Serialize, Default)]
struct Reaction {
    sensor_force_n: [f64; 3],
    sensor_moment_nm: [f64; 3],
    center_m: [f64; 3],
    force_on_foot_n: [f64; 3],
    impulse_on_foot_ns: [f64; 3],
    normal_impulse_ns: f64,
    support_work_j: f64,
    possible_missed_contacts: u32,
}
fn world(right_k: f64, substeps: u32, adaptive: bool) -> String {
    let policy = if adaptive {
        "<max_substeps>512</max_substeps><max_quadrature_depth>3</max_quadrature_depth>
         <position_tolerance>1e-7</position_tolerance><rotation_tolerance>1e-6</rotation_tolerance>
         <impulse_tolerance>2e-5</impulse_tolerance><angular_impulse_tolerance>2e-6</angular_impulse_tolerance>
         <heat_tolerance>1e-5</heat_tolerance><work_tolerance>1e-5</work_tolerance>"
    } else {
        ""
    };
    let panel = |name: &str, x: f64, static_panel: bool| {
        let motion = if static_panel {
            "<static>true</static>"
        } else {
            r#"<plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>0 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>"#
        };
        format!(
            r#"<model name="{name}"><pose>{x} 0 -0.03 0 0 0</pose>{motion}<link name="body"><collision name="c"><geometry><box><size>0.4 0.32 0.06</size></box></geometry></collision><visual name="v"><geometry><box><size>0.4 0.32 0.06</size></box></geometry></visual></link></model>"#
        )
    };
    // A 2 kg level foot straddles a seam 7 cm to the right of its center.
    // The equal-stiffness scene starts at its exact static compression.
    let preload = MASS * 9.81 / (0.36 * 0.20 * (1. / (1. / 2e8 + 1. / LEFT_K)));
    let z = 0.06 - preload;
    format!(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity>
 <physics name="fixed" type="ignored"><max_step_size>0.0005</max_step_size></physics>
 <plugin name="gz::sim::systems::DampedSole" filename="gz-sim-damped-sole-system">
 <foot>default::foot::body</foot><support>default::left::body</support><support deck_stiffness="{right_k}">default::right::body</support>
 <foot_stiffness>2e8</foot_stiffness><deck_stiffness>{LEFT_K}</deck_stiffness><damping>30</damping>
 <friction>0.4</friction><slip_speed>0.005</slip_speed><substeps>{substeps}</substeps><quadrature_depth>0</quadrature_depth>{policy}</plugin>
 <model name="foot"><pose>0 0 {z} 0 0 0</pose><link name="body">
 <inertial><mass>2</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz></inertia></inertial>
 <collision name="c"><geometry><box><size>0.36 0.2 0.12</size></box></geometry></collision>
 <visual name="v"><geometry><box><size>0.36 0.2 0.12</size></box></geometry></visual></link></model>
 {}{}</world></sdf>"#,
        panel("left", -0.13, true),
        panel("right", 0.27, false)
    )
}
fn run(xml: &str) -> Result<(Vec<Sample>, Vec<u64>)> {
    let mut sim = load_world_with_sdf_step(
        xml,
        "seam.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    #[allow(deprecated)]
    let id = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|s| s.0 == name))
            .and_then(|e| e.get::<EntityId>().map(EntityId::get))
            .ok_or("missing panel")
    };
    let left = id("default::left::body")?;
    let right = id("default::right::body")?;
    let mut samples = Vec::with_capacity(END);
    let mut timings = Vec::with_capacity(END);
    for tick in 1..=END {
        let speed = match tick - 1 {
            SETTLE => Some(-0.3),
            LOWER_END | RISE_END => Some(0.),
            RETURN => Some(0.3),
            _ => None,
        };
        if let Some(speed) = speed {
            queue_link_world_velocity(&mut sim, right, [0., 0., speed], [0.; 3])?;
        }
        let before = Instant::now();
        if let Err(e) = sim.step_exact(1) {
            return Err(format!(
                "public tick {tick}: {} (preceding committed sample {})",
                sim.fault()
                    .map_or_else(|| e.to_string(), |fault| fault.to_string()),
                serde_json::to_string(&samples.last())?
            )
            .into());
        }
        timings.push(before.elapsed().as_nanos() as u64);
        let step = last_step(&sim).ok_or("no live foot reaction")?;
        let s = step.state;
        let rows = panel_steps(&sim);
        if rows.len() != 2 {
            return Err("missing panel reaction".into());
        }
        let mut panels = [Reaction::default(), Reaction::default()];
        let reactions: Vec<_> = rows
            .iter()
            .map(|r| gz_physics::sole_supports::PanelReaction {
                id: r.support,
                on_sole: r.step,
                support_work_j: r.support_work_j,
            })
            .collect();
        let footprint = gz_physics::sole_balance::SupportFootprint::from_panels(
            &reactions,
            DVec3::ZERO,
            DQuat::IDENTITY,
            0.01,
        );
        let support_margin_m = footprint
            .as_ref()
            .and_then(|fp| fp.margin_m(s.center.truncate()));
        for (p, id) in panels.iter_mut().zip([left, right]) {
            let row = rows
                .iter()
                .find(|r| r.support == id)
                .ok_or("missing stable panel ID")?;
            let reaction = reactions
                .iter()
                .find(|r| r.id == id)
                .ok_or("missing panel wrench")?;
            let sensor = gz_physics::sole_balance::SupportWrench::at_sensor(
                reaction,
                s.center,
                row.deck.center,
                DQuat::IDENTITY,
            )
            .ok_or("invalid panel wrench")?;
            *p = Reaction {
                sensor_force_n: sensor.force_n.to_array(),
                sensor_moment_nm: sensor.moment_nm.to_array(),
                center_m: row.deck.center.to_array(),
                force_on_foot_n: row.step.contact.force.to_array(),
                impulse_on_foot_ns: row.step.impulse.to_array(),
                normal_impulse_ns: row.step.normal_impulse,
                support_work_j: row.support_work_j,
                possible_missed_contacts: row.step.possible_missed_contacts,
            };
        }
        samples.push(Sample {
            tick,
            time_s: sim.elapsed().as_secs_f64(),
            center_m: s.center.to_array(),
            orientation_xyzw: s.orientation.to_array(),
            velocity_m_s: s.velocity.to_array(),
            spin_rad_s: s.spin.to_array(),
            selected_substeps: step.panel_accuracy.map(|p| p.substeps),
            support_margin_m,
            panels,
        });
    }
    Ok((samples, timings))
}

fn compare(coarse: &[Sample], fine: &[Sample]) -> Result<serde_json::Value> {
    let mut panel_impulse = [0_f64; 2];
    let mut panel_force = [0_f64; 2];
    let (mut com, mut rot, mut momentum) = (0_f64, 0_f64, 0_f64);
    for (c, f) in coarse.iter().zip(fine) {
        if c.tick != f.tick || c.time_s != f.time_s {
            return Err("mismatched seam clock".into());
        }
        com = com.max(DVec3::from(c.center_m).distance(f.center_m.into()));
        rot = rot.max(
            (DQuat::from_array(c.orientation_xyzw)
                * DQuat::from_array(f.orientation_xyzw).inverse())
            .to_scaled_axis()
            .length(),
        );
        for i in 0..2 {
            panel_force[i] = panel_force[i].max(
                DVec3::from(c.panels[i].force_on_foot_n)
                    .distance(f.panels[i].force_on_foot_n.into()),
            );
            panel_impulse[i] = panel_impulse[i].max(
                DVec3::from(c.panels[i].impulse_on_foot_ns)
                    .distance(f.panels[i].impulse_on_foot_ns.into()),
            );
        }
        let prev = if c.tick == 1 {
            DVec3::ZERO
        } else {
            coarse[c.tick - 2].velocity_m_s.into()
        };
        let predicted = MASS * (DVec3::from(c.velocity_m_s) - prev + DT * 9.81 * DVec3::Z);
        let total = c
            .panels
            .iter()
            .map(|p| DVec3::from(p.impulse_on_foot_ns))
            .sum();
        momentum = momentum.max(predicted.distance(total));
    }
    let absent = &coarse[LOWER_END..RETURN];
    if absent.iter().any(|s| {
        s.panels[1].force_on_foot_n != [0.; 3]
            || s.panels[1].impulse_on_foot_ns != [0.; 3]
            || s.panels[0].normal_impulse_ns <= 0.
    }) {
        return Err("stale moving-panel force or lost standing support".into());
    }
    let both = |s: &Sample| s.panels.iter().all(|p| p.normal_impulse_ns > 0.);
    if !both(&coarse[SETTLE - 1]) || !both(coarse.last().ok_or("missing endpoint")?) {
        return Err(format!("both panels must carry load: seated {:?}, endpoint {:?}, right height {:.8}, foot {:?}",coarse[SETTLE-1].panels.each_ref().map(|p|p.normal_impulse_ns),coarse[END-1].panels.each_ref().map(|p|p.normal_impulse_ns),coarse[END-1].panels[1].center_m[2],coarse[END-1].center_m).into());
    }
    fn returned(rows: &[Sample]) -> Result<&Sample> {
        rows[RETURN..]
            .iter()
            .find(|s| s.panels[1].normal_impulse_ns > 0.)
            .ok_or_else(|| "no reacquisition".into())
    }
    let min_margin = absent
        .iter()
        .map(|s| s.support_margin_m.ok_or("missing loaded footprint"))
        .collect::<std::result::Result<Vec<_>, _>>()?
        .into_iter()
        .fold(f64::INFINITY, f64::min);
    let signed_error = coarse
        .iter()
        .flat_map(|s| &s.panels)
        .map(|p| (DVec3::from(p.sensor_force_n) + DVec3::from(p.force_on_foot_n)).length())
        .fold(0., f64::max);
    let lift_work: f64 = coarse[RETURN..RISE_END]
        .iter()
        .map(|s| s.panels[1].support_work_j)
        .sum();
    let stationary_work = coarse
        .iter()
        .map(|s| s.panels[0].support_work_j.abs())
        .fold(0., f64::max);
    if min_margin <= 0. || signed_error > 1e-12 || lift_work <= 0. || stationary_work != 0. {
        return Err("invalid signed plate, support margin or work".into());
    }
    let c = returned(coarse)?;
    let f = returned(fine)?;
    if c.tick != f.tick || com > 1e-5 || momentum > 1e-9 || panel_impulse.iter().any(|i| *i > 0.001)
    {
        return Err(format!("seam convergence failed: return ticks {}/{}, position {com}, panel impulses {panel_impulse:?}, momentum {momentum}",c.tick,f.tick).into());
    }
    Ok(
        json!({"minimum_open_balance_margin_m":min_margin,"max_signed_force_error_n":signed_error,
        "moving_panel_rise_work_j":lift_work,"fixed_panel_max_work_j":stationary_work,
        "min_selected_substeps":coarse.iter().filter_map(|s|s.selected_substeps).min(),"max_selected_substeps":coarse.iter().filter_map(|s|s.selected_substeps).max(),
        "matched_clocks":coarse.len(),"open_seam_ticks_without_stale_impulses":absent.len(),"return_tick":c.tick,"fine_return_tick":f.tick,
    "return_force_on_foot_n":c.panels[1].force_on_foot_n,"return_impulse_on_foot_ns":c.panels[1].impulse_on_foot_ns,
    "fine_return_impulse_on_foot_ns":f.panels[1].impulse_on_foot_ns,
    "seated_normal_reactions_n":coarse[SETTLE-1].panels.each_ref().map(|p|p.force_on_foot_n[2]),
    "endpoint_normal_reactions_n":coarse.last().unwrap().panels.each_ref().map(|p|p.force_on_foot_n[2]),
    "max_per_panel_impulse_error_ns":panel_impulse,"max_per_panel_force_error_n":panel_force,
    "max_com_error_m":com,"max_rotation_error_rad":rot,"max_momentum_budget_error_ns":momentum}),
    )
}
fn cost(xml: &str, repetitions: usize) -> Result<f64> {
    let mut costs = Vec::with_capacity(END * repetitions);
    for _ in 0..repetitions {
        costs.extend(run(xml)?.1);
    }
    costs.sort_unstable();
    Ok(costs[(95 * costs.len()).div_ceil(100) - 1] as f64 / 1000.)
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.repetitions == 0 || args.substeps == 0 || args.substeps >= args.fine_substeps {
        return Err("choose positive repetitions and a finer comparison grid".into());
    }
    if args.output.exists() {
        return Err("choose a new output directory".into());
    }
    let parent = args
        .output
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(std::path::Path::new("."));
    fs::create_dir_all(parent)?;
    let dir = tempfile::Builder::new()
        .prefix(".seam-")
        .tempdir_in(parent)?;
    let mut histories = Vec::new();
    let mut checks = Vec::new();
    for (name, right_k) in [
        ("Equal panels", LEFT_K),
        ("Stiffer moving panel", 4. * LEFT_K),
    ] {
        let xml = world(right_k, args.substeps, args.adaptive);
        let fine_xml = world(right_k, args.fine_substeps, false);
        let (coarse, _) = run(&xml)?;
        let (fine, _) = run(&fine_xml)?;
        let mut report = compare(&coarse, &fine)?;
        report["material"] = json!(name);
        report["p95_step_us"] = json!(cost(&xml, args.repetitions)?);
        report["fine_p95_step_us"] = json!(cost(&fine_xml, args.repetitions)?);
        fs::write(
            dir.path().join(format!("material-{}.sdf", histories.len())),
            xml,
        )?;
        histories.push(MaterialRun {
            name,
            right_stiffness_n_m3: right_k,
            samples: coarse,
        });
        checks.push(report);
    }
    fs::write(
        dir.path().join("checks.json"),
        serde_json::to_vec_pretty(
            &json!({"substeps":args.substeps,"fine_substeps":args.fine_substeps,
    "adaptive":args.adaptive,"repetitions":args.repetitions,"public_step_s":DT,"results":checks}),
        )?,
    )?;
    fs::write(
        dir.path().join("reaction.json"),
        serde_json::to_vec(&histories)?,
    )?;
    view::write(dir.path(), &histories)?;
    fs::rename(dir.path(), args.output)?;
    Ok(())
}
