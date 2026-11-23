// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare fixed and adaptive contact on a driven turntable, including a queued stop.
mod soft_replay;
#[path = "live_sole_turntable/view.rs"]
mod view;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_sim_core::EntityId;
use gz_sim_scene::{
    Name, PhysicsEngine,
    damped_sole::{last_step, panel_steps},
    load_world_with_sdf_step, queue_link_world_velocity, queue_link_world_wrench,
};
use serde::Serialize;
use serde_json::json;
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const DT: f64 = 0.0005;
const STOP: usize = 150;
const END: usize = 230;
const RATE: f64 = 4.;
const INERTIA: f64 = 0.05;
const LOAD: f64 = 0.25;
const PULSE_BEGIN: usize = 70;
const PULSE_END: usize = 90;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    output: PathBuf,
    #[arg(long, default_value_t = 3)]
    repetitions: usize,
}
#[derive(Copy, Clone)]
struct Grid {
    name: &'static str,
    n: u32,
    d: u8,
    adaptive: bool,
}
#[derive(Serialize)]
struct Run {
    name: &'static str,
    samples: Vec<Sample>,
}
#[derive(Serialize)]
struct Sample {
    tick: usize,
    center_m: [f64; 3],
    orientation_xyzw: [f64; 4],
    table_orientation_xyzw: [f64; 4],
    spin_rad_s: [f64; 3],
    table_rate_rad_s: f64,
    force_n: [f64; 3],
    contact_torque_nm: [f64; 3],
    external_torque_nm: f64,
    cumulative_angular_impulse_nms: [f64; 3],
    cumulative_spin_work_j: f64,
    spin_work_in_step_j: f64,
    cumulative_sliding_work_j: f64,
    selected_substeps: Option<u32>,
    angular_momentum_error_nms: f64,
}
fn scene(grid: Grid) -> String {
    let z = 0.06 - 2. * 9.81 / (0.24 * 0.12 * 2e6);
    let policy = if grid.adaptive {
        r#"<max_substeps>256</max_substeps><max_quadrature_depth>4</max_quadrature_depth>
      <position_tolerance>1e-7</position_tolerance><rotation_tolerance>1e-6</rotation_tolerance>
      <impulse_tolerance>0.00005</impulse_tolerance><angular_impulse_tolerance>0.00001</angular_impulse_tolerance>
      <heat_tolerance>0.00001</heat_tolerance><work_tolerance>0.00002</work_tolerance>"#
    } else {
        ""
    };
    format!(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity><physics name="clock" type="ignored"><max_step_size>0.0005</max_step_size></physics>
 <plugin name="gz::sim::systems::DampedSole" filename="gz-sim-damped-sole-system"><foot>default::foot::body</foot><support>default::table::body</support>
 <foot_stiffness>4e6</foot_stiffness><deck_stiffness>4e6</deck_stiffness><damping>20</damping><friction>0.4</friction><slip_speed>0.01</slip_speed>
 <substeps>{}</substeps><quadrature_depth>{}</quadrature_depth>{policy}</plugin>
 <model name="foot"><pose>0.02 0 {z} 0 0 0</pose><link name="body"><inertial><mass>2</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz></inertia></inertial>
 <collision name="c"><geometry><box><size>0.24 0.12 0.12</size></box></geometry></collision></link></model>
 <model name="table"><pose>0 0 -0.025 0 0 0</pose><plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>0 0 0</linear_velocity><angular_velocity>0 0 4</angular_velocity></plugin>
 <link name="body"><collision name="c"><geometry><box><size>0.58 0.42 0.05</size></box></geometry></collision></link></model></world></sdf>"#,
        grid.n, grid.d
    )
}
fn run(grid: Grid, xml: &str) -> Result<(Run, Vec<u64>)> {
    let mut sim = load_world_with_sdf_step(
        xml,
        "turntable.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    #[allow(deprecated)]
    let id = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .and_then(|e| e.get::<EntityId>().map(EntityId::get))
            .ok_or("missing body")
    };
    let foot = id("default::foot::body")?;
    let table = id("default::table::body")?;
    let mut samples = Vec::with_capacity(END);
    let mut costs = Vec::with_capacity(END);
    let (mut angular, mut support_work, mut sliding, mut torque_j) = (DVec3::ZERO, 0., 0., 0.);
    for tick in 1..=END {
        if tick == STOP + 1 {
            let before = last_step(&sim).unwrap().deck.orientation;
            sim.set_paused(true);
            queue_link_world_velocity(&mut sim, table, [0.; 3], [0.; 3])?;
            sim.tick()?;
            if last_step(&sim).unwrap().deck.orientation != before {
                return Err("queued stop changed the paused table".into());
            }
            sim.set_paused(false);
        }
        let torque = if (PULSE_BEGIN..PULSE_END).contains(&tick) {
            LOAD
        } else {
            0.
        };
        if torque != 0. {
            queue_link_world_wrench(&mut sim, foot, [0.; 3], [0., 0., torque])?;
        }
        let now = Instant::now();
        if let Err(e) = sim.step_exact(1) {
            return Err(format!(
                "{} tick {tick}: {}",
                grid.name,
                sim.fault().map_or_else(|| e.to_string(), |f| f.to_string())
            )
            .into());
        }
        costs.push(now.elapsed().as_nanos() as u64);
        let report = last_step(&sim).ok_or("missing turntable state")?;
        let row = panel_steps(&sim).first().ok_or("missing force plate")?;
        let q = row.deck.orientation;
        let expected = DQuat::from_rotation_z(DT * RATE * STOP.min(tick) as f64);
        if (q * expected.inverse()).to_scaled_axis().length() > 1e-10 {
            return Err("table missed a commanded angle or stop".into());
        }
        if tick > STOP && (row.spin_work_j != 0. || row.angular_velocity != DVec3::ZERO) {
            return Err("stale prescribed rotation after stop".into());
        }
        support_work += row.spin_work_j;
        angular += row.step.angular_impulse;
        sliding += row.step.sliding_work;
        torque_j += DT * torque;
        let residual = (INERTIA * report.state.spin - angular - torque_j * DVec3::Z).length();
        if residual > 1e-10 || (row.support_work_j - row.spin_work_j).abs() > 1e-12 {
            return Err(format!("rotational momentum/work mismatch at {tick}: {residual}").into());
        }
        samples.push(Sample {
            tick,
            center_m: report.state.center.to_array(),
            orientation_xyzw: report.state.orientation.to_array(),
            table_orientation_xyzw: q.to_array(),
            spin_rad_s: report.state.spin.to_array(),
            table_rate_rad_s: row.angular_velocity.z,
            force_n: row.step.contact.force.to_array(),
            contact_torque_nm: row.step.contact.torque.to_array(),
            cumulative_angular_impulse_nms: angular.to_array(),
            external_torque_nm: torque,
            cumulative_spin_work_j: support_work,
            spin_work_in_step_j: row.spin_work_j,
            cumulative_sliding_work_j: sliding,
            selected_substeps: report
                .turntable_accuracy
                .map(|a| a.substeps)
                .or_else(|| report.adaptive.map(|a| a.substeps))
                .or_else(|| report.panel_accuracy.map(|a| a.substeps)),
            angular_momentum_error_nms: residual,
        });
    }
    if support_work <= 0. || sliding >= 0. || samples[STOP - 1].spin_rad_s[2] <= 0. {
        return Err("missing rotational energy and angular impulse transfer".into());
    }
    Ok((
        Run {
            name: grid.name,
            samples,
        },
        costs,
    ))
}
fn compare(h: &Run, fine: &Run) -> Result<serde_json::Value> {
    let (mut orientation, mut position, mut impulse, mut work, mut spin, mut residual) =
        (0_f64, 0_f64, 0_f64, 0_f64, 0_f64, 0_f64);
    for (a, b) in h.samples.iter().zip(&fine.samples) {
        if a.tick != b.tick {
            return Err("turntable clock mismatch".into());
        }
        orientation = orientation.max(
            (DQuat::from_array(a.orientation_xyzw)
                * DQuat::from_array(b.orientation_xyzw).inverse())
            .to_scaled_axis()
            .length(),
        );
        position = position.max(DVec3::from(a.center_m).distance(b.center_m.into()));
        impulse = impulse.max(
            DVec3::from(a.cumulative_angular_impulse_nms)
                .distance(b.cumulative_angular_impulse_nms.into()),
        );
        work = work.max((a.cumulative_spin_work_j - b.cumulative_spin_work_j).abs());
        spin = spin.max(DVec3::from(a.spin_rad_s).distance(b.spin_rad_s.into()));
        residual = residual.max(a.angular_momentum_error_nms);
    }
    if position > 0.0001 || orientation > 0.001 || impulse > 0.001 || work > 0.002 {
        return Err(format!("{} turntable comparison exceeds fixture tolerances: {position} m, {orientation} rad, {impulse} Nms, {work} J",h.name).into());
    }
    let stop = &h.samples[STOP - 1];
    Ok(
        json!({"name":h.name,"matched_clocks":h.samples.len(),"stop_table_angle_rad":RATE*DT*STOP as f64,
       "max_foot_position_error_m":position,"max_foot_orientation_error_rad":orientation,"max_angular_impulse_error_nms":impulse,"max_support_work_error_j":work,
       "max_spin_error_rad_s":spin,"max_angular_momentum_error_nms":residual,"stop_contact_angular_impulse_nms":stop.cumulative_angular_impulse_nms,
       "spin_up_support_work_j":stop.cumulative_spin_work_j,"stop_foot_spin_rad_s":stop.spin_rad_s,
       "external_torque_impulse_nms":LOAD*DT*(PULSE_END-PULSE_BEGIN) as f64,"stopped_ticks_with_zero_spin_work":END-STOP,
       "min_selected_substeps":h.samples.iter().filter_map(|a|a.selected_substeps).min(),"max_selected_substeps":h.samples.iter().filter_map(|a|a.selected_substeps).max()}),
    )
}
fn main() -> Result<()> {
    let args = Args::parse();
    if args.output.exists() || args.repetitions == 0 {
        return Err("provide a fresh directory and positive repetitions".into());
    }
    let parent = args
        .output
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::Builder::new()
        .prefix(".turntable-")
        .tempdir_in(parent)?;
    let mut runs = vec![];
    let mut timings = vec![];
    for (i, grid) in [
        Grid {
            name: "Fixed",
            n: 16,
            d: 1,
            adaptive: false,
        },
        Grid {
            name: "Fine",
            n: 64,
            d: 3,
            adaptive: false,
        },
        Grid {
            name: "Adaptive",
            n: 2,
            d: 0,
            adaptive: true,
        },
    ]
    .into_iter()
    .enumerate()
    {
        let xml = scene(grid);
        fs::write(stage.path().join(format!("grid-{i}.sdf")), &xml)?;
        let (r, _) = run(grid, &xml)?;
        runs.push(r);
        let mut costs = Vec::new();
        for _ in 0..args.repetitions {
            costs.extend(run(grid, &xml)?.1);
        }
        costs.sort_unstable();
        timings.push(costs[(95 * costs.len()).div_ceil(100) - 1] as f64 / 1000.);
    }
    let mut checks = vec![];
    for (r, us) in runs.iter().zip(timings) {
        let mut report = compare(r, &runs[1])?;
        report["p95_full_step_us"] = json!(us);
        checks.push(report);
    }
    fs::write(
        stage.path().join("checks.json"),
        serde_json::to_vec_pretty(
            &json!({"repetitions":args.repetitions,"public_step_s":DT,"results":checks}),
        )?,
    )?;
    fs::write(
        stage.path().join("rotation.json"),
        serde_json::to_vec(&runs)?,
    )?;
    view::write(stage.path(), &runs)?;
    fs::rename(stage.path(), args.output)?;
    Ok(())
}
