// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Finite plantar support as a freely hinged shank strikes or misses the same deck.
#![allow(deprecated)]
#[allow(dead_code)]
#[path = "compliant_ankle/reference.rs"]
mod planar;
#[path = "compliant_ankle_edge/reference.rs"]
mod reference;
use glam::{DVec2, DVec3};
use gz_physics::PhysicsBackend;
use gz_sim_core::EntityId;
use gz_sim_scene::{
    BodyHandle, JointHandle, Name, PhysicsEngine, damped_sole::ankle_step,
    load_world_with_sdf_step, queue_link_world_wrench,
};
use reference::{Fixture, Sample};
use serde_json::{Value, json};
use std::{error::Error, path::Path};
const WORLD: &str = include_str!("../../../examples/worlds/local_compliant_ankle_edge.sdf");
const DT: f64 = 0.0005;
const TICKS: usize = 100;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn trial(f: Fixture, grid: u32, reference: &[Sample], dir: Option<&Path>) -> Result<Value> {
    let mut xml = WORLD.replace("<substeps>8", &format!("<substeps>{grid}"));
    if f.shank_half < 0.1 {
        xml = xml.replace("0.08 0.10 0.34", "0.08 0.10 0.12");
    }
    if f.right > 0.5 {
        xml = xml.replace("-0.415 0 -0.03", "0 0 -0.03");
    }
    let sdf = sdformat::load_str(&xml, "edge.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "edge.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let get = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
    };
    let sole = get("default::boot::sole").get::<BodyHandle>().unwrap().id();
    let upper = get("default::boot::upper");
    let upper_id = upper.get::<EntityId>().unwrap().get();
    let shank = upper.get::<BodyHandle>().unwrap().id();
    let joint = get("default::boot::ankle")
        .get::<JointHandle>()
        .unwrap()
        .id();
    let mut frames = vec![];
    let mut trajectory = vec![];
    let mut pulse = 0.;
    let mut first = None;
    let mut first_impulse = None;
    let mut impulse = DVec3::ZERO;
    let mut angular = DVec3::ZERO;
    let mut angle_error = 0_f64;
    let mut center_error = 0_f64;
    let mut normal_error = 0_f64;
    let mut shear_error = 0_f64;
    let mut shank_impulse_error = 0_f64;
    let mut speed_error = 0_f64;
    let mut momentum_residual = 0_f64;
    let mut last_momentum = None;
    let mut max_loaded_gap: Option<f64> = None;
    let mut min_edge_margin = f64::INFINITY;
    let mut min_shank_margin = f64::INFINITY;
    let mut min_sole_force = f64::INFINITY;
    let mut min_area = f64::INFINITY;
    let mut min_sole_force_during_strike = f64::INFINITY;
    let mut toe_overhang = 0_f64;
    let mut pts = 0;
    let mut boundary = 0.;
    let mut covered = 0.;
    let mut ending_angle = 0.;
    let mut sum_hard_at_ref_window = 0.;
    for (i, r) in reference.iter().enumerate() {
        if i > 0 {
            if reference::moment(i) != 0. {
                queue_link_world_wrench(
                    &mut sim,
                    upper_id,
                    [0.; 3],
                    [0., reference::moment(i), 0.],
                )?;
            }
            sim.step_exact(1).map_err(|e| {
                format!(
                    "grid {grid} tick {i}: {e} {:?}",
                    sim.world().resource::<gz_sim_core::StepFault>().error()
                )
            })?;
        }
        let backend = sim.world().resource::<PhysicsBackend>();
        let low = backend.state(sole).unwrap();
        let hi = backend.state(shank).unwrap();
        let joint_state = backend.joint_state(joint).unwrap();
        let q = joint_state.position;
        let rot = glam::DQuat::from_array(low.orientation);
        let front = low.position[0]
            + (rot * DVec3::X).x.abs() * 0.18
            + (rot * DVec3::Y).x.abs() * 0.10
            + (rot * DVec3::Z).x.abs() * 0.025;
        toe_overhang = toe_overhang.max(front - f.right);
        let momentum = DVec3::from(low.velocity) + DVec3::from(hi.velocity);
        let mut balance = 0.;
        let mut lower = 0.;
        let mut upper = 0.;
        let mut margin = 0.;
        let mut area = 0.;
        if let Some(row) = ankle_step(&sim) {
            lower = row.panel.step.normal_impulse;
            upper = row.shank_contact.normal_ns;
            pulse += upper;
            if let Some(gap) = row.shank_contact.max_loaded_gap_m {
                max_loaded_gap = Some(max_loaded_gap.map_or(gap, |v| v.max(gap)));
            }
            let external = row.panel.step.impulse + DVec3::from(row.shank_contact.impulse)
                - 2. * 9.81 * DT * DVec3::Z;
            balance = (momentum - last_momentum.unwrap_or(momentum) - external).length();
            angular += DVec3::from(row.shank_contact.angular_impulse);
            impulse += row.panel.step.impulse;
            margin = row.edge_margin_m;
            area = row.panel.step.contact.area;
            covered = area;
            min_edge_margin = min_edge_margin.min(margin);
            min_shank_margin = min_shank_margin.min(row.shank_margin_m);
            min_sole_force = min_sole_force.min(lower / DT);
            min_area = min_area.min(area);
            boundary += row.panel.step.boundary_work;
            if upper > 0. {
                min_sole_force_during_strike = min_sole_force_during_strike.min(lower / DT);
                pts = pts.max(row.shank_contact.peak_points);
                if first.is_none() {
                    first = Some(
                        (i - 1) as f64 * DT + row.shank_contact.first_loaded_offset_s.unwrap(),
                    );
                    first_impulse = Some(upper);
                }
            }
        }
        let upper_ref = DVec2::new(r.state.q.x, r.state.q.y)
            + 0.16 * planar::up(r.state.q.z)
            + 0.2 * planar::up(r.state.q.w);
        angle_error = angle_error.max((q - (r.state.q.w - r.state.q.z + 1.96)).abs());
        center_error =
            center_error.max(DVec2::new(hi.position[0], hi.position[2]).distance(upper_ref));
        normal_error = normal_error.max((impulse.z - r.state.impulse.y).abs());
        shear_error = shear_error.max((impulse.x - r.state.impulse.x).abs());
        shank_impulse_error = shank_impulse_error.max((pulse - r.hard).abs());
        speed_error = speed_error.max((joint_state.velocity - (r.state.dq.w - r.state.dq.z)).abs());
        momentum_residual = momentum_residual.max(balance);
        last_momentum = Some(momentum);
        ending_angle = q;
        if i == 68 {
            sum_hard_at_ref_window = pulse;
        }
        if dir.is_some() {
            frames.push(gz_sim_cli::primitive_frame(
                &sim,
                &sdf,
                "default",
                i as u64 + 1,
            )?);
        }
        trajectory.push(json!([
            i,
            margin,
            area,
            lower,
            upper,
            q,
            r.state.q.w - r.state.q.z + 1.96,
            pulse,
            r.hard,
            low.position,
            hi.position
        ]));
    }
    if let Some(dir) = dir {
        gz_sim_cli::export_archive(dir, Default::default(), frames)?;
        std::fs::write(
            dir.with_extension("trace.json"),
            serde_json::to_vec(&trajectory)?,
        )?;
    }
    Ok(json!({
        "substeps": grid,
        "end_angle_rad": ending_angle,
        "hard_total_ns": pulse,
        "first_hard_time_s": first,
        "first_hard_public_impulse_ns": first_impulse,
        "hard_at_34ms_ns": sum_hard_at_ref_window,
        "hard_angular_impulse_nms": angular.to_array(),
        "max_loaded_shank_points": pts,
        "plantar_impulse_ns": impulse.to_array(),
        "plantar_boundary_work_j": boundary,
        "max_joint_speed_error_rad_s": speed_error,
        "max_tick_momentum_residual_ns": momentum_residual,
        "max_loaded_endpoint_gap_m": max_loaded_gap,
        "max_overhang_m": toe_overhang,
        "max_swept_overhang_bound_m": (-min_edge_margin).max(0.),
        "min_shank_swept_margin_m": min_shank_margin,
        "min_mean_plantar_force_n": min_sole_force,
        "min_plantar_force_during_strike_n": first.map(|_| min_sole_force_during_strike),
        "min_pressure_area_m2": min_area,
        "end_pressure_area_m2": covered,
        "max_angle_error_rad": angle_error,
        "max_shank_trajectory_error_m": center_error,
        "max_cumulative_plantar_normal_error_ns": normal_error,
        "max_cumulative_plantar_shear_error_ns": shear_error,
        "max_cumulative_shank_error_ns": shank_impulse_error
    }))
}
fn record(dir: &Path) -> Result<Value> {
    let mut lanes = vec![];
    for (name, f) in [
        (
            "strike",
            Fixture {
                right: 0.185,
                shank_half: 0.17,
            },
        ),
        (
            "miss",
            Fixture {
                right: 0.185,
                shank_half: 0.06,
            },
        ),
        (
            "full",
            Fixture {
                right: 0.60,
                shank_half: 0.17,
            },
        ),
    ] {
        let r = reference::solve(f, TICKS, DT, 64);
        let half = reference::solve(f, TICKS, DT, 32);
        let change = r
            .iter()
            .zip(half)
            .map(|(a, b)| (a.hard - b.hard).abs())
            .fold(0., f64::max);
        let last = r.last().unwrap();
        let mut rows = vec![];
        for grid in [2, 4, 8, 16, 32] {
            rows.push(trial(
                f,
                grid,
                &r,
                (grid == 8).then(|| dir.join(name)).as_deref(),
            )?);
        }
        lanes.push(json!({
            "case": name,
            "reference_hard_onset_s": last.onset,
            "reference_impact_ns": last.impact,
            "reference_shank_total_ns": last.hard,
            "reference_hard_at_34ms_ns": r[68].hard,
            "reference_plantar_impulse_xz_ns": last.state.impulse.to_array(),
            "reference_halving_max_shank_impulse_delta_ns": change,
            "grids": rows
        }));
    }
    Ok(json!({
        "dt_s": DT,
        "ticks": TICKS,
        "torque_nm": -0.45,
        "first_command_tick": 33,
        "reference_steps_per_public_tick": 64,
        "cases": lanes
    }))
}
fn main() -> Result<()> {
    let d = std::env::args_os()
        .nth(1)
        .ok_or("supply a fresh output directory")?;
    let dir = Path::new(&d);
    std::fs::create_dir(dir)?;
    let report = serde_json::to_string_pretty(&record(dir)?)?;
    std::fs::write(dir.join("comparison.json"), &report)?;
    println!("{report}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn independent_event_reference_and_native_rows_agree_on_the_strike_and_the_miss() {
        for (half, hit) in [(0.17, true), (0.06, false)] {
            let f = Fixture {
                right: 0.185,
                shank_half: half,
            };
            let reference = reference::solve(f, TICKS, DT, 64);
            let row = trial(f, 8, &reference, None).unwrap();
            let r = reference.last().unwrap();
            assert_eq!(r.onset.is_some(), hit);
            assert_eq!(row["max_loaded_shank_points"].as_u64().unwrap() > 0, hit);
            assert!(row["max_overhang_m"].as_f64().unwrap() > 0.025);
            assert!(row["min_mean_plantar_force_n"].as_f64().unwrap() > 5.);
            assert!(row["max_angle_error_rad"].as_f64().unwrap() < 0.0001);
            assert!(row["max_shank_trajectory_error_m"].as_f64().unwrap() < 2e-5);
            assert!(row["max_tick_momentum_residual_ns"].as_f64().unwrap() < 5e-5);
            if let Some(t) = r.onset {
                assert!((row["first_hard_time_s"].as_f64().unwrap() - t).abs() < DT / 8.);
                assert!((row["hard_total_ns"].as_f64().unwrap() - r.hard).abs() < 0.001);
                assert!(
                    (row["hard_at_34ms_ns"].as_f64().unwrap() - reference[68].hard).abs() < 0.001
                );
                assert!(row["max_loaded_endpoint_gap_m"].as_f64().unwrap() <= 1e-9);
            } else {
                assert_eq!(row["hard_total_ns"], 0.);
                assert!(row["max_loaded_endpoint_gap_m"].is_null());
            }
        }
    }
}
