// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cross-check the coupled finite-panel solver against independent closed-form
//! normal-drop mechanics, then export the measured poses to a Gazebo replay.
#[path = "panel_landings/layouts.rs"]
mod layouts;
#[allow(dead_code)]
mod soft_replay;
use glam::{DQuat, DVec3};
use gz_physics::{
    flat_landing::FlatLanding,
    sole_contact::{SoleBody, SoleState},
    sole_supports::advance_panels,
};
use layouts::{Tile, Trial};
use serde_json::{Value, json};
use soft_replay::Frame;
use std::{
    collections::BTreeMap, error::Error, f64::consts::PI, fs, path::PathBuf, time::Duration,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn run(
    number: usize,
    name: &str,
    tiles: &[Tile],
    trial: &Trial,
    steps: u32,
    frames: &mut Vec<(String, String)>,
) -> Result<Value> {
    let half = DVec3::new(0.2, 0.1, 0.045);
    let mass = trial.mass;
    let body = SoleBody {
        half_extents: half,
        foot_offset: DVec3::ZERO,
        mass,
        inertia: mass / 3.
            * DVec3::new(
                half.y * half.y + half.z * half.z,
                half.x * half.x + half.z * half.z,
                half.x * half.x + half.y * half.y,
            ),
    };
    let normal = trial.rotation * DVec3::Z;
    let total_k = tiles
        .iter()
        .map(|t| t.area * t.stiffness_scale * trial.stiffness)
        .sum();
    let reference = FlatLanding::new(mass, total_k, trial.damping, trial.incoming)
        .ok_or("unsupported analytical drop")?;
    let panels = trial.panels(tiles);
    let mut moving = panels.clone();
    let mut s = SoleState {
        center: half.z * normal,
        orientation: trial.rotation,
        velocity: trial.drift - trial.incoming * normal,
        spin: DVec3::ZERO,
    };
    let mut impulses = vec![0.; tiles.len()];
    let mut heat = 0.;
    let mut previous = 0.;
    let (mut position, mut speed, mut force, mut impulse, mut heat_error, mut spin, mut work) =
        (0_f64, 0_f64, 0_f64, 0_f64, 0_f64, 0_f64, 0_f64);
    let mut support_work = 0.;
    let mut samples = vec![];
    let start_frame = frames.len();
    for i in 0..=12 {
        let exact = reference
            .at_phase(PI * f64::from(i) / 12.)
            .ok_or("invalid reference phase")?;
        let dt = exact.elapsed_s - previous;
        let mut fz = 0.;
        let mut pressure_center = None;
        let mut margin = None;
        let mut support_wrenches = vec![];
        if i > 0 {
            let rows = advance_panels(
                &body,
                &mut s,
                &moving,
                DVec3::ZERO,
                DVec3::ZERO,
                DVec3::ZERO,
                dt,
                steps,
                0,
            )?;
            let plane_origin = trial.drift * exact.elapsed_s;
            if let Some(area) = gz_physics::sole_balance::SupportFootprint::from_panels(
                &rows,
                plane_origin,
                trial.rotation,
                1e-8,
            ) {
                let margin_m = area
                    .margin_m(area.pressure_center)
                    .ok_or("nonfinite support margin")?;
                if area.pressure_center.length() > 1e-8 || margin_m < -1e-12 {
                    return Err("centered drop violates measured pressure balance".into());
                }
                pressure_center = Some(area.pressure_center.to_array());
                margin = Some(margin_m);
            }
            for (index, (r, p)) in rows.iter().zip(&mut moving).enumerate() {
                p.deck.center = panels[index].deck.center + exact.elapsed_s * trial.drift;
                let w = gz_physics::sole_balance::SupportWrench::at_sensor(
                    r,
                    s.center,
                    p.deck.center,
                    p.deck.orientation,
                )
                .ok_or("invalid force plate frame")?;
                if (w.force_n.z + r.on_sole.contact.normal_force).abs() > 1e-9 {
                    return Err("force plate normal mismatch".into());
                }
                support_wrenches.push(
                    json!({"force_n":w.force_n.to_array(),"moment_nm":w.moment_nm.to_array()}),
                );
                let k = tiles[index].area * tiles[index].stiffness_scale * trial.stiffness;
                impulses[index] += r.on_sole.normal_impulse;
                heat += r.on_sole.dissipated_energy;
                support_work += r.support_work_j;
                impulse = impulse.max((impulses[index] - exact.impulse_ns * k / total_k).abs());
                fz += r.on_sole.contact.normal_force;
            }
        }
        let expected_center =
            (half.z - exact.compression_m) * normal + exact.elapsed_s * trial.drift;
        position = position.max(expected_center.distance(s.center));
        speed = speed.max(
            s.velocity
                .distance(trial.drift - exact.closing_m_s * normal),
        );
        force = force.max((fz - exact.normal_force_n).abs());
        heat_error = heat_error.max((heat - exact.heat_j).abs());
        work = work.max((support_work - exact.impulse_ns * normal.dot(trial.drift)).abs());
        spin = spin.max(s.spin.length());
        samples.push(json!({"t_s":exact.elapsed_s,
            "closing_m_s":normal.dot(trial.drift-s.velocity),
            "compression_m":half.z-normal.dot(s.center-trial.drift*exact.elapsed_s),
            "footprint_margin_m":margin,
            "pressure_center_xy_m":pressure_center,
            "support_wrenches":support_wrenches,
            "center_m":s.center.to_array(),
            "expected_center_m":expected_center.to_array(),
            "normal_n":fz,
            "expected_normal_n":exact.normal_force_n,
            "panel_impulses_ns":&impulses,
            "heat_j":heat,
            "expected_heat_j":exact.heat_j,
            "support_work_j":support_work}));
        let mut frame = Frame::default();
        let tag = format!("{number:03}_{name}_{}", trial.name);
        let display = |x: DVec3, lane: f64| 2.5 * x + DVec3::new(0., lane, 0.6);
        for (label, offset, center, q, color) in [
            (
                "solver",
                -0.5,
                s.center,
                s.orientation,
                [0.12, 0.65, 0.9, 1.],
            ),
            (
                "reference",
                0.5,
                expected_center,
                trial.rotation,
                [0.95, 0.56, 0.15, 1.],
            ),
        ] {
            for (j, p) in moving.iter().enumerate() {
                frame.add(
                    &format!("{tag}_{label}_deck_{j}"),
                    display(p.deck.center, offset),
                    p.deck.orientation,
                    json!({"kind":"box","size":(5.*p.deck.half_extents).to_array()}),
                    [0.28, 0.34, 0.4, 1.],
                );
            }
            frame.add(
                &format!("{tag}_{label}_foot"),
                display(center, offset),
                q,
                json!({"kind":"box","size":(5.*half).to_array()}),
                color,
            );
            // Separate force gauge in kN. Contact compression is small at true scale.
            let load = if label == "solver" {
                fz
            } else {
                exact.normal_force_n
            };
            let height = 0.00015 * load;
            frame.add(
                &format!("{tag}_{label}_normal_force"),
                DVec3::new(-0.95, offset, 0.15 + height),
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":0.04}),
                color,
            );
        }
        // A new experiment has new identities, even for similarly named bodies.
        for (j, (visual, model)) in frame.visuals.iter_mut().zip(&mut frame.models).enumerate() {
            let id = ((number as u64) << 16) + 2 * j as u64 + 2;
            visual["id"] = json!(id + 1);
            model["model_id"] = json!(id);
            model["visual_ids"] = json!([id + 1]);
        }
        let n = frames.len();
        frames.push(frame.finish(Duration::from_millis(n as u64 * 100), n + 1));
        previous = exact.elapsed_s;
    }
    // Scale each residual by a physical quantity of the same dimension.
    let x_scale = reference.maximum_compression_m();
    let v_scale = trial.incoming;
    let f_scale = total_k * x_scale;
    let j_scale = mass * v_scale;
    let energy = 0.5 * mass * v_scale * v_scale;
    let relative = [
        position / x_scale,
        speed / v_scale,
        force / f_scale,
        impulse / j_scale,
        heat_error / energy,
        work / (energy + j_scale * trial.drift.length()),
    ]
    .into_iter()
    .fold(0_f64, f64::max);
    if !relative.is_finite() || relative > 0.0004 || spin > 1e-8 {
        return Err(format!(
            "case {number} {name}/{} error {relative:.6e}, rotation {spin:.6e}",
            trial.name
        )
        .into());
    }
    Ok(json!({"case":number,
            "layout":name,
            "drop":trial.name,
            "first_frame":start_frame+1,
            "last_frame":frames.len(),
            "covered_area_m2":tiles.iter().map(|t|t.area).sum::<f64>(),
            "stiffness_n_m":total_k,
            "mass_kg":mass,
            "damping_s_m":trial.damping,
            "impact_m_s":v_scale,
            "substeps_per_sample":steps,
            "max_errors":{"center_m":position,
            "velocity_m_s":speed,
            "normal_n":force,
            "panel_impulse_ns":impulse,
            "heat_j":heat_error,
            "support_work_j":work,
            "angular_speed_rad_s":spin,
            "scaled":relative},
            "samples":samples}))
}
fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1);
    let out = PathBuf::from(
        args.next()
            .ok_or("usage: panel_landings OUTPUT [SUBSTEPS]")?,
    );
    let steps: u32 = args
        .next()
        .map(|s| s.into_string().map_err(|_| "invalid substeps"))
        .transpose()?
        .map(|s| s.parse())
        .transpose()?
        .unwrap_or(24);
    if args.next().is_some() || steps == 0 {
        return Err("usage: panel_landings OUTPUT [SUBSTEPS]".into());
    }
    let (mut frames, mut cases) = (vec![], vec![]);
    for trial in layouts::trials() {
        for (name, tiles) in layouts::layouts() {
            cases.push(run(
                cases.len() + 1,
                name,
                &tiles,
                &trial,
                steps,
                &mut frames,
            )?);
        }
    }
    fs::create_dir_all(&out)?;
    gz_sim_cli::export_archive(&out.join("replay"), BTreeMap::new(), frames)?;
    let worst = cases
        .iter()
        .filter_map(|r| r["max_errors"]["scaled"].as_f64())
        .fold(0_f64, f64::max);
    fs::write(out.join("cases.json"), serde_json::to_vec(&cases)?)?;
    println!(
        "{} physical cases passed; maximum scaled residual {worst:.6e}",
        cases.len()
    );
    Ok(())
}
