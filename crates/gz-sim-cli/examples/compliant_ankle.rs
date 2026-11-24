// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A planted foot, a moving native ankle, and the same foot with its ankle locked.
#![allow(deprecated)]
#[path = "compliant_ankle/reference.rs"]
mod reference;
use glam::{DQuat, DVec2, DVec3};
use gz_physics::PhysicsBackend;
use gz_sim_core::EntityId;
use gz_sim_scene::{
    BodyHandle, JointHandle, Name, PhysicsEngine, damped_sole::ankle_step,
    load_world_with_sdf_step, queue_link_world_wrench,
};
use reference::State;
use serde_json::{Value, json};
use std::{error::Error, path::Path};
const WORLD: &str = include_str!("../../../examples/worlds/local_compliant_ankle.sdf");
const TICKS: usize = 480;
const DT: f64 = 0.0005;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn world(locked: bool, grid: usize) -> String {
    let mut s = WORLD.replace("<substeps>8", &format!("<substeps>{grid}"));
    if locked {
        s = s
            .replace("type=\"revolute\"", "type=\"fixed\"")
            .replace("<axis><xyz>0 1 0</xyz></axis>", "");
    }
    s
}
fn angle(q: [f64; 4]) -> f64 {
    let up = DQuat::from_array(q) * DVec3::Z;
    up.x.atan2(up.z)
}
fn run(
    locked: bool,
    grid: usize,
    reference: &[State],
    driven: bool,
    out: Option<&Path>,
) -> Result<Value> {
    let xml = world(locked, grid);
    let sdf = sdformat::load_str(&xml, "ankle.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "ankle.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let named = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
    };
    let foot = named("default::boot::sole")
        .get::<BodyHandle>()
        .unwrap()
        .id();
    let shank = named("default::boot::upper");
    let shank_id = shank.get::<EntityId>().unwrap().get();
    let upper = shank.get::<BodyHandle>().unwrap().id();
    let joint = named("default::boot::ankle")
        .get::<JointHandle>()
        .unwrap()
        .id();
    let mut impulse = DVec3::ZERO;
    let mut work = 0.;
    let mut previous = DVec3::new(0.36, 0., 0.);
    let mut max = [0_f64; 8];
    let mut bounds = [f64::INFINITY; 4];
    let mut frames = vec![];
    let mut trace = vec![];
    let mut final_angle = 0.;
    let mut final_x = 0.;
    let mut peak_angle = 0_f64;
    let mut peak_load = 0_f64;
    let mut previous_upper = DVec2::new(0.18, 0.);
    let mut joint_error = 0_f64;
    let mut joint_up = 0_f64;
    let mut slip_range = [f64::INFINITY, f64::NEG_INFINITY];
    let mut rigid_impulse = 0_f64;
    let mut joint_total = DVec2::ZERO;
    for (tick, r) in reference.iter().enumerate() {
        if tick > 0 {
            let m = if driven { reference::torque(tick) } else { 0. };
            if m != 0. {
                queue_link_world_wrench(&mut sim, shank_id, [0.; 3], [0., m, 0.])?;
            }
            sim.step_exact(1).map_err(|e| {
                format!(
                    "tick {tick}: {e} {:?}",
                    sim.world().resource::<gz_sim_core::StepFault>().error()
                )
            })?;
        }
        let b = sim.world().resource::<PhysicsBackend>();
        let low = b.state(foot).unwrap();
        let hi = b.state(upper).unwrap();
        let pos = DVec2::new(low.position[0], low.position[2]);
        let upper_pos = DVec2::new(hi.position[0], hi.position[2]);
        let p = DVec3::from(low.velocity) + DVec3::from(hi.velocity);
        let a = angle(low.orientation);
        let u = angle(hi.orientation);
        let public = b.joint_state(joint).unwrap().position;
        let mut normal = 0.;
        let mut shear = 0.;
        if let Some(s) = ankle_step(&sim) {
            let row = s.panel.step;
            let vu = DVec2::new(hi.velocity[0], hi.velocity[2]);
            let transmission = vu - previous_upper + 9.81 * DT * DVec2::Y;
            let independent =
                r.shank_velocity() - reference[tick - 1].shank_velocity() + 9.81 * DT * DVec2::Y;
            joint_error = joint_error.max(transmission.distance(independent));
            joint_up = joint_up.max(transmission.y / DT);
            joint_total += transmission;
            previous_upper = vu;
            if let Some(site) = row.contact.center_of_pressure {
                let slip = (DVec3::from(low.velocity)
                    + DVec3::from(low.angular_velocity).cross(site - DVec3::from(low.position)))
                .x;
                slip_range[0] = slip_range[0].min(slip);
                slip_range[1] = slip_range[1].max(slip);
            }
            let solver = b
                .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
                .unwrap();
            for pair in solver.rigid_contact_observations() {
                rigid_impulse = rigid_impulse.max(pair.normal_impulse);
            }

            impulse += row.impulse;
            work += row.sliding_work;
            normal = row.normal_impulse / DT;
            shear = row.impulse.x / DT;
            bounds[0] = bounds[0].min(normal);
            bounds[1] = bounds[1].min(s.edge_margin_m);
            bounds[2] = bounds[2].min(s.shank_margin_m);
            bounds[3] = bounds[3].min(0.32 * row.normal_impulse - row.impulse.x.abs());
            max[6] = max[6].max((p - previous - row.impulse + DT * 19.62 * DVec3::Z).length());
            peak_load = peak_load.max(row.peak_normal_force);
        }
        max[7] = max[7].max((public - (u - a)).abs());
        for (e, v) in max.iter_mut().zip([
            pos.distance(DVec2::new(r.q.x, r.q.y)),
            upper_pos.distance(r.shank_center()),
            (public - (r.q.w - r.q.z)).abs(),
            (impulse.z - r.impulse.y).abs(),
            (impulse.x - r.impulse.x).abs(),
            (work - r.work).abs(),
        ]) {
            *e = e.max(v);
        }
        peak_angle = peak_angle.max(public.abs());
        final_angle = public;
        final_x = pos.x;
        if out.is_some() && tick % 4 == 0 {
            frames.push(gz_sim_cli::primitive_frame(
                &sim,
                &sdf,
                "default",
                frames.len() as u64 + 1,
            )?);
        }
        trace.push(json!([
            tick,
            public,
            r.q.w - r.q.z,
            pos.x,
            r.q.x,
            normal,
            shear,
            impulse.x,
            impulse.z,
            work
        ]));
        previous = p;
    }
    if let Some(path) = out {
        gz_sim_cli::export_archive(path, Default::default(), frames)?;
        std::fs::write(
            path.with_extension("trace.json"),
            serde_json::to_vec(&trace)?,
        )?;
    }
    Ok(
        json!({"substeps":grid,"locked":locked,"driven":driven,"normal_impulse_ns":impulse.z,"shear_impulse_ns":impulse.x,"sliding_work_j":work,"final_foot_x_m":final_x,"final_joint_rad":final_angle,"max_joint_rad":peak_angle,
        "max_per_tick_joint_transmission_error_ns":joint_error,"joint_transmission_total_xz_ns":joint_total.to_array(),"peak_mean_joint_upward_n":joint_up,"pressure_center_slip_speed_range_m_s":slip_range,"maximum_duplicate_rigid_impulse_ns":rigid_impulse,
        "min_mean_normal_n":bounds[0],"peak_normal_n":peak_load,"min_edge_margin_m":bounds[1],"min_shank_margin_m":bounds[2],"min_coulomb_impulse_margin_ns":bounds[3],"max_foot_error_m":max[0],"max_shank_error_m":max[1],"max_joint_error_rad":max[2],"max_cumulative_normal_error_ns":max[3],"max_cumulative_shear_error_ns":max[4],"max_friction_work_error_j":max[5],"max_linear_balance_residual_ns":max[6],"max_joint_observer_error_rad":max[7]}),
    )
}
fn record(path: &Path) -> Result<Value> {
    let mut branches = vec![];
    for locked in [false, true] {
        let r = reference::trajectory(locked, 64, TICKS, DT, true);
        let check = reference::trajectory(locked, 32, TICKS, DT, true);
        let mut delta = [0_f64; 3];
        for (a, b) in r.iter().zip(check) {
            for (d, v) in delta.iter_mut().zip([
                (a.q - b.q).abs().max_element(),
                a.impulse.distance(b.impulse),
                (a.work - b.work).abs(),
            ]) {
                *d = d.max(v);
            }
        }
        let mut rows = vec![];
        for grid in [1, 2, 4, 8, 16] {
            rows.push(run(
                locked,
                grid,
                &r,
                true,
                (grid == 8)
                    .then(|| path.join(if locked { "locked" } else { "free" }))
                    .as_deref(),
            )?);
        }
        let neutral = run(
            locked,
            8,
            &reference::trajectory(locked, 64, TICKS, DT, false),
            false,
            None,
        )?;
        let last = r.last().unwrap();
        branches.push(json!({"locked":locked,"reference_halving_max_pose_si":delta[0],"reference_halving_max_impulse_ns":delta[1],"reference_halving_max_work_j":delta[2],"reference_final_joint_rad":last.q.w-last.q.z,"reference_normal_impulse_ns":last.impulse.y,"reference_shear_impulse_ns":last.impulse.x,"reference_sliding_work_j":last.work,"driven":rows,"without_external_moment":neutral}));
    }
    Ok(
        json!({"public_dt_s":DT,"public_ticks":TICKS,"moment_nm":0.55,"moment_first_tick":81,"moment_last_tick":200,"reference_steps_per_tick":64,"branches":branches}),
    )
}
fn main() -> Result<()> {
    let dir = std::env::args_os()
        .nth(1)
        .ok_or("supply a fresh output directory")?;
    let path = Path::new(&dir);
    std::fs::create_dir(path)?;
    let result = serde_json::to_string_pretty(&record(path)?)?;
    std::fs::write(path.join("comparison.json"), &result)?;
    println!("{result}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn jointly_measured_slip_pressures_and_joint_trajectory_converge_to_the_independent_reference()
    -> Result<()> {
        let dir = tempfile::tempdir()?;
        let report = record(dir.path())?;
        for b in report["branches"].as_array().unwrap() {
            let rows = b["driven"].as_array().unwrap();
            for field in [
                "max_cumulative_normal_error_ns",
                "max_cumulative_shear_error_ns",
                "max_foot_error_m",
                "max_per_tick_joint_transmission_error_ns",
            ] {
                let mut previous = f64::INFINITY;
                for row in rows {
                    let v = row[field].as_f64().unwrap();
                    assert!(v < previous, "{field}: {v} vs {previous}");
                    previous = v;
                    assert!(row["min_coulomb_impulse_margin_ns"].as_f64().unwrap() >= 0.);
                    assert!(row["min_mean_normal_n"].as_f64().unwrap() > 17.);
                    assert_eq!(row["maximum_duplicate_rigid_impulse_ns"], 0.);
                    assert!(row["sliding_work_j"].as_f64().unwrap() < -0.02);
                }
            }
            let eight = &rows[3];
            if b["locked"] == true {
                assert_eq!(eight["final_joint_rad"], 0.);
            } else {
                assert!(eight["max_joint_error_rad"].as_f64().unwrap() < 0.00009);
                assert!(
                    eight["final_joint_rad"].as_f64().unwrap()
                        - b["without_external_moment"]["final_joint_rad"]
                            .as_f64()
                            .unwrap()
                        > 0.13
                );
            }
        }
        Ok(())
    }
}
