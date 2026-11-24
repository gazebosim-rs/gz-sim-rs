// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the oblique two-link upper strike to independently integrated Cartesian dynamics.
#![allow(deprecated)]
#[path = "compliant_upper/reference.rs"]
mod reference;
use glam::{DQuat, DVec3};
use gz_physics::PhysicsBackend;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine,
    damped_sole::{last_step, panel_steps},
    load_world_with_sdf_step,
};
use reference::{G, INERTIA, MASS, State};
use serde_json::{Value, json};
use std::{error::Error, path::Path};
const WORLD: &str = include_str!("../../../examples/worlds/local_compliant_upper.sdf");
const TICKS: usize = 120;
const DT: f64 = 0.0005;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn rotation_error(a: DQuat, b: DQuat) -> f64 {
    let q = a * b.conjugate();
    2. * DVec3::new(q.x, q.y, q.z).length().atan2(q.w.abs())
}
fn run(grid: usize, miss: bool, reference: &[State], path: Option<&Path>) -> Result<Value> {
    let mut world = WORLD.replace("<substeps>8", &format!("<substeps>{grid}"));
    if miss {
        world = world.replace("0.350 0 0.16", "0.350 0.65 0.16");
    }
    let sdf = sdformat::load_str(&world, "upper.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &world,
        "upper.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let get = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .get::<BodyHandle>()
            .unwrap()
            .id()
    };
    let low = get("default::boot::sole");
    let high = get("default::boot::upper");
    let mut frames = vec![];
    let mut trace = vec![];
    let mut upper = 0.;
    let mut lower = 0.;
    let mut max = [0_f64; 8];
    let mut first = None;
    let mut last = None;
    let mut peak = 0_f64;
    let mut min_support = f64::INFINITY;
    let mut prior = (reference[0].p, reference[0].l);
    for (i, r) in reference.iter().enumerate() {
        if i > 0 {
            sim.step_exact(1)?;
        }
        let native = sim.world().resource::<PhysicsBackend>();
        let a = native.state(low).unwrap();
        let b = native.state(high).unwrap();
        let (xa, xb) = (DVec3::from(a.position), DVec3::from(b.position));
        let (va, vb) = (DVec3::from(a.velocity), DVec3::from(b.velocity));
        let center = 0.5 * (xa + xb);
        let p = va + vb;
        let q = DQuat::from_array(a.orientation);
        let w = DVec3::from(a.angular_velocity);
        let l = q * (DVec3::new(0.012, 0.016, 0.020) * (q.conjugate() * w))
            + 0.030 * DVec3::from(b.angular_velocity)
            + (xa - center).cross(va)
            + (xb - center).cross(vb);
        let (mut n, mut j) = (0., 0.);
        if let Some(row) = last_step(&sim) {
            let side = row.upper_side.unwrap();
            let mut joint = side.impulse;
            let mut angular = side.angular_impulse;
            for row in panel_steps(&sim) {
                joint += row.step.impulse;
                angular += row.step.angular_impulse;
                n += row.step.normal_impulse;
            }
            j = -side.impulse.x;
            upper += j;
            lower += n;
            max[6] = max[6].max((p - prior.0 - joint - DT * MASS * G).length());
            max[7] = max[7].max((l - prior.1 - angular).length());
            if j > 0. {
                first.get_or_insert(i);
                last = Some(i);
                min_support = min_support.min(n / DT);
            }
            peak = peak.max(side.peak_force_n);
        }
        for (m, d) in max.iter_mut().zip([
            center.distance(r.center),
            rotation_error(q, r.rotation()),
            p.distance(r.p),
            l.distance(r.l),
            (upper - r.upper_ns).abs(),
            (lower - r.lower_ns).abs(),
        ]) {
            *m = m.max(d);
        }
        if path.is_some() {
            frames.push(gz_sim_cli::primitive_frame(
                &sim,
                &sdf,
                "default",
                i as u64 + 1,
            )?);
        }
        trace.push(json!([i, j, n, upper, r.upper_ns, lower, r.lower_ns]));
        prior = (p, l);
    }
    if let Some(path) = path {
        gz_sim_cli::export_archive(path, Default::default(), frames)?;
        std::fs::write(
            path.with_extension("trace.json"),
            serde_json::to_vec(&trace)?,
        )?;
    }
    Ok(
        json!({"substeps":grid,"upper_total_ns":upper,"lower_total_ns":lower,"first_upper_tick":first,"last_upper_tick":last,"min_lower_mean_force_during_upper_n":first.map(|_|min_support),"peak_upper_n":peak,
        "max_com_error_m":max[0],"max_rotation_error_rad":max[1],"max_momentum_error_ns":max[2],"max_angular_momentum_error_nms":max[3],"max_cumulative_upper_error_ns":max[4],"max_cumulative_lower_error_ns":max[5],"max_linear_balance_residual_ns":max[6],"max_angular_balance_residual_nms":max[7]}),
    )
}
fn record(dir: &Path) -> Result<Value> {
    let fine = reference::trajectory(128, TICKS, DT, false);
    let check = reference::trajectory(64, TICKS, DT, false);
    let mut drift = [0_f64; 3];
    for (a, b) in fine.iter().zip(check) {
        for (m, e) in drift.iter_mut().zip([
            a.center.distance(b.center),
            (a.upper_ns - b.upper_ns).abs(),
            (a.lower_ns - b.lower_ns).abs(),
        ]) {
            *m = m.max(e);
        }
    }
    let mut trials = vec![];
    for n in [1, 2, 4, 8, 16] {
        trials.push(run(
            n,
            false,
            &fine,
            (n == 8).then(|| dir.join("strike")).as_deref(),
        )?);
    }
    let miss = run(
        8,
        true,
        &reference::trajectory(8, TICKS, DT, true),
        Some(&dir.join("miss")),
    )?;
    let last = fine.last().unwrap();
    Ok(
        json!({"public_dt_s":DT,"ticks":TICKS,"reference_substeps":128,"reference_refinement_delta":[drift[0],drift[1],drift[2]],"reference_upper_total_ns":last.upper_ns,"reference_lower_total_ns":last.lower_ns,"reference_final_spin_rad_s":(last.rotation()*((last.rotation().conjugate()*last.l)/INERTIA)).to_array(),"strike":trials,"miss":miss}),
    )
}
fn main() -> Result<()> {
    let root = std::env::args_os()
        .nth(1)
        .ok_or("supply a fresh output directory")?;
    let dir = Path::new(&root);
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
    fn reducing_the_material_step_converges_to_an_independent_coupled_trajectory() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let report = record(dir.path())?;
        let trials = report["strike"].as_array().unwrap();
        for (field, limit) in [
            ("max_cumulative_upper_error_ns", 3e-6),
            ("max_cumulative_lower_error_ns", 3e-6),
            ("max_momentum_error_ns", 3e-6),
            ("max_com_error_m", 2e-8),
        ] {
            let mut previous = f64::INFINITY;
            for row in trials {
                let value = row[field].as_f64().unwrap();
                assert!(value < previous, "{field}: {value} vs {previous}");
                previous = value;
                assert_eq!(row["first_upper_tick"], 27);
                assert!(row["min_lower_mean_force_during_upper_n"].as_f64().unwrap() > 19.);
                assert!(row["max_linear_balance_residual_ns"].as_f64().unwrap() < 2e-14);
                assert!(row["max_angular_balance_residual_nms"].as_f64().unwrap() < 2e-14);
            }
            assert!(previous < limit);
        }
        assert_eq!(report["miss"]["upper_total_ns"], 0.);
        assert!(
            (report["miss"]["lower_total_ns"].as_f64().unwrap() - 19.62 * DT * TICKS as f64).abs()
                < 1e-12
        );
        Ok(())
    }
}
