// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One stationary interval followed by a short, analytically soluble guide collision.
use super::*;
use std::time::Instant;

pub(super) fn fixture(step_ns: u64) -> Result<String> {
    if step_ns == 0 || 1_000_000_000 % step_ns != 0 {
        return Err("choose a positive nanosecond step dividing one second".into());
    }
    let mut sdf = xml(31)
        .replace("<gravity>0 0 -9.81", "<gravity>0 0 0")
        .replace(
            "<max_step_size>0.002",
            &format!("<max_step_size>{:.12}", step_ns as f64 * 1e-9),
        );
    let a = sdf.find("<trajectory ").unwrap();
    let b = a + sdf[a..].find("</trajectory>").unwrap() + 13;
    let mut curve = String::from("<trajectory end='hold'>");
    for (t, z) in [
        (0, 0.),
        (400_000_000, 0.),
        (408_000_000, 0.04),
        (416_000_000, 0.),
        (850_000_000, 0.),
    ] {
        curve.push_str(&format!(
            "<waypoint time_ns='{t}' displacement='0 0 {z}' velocity='0 0 0'/>"
        ));
    }
    curve.push_str("</trajectory>");
    sdf.replace_range(a..b, &curve);
    // All original points have the same authored height. Preserve their nonuniform Y positions.
    let start = sdf.find("<particle position").unwrap();
    let end = start + sdf[start..].find("</cable>").unwrap();
    let mut points = String::new();
    for i in 0..16 {
        points.push_str(&format!(
            "<particle position='0 {:.17} 0.61' mass='0.0125'/>\n",
            -0.1 + i as f64 * 0.02
        ));
    }
    sdf.replace_range(start..end, &points);
    Ok(sdf)
}
pub(super) fn run(worker: Option<&Path>, step_ns: u64, sdf_out: Option<&Path>) -> Result<()> {
    let sdf = fixture(step_ns)?;
    if let Some(p) = sdf_out {
        fs::write(p, &sdf)?;
    }
    let mut sim = setup(&sdf, worker)?;
    let mut control = worker.map(|_| setup(&sdf, None)).transpose()?;
    let mut slices = [0u64; 3];
    let mut nanos = [0u128; 3];
    let mut first = None;
    let mut total = 0.;
    let mut drift = 0f64;
    let mut loaded = 0;
    let mut first_after = None;
    let mut killed = 0;
    let mut moves = 0;
    let mut cuts = 0;
    let mut votes = Vec::new();
    for tick in 1..=1_000_000_000 / step_ns {
        let start = (tick - 1) * step_ns;
        let end = tick * step_ns;
        if worker.is_some() {
            if [200_000_000, 401_000_000, 410_000_000, 600_000_000]
                .iter()
                .any(|&n| start <= n && end > n)
            {
                let cut = sim.committed_update_info()?;
                let prev = sim.world().resource::<TrajectoryProcessPool>().owner();
                trajectory_workers::select(&mut sim, cut.epoch, cut.iteration, 1 - prev)?;
                moves += 1;
            } else if [392_000_000, 404_000_000, 417_000_000, 502_000_000]
                .iter()
                .any(|&n| start <= n && end > n)
            {
                let p = sim.world().resource::<TrajectoryProcessPool>();
                if let Some(pid) = p.pid(p.owner()) {
                    terminate_worker(pid)?;
                    killed += 1;
                }
            }
        }
        let now = Instant::now();
        sim.step_exact(1)?;
        let elapsed = now.elapsed();
        let zone = if end <= 400_000_000 {
            0
        } else if start >= 416_000_000 {
            2
        } else {
            1
        };
        let soft = deformable::bodies(&sim).next().ok_or("no cable")?.1;
        slices[zone] += u64::from(soft.last_path_substeps());
        nanos[zone] += elapsed.as_nanos();
        let imp = -soft.last_boundary_impulses()[0].linear[2];
        total += imp;
        let p = soft.particles()[0];
        if imp > 1e-12 {
            first.get_or_insert((start, end));
            loaded += 1;
        }
        if start >= 416_000_000 && first_after.is_none() {
            first_after = Some(p.velocity[2]);
        }
        if start >= 416_000_000 {
            drift = drift.max((p.velocity[2] - first_after.unwrap()).abs());
        }
        if let Some(c) = &mut control {
            c.step_exact(1)?;
            equal(&observation(&sim)?, &observation(c)?)?;
            cuts += 1;
        }
        if zone == 1 {
            votes.push(json!([
                start,
                end,
                soft.last_path_substeps(),
                imp,
                p.velocity[2]
            ]));
        }
    }
    let last = deformable::bodies(&sim).next().unwrap().1.particles()[0];
    if (total - 0.2 * last.velocity[2]).abs() > 1e-9
        || (last.velocity[2] - 7.5).abs() > 0.03
        || drift > 1e-9
    {
        return Err("pulse impulse or free-flight bound failed".into());
    }
    println!(
        "{}",
        json!({"step_ns":step_ns,"first_loaded_interval_ns":first,"contact_intervals":loaded,"physical_hit_ns":404000000,
        "analytical_departure_mps":7.5,"departure_mps":last.velocity[2],"summed_reaction_ns":total,
        "final_z_m":last.position[2],"analytical_final_z_m":0.61+(1.-0.404)*7.5,"free_speed_drift":drift,"slices_before_during_after":slices,
        "wall_ns_before_during_after":nanos,"moves":moves,"worker_sigkills":killed,"exact_native_material_cuts":cuts,"pulse":votes})
    );
    Ok(())
}
