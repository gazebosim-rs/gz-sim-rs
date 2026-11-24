// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a two-link foot with its rigid compound through a compliant landing.
#![allow(deprecated)]
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, damped_sole::last_step, load_world_with_sdf_step,
    queue_link_world_wrench,
};
use serde_json::{Value, json};
use std::{error::Error, path::Path, time::Instant};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WELD: &str = include_str!("../../../examples/worlds/local_compliant_weld.sdf");
const COMPOUND: &str = include_str!("../../../examples/worlds/local_compliant_compound.sdf");
const TICKS: usize = 720;
const DT: f64 = 0.0005;
#[derive(Clone, Copy, Debug)]
struct Sample {
    center: DVec3,
    velocity: DVec3,
    orientation: DQuat,
    spin: DVec3,
    p: DVec3,
    l: DVec3,
    j: DVec3,
    k: DVec3,
    normal: f64,
}
fn load(xml: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        xml,
        "compliant-foot.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn part(sim: &Simulation, name: &str) -> (u64, gz_physics::BodyId) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .expect("fixture link");
    (
        e.get::<EntityId>().unwrap().get(),
        e.get::<BodyHandle>().unwrap().id(),
    )
}
fn rotation_distance(a: DQuat, b: DQuat) -> f64 {
    let delta = a * b.conjugate();
    2. * DVec3::new(delta.x, delta.y, delta.z)
        .length()
        .atan2(delta.w.abs())
}
fn spin_momentum(s: BodyState, i: DVec3) -> DVec3 {
    let q = DQuat::from_array(s.orientation);
    q * (i * (q.conjugate() * DVec3::from(s.angular_velocity)))
}
fn run(xml: &str, weld: bool, path: &Path) -> Result<(Value, Vec<Sample>)> {
    let sdf = sdformat::load_str(xml, "compliant-foot.sdf", &sdformat::NoResources)?;
    let mut sim = load(xml)?;
    let lower = part(&sim, "default::boot::sole");
    let upper = weld.then(|| part(&sim, "default::boot::upper"));
    let mut samples = Vec::new();
    let mut frames = Vec::new();
    let mut costs = Vec::new();
    let (mut force_residual, mut torque_residual, mut weld_error) = (0_f64, 0_f64, 0_f64);
    let mut first = None;
    let mut substantial = None;
    let (mut weld_spin, mut weld_twist) = (0_f64, 0_f64);
    let mut normal_sum = 0.;
    let mut max_force = 0_f64;
    let (mut applied, mut applied_moment) = (DVec3::ZERO, DVec3::ZERO);
    for tick in 0..=TICKS {
        if tick > 0 {
            let last: &Sample = samples.last().unwrap();
            applied = DVec3::ZERO;
            applied_moment = DVec3::ZERO;
            if (105..=135).contains(&tick) || (360..=385).contains(&tick) {
                applied = DVec3::new(4., -1.4, 0.8);
                let twist = DVec3::new(-0.015, 0.03, 0.01);
                applied_moment = twist + (last.orientation * (DVec3::Z * 0.07)).cross(applied);
                queue_link_world_wrench(
                    &mut sim,
                    upper.unwrap_or(lower).0,
                    applied.into(),
                    if weld { twist } else { applied_moment }.into(),
                )?;
            }
            let time = Instant::now();
            if let Err(e) = sim.step_exact(1) {
                return Err(format!(
                    "weld={weld} at tick {tick}: {e}; {:?}",
                    sim.world().resource::<gz_sim_core::StepFault>().error()
                )
                .into());
            }
            costs.push(time.elapsed().as_nanos() as u64);
        }
        let b = sim.world().resource::<PhysicsBackend>();
        let foot = b.state(lower.1).unwrap();
        let q = DQuat::from_array(foot.orientation);
        let spin = DVec3::from(foot.angular_velocity);
        let center = DVec3::from(foot.position) + q * (DVec3::Z * 0.07);
        let (p, l) = if let Some((_, id)) = upper {
            let other = b.state(id).unwrap();
            let a = DVec3::from(foot.velocity);
            let u = DVec3::from(other.velocity);
            weld_error = weld_error.max(
                (DVec3::from(other.position) - DVec3::from(foot.position))
                    .distance(q * (DVec3::Z * 0.14)),
            );
            let upper_q = DQuat::from_array(other.orientation);
            weld_spin = weld_spin.max(rotation_distance(upper_q, q));
            weld_twist = weld_twist
                .max((u - a).distance(
                    spin.cross(DVec3::from(other.position) - DVec3::from(foot.position)),
                ));
            (
                a + u,
                spin_momentum(foot, DVec3::new(0.012, 0.016, 0.020))
                    + spin_momentum(other, DVec3::new(0.003, 0.004, 0.005))
                    + (DVec3::from(foot.position) - center).cross(a)
                    + (DVec3::from(other.position) - center).cross(u),
            )
        } else {
            (
                2. * DVec3::from(foot.velocity),
                spin_momentum(foot, DVec3::new(0.0248, 0.0298, 0.025)),
            )
        };
        let row = last_step(&sim);
        let (j, k, normal) = row.map_or((DVec3::ZERO, DVec3::ZERO, 0.), |r| {
            (
                r.step.impulse,
                r.step.angular_impulse,
                r.step.normal_impulse,
            )
        });
        if let Some(row) = row {
            let previous = samples.last().unwrap();
            force_residual = force_residual
                .max((p - previous.p - j - DT * (applied + DVec3::new(0., 0., -19.62))).length());
            torque_residual =
                torque_residual.max((l - previous.l - k - DT * applied_moment).length());
            if first.is_none() && normal > 0. {
                first = Some((tick, normal));
            }
            if substantial.is_none() && normal > 0.01 {
                substantial = Some((tick, normal));
            }
            max_force = max_force.max(row.step.peak_normal_force);
            normal_sum += normal;
        }
        samples.push(Sample {
            center,
            velocity: p / 2.,
            orientation: q,
            spin,
            p,
            l,
            j,
            k,
            normal,
        });
        if tick % 4 == 0 {
            frames.push(gz_sim_cli::primitive_frame(
                &sim,
                &sdf,
                "default",
                frames.len() as u64 + 1,
            )?);
        }
    }
    gz_sim_cli::export_archive(path, Default::default(), frames)?;
    costs.sort_unstable();
    Ok((
        json!({"first_contact":first,"first_above_0_01_ns":substantial,"max_weld_rotation_error_rad":weld_spin,"max_weld_twist_error_m_s":weld_twist,"normal_impulse_ns":normal_sum,"peak_support_n":max_force,"max_momentum_residual_ns":force_residual,
        "max_angular_momentum_residual_nms":torque_residual,"max_weld_shape_error_m":weld_error,"p95_public_step_ns":costs[(TICKS*95).div_ceil(100)-1]}),
        samples,
    ))
}
fn record(path: &Path) -> Result<Value> {
    let (wa, w) = run(WELD, true, &path.join("welded"))?;
    let (ca, c) = run(COMPOUND, false, &path.join("compound"))?;
    let mut max = [0_f64; 9];
    for (a, b) in w.iter().zip(&c) {
        for (m, v) in max.iter_mut().zip([
            a.center.distance(b.center),
            a.velocity.distance(b.velocity),
            a.spin.distance(b.spin),
            a.p.distance(b.p),
            a.l.distance(b.l),
            a.j.distance(b.j),
            a.k.distance(b.k),
            (a.normal - b.normal).abs(),
            rotation_distance(a.orientation, b.orientation),
        ]) {
            *m = m.max(v);
        }
    }
    Ok(
        json!({"dt_ns":500000,"ticks":TICKS,"welded":wa,"compound":ca,"max_center_m":max[0],"max_velocity_m_s":max[1],"max_spin_rad_s":max[2],
        "max_linear_momentum_ns":max[3],"max_angular_momentum_nms":max[4],"max_contact_impulse_ns":max[5],"max_contact_angular_impulse_nms":max[6],"max_normal_impulse_ns":max[7],"max_rotation_rad":max[8]}),
    )
}
fn main() -> Result<()> {
    let dir = std::env::args_os()
        .nth(1)
        .ok_or("supply a fresh output directory")?;
    let path = Path::new(&dir);
    std::fs::create_dir(path)?;
    let report = serde_json::to_string_pretty(&record(path)?)?;
    std::fs::write(path.join("comparison.json"), &report)?;
    println!("{report}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_rejected_physical_step_or_archive_write_stops_the_recording() -> Result<()> {
        let dir = tempfile::tempdir()?;
        assert!(run("<sdf", true, dir.path()).is_err());
        let unknown = WELD.replace("world name=\"default\"", "world name=\"other\"");
        assert!(run(&unknown, true, dir.path()).is_err());
        let crossing = WELD
            .replace("0 0 0.13 0.04 -0.06 0.12", "-0.43 0 0.13 0 0 0")
            .replace("0.035 -0.018 -0.24", "20 0 0")
            .replace("0 0 0.14 0 0 0", "0 0 -0.13 0 0 0");
        let err = run(&crossing, true, dir.path()).unwrap_err().to_string();
        assert!(err.contains("tick 1") && err.contains("upper foot collider"));
        std::fs::write(dir.path().join("compound"), "reserved")?;
        assert!(record(dir.path()).is_err());
        assert!(dir.path().join("welded/manifest.json").is_file());
        Ok(())
    }
    #[test]
    fn native_weld_matches_its_compound_through_airborne_shove_and_loaded_landing() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let r = record(dir.path())?;
        for name in ["welded", "compound"] {
            let b = &r[name];
            assert_eq!(b["first_contact"][0], 229);
            assert!(b["normal_impulse_ns"].as_f64().unwrap() > 7.);
            assert!(b["peak_support_n"].as_f64().unwrap() > 250.);
            for key in [
                "max_momentum_residual_ns",
                "max_angular_momentum_residual_nms",
                "max_weld_shape_error_m",
                "max_weld_twist_error_m_s",
                "max_weld_rotation_error_rad",
            ] {
                assert!(b[key].as_f64().unwrap() < 1e-11, "{key} {b}");
            }
        }
        for key in [
            "max_center_m",
            "max_rotation_rad",
            "max_linear_momentum_ns",
            "max_angular_momentum_nms",
            "max_normal_impulse_ns",
            "max_contact_impulse_ns",
            "max_contact_angular_impulse_nms",
        ] {
            assert!(r[key].as_f64().unwrap() < 1e-10, "{key}: {r}");
        }
        Ok(())
    }
}
