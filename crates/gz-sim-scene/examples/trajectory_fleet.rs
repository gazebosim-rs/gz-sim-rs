// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure a whole SDF/Bevy publication of independent nonlinear shuttles.
#[path = "support/trajectory_fleet_view.rs"]
mod view;
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{
    BodyHandle, ImuSample, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
};
use serde_json::json;
use std::{error::Error, fmt::Write, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let n = args
        .next()
        .ok_or("trajectory_fleet DECKS TICKS analytic|native")?
        .parse::<usize>()?;
    let ticks = args.next().ok_or("ticks")?.parse::<u32>()?;
    let layout = args.next().ok_or("layout")?;
    let capture_file = args.next().map(std::path::PathBuf::from);
    if !matches!(layout.as_str(), "native" | "analytic")
        || n == 0
        || n > 100000
        || ticks < 20
        || args.next().is_some()
    {
        return Err("invalid scenario".into());
    }
    let mut xml = String::from(
        "<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity><physics name='fixed' type='ignored'><max_step_size>0.005</max_step_size></physics>",
    );
    // The distant fixed witness makes Rapier solve the world at every curve substep without
    // changing a shuttle's motion or creating a contact. Both paths publish the same population.
    if layout == "native" {
        xml.push_str("<model name='witness'><static>true</static><pose>1000 1000 -500 0 0 0</pose><link name='body'><collision name='gap'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model>");
    }
    let scale = |i: usize| 0.125 + (i % 67) as f64 / 128.;
    for i in 0..n {
        write!(
            xml,
            "<model name='shuttle_{i}'><pose>{} {} 0 0 0 0</pose><link name='body'>",
            (i % 317) * 2,
            (i / 317) * 2
        )?;
        if i == 0 {
            xml.push_str("<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>200</update_rate><imu/></sensor>");
        }
        xml.push_str("<visual name='visual'><geometry><box><size>1 1 0.2</size></box></geometry></visual><collision name='solid'><geometry><box><size>1 1 0.2</size></box></geometry></collision></link>");
        let s = scale(i);
        write!(
            xml,
            "<plugin name='gz::sim::systems::PrescribedMotion' filename='gz-sim-prescribed-motion-system'><trajectory end='repeat'><waypoint time_ns='0' displacement='0 0 0' velocity='0 0 0'/><waypoint time_ns='500000000' displacement='{s} {} {}' velocity='0 0 0'/><waypoint time_ns='1000000000' displacement='0 0 0' velocity='0 0 0'/></trajectory></plugin></model>",
            s * 0.5,
            s * 0.25
        )?;
    }
    xml.push_str("</world></sdf>");
    let load = Instant::now();
    let mut sim = load_world_with_sdf_step(
        &xml,
        "cubic-fleet.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let load_s = load.elapsed().as_secs_f64();
    drop(xml);
    let mut measured = Vec::new();
    let mut max_error = 0f64;
    let mut max_imu_error = 0f64;
    let mut checked = 0;
    let mut native_substeps = 0;
    let mut fingerprint = 0xcbf29ce484222325u64;
    let expected = |i: usize, tick: u32| {
        let phase = (tick % 200) as f64 / 100.;
        let (u, sgn) = if phase < 1. {
            (phase, 1.)
        } else {
            (2. - phase, -1.)
        };
        let p = u * u * (3. - 2. * u);
        let v = sgn * 12. * u * (1. - u);
        let factor = [scale(i), 0.5 * scale(i), 0.25 * scale(i)];
        let origin = [((i % 317) * 2) as f64, ((i / 317) * 2) as f64, 0.];
        (
            std::array::from_fn::<_, 3, _>(|a| origin[a] + p * factor[a]),
            factor.map(|a| v * a),
        )
    };
    let mut capture = capture_file
        .as_ref()
        .map(|_| view::Capture::new(&sim, n))
        .transpose()?;
    if let Some(c) = &mut capture {
        c.sample(&sim)?;
    }
    for t in 1..=ticks {
        let start = Instant::now();
        sim.step_exact(1)?;
        let used = start.elapsed();
        if t > 10 {
            measured.push(used.as_nanos() as u64);
        }
        if t % 2 == 0
            && let Some(c) = &mut capture
        {
            c.sample(&sim)?;
        }
        let resource = sim.world().resource::<PhysicsBackend>();
        let (solves, hits) = resource
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .translation_interval();
        assert!(hits.is_empty());
        assert_eq!(solves, if layout == "native" { 10 } else { 0 });
        native_substeps += solves;
        if (t - 1) % 25 != 0 && t != ticks {
            continue;
        }
        let mut observed_imu = false;
        for e in sim.world().iter_entities() {
            let Some(name) = e
                .get::<Name>()
                .and_then(|n| n.0.strip_prefix("default::shuttle_"))
            else {
                continue;
            };
            let i = name.split("::").next().unwrap().parse::<usize>()?;
            let (p, v) = expected(i, t);
            if let Some(pose) = e.get::<WorldPose>() {
                for (a, b) in pose.translation.into_iter().zip(p) {
                    max_error = max_error.max((a - b).abs());
                    fingerprint = (fingerprint ^ a.to_bits()).wrapping_mul(0x100000001b3);
                }
                checked += 1;
            }
            if let Some(h) = e.get::<BodyHandle>() {
                let s = resource.state(h.id()).unwrap();
                assert_eq!(e.get::<Velocity>().unwrap().linear, s.velocity);
                for (a, b) in s.velocity.into_iter().zip(v) {
                    max_error = max_error.max((a - b).abs());
                }
            }
            if let Some(imu) = e.get::<ImuSample>() {
                assert_eq!(imu.time, sim.elapsed());
                let a = imu.reading.as_ref().unwrap().specific_force;
                let (_, before) = expected(0, t - 1);
                if t > 1 {
                    for k in 0..3 {
                        max_imu_error = max_imu_error
                            .max((a[k] - (v[k] - before[k]) / 0.005 - [0., 0., 9.81][k]).abs());
                    }
                }
                observed_imu = true;
            }
        }
        assert!(observed_imu);
    }
    if max_error > 2e-12 || max_imu_error > 5e-10 {
        return Err(format!("pose/velocity error {max_error}, IMU error {max_imu_error}").into());
    }
    measured.sort_unstable();
    let q = |v: usize| measured[(measured.len() * v).div_ceil(100) - 1] / 1000;
    println!(
        "{}",
        json!({"layout":layout,"decks":n,"public_ticks":ticks,"native_substeps":native_substeps,"poses_checked":checked,"pose_digest_fnv":format!("{fingerprint:016x}"),"max_pose_or_velocity_error":max_error,"max_imu_error_mps2":max_imu_error,"load_s":load_s,"p50_us":q(50usize),"p95_us":q(95),"p99_us":q(99),"rtf":0.005*measured.len() as f64/(measured.iter().sum::<u64>() as f64*1e-9)})
    );
    if let Some((c, f)) = capture.zip(capture_file) {
        c.write(&f)?;
    }
    Ok(())
}
