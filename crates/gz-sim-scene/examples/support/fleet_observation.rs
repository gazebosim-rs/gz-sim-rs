// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! End-to-end loaded SDF benchmark: hierarchical model/link/collision/visual poses and a real IMU.
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    ImuSample, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    queue_link_world_velocity, reset_loaded_world,
};
use std::{error::Error, fmt::Write, time::Instant};
pub(super) fn scene(count: usize, mixed: bool) -> Result<Simulation, Box<dyn Error>> {
    let columns = count.isqrt() + usize::from(count.isqrt().pow(2) != count);
    scene_with_columns(count, mixed, columns.max(1))
}
pub(super) fn scene_with_columns(
    count: usize,
    mixed: bool,
    columns: usize,
) -> Result<Simulation, Box<dyn Error>> {
    if columns == 0 {
        return Err("fleet requires at least one grid column".into());
    }
    let mut xml = String::from(
        "<sdf version='1.12'><world name='default'><physics type='ignored' name='fixed'><max_step_size>0.01</max_step_size></physics><gravity>0 0 -9.81</gravity>",
    );
    for i in 0..count {
        let [x, y] = if mixed {
            [(i % columns) as f64 * 1.1, (i / columns) as f64 * 1.1]
        } else {
            [0., 4. * i as f64]
        };
        if mixed && i % 100 == 0 {
            write!(
                xml,
                "<model name='payload_{i}'><pose>{x} {y} 0.201 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.006666666666666667</ixx><iyy>0.006666666666666667</iyy><izz>0.006666666666666667</izz></inertia></inertial><collision name='collision'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision><visual name='visual'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></visual></link></model>"
            )?;
        }
        write!(
            xml,
            "<model name='deck_{i}'><pose>{x} {y} 0 0 0 0</pose><plugin name='gz::sim::systems::PrescribedMotion' filename='gz-sim-prescribed-motion-system'><linear_velocity>0.25 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin><link name='body'><collision name='collision'><geometry><box><size>1 1 0.2</size></box></geometry></collision><visual name='visual'><geometry><box><size>1 1 0.2</size></box></geometry></visual>",
        )?;
        if i == 0 {
            xml.push_str("<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>100</update_rate><imu/></sensor>");
        }
        xml.push_str("</link></model>");
    }
    xml.push_str("</world></sdf>");
    Ok(load_world_with_sdf_step(
        &xml,
        "observed-fleet.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
pub(super) fn sample(sim: &Simulation) -> &ImuSample {
    sim.world()
        .iter_entities()
        .find_map(|e| e.get::<ImuSample>())
        .unwrap()
}
pub fn run(count: usize) -> Result<(), Box<dyn Error>> {
    if !(1..=100000).contains(&count) {
        return Err("expected 1–100000 decks".into());
    }
    let loading = Instant::now();
    let mut sim = scene(count, false)?;
    let load = loading.elapsed();
    let first = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::deck_0::body")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    sim.step_exact(20)?;
    let mut times = Vec::new();
    let start = Instant::now();
    for _ in 0..100 {
        let begin = Instant::now();
        sim.step_exact(1)?;
        times.push(begin.elapsed());
    }
    let wall = start.elapsed();
    times.sort();
    let mut links = 0;
    let mut poses = 0;
    let mut max_error = 0.0_f64;
    for entity in sim.world().iter_entities() {
        if let Some(velocity) = entity.get::<Velocity>() {
            assert_eq!(velocity.linear, [0.25, 0., 0.]);
            links += 1;
        }
        if entity
            .get::<Name>()
            .is_some_and(|n| n.0.starts_with("default::deck_"))
        {
            let pose = entity.get::<WorldPose>().unwrap();
            max_error = max_error.max((pose.translation[0] - 0.3).abs());
            poses += 1;
        }
    }
    assert_eq!(links, count);
    assert_eq!(poses, 4 * count + 1);
    assert!(max_error < 1e-12);
    assert_eq!(sample(&sim).time, sim.elapsed());
    assert_eq!(
        sample(&sim).reading.as_ref().unwrap().specific_force,
        [0., 0., 9.81]
    );
    // A zero-duration tick preserves the pending replacement. The IMU reads
    // the new world velocity on the first changing tick and its timestamp once.
    queue_link_world_velocity(&mut sim, first, [-0.25, 0., 0.], [0.; 3])?;
    let before_pause = sim.elapsed();
    let link = gz_sim_core::entity_by_id(sim.world(), first).unwrap();
    sim.set_paused(true);
    sim.tick()?;
    assert_eq!(sim.elapsed(), before_pause);
    assert_eq!(
        sim.world().get::<Velocity>(link).unwrap().linear,
        [0.25, 0., 0.]
    );
    assert_eq!(sample(&sim).time, sim.elapsed());
    sim.step_exact(1)?;
    let reading = sample(&sim).reading.as_ref().unwrap();
    assert_eq!(reading.specific_force, [0., 0., 9.81]);
    assert_eq!(
        sim.world().get::<Velocity>(link).unwrap().linear,
        [-0.25, 0., 0.]
    );
    reset_loaded_world(&mut sim, |_| ())?;
    sim.step_exact(1)?;
    let reset_link = gz_sim_core::entity_by_id(sim.world(), first).unwrap();
    assert_eq!(
        sim.world().get::<Velocity>(reset_link).unwrap().linear,
        [0.25, 0., 0.]
    );
    assert_eq!(
        sim.world()
            .get::<WorldPose>(reset_link)
            .unwrap()
            .translation,
        [0.0025, 0., 0.]
    );
    assert_eq!(sample(&sim).epoch, 1);
    assert_eq!(
        sample(&sim).reading.as_ref().unwrap().specific_force,
        [0., 0., 9.81]
    );
    println!(
        "loaded_sdf_decks={count} published_poses={poses} sensors=1 load_s={:.4} warmup=20 measured=100 dt_ms=10 wall_s={:.6} real_time_factor={:.4} p50_ms={:.4} p99_ms={:.4} max_pose_error_m={max_error:.3e} edited_velocity_x=-0.25 imu_specific_force_z=9.81",
        load.as_secs_f64(),
        wall.as_secs_f64(),
        1. / wall.as_secs_f64(),
        times[49].as_secs_f64() * 1000.,
        times[98].as_secs_f64() * 1000.
    );
    Ok(())
}

pub fn run_mixed(count: usize, automatic: bool) -> Result<(), Box<dyn Error>> {
    use gz_physics::PhysicsBackend;
    use gz_physics_multifidelity::MultiFidelityBackend;
    if !(256..=100_000).contains(&count) {
        return Err("expected 256..=100000 mixed decks".into());
    }
    let start = Instant::now();
    let mut sim = scene(count, true)?;
    let load_s = start.elapsed().as_secs_f64();
    if !automatic {
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<MultiFidelityBackend>()
            .unwrap()
            .disable_fleet_isolation();
    }
    let first = Instant::now();
    sim.step_exact(1)?;
    let first_ms = first.elapsed().as_secs_f64() * 1000.;
    sim.step_exact(19)?;
    let mut times = Vec::new();
    let start = Instant::now();
    for _ in 0..100 {
        let t = Instant::now();
        sim.step_exact(1)?;
        times.push(t.elapsed());
    }
    let wall = start.elapsed().as_secs_f64();
    times.sort_unstable();
    let mut observed = 0;
    let mut loads = 0;
    let mut max_deck_error = 0.0f64;
    let columns = count.isqrt() + usize::from(count.isqrt().pow(2) != count);
    for entity in sim.world().iter_entities() {
        let Some(Name(name)) = entity.get::<Name>() else {
            continue;
        };
        let Some(pose) = entity.get::<WorldPose>() else {
            continue;
        };
        let mut parts = name.split("::");
        let Some(model) = parts.nth(1) else { continue };
        if let Some(index) = model.strip_prefix("deck_") {
            let i: usize = index.parse()?;
            let expected = [
                (i % columns) as f64 * 1.1 + 0.3,
                (i / columns) as f64 * 1.1,
                0.,
            ];
            for (v, e) in pose.translation.into_iter().zip(expected) {
                assert!(v.is_finite());
                max_deck_error = max_deck_error.max((v - e).abs());
            }
            if let Some(v) = entity.get::<Velocity>() {
                assert_eq!(v.linear, [0.25, 0., 0.]);
            }
            observed += 1;
        } else if let Some(index) = model.strip_prefix("payload_") {
            if let Some(v) = entity.get::<Velocity>() {
                let i: usize = index.parse()?;
                assert!((v.linear[0] - 0.25).abs() < 0.01);
                assert!((pose.translation[0] - (i % columns) as f64 * 1.1 - 0.3).abs() < 0.05);
                assert!((pose.translation[2] - 0.2).abs() < 0.01);
                loads += 1;
            }
            observed += 1;
        }
    }
    assert_eq!(loads, count.div_ceil(100));
    assert_eq!(observed, 4 * (count + loads) + 1);
    assert!(max_deck_error < 1e-8);
    assert_eq!(sample(&sim).time, sim.elapsed());
    assert_eq!(
        sample(&sim).reading.as_ref().unwrap().specific_force,
        [0., 0., 9.81]
    );
    let backend = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    assert_eq!(backend.active_contact_pairs(), loads);
    let stats = backend.fleet_isolation_stats();
    if !automatic {
        assert!(stats.is_none());
    }
    println!(
        "automatic={automatic} decks={count} payloads={loads} poses={observed} sensors=1 load_s={load_s:.4} first_step_ms={first_ms:.3} wall_s={wall:.6} real_time_factor={:.5} p50_ms={:.4} p99_ms={:.4} max_deck_error_m={max_deck_error:.6e} imu_z=9.81 isolation={stats:?}",
        1. / wall,
        times[49].as_secs_f64() * 1000.,
        times[98].as_secs_f64() * 1000.
    );
    Ok(())
}
