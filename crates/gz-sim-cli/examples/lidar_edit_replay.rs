// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Export a measured scene-edit sequence and the finite returns of its onboard lidar.
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    EntityKind, LidarPublication, LidarSample, Name, PhysicsEngine, VisualGeometry, WorldPose,
    load_world_with_sdf_step, model_state, prepare_model_template, remove_model,
    reset_loaded_world, set_lidar_interest, set_model_state, spawn_model_template,
};
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, path::PathBuf};
const SCENE: &str = include_str!("../../../examples/multifidelity_lidar_edits.sdf");
// Separate presentation-only IDs from the four-node models this fixture creates.
const SCAN_MODEL: u64 = 1_000_000;
fn frame(sim: &Simulation, sequence: usize) -> (String, String) {
    let mut visuals = BTreeMap::new();
    let mut ownership = BTreeMap::<String, Vec<u64>>::new();
    for e in sim.world().iter_entities() {
        let Some(shape) = e.get::<VisualGeometry>() else {
            continue;
        };
        let name = &e.get::<Name>().unwrap().0;
        let id = e.get::<EntityId>().unwrap().get();
        let pose = e.get::<WorldPose>().unwrap();
        let model = name.split("::").take(2).collect::<Vec<_>>().join("::");
        ownership.entry(model).or_default().push(id);
        let (shape, color) = match shape.0.as_ref() {
            sdformat::Geometry::Sphere { radius } => (
                json!({"kind":"sphere","radius":radius}),
                [0.94, 0.30, 0.05, 1.],
            ),
            sdformat::Geometry::Box { size } => (
                json!({"kind":"box","size":size}),
                if name.contains("::scanner::") {
                    [0.05, 0.37, 0.83, 1.]
                } else {
                    [0.38, 0.54, 0.57, 1.]
                },
            ),
            _ => unreachable!("fixture uses only boxes and spheres"),
        };
        visuals.insert(id,json!({"id":id,"name":name,"shape":shape,"color":color,"translation":pose.translation,"rotation":pose.rotation}));
    }
    let mut models = BTreeMap::new();
    for e in sim
        .world()
        .iter_entities()
        .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Model))
    {
        let name = &e.get::<Name>().unwrap().0;
        let id = e.get::<EntityId>().unwrap().get();
        let mut ids = ownership.remove(name).unwrap();
        ids.sort_unstable();
        models.insert(id,json!({"model_id":id,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false}));
    }
    if let Some((scan, pubn)) = sim
        .world()
        .iter_entities()
        .find_map(|e| Some((e.get::<LidarSample>()?, e.get::<LidarPublication>()?)))
    {
        let origin = DVec3::from_array(scan.world_pose.translation);
        let orientation = DQuat::from_array(scan.world_pose.rotation);
        let mut ids = Vec::new();
        for (i, &r) in scan.ranges.iter().enumerate() {
            let angle = pubn.config.angle_min + i as f64 * pubn.angle_step;
            let (s, c) = angle.sin_cos();
            // Keep each beam's identity. Its marker fits entirely within the
            // opaque scanner's 0.25 m height when a beam has no return.
            let p = if r.is_finite() {
                origin + orientation * DVec3::new(c, s, 0.) * r
            } else {
                origin
            };
            let id = SCAN_MODEL + 1 + i as u64;
            ids.push(id);
            visuals.insert(id,json!({"id":id,"name":format!("default::returns::beam_{i}"),"shape":{"kind":"sphere","radius":0.022},"color":[0.1,0.99,0.55,1.],"translation":p.to_array(),"rotation":[0.,0.,0.,1.]}));
        }
        models.insert(SCAN_MODEL,json!({"model_id":SCAN_MODEL,"name":"default::returns","visual_ids":ids,"can_remove":false,"can_set_state":false}));
    }
    let time = sim.elapsed();
    let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":sim.epoch(),"sequence":sequence,"seconds":time.as_secs(),"nanoseconds":time.subsec_nanos(),"paused":false,"visuals":visuals.into_values().collect::<Vec<_>>()});
    let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":sim.epoch(),"scene_sequence":sequence,"templates":[],"models":models.into_values().collect::<Vec<_>>()});
    (scene.to_string(), catalog.to_string())
}
fn pose(x: f64) -> WorldPose {
    WorldPose {
        translation: [x, 0., 0.45],
        rotation: [0., 0., 0., 1.],
    }
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args_os().skip(1);
    let archive = PathBuf::from(args.next().ok_or("expected NEW_ARCHIVE")?);
    if args.next().is_some() {
        return Err("expected exactly one archive directory".into());
    }
    let sdf = sdformat::load_str(SCENE, "scan.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        SCENE,
        "scan.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let template = |name| {
        prepare_model_template(
            &sim,
            &sdf,
            sdf.worlds[0]
                .models
                .iter()
                .find(|m| m.name == name)
                .unwrap(),
        )
    };
    let sphere = template("inspection_sphere")?;
    let wall = template("backstop")?;
    let laser = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<LidarPublication>())
        .unwrap()
        .id();
    set_lidar_interest(&mut sim, [laser]);
    let (mut ball, mut barrier) = (None, None);
    let mut frames = vec![frame(&sim, 1)];
    let mut events = Vec::<Value>::new();
    for sample in 0..=140 {
        let action = match sample {
            15 => {
                ball = Some(spawn_model_template(&mut sim, &sphere, pose(-0.15))?);
                Some("spawn sphere")
            }
            35 => {
                let id = ball.unwrap();
                let mut s = model_state(&sim, id)?;
                s.linear_velocity = [2., 0., 0.];
                set_model_state(&mut sim, id, s)?;
                Some("give sphere 2 m/s")
            }
            83 => {
                let id = ball.unwrap();
                let s = model_state(&sim, id)?;
                assert!(s.linear_velocity[0].abs() < 0.01);
                Some("sphere stopped against backstop")
            }
            90 => {
                remove_model(&mut sim, ball.take().unwrap())?;
                Some("remove sphere")
            }
            105 => {
                barrier = Some(spawn_model_template(&mut sim, &wall, pose(0.35))?);
                Some("spawn backstop")
            }
            125 => {
                remove_model(&mut sim, barrier.take().unwrap())?;
                Some("remove backstop")
            }
            _ => None,
        };
        if let Some(action) = action {
            events.push(json!({"t":sim.elapsed().as_secs_f64(),"action":action,"sphere":ball,"backstop":barrier}));
        }
        // At sample zero the first visible scan and range landmarks arrive
        // after 20 ms. The live scene's reset test covers the empty acquisition.
        sim.step_exact(4)?;
        frames.push(frame(&sim, frames.len() + 1));
    }
    let count = frames.len();
    gz_sim_cli::export_archive(&archive, Default::default(), frames)?;
    let first = sim.world().get::<EntityId>(laser).unwrap().get();
    reset_loaded_world(&mut sim, |_| ())?;
    assert!(
        sim.world()
            .get::<LidarSample>(entity_by_id(sim.world(), first).unwrap())
            .is_none()
    );
    println!(
        "{}",
        json!({"frames":count,"events":events,"verified_reset_epoch":sim.epoch()})
    );
    Ok(())
}
