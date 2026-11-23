// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Observe rotating offset frames, an IMU and editable runtime models over reset.
#[path = "support/fleet_visual.rs"]
mod visual;
use bevy_ecs::prelude::{ChildOf, Entity};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    EntityKind, ImuSample, LocalPose, ModelState, Name, PhysicsEngine, Velocity, WorldPose,
    load_world_with_sdf_step, prepare_model_template, queue_link_world_velocity, remove_model,
    reset_loaded_world, set_model_state, spawn_model_template,
};
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, path::PathBuf};
use visual::{box_visual, catalog};
const XML: &str = r#"<sdf version="1.12"><world name="default"><physics type="ignored" name="clock"><max_step_size>0.01</max_step_size></physics><gravity>0 0 0</gravity>
<model name="arm"><pose>-1.5 0 0.65 0 0 0</pose><link name="body"><pose>0.55 0 0 0 0 0.31</pose><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name="shape"><geometry><box><size>0.2 0.2 0.15</size></box></geometry></collision><visual name="arm"><pose>0.55 0 0 0 0 0</pose><geometry><box><size>1.3 0.22 0.16</size></box></geometry></visual><sensor name="tip_imu" type="imu"><pose>0 0 0 0 0 0.22</pose><always_on>true</always_on><update_rate>100</update_rate><imu/></sensor></link></model>
<model name="bar"><pose>1.3 0 0.2 0 0 0</pose><link name="body"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name="shape"><geometry><box><size>0.9 0.31 0.19</size></box></geometry></collision><visual name="bar"><pose>0.18 0 0.1 0 0 0.28</pose><geometry><box><size>0.9 0.31 0.19</size></box></geometry></visual></link></model></world></sdf>"#;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn id(sim: &Simulation, n: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|v| v.0 == n))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn pose_bits(p: WorldPose) -> Value {
    json!([
        p.translation.map(f64::to_bits),
        p.rotation.map(f64::to_bits)
    ])
}
fn frame(sim: &Simulation, ordinal: usize) -> (String, String, Value) {
    let mut visuals = BTreeMap::new();
    let mut models = BTreeMap::<u64, (String, Vec<u64>)>::new();
    let mut poses = BTreeMap::new();
    let mut imu = Value::Null;
    let w = sim.world();
    let model_of = |mut e: Entity| {
        while w.get::<EntityKind>(e) != Some(&EntityKind::Model) {
            e = w.get::<ChildOf>(e).unwrap().parent();
        }
        (
            w.get::<EntityId>(e).unwrap().get(),
            w.get::<Name>(e).unwrap().0.clone(),
        )
    };
    for row in w.iter_entities() {
        let (Some(entity_id), Some(name), Some(kind), Some(world), Some(local)) = (
            row.get::<EntityId>(),
            row.get::<Name>(),
            row.get::<EntityKind>(),
            row.get::<WorldPose>(),
            row.get::<LocalPose>(),
        ) else {
            continue;
        };
        let key = entity_id.get();
        let twist = row
            .get::<Velocity>()
            .map(|v| (v.linear.map(f64::to_bits), v.angular.map(f64::to_bits)));
        poses.insert(
            key,
            json!([name.0, pose_bits(*world), pose_bits(local.0), twist]),
        );
        if let Some(sample) = row.get::<ImuSample>() {
            let reading = sample.reading.as_ref().unwrap();
            imu = json!([
                sample.epoch,
                sample.time.as_nanos() as u64,
                reading.orientation.map(f64::to_bits),
                reading.specific_force.map(f64::to_bits),
                reading.angular_velocity.map(f64::to_bits)
            ]);
        }
        let (size, color, role) = match (*kind, name.0.as_str()) {
            (EntityKind::Visual, "default::arm::body::arm") => {
                ([1.3, 0.22, 0.16], [0.055, 0.78, 0.68], "visual")
            }
            (EntityKind::Visual, n) if n.starts_with("default::runtime_") => {
                ([0.9, 0.31, 0.19], [0.94, 0.43, 0.10], "visual")
            }
            (EntityKind::Visual, _) => ([0.9, 0.31, 0.19], [0.18, 0.51, 0.93], "visual"),
            (EntityKind::Model, "default::arm") => ([0.14; 3], [0.8, 0.82, 0.90], "model_origin"),
            (EntityKind::Link, "default::arm::body") => {
                ([0.18; 3], [0.17, 0.58, 0.83], "link_origin")
            }
            (EntityKind::Sensor, _) => ([0.22; 3], [0.98, 0.74, 0.12], "imu"),
            _ => continue,
        };
        let (model, model_name) = model_of(row.id());
        let visual_id = 1000 + key;
        visuals.insert(
            visual_id,
            box_visual(
                visual_id,
                &format!("{model_name}::{role}"),
                world.translation,
                world.rotation,
                size,
                color,
            ),
        );
        models
            .entry(model)
            .or_insert_with(|| (model_name, vec![]))
            .1
            .push(visual_id);
    }
    let t = sim.elapsed();
    let epoch = sim.epoch();
    let nodes: Vec<_> = models
        .into_iter()
        .map(|(id, (name, mut ids))| {
            ids.sort_unstable();
            catalog(id, &name, &ids)
        })
        .collect();
    let world_id = id(sim, "default");
    let scene = json!({"version":1,"kind":"scene","world":"default","world_id":world_id,"epoch":epoch,"sequence":ordinal,"seconds":t.as_secs(),"nanoseconds":t.subsec_nanos(),"paused":false,"visuals":visuals.into_values().collect::<Vec<_>>()});
    let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":world_id,"epoch":epoch,"scene_sequence":ordinal,"templates":[],"models":nodes});
    (
        scene.to_string(),
        catalog.to_string(),
        json!({"epoch":epoch,"time_ns":t.as_nanos() as u64,"entities":poses,"imu":imu}),
    )
}
fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1);
    let dir = PathBuf::from(
        args.next()
            .ok_or("usage: hierarchy_lifecycle REPLAY_DIRECTORY TRACE_JSON")?,
    );
    let trace = PathBuf::from(args.next().ok_or("missing trace path")?);
    if args.next().is_some() {
        return Err("too many arguments".into());
    }
    let parsed = sdformat::load_str(XML, "hierarchy.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        XML,
        "hierarchy.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )?;
    let template = prepare_model_template(
        &sim,
        &parsed,
        parsed.worlds[0]
            .models
            .iter()
            .find(|m| m.name == "bar")
            .unwrap(),
    )?;
    let source = id(&sim, "default::bar");
    let original = entity_by_id(sim.world(), source).unwrap();
    let arm = id(&sim, "default::arm::body");
    let mut clone = 0;
    let mut frames = Vec::new();
    let mut snapshots = Vec::new();
    for tick in 0..=140 {
        if tick == 1 || tick == 111 {
            queue_link_world_velocity(&mut sim, arm, [0.22, 0., 0.], [0., 0., 2.7])?;
        }
        if tick == 40 {
            clone = spawn_model_template(
                &mut sim,
                &template,
                WorldPose {
                    translation: [0.9, 1.2, 0.3],
                    rotation: [0., 0., 0.6, 0.8],
                },
            )?;
        }
        if tick == 62 {
            set_model_state(
                &mut sim,
                source,
                ModelState {
                    translation: [1.1, -0.9, 0.1],
                    rotation: [0., 0., 0.6, 0.8],
                    linear_velocity: [-0.4, 0., 0.],
                    angular_velocity: [0., 0., -1.4],
                },
            )?;
        }
        if tick == 85 {
            remove_model(&mut sim, source)?;
            assert!(entity_by_id(sim.world(), source).is_none());
        }
        if tick == 110 {
            reset_loaded_world(&mut sim, |_| ())?;
            assert_ne!(entity_by_id(sim.world(), source).unwrap(), original);
            assert!(entity_by_id(sim.world(), clone).is_none());
        } else if tick != 0 {
            sim.step_exact(1)?;
        }
        let (scene, catalog, snapshot) = frame(&sim, frames.len() + 1);
        snapshots.push(snapshot);
        if tick % 4 == 0 || matches!(tick, 62 | 85 | 110 | 140) {
            frames.push((scene, catalog));
        }
    }
    gz_sim_cli::export_archive(&dir, Default::default(), frames)?;
    std::fs::write(trace, serde_json::to_vec(&snapshots)?)?;
    println!(
        "141 complete public frames including pose, velocity and IMU bits; two epochs; loaded ID {source} restored; runtime ID {clone} retired."
    );
    Ok(())
}
