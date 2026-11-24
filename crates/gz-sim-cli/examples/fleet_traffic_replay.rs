// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Play the measured contact and retirement history of a cargo transfer.
#[path = "support/fleet_visual.rs"]
mod visual;
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::EntityId;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step, queue_link_world_wrench,
};
use serde_json::json;
use std::{error::Error, fmt::Write, path::PathBuf};
use visual::{box_visual, catalog};

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args_os().skip(1);
    let dir = PathBuf::from(
        args.next()
            .ok_or("usage: fleet_traffic_replay ARCHIVE TRAJECTORY_JSON")?,
    );
    let trace = PathBuf::from(args.next().ok_or("missing trajectory JSON")?);
    if args.next().is_some() {
        return Err("unexpected arguments".into());
    }
    let mut xml = String::from(
        "<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity><physics type='ignored' name='clock'><max_step_size>0.01</max_step_size></physics>",
    );
    for i in 0..256 {
        write!(
            xml,
            "<model name='deck_{i}'><pose>{} 0 0 0 0 0</pose><plugin name='gz::sim::systems::PrescribedMotion' filename='gz-sim-prescribed-motion-system'><linear_velocity>0.25 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin><link name='body'><collision name='shape'><geometry><box><size>1 1 0.2</size></box></geometry></collision></link></model>",
            (f64::from(i) - 2.) * 1.1
        )?;
    }
    xml.push_str("<model name='cargo'><pose>0 0 0.201 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.006666666666666667</ixx><iyy>0.006666666666666667</iyy><izz>0.006666666666666667</izz></inertia></inertial><collision name='shape'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision></link></model></world></sdf>");
    let mut sim = load_world_with_sdf_step(
        &xml,
        "traffic.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut actors = Vec::new();
    for n in std::iter::once("cargo".to_owned()).chain((0..8).map(|i| format!("deck_{i}"))) {
        let name = format!("default::{n}");
        let full = format!("{name}::body");
        let e = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == full))
            .unwrap();
        actors.push((
            name,
            e.id(),
            e.get::<EntityId>().unwrap().get(),
            e.get::<BodyHandle>().unwrap().id(),
        ));
    }
    let mut frames = Vec::new();
    let mut samples = Vec::new();
    let mut last = None;
    for tick in 0..=260 {
        if matches!(tick, 26 | 126) {
            let force = if tick == 26 {
                [510., 0., 185.]
            } else {
                [-620., 0., 157.]
            };
            queue_link_world_wrench(&mut sim, actors[0].2, force, [0.; 3])?;
        }
        if tick > 0 {
            sim.step_exact(1)?;
        }
        let backend = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let stats = backend.fleet_isolation_stats();
        let pairs = backend.rigid_contact_observations();
        let visible: Vec<_> = actors[1..]
            .iter()
            .map(|a| backend.fleet_deck_is_native(a.3).unwrap_or(false))
            .collect();
        let touching: Vec<_> = actors[1..]
            .iter()
            .enumerate()
            .filter_map(|(i, a)| {
                pairs
                    .iter()
                    .any(|p| p.active && (p.a == a.3 || p.b == a.3))
                    .then_some(i)
            })
            .collect();
        let signature = (visible.clone(), touching.clone());
        if tick % 4 != 0 && last.as_ref() == Some(&signature) && !matches!(tick, 26 | 126) {
            continue;
        }
        last = Some(signature);
        let mut visuals = Vec::new();
        let mut models = Vec::new();
        for (i, (name, e, _, _)) in actors.iter().enumerate() {
            let p = sim.world().get::<WorldPose>(*e).unwrap();
            let [x, y, z] = p.translation;
            let [qx, qy, qz, qw] = p.rotation;
            let s = std::f64::consts::FRAC_1_SQRT_2;
            let (color, size) = if i == 0 {
                ([0.99, 0.49, 0.08], [0.2; 3])
            } else {
                ([0.22, 0.40, 0.62], [1., 1., 0.2])
            };
            let id = 10 * (i as u64 + 1);
            visuals.push(box_visual(
                id + 1,
                &format!("{name}::body"),
                [y, -x, z],
                [s * (qx + qy), s * (qy - qx), s * (qz - qw), s * (qw + qz)],
                size,
                color,
            ));
            if i == 0 {
                models.push(catalog(id, name, &[id + 1]));
            } else {
                // The immutable strip is parked inside the deck while its
                // contact geometry is analytic, then exposed on the front edge.
                let strip_height = if visible[i - 1] { 0.106 } else { 0. };
                visuals.push(box_visual(
                    id + 2,
                    &format!("{name}::native_marker"),
                    [y - 0.42, -x, z + strip_height],
                    [0., 0., -s, s],
                    [0.94, 0.12, 0.01],
                    [0.07, 0.89, 0.67],
                ));
                models.push(catalog(id, name, &[id + 1, id + 2]));
            }
        }
        let seq = frames.len() + 1;
        let elapsed = sim.elapsed();
        let scene=json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":seq,"seconds":elapsed.as_secs(),"nanoseconds":elapsed.subsec_nanos(),"paused":false,"visuals":visuals}).to_string();
        let tree=json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":seq,"templates":[],"models":models}).to_string();
        frames.push((scene, tree));
        samples.push(json!({"tick":tick,"native":visible,"contacts":touching,"retired":stats.map_or(0,|s|s.retired_decks),"cargo_position":sim.world().get::<WorldPose>(actors[0].1).unwrap().translation}));
    }
    assert!(samples.last().unwrap()["retired"].as_u64().unwrap() > 0);
    gz_sim_cli::export_archive(&dir, Default::default(), frames)?;
    std::fs::write(trace, serde_json::to_vec_pretty(&samples)?)?;
    println!(
        "{} native scene frames, final retirement count {}",
        samples.len(),
        samples.last().unwrap()["retired"]
    );
    Ok(())
}
