// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native playback of a loaded 256-deck scene, a commanded finite body and an
//! automatically activated contact. Also writes the actual per-frame solver counts.
#[path = "support/fleet_visual.rs"]
mod visual;
use bevy_ecs::prelude::Entity;
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, WorldPose, load_world_with_sdf_step, queue_link_world_wrench,
};
use serde_json::json;
use std::{error::Error, fmt::Write, path::PathBuf};
use visual::{box_visual, catalog};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn load() -> Result<Simulation> {
    let mut xml = String::from(
        "<sdf version='1.12'><world name='default'><physics type='ignored' name='default'><max_step_size>0.01</max_step_size></physics><gravity>0 0 0</gravity>",
    );
    for i in 0..256 {
        write!(
            xml,
            "<model name='deck_{i}'><pose>{} 0 0 0 0 0</pose><plugin name='gz::sim::systems::PrescribedMotion' filename='gz-sim-prescribed-motion-system'><linear_velocity>0.25 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin><link name='body'><collision name='shape'><geometry><box><size>1 0.8 0.8</size></box></geometry></collision></link></model>",
            2 * i
        )?;
    }
    xml.push_str("<model name='payload'><pose>-7 0 0 0 0 0</pose><link name='body'><inertial><mass>1</mass></inertial><collision name='shape'><geometry><box><size>0.35 0.35 0.35</size></box></geometry></collision></link></model></world></sdf>");
    Ok(load_world_with_sdf_step(
        &xml,
        "fleet-dispatch.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn link(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1);
    let dir = PathBuf::from(
        args.next()
            .ok_or("usage: fleet_dispatch REPLAY_DIRECTORY TRACE_JSON")?,
    );
    let trace = PathBuf::from(args.next().ok_or("missing output trace")?);
    if args.next().is_some() {
        return Err("unexpected arguments".into());
    }
    let mut sim = load()?;
    let links = [
        "default::payload::body",
        "default::deck_0::body",
        "default::deck_1::body",
    ]
    .map(|name| link(&sim, name));
    let payload_id = sim.world().get::<EntityId>(links[0]).unwrap().get();
    let mut frames = Vec::new();
    let mut samples = Vec::new();
    let mut earliest_guard = None;
    let mut earliest_contact = None;
    let mut peak_impulse = 0.0f64;
    for tick in 0..=180 {
        if tick != 0 {
            if tick == 16 {
                queue_link_world_wrench(&mut sim, payload_id, [500., 0., 0.], [0.; 3])?;
            }
            sim.step_exact(1)?;
        }
        let backend = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let native = backend
            .fleet_isolation_stats()
            .map_or(0, |s| s.native_decks);
        let contacts = backend.active_contact_pairs();
        let impulse = backend
            .rigid_contact_observations()
            .iter()
            .map(|r| r.normal_impulse)
            .sum::<f64>();
        peak_impulse = peak_impulse.max(impulse);
        if native > 0 && earliest_guard.is_none() {
            earliest_guard = Some(tick);
        }
        if contacts > 0 && earliest_contact.is_none() {
            earliest_contact = Some(tick);
        }
        if tick % 4 != 0
            && Some(tick) != earliest_guard
            && Some(tick) != earliest_contact
            && tick != 15
        {
            continue;
        }
        let sequence = frames.len() + 1;
        let mut visuals = vec![box_visual(
            3,
            "default::rail::guide",
            [0., 2.25, -0.50],
            [0., 0., 0., 1.],
            [0.05, 11.8, 0.04],
            [0.26, 0.32, 0.38],
        )];
        let mut models = vec![catalog(2, "default::rail", &[3])];
        for (i, name) in ["payload", "target_deck", "neighbor_deck"]
            .into_iter()
            .enumerate()
        {
            let pose = sim.world().get::<WorldPose>(links[i]).unwrap();
            let [x, y, z] = pose.translation;
            let [qx, qy, qz, qw] = pose.rotation;
            let s = std::f64::consts::FRAC_1_SQRT_2;
            let model = 10 * (i as u64 + 1);
            let color = [[0.97, 0.59, 0.12], [0.06, 0.80, 0.69], [0.24, 0.51, 0.84]][i];
            let size = if i == 0 { [0.35; 3] } else { [1., 0.8, 0.8] };
            visuals.push(box_visual(
                model + 1,
                &format!("default::{name}::body"),
                [y, -x, z],
                [s * (qx + qy), s * (qy - qx), s * (qz - qw), s * (qw + qz)],
                size,
                color,
            ));
            models.push(catalog(model, &format!("default::{name}"), &[model + 1]));
        }
        let t = sim.elapsed();
        let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":sequence,"seconds":t.as_secs(),"nanoseconds":t.subsec_nanos(),"paused":false,"visuals":visuals});
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"templates":[],"models":models});
        frames.push((scene.to_string(), catalog.to_string()));
        samples.push(json!({"tick":tick,"native_decks":native,"contacts":contacts,"normal_impulse_ns":impulse}));
    }
    assert!(earliest_guard.unwrap() < earliest_contact.unwrap() && peak_impulse > 0.1);
    gz_sim_cli::export_archive(&dir, Default::default(), frames)?;
    let result = json!({"decks":256,"first_guard_tick":earliest_guard,"first_contact_tick":earliest_contact,"peak_normal_impulse_ns":peak_impulse,"samples":samples});
    std::fs::write(trace, serde_json::to_vec_pretty(&result)?)?;
    println!(
        "first guard tick {:?}, contact tick {:?}, peak normal impulse {peak_impulse:.6} N s",
        earliest_guard, earliest_contact
    );
    Ok(())
}
