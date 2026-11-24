// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native replay of actual SDF cloth edges, with interpolated display samples on each edge.
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, deformable::bodies, load_world_with_sdf_step};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::Path};

const SDF: &str = include_str!("../../../examples/worlds/local_cloth_edge_strike.sdf");
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn export(out: &Path, source: &str) -> Result<()> {
    let sdf = sdformat::load_str(source, "edge.sdf", &NoResources)?;
    let mut sim = load_world_with_sdf_step(
        source,
        "edge.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut frames = vec![];
    let mut trace = vec![];
    for tick in 0..=90 {
        if tick > 0 {
            sim.step_exact(1)?;
        }
        let (scene, catalog) = gz_sim_cli::primitive_frame(&sim, &sdf, "default", 1 + tick)?;
        let mut scene: Value = serde_json::from_str(&scene)?;
        let mut catalog: Value = serde_json::from_str(&catalog)?;
        let visuals = scene["visuals"]
            .as_array_mut()
            .ok_or("missing visual list")?;
        let models = catalog["models"]
            .as_array_mut()
            .ok_or("missing model catalog")?;
        let mut next = visuals
            .iter()
            .map(|v| v["id"].as_u64().unwrap())
            .max()
            .unwrap()
            + 1;
        let mut cases = BTreeMap::new();
        for (name, material) in bodies(&sim) {
            let p = material.particles();
            let prefix = format!("default::{name}::ridge::");
            let model = models
                .iter_mut()
                .find(|m| m["name"] == format!("default::{name}"))
                .ok_or("missing display model")?;
            let style = visuals
                .iter()
                .find(|v| v["name"] == format!("{prefix}p0"))
                .ok_or("missing physical node color")?["color"]
                .clone();
            for (e, (a, b)) in [(0, 1), (1, 3), (3, 2), (2, 0), (0, 3)]
                .into_iter()
                .enumerate()
            {
                for k in 1..8 {
                    let pos =
                        DVec3::from(p[a].position).lerp(p[b].position.into(), f64::from(k) / 8.);
                    visuals.push(json!({"id":next,"name":format!("{prefix}edge{e}_{k}"),"translation":pos.to_array(),
                        "rotation":[0.,0.,0.,1.],"shape":{"kind":"sphere","radius":0.0025},"color":style}));
                    model["visual_ids"]
                        .as_array_mut()
                        .unwrap()
                        .push(json!(next));
                    next += 1;
                }
            }
            cases.insert(name,json!({"positions_m":p.iter().map(|p|p.position).collect::<Vec<_>>(),
                "velocities_m_s":p.iter().map(|p|p.velocity).collect::<Vec<_>>(),
                "impulses":material.last_boundary_impulses().iter().map(|r|json!({"linear_ns":r.linear,"angular_nms":r.angular})).collect::<Vec<_>>()}));
        }
        frames.push((scene.to_string(), catalog.to_string()));
        trace.push(json!({"time_ns":tick*1_000_000,"cases":cases}));
    }
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    serde_json::to_writer(fs::File::create(out.join("trace.json"))?, &trace)?;
    Ok(())
}
fn main() -> Result<()> {
    let out = std::env::args_os()
        .nth(1)
        .ok_or("expected a new output directory")?;
    let out = Path::new(&out);
    if out.try_exists()? {
        return Err("choose a new output directory".into());
    }
    let parent = out
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::tempdir_in(parent)?;
    export(stage.path(), SDF)?;
    fs::rename(stage.path(), out)?;
    Ok(())
}

#[cfg(test)]
#[path = "cloth_edge_strike/validation.rs"]
mod validation;
