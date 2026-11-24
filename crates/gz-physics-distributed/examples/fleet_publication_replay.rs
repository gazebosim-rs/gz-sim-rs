// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record a real patch of the 10,000-deck end-to-end benchmark in the native viewer.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/fleet_observation.rs"]
mod fleet;
#[allow(dead_code)]
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
use clap::Parser;
use gz_sim_scene::{Name, WorldPose};
use primitive_replay_view::View;
use serde_json::json;
use std::{error::Error, fs, path::PathBuf};
#[derive(Parser)]
struct Args {
    #[arg(long, default_value_t=10000, value_parser=clap::value_parser!(u32).range(9..=100000))]
    decks: u32,
    /// Launch the first payload with a one-tick native wrench at 1.20 s.
    #[arg(long)]
    launch: bool,
    /// Display the origin and first solid hit of a downward beam following deck 0.
    #[arg(long)]
    range: bool,
    output: PathBuf,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let out = args.output;
    fs::create_dir(&out)?;
    let count = args.decks as usize;
    let columns = count.isqrt() + usize::from(count.isqrt().pow(2) != count);
    let mut sim = fleet::scene(count, true)?;
    let first_payload = sim
        .world()
        .iter_entities()
        .find_map(|entity| {
            (entity.get::<Name>()?.0 == "default::payload_0::body")
                .then(|| entity.get::<gz_sim_core::EntityId>().unwrap().get())
        })
        .expect("the fleet authors its first payload");
    let visible_columns = if args.launch { columns.min(4) } else { 3 };
    let mut selected = Vec::new();
    for row in 0..3 {
        for col in 0..visible_columns {
            let i = row * columns + col;
            if i >= count {
                continue;
            }
            for payload in [false, true] {
                if payload && i % 100 != 0 {
                    continue;
                }
                let model = format!("{}_{i}", if payload { "payload" } else { "deck" });
                let leaf = format!("default::{model}::body::visual");
                let entity = sim
                    .world()
                    .iter_entities()
                    .find(|e| e.get::<Name>().is_some_and(|n| n.0 == leaf))
                    .ok_or_else(|| format!("missing {leaf}"))?
                    .id();
                selected.push((entity, model, payload));
            }
        }
    }
    let mut frames = Vec::new();
    let mut samples = Vec::new();
    let last_tick = if args.launch { 220 } else { 200 };
    for tick in 0..=last_tick {
        if args.launch && tick == 121 {
            gz_sim_scene::queue_link_world_wrench(
                &mut sim,
                first_payload,
                [510., 0., 185.],
                [0.; 3],
            )?;
        }
        if tick > 0 {
            sim.step_exact(1)?;
        }
        if tick % 4 != 0 && !(args.launch && (120..=180).contains(&tick)) {
            continue;
        }
        let mut view = View::default();
        let mut beam = None;
        let mut poses = Vec::new();
        for (i, (entity, name, payload)) in selected.iter().enumerate() {
            let pose = sim
                .world()
                .get::<WorldPose>(*entity)
                .ok_or("missing actual fleet visual pose")?;
            let [x, y, z] = pose.translation;
            if !payload {
                let n: usize = name.strip_prefix("deck_").unwrap().parse()?;
                let expected = (n % columns) as f64 * 1.1 + 0.0025 * tick as f64;
                if (x - expected).abs() > 1e-9 || (y - (n / columns) as f64 * 1.1).abs() > 1e-9 {
                    return Err("deck left commanded closed-form path".into());
                }
            }
            if args.range && name == "deck_0" {
                let origin = [x, y, 1.];
                let distance = sim
                    .world()
                    .resource::<gz_physics::PhysicsBackend>()
                    .cast_ray(origin, [0., 0., -1.], 2., None)?
                    .ok_or("expected a downward return from the moving deck")?;
                let hit = [x, y, 1. - distance];
                view.cuboid(
                    102,
                    "range_origin",
                    [x - 1.1, y - 1.1, 1.],
                    [0.045; 3],
                    [0.98, 0.82, 0.18, 1.],
                );
                view.cuboid(
                    104,
                    "range_hit",
                    [x - 1.1, y - 1.1, hit[2]],
                    [0.055; 3],
                    [0.20, 1., 0.36, 1.],
                );
                beam = Some(json!({"origin":origin, "hit":hit, "distance_m":distance}));
            }
            let (size, color) = if *payload {
                ([0.2; 3], [0.99, 0.56, 0.08, 1.])
            } else {
                ([1., 1., 0.2], [0.13, 0.53, 0.72, 1.])
            };
            view.add(
                10 + i as u64 * 2,
                name,
                [x - 1.1, y - 1.1, z],
                pose.rotation,
                json!({"kind":"box","size":size}),
                color,
            );
            poses.push(
                json!({"name":name,"world_position_m":pose.translation,"rotation":pose.rotation}),
            );
        }
        view.cuboid(
            100,
            "stationary_origin_marker",
            [-1.1, -1.77, 0.0],
            [0.025, 0.17, 0.13],
            [0.84, 0.22, 0.22, 1.],
        );
        let imu = if tick == 0 {
            None
        } else {
            Some(
                fleet::sample(&sim)
                    .reading
                    .as_ref()
                    .map_err(|_| "invalid scheduled IMU")?
                    .specific_force,
            )
        };
        let mut sample = json!({"tick":tick,"poses":poses,"imu_specific_force_mps2":imu});
        if let Some(beam) = beam {
            sample["range"] = beam;
        }
        samples.push(sample);
        frames.push(view.finish_at(frames.len() as u64 + 1, tick * 10_000_000));
    }
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    fs::write(
        out.join("index.json"),
        serde_json::to_vec(
            &json!({"decks":count,"payloads":count.div_ceil(100),"public_step_ns":10000000,"display_origin_m":[1.1,1.1,0.],"frames":samples}),
        )?,
    )?;
    Ok(())
}
