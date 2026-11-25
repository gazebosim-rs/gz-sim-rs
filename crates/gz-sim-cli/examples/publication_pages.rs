// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure four discrete trigger volumes and ordinary Bevy observations on a prescribed SDF fleet.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/fleet_observation.rs"]
mod fleet;
#[allow(dead_code)]
#[path = "../../gz-physics-distributed/examples/support/primitive_replay_view.rs"]
mod primitive_replay_view;

use bevy_ecs::prelude::*;
use clap::{Parser, ValueEnum};
use gz_physics::{BodyId, PhysicsBackend};
use gz_sim_core::{EntityId, Phase, UpdateInfo};
use gz_sim_scene::{BodyHandle, ImuSample, LocalPose, Name, Velocity, WorldPose};
use serde_json::json;
use std::{error::Error, fs, hint::black_box, path::PathBuf, time::Instant};

const GATES: [[f64; 2]; 4] = [[0.7, 0.], [-0.2, 4.], [1.0, 8.], [0.25, 10.]];
const HALF: [f64; 3] = [0.05, 0.40, 0.15];
#[derive(Clone, Copy, ValueEnum)]
enum Reader {
    Sparse,
    Full,
}
#[derive(Parser)]
struct Args {
    #[arg(long, default_value_t = 100_000)]
    decks: usize,
    #[arg(long, default_value_t = 240)]
    ticks: usize,
    #[arg(long, value_enum, default_value = "sparse")]
    reader: Reader,
    #[arg(long)]
    replay: Option<PathBuf>,
}

#[derive(Resource, Default)]
struct GateReadings {
    members: [Vec<BodyId>; 4],
    calls: u64,
    transitions: Vec<(u64, usize, bool)>,
    elapsed_ns: u64,
}

fn sample_gates(
    info: Res<UpdateInfo>,
    backend: Res<PhysicsBackend>,
    mut sensor: ResMut<GateReadings>,
) {
    let start = Instant::now();
    for (i, [x, y]) in GATES.into_iter().enumerate() {
        let hits = backend
            .bodies_overlapping_aabb(
                [x - HALF[0], y - HALF[1], -HALF[2]],
                [x + HALF[0], y + HALF[1], HALF[2]],
            )
            .expect("finite registered trigger");
        if sensor.members[i] != hits {
            sensor
                .transitions
                .push((info.iteration, i, !hits.is_empty()));
        }
        sensor.members[i] = hits;
    }
    sensor.calls += 1;
    sensor.elapsed_ns = start.elapsed().as_nanos() as u64;
}

fn record(tick: usize, decks: &[[f64; 3]; 3], sensor: &GateReadings) -> (String, String) {
    let mut view = primitive_replay_view::View::default();
    for (i, p) in decks.iter().enumerate() {
        view.cuboid(
            10 + 2 * i as u64,
            &format!("deck_{i}"),
            *p,
            [1., 1., 0.2],
            [0.26, 0.57, 0.69, 1.],
        );
    }
    for (i, [x, y]) in GATES.into_iter().enumerate() {
        view.cuboid(
            20 + 4 * i as u64,
            &format!("gate_{i}"),
            [x, y, 0.],
            HALF.map(|v| 2. * v),
            [0.92, 0.66, 0.19, 1.],
        );
        let z = if sensor.members[i].is_empty() {
            -0.32
        } else {
            0.44
        };
        view.cuboid(
            22 + 4 * i as u64,
            &format!("occupied_{i}"),
            [x, y, z],
            [0.12; 3],
            [0.30, 0.93, 0.40, 1.],
        );
    }
    view.finish(tick as u64)
}

mod pages {
    include!(
        r"/home/nkoenig/work/gazebo/rust/gz-sim-rs-local-manipulation-integration/artifacts/local-kinematic-publication/pages.rs"
    );
}
fn run(args: Args) -> Result<(), Box<dyn Error>> {
    if args.decks < 3 || args.ticks == 0 {
        return Err("at least three decks and one measured tick are required".into());
    }
    let last_tick = args
        .ticks
        .checked_add(20)
        .ok_or("tick count overflows the warm-up interval")?;
    let mut sim = fleet::scene(args.decks, false)?;
    sim.world_mut().init_resource::<GateReadings>();
    sim.add_systems(Phase::Publish, sample_gates);
    let ids = [0, 1, 2].map(|n| {
        let name = format!("default::deck_{n}::body");
        let e = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|v| v.0 == name))
            .unwrap();
        (
            e.id(),
            e.get::<EntityId>().unwrap().get(),
            e.get::<BodyHandle>().unwrap().id(),
        )
    });
    let imu = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::deck_0::body::imu")
        })
        .unwrap()
        .id();
    let mut poses = sim
        .world_mut()
        .query::<(&EntityId, &WorldPose, &LocalPose)>();
    let mut twists = sim.world_mut().query::<&Velocity>();
    let mut times = Vec::new();
    let mut first_step = [0; 2];
    let mut frames = Vec::new();
    let mut trace = Vec::new();
    if let Some(out) = &args.replay {
        fs::create_dir(out)?;
    }
    let first = usize::from(args.replay.is_none());
    for tick in first..=last_tick {
        let audit = matches!(tick, 21 | 60 | 141 | 180);
        let before = audit.then(|| pages::snapshot(sim.world()));
        if let Some(before) = &before {
            pages::clear();
            let no_step = pages::snapshot(sim.world());
            let quiet = pages::compare(before, &no_step);
            eprintln!("PAGES {} quiet {}", tick, quiet);
            pages::clear();
        }
        let start = Instant::now();
        if tick == 0 {
            sim.set_paused(true);
            sim.tick()?;
            sim.set_paused(false);
        } else {
            sim.step_exact(1)?;
        }
        if let Some(before) = &before {
            let after = pages::snapshot(sim.world());
            eprintln!("PAGES {} step {}", tick, pages::compare(before, &after));
        }
        let world = sim.world();
        let readout = world.resource::<GateReadings>();
        let state = ids.map(|(entity, id, _)| {
            let e = world.entity(entity);
            assert_eq!(e.get::<EntityId>().unwrap().get(), id);
            assert_eq!(e.get::<Velocity>().unwrap().linear, [0.25, 0., 0.]);
            black_box(e.get::<LocalPose>().unwrap());
            e.get::<WorldPose>().unwrap().translation
        });
        let time = sim.elapsed();
        let x = 0.25 * time.as_secs_f64();
        for (i, [gate_x, gate_y]) in GATES.into_iter().enumerate() {
            let expected: Vec<_> = ids
                .iter()
                .enumerate()
                .filter(|(j, _)| {
                    let y = 4. * (*j as f64);
                    x - 0.5 <= gate_x + HALF[0]
                        && x + 0.5 >= gate_x - HALF[0]
                        && y - 0.5 <= gate_y + HALF[1]
                        && y + 0.5 >= gate_y - HALF[1]
                })
                .map(|(_, (_, _, id))| *id)
                .collect();
            assert_eq!(readout.members[i], expected);
        }
        if tick != 0 {
            let sample = world.get::<ImuSample>(imu).unwrap();
            assert_eq!(sample.time, time);
            assert_eq!(
                sample.reading.as_ref().unwrap().specific_force,
                [0., 0., 9.81]
            );
        }
        let native = world.resource::<PhysicsBackend>();
        let ray = native
            .cast_ray([x + 0.35, 0., 1.], [0., 0., -1.], 2., None)?
            .unwrap();
        assert!((ray - 0.9).abs() < 1e-12);
        if matches!(args.reader, Reader::Full) {
            for (id, w, l) in poses.iter(world) {
                black_box((id.get(), *w, *l));
            }
            for v in twists.iter(world) {
                black_box(*v);
            }
        }
        let [links, models, attachments] = gz_sim_scene::measured_fleet_stages();
        let elapsed = [
            start.elapsed().as_nanos() as u64,
            readout.elapsed_ns,
            links,
            models,
            attachments,
        ];
        if tick == 1 {
            first_step = [elapsed[0], elapsed[1]];
        }
        if tick > 20 {
            times.push(elapsed);
        }
        if args.replay.is_some() {
            frames.push(record(tick, &state, readout));
            trace.push(json!({"tick":tick,"time_ns":time.as_nanos(),"decks":state,"occupied":readout.members.each_ref().map(|v| !v.is_empty())}));
        }
    }
    let sensor = sim.world().resource::<GateReadings>();
    if let Some(out) = args.replay {
        gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
        fs::write(out.join("trace.json"), serde_json::to_vec(&trace)?)?;
    }
    println!(
        "{}",
        json!({"decks":args.decks,"step_ns":10_000_000,"warmup":20,"poses":poses.iter(sim.world()).count(),"twists":twists.iter(sim.world()).count(),"calls":sensor.calls,"transitions":sensor.transitions,"sample_columns":["total_ns","four_gates_ns","link_states_ns","model_and_local_ns","attached_poses_ns"],"first_advancing_step":first_step,"samples":times})
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    run(Args::parse())
}

#[cfg(test)]
mod tests {
    use super::*;
    use gz_sim_scene::{queue_link_world_velocity, reset_loaded_world};

    #[test]
    fn cross_a_gate_then_change_velocity_while_paused_and_repeat_after_reset() {
        let mut sim = fleet::scene(256, false).unwrap();
        sim.world_mut().init_resource::<GateReadings>();
        sim.add_systems(Phase::Publish, sample_gates);
        let id = sim
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
        sim.step_exact(100).unwrap();
        let before = sim.world().resource::<GateReadings>().transitions.clone();
        assert!(!sim.world().resource::<GateReadings>().members[0].is_empty());
        sim.set_paused(true);
        queue_link_world_velocity(&mut sim, id, [-1., 0., 0.], [0.; 3]).unwrap();
        sim.tick().unwrap();
        assert_eq!(sim.world().resource::<GateReadings>().transitions, before);
        sim.step_exact(20).unwrap();
        assert!(sim.world().resource::<GateReadings>().members[0].is_empty());
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        *sim.world_mut().resource_mut::<GateReadings>() = GateReadings::default();
        sim.step_exact(100).unwrap();
        assert_eq!(sim.world().resource::<GateReadings>().transitions, before);
    }

    #[test]
    fn recorded_callbacks_include_the_initial_occupancy_and_both_clocked_crossings() {
        let tmp = tempfile::tempdir().unwrap();
        let output = tmp.path().join("run");
        run(Args {
            decks: 3,
            ticks: 160,
            reader: Reader::Full,
            replay: Some(output.clone()),
        })
        .unwrap();
        let frames: serde_json::Value =
            serde_json::from_slice(&fs::read(output.join("trace.json")).unwrap()).unwrap();
        let rows = frames.as_array().unwrap();
        assert_eq!(rows.len(), 181);
        for (tick, occupancy) in [
            (0, [false, true, false, false]),
            (60, [true, true, false, false]),
            (141, [true, false, false, false]),
            (180, [true, false, true, false]),
        ] {
            assert_eq!(rows[tick]["time_ns"], tick as u64 * 10_000_000);
            assert_eq!(rows[tick]["occupied"], serde_json::json!(occupancy));
        }
        run(Args {
            decks: 3,
            ticks: 1,
            reader: Reader::Sparse,
            replay: None,
        })
        .unwrap();
    }

    #[test]
    fn invalid_sizes_and_existing_output_are_refused() {
        for (decks, ticks) in [(2, 1), (3, 0), (3, usize::MAX)] {
            assert!(
                run(Args {
                    decks,
                    ticks,
                    reader: Reader::Sparse,
                    replay: None
                })
                .is_err()
            );
        }
        assert!(
            run(Args {
                decks: 3,
                ticks: 1,
                reader: Reader::Full,
                replay: Some(std::env::current_dir().unwrap())
            })
            .is_err()
        );
    }
}
