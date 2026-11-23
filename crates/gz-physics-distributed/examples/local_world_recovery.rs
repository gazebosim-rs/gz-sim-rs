// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify and replay four loaded native worlds across worker migration and restart.
#[path = "support/world_profile_scene.rs"]
mod fixture;
#[path = "support/local_world_plot.rs"]
mod plot;
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
#[path = "support/local_world_recovery.rs"]
mod trial;
use clap::Parser;
use primitive_replay_view::View;
use serde_json::json;
use std::{
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// New output directory for the trajectory, checkpoint, qualification and gz-web-rs replay.
    directory: PathBuf,
}

fn render(f: &trial::Frame<'_>) -> (String, String) {
    let mut view = View::default();
    view.cuboid(
        2,
        "ground",
        [0., 0., -0.03],
        [16., 16., 0.06],
        [0.28, 0.33, 0.39, 1.],
    );
    for (b, &owner) in f.bodies.iter().zip(&f.owners) {
        let id = 10 + 2 * (b.id + 8 * u64::from(owner));
        let color = if owner == 0 {
            [0.11, 0.48, 0.85, 1.]
        } else {
            [1., 0.55, 0.12, 1.]
        };
        view.add(
            id,
            &format!("body_{}_worker_{owner}", b.id),
            b.position,
            b.orientation,
            json!({"kind":"sphere","radius":b.size[0] * 0.5}),
            color,
        );
    }
    // Height above the floor encodes the last 10 ms contact impulse in N·s.
    // The two pointers share one scale; display only the settled interval.
    if f.tick >= trial::SETTLED {
        for (column, (pair, floor)) in f.pairs.iter().zip(&f.floors).enumerate() {
            let [x, y, _] = f.bodies.iter().find(|b| b.id == pair.a).unwrap().position;
            let id = 64 + column as u64 * 6;
            view.cuboid(
                id,
                &format!("stack_{}_impulse_scale_0_to_0.25_Ns", column + 1),
                [x + 1.0, y, 1.0],
                [0.08, 0.08, 2.0],
                [0.40, 0.44, 0.47, 1.],
            );
            view.cuboid(
                id + 2,
                &format!("stack_{}_pair_impulse", column + 1),
                [x + 1.0, y, 8.0 * pair.normal_impulse],
                [0.48, 0.18, 0.08],
                [0.85, 0.34, 0.22, 1.],
            );
            view.cuboid(
                id + 4,
                &format!("stack_{}_ground_impulse", column + 1),
                [x + 1.0, y, 8.0 * floor.normal_impulse],
                [0.48, 0.18, 0.08],
                [0.65, 0.36, 0.82, 1.],
            );
        }
    }
    // Commit-time witnesses appear only after the corresponding operation has succeeded.
    for (i, (at, label)) in [
        (trial::HANDOFF, "loaded_handoff"),
        (trial::RETRY, "retry_committed"),
        (trial::REPLACEMENT, "worker_recovered"),
        (trial::RESTORE, "archive_restored"),
    ]
    .into_iter()
    .enumerate()
    {
        if f.tick >= at {
            view.cuboid(
                50 + i as u64 * 2,
                &format!("{label}_tick_{at}"),
                [-1.5 + i as f64, -3.5, 0.24],
                [0.65, 0.65, 0.48],
                [0.16, 0.73, 0.48, 1.],
            );
        }
    }
    view.finish(f.tick)
}

fn main() -> trial::Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.directory)?;
    let mut trace = BufWriter::new(File::create_new(args.directory.join("distributed.jsonl"))?);
    let mut stream = Vec::new();
    let mut points = Vec::new();
    let report = trial::qualify(
        &args.worker,
        &args.directory.join("native.gzpool"),
        fixture::stacks(),
        0,
        |frame| {
            serde_json::to_writer(&mut trace, frame)?;
            writeln!(trace)?;
            stream.push(render(frame));
            points.push(plot::Point::from_frame(frame));
            Ok(())
        },
    )?;
    trace.flush()?;
    gz_sim_cli::export_archive(&args.directory.join("replay"), Default::default(), stream)?;
    plot::write(&args.directory.join("contact-continuity.svg"), &points)?;
    let summary = serde_json::to_string_pretty(&report)?;
    fs::write(args.directory.join("qualification.json"), &summary)?;
    println!("{summary}");
    Ok(())
}
