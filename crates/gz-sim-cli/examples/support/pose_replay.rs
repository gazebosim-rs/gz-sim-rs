// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Convert measured Gazebo example visuals to the public replay format.
use serde::Deserialize;
use serde_json::{Value, json};
use std::{error::Error, fs::File, io::BufReader, path::PathBuf, time::Duration};

#[derive(Deserialize)]
struct Capture {
    geometry: Vec<Value>,
    models: Vec<Value>,
    frames: Vec<Frame>,
}
#[derive(Deserialize)]
struct Frame {
    time_ns: u64,
    poses: Vec<([f64; 3], [f64; 4])>,
}
pub fn run() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args_os().skip(1);
    let capture = PathBuf::from(args.next().ok_or("expected VISUALS_JSON NEW_ARCHIVE")?);
    let out = PathBuf::from(args.next().ok_or("missing archive path")?);
    if args.next().is_some() {
        return Err("unexpected arguments".into());
    }
    let Capture {
        geometry,
        models,
        frames,
    } = serde_json::from_reader(BufReader::new(File::open(capture)?))?;
    let mut encoded = Vec::with_capacity(frames.len());
    for (i, frame) in frames.into_iter().enumerate() {
        if frame.poses.len() != geometry.len() {
            return Err("incomplete visual pose sample".into());
        }
        let visuals: Vec<_> = geometry
            .iter()
            .cloned()
            .zip(frame.poses)
            .map(|(mut v, (p, q))| {
                v["translation"] = json!(p);
                v["rotation"] = json!(q);
                v
            })
            .collect();
        let t = Duration::from_nanos(frame.time_ns);
        let sequence = i + 1;
        let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":sequence,"seconds":t.as_secs(),"nanoseconds":t.subsec_nanos(),"paused":false,"visuals":visuals});
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"templates":[],"models":models});
        encoded.push((scene.to_string(), catalog.to_string()));
    }
    let count = encoded.len();
    gz_sim_cli::export_archive(&out, Default::default(), encoded)?;
    println!("{count} committed scene samples in {}", out.display());
    Ok(())
}
