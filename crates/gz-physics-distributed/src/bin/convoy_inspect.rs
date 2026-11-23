// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read-only inspection and comparison of published two-sphere checkpoints.
#[path = "convoy_inspect/compare.rs"]
mod compare;
#[path = "convoy_inspect/series.rs"]
mod series;
use gz_physics_distributed::multi_sphere::inspect_published_path;
use std::{env, error::Error, ffi::OsStr, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const USAGE: &str = "usage: convoy_inspect ARCHIVE | --compare REFERENCE CANDIDATE | --series MANIFEST.json
The manifest is an ordered JSON array of {reference, candidate} path pairs relative to the manifest.";

fn main() -> Result<()> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let json = match args.as_slice() {
        [flag] if flag == OsStr::new("--help") || flag == OsStr::new("-h") => {
            println!("{USAGE}");
            return Ok(());
        }
        [archive] => serde_json::to_string_pretty(&inspect_published_path(Path::new(archive))?)?,
        [flag, a, b] if flag == OsStr::new("--compare") => {
            let a = inspect_published_path(Path::new(a))?;
            let b = inspect_published_path(Path::new(b))?;
            serde_json::to_string_pretty(&compare::Comparison::new(&a, &b)?)?
        }
        [flag, manifest] if flag == OsStr::new("--series") => {
            serde_json::to_string_pretty(&series::compare_series(Path::new(manifest))?)?
        }
        _ => return Err(USAGE.into()),
    };
    println!("{json}");
    Ok(())
}
