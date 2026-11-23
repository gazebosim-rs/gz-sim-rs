// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Generate reference and changed convoy cuts for the read-only comparison tool.
#[path = "support/inspection_series.rs"]
mod inspection_series;
use std::{env, error::Error, path::Path};

fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    let [worker, output] = args.as_slice() else {
        return Err("usage: convoy_inspection_series PHYSICS_WORKER NEW_OUTPUT_DIR".into());
    };
    inspection_series::generate(Path::new(worker), Path::new(output))
}
