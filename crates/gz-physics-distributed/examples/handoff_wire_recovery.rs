// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare two native handoff encodings at every barrier, including recovery from a lost prepare.
#[path = "support/changing_worlds.rs"]
mod fixture;
#[allow(dead_code)]
#[path = "support/changing_recovery.rs"]
mod trial;
use clap::{Parser, ValueEnum};
use sha2::{Digest, Sha256};
use std::{
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, value_enum)]
    layout: fixture::Layout,
    /// Compare four separate packed calls to one packed atomic batch.
    #[arg(long)]
    compare_batch: bool,
    /// Fresh destination for the common trace, checkpoints, and both encoding reports.
    directory: PathBuf,
}
fn main() -> trial::Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.directory)?;
    let mut trace = BufWriter::new(File::create_new(args.directory.join("distributed.jsonl"))?);
    let mut frames = Vec::<[u8; 32]>::new();
    let groups = fixture::seeds(args.layout);
    let names = if args.compare_batch {
        ["separate", "atomic"]
    } else {
        ["json", "packed"]
    };
    for (i, name) in names.into_iter().enumerate() {
        let revised = i == 1;
        let mode = if args.compare_batch {
            trial::SplitHandoff::CompareBatch { atomic: revised }
        } else {
            trial::SplitHandoff::Encoding(revised)
        };
        let checkpoint = args.directory.join(format!("{name}.gzpool"));
        let report = trial::qualify_scenario(
            &args.worker,
            &checkpoint,
            &groups,
            trial::Scenario {
                schedule: trial::Schedule {
                    migrate: 75,
                    retry: 120,
                    replace: 179,
                    restore: 185,
                    pad: 1,
                },
                placement: fixture::Placement::Single,
                helper_limit: None,
                split_wire: Some(mode),
            },
            |frame| {
                let data = serde_json::to_vec(frame)?;
                let hash = Sha256::digest(&data).into();
                if revised {
                    if frames.get(frame.tick as usize) != Some(&hash) {
                        return Err(format!("handoff mode changed tick {}", frame.tick).into());
                    }
                } else {
                    if frames.len() != frame.tick as usize {
                        return Err("non-contiguous capture".into());
                    }
                    frames.push(hash);
                    trace.write_all(&data)?;
                    writeln!(trace)?;
                }
                Ok(())
            },
        )?;
        if report.migrant_ids.len() != 31
            || report.wire.as_ref().is_none_or(|w| {
                w.transfers.len() != if w.atomic { 1 } else { 4 } || w.dropped_prepare == 0
            })
        {
            return Err("incomplete four-group transfer".into());
        }
        fs::write(
            args.directory.join(format!("{name}.json")),
            serde_json::to_vec_pretty(&report)?,
        )?;
    }
    trace.flush()?;
    let a = args.directory.join(format!("{}.gzpool", names[0]));
    let b = args.directory.join(format!("{}.gzpool", names[1]));
    // Atomic publication assigns fewer source verification sequence numbers, which can yield
    // distinct journal metadata. Both persisted coordinators were resumed and physics-checked.
    let same_checkpoint = fs::read(&a)? == fs::read(&b)?;
    if !args.compare_batch && !same_checkpoint {
        return Err("encoding changed persisted native state".into());
    }
    // Keep one canonical checkpoint after both separately generated files have been compared.
    if same_checkpoint {
        fs::rename(a, args.directory.join("native.gzpool"))?;
        fs::remove_file(b)?;
    }
    fs::write(
        args.directory.join("sources.json"),
        serde_json::to_vec_pretty(&serde_json::json!({
            "layout":args.layout.to_possible_value().unwrap().get_name(),"compared_frames":frames.len(),"identical_checkpoint_files":same_checkpoint,
            "trace_sha256":format!("{:x}",Sha256::digest(fs::read(args.directory.join("distributed.jsonl"))?)),
            "worker_sha256":format!("{:x}",Sha256::digest(fs::read(args.worker)?))
        }))?,
    )?;
    println!(
        "Matched all {} distributed frames, including ownership, receipts and resumed state.",
        frames.len()
    );
    Ok(())
}
