// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Paired release trials for worker-local fine worlds and the two dense scene controls.
//! Run under taskset to give both coordinator and workers the same CPU affinity.
#[path = "support/local_world_control.rs"]
mod local_world_control;
use clap::Parser;
use local_world_control::physics_and_layout;
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    process::Command,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// Release profile directory containing the prior physics_worker and examples.
    baseline: PathBuf,
    /// Matching profile directory for the revised build.
    current: PathBuf,
    /// Write a new JSONL file, one pair per workload and round.
    output: PathBuf,
    #[arg(long, default_value_t = 8, value_parser = clap::value_parser!(u8).range(1..))]
    rounds: u8,
    /// Compare differently sized loaded worlds, then run the coarse and dense scene controls.
    #[arg(long)]
    heterogeneous: bool,
    /// Cross both coordinators with both workers in a counterbalanced four-arm block.
    #[arg(long)]
    crossed: bool,
}

#[derive(Clone, Copy)]
enum Case {
    Heterogeneous(&'static str),
    OwnerCounts(&'static str),
    Profile {
        scene: &'static str,
        mode: &'static str,
        count: usize,
        tiles: u8,
    },
    Encounter,
    LoadedSdf,
}

impl Case {
    fn run(&self, coordinator: &Path, worker_dir: &Path) -> Result<Value> {
        let worker = worker_dir.join("physics_worker");
        let examples = coordinator.join("examples");
        let mut cmd = match self {
            Case::OwnerCounts(counts) => {
                let mut c = Command::new(examples.join("native_world_profile"));
                c.args([
                    "--scene",
                    "stacks",
                    "--mode",
                    "pool",
                    "--stack-tiles",
                    "2",
                    "--owner-counts",
                    counts,
                ]);
                c
            }
            Case::Heterogeneous(layout) => {
                let mut c = Command::new(examples.join("heterogeneous_world_profile"));
                c.args(["--layout", layout]);
                c
            }
            Case::Profile {
                scene,
                mode,
                count,
                tiles,
            } => {
                let mut c = Command::new(examples.join("native_world_profile"));
                c.args(["--scene", scene, "--mode", mode, "--count"])
                    .arg(count.to_string())
                    .arg("--stack-tiles")
                    .arg(tiles.to_string());
                c
            }
            Case::Encounter => {
                let mut c = Command::new(examples.join("distributed_dense_separation"));
                c.arg("--benchmark");
                c
            }
            Case::LoadedSdf => {
                let mut c = Command::new(examples.join("distributed_loaded_stacks"));
                c.args(["--bound", "tight"]);
                c
            }
        };
        let result = cmd.arg("--worker").arg(worker).output()?;
        if !result.status.success() {
            return Err(format!(
                "{cmd:?} failed: {}",
                String::from_utf8_lossy(&result.stderr)
            )
            .into());
        }
        Ok(serde_json::from_slice(&result.stdout)?)
    }
}

fn main() -> Result<()> {
    let a = Args::parse();
    let stacks = |mode, count, tiles| Case::Profile {
        scene: "stacks",
        mode,
        count,
        tiles,
    };
    let mut cases = if a.heterogeneous {
        ["first", "middle", "last", "twins", "uniform"]
            .map(|layout| (layout, Case::Heterogeneous(layout)))
            .to_vec()
    } else {
        vec![
            ("local_four", stacks("local", 4, 1)),
            ("pool_four_one_owner", stacks("pool", 1, 1)),
            ("pool_four_two_owners", stacks("pool", 2, 1)),
            ("pool_four_four_owners", stacks("pool", 4, 1)),
            ("local_sixteen", stacks("local", 16, 4)),
            ("pool_sixteen_one_owner", stacks("pool", 1, 4)),
            ("pool_sixteen_four_owners", stacks("pool", 4, 4)),
        ]
    };
    if a.heterogeneous {
        cases.extend([
            ("owner_2_5_1", Case::OwnerCounts("2,5,1")),
            ("owner_5_2_1", Case::OwnerCounts("5,2,1")),
        ]);
    }
    cases.extend([
        (
            "coarse_one_owner",
            Case::Profile {
                scene: "fleet",
                mode: "pool",
                count: 1,
                tiles: 1,
            },
        ),
        ("dense_encounter", Case::Encounter),
        ("loaded_sdf", Case::LoadedSdf),
    ]);
    let mut output = BufWriter::new(File::create_new(a.output)?);
    let mut controls = Vec::new();
    // Round zero warms the binary/filesystem caches and captures each workload's
    // expected physics and placement. Rotate workload order, alternate AB/BA.
    for round in 0..=usize::from(a.rounds) {
        for i in 0..cases.len() {
            let index = (round + i) % cases.len();
            let (name, case) = &cases[index];
            let prior_first = (round + index).is_multiple_of(2);
            let mut run = |coordinator: &Path, worker: &Path| -> Result<Value> {
                let row = case.run(coordinator, worker)?;
                let control = physics_and_layout(&row)?;
                if index == controls.len() && round == 0 {
                    controls.push(control.clone());
                }
                if controls[index] != control {
                    return Err(format!("physics/layout mismatch in {name}, round {round}").into());
                }
                Ok(row)
            };
            let row = if a.crossed {
                // Each row of this Williams design uses every position once.
                // Across four rounds every distinct ordered predecessor occurs once.
                let order =
                    [[0, 1, 3, 2], [1, 2, 0, 3], [2, 3, 1, 0], [3, 0, 2, 1]][(round + index) % 4];
                let profiles = [&a.baseline, &a.current];
                let names = ["old_old", "old_new", "new_old", "new_new"];
                let mut arms = serde_json::Map::new();
                for arm in order {
                    arms.insert(
                        names[arm].to_owned(),
                        run(profiles[arm / 2], profiles[arm % 2])?,
                    );
                }
                json!({"workload":name,"round":round,"order":order.map(|i|names[i]),"arms":arms})
            } else {
                let (prior, current) = if prior_first {
                    (run(&a.baseline, &a.baseline)?, run(&a.current, &a.current)?)
                } else {
                    let current = run(&a.current, &a.current)?;
                    (run(&a.baseline, &a.baseline)?, current)
                };
                json!({"workload":name,"round":round,"baseline_first":prior_first,"baseline":prior,"current":current})
            };
            if round > 0 {
                serde_json::to_writer(&mut output, &row)?;
                writeln!(output)?;
            }
        }
        output.flush()?;
    }
    Ok(())
}
