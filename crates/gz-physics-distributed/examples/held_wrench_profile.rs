// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Matched disk capture/restore of one native cut with 0, 3 and 32 held contributions.
#[path = "../tests/private/passive_pose_proof.rs"]
mod passive_pose_proof;
use clap::{Parser, Subcommand, ValueEnum};
use gz_physics::PhysicsBackend;
use gz_physics_distributed::SceneBackend;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, passive_checkpoint::PassivePoolScene};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const SDF: &str = include_str!("../../../examples/distributed_stack.sdf");
#[derive(Parser)]
struct Cli {
    #[command(subcommand)]
    mode: Mode,
}
#[derive(Subcommand)]
enum Mode {
    /// Run one independent measurement process per arm per counterbalanced block.
    Campaign {
        #[arg(long)]
        worker: PathBuf,
        /// Repeat each of the six arm orders this many times.
        #[arg(long,default_value_t=12,value_parser=clap::value_parser!(u8).range(1..=100))]
        rounds: u8,
        output: PathBuf,
    },
    #[command(hide = true)]
    Sample {
        worker: PathBuf,
        seed: PathBuf,
        case: Case,
    },
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, ValueEnum)]
#[serde(rename_all = "lowercase")]
enum Case {
    Zero,
    Typical,
    Maximum,
}
impl Case {
    fn index(self) -> usize {
        match self {
            Self::Zero => 0,
            Self::Typical => 1,
            Self::Maximum => 2,
        }
    }
    fn count(self) -> usize {
        [0, 3, 32][self.index()]
    }
    fn cli(self) -> &'static str {
        ["zero", "typical", "maximum"][self.index()]
    }
}
fn options(worker: &Path) -> WorldLoadOptions<'_> {
    WorldLoadOptions {
        engine: PhysicsEngine::DistributedIslands(4),
        distributed_worker: Some(worker),
        step: None,
        noise_seed: [0; 32],
    }
}
fn resume(worker: &Path, path: &Path) -> Result<PassivePoolScene> {
    Ok(PassivePoolScene::resume(
        SDF,
        "stack.sdf",
        "default",
        options(worker),
        Default::default(),
        path,
    )?)
}
fn add(s: &mut PassivePoolScene, c: Case) -> Result<()> {
    let divisions = match c {
        Case::Zero => return Ok(()),
        Case::Typical => [1u8; 3],
        Case::Maximum => [16, 8, 8],
    };
    for ((link, force, torque), pieces) in [
        (11, [160., 0., 0.], [0., 8., 0.]),
        (7, [0., 0., -0.125], [0.; 3]),
        (11, [240., 0., 0.], [0., 16., 0.]),
    ]
    .into_iter()
    .zip(divisions)
    {
        let n = f64::from(pieces);
        for _ in 0..pieces {
            s.queue_wrench(link, force.map(|v| v / n), torque.map(|v| v / n))?;
        }
    }
    Ok(())
}
fn sha(bytes: impl AsRef<[u8]>) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn hex(hash: [u8; 32]) -> String {
    hash.iter().map(|b| format!("{b:02x}")).collect()
}
#[derive(Serialize, Deserialize)]
struct Row {
    case: Case,
    capture_ns: u64,
    restore_ns: u64,
    bytes: usize,
    metadata_bytes: usize,
    native_sha256: String,
    cut_sha256: String,
    following_sha256: String,
}
fn sample(worker: &Path, seed: &Path, case: Case) -> Result<Row> {
    let scratch = tempfile::tempdir()?;
    let file = scratch.path().join("sample");
    let mut s = resume(worker, seed)?;
    add(&mut s, case)?;
    let before = passive_pose_proof::proof(s.simulation());
    let start = Instant::now();
    s.save(&file)?;
    let capture_ns = start.elapsed().as_nanos().try_into()?;
    drop(s);
    let start = Instant::now();
    let mut r = resume(worker, &file)?;
    let restore_ns = start.elapsed().as_nanos().try_into()?;
    let backend = r
        .simulation()
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap();
    if backend.held_scene_wrenches().len() != case.count()
        || passive_pose_proof::proof(r.simulation()) != before
    {
        return Err("source and restored cut or contribution count differ".into());
    }
    r.step_exact(1)?;
    let following_sha256 = hex(passive_pose_proof::proof(r.simulation()));
    let bytes = fs::read(file)?;
    let metadata_bytes = u32::from_le_bytes(bytes[40..44].try_into()?) as usize;
    Ok(Row {
        case,
        capture_ns,
        restore_ns,
        bytes: bytes.len(),
        metadata_bytes,
        native_sha256: sha(&bytes[44 + metadata_bytes..]),
        cut_sha256: hex(before),
        following_sha256,
    })
}
fn quantile(sorted: &[i64], num: usize, den: usize) -> i64 {
    sorted[(sorted.len() * num).div_ceil(den).saturating_sub(1)]
}
#[derive(Serialize)]
struct Timing {
    median_ns: i64,
    p95_ns: i64,
}
fn timing(mut v: Vec<i64>) -> Timing {
    v.sort_unstable();
    Timing {
        median_ns: quantile(&v, 1, 2),
        p95_ns: quantile(&v, 95, 100),
    }
}
#[derive(Serialize)]
struct Delta {
    median_ns: i64,
    median_interval95_ns: [i64; 2],
    p95_ns: i64,
}
fn paired(deltas: Vec<i64>) -> Delta {
    let mut seed = 0xe21c_94b7_9138_675du64;
    let mut trials = Vec::with_capacity(4096);
    let mut sample = Vec::with_capacity(deltas.len());
    for _ in 0..4096 {
        sample.clear();
        for _ in 0..deltas.len() {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            sample.push(deltas[((u128::from(seed) * deltas.len() as u128) >> 64) as usize]);
        }
        sample.sort_unstable();
        trials.push(quantile(&sample, 1, 2));
    }
    trials.sort_unstable();
    let t = timing(deltas);
    Delta {
        median_ns: t.median_ns,
        median_interval95_ns: [quantile(&trials, 25, 1000), quantile(&trials, 975, 1000)],
        p95_ns: t.p95_ns,
    }
}
#[derive(Serialize)]
struct Summary {
    case: Case,
    n: usize,
    bytes: usize,
    metadata_bytes: usize,
    capture: Timing,
    restore: Timing,
    paired_capture: Delta,
    paired_restore: Delta,
}
#[derive(Serialize)]
struct Campaign {
    worker_sha256: String,
    measurement_sha256: String,
    native_sha256: String,
    cut_sha256: String,
    summaries: Vec<Summary>,
    blocks: Vec<Vec<Row>>,
}
fn campaign(worker: PathBuf, rounds: u8, output: PathBuf) -> Result<()> {
    use Case::{Maximum as M, Typical as T, Zero as Z};
    let scratch = tempfile::tempdir()?;
    let seed = scratch.path().join("single-native-cut");
    let worker = worker.canonicalize()?;
    let mut s = PassivePoolScene::load(
        SDF,
        "stack.sdf",
        "default",
        options(&worker),
        Default::default(),
    )?;
    s.step_exact(110)?;
    s.queue_impulse(11, [0.25, 0., 0.])?;
    s.save(&seed)?;
    drop(s);
    let exec = std::env::current_exe()?;
    let mut blocks = Vec::new();
    let mut next: [Option<String>; 3] = Default::default();
    let mut native = None;
    let mut cut = None;
    for _ in 0..rounds {
        for order in [
            [Z, T, M],
            [M, T, Z],
            [T, M, Z],
            [Z, M, T],
            [M, Z, T],
            [T, Z, M],
        ] {
            let mut block = Vec::new();
            for case in order {
                let cmd = Command::new(&exec)
                    .arg("sample")
                    .arg(&worker)
                    .arg(&seed)
                    .arg(case.cli())
                    .output()?;
                if !cmd.status.success() {
                    return Err(String::from_utf8_lossy(&cmd.stderr).into_owned().into());
                }
                let r: Row = serde_json::from_slice(&cmd.stdout)?;
                if r.case != case
                    || next[case.index()].get_or_insert_with(|| r.following_sha256.clone())
                        != &r.following_sha256
                    || native.get_or_insert_with(|| r.native_sha256.clone()) != &r.native_sha256
                    || cut.get_or_insert_with(|| r.cut_sha256.clone()) != &r.cut_sha256
                {
                    return Err("independent samples did not hold the native cut and following trajectory constant".into());
                }
                block.push(r);
            }
            blocks.push(block);
        }
    }
    if next[0] == next[1] || next[1] != next[2] {
        return Err("equivalent held forces diverged or failed to act".into());
    }
    let mut summaries = Vec::new();
    for case in [Z, T, M] {
        let mut captures = Vec::new();
        let mut restores = Vec::new();
        let mut dc = Vec::new();
        let mut dr = Vec::new();
        let mut size = None;
        for block in &blocks {
            let baseline = block.iter().find(|r| r.case == Z).unwrap();
            let r = block.iter().find(|r| r.case == case).unwrap();
            if size.get_or_insert((r.bytes, r.metadata_bytes)) != &(r.bytes, r.metadata_bytes) {
                return Err("archive length varied within arm".into());
            }
            captures.push(r.capture_ns as i64);
            restores.push(r.restore_ns as i64);
            dc.push(r.capture_ns as i64 - baseline.capture_ns as i64);
            dr.push(r.restore_ns as i64 - baseline.restore_ns as i64);
        }
        let (bytes, metadata_bytes) = size.unwrap();
        summaries.push(Summary {
            case,
            n: blocks.len(),
            bytes,
            metadata_bytes,
            capture: timing(captures),
            restore: timing(restores),
            paired_capture: paired(dc),
            paired_restore: paired(dr),
        });
    }
    let result = Campaign {
        worker_sha256: sha(fs::read(worker)?),
        measurement_sha256: sha(fs::read(exec)?),
        native_sha256: native.unwrap(),
        cut_sha256: cut.unwrap(),
        summaries,
        blocks,
    };
    println!("{}", serde_json::to_string_pretty(&result.summaries)?);
    fs::write(output, serde_json::to_vec_pretty(&result)?)?;
    Ok(())
}
fn main() -> Result<()> {
    match Cli::parse().mode {
        Mode::Sample { worker, seed, case } => {
            println!("{}", serde_json::to_string(&sample(&worker, &seed, case)?)?)
        }
        Mode::Campaign {
            worker,
            rounds,
            output,
        } => campaign(worker, rounds, output)?,
    }
    Ok(())
}
