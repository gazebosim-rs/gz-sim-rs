// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run self-contained physical worker fixtures from the exact headless Cargo graph.
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    error::Error,
    fs::{self, File},
    io,
    path::{Path, PathBuf},
    process::Command,
    time::Instant,
};

#[path = "support/curved_checkpoint.rs"]
mod curved_checkpoint;
#[path = "support/loaded_grippers.rs"]
mod loaded_grippers;
#[path = "support/production_example.rs"]
mod production_example;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const CASES: &str = include_str!("support/headless-worker-cases.json");
const WORKER: &str = "gz-physics-distributed/src/bin/physics_worker.rs";

fn executable(artifacts: &[Value], source: &Path, target: &Path, test: bool) -> Result<PathBuf> {
    let matches: BTreeSet<_> = artifacts
        .iter()
        .filter(|a| {
            a["target"]["src_path"].as_str().map(Path::new) == Some(source)
                && a["profile"]["test"].as_bool() == Some(test)
        })
        .filter_map(|a| a["executable"].as_str().map(PathBuf::from))
        .collect();
    if matches.len() != 1 {
        return Err(format!(
            "expected one Cargo executable for {source:?}, found {}",
            matches.len()
        )
        .into());
    }
    let path = matches.into_iter().next().unwrap();
    if path.is_symlink() || !path.is_file() || !path.canonicalize()?.starts_with(target) {
        return Err(format!("invalid headless Cargo executable {path:?}").into());
    }
    Ok(path)
}

fn sha256(path: &Path) -> Result<String> {
    let mut hash = Sha256::new();
    io::copy(&mut File::open(path)?, &mut hash)?;
    Ok(format!("{:x}", hash.finalize()))
}

fn all_parents_passed(output: &str, count: usize) -> bool {
    let expected = format!("test result: ok. {count} passed; 0 failed; 0 ignored;");
    // Recovery children can print earlier test summaries into the same stream.
    output
        .lines()
        .rev()
        .find(|l| l.starts_with("test result: "))
        .is_some_and(|summary| summary.starts_with(&expected))
}

struct Inputs {
    directory: PathBuf,
    root: PathBuf,
    target: PathBuf,
    worker: PathBuf,
    artifacts: Vec<Value>,
}

impl Inputs {
    fn load() -> Result<Self> {
        let directory =
            PathBuf::from(env::var_os("GZ_HEADLESS_EVIDENCE").ok_or("missing coverage directory")?);
        let artifacts: Vec<Value> =
            serde_json::from_reader(File::open(directory.join("headless-artifacts.json"))?)?;
        let root = env::current_dir()?;
        let target = fs::canonicalize(
            env::var_os("CARGO_TARGET_DIR").ok_or("missing Cargo target directory")?,
        )?;
        let worker = executable(
            &artifacts,
            &root.join("crates").join(WORKER),
            &target,
            false,
        )?;
        Ok(Self {
            directory,
            root,
            target,
            worker,
            artifacts,
        })
    }
}

#[test]
#[ignore = "requires the headless qualification's Cargo artifacts"]
fn real_trajectory_checkpoint() -> Result<()> {
    let input = Inputs::load()?;
    curved_checkpoint::run(
        &input.root,
        &input.target,
        &input.directory,
        &input.artifacts,
        &input.worker,
    )
}

#[test]
#[ignore = "requires the headless qualification's Cargo artifacts"]
fn loaded_grippers_match_local_scene() -> Result<()> {
    loaded_grippers::run(Inputs::load()?)
}

// These are the cuts actually reopened by the rigid/material, off-grid and loaded-foot cases.
fn initial_witnesses(directory: &Path) -> Result<BTreeMap<String, String>> {
    let root = directory.join("worker-inputs");
    let mut files = Vec::new();
    for (scenario, suffix) in [
        ("articulated", "gzmaterialtree"),
        ("automatic", "gzautmaterial"),
    ] {
        for material in ["cloth", "cable"] {
            for state in ["prior", "accepted"] {
                files.push(format!("{scenario}/{material}/{state}.{suffix}"));
            }
        }
    }
    files.extend(
        [
            "rotor/prior.gzrotmaterial",
            "rotor/accepted.gzrotmaterial",
            "grounded/push.gzgroundedrobot",
        ]
        .map(str::to_owned),
    );
    files
        .into_iter()
        .map(|file| Ok((file.clone(), sha256(&root.join(file))?)))
        .collect()
}

fn run_cases(
    suite: &Path,
    worker: &Path,
    directory: &Path,
    label: &str,
    cases: &[impl AsRef<str>],
    inputs_env: &[(&str, &Path)],
) -> Result<u128> {
    let stdout = directory.join(format!("{label}.stdout"));
    let stderr = directory.join(format!("{label}.stderr"));
    let pattern = directory.join("raw").join(format!("{label}-%p-%m.profraw"));
    let mut command = Command::new(suite);
    for name in [
        "GZ_MULTI_TEST_WORKER",
        "GZ_FLOATING_TREE_TEST_WORKER",
        "GZ_HINGE_TEST_WORKER",
        "GZ_MOVING_TEST_WORKER",
    ] {
        command.env(name, worker);
    }
    let inputs = directory.join("worker-inputs");
    command
        .env("GZ_MATERIAL_WITNESSES", inputs.join("articulated"))
        .env("GZ_AUTOMATIC_WITNESS", inputs.join("automatic"))
        .env("GZ_ROTOR_CLOTH_WITNESS", inputs.join("rotor"))
        .env(
            "GZ_ROBOT_LOADED_CUT",
            inputs.join("grounded/push.gzgroundedrobot"),
        );
    for (key, path) in inputs_env {
        command.env(key, path);
    }
    let start = Instant::now();
    let status = command
        .args(["--ignored", "--exact", "--test-threads=1"])
        .args(cases.iter().map(AsRef::as_ref))
        .env("LLVM_PROFILE_FILE", pattern)
        .stdout(File::create_new(&stdout)?)
        .stderr(File::create_new(&stderr)?)
        .status()?;
    if !status.success() || !all_parents_passed(&fs::read_to_string(&stdout)?, cases.len()) {
        return Err(format!(
            "{label} failed the physical worker qualification; see {stdout:?} and {stderr:?}"
        )
        .into());
    }
    Ok(start.elapsed().as_millis())
}

#[test]
#[ignore = "requires the headless qualification's Cargo artifact list and coverage directory"]
fn real_worker_fixtures() -> Result<()> {
    let Inputs {
        directory,
        root,
        target,
        worker,
        artifacts,
    } = Inputs::load()?;
    let suites: BTreeMap<String, Vec<String>> = serde_json::from_str(CASES)?;
    let mut receipt = json!({"worker_sha256": sha256(&worker)?, "input_sha256": initial_witnesses(&directory)?, "suites": {}});
    for (source, mut cases) in suites {
        if cases.is_empty() || cases.iter().collect::<BTreeSet<_>>().len() != cases.len() {
            return Err(format!("invalid headless test selection for {source}").into());
        }
        let suite = executable(
            &artifacts,
            &root.join("crates").join(&source),
            &target,
            true,
        )?;
        let label = format!(
            "worker-{}",
            suite
                .file_name()
                .and_then(|n| n.to_str())
                .ok_or("invalid suite filename")?
        );
        let digest = sha256(&suite)?;
        let humanoid = source == "gz-sim-cli/examples/humanoid_stand.rs";
        let returned = directory.join("worker-returned-humanoid.gzgroundedrobot");
        let cli = humanoid
            .then(|| {
                executable(
                    &artifacts,
                    &root.join("crates/gz-sim-cli/src/main.rs"),
                    &target,
                    false,
                )
            })
            .transpose()?;
        let input_env = cli.as_ref().map(|path| {
            [
                ("GZ_ROBOT_RETURN_WITNESS", returned.as_path()),
                ("GZ_RETURN_LOCAL_CLI", path.as_path()),
            ]
        });
        let mut elapsed = run_cases(
            &suite,
            &worker,
            &directory,
            &label,
            &cases,
            input_env.as_ref().map_or(&[][..], |vars| vars.as_slice()),
        )?;
        let mut cut_hash = None;
        if humanoid {
            let following = [
                "return_tests::failed_robot_publication_fences_the_live_scene_until_disk_reopen",
                "return_tests::same_ledger_rejoins_only_the_last_checkpoint_and_preserves_two_ownership_cycles",
            ];
            let extra = run_cases(
                &suite,
                &worker,
                &directory,
                "worker-returned-humanoid",
                &following,
                &[("GZ_ROBOT_RETURN_SOURCE", &returned)],
            )?;
            cases.extend(following.map(str::to_owned));
            elapsed += extra;
            cut_hash = Some(sha256(&returned)?);
        }
        let mut row = json!({"sha256":digest, "cases":cases, "milliseconds":elapsed});
        if let Some(path) = cli {
            row["local_cli_sha256"] = Value::from(sha256(&path)?);
        }
        if let Some(hash) = cut_hash {
            row["returned_checkpoint_sha256"] = Value::from(hash);
        }
        receipt["suites"][source] = row;
        fs::write(
            directory.join("headless-workers.json"),
            serde_json::to_vec_pretty(&receipt)?,
        )?;
    }
    Ok(())
}

#[test]
fn rejects_missing_ambiguous_external_and_symlinked_executables() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let target = temp.path().join("target");
    fs::create_dir(&target)?;
    let source = Path::new("crates/worker.rs");
    let worker = target.join("worker");
    let external = temp.path().join("external");
    let symlink = target.join("symlink");
    fs::write(&worker, "worker")?;
    fs::write(&external, "external")?;
    #[cfg(unix)]
    std::os::unix::fs::symlink(&worker, &symlink)?;
    let artifact = |path: &Path, test| json!({"target":{"src_path":source},"executable":path,"profile":{"test":test}});
    let good = artifact(&worker, false);
    assert_eq!(
        executable(std::slice::from_ref(&good), source, &target, false)?,
        worker
    );
    for invalid in [
        vec![],
        vec![artifact(&worker, true)],
        vec![artifact(&external, false)],
        vec![artifact(&symlink, false)],
        vec![good, artifact(&external, false)],
    ] {
        assert!(executable(&invalid, source, &target, false).is_err());
    }
    assert_eq!(sha256(&worker)?.len(), 64);
    Ok(())
}

#[test]
fn requires_all_selected_parents_in_the_final_libtest_summary() {
    let child = "test result: ok. 2 passed; 0 failed; 0 ignored; 0 measured; 23 filtered out";
    assert!(all_parents_passed(child, 2));
    assert!(!all_parents_passed(
        &format!("{child}\ntest result: ok. 1 passed; 0 failed; 0 ignored;"),
        2
    ));
    assert!(!all_parents_passed(
        &format!("{child}\ntest result: FAILED. 0 passed; 1 failed;"),
        2
    ));
    assert!(!all_parents_passed("", 0));
}
