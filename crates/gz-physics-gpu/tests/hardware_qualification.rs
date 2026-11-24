// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run the hardware-only physics cases from the same Cargo graph as the renderer profile.
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    env,
    error::Error,
    fs, io,
    path::{Path, PathBuf},
    process::Command,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const TARGETS: &[&str] = &[
    "gz-physics-gpu/src/lib.rs",
    "gz-physics-gpu/tests/contact_profile.rs",
    "gz-physics-gpu/tests/oblique_contact.rs",
    "gz-physics-gpu/tests/resident_observation.rs",
    "gz-physics-gpu/tests/scheduled_batch.rs",
    "gz-physics-gpu/tests/sphere_pair.rs",
    "gz-physics-gpu/tests/supported_pair.rs",
    "gz-sim-scene/tests/gpu_automatic.rs",
    "gz-sim-scene/tests/gpu_resident_sdf.rs",
    "gz-sim-scene/tests/gpu_sdf.rs",
    "gz-sim-scene/tests/gpu_sdf_tool.rs",
    "gz-sim-scene/tests/gpu_sensors.rs",
    "gz-sim-scene/tests/native_gpu.rs",
    "gz-sim-cli/tests/gpu_sensor_transport.rs",
];

fn digest(path: &Path) -> Result<String> {
    let mut hash = Sha256::new();
    io::copy(&mut fs::File::open(path)?, &mut hash)?;
    Ok(format!("{:x}", hash.finalize()))
}

fn executables(
    artifacts: &[Value],
    root: &Path,
    target: &Path,
) -> Result<BTreeMap<String, PathBuf>> {
    let mut found = BTreeMap::new();
    for a in artifacts.iter().filter(|a| a["profile"]["test"] == true) {
        let Some(src) = a["target"]["src_path"]
            .as_str()
            .and_then(|s| Path::new(s).strip_prefix(root).ok())
            .and_then(Path::to_str)
        else {
            continue;
        };
        if !TARGETS.contains(&src) {
            continue;
        }
        let path = PathBuf::from(a["executable"].as_str().ok_or("missing test executable")?);
        if !path.is_file() || path.is_symlink() || !path.canonicalize()?.starts_with(target) {
            return Err(format!("unexpected Cargo test executable {path:?}").into());
        }
        if found
            .insert(src.to_owned(), path.clone())
            .is_some_and(|previous| previous != path)
        {
            return Err(format!("ambiguous Cargo artifacts for {src}").into());
        }
    }
    for &name in TARGETS {
        if !found.contains_key(name) {
            return Err(format!("missing hardware target {name}").into());
        }
    }
    Ok(found)
}

#[test]
#[ignore = "requires an explicitly selected hardware f64 Vulkan adapter and an exact renderer build"]
fn selected_hardware_physics() -> Result<()> {
    let adapter = env::var("GZ_GPU_ADAPTER")?;
    let directory = PathBuf::from(
        env::var_os("GZ_GPU_PROFILE_EVIDENCE").ok_or("missing renderer evidence directory")?,
    );
    let target =
        PathBuf::from(env::var_os("CARGO_TARGET_DIR").ok_or("missing Cargo target directory")?)
            .canonicalize()?;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(1)
        .unwrap()
        .to_owned();
    let artifacts: Vec<Value> =
        serde_json::from_slice(&fs::read(directory.join("gpu-hardware-artifacts.json"))?)?;
    let binaries = executables(&artifacts, &root, &target)?;
    let mut receipts = vec![];
    for (source, exe) in binaries {
        let list = Command::new(&exe).args(["--ignored", "--list"]).output()?;
        let stdout = String::from_utf8(list.stdout)?;
        let cases: Vec<_> = stdout
            .lines()
            .filter_map(|s| s.strip_suffix(": test"))
            .collect();
        if !list.status.success() || cases.is_empty() {
            return Err(format!("no runnable hardware cases in {source}").into());
        }
        let hash = digest(&exe)?;
        let output = Command::new(&exe)
            .args(["--ignored", "--test-threads=1", "--nocapture"])
            .env_remove("GZ_GPU_QUALIFICATION_WORKER")
            .output()?;
        let name = exe.file_name().unwrap().to_string_lossy();
        fs::write(directory.join(format!("gpu-{name}.stdout")), &output.stdout)?;
        fs::write(directory.join(format!("gpu-{name}.stderr")), &output.stderr)?;
        let expected = format!(
            "test result: ok. {} passed; 0 failed; 0 ignored;",
            cases.len()
        );
        if !output.status.success()
            || !String::from_utf8_lossy(&output.stdout)
                .lines()
                .rev()
                .find(|s| s.starts_with("test result: "))
                .is_some_and(|s| s.starts_with(&expected))
            || digest(&exe)? != hash
        {
            return Err(format!(
                "hardware qualification failed for {source}, see gpu-{name}.stdout and .stderr"
            )
            .into());
        }
        receipts.push(json!({"source":source,"executable":exe,"sha256":hash,"passed":cases}));
    }
    fs::write(
        directory.join("gpu-hardware.json"),
        serde_json::to_vec_pretty(&json!({"adapter":adapter,"targets":receipts}))?,
    )?;
    Ok(())
}
