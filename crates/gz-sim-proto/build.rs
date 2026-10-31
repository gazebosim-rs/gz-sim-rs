// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeSet,
    env,
    error::Error,
    fs,
    path::{Component, Path, PathBuf},
    process::Command,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Source {
    repository: String,
    revision: String,
    upstream_commit: String,
    schema_only_candidate: String,
    schema_count: usize,
    note: String,
    manifest_sha256: String,
    compiler_version: String,
    external_any_sha256: String,
    generator: String,
    descriptor_sha256: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    upstream: String,
    commit: String,
    schemas: Vec<Schema>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Schema {
    path: String,
    sha256: String,
}

fn require(valid: bool, message: &str) -> Result<()> {
    if !valid {
        return Err(message.into());
    }
    Ok(())
}

pub(crate) fn hash(path: &Path) -> Result<String> {
    Ok(format!("{:x}", Sha256::digest(fs::read(path)?)))
}

fn inventory(directory: &Path, files: &mut BTreeSet<PathBuf>) -> Result<()> {
    require(
        fs::symlink_metadata(directory)?.is_dir(),
        "schema tree root is not a real directory",
    )?;
    for entry in fs::read_dir(directory)? {
        let entry = entry?;
        let kind = entry.file_type()?;
        require(!kind.is_symlink(), "schema tree contains a symlink")?;
        if kind.is_dir() {
            inventory(&entry.path(), files)?;
        } else if entry.path().extension().is_some_and(|ext| ext == "proto") {
            files.insert(entry.path());
        }
    }
    Ok(())
}

pub(crate) fn compiler(path: &Path) -> Result<PathBuf> {
    let resolved = if path.components().count() > 1 {
        path.to_owned()
    } else {
        env::split_paths(&env::var_os("PATH").ok_or("PATH is missing")?)
            .map(|directory| directory.join(path))
            .find(|candidate| candidate.is_file())
            .ok_or("protoc was not found on PATH")?
    };
    Ok(resolved.canonicalize()?)
}

/// Validate the immutable source and actual compiler before generating bindings.
pub(crate) fn generate(root: &Path, output: &Path, protoc: &Path, include: &Path) -> Result<()> {
    let source: Source = serde_json::from_slice(&fs::read(root.join("schema-source.json"))?)?;
    let manifest_path = root.join("schemas/manifest.json");
    require(
        hash(&manifest_path)? == source.manifest_sha256,
        "schema manifest hash differs",
    )?;
    let manifest: Manifest = serde_json::from_slice(&fs::read(&manifest_path)?)?;
    require(
        source.repository == "https://github.com/gazebosim-rs/gz-msgs-rs"
            && manifest.upstream == "https://github.com/gazebosim/gz-msgs"
            && manifest.commit == source.upstream_commit
            && source.schema_count == 216
            && manifest.schemas.len() == source.schema_count
            && source.generator == "prost-build 0.14.4"
            && !source.note.is_empty()
            && [
                &source.revision,
                &source.schema_only_candidate,
                &source.upstream_commit,
            ]
            .iter()
            .all(|value| {
                value.len() == 40
                    && value
                        .bytes()
                        .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
            }),
        "schema source identity differs",
    )?;
    let mut inputs = Vec::new();
    for entry in &manifest.schemas {
        let path = Path::new(&entry.path);
        require(
            path.starts_with("schemas/gz/msgs")
                && path.extension().is_some_and(|ext| ext == "proto")
                && path
                    .components()
                    .all(|part| matches!(part, Component::Normal(_)))
                && !entry.path.contains('\\'),
            "invalid schema inventory path",
        )?;
        let path = root.join(path);
        require(
            hash(&path)? == entry.sha256,
            "canonical schema bytes differ",
        )?;
        require(
            inputs.last().is_none_or(|previous| previous < &path),
            "schema inventory is not sorted and unique",
        )?;
        inputs.push(path);
    }
    let mut actual = BTreeSet::new();
    inventory(&root.join("schemas"), &mut actual)?;
    require(
        actual == inputs.iter().cloned().collect(),
        "schema file inventory differs",
    )?;
    let any = include.join("google/protobuf/any.proto");
    require(
        hash(&any)? == source.external_any_sha256,
        "standard Any source differs",
    )?;
    let compiler_hash = hash(protoc)?;
    let version = Command::new(protoc).arg("--version").output()?;
    require(
        version.status.success()
            && version.stdout == source.compiler_version.as_bytes()
            && version.stderr.is_empty(),
        "protoc identity differs",
    )?;
    let descriptor = output.join("schema-descriptor.pb");
    let mut config = prost_build::Config::new();
    config
        .protoc_executable(protoc)
        .out_dir(output)
        .enable_type_names()
        .file_descriptor_set_path(&descriptor);
    let descriptors = config.load_fds(&inputs, &[root.join("schemas"), include.to_owned()])?;
    let expected: BTreeSet<_> = manifest
        .schemas
        .iter()
        .map(|entry| entry.path.trim_start_matches("schemas/").to_owned())
        .chain(["google/protobuf/any.proto".to_owned()])
        .collect();
    let names: Vec<_> = descriptors
        .file
        .iter()
        .map(|file| file.name.clone().unwrap_or_default())
        .collect();
    require(
        names.len() == expected.len() && names.into_iter().collect::<BTreeSet<_>>() == expected,
        "compiled import closure differs",
    )?;
    require(
        hash(&descriptor)? == source.descriptor_sha256,
        "compiled descriptor differs",
    )?;
    config.compile_fds(descriptors)?;
    require(
        hash(protoc)? == compiler_hash,
        "protoc changed during generation",
    )?;
    let generated = output.join("gz.msgs.rs");
    let evidence = serde_json::json!({
        "schema_source_sha256": hash(&root.join("schema-source.json"))?,
        "manifest_sha256": source.manifest_sha256,
        "schema_revision": source.schema_only_candidate,
        "schema_count": inputs.len(), "generator": source.generator,
        "compiler": protoc, "compiler_sha256": compiler_hash,
        "compiler_version": source.compiler_version,
        "external_any_sha256": source.external_any_sha256,
        "external_any_path": any,
        "descriptor_sha256": source.descriptor_sha256,
        "generated": {"path": generated, "sha256": hash(&generated)?}
    });
    fs::write(
        output.join("schema-validation.json"),
        serde_json::to_vec_pretty(&evidence)?,
    )?;
    for path in [
        root.join("schemas"),
        root.join("schema-source.json"),
        any,
        protoc.to_owned(),
    ] {
        println!("cargo:rerun-if-changed={}", path.display());
    }
    Ok(())
}

fn run() -> Result<()> {
    for name in ["PROTOC", "PATH", "PROTOC_INCLUDE"] {
        println!("cargo:rerun-if-env-changed={name}");
    }
    generate(
        &PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").ok_or("manifest directory missing")?),
        &PathBuf::from(env::var_os("OUT_DIR").ok_or("output directory missing")?),
        &compiler(&PathBuf::from(
            env::var_os("PROTOC").unwrap_or_else(|| "protoc".into()),
        ))?,
        &PathBuf::from(env::var_os("PROTOC_INCLUDE").unwrap_or_else(|| "/usr/include".into())),
    )
}

pub(crate) fn main() {
    run().expect("canonical simulator schema generation failed");
}
