// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../build.rs"]
mod build_script;

use serde_json::Value;
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

struct Fixture {
    root: tempfile::TempDir,
    output: tempfile::TempDir,
}

fn copy_tree(source: &Path, destination: &Path) {
    fs::create_dir_all(destination).unwrap();
    for entry in fs::read_dir(source).unwrap() {
        let entry = entry.unwrap();
        if entry.file_type().unwrap().is_dir() {
            copy_tree(&entry.path(), &destination.join(entry.file_name()));
        } else {
            fs::copy(entry.path(), destination.join(entry.file_name())).unwrap();
        }
    }
}

impl Fixture {
    fn new() -> Self {
        let root = tempfile::tempdir().unwrap();
        let original = Path::new(env!("CARGO_MANIFEST_DIR"));
        copy_tree(&original.join("schemas"), &root.path().join("schemas"));
        fs::copy(
            original.join("schema-source.json"),
            root.path().join("schema-source.json"),
        )
        .unwrap();
        Self {
            root,
            output: tempfile::tempdir().unwrap(),
        }
    }

    fn generate(&self) -> Result<(), Box<dyn std::error::Error>> {
        build_script::generate(
            self.root.path(),
            self.output.path(),
            Path::new("/usr/bin/protoc"),
            Path::new("/usr/include"),
        )
    }

    fn source(&self, edit: impl FnOnce(&mut Value)) {
        let path = self.root.path().join("schema-source.json");
        let mut value: Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        edit(&mut value);
        fs::write(path, serde_json::to_vec_pretty(&value).unwrap()).unwrap();
    }

    fn manifest(&self, edit: impl FnOnce(&mut Value)) {
        let path = self.root.path().join("schemas/manifest.json");
        let mut value: Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        edit(&mut value);
        fs::write(&path, serde_json::to_vec_pretty(&value).unwrap()).unwrap();
        self.source(|source| source["manifest_sha256"] = build_script::hash(&path).unwrap().into());
    }

    fn changed_double(&self, edit: impl FnOnce(String) -> String) {
        let path = self.root.path().join("schemas/gz/msgs/double.proto");
        fs::write(&path, edit(fs::read_to_string(&path).unwrap())).unwrap();
        self.manifest(|manifest| {
            let entry = manifest["schemas"]
                .as_array_mut()
                .unwrap()
                .iter_mut()
                .find(|entry| entry["path"] == "schemas/gz/msgs/double.proto")
                .unwrap();
            entry["sha256"] = build_script::hash(&path).unwrap().into();
        });
    }
}

#[test]
fn exact_standard_compiler_generates_source_bound_bindings() {
    let fixture = Fixture::new();
    fixture.generate().unwrap();
    let evidence: Value = serde_json::from_slice(
        &fs::read(fixture.output.path().join("schema-validation.json")).unwrap(),
    )
    .unwrap();
    assert_eq!(evidence["schema_count"], 216);
    assert_eq!(
        evidence["compiler_sha256"],
        build_script::hash(Path::new("/usr/bin/protoc")).unwrap()
    );
    assert_eq!(
        evidence["generated"]["sha256"],
        build_script::hash(&fixture.output.path().join("gz.msgs.rs")).unwrap()
    );
    assert!(
        fs::read_to_string(fixture.output.path().join("gz.msgs.rs"))
            .unwrap()
            .contains("impl ::prost::Name for Clock")
    );
}

#[test]
fn immutable_manifest_source_and_schema_bytes_are_checked() {
    let fixture = Fixture::new();
    fixture.source(|source| source["manifest_sha256"] = "bad".into());
    assert!(
        fixture
            .generate()
            .unwrap_err()
            .to_string()
            .contains("manifest hash")
    );
    let fixture = Fixture::new();
    let path = fixture.root.path().join("schemas/gz/msgs/double.proto");
    fs::write(&path, "corrupt").unwrap();
    assert!(
        fixture
            .generate()
            .unwrap_err()
            .to_string()
            .contains("schema bytes")
    );
    fs::remove_file(path).unwrap();
    assert!(fixture.generate().is_err());
    for edit in [
        "repository",
        "schema_count",
        "generator",
        "note",
        "revision",
        "schema_only_candidate",
        "upstream_commit",
    ] {
        let fixture = Fixture::new();
        fixture.source(|source| {
            source[edit] = if edit == "schema_count" {
                0.into()
            } else {
                "".into()
            }
        });
        assert!(
            fixture
                .generate()
                .unwrap_err()
                .to_string()
                .contains("source identity"),
            "{edit}"
        );
    }
    for revision in ["Z".repeat(40), "A".repeat(40)] {
        let fixture = Fixture::new();
        fixture.source(|source| source["revision"] = revision.into());
        assert!(
            fixture
                .generate()
                .unwrap_err()
                .to_string()
                .contains("source identity")
        );
    }
    for field in ["upstream", "commit"] {
        let fixture = Fixture::new();
        fixture.manifest(|manifest| manifest[field] = "different".into());
        assert!(
            fixture
                .generate()
                .unwrap_err()
                .to_string()
                .contains("source identity")
        );
    }
}

#[test]
fn missing_malformed_and_unwritable_build_inputs_fail_without_bindings() {
    for contents in [None, Some("not JSON"), Some("{\"unknown\":true}")] {
        let fixture = Fixture::new();
        let source = fixture.root.path().join("schema-source.json");
        if let Some(contents) = contents {
            fs::write(source, contents).unwrap();
        } else {
            fs::remove_file(source).unwrap();
        }
        assert!(fixture.generate().is_err());
        assert!(!fixture.output.path().join("gz.msgs.rs").exists());
    }
    let fixture = Fixture::new();
    fs::remove_file(fixture.root.path().join("schemas/manifest.json")).unwrap();
    assert!(fixture.generate().is_err());
    let fixture = Fixture::new();
    let manifest = fixture.root.path().join("schemas/manifest.json");
    fs::write(&manifest, "malformed JSON").unwrap();
    fixture
        .source(|source| source["manifest_sha256"] = build_script::hash(&manifest).unwrap().into());
    assert!(fixture.generate().is_err());
    let fixture = Fixture::new();
    let include = tempfile::tempdir().unwrap();
    assert!(
        build_script::generate(
            fixture.root.path(),
            fixture.output.path(),
            Path::new("/usr/bin/protoc"),
            include.path()
        )
        .is_err()
    );
    assert!(
        build_script::generate(
            fixture.root.path(),
            fixture.output.path(),
            Path::new("/no-such-protoc"),
            Path::new("/usr/include")
        )
        .is_err()
    );
    let compiler = fixture.root.path().join("not-executable");
    fs::write(&compiler, "not executable").unwrap();
    assert!(
        build_script::generate(
            fixture.root.path(),
            fixture.output.path(),
            &compiler,
            Path::new("/usr/include")
        )
        .is_err()
    );
    // An output file collision must be reported by the real maintained generator.
    fs::create_dir(fixture.output.path().join("gz.msgs.rs")).unwrap();
    assert!(fixture.generate().is_err());
    assert!(
        !fixture
            .output
            .path()
            .join("schema-validation.json")
            .exists()
    );
}

#[test]
fn missing_extra_duplicate_and_unsafe_inventory_fail() {
    let fixture = Fixture::new();
    fixture.manifest(|manifest| {
        manifest["schemas"].as_array_mut().unwrap().pop();
    });
    assert!(
        fixture
            .generate()
            .unwrap_err()
            .to_string()
            .contains("source identity")
    );
    let fixture = Fixture::new();
    fs::write(
        fixture.root.path().join("schemas/extra.proto"),
        "syntax=\"proto3\";",
    )
    .unwrap();
    assert!(
        fixture
            .generate()
            .unwrap_err()
            .to_string()
            .contains("file inventory")
    );
    let fixture = Fixture::new();
    fixture.manifest(|manifest| manifest["schemas"][1] = manifest["schemas"][0].clone());
    assert!(
        fixture
            .generate()
            .unwrap_err()
            .to_string()
            .contains("sorted and unique")
    );
    for path in [
        "../outside.proto",
        "/absolute.proto",
        "schemas/gz/msgs/../bad.proto",
        "schemas/gz/msgs/bad.txt",
        "schemas/gz/msgs/bad\\name.proto",
    ] {
        let fixture = Fixture::new();
        fixture.manifest(|manifest| manifest["schemas"][0]["path"] = path.into());
        assert!(
            fixture
                .generate()
                .unwrap_err()
                .to_string()
                .contains("inventory path"),
            "{path}"
        );
    }
    let fixture = Fixture::new();
    let path = fixture.root.path().join("schemas/gz/msgs/double.proto");
    let original = fs::read(&path).unwrap();
    fs::remove_file(&path).unwrap();
    let target = fixture.root.path().join("target.proto");
    fs::write(&target, original).unwrap();
    std::os::unix::fs::symlink(target, path).unwrap();
    assert!(
        fixture
            .generate()
            .unwrap_err()
            .to_string()
            .contains("symlink")
    );
}

#[test]
fn real_compiler_rejects_changed_tags_and_import_closure() {
    let fixture = Fixture::new();
    fixture.changed_double(|text| text.replace("double data = 2;", "double data = 22;"));
    assert!(
        fixture
            .generate()
            .unwrap_err()
            .to_string()
            .contains("compiled descriptor")
    );
    let fixture = Fixture::new();
    fixture.changed_double(|text| text.replace("gz/msgs/header.proto", "missing.proto"));
    assert!(
        fixture
            .generate()
            .unwrap_err()
            .to_string()
            .contains("protoc failed")
    );
    let fixture = Fixture::new();
    fixture.changed_double(|text| text + "\nimport \"extra.proto\";\n");
    let include = tempfile::tempdir().unwrap();
    copy_tree(
        Path::new("/usr/include/google/protobuf"),
        &include.path().join("google/protobuf"),
    );
    fs::write(
        include.path().join("extra.proto"),
        "syntax=\"proto3\"; message Extra {}",
    )
    .unwrap();
    let error = build_script::generate(
        fixture.root.path(),
        fixture.output.path(),
        Path::new("/usr/bin/protoc"),
        include.path(),
    )
    .unwrap_err();
    assert!(error.to_string().contains("import closure"), "{error}");
}

#[test]
fn an_external_symlinked_schema_root_is_rejected_before_generation() {
    let fixture = Fixture::new();
    let external = tempfile::tempdir().unwrap();
    let schemas = fixture.root.path().join("schemas");
    let moved = external.path().join("schemas");
    fs::rename(&schemas, &moved).unwrap();
    std::os::unix::fs::symlink(&moved, &schemas).unwrap();
    let error = fixture.generate().unwrap_err();
    assert!(
        error
            .to_string()
            .contains("schema tree root is not a real directory")
    );
    assert!(!fixture.output.path().join("gz.msgs.rs").exists());
    assert!(
        !fixture
            .output
            .path()
            .join("schema-validation.json")
            .exists()
    );
}

fn executable(path: &Path, body: &str) {
    use std::os::unix::fs::PermissionsExt;
    fs::write(path, format!("#!/bin/sh\n# Copyright (C) 2026 Open Source Robotics Foundation\n# SPDX-License-Identifier: Apache-2.0\n{body}\n")).unwrap();
    fs::set_permissions(path, fs::Permissions::from_mode(0o755)).unwrap();
}

#[test]
fn compiler_and_standard_dependency_drift_fail() {
    let fixture = Fixture::new();
    let include = tempfile::tempdir().unwrap();
    copy_tree(
        Path::new("/usr/include/google/protobuf"),
        &include.path().join("google/protobuf"),
    );
    fs::write(include.path().join("google/protobuf/any.proto"), "changed").unwrap();
    assert!(
        build_script::generate(
            fixture.root.path(),
            fixture.output.path(),
            Path::new("/usr/bin/protoc"),
            include.path()
        )
        .unwrap_err()
        .to_string()
        .contains("Any source")
    );
    for body in [
        "echo 'libprotoc 0.0'",
        "exit 9",
        "echo diagnostic >&2; /usr/bin/protoc --version",
    ] {
        let compiler = fixture.root.path().join("protoc");
        executable(&compiler, body);
        assert!(
            build_script::generate(
                fixture.root.path(),
                fixture.output.path(),
                &compiler,
                Path::new("/usr/include")
            )
            .unwrap_err()
            .to_string()
            .contains("protoc identity")
        );
    }
    let compiler = fixture.root.path().join("protoc");
    executable(
        &compiler,
        "if [ \"$1\" = --version ]; then exec /usr/bin/protoc \"$@\"; fi\n/usr/bin/protoc \"$@\"\nresult=$?\necho '# changed' >> \"$0\"\nexit \"$result\"",
    );
    assert!(
        build_script::generate(
            fixture.root.path(),
            fixture.output.path(),
            &compiler,
            Path::new("/usr/include")
        )
        .unwrap_err()
        .to_string()
        .contains("changed during generation")
    );
    assert!(build_script::compiler(Path::new("/no-such-compiler/protoc")).is_err());
}

fn child(fixture: &Fixture) -> Command {
    let mut command = Command::new(std::env::current_exe().unwrap());
    command
        .args([
            "--exact",
            "build_script_main_child",
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env("CARGO_MANIFEST_DIR", fixture.root.path())
        .env("OUT_DIR", fixture.output.path())
        .env_remove("PROTOC")
        .env_remove("PROTOC_INCLUDE");
    command
}

#[test]
fn actual_main_child_exercises_cargo_inputs_and_directives() {
    let fixture = Fixture::new();
    let output = child(&fixture).output().unwrap();
    assert!(output.status.success(), "{output:?}");
    let stdout = String::from_utf8(output.stdout).unwrap();
    for variable in ["PROTOC", "PATH", "PROTOC_INCLUDE"] {
        assert!(stdout.contains(&format!("cargo:rerun-if-env-changed={variable}")));
    }
    for path in [
        fixture.root.path().join("schemas"),
        fixture.root.path().join("schema-source.json"),
        PathBuf::from("/usr/include/google/protobuf/any.proto"),
        PathBuf::from("/usr/bin/protoc"),
    ] {
        assert!(stdout.contains(&format!("cargo:rerun-if-changed={}", path.display())));
    }
    assert!(fixture.output.path().join("gz.msgs.rs").is_file());
    for variable in ["CARGO_MANIFEST_DIR", "OUT_DIR", "PATH"] {
        let output = child(&fixture).env_remove(variable).output().unwrap();
        assert!(!output.status.success(), "{variable}");
    }
    let output = child(&fixture).env("PATH", "").output().unwrap();
    assert!(!output.status.success());
    let output = child(&fixture)
        .env("PROTOC", "/usr/bin/protoc")
        .env("PROTOC_INCLUDE", "/usr/include")
        .output()
        .unwrap();
    assert!(output.status.success(), "{output:?}");
}

#[test]
#[ignore = "executed by actual_main_child_exercises_cargo_inputs_and_directives"]
fn build_script_main_child() {
    build_script::main();
}
