// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_assets::{AssetError, LocalResources, Warning};
use sdformat::{ModelConfigWarningKind, ResourceResolver};
use std::{fs, path::Path};
use tempfile::TempDir;

fn directory(root: &Path, name: &str) -> std::path::PathBuf {
    let path = root.join(name);
    fs::create_dir_all(&path).unwrap();
    path
}
fn config(path: &Path, xml: &str) {
    fs::write(path.join("model.config"), xml).unwrap();
}

#[test]
fn local_lookup_preserves_cwd_maps_roots_and_literal_uri_suffixes() {
    let temp = TempDir::new().unwrap();
    let cwd = directory(temp.path(), "cwd");
    let a = directory(temp.path(), "a");
    let b = directory(temp.path(), "b");
    for (path, value) in [(&cwd, "cwd"), (&a, "a"), (&b, "b")] {
        fs::write(path.join("choice.sdf"), value).unwrap();
    }
    let mut resolver = LocalResources::new(cwd.clone(), vec![a.clone(), b.clone()], 1024).unwrap();
    assert_eq!(resolver.load_uri("choice.sdf").unwrap().xml, "cwd");
    assert_eq!(resolver.load_uri("model://choice.sdf").unwrap().xml, "cwd");
    assert_eq!(resolver.load_uri("file://choice.sdf").unwrap().xml, "cwd");
    resolver.add_uri_path("model://".into(), b.clone());
    resolver.add_uri_path("model://".into(), a.clone());
    assert_eq!(resolver.load_uri("model://choice.sdf").unwrap().xml, "b");
    assert_eq!(resolver.load_uri("file://choice.sdf").unwrap().xml, "cwd");
    assert_eq!(
        resolver
            .load_uri(&format!("file://{}", a.join("choice.sdf").display()))
            .unwrap()
            .xml,
        "a"
    );
    fs::remove_file(cwd.join("choice.sdf")).unwrap();
    assert_eq!(resolver.load_uri("choice.sdf").unwrap().xml, "a");
    fs::write(cwd.join("block"), "file").unwrap();
    fs::create_dir(a.join("block")).unwrap();
    fs::write(a.join("block/selected.sdf"), "selected").unwrap();
    assert_eq!(
        resolver.load_uri("block/selected.sdf").unwrap().xml,
        "selected"
    );
    fs::write(cwd.join("a b.sdf"), "space").unwrap();
    fs::write(cwd.join("a%20b.sdf"), "percent").unwrap();
    assert_eq!(resolver.load_uri("file://a b.sdf").unwrap().xml, "space");
    assert_eq!(
        resolver.load_uri("file://a%20b.sdf").unwrap().xml,
        "percent"
    );
    assert!(matches!(
        resolver.load_uri("missing.sdf"),
        Err(AssetError::NotFound(_))
    ));
    assert!(
        resolver
            .resolve("missing.sdf", "ignored-parent.sdf")
            .unwrap_err()
            .contains("not found")
    );
}

#[test]
fn config_selection_keeps_first_tie_future_warnings_and_manifest_fallback() {
    let temp = TempDir::new().unwrap();
    let model = directory(temp.path(), "model");
    config(
        &model,
        "<model><name>model</name><sdf version='1.6'>old.sdf</sdf><sdf version='1.12'>current.sdf</sdf><sdf version='1.12'>tie.sdf</sdf><sdf version='1.99'>future.sdf</sdf></model>",
    );
    fs::write(model.join("current.sdf"), "current").unwrap();
    let resolver = LocalResources::new(temp.path().into(), vec![], 4096).unwrap();
    let resource = resolver.read_path(&model).unwrap();
    assert_eq!(resource.xml, "current");
    let warnings = resolver.take_warnings().unwrap();
    assert_eq!(warnings.len(), 1);
    assert!(
        matches!(&warnings[0],Warning::Config(w) if w.kind==ModelConfigWarningKind::NewerVersion("1.99".into()))
    );
    assert!(resolver.take_warnings().unwrap().is_empty());
    fs::remove_file(model.join("model.config")).unwrap();
    fs::write(
        model.join("manifest.xml"),
        "<model><sdf>current.sdf</sdf></model>",
    )
    .unwrap();
    assert_eq!(resolver.read_path(&model).unwrap().xml, "current");
    let warnings = resolver.take_warnings().unwrap();
    assert_eq!(warnings.len(), 2);
    assert!(
        matches!(&warnings[0],Warning::LegacyManifest(path) if path==&model.join("manifest.xml"))
    );
    assert!(
        matches!(&warnings[1],Warning::Config(w) if w.kind==ModelConfigWarningKind::MissingVersion)
    );
    // Presence of malformed model.config prevents falling back to valid manifest.xml.
    config(&model, "broken");
    assert!(matches!(
        resolver.read_path(&model),
        Err(AssetError::Config(_))
    ));
    fs::remove_file(model.join("model.config")).unwrap();
    fs::write(model.join("manifest.xml"), "broken").unwrap();
    assert!(matches!(
        resolver.read_path(&model),
        Err(AssetError::Config(_))
    ));
    let warnings = resolver.take_warnings().unwrap();
    assert_eq!(warnings.len(), 1);
    assert!(
        matches!(&warnings[0],Warning::LegacyManifest(path) if path==&model.join("manifest.xml"))
    );
}

#[test]
fn missing_selected_files_are_not_replaced_by_other_versions() {
    let temp = TempDir::new().unwrap();
    let model = directory(temp.path(), "model");
    config(
        &model,
        "<model><name>m</name><sdf version='9.0'>absent.sdf</sdf><sdf version='8.0'>exists.sdf</sdf></model>",
    );
    fs::write(model.join("exists.sdf"), "existing").unwrap();
    let resolver = LocalResources::new(temp.path().into(), vec![], 4096).unwrap();
    assert!(
        matches!(resolver.read_path(&model),Err(AssetError::Io{source,..}) if source.kind()==std::io::ErrorKind::NotFound)
    );
    assert_eq!(resolver.take_warnings().unwrap().len(), 2);
    fs::remove_file(model.join("model.config")).unwrap();
    assert!(matches!(
        resolver.read_path(&model),
        Err(AssetError::Io { .. })
    ));
}

#[test]
fn byte_utf8_and_warning_budgets_fail_explicitly() {
    let temp = TempDir::new().unwrap();
    let resolver = LocalResources::new(temp.path().into(), vec![], 3).unwrap();
    fs::write(temp.path().join("limit"), b"abc").unwrap();
    assert_eq!(resolver.read_path(Path::new("limit")).unwrap().xml, "abc");
    fs::write(temp.path().join("limit"), b"abcd").unwrap();
    assert!(matches!(
        resolver.read_path(Path::new("limit")),
        Err(AssetError::TooLarge(_))
    ));
    fs::write(temp.path().join("bad"), [255]).unwrap();
    assert!(matches!(
        resolver.read_path(Path::new("bad")),
        Err(AssetError::Utf8(_))
    ));
    assert!(matches!(
        resolver.read_path(Path::new("absent")),
        Err(AssetError::Io { .. })
    ));
    assert!(matches!(
        LocalResources::new("relative".into(), vec![], 100),
        Err(AssetError::RelativeBase)
    ));
    let model = directory(temp.path(), "model");
    config(&model, "<model><sdf>file.sdf</sdf></model>");
    fs::write(model.join("file.sdf"), "file").unwrap();
    let mut resolver = LocalResources::new(temp.path().into(), vec![], 4096).unwrap();
    resolver.set_warning_limit(0);
    assert!(matches!(
        resolver.read_path(&model),
        Err(AssetError::WarningLimit)
    ));
    assert!(resolver.take_warnings().unwrap().is_empty());
    resolver.set_warning_limit(1);
    assert_eq!(resolver.read_path(&model).unwrap().xml, "file");
    assert!(matches!(
        resolver.read_path(&model),
        Err(AssetError::WarningLimit)
    ));
    assert_eq!(resolver.take_warnings().unwrap().len(), 1);
    fs::remove_file(model.join("model.config")).unwrap();
    fs::write(
        model.join("manifest.xml"),
        "<model><name>model</name><sdf>file.sdf</sdf></model>",
    )
    .unwrap();
    resolver.set_warning_limit(0);
    assert!(matches!(
        resolver.read_path(&model),
        Err(AssetError::WarningLimit)
    ));
    assert!(resolver.take_warnings().unwrap().is_empty());
    resolver.set_warning_limit(1);
    assert!(matches!(
        resolver.read_path(&model),
        Err(AssetError::WarningLimit)
    ));
    assert!(matches!(
        &resolver.take_warnings().unwrap()[0],
        Warning::LegacyManifest(_)
    ));
}

#[cfg(unix)]
#[test]
fn canonical_source_identity_preserves_symlink_aliases_and_non_utf8_paths() {
    use std::{
        ffi::OsString,
        os::unix::{ffi::OsStringExt, fs::symlink},
    };
    let temp = TempDir::new().unwrap();
    let name = OsString::from_vec(b"model-\xff.sdf".to_vec());
    let path = temp.path().join(name);
    fs::write(&path, "model").unwrap();
    symlink(&path, temp.path().join("alias.sdf")).unwrap();
    let resolver = LocalResources::new(temp.path().into(), vec![], 100).unwrap();
    let resource = resolver.read_path(&path).unwrap();
    assert_eq!(
        resource.canonical_uri,
        resolver
            .read_path(Path::new("alias.sdf"))
            .unwrap()
            .canonical_uri
    );
    assert!(resource.canonical_uri.contains("%FF"));
}

#[test]
fn canonical_include_cycles_are_rejected_by_sdf_before_world_construction() {
    let temp = TempDir::new().unwrap();
    fs::write(temp.path().join("loop.sdf"),"<sdf version='1.12'><model name='loop'><include><uri>loop.sdf</uri></include></model></sdf>").unwrap();
    let resolver = LocalResources::new(temp.path().into(), vec![], 4096).unwrap();
    let root = resolver.read_path(Path::new("loop.sdf")).unwrap();
    let error = sdformat::load_str(&root.xml, &root.canonical_uri, &resolver).unwrap_err();
    assert_eq!(error.code, sdformat::ErrorCode::Cycle);
}

#[cfg(unix)]
#[test]
fn lookup_stat_failures_follow_native_fallback_while_explicit_reads_report_errors() {
    use std::os::unix::fs::symlink;
    let temp = TempDir::new().unwrap();
    let other = directory(temp.path(), "other");
    fs::write(other.join("loop.sdf"), "fallback").unwrap();
    symlink("loop.sdf", temp.path().join("loop.sdf")).unwrap();
    let resolver = LocalResources::new(temp.path().into(), vec![other], 4096).unwrap();
    assert_eq!(resolver.load_uri("loop.sdf").unwrap().xml, "fallback");
    assert!(matches!(
        resolver.read_path(Path::new("loop.sdf")),
        Err(AssetError::Io { .. })
    ));
    let model = directory(temp.path(), "model");
    symlink("model.config", model.join("model.config")).unwrap();
    fs::write(
        model.join("manifest.xml"),
        "<model><name>model</name><sdf version='1.12'>alternative.sdf</sdf></model>",
    )
    .unwrap();
    fs::write(model.join("alternative.sdf"), "alternative").unwrap();
    assert_eq!(resolver.read_path(&model).unwrap().xml, "alternative");
    assert!(matches!(
        &resolver.take_warnings().unwrap()[0],
        Warning::LegacyManifest(_)
    ));
}

#[cfg(unix)]
#[test]
fn absolute_include_suffixes_preserve_native_concatenation_and_explicit_paths() {
    let temp = TempDir::new().unwrap();
    let cwd = directory(temp.path(), "cwd");
    let root = directory(temp.path(), "root");
    let absolute = temp.path().join("absolute.sdf");
    fs::write(&absolute, "direct").unwrap();
    let mut resolver = LocalResources::new(cwd.clone(), vec![root.clone()], 4096).unwrap();
    let uri = format!("file://{}", absolute.display());
    assert_eq!(resolver.load_uri(&uri).unwrap().xml, "direct");
    let shadow = cwd.join(absolute.strip_prefix(Path::new("/")).unwrap());
    fs::create_dir_all(shadow.parent().unwrap()).unwrap();
    fs::write(&shadow, "shadow").unwrap();
    assert_eq!(resolver.load_uri(&uri).unwrap().xml, "shadow");
    assert_eq!(resolver.read_path(&absolute).unwrap().xml, "direct");
    fs::write(root.join("mapped.sdf"), "mapped").unwrap();
    resolver.add_uri_path("model://".into(), root);
    assert_eq!(
        resolver.load_uri("model:///mapped.sdf").unwrap().xml,
        "mapped"
    );
    let model = directory(temp.path(), "model");
    config(
        &model,
        &format!(
            "<model><name>model</name><sdf version='1.12'>{}</sdf></model>",
            absolute.display()
        ),
    );
    let configured = model.join(absolute.strip_prefix(Path::new("/")).unwrap());
    fs::create_dir_all(configured.parent().unwrap()).unwrap();
    fs::write(&configured, "configured").unwrap();
    assert_eq!(resolver.read_path(&model).unwrap().xml, "configured");
}

#[cfg(unix)]
#[test]
fn config_warning_source_identities_keep_non_utf8_directory_names_distinct() {
    use std::{ffi::OsString, os::unix::ffi::OsStringExt};
    let temp = TempDir::new().unwrap();
    let resolver = LocalResources::new(temp.path().into(), vec![], 4096).unwrap();
    for byte in [0xff, 0xfe] {
        let path = temp.path().join(OsString::from_vec(vec![b'm', byte]));
        fs::create_dir(&path).unwrap();
        config(
            &path,
            "<model><name>model</name><sdf>file.sdf</sdf></model>",
        );
        fs::write(path.join("file.sdf"), "model").unwrap();
        resolver.read_path(&path).unwrap();
    }
    let warnings = resolver.take_warnings().unwrap();
    assert_eq!(warnings.len(), 2);
    let sources: Vec<_> = warnings
        .iter()
        .map(|warning| match warning {
            Warning::Config(config) => config.source.uri.clone(),
            _ => panic!("expected config warning"),
        })
        .collect();
    assert_ne!(sources[0], sources[1]);
    assert!(sources[0].contains("%FF"));
    assert!(sources[1].contains("%FE"));
}

#[test]
fn selected_directories_fail_as_files_without_manifest_or_config_recursion() {
    let temp = TempDir::new().unwrap();
    let model = directory(temp.path(), "model");
    fs::create_dir(model.join("model.config")).unwrap();
    fs::write(
        model.join("manifest.xml"),
        "<model><name>model</name><sdf version='1.12'>file.sdf</sdf></model>",
    )
    .unwrap();
    fs::write(model.join("file.sdf"), "fallback").unwrap();
    let resolver = LocalResources::new(temp.path().into(), vec![], 4096).unwrap();
    assert!(
        matches!(resolver.read_path(&model),Err(AssetError::Io{source,..}) if source.kind()==std::io::ErrorKind::IsADirectory)
    );
    assert!(resolver.take_warnings().unwrap().is_empty());
    fs::remove_dir(model.join("model.config")).unwrap();
    config(
        &model,
        "<model><name>model</name><sdf version='1.12'>.</sdf></model>",
    );
    assert!(
        matches!(resolver.read_path(&model),Err(AssetError::Io{source,..}) if source.kind()==std::io::ErrorKind::IsADirectory)
    );
    assert!(resolver.take_warnings().unwrap().is_empty());
}
