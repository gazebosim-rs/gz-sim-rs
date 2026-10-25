// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_assets::{AssetError, LocalResources};
use gz_fuel_tools::{ClientConfig, FuelClient, ResourceKind, ServerConfig};
use std::{fs, net::TcpListener, path::Path};
use tempfile::TempDir;

fn cached_model(cache: &Path, port: u16, version: u32, value: &str) {
    let path = cache.join(format!("127.0.0.1%3A{port}/alice/models/robot/{version}"));
    fs::create_dir_all(&path).unwrap();
    fs::write(
        path.join("model.config"),
        "<model><name>robot</name><sdf version='1.12'>model.sdf</sdf></model>",
    )
    .unwrap();
    fs::write(path.join("model.sdf"), value).unwrap();
}

#[test]
fn actual_fuel_cache_api_selects_exact_and_tip_without_network_on_hit_or_miss() {
    let temp = TempDir::new().unwrap();
    let cache = temp.path().join("cache");
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    listener.set_nonblocking(true).unwrap();
    let port = listener.local_addr().unwrap().port();
    cached_model(&cache, port, 1, "one");
    cached_model(&cache, port, 2, "two");
    let mut config = ClientConfig {
        cache_location: cache,
        servers: vec![ServerConfig {
            url: format!("http://127.0.0.1:{port}"),
            ..Default::default()
        }],
        ..Default::default()
    };
    config.limits.request_timeout_ms = Some(100);
    let client = FuelClient::new(config);
    let base = format!("http://127.0.0.1:{port}/1.0/ALICE/models/ROBOT");
    let id = client
        .parse(&format!("{base}/1"), ResourceKind::Model)
        .unwrap();
    assert_eq!(
        (id.owner.as_str(), id.name.as_str(), id.version),
        ("alice", "robot", 1)
    );
    assert!(
        client
            .cached(ResourceKind::Model, &id)
            .unwrap()
            .ends_with("1")
    );
    let resolver = LocalResources::new(temp.path().into(), vec![], 4096)
        .unwrap()
        .with_cached_fuel(client);
    assert_eq!(resolver.load_uri(&format!("{base}/1")).unwrap().xml, "one");
    assert_eq!(resolver.load_uri(&base).unwrap().xml, "two");
    assert_eq!(
        resolver.load_uri(&format!("{base}/tip")).unwrap().xml,
        "two"
    );
    assert!(matches!(
        resolver.load_uri(&format!("{base}/3")),
        Err(AssetError::CacheMiss)
    ));
    assert!(matches!(
        resolver.load_uri(&format!("{base}/invalid")),
        Err(AssetError::Fuel(_))
    ));
    assert!(matches!(
        resolver.load_uri("model://missing-local-model"),
        Err(AssetError::NotFound(_))
    ));
    assert!(matches!(listener.accept(),Err(error) if error.kind()==std::io::ErrorKind::WouldBlock));
}

#[test]
fn local_candidates_precede_cached_fuel_and_missing_files_do_not_trigger_downloads() {
    let temp = TempDir::new().unwrap();
    let cache = temp.path().join("cache");
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    listener.set_nonblocking(true).unwrap();
    let port = listener.local_addr().unwrap().port();
    cached_model(&cache, port, 1, "cached");
    let mut config = ClientConfig {
        cache_location: cache.clone(),
        servers: vec![ServerConfig {
            url: format!("http://127.0.0.1:{port}"),
            ..Default::default()
        }],
        ..Default::default()
    };
    config.limits.request_timeout_ms = Some(100);
    let resolver = LocalResources::new(temp.path().into(), vec![], 4096)
        .unwrap()
        .with_cached_fuel(FuelClient::new(config));
    let suffix = format!("127.0.0.1:{port}/1.0/alice/models/robot/1");
    let shadow = temp.path().join(&suffix);
    fs::create_dir_all(shadow.parent().unwrap()).unwrap();
    fs::write(&shadow, "local").unwrap();
    let uri = format!("http://{suffix}");
    assert_eq!(resolver.load_uri(&uri).unwrap().xml, "local");
    fs::remove_file(shadow).unwrap();
    fs::remove_file(cache.join(format!("127.0.0.1%3A{port}/alice/models/robot/1/model.sdf")))
        .unwrap();
    assert!(
        matches!(resolver.load_uri(&uri),Err(AssetError::Io{source,..}) if source.kind()==std::io::ErrorKind::NotFound)
    );
    assert!(matches!(listener.accept(),Err(error) if error.kind()==std::io::ErrorKind::WouldBlock));
}

#[test]
fn cached_resource_selection_preserves_xml_budgets_and_metadata_only_errors() {
    let temp = TempDir::new().unwrap();
    let directory = temp.path().join("fuel.gazebosim.org/alice/models/robot/1");
    fs::create_dir_all(&directory).unwrap();
    fs::write(directory.join("metadata.pbtxt"), "name: 'robot'").unwrap();
    let client = FuelClient::new(ClientConfig {
        cache_location: temp.path().into(),
        ..Default::default()
    });
    let uri = "https://fuel.gazebosim.org/1.0/alice/models/robot/1";
    let id = client.parse(uri, ResourceKind::Model).unwrap();
    assert_eq!(client.cached(ResourceKind::Model, &id).unwrap(), directory);
    let resolver = LocalResources::new(temp.path().into(), vec![], 128)
        .unwrap()
        .with_cached_fuel(client);
    assert!(
        matches!(resolver.load_uri(uri), Err(AssetError::Io { source, .. }) if source.kind() == std::io::ErrorKind::NotFound)
    );
    fs::write(
        directory.join("model.config"),
        "<model><sdf version='1.12'>model.sdf</sdf></model>",
    )
    .unwrap();
    fs::write(directory.join("model.sdf"), "x".repeat(129)).unwrap();
    assert!(matches!(
        resolver.load_uri(uri),
        Err(AssetError::TooLarge(_))
    ));
    fs::write(directory.join("model.sdf"), [0xff]).unwrap();
    assert!(matches!(resolver.load_uri(uri), Err(AssetError::Utf8(_))));
}

#[cfg(unix)]
#[test]
fn producer_rejects_symlinked_cache_entry_without_treating_it_as_a_download_request() {
    let temp = TempDir::new().unwrap();
    let versions = temp.path().join("fuel.gazebosim.org/alice/models/robot");
    fs::create_dir_all(&versions).unwrap();
    let target = temp.path().join("target");
    fs::create_dir(&target).unwrap();
    fs::write(target.join("model.config"), "<model/>").unwrap();
    std::os::unix::fs::symlink(target, versions.join("1")).unwrap();
    let client = FuelClient::new(ClientConfig {
        cache_location: temp.path().into(),
        ..Default::default()
    });
    let resolver = LocalResources::new(temp.path().into(), vec![], 4096)
        .unwrap()
        .with_cached_fuel(client);
    assert!(matches!(
        resolver.load_uri("https://fuel.gazebosim.org/1.0/alice/models/robot/1"),
        Err(AssetError::CacheMiss)
    ));
}
