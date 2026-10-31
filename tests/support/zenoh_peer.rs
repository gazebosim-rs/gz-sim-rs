// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Selected C++ fixture build metadata; network configuration stays case-owned.
use std::{
    path::{Path, PathBuf},
    process::Command,
};

const PROFILE: &str = "gazebo-zenoh-client-v1";

pub struct NativePeer {
    pub binary: PathBuf,
    prefix: PathBuf,
    vendor_prefix: PathBuf,
    provenance: PathBuf,
}

impl NativePeer {
    pub fn build(root: &Path, client: &str) -> Self {
        let output = Command::new("python3")
            .arg(root.join("scripts/native-peer.py"))
            .args(["--client", client, "--profile", PROFILE, "--json"])
            .output()
            .unwrap();
        assert!(output.status.success(), "{output:?}");
        let metadata: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(metadata["profile"].as_str(), Some(PROFILE));
        let path = |name: &str, directory: bool| {
            let value = PathBuf::from(metadata[name].as_str().unwrap());
            assert!(value.is_absolute(), "{name} is not absolute: {value:?}");
            assert!(
                if directory {
                    value.is_dir()
                } else {
                    value.is_file()
                },
                "missing selected {name}: {value:?}"
            );
            value
        };
        let peer = Self {
            binary: path("binary", false),
            prefix: path("prefix", true),
            vendor_prefix: path("vendor_prefix", true),
            provenance: path("provenance", false),
        };
        let provenance: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&peer.provenance).unwrap()).unwrap();
        assert_eq!(provenance["profile"].as_str(), Some(PROFILE));
        assert_eq!(provenance["client"].as_str(), Some(client));
        peer
    }

    pub fn configure_runtime(&self, command: &mut Command) {
        assert!(self.provenance.is_file(), "native provenance disappeared");
        command
            .env("GZ_TRANSPORT_IMPLEMENTATION", "zenoh")
            .env(
                "LD_LIBRARY_PATH",
                std::env::join_paths([self.prefix.join("lib"), self.vendor_prefix.join("lib")])
                    .unwrap(),
            )
            .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
            .env_remove("LD_PRELOAD")
            .env_remove("LD_AUDIT");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    struct Fixture {
        root: tempfile::TempDir,
        metadata: serde_json::Value,
    }
    impl Fixture {
        fn new() -> Self {
            let root = tempfile::tempdir().unwrap();
            let directory = root.path().join("selected reference with spaces");
            std::fs::create_dir_all(directory.join("prefix/lib")).unwrap();
            std::fs::create_dir_all(directory.join("vendor/lib")).unwrap();
            std::fs::write(directory.join("client"), b"nonexecuted fixture").unwrap();
            std::fs::write(
                directory.join("provenance.json"),
                json!({"profile":PROFILE,"client":"clock"}).to_string(),
            )
            .unwrap();
            let metadata = json!({
                "profile":PROFILE, "binary":directory.join("client"),
                "prefix":directory.join("prefix"),"vendor_prefix":directory.join("vendor"),
                "provenance":directory.join("provenance.json")
            });
            std::fs::create_dir(root.path().join("scripts")).unwrap();
            // This is a controlled JSON producer, not a native build or SDK proof.
            std::fs::write(root.path().join("scripts/native-peer.py"),
                "import pathlib,sys\nassert sys.argv[1:] == ['--client','clock','--profile','gazebo-zenoh-client-v1','--json']\nprint((pathlib.Path(__file__).parent.parent/'metadata.json').read_text())\n").unwrap();
            Self { root, metadata }
        }
        fn build(&self) -> NativePeer {
            std::fs::write(
                self.root.path().join("metadata.json"),
                self.metadata.to_string(),
            )
            .unwrap();
            NativePeer::build(self.root.path(), "clock")
        }
    }

    #[test]
    fn selected_metadata_controls_runtime_without_overwriting_case_config() {
        let fixture = Fixture::new();
        let peer = fixture.build();
        assert_eq!(
            peer.binary,
            Path::new(fixture.metadata["binary"].as_str().unwrap())
        );
        let mut command = Command::new(&peer.binary);
        command
            .env("ZENOH_CONFIG", "case-owned.json")
            .env("LD_LIBRARY_PATH", "unselected")
            .env("GZ_TRANSPORT_IMPLEMENTATION", "zeromq")
            .env("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE", "unselected")
            .env("LD_PRELOAD", "unselected")
            .env("LD_AUDIT", "unselected");
        peer.configure_runtime(&mut command);
        let env: std::collections::BTreeMap<_, _> = command.get_envs().collect();
        let expected =
            std::env::join_paths([peer.prefix.join("lib"), peer.vendor_prefix.join("lib")])
                .unwrap();
        assert_eq!(
            env[std::ffi::OsStr::new("LD_LIBRARY_PATH")],
            Some(expected.as_os_str())
        );
        assert_eq!(
            env[std::ffi::OsStr::new("ZENOH_CONFIG")],
            Some(std::ffi::OsStr::new("case-owned.json"))
        );
        assert_eq!(
            env[std::ffi::OsStr::new("GZ_TRANSPORT_IMPLEMENTATION")],
            Some(std::ffi::OsStr::new("zenoh"))
        );
        for name in [
            "GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE",
            "LD_PRELOAD",
            "LD_AUDIT",
        ] {
            assert_eq!(env[std::ffi::OsStr::new(name)], None);
        }
    }

    #[test]
    fn mismatched_or_unusable_selected_metadata_is_rejected() {
        for invalid in ["profile", "relative", "missing", "client"] {
            let mut fixture = Fixture::new();
            match invalid {
                "profile" => fixture.metadata["profile"] = json!("transport-client-v1"),
                "relative" => fixture.metadata["binary"] = json!("relative-client"),
                "missing" => {
                    std::fs::remove_file(fixture.metadata["binary"].as_str().unwrap()).unwrap()
                }
                "client" => std::fs::write(
                    fixture.metadata["provenance"].as_str().unwrap(),
                    json!({"profile":PROFILE,"client":"rate"}).to_string(),
                )
                .unwrap(),
                _ => unreachable!(),
            }
            assert!(
                std::panic::catch_unwind(|| fixture.build()).is_err(),
                "{invalid}"
            );
        }
    }
}
