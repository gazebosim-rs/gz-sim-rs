// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use clap::CommandFactory;

#[test]
fn rgb_family_is_an_explicit_default_off_cli_request() {
    assert!(
        !Args::try_parse_from(["gz-sim-rs", "fixture.sdf"])
            .unwrap()
            .camera_rgb_family
    );
    let args = Args::try_parse_from([
        "gz-sim-rs",
        "fixture.sdf",
        "--server",
        "--camera-worker",
        "/not/executed/worker",
        "--camera-rgb-family",
    ])
    .unwrap();
    assert!(args.camera_rgb_family);
    assert!(!args.camera_mixed_pair);
    assert_eq!(
        args.camera_worker,
        Some(PathBuf::from("/not/executed/worker"))
    );
}

#[test]
fn rgb_family_requires_worker_server_and_excludes_mixed_pair() {
    for (argv, kind) in [
        (
            vec!["gz-sim-rs", "fixture.sdf", "--camera-rgb-family"],
            clap::error::ErrorKind::MissingRequiredArgument,
        ),
        (
            vec![
                "gz-sim-rs",
                "fixture.sdf",
                "--camera-worker",
                "/worker",
                "--camera-rgb-family",
            ],
            clap::error::ErrorKind::MissingRequiredArgument,
        ),
        (
            vec![
                "gz-sim-rs",
                "fixture.sdf",
                "--server",
                "--camera-worker",
                "/worker",
                "--camera-rgb-family",
                "--camera-mixed-pair",
            ],
            clap::error::ErrorKind::ArgumentConflict,
        ),
    ] {
        assert_eq!(
            Args::try_parse_from(argv)
                .err()
                .expect("invalid CLI request")
                .kind(),
            kind
        );
    }
}

#[test]
fn rgb_family_help_names_the_closed_camera_count() {
    let help = Args::command().render_long_help().to_string();
    assert!(help.contains("--camera-rgb-family"));
    assert!(help.contains("three") && help.contains("four"));
}
