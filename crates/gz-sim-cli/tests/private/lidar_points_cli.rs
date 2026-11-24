// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use clap::CommandFactory;

#[test]
fn lidar_point_clouds_are_explicit_and_preserve_other_output_defaults() {
    let ordinary = Args::try_parse_from(["gz-sim-rs", "fixture.sdf"]).unwrap();
    assert!(!ordinary.lidar_point_clouds);
    let selected = Args::try_parse_from([
        "gz-sim-rs",
        "fixture.sdf",
        "--server",
        "--lidar-point-clouds",
    ])
    .unwrap();
    assert!(selected.server && selected.lidar_point_clouds);
    assert!(!selected.run && !selected.replay);
    assert!(!selected.camera_mixed_pair && !selected.camera_rgb_family);
    assert!(selected.camera_worker.is_none() && selected.web_viewer.is_none());
    let help = Args::command().render_long_help().to_string();
    assert!(help.contains("--lidar-point-clouds"));
    assert!(help.contains("organized CPU-lidar point clouds"));
}

#[test]
fn lidar_point_clouds_require_a_live_server_and_cannot_modify_replay() {
    assert_eq!(
        Args::try_parse_from(["gz-sim-rs", "fixture.sdf", "--lidar-point-clouds"])
            .err()
            .expect("offline mode has no publishers")
            .kind(),
        clap::error::ErrorKind::MissingRequiredArgument,
    );
    assert_eq!(
        Args::try_parse_from([
            "gz-sim-rs",
            "recording",
            "--server",
            "--web-viewer",
            "9014",
            "--replay",
            "--lidar-point-clouds",
        ])
        .err()
        .expect("replay is read-only")
        .kind(),
        clap::error::ErrorKind::ArgumentConflict,
    );
}
