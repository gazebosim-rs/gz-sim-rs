// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[test]
fn replay_and_capture_are_explicit_and_conflicting_native_options_are_refused() {
    let args = Args::try_parse_from([
        "gz-sim-rs",
        "capture",
        "--server",
        "--web-viewer",
        "9012",
        "--replay",
    ])
    .unwrap();
    assert!(args.replay);
    assert!(args.record_scene.is_none());
    let args = Args::try_parse_from([
        "gz-sim-rs",
        "world.sdf",
        "--server",
        "--web-viewer",
        "9011",
        "--web-lifecycle",
        "--record-scene",
        "capture",
    ])
    .unwrap();
    assert_eq!(args.record_scene, Some(PathBuf::from("capture")));
    assert!(!args.replay);
    for extra in [
        vec!["--run"],
        vec!["--iterations", "1"],
        vec!["--step-ns", "1000"],
        vec!["--web-lifecycle"],
        vec!["--record-scene", "other"],
        vec!["--resource-path", "assets"],
        vec!["--camera-worker", "/tmp/worker"],
    ] {
        let mut argv = vec![
            "gz-sim-rs",
            "capture",
            "--server",
            "--web-viewer",
            "9012",
            "--replay",
        ];
        argv.extend(extra);
        assert!(Args::try_parse_from(argv).is_err());
    }
    assert!(Args::try_parse_from(["gz-sim-rs", "world.sdf", "--record-scene", "capture"]).is_err());
}
