// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn inapplicable_options_fail_before_recording() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let archive = dir.path().join("archive");
    for (options, message) in [
        (vec!["--pull", "5000,1,2"], "requires a monitored carry"),
        (vec!["--control-ms", "2"], "require a monitored carry"),
        (
            vec!["--fidelity", "adaptive"],
            "requires the clear-obstacle",
        ),
        (vec!["--parcel-kg", "0"], "mass must be"),
    ] {
        let args = Args::try_parse_from(
            ["serial_transfer", "transfer", archive.to_str().unwrap()]
                .into_iter()
                .chain(options),
        )?;
        let err = record(args, XML).unwrap_err();
        assert!(err.to_string().contains(message), "{err}");
        assert!(!archive.exists());
    }
    Ok(())
}

#[test]
fn existing_step_timings_are_preserved() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let archive = dir.path().join("recording");
    let timings = dir.path().join("prior-timings.json");
    std::fs::write(&timings, b"prior measurements")?;
    let args = Args::try_parse_from([
        "serial_transfer",
        "transfer",
        archive.to_str().unwrap(),
        "--timings",
        timings.to_str().unwrap(),
    ])?;
    let err = record(args, XML).unwrap_err();
    assert_eq!(
        err.downcast_ref::<std::io::Error>().unwrap().kind(),
        std::io::ErrorKind::AlreadyExists
    );
    assert_eq!(std::fs::read(timings)?, b"prior measurements");
    Ok(())
}
