// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check that the readback tool detects a moved finger and a missing impact frame.
use serde_json::{Value, json};
use std::{error::Error, fs};

#[test]
fn readback_requires_the_measured_pose_and_first_contact_frame() -> Result<(), Box<dyn Error>> {
    let dir = std::env::temp_dir().join(format!("gz-powered-slider-view-{}", std::process::id()));
    fs::create_dir(&dir)?;
    let trace = dir.join("trace.jsonl");
    let mut rows = vec![json!({"step_ns":33_400})];
    let mut archive = Vec::new();
    let mut sphere_x = -0.02405;
    let mut speed = 0.75;
    for tick in 0..4 {
        let time = tick * 33_400;
        let impulse = if tick == 2 { 0.07 } else { 0. };
        if tick > 0 {
            sphere_x += speed * 33_400e-9;
            speed -= impulse / 0.20;
        }
        rows.push(
            json!({"time_ns":time,"held_joint_forces":[[11,2.]],"scalar_joints":[
            {"id":10,"position":0.},{"id":11,"position":0.}],"delivered_normal_ns":impulse}),
        );
        let visuals = [
            ("wrist_rig::wrist::hub",[0.,0.,0.1]),("wrist_rig::wrist::guide",[0.,0.052,0.108]),
            ("wrist_rig::finger::finger",[0.,0.1,0.1]),("parcel::body::sphere",[sphere_x,0.12,0.1])
        ].map(|(name,translation)|json!({"name":format!("default::{name}"),"translation":translation,"rotation":[0.,0.,0.,1.]}));
        archive.push(json!({"seconds":0,"nanoseconds":time,"visuals":visuals}));
    }
    fs::write(
        &trace,
        rows.iter()
            .map(Value::to_string)
            .collect::<Vec<_>>()
            .join("\n"),
    )?;
    let run = |frames: &[Value]| -> Result<_, Box<dyn Error>> {
        let data = frames
            .iter()
            .map(|scene| json!({"kind":"frame","scene":scene.to_string()}).to_string())
            .collect::<Vec<_>>()
            .join("\n");
        fs::write(dir.join("frames.jsonl"), data)?;
        super::check(&trace, &dir, &dir.join("readback"))
    };
    run(&archive)?;
    let receipt: Value =
        serde_json::from_slice(&fs::read(dir.join("readback/measurements.json"))?)?;
    assert_eq!(receipt["first_contact_ns"], 66_800);
    let mut bad = archive.clone();
    bad[1]["visuals"][2]["translation"][0] = 0.003.into();
    assert!(
        run(&bad)
            .unwrap_err()
            .to_string()
            .contains("native replay errors")
    );
    archive.remove(2);
    assert!(
        run(&archive)
            .unwrap_err()
            .to_string()
            .contains("cadence missed")
    );
    fs::remove_dir_all(dir)?;
    Ok(())
}
