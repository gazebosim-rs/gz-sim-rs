// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify the finite CPU fleet display patch against public observations and the native archive.
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, error::Error, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-12, "measured {a}, expected {b}");
}
fn pose(v: &Value) -> [f64; 3] {
    serde_json::from_value(v.clone()).unwrap()
}

#[test]
#[ignore = "set GZ_FLEET_PATCH to the 100,000-deck archive directory"]
fn public_observations_match_the_finite_display_patch() -> Result<()> {
    let input = env::var_os("GZ_FLEET_PATCH").ok_or("set GZ_FLEET_PATCH")?;
    let dir = Path::new(&input);
    let cpu: Value = serde_json::from_slice(&fs::read(dir.join("index.json"))?)?;
    assert_eq!(cpu["decks"], 100000);
    assert_eq!(cpu["payloads"], 1000);
    assert_eq!(cpu["public_step_ns"], 10_000_000);
    let origin = pose(&cpu["display_origin_m"]);
    let rows = cpu["frames"].as_array().unwrap();
    let text = fs::read_to_string(dir.join("replay/frames.jsonl"))?;
    let records = text
        .lines()
        .map(serde_json::from_str)
        .collect::<std::result::Result<Vec<Value>, _>>()?;
    let frames = records
        .iter()
        .filter(|r| r["kind"] == "frame")
        .collect::<Vec<_>>();
    assert_eq!(frames.len(), 51);
    assert_eq!(rows.len(), frames.len());
    let mut payload_start = None;
    let mut payload_end = None;
    let mut max_command_error = 0_f64;
    for (i, (sample, frame)) in rows.iter().zip(frames).enumerate() {
        let tick = sample["tick"].as_u64().unwrap();
        assert_eq!(tick, i as u64 * 4);
        let ns = tick * 10_000_000;
        assert_eq!(frame["ordinal"], i + 1);
        let scene: Value = serde_json::from_str(frame["scene"].as_str().unwrap())?;
        assert_eq!(
            scene["seconds"].as_u64().unwrap() * 1_000_000_000
                + scene["nanoseconds"].as_u64().unwrap(),
            ns
        );
        let visuals = scene["visuals"].as_array().unwrap();
        let by_name: BTreeMap<_, _> = visuals
            .iter()
            .map(|v| (v["name"].as_str().unwrap(), v))
            .collect();
        let obs: BTreeMap<_, _> = sample["poses"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| (v["name"].as_str().unwrap(), v))
            .collect();
        assert_eq!(visuals.len(), 11);
        assert_eq!(by_name.len(), 11);
        assert_eq!(obs.len(), 10);
        for (name, expected_size) in (0..3)
            .flat_map(|row| {
                (0..3).map(move |col| (format!("deck_{}", 317 * row + col), [1., 1., 0.2]))
            })
            .chain([(String::from("payload_0"), [0.2; 3])])
        {
            let measured = obs[name.as_str()];
            let world = pose(&measured["world_position_m"]);
            let native = by_name[format!("default::{name}::visual").as_str()];
            let local = pose(&native["translation"]);
            for j in 0..3 {
                near(world[j], local[j] + origin[j]);
            }
            assert_eq!(native["rotation"], measured["rotation"]);
            assert_eq!(native["shape"]["kind"], "box");
            assert_eq!(pose(&native["shape"]["size"]), expected_size);
            if let Some(label) = name.strip_prefix("deck_") {
                let deck: usize = label.parse()?;
                let command = [
                    (deck % 317) as f64 * 1.1 + ns as f64 * 1e-9 * 0.25,
                    (deck / 317) as f64 * 1.1,
                    0.,
                ];
                for j in 0..3 {
                    max_command_error = max_command_error.max((world[j] - command[j]).abs());
                    near(world[j], command[j]);
                }
            } else {
                payload_start.get_or_insert(world);
                payload_end = Some(world);
            }
        }
        let marker = by_name["default::stationary_origin_marker::visual"];
        assert_eq!(pose(&marker["translation"]), [-1.1, -1.77, 0.]);
        assert_eq!(marker["rotation"], json!([0., 0., 0., 1.]));
        assert_eq!(pose(&marker["shape"]["size"]), [0.025, 0.17, 0.13]);
        if tick == 0 {
            assert!(sample["imu_specific_force_mps2"].is_null());
        } else {
            assert_eq!(pose(&sample["imu_specific_force_mps2"]), [0., 0., 9.81]);
        }
    }
    let start = payload_start.unwrap();
    let end = payload_end.unwrap();
    let travel = end[0] - start[0];
    let summary = json!({
        "frames":rows.len(),
        "sampled_deck_poses":9*rows.len(),
        "sampled_payload_poses":rows.len(),
        "static_marker_checks":rows.len(),
        "imu_observations":rows.len()-1,
        "maximum_deck_command_error_m":max_command_error,
        "payload_travel_x_m":travel,
        "payload_lag_behind_deck_m":0.5-travel,
        "final_payload_center_height_m":end[2],
    });
    println!("{summary}");
    if let Some(output) = env::var_os("GZ_FLEET_PATCH_AUDIT") {
        fs::write(output, serde_json::to_vec_pretty(&summary)?)?;
    }
    Ok(())
}
