// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct the downward sensor in the 100,000-deck native launch recording.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, error::Error, fs, path::PathBuf};

fn vector(v: &Value) -> DVec3 {
    DVec3::from_array(serde_json::from_value(v.clone()).unwrap())
}

// Slab intersection in the independently measured payload's local frame.
fn payload_distance(origin: DVec3, center: DVec3, orientation: [f64; 4]) -> Option<f64> {
    let inverse = DQuat::from_array(orientation).conjugate();
    let o = inverse * (origin - center);
    let d = inverse * -DVec3::Z;
    let (mut enter, mut leave) = (0_f64, 2_f64);
    for i in 0..3 {
        if d[i] == 0. {
            if o[i].abs() > 0.1 {
                return None;
            }
        } else {
            let a = (-0.1 - o[i]) / d[i];
            let b = (0.1 - o[i]) / d[i];
            enter = enter.max(a.min(b));
            leave = leave.min(a.max(b));
        }
    }
    (enter <= leave).then_some(enter)
}

#[test]
#[ignore = "set GZ_FLEET_PATCH and GZ_FLEET_LAUNCH_TRACE to independent ordinary CPU outputs"]
fn first_solid_return_matches_oriented_cargo_and_deck() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(env::var("GZ_FLEET_PATCH")?);
    let input: Value = serde_json::from_slice(&fs::read(dir.join("index.json"))?)?;
    let trace: Value = serde_json::from_slice(&fs::read(env::var("GZ_FLEET_LAUNCH_TRACE")?)?)?;
    assert_eq!(input["decks"], 100000);
    assert_eq!(input["payloads"], 1000);
    let mut derived = Vec::new();
    let mut deck_returns = 0;
    let mut payload_returns = 0;
    let mut max_error = 0_f64;
    let mut imu = 0;
    let display_origin = vector(&input["display_origin_m"]);
    let identity = [0., 0., 0., 1.];
    for sample in input["frames"].as_array().unwrap() {
        let tick = sample["tick"].as_u64().unwrap();
        let mut poses: BTreeMap<String, Value> = BTreeMap::new();
        let mut add = |name: &str, p: DVec3, q| {
            poses.insert(
                format!("default::{name}::visual"),
                json!({"translation": (p-display_origin).to_array(),"rotation":q}),
            );
        };
        let (payload, rotation) = if tick == 0 {
            (DVec3::new(0., 0., 0.201), identity)
        } else {
            let load = &trace["primary"][tick as usize - 1];
            assert_eq!(load["tick"], tick);
            assert_eq!(sample["imu_specific_force_mps2"], json!([0., 0., 9.81]));
            imu += 1;
            (
                vector(&load["position_m"]),
                serde_json::from_value(load["rotation"].clone())?,
            )
        };
        add("payload_0", payload, rotation);
        let x = tick as f64 * 0.0025;
        for row in 0..3 {
            for column in 0..4 {
                let name = format!("deck_{}", row * 317 + column);
                let center = DVec3::new(column as f64 * 1.1 + x, row as f64 * 1.1, 0.);
                add(&name, center, identity);
            }
        }
        let origin = DVec3::new(x, 0., 1.);
        let distance = payload_distance(origin, payload, rotation)
            .unwrap_or(0.9)
            .min(0.9);
        if distance < 0.9 {
            payload_returns += 1;
        } else {
            deck_returns += 1;
        }
        let hit = origin - DVec3::Z * distance;
        for (field, value) in [("origin", origin), ("hit", hit)] {
            let error = (value - vector(&sample["range"][field]))
                .abs()
                .max_element();
            max_error = max_error.max(error);
            assert!(error < 1e-11);
        }
        assert!((sample["range"]["distance_m"].as_f64().unwrap() - distance).abs() < 1e-11);
        for (name, p) in [
            ("range_origin", origin),
            ("range_hit", hit),
            (
                "stationary_origin_marker",
                display_origin + DVec3::new(-1.1, -1.77, 0.),
            ),
        ] {
            add(name, p, identity);
        }
        derived.push(json!({"time_ns":tick*10_000_000,"poses":poses}));
    }
    assert_eq!(derived.len(), 101);
    assert!(deck_returns > 10 && payload_returns > 10);
    fs::write(
        dir.join("visual-reference.json"),
        serde_json::to_vec(&derived)?,
    )?;
    println!(
        "{}",
        json!({"frames":derived.len(),"displayed_poses":derived.len()*16,
        "deck_returns":deck_returns,"payload_returns":payload_returns,"imu":imu,"max_error_m":max_error})
    );
    Ok(())
}
