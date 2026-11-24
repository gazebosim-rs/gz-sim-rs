// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;

fn snapshot() -> (Vec<Value>, Vec<Value>, Vec<String>) {
    let names = vec!["head".to_string(), "tail".to_string()];
    let sphere = |name, z| json!({"name":name,"shape":{"kind":"sphere"},"translation":[0.,0.,z]});
    let frames = vec![json!({"ns":10,"scene":{"visuals":[sphere("tail",0.),sphere("head",0.5)]}})];
    let samples = vec![json!({"time_ns":10,"positions":[[0.,0.,0.5],[0.,0.,0.]]})];
    (frames, samples, names)
}

#[test]
fn named_centers_match_particle_order_and_clock() {
    let (frames, mut samples, names) = snapshot();
    verify(&frames, &samples, names.clone()).unwrap();
    samples[0]["time_ns"] = json!(9);
    assert!(verify(&frames, &samples, names.clone()).is_err());
    samples[0]["time_ns"] = Value::Null;
    assert!(verify(&frames, &samples, names).is_err());
}

#[test]
fn incomplete_or_duplicate_bindings_and_captures_fail() {
    let (frames, samples, names) = snapshot();
    assert!(verify(&frames, &samples, vec![]).is_err());
    assert!(verify(&frames, &samples, vec![names[0].clone(); 2]).is_err());
    assert!(verify(&frames, &[], names.clone()).is_err());
    assert!(verify(&[], &[], names.clone()).is_err());
    assert!(verify(&frames, &samples, vec![names[0].clone()]).is_err());
    assert!(verify(&frames, &samples, vec![names[0].clone(), "other".into()]).is_err());
}

#[test]
fn wrong_shapes_positions_and_ambiguous_names_fail() {
    let (mut frames, mut samples, names) = snapshot();
    samples[0]["positions"][0][0] = json!(0.001);
    assert!(verify(&frames, &samples, names.clone()).is_err());
    samples[0]["positions"][0] = json!([0, 0]);
    assert!(verify(&frames, &samples, names.clone()).is_err());
    samples[0]["positions"] = Value::Null;
    assert!(verify(&frames, &samples, names.clone()).is_err());
    let (_, samples, _) = snapshot();
    frames[0]["scene"]["visuals"][1]["translation"] = Value::Null;
    assert!(verify(&frames, &samples, names.clone()).is_err());
    frames[0]["scene"]["visuals"][1]["translation"] = json!([0., 0., 0.5]);
    frames[0]["scene"]["visuals"][1]["shape"]["kind"] = json!("box");
    assert!(verify(&frames, &samples, names.clone()).is_err());
    frames[0]["scene"]["visuals"][1]["shape"]["kind"] = json!("sphere");
    let duplicate = frames[0]["scene"]["visuals"][0].clone();
    frames[0]["scene"]["visuals"]
        .as_array_mut()
        .unwrap()
        .push(duplicate);
    assert!(verify(&frames, &samples, names.clone()).is_err());
    frames[0]["scene"]["visuals"] = Value::Null;
    assert!(verify(&frames, &samples, names).is_err());
}
