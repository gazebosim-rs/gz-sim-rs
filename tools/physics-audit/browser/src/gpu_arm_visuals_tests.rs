// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;

fn sample(time: u64, angle: f64) -> (Value, Value) {
    let mut arms = Vec::new();
    let mut visuals = Vec::new();
    for id in 0..9 {
        let base = [id as f64 * 0.7, 0., 0.1];
        let a = angle + id as f64 * 0.01;
        let b = -0.72;
        let elbow = [base[0] + a.cos(), base[1], base[2] + a.sin()];
        let tip = [
            elbow[0] + 0.75 * (a + b).cos(),
            elbow[1],
            elbow[2] + 0.75 * (a + b).sin(),
        ];
        arms.push(json!({"base":base,"elbow":elbow,"tip":tip,"angles":[[a.cos(),a.sin()],[b.cos(),b.sin()]],"lane_id":id}));
        for (part, pos) in [("base", base), ("tool", tip)] {
            visuals.push(json!({"name":format!("default::arm_{id}::{part}"),"translation":pos}));
        }
        for (part, start, end, angle, length) in [
            ("upper", base, elbow, a, 1.),
            ("forearm", elbow, tip, a + b, 0.75),
        ] {
            let center = std::array::from_fn::<_, 3, _>(|i| 0.5 * (start[i] + end[i]));
            visuals.push(json!({
                "name":format!("default::arm_{id}::{part}"),
                "translation":center,
                "rotation":[0.,(-0.5*angle).sin(),0.,(-0.5*angle).cos()],
                "shape":{"kind":"box","size":[length,0.1,0.1]}
            }));
        }
    }
    (
        json!({"ns":time,"scene":{"visuals":visuals}}),
        json!({"time_ns":time,"arms":arms}),
    )
}

#[test]
fn matches_rotations_composed_from_both_joints_in_all_lanes() -> Result<()> {
    let (a, s) = sample(0, 1.1);
    let (b, t) = sample(100_000, -0.5);
    verify(&[a, b], &[s, t])
}

#[test]
fn catches_clock_binding_and_kinematic_errors() {
    let (f, s) = sample(0, 0.);
    let verify_row = |f: Value, s: Value| verify(&[f], &[s]);
    assert!(verify(&[], &[]).is_err());
    for (pointer, value) in [
        ("/time_ns", json!(1)),
        ("/time_ns", Value::Null),
        ("/arms/4/base", Value::Null),
        ("/arms/4/elbow", Value::Null),
        ("/arms/4/tip", Value::Null),
        ("/arms/4/angles/0", Value::Null),
        ("/arms/4/angles/1", Value::Null),
        ("/arms/4/lane_id", json!(3)),
        ("/arms/4/lane_id", Value::Null),
        ("/arms/4/angles/0", json!([2., 0.])),
        ("/arms/4/angles/1", json!([0., 0.])),
        ("/arms/4/angles/1", json!([1., 0.])),
        ("/arms/4/base", json!([3., 0., 0.])),
        ("/arms/4/elbow", json!([1., 0., 0.])),
        ("/arms/4/tip", json!([1., 0., 0.])),
        ("/arms", json!([])),
        ("/arms", Value::Null),
    ] {
        let mut bad = s.clone();
        *bad.pointer_mut(pointer).unwrap() = value;
        assert!(verify_row(f.clone(), bad).is_err(), "{pointer}");
    }
    for (pointer, value) in [
        ("/scene", Value::Null),
        ("/scene/visuals/0/name", Value::Null),
        ("/scene/visuals/0/name", json!("default::arm_0::tool")),
        ("/scene/visuals/0/name", json!("another base")),
        ("/scene/visuals/3/translation", json!([0., 0., 0.])),
        ("/scene/visuals/0/translation", Value::Null),
        ("/scene/visuals/2/translation", Value::Null),
        ("/scene/visuals/2/shape/size", Value::Null),
        ("/scene/visuals/2/rotation", Value::Null),
        ("/scene/visuals/2/rotation", json!([1., 0., 0., 0.])),
        ("/scene/visuals/2/rotation", json!([0., 0., 1., 0.])),
        ("/scene/visuals/2/rotation", json!([0., 0., 0., 2.])),
        ("/scene/visuals/2/rotation", json!([0., 1., 0., 0.])),
        ("/scene/visuals/2/shape/kind", json!("sphere")),
        ("/scene/visuals/2/shape/size/0", json!(0.)),
    ] {
        let mut bad = f.clone();
        *bad.pointer_mut(pointer).unwrap() = value;
        assert!(verify_row(bad, s.clone()).is_err(), "{pointer}");
    }
    let (mut moved, mut source) = (f.clone(), s.clone());
    for key in ["base", "elbow", "tip"] {
        let y = &mut source["arms"][0][key][1];
        *y = json!(y.as_f64().unwrap() + 1.);
    }
    for v in moved["scene"]["visuals"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .take(4)
    {
        let y = &mut v["translation"][1];
        *y = json!(y.as_f64().unwrap() + 1.);
    }
    assert!(verify(&[f, moved], &[s, source]).is_err());
}
