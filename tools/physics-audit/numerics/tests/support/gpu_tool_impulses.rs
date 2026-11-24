// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;
fn sample() -> (Value, Value) {
    let keys = SOLVERS.map(|(key, _, _)| key);
    let mut case = json!({"lane_id":4,"friction":0.2,"payload_kg":0.1});
    let mut trace = vec![];
    for tick in 1..=20 {
        let mut record = json!({"lane_id":4});
        for (i, k) in keys.iter().enumerate() {
            let normal = if (4 + i..=14 + i).contains(&tick) {
                0.02
            } else {
                0.
            };
            record[k] = json!([
                0.4,
                0.08,
                normal,
                -0.1 * normal,
                if normal > 0. { Some(0.0004) } else { None }
            ]);
            case[k] = json!({"total_normal_ns":0.22,"total_tangent_ns":-0.022});
        }
        trace.push(json!({"tick":tick,"cases":[record]}));
    }
    (
        json!({"ticks":20,"cases":[case],"lift_before_tick":14}),
        json!(trace),
    )
}
#[test]
fn compares_public_impulses_in_overlapping_solver_windows() -> Result<()> {
    let (r, t) = sample();
    let s = read(&r, &t, 4)?;
    assert_eq!(s.first, [4, 5, 6]);
    assert_eq!(s.last, [14, 15, 16]);
    assert_eq!(window(s.first, 20), 1..=14);
    assert_eq!(window(s.last, 20), 9..=20);
    let svg = draw(&s)?;
    for expected in [
        "touchdown",
        "Lift-off",
        "sideways",
        "Fine Cartesian",
        "mN·s",
        "0.10 kg",
    ] {
        assert!(svg.contains(expected));
    }
    Ok(())
}
#[test]
fn rejects_gaps_unmatched_lanes_and_inconsistent_trial_totals() {
    let (r, t) = sample();
    assert!(read(&r, &t, 7).is_err());
    assert!(read(&r, &Value::Null, 4).is_err());
    for (p, val) in [
        ("/ticks", json!(22)),
        ("/cases", Value::Null),
        ("/cases/0/friction", Value::Null),
        ("/cases/0/payload_kg", Value::Null),
        ("/lift_before_tick", Value::Null),
        ("/cases/0/gpu/total_normal_ns", json!(2.)),
        ("/cases/0/gpu/total_tangent_ns", Value::Null),
    ] {
        let mut r = r.clone();
        *r.pointer_mut(p).unwrap() = val;
        assert!(read(&r, &t, 4).is_err(), "{p}");
    }
    for (p, val) in [
        ("/0/tick", json!(2)),
        ("/0/cases", json!([])),
        ("/0/cases/0/gpu/2", json!(-1.)),
        ("/0/cases/0/rapier/3", Value::Null),
    ] {
        let mut bad = t.clone();
        *bad.pointer_mut(p).unwrap() = val;
        assert!(read(&r, &bad, 4).is_err(), "{p}");
    }
    let mut empty = t;
    for e in empty.as_array_mut().unwrap() {
        e["cases"][0]["rapier"][2] = json!(0.);
    }
    assert!(read(&r, &empty, 4).is_err());
}
