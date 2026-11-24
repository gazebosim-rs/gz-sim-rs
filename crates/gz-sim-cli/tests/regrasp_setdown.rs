// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check the same moving robot, actuator commands and traces as the public placement example.
#[allow(dead_code)]
#[path = "../examples/regrasp_setdown.rs"]
mod experiment;
use Case::*;
use bevy_tasks::{ComputeTaskPool, TaskPoolBuilder};
use experiment::{Case, Grid};
use serde_json::Value;
use std::{fs, path::Path};
fn trial(case: Case, grid: Grid) -> (Value, Vec<Value>) {
    ComputeTaskPool::get_or_init(|| TaskPoolBuilder::new().num_threads(1).build());
    let root = tempfile::tempdir().unwrap();
    let out = root.path().join("run");
    let result = experiment::run(case, grid, &out).unwrap();
    let rows = fs::read_to_string(out.join("ticks.jsonl"))
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    (result, rows)
}
fn field(row: &Value, key: &str) -> f64 {
    row[key].as_f64().unwrap()
}
fn idx(summary: &Value, key: &str) -> usize {
    summary[key].as_u64().unwrap() as usize - 1
}
#[test]
fn guiding_the_parcel_onto_its_stand_transfers_its_weight_before_release() {
    for case in [Nominal, Brake, High, Heavy, Mirror, Dropout] {
        let (s, rows) = trial(case, Grid::Adaptive);
        let (loss, hit, opened, clear) = (
            idx(&s, "handoff_tick"),
            idx(&s, "arrival_tick"),
            idx(&s, "opening_tick"),
            idx(&s, "clear_tick"),
        );
        assert!(loss < hit && hit < opened && opened < clear);
        assert!(
            rows[loss]["normal_force_n"]
                .as_array()
                .unwrap()
                .iter()
                .any(|f| f.as_f64().unwrap() < 0.05)
        );
        // Both jaws physically bear load for the last ten milliseconds of descent.
        for row in &rows[hit - 50..hit] {
            assert!(
                row["normal_force_n"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|v| v.as_f64().unwrap() > 0.1)
            );
        }
        let weight = field(&s, "parcel_mass_kg") * 9.81;
        for row in &rows[opened - 99..=opened] {
            assert_eq!(row["support_valid"], true);
            assert!((0.8 * weight..=1.5 * weight).contains(&field(row, "receiver_force_n")));
            assert!(field(row, "deck_margin_m") > 0.0015);
        }
        let last = rows.last().unwrap();
        assert_eq!(last["state"], "Placed");
        assert!(
            field(last, "jaw_clearance_m") > 0.0005
                && field(last, "pad_z_m") > field(&rows[clear], "pad_z_m") + 0.010
        );
        assert!(
            (field(last, "receiver_force_n") - weight).abs() < 0.002
                && field(last, "parcel_vz_m_s").abs() < 0.0001
        );
        if case == Dropout {
            assert_eq!(
                rows.iter().filter(|r| r["support_valid"] == false).count(),
                35
            );
        }
    }
}
#[test]
fn missing_unreachable_wrong_and_edge_supports_never_authorize_opening() {
    for case in [Missing, Low, Wrong, Edge] {
        let (s, rows) = trial(case, Grid::Adaptive);
        assert!(s["opening_tick"].is_null() && s["clear_tick"].is_null());
        assert_eq!(s["final"]["state"], "Hold");
        assert!(
            s["final"]["jaw_force_n"]
                .as_array()
                .unwrap()
                .iter()
                .all(|v| v.as_f64().unwrap() > 0.)
        );
        match case {
            Wrong => assert!(rows.iter().any(|r| field(r, "obstacle_force_n") > 1.96)),
            Edge => assert!(
                rows.iter()
                    .any(|r| field(r, "deck_margin_m") < 0.0015
                        && field(r, "receiver_force_n") > 1.96)
            ),
            _ => assert!(rows.iter().all(|r| field(r, "receiver_force_n") == 0.)),
        }
    }
}
#[test]
fn recording_claim_preserves_existing_files() {
    let root = tempfile::tempdir().unwrap();
    let keep = root.path().join("keep");
    fs::write(&keep, b"existing").unwrap();
    for p in [root.path(), Path::new(""), &keep.join("child")] {
        assert!(experiment::run(Nominal, Grid::Adaptive, p).is_err());
    }
    assert_eq!(fs::read(keep).unwrap(), b"existing");
}
#[test]
#[ignore = "complete placement records with 1 microsecond continuous reference ticks"]
fn fine_physics_reaches_the_same_accept_or_hold_decision() {
    let cases = [
        Nominal, Brake, High, Low, Heavy, Mirror, Dropout, Missing, Wrong, Edge, Jammed,
    ];
    std::thread::scope(|workers| {
        for group in cases.chunks(3) {
            workers.spawn(move || {
                for &case in group {
                    compare_grids(case);
                }
            });
        }
    });
}

fn compare_grids(case: Case) {
    let (a, ar) = trial(case, Grid::Adaptive);
    let (f, fr) = trial(case, Grid::Fine);
    assert_eq!(a["final"]["state"], f["final"]["state"], "{case:?}");
    assert_eq!(ar.len(), fr.len());
    assert!(fr.iter().all(|r| r["native_steps"] == 200));
    if let Some(k) = a["opening_tick"].as_u64() {
        assert!(
            k.abs_diff(f["opening_tick"].as_u64().unwrap()) <= 30,
            "release time for {case:?}"
        );
    } else {
        assert!(f["opening_tick"].is_null());
    }
}

#[test]
fn permanently_open_jaw_distinguishes_a_drop_onto_the_receiver_from_placement() {
    let (summary, rows) = trial(Jammed, Grid::Adaptive);
    let handoff = idx(&summary, "handoff_tick");
    let touchdown = idx(&summary, "arrival_tick");
    assert!(touchdown > handoff + 100);
    assert!(rows[handoff..].iter().all(|r| r["jaw_force_n"][0] == -6.0));
    // The last 20 ms before reaching the deck are ballistic. Neither finger
    // supports the parcel and the deck has not yet started to carry it.
    let flight = &rows[touchdown - 100..touchdown];
    for r in flight {
        assert!(
            r["normal_force_n"]
                .as_array()
                .unwrap()
                .iter()
                .all(|x| x.as_f64().unwrap() < 0.05)
        );
        assert!(field(r, "receiver_force_n").abs() < 1e-8);
    }
    let fall_speed = field(flight.last().unwrap(), "parcel_vz_m_s");
    let dv = fall_speed - field(&flight[0], "parcel_vz_m_s");
    assert!((dv + 9.81 * 0.0002 * 99.).abs() < 1e-6);
    assert!(fall_speed < -0.5);
    assert!(
        rows[touchdown..]
            .iter()
            .all(|r| r["state"] == "UnguidedLanding")
    );
    assert!(summary["opening_tick"].is_null() && summary["clear_tick"].is_null());
    let final_row = rows.last().unwrap();
    assert!(field(final_row, "receiver_force_n") > 0.8 * 0.2 * 9.81);
}
