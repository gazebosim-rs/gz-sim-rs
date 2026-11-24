// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise physical single-finger loss and the public moving-gripper recording.
#[allow(dead_code)]
#[path = "../examples/local_regrasp.rs"]
mod regrasp;
use bevy_tasks::{ComputeTaskPool, TaskPoolBuilder};
use regrasp::{Case, Grid};
use serde::Deserialize;
use std::io::{BufRead, BufReader};

#[derive(Deserialize)]
struct Tick {
    time_ns: u64,
    state: String,
    normal_force_n: [f64; 2],
    jaw_force_n: [f64; 2],
    next_jaw_force_n: [f64; 2],
    external_opening_n: [f64; 2],
    pad_vertical_force_n: [f64; 2],
    parcel_vz_m_s: f64,
    slip_m: f64,
    tactile_travel_m: Option<f64>,
}

fn trace(case: Case, grid: Grid) -> Vec<Tick> {
    ComputeTaskPool::get_or_init(|| TaskPoolBuilder::new().num_threads(1).build());
    let tmp = tempfile::tempdir().unwrap();
    let path = tmp.path().join("capture");
    regrasp::run(case, grid, &path).unwrap();
    let file = std::fs::File::open(path.join("ticks.jsonl")).unwrap();
    BufReader::new(file)
        .lines()
        .map(|line| serde_json::from_str(&line.unwrap()).unwrap())
        .collect()
}

fn transitions(trace: &[Tick], state: &str) -> Vec<u64> {
    trace
        .windows(2)
        .filter(|s| s[0].state != state && s[1].state == state)
        .map(|s| s[1].time_ns)
        .collect()
}
#[test]
fn missing_packet_and_short_physical_gap_have_different_signatures() {
    let base = trace(Case::Baseline, Grid::Adaptive);
    let blank = trace(Case::Dropout, Grid::Adaptive);
    let brief = trace(Case::Brief, Grid::Adaptive);
    assert_eq!(transitions(&blank, "Uncertain").len(), 1);
    assert!(!transitions(&brief, "ConfirmingLoss").is_empty());
    for ((a, b), c) in base.iter().zip(&blank).zip(&brief) {
        assert_eq!(a.slip_m, b.slip_m);
        assert_eq!(a.normal_force_n, b.normal_force_n);
        assert_eq!(b.jaw_force_n, [4.2; 2]);
        assert_eq!(c.jaw_force_n, [4.2; 2]);
    }
    assert!(
        blank
            .iter()
            .filter(|s| s.state == "Uncertain")
            .all(|s| s.normal_force_n.iter().all(|n| *n > 4.))
    );
    assert!(
        brief
            .iter()
            .any(|s| s.normal_force_n[0] == 0. && s.normal_force_n[1] > 1.)
    );
}
#[test]
fn opposite_interruptions_restart_the_receiver_and_obey_actuator_slew() {
    for (case, side) in [
        (Case::Left, 0),
        (Case::Right, 1),
        (Case::Reacquire, 1),
        (Case::Bounded, 0),
    ] {
        let obs = trace(case, Grid::Adaptive);
        let settling = transitions(&obs, "Settling")[0];
        let return_tick = obs.iter().find(|s| s.time_ns == settling).unwrap();
        assert_eq!(return_tick.tactile_travel_m, Some(0.));
        assert!(obs.iter().any(|s| s.external_opening_n[side] > 0.
            && s.normal_force_n[side] < 0.05
            && s.normal_force_n[1 - side] > 0.1));
        assert_eq!(obs.last().unwrap().state, "Held");
        for (a, b) in obs.iter().zip(&obs[1..]) {
            assert_eq!(a.next_jaw_force_n, b.jaw_force_n);
            assert!(b.jaw_force_n[0] - a.jaw_force_n[0] <= 0.0200000001);
            assert!(b.jaw_force_n[0] <= 5.2);
            let dv = b.parcel_vz_m_s - a.parcel_vz_m_s;
            let force = b.pad_vertical_force_n.iter().sum::<f64>() - 0.2 * 9.81;
            assert!((0.2 * dv - 0.0002 * force).abs() < 1e-9);
        }
        if case == Case::Bounded {
            assert_eq!(obs.last().unwrap().jaw_force_n, [5.2; 2]);
        }
    }
}
#[test]
fn interrupted_settling_and_an_independent_second_loss_restart_confirmation() {
    let settling = trace(Case::Settling, Grid::Adaptive);
    let held = transitions(&settling, "Held");
    let contact = transitions(&settling, "Settling");
    assert_eq!(contact.len(), 2);
    assert_eq!(held.len(), 2);
    assert!(held[1] >= contact[1] + 2_800_000);
    let repeat = trace(Case::Repeat, Grid::Adaptive);
    let held = transitions(&repeat, "Held");
    let closes = transitions(&repeat, "Reclosing");
    assert_eq!(held.len(), 3);
    assert!(
        held[0] < closes[0] && closes[0] < held[1] && held[1] < closes[1] && closes[1] < held[2]
    );
    let lost = trace(Case::Prolonged, Grid::Adaptive);
    let lower = transitions(&lost, "Lower");
    assert_eq!(lower.len(), 1);
    assert!(
        lost.iter()
            .filter(|s| s.time_ns >= lower[0])
            .all(|s| s.state == "Lower")
    );
}
#[test]
#[ignore = "continuously fine qualification of the physical contact-loss matrix"]
fn decision_and_drop_agree_with_the_continuous_one_microsecond_reference() {
    for case in [
        Case::Baseline,
        Case::Left,
        Case::Right,
        Case::Dropout,
        Case::Brief,
        Case::Bounded,
        Case::Reacquire,
        Case::Settling,
        Case::Repeat,
        Case::Prolonged,
    ] {
        let a = trace(case, Grid::Adaptive);
        let f = trace(case, Grid::Fine);
        assert_eq!(a.last().unwrap().state, f.last().unwrap().state);
        for state in ["Held", "Reclosing", "Lower"] {
            let x = transitions(&a, state);
            let y = transitions(&f, state);
            assert_eq!(x.len(), y.len());
            assert!(x.iter().zip(y).all(|(x, y)| x.abs_diff(y) <= 400_000));
        }
        let pa = a.iter().map(|s| s.slip_m).fold(0., f64::max);
        let pf = f.iter().map(|s| s.slip_m).fold(0., f64::max);
        assert!(
            (pa - pf).abs() < 0.000035,
            "{case:?}: {} µm",
            (pa - pf) * 1e6
        );
    }
}
