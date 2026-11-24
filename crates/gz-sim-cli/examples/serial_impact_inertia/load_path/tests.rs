// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn momentum_balance_preserves_all_three_signs_and_includes_the_other_finger() {
    let zero = State {
        xyz: [0.; 3],
        q: [0., 0., 0., 1.],
        v: [0.; 3],
        w: [0.; 3],
    };
    let mut empty = vec![zero; 6];
    empty[2].w = [1., 2., 3.];
    let mut hit = empty.clone();
    hit[5].v = [-1., 0.5, 0.25];
    hit[3].v = [1., 0., -0.5];
    hit[4].v = [-0.5, 0.25, 0.];
    hit[2].v = [0.25, -0.5, 0.];
    hit[1].w = [-1., 0., 1.];
    let change = Transmission::from_motion(&hit, &empty, 3);
    for (a, b) in [
        (change.parcel_on_finger_ns, [0.2, -0.1, -0.05]),
        (change.finger_on_palm_ns, [0.12, -0.1, -0.01]),
        (change.hand_on_wrist_ns, [0.06, 0.08, -0.01]),
        (change.palm_relative_to_nut_delta_rad_s, [1., 0., -1.]),
    ] {
        assert!(DVec3::from(a).distance(DVec3::from(b)) < 1e-14);
    }
}

#[test]
fn native_strikes_start_from_zero_and_replay_the_measured_load_at_both_joints() {
    for side in ["left", "right"] {
        let mut args = vec![
            "study",
            "--substeps",
            "64",
            "--automatic",
            "--impact-iterations",
            "8",
            "--impact-pgs",
            "8",
            "--gap-um",
            "50",
            "--divisor",
            "1.75",
            "--guide-hz",
            "100000",
            "--early-face-step-ns",
            "250",
            "--early-face-max-steps",
            "4096",
        ];
        if side == "right" {
            args.push("--right");
        }
        let mut t = Args::try_parse_from(args).unwrap().trial;
        let folder = tempfile::tempdir().unwrap();
        t.replay = Some(folder.path().join("replay"));
        let result = audit(t.clone(), true).unwrap();
        let begin = result.pulses[0].start_ns;
        let end = result.pulses.last().unwrap().end_ns;
        let loaded = result
            .samples
            .iter()
            .find(|s| u128::from(s.end_ns) == end)
            .unwrap()
            .transmitted
            .as_ref()
            .unwrap();
        for f in result
            .samples
            .iter()
            .filter(|s| u128::from(s.end_ns) <= begin)
        {
            let pre = f.transmitted.as_ref().unwrap();
            assert_eq!(pre.parcel_on_finger_ns, [0.; 3]);
            assert_eq!(pre.finger_on_palm_ns, [0.; 3]);
            assert_eq!(pre.hand_on_wrist_ns, [0.; 3]);
        }
        let normal_x = result
            .pulses
            .iter()
            .map(|p| p.linear_on_parcel_ns[0])
            .sum::<f64>();
        assert!((normal_x + loaded.parcel_on_finger_ns[0]).abs() < 1e-7);
        assert!(loaded.finger_on_palm_ns[0].abs() < 0.0001);
        assert!(loaded.finger_on_palm_ns[2].abs() > 0.005);
        let records =
            std::fs::read_to_string(t.replay.as_ref().unwrap().join("frames.jsonl")).unwrap();
        for (row, s) in records.lines().skip(1).zip(&result.samples) {
            let r: Value = serde_json::from_str(row).unwrap();
            let frame: Value = serde_json::from_str(r["scene"].as_str().unwrap()).unwrap();
            let load = s.transmitted.as_ref().unwrap();
            for (name, vec) in [
                ("finger_to_palm", load.finger_on_palm_ns),
                ("hand_to_wrist", load.hand_on_wrist_ns),
            ] {
                let scoped = format!("default::load_path::readout::{name}");
                let v = frame["visuals"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .find(|v| v["name"] == scoped)
                    .unwrap();
                assert!(
                    (v["translation"][2].as_f64().unwrap() - 0.54 - 2. * DVec3::from(vec).length())
                        .abs()
                        < 1e-14
                );
            }
        }
        t.replay = None;
        t.miss = true;
        let mut no_hit = audit(t.clone(), true).unwrap();
        assert!(
            no_hit
                .samples
                .iter()
                .all(|v| v.transmitted.as_ref().unwrap().hand_on_wrist_ns == [0.; 3])
        );
        no_hit.unexpected_contact = true;
        assert!(measure(&t, vec![], &mut no_hit).is_err());
    }
}
