// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::gpu_scene::parallel_gripper::{carry, observations, set_population_targets};
const CARRY: &str = include_str!("../../../../examples/gpu_carry_gripper.sdf");
#[test]
fn carry_rejects_incompatible_rails_shelves_and_release_rules() {
    for (a, b, want) in [
        (
            "track=\"track\" shelf",
            "track=\"unknown\" shelf",
            "unknown track joint",
        ),
        (
            "shelf=\"shelf::top\"",
            "shelf=\"unknown::top\"",
            "unknown static support",
        ),
        ("shelf=\"shelf::top\"", "shelf=\"top\"", "model::link"),
        (
            "shelf=\"shelf::top\"",
            "shelf=\"table::base\"",
            "distinct source",
        ),
        (
            "type=\"prismatic\"><parent>world</parent><child>rail",
            "type=\"revolute\"><parent>world</parent><child>rail",
            "world-rooted",
        ),
        (
            "<parent>rail</parent><child>carriage",
            "<parent>world</parent><child>carriage",
            "distinct moving fingertips",
        ),
        (
            "<box><size>0.088 0.12 0.014</size></box>",
            "<sphere><radius>0.012</radius></sphere>",
            "finite receiving box",
        ),
        ("0.088 0.12 0.014", "0.088 0.015 0.014", "transverse width"),
        (
            "0.18 0 0.007 0 0 0",
            "0.18 0 0.014 0 0 0",
            "travel must clear",
        ),
        (
            "<track-drive stiffness",
            "<unknown-drive stiffness",
            "unknown gripper drive element",
        ),
        (
            "after_release=\"true\"",
            "after_release=\"perhaps\"",
            "after_release flag",
        ),
        ("tick=\"571\"", "tick=\"201\"", "duplicate or out-of-range"),
        (
            "track=\"0.18\" lift",
            "track=\"0.31\" lift",
            "duplicate or out-of-range",
        ),
        (
            "weight_fraction=\"0.65\"",
            "weight_fraction=\"-1\"",
            "invalid shelf load",
        ),
        (
            "dwell=\"24\"",
            "dwell=\"0\"",
            "invalid carrying-gripper tick",
        ),
        ("max_gap=\"0.0008\"", "max_gap=\"-1\"", "invalid shelf load"),
        ("right=\"0\"/>", "right=\"-0.03\"/>", "invalid shelf load"),
    ] {
        let s = CARRY.replace(a, b);
        assert_ne!(s, CARRY);
        let error = load(&s, Some("unavailable")).err().unwrap().to_string();
        assert!(error.contains(want), "{a}: {error}");
    }
    for (source, want) in [
        (
            CARRY.replace(
                "      </environment>",
                "      </environment><environment profile=\"carrying-gripper\"/>",
            ),
            "one horizontal carrying gripper",
        ),
        (
            CARRY.replace("model=\"gripper\" object", "model=\"missing\" object"),
            "unknown carrier",
        ),
        (
            CARRY.replace(
                "        <track-drive stiffness=\"175\" damping=\"23\" target=\"0\"/>",
                "",
            ),
            "missing rail drive",
        ),
        (
            CARRY.replace("        <release after", "        <release/><release after"),
            "one shelf release rule",
        ),
        (
            CARRY
                .lines()
                .filter(|l| !l.contains("<release "))
                .collect::<Vec<_>>()
                .join("\n"),
            "missing shelf release rule",
        ),
        (
            CARRY.replace("max_weight_fraction=\"1.4\"", "max_weight_fraction=\"0.3\""),
            "invalid shelf load",
        ),
    ] {
        let e = load(&source, Some("unavailable"))
            .err()
            .unwrap()
            .to_string();
        assert!(e.contains(want), "{e}");
    }
}
#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn normal_carrier_places_on_the_named_shelf_and_repeats_after_reset() {
    let mut sim = load(CARRY, Some(&std::env::var("GZ_GPU_ADAPTER").unwrap())).unwrap();
    assert_eq!(execution_counts(&sim), Some((0, 1)));
    assert!(matches!(
        carry::observe(&sim, 55),
        Err(GpuError::UnknownEnvironment(55))
    ));
    let mut trace = Vec::new();
    for epoch in 0..2 {
        if epoch > 0 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        assert!(carry::set_targets(&mut sim, 12, [0.5, 0., 0., 0.]).is_err());
        carry::set_targets(&mut sim, 12, [0., 0., 0.028, -0.028]).unwrap();
        assert!(set_population_targets(&mut sim, &[]).is_err());
        set_population_targets(&mut sim, &[[0., 0.028, -0.028]]).unwrap();
        assert!(carry::observe(&sim, 12).unwrap().1.is_none());
        for t in 1..=2300 {
            sim.step_exact(1).unwrap();
            if epoch == 1 && t == 1400 {
                sim.set_paused(true);
                let time = sim.elapsed();
                for _ in 0..30 {
                    assert!(sim.tick().unwrap().paused);
                    assert!(carry::observe(&sim, 12).unwrap().1.is_none());
                }
                assert_eq!(sim.elapsed(), time);
            }
            let (l, receipt) = carry::observe(&sim, 12).unwrap();
            assert_eq!(
                observations(&sim).unwrap().next().unwrap().1.ball,
                l.hand.ball
            );
            let reading = sim
                .world()
                .iter_entities()
                .find_map(|e| e.get::<gz_sim_scene::JointStateSample>())
                .unwrap();
            assert_eq!(reading.joints.len(), 4);
            for (j, &(position, speed)) in [
                (l.track, l.track_speed),
                (l.hand.position[0] - 0.03, l.hand.speed[0]),
                (
                    l.hand.position[1] + 0.05 - l.track,
                    l.hand.speed[1] - l.track_speed,
                ),
                (
                    l.hand.position[2] - 0.05 - l.track,
                    l.hand.speed[2] - l.track_speed,
                ),
            ]
            .iter()
            .enumerate()
            {
                assert!((reading.joints[j].position - position).abs() < 1e-12);
                assert!((reading.joints[j].velocity - speed).abs() < 1e-12);
            }
            if (800..=1100).contains(&t) {
                assert!(
                    l.hand.ball[0] > 0.14
                        && l.hand.ball[1] - l.hand.radius > 0.09
                        && l.shelf_normal == 0.
                        && l.hand.normal_impulse[2] == 0.
                );
            }
            if t < 1418 {
                assert!(receipt.is_none());
            } else {
                let s = receipt.unwrap();
                assert_eq!((s.tick, s.sustained_ticks), (1418, 24));
                assert!(s.upward_force_n >= 0.65 * 0.08 * 9.81 && s.clearance_m.abs() < 0.0008);
            }
            if t == 2300 {
                assert!(
                    (l.hand.ball[0] - 0.18).abs() < 0.002
                        && (l.shelf_up / 0.001 - 0.08 * 9.81).abs() < 1e-9
                        && l.hand.normal_impulse == [0.; 3]
                );
            }
            let frame = (
                l.track,
                l.hand.position,
                l.hand.ball,
                l.hand.velocity,
                l.shelf_up,
                receipt,
            );
            if epoch == 0 {
                trace.push(frame);
            } else {
                assert_eq!(trace[t - 1], frame);
            }
        }
    }
    let shelf = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::shelf::top")
        })
        .unwrap()
        .id();
    sim.world_mut()
        .get_mut::<WorldPose>(shelf)
        .unwrap()
        .translation[0] += 0.02;
    assert!(sim.step_exact(1).is_err());
    assert!(carry::observe(&sim, 12).is_err());
}
#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn wrong_destination_and_frictionless_pickup_keep_jaws_closed() {
    let named = std::env::var("GZ_GPU_ADAPTER").unwrap();
    for (s, x) in [
        (CARRY.replace("0.18 0 0.007", "0.34 0 0.007"), 0.18),
        (
            CARRY.replace("<mu>0.6</mu><mu2>0.6", "<mu>0</mu><mu2>0"),
            0.,
        ),
    ] {
        let mut sim = load(&s, Some(&named)).unwrap();
        for _ in 0..2300 {
            sim.step_exact(1).unwrap();
        }
        let (l, r) = carry::observe(&sim, 12).unwrap();
        assert!(r.is_none());
        assert_eq!(l.shelf_normal, 0.);
        if x > 0. {
            assert!(l.hand.ball[1] - l.hand.radius > 0.005 && l.hand.normal_impulse[2] == 0.);
        } else {
            assert!(l.hand.normal_impulse[2] > 0.);
        }
        assert!((l.hand.ball[0] - x).abs() < 0.002);
        assert_eq!(l.hand.target[1], -0.05 + 0.028);
    }
}
