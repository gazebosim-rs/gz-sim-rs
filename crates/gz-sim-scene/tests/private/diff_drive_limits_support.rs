// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::collections::BTreeMap;
const XML: &str = include_str!("../../../../examples/m8_diff_drive.sdf");

pub(super) fn xml(extra: &str, count: usize) -> String {
    let begin = XML.find("<model name=\"robot\"").unwrap();
    let end = begin + XML[begin..].find("</model>").unwrap() + "</model>".len();
    let robot = XML[begin..end].replace("</plugin>", &format!("{extra}</plugin>"));
    let robots = (0..count)
        .map(|index| {
            robot
                .replace("name=\"robot\"", &format!("name=\"fleet_{index}\""))
                .replace(
                    "<pose>0 0 0.2 0 0 0</pose>",
                    &format!(
                        "<pose>{} {} 0.2 0 0 0</pose>",
                        if index.is_multiple_of(2) { -1.25 } else { 1.25 },
                        -2.25 + (index / 2) as f64 * 1.5
                    ),
                )
                .replace("/model/robot/", &format!("/model/fleet_{index}/"))
                .replace("robot/odom", &format!("fleet_{index}/odom"))
                .replace("robot/base_link", &format!("fleet_{index}/base_link"))
        })
        .collect::<Vec<_>>()
        .join("");
    XML.replacen(&XML[begin..end], &robots, 1).replace(
        "<max_step_size>0.001</max_step_size>",
        "<max_step_size>0.0078125</max_step_size>",
    )
}

pub(super) fn load(extra: &str, count: usize) -> Simulation {
    load_world_with_sdf_step(
        &xml(extra, count),
        "m112-m113-limits.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}

pub(super) fn parsed_model(extra: &str) -> Model {
    sdformat::load_str(&xml(extra, 1), "m112-config.sdf", &sdformat::NoResources)
        .unwrap()
        .worlds
        .remove(0)
        .models
        .into_iter()
        .find(|m| !m.plugins.is_empty())
        .unwrap()
}

pub(super) fn owners(sim: &Simulation) -> Vec<(Entity, u64)> {
    sim.world()
        .iter_entities()
        .filter(|r| r.contains::<Drive>())
        .map(|r| (r.id(), r.get::<EntityId>().unwrap().get()))
        .collect()
}

pub(super) fn histories(sim: &Simulation) -> Vec<(Entity, [u64; 4])> {
    owners(sim)
        .into_iter()
        .map(|(e, _)| {
            let h = sim.world().get::<Drive>(e).unwrap().command_history;
            (
                e,
                [
                    h.latest[0].to_bits(),
                    h.latest[1].to_bits(),
                    h.older[0].to_bits(),
                    h.older[1].to_bits(),
                ],
            )
        })
        .collect()
}

pub(super) fn native(sim: &Simulation) -> BTreeMap<JointId, [f64; 2]> {
    let backend = sim.world().resource::<PhysicsBackend>();
    sim.world()
        .iter_entities()
        .filter_map(|r| r.get::<JointHandle>())
        .map(|h| {
            let s = backend.joint_state(h.id()).unwrap();
            (h.id(), [s.position, s.velocity])
        })
        .collect()
}

pub(super) fn assert_physical_twins(left: &Simulation, right: &Simulation) {
    let l = native(left);
    let r = native(right);
    assert_eq!(l.len(), r.len());
    for (id, values) in l {
        for (actual, expected) in values.into_iter().zip(r[&id]) {
            assert!(
                (actual - expected).abs() <= 1e-11,
                "native joint {id:?}: {actual} vs {expected}"
            );
        }
    }
    for ((le, lid), (re, rid)) in owners(left).into_iter().zip(owners(right)) {
        assert_eq!(lid, rid);
        let l = left.world().get::<WorldPose>(le).unwrap();
        let r = right.world().get::<WorldPose>(re).unwrap();
        for (x, y) in l.translation.into_iter().zip(r.translation) {
            assert!((x - y).abs() <= 1e-11);
        }
        for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
            assert!(
                (DQuat::from_array(l.rotation) * axis - DQuat::from_array(r.rotation) * axis)
                    .abs()
                    .max_element()
                    <= 1e-11
            );
        }
    }
}
