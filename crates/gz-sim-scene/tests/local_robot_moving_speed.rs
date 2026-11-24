// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![allow(deprecated)]
use gz_sim_core::Simulation;
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{
    JointSample, Name, PhysicsEngine, UniversalJointSample, Velocity, WorldPose,
    load_world_with_sdf_step, queue_joint_effort, queue_link_world_wrench,
    queue_universal_joint_effort, reset_loaded_world,
};
use sdformat::NoResources;

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "moving.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    let name = format!("default::drive::{name}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn link(sim: &Simulation, link: u64) -> (WorldPose, Velocity) {
    let e = entity_by_id(sim.world(), link).unwrap();
    (*sim.world().get(e).unwrap(), *sim.world().get(e).unwrap())
}
fn free_pair(kind: &str, axis: usize, cap: f64) -> String {
    let direction = if axis == 0 { "1 0 0" } else { "0 1 0" };
    let positions = if axis == 0 {
        ["-0.15 0 1", "0.15 0 1", "-0.15 0 0"]
    } else {
        ["0 -0.15 1", "0 0.15 1", "0 -0.15 0"]
    };
    let link = |name: &str, pos: &str| {
        format!(
            "<link name='{name}'><pose>{pos} 0 0 0</pose><inertial><mass>1</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz></inertia></inertial><collision name='bearing'><geometry><sphere><radius>0.05</radius></sphere></geometry></collision></link>"
        )
    };
    let lim = format!("<limit><velocity>{cap}</velocity></limit>");
    let axes = if kind == "screw" {
        format!(
            "<screw_thread_pitch>0.4</screw_thread_pitch><axis><xyz>{direction}</xyz>{lim}</axis>"
        )
    } else {
        let first = if axis == 0 { lim.as_str() } else { "" };
        let second = if axis == 1 { lim.as_str() } else { "" };
        format!("<axis><xyz>1 0 0</xyz>{first}</axis><axis2><xyz>0 1 0</xyz>{second}</axis2>")
    };
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='local' type='ignored'><max_step_size>0.001</max_step_size></physics><model name='drive'>{}{}<joint name='drive_joint' type='{kind}'><parent>bearing</parent><child>output</child><pose>{} 0 0 0</pose>{axes}</joint></model></world></sdf>",
        link("bearing", positions[0]),
        link("output", positions[1]),
        positions[2]
    )
}

#[test]
fn free_wrist_and_screw_speed_reactions_conserve_momentum_during_reversal() {
    for (kind, axis) in [("universal", 0), ("revolute2", 1), ("screw", 0)] {
        let mut sim = load(&free_pair(kind, axis, 1.));
        let bearing = id(&sim, "bearing");
        let tip = id(&sim, "output");
        let joint = id(&sim, "drive_joint");
        let mut replay = None;
        for _ in 0..2 {
            let mut peak = 0_f64;
            let mut worst_momentum = 0_f64;
            let mut max_lead_error = 0_f64;
            let mut previous = 0_f64;
            for sign in [1., -1.] {
                for tick in 0..300 {
                    if kind == "screw" {
                        queue_joint_effort(&mut sim, joint, sign).unwrap();
                    } else {
                        let mut load = [0.; 2];
                        load[axis] = sign;
                        queue_universal_joint_effort(&mut sim, joint, load).unwrap();
                    }
                    sim.step_exact(1).unwrap();
                    let (ap, a) = link(&sim, bearing);
                    let (bp, b) = link(&sim, tip);
                    let s = if kind == "screw" {
                        let c = sim
                            .world()
                            .get::<JointSample>(entity_by_id(sim.world(), joint).unwrap())
                            .unwrap()
                            .state;
                        max_lead_error = max_lead_error.max(
                            (bp.translation[axis]
                                - ap.translation[axis]
                                - 0.30
                                - 0.4 / std::f64::consts::TAU * c.position)
                                .abs(),
                        );
                        c.velocity
                    } else {
                        sim.world()
                            .get::<UniversalJointSample>(entity_by_id(sim.world(), joint).unwrap())
                            .unwrap()
                            .state
                            .velocity[axis]
                    };
                    peak = peak.max(s.abs());
                    worst_momentum =
                        worst_momentum.max((0.02 * (a.angular[axis] + b.angular[axis])).abs());
                    assert!((a.linear[axis] + b.linear[axis]).abs() < 1e-6);
                    if tick == 299 {
                        eprintln!(
                            "moving {kind} axis {axis} sign {sign} speed {s} parent {} child {}",
                            a.angular[axis], b.angular[axis]
                        );
                        assert!((s - sign).abs() < 0.003);
                    }
                    previous = s;
                }
            }
            eprintln!(
                "moving {kind} max speed {peak}, angular momentum error {worst_momentum}, helical error {max_lead_error}"
            );
            assert!(peak < 1.003 && worst_momentum < 1e-6 && max_lead_error < 1e-5);
            if let Some(prev) = replay {
                assert_eq!(previous, prev);
            }
            replay = Some(previous);
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        // Add angular momentum while coasting. A moving speed governor must
        // admit the same common rotation on both bodies without braking it.
        let mut torque = [0.; 3];
        torque[axis] = 12.;
        queue_link_world_wrench(&mut sim, bearing, [0.; 3], torque).unwrap();
        queue_link_world_wrench(&mut sim, tip, [0.; 3], torque).unwrap();
        sim.step_exact(1).unwrap();
        let a = link(&sim, bearing).1.angular[axis];
        let b = link(&sim, tip).1.angular[axis];
        assert!(
            (a - 0.6).abs() < 1e-4 && (b - 0.6).abs() < 1e-4,
            "{kind} common speed {a} {b}"
        );
    }
}

#[test]
fn active_wrist_limits_remain_independent_of_the_speed_stop_and_zero_axis() {
    for bound in [0.24, 2.3] {
        let xml = free_pair("universal", 0, 2.)
            .replace(
                "<velocity>2</velocity>",
                &format!("<lower>-0.21</lower><upper>{bound}</upper><velocity>2</velocity>"),
            )
            .replace(
                "<axis2><xyz>0 1 0</xyz></axis2>",
                "<axis2><xyz>0 1 0</xyz><limit><velocity>0</velocity></limit></axis2>",
            );
        let mut sim = load(&xml);
        let joint = id(&sim, "drive_joint");
        let mut max_cross = 0_f64;
        for (direction, target) in [(1., bound), (-1., -0.21)] {
            for _ in 0..2100 {
                queue_universal_joint_effort(&mut sim, joint, [direction, 0.8]).unwrap();
                sim.step_exact(1).unwrap();
                let s = sim
                    .world()
                    .get::<UniversalJointSample>(entity_by_id(sim.world(), joint).unwrap())
                    .unwrap()
                    .state;
                max_cross = max_cross.max(s.position[1].abs());
                assert!(s.position[0] <= bound + 0.002 && s.position[0] >= -0.212);
            }
            let s = sim
                .world()
                .get::<UniversalJointSample>(entity_by_id(sim.world(), joint).unwrap())
                .unwrap()
                .state;
            eprintln!(
                "moving loaded stop {target}: {:?}, cross-axis drift {max_cross}",
                s.position
            );
            assert!((s.position[0] - target).abs() < 0.0001 && max_cross < 1e-4);
        }
    }
}
