// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![allow(deprecated)]
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, JointSample, Name, PhysicsEngine, UniversalJointSample, Velocity, WorldPose,
    load_world_with_sdf_step, reset_loaded_world,
};
use sdformat::NoResources;

const XML: &str = include_str!("../../../examples/worlds/local_robot_serial_grasp.sdf");

fn find(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    let name = format!("default::{name}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}

#[derive(Debug, PartialEq)]
struct Lift {
    start_z: f64,
    top_z: f64,
    hand_z: f64,
    last_z: f64,
    both: usize,
    post: usize,
    start_post: usize,
    support_impulse: f64,
    max_slip: f64,
    pad_impulses: [f64; 2],
    max_speed: f64,
    first_contact: Option<usize>,
    other_impulse: f64,
    screw_angle: f64,
    wrist_angle: f64,
}

fn cycle(sim: &mut Simulation) -> Lift {
    let [parcel, left, right, post, palm] = [
        "parcel::box",
        "gripper::left",
        "gripper::right",
        "stand::post",
        "gripper::palm",
    ]
    .map(|s| find(sim, s));
    let [load, a, b, stand]: [BodyId; 4] =
        [parcel, left, right, post].map(|e| sim.world().get::<BodyHandle>(e).unwrap().id());
    let read = |sim: &Simulation, e| *sim.world().get::<WorldPose>(e).unwrap();
    let spindle = find(sim, "gripper::spindle");
    let wrist = find(sim, "gripper::wrist");
    let start_z = read(sim, parcel).translation[2];
    let mut m = Lift {
        start_z,
        top_z: start_z,
        hand_z: 0.,
        last_z: 0.,
        both: 0,
        post: 0,
        start_post: 0,
        support_impulse: 0.,
        max_slip: 0.,
        pad_impulses: [0.; 2],
        max_speed: 0.,
        first_contact: None,
        other_impulse: 0.,
        screw_angle: 0.,
        wrist_angle: 0.,
    };
    let mut hold = None;
    for tick in 1..=3600 {
        sim.step_exact(1).unwrap();
        let mut j = [0.; 4];
        for c in sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .rigid_contact_observations()
        {
            let peer = if c.a == load {
                c.b
            } else if c.b == load {
                c.a
            } else {
                continue;
            };
            let i = [a, b, stand]
                .iter()
                .position(|&other| peer == other)
                .unwrap_or(3);
            j[i] += c.normal_impulse;
        }
        let p = read(sim, parcel);
        let h = read(sim, palm);
        let z = p.translation[2];
        m.top_z = m.top_z.max(z);
        m.last_z = z;
        m.hand_z = h.translation[2];
        if j[0] > 0. && j[1] > 0. {
            m.first_contact.get_or_insert(tick);
        }
        if tick < 51 && j[2] > 0. {
            m.start_post += 1;
        }
        if tick > 1800 {
            m.both += usize::from(j[0] > 0. && j[1] > 0.);
            m.post += usize::from(j[2] > 0.);
            m.support_impulse += j[2];
            m.other_impulse += j[3];
            for (total, pad_j) in m.pad_impulses.iter_mut().zip(j) {
                *total += pad_j;
            }
            let local = DQuat::from_array(h.rotation).inverse()
                * (DVec3::from(p.translation) - DVec3::from(h.translation));
            m.max_slip = m.max_slip.max(hold.get_or_insert(local).distance(local));
            if tick > 3200 {
                m.max_speed = m
                    .max_speed
                    .max(DVec3::from(sim.world().get::<Velocity>(parcel).unwrap().linear).length());
            }
        }
    }
    m.screw_angle = sim
        .world()
        .get::<JointSample>(spindle)
        .unwrap()
        .state
        .position;
    m.wrist_angle = sim
        .world()
        .get::<UniversalJointSample>(wrist)
        .unwrap()
        .state
        .position[1];
    m
}

#[test]
fn a_serial_lift_carries_a_free_parcel_through_friction_only_and_repeats_after_reset() {
    for (name, xml) in [
        ("grip", XML.to_owned()),
        (
            "open",
            XML.replace("<spring_reference>0.048", "<spring_reference>0"),
        ),
        ("smooth", XML.replace(">0.85<", ">0<")),
    ] {
        let mut sim = load_world_with_sdf_step(
            &xml,
            "serial_grasp.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let result = cycle(&mut sim);
        eprintln!("serial grasp {name}: {result:?}");
        assert!(result.start_post > 10, "{result:?}");
        assert!((result.screw_angle - 1.6).abs() < 0.001);
        assert!((result.wrist_angle - 0.06).abs() < 0.001);
        assert!((result.hand_z - 0.8224).abs() < 0.001);
        assert_eq!(result.other_impulse, 0.);
        if name == "grip" {
            assert!(result.first_contact.is_some_and(|tick| tick < 50));
            assert!(result.last_z - result.start_z > 0.125);
            assert_eq!(
                (result.both, result.post, result.support_impulse),
                (1800, 0, 0.)
            );
            assert!(result.pad_impulses.iter().all(|j| *j > 10.));
            assert!(result.max_slip < 0.002 && result.max_speed < 0.0005);
        } else {
            assert!(result.top_z - result.start_z < 0.002);
            assert!((result.last_z - result.start_z).abs() < 0.001);
            assert_eq!((result.both, result.post), (0, 1800));
            assert_eq!(result.pad_impulses, [0.; 2]);
            assert!(result.support_impulse >= 0.99 * 0.2 * 9.81 * 1.8);
            assert_eq!(result.first_contact.is_some(), name == "smooth");
        }
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(cycle(&mut sim), result);
    }
}
