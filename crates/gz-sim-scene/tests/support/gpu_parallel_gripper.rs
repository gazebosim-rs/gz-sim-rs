// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::gpu_scene::{
    fixed_population,
    parallel_gripper::{observe as read, set_targets as command},
};
const WORLD: &str = include_str!("../../../../examples/gpu_parallel_gripper.sdf");
#[test]
fn unsupported_mechanisms_do_not_silently_select_another_backend() {
    let cases = [
        ("id=\"12\"", "id=\"bad\"", "environment id"),
        (
            "profile=\"parallel-gripper\"",
            "profile=\"parallel-gripper\" extra=\"x\"",
            "independent gripper world",
        ),
        (
            "0.001</max_step_size>",
            "0.002</max_step_size>",
            "independent gripper world",
        ),
        ("0 0 -9.81", "0.1 0 -9.81", "independent gripper world"),
        ("model=\"gripper\"", "model=\"absent\"", "unknown gripper"),
        ("object=\"parcel\"", "object=\"table\"", "free solid sphere"),
        ("left=\"left_slider\"", "left=\"unknown\"", "unknown slider"),
        (
            "left=\"left_slider\"",
            "left=\"right_slider\"",
            "distinct moving fingertips",
        ),
        (
            "<parent>carriage</parent><child>left",
            "<parent>world</parent><child>left",
            "distinct moving fingertips",
        ),
        (
            "<joint name=\"left_slider\" type=\"prismatic\"",
            "<joint name=\"left_slider\" type=\"revolute\"",
            "ordinary slider",
        ),
        ("1 0 0</xyz>", "0 1 0</xyz>", "ordinary slider"),
        (".012 .064 .036", ".012 .006 .036", "covering"),
        (
            "<box><size>.012 .064 .036</size></box>",
            "<sphere><radius>0.015</radius></sphere>",
            "finite box fingertip",
        ),
        ("<pose>0.05 0 0.028", "<pose>0.05 0 0.029", "level finger"),
        (
            "<lower>-0.008</lower>",
            "<lower>-0.012</lower>",
            "travel envelope",
        ),
        (
            "<upper>0.035</upper>",
            "<upper>0.055</upper>",
            "travel envelope",
        ),
        (
            "joint=\"left_slider\" stiffness",
            "joint=\"missing\" stiffness",
            "unknown gripper drive",
        ),
        (
            "joint=\"left_slider\" stiffness",
            "joint=\"right_slider\" stiffness",
            "duplicate gripper drive",
        ),
        ("tick=\"651\"", "tick=\"201\"", "duplicate or out-of-range"),
        ("tick=\"201\"", "tick=\"0\"", "invalid target tick"),
        (
            "lift=\"0.122\"",
            "lift=\"0.222\"",
            "duplicate or out-of-range",
        ),
        (
            "<drive joint=\"lift\"",
            "<unknown joint=\"lift\"",
            "unknown gripper drive element",
        ),
        ("stiffness=\"270\"", "stiffness=\"NaN\"", "numeric drive"),
        (
            "<mass>0.08</mass>",
            "<mass>0.09</mass>",
            "solid-sphere inertia",
        ),
        (
            "<link name=\"ball\">",
            "<link name=\"ball\"><kinematic>true</kinematic>",
            "ordinary dynamic sphere",
        ),
    ];
    for (a, b, want) in cases {
        let s = WORLD.replace(a, b);
        assert_ne!(s, WORLD);
        let msg = load(&s, Some("unavailable")).err().unwrap().to_string();
        assert!(msg.contains(want), "{a}: {msg}");
    }
    assert!(
        load(WORLD, None)
            .err()
            .unwrap()
            .to_string()
            .contains("--gpu-adapter")
    );
}
#[test]
#[ignore = "requires named Vulkan f64 device through GZ_GPU_ADAPTER"]
fn loaded_gripper_lifts_without_ground_releases_and_repeats() {
    let mut sim = load(WORLD, Some(&std::env::var("GZ_GPU_ADAPTER").unwrap())).unwrap();
    assert!(fixed_population(&sim));
    assert_eq!(execution_counts(&sim), Some((0, 1)));
    assert_eq!(
        environment_placement(&sim, 12).unwrap(),
        ExecutionPlacement::Gpu
    );
    assert!(matches!(
        read(&sim, 9),
        Err(GpuError::UnknownEnvironment(9))
    ));
    let mut trace = Vec::new();
    for epoch in 0..2 {
        if epoch == 1 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        assert!(command(&mut sim, 12, [1., 0., 0.]).is_err());
        command(&mut sim, 12, [0., 0.028, -0.028]).unwrap();
        let find = |name: &str| {
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap()
                .id()
        };
        let ball = find("default::parcel::ball");
        let joint = find("default::gripper::left_slider");
        for t in 1..=1000 {
            sim.step_exact(1).unwrap();
            let l = read(&sim, 12).unwrap();
            let pose = sim.world().get::<WorldPose>(ball).unwrap();
            assert_eq!([pose.translation[0], pose.translation[2]], l.ball);
            assert!(
                (sim.world()
                    .get::<gz_sim_scene::JointSample>(joint)
                    .unwrap()
                    .state
                    .position
                    - (l.position[1] + 0.05))
                    .abs()
                    < 1e-12
            );
            let row = (
                l.position,
                l.ball,
                l.velocity,
                l.normal_impulse,
                l.tangent_impulse,
            );
            if (500..=640).contains(&t) {
                assert!(l.ball[1] - l.radius > 0.118 && l.normal_impulse[2] == 0.);
            }
            if t == 1000 {
                assert!(l.normal_impulse[2] > 0. && l.normal_impulse[..2] == [0.; 2]);
            }
            if epoch == 0 {
                trace.push(row);
            } else {
                assert_eq!(trace[t - 1], row);
            }
        }
    }
    let extra = sim.world_mut().spawn(gz_sim_scene::EntityKind::Joint).id();
    assert!(matches!(read(&sim, 12), Err(GpuError::StaleBoundary)));
    sim.world_mut().despawn(extra);
    let floor = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::table::base")
        })
        .unwrap()
        .id();
    sim.world_mut()
        .get_mut::<WorldPose>(floor)
        .unwrap()
        .translation[2] = 0.1;
    assert!(sim.step_exact(1).is_err());
    assert!(matches!(read(&sim, 12), Err(GpuError::StaleBoundary)));
    assert!(command(&mut sim, 12, [0.; 3]).is_err());
}

#[path = "../../examples/support/gripper_population.rs"]
mod population;

#[test]
fn independent_grippers_require_disjoint_geometry_and_unique_owners() {
    let base = population::xml(2);
    for (a, b, want) in [
        (
            "id=\"1\" profile",
            "id=\"0\" profile",
            "duplicate gripper environment id",
        ),
        ("0.16 0.08 ", "0.16 0.01 ", "disjoint lateral swept volumes"),
        (
            "object=\"parcel_0001\"",
            "object=\"parcel_0000\"",
            "duplicate gripper or sphere ownership",
        ),
        (
            "model=\"gripper_0001\"",
            "model=\"gripper_0000\"",
            "duplicate gripper or sphere ownership",
        ),
        (
            "plane=\"table::base\"",
            "plane=\"table::other\"",
            "shared table",
        ),
    ] {
        let s = base.replace(a, b);
        assert_ne!(s, base);
        let err = load(&s, Some("absent adapter")).err().unwrap().to_string();
        assert!(err.contains(want), "{a}: {err}");
    }
}
#[test]
#[ignore = "requires named f64 Vulkan adapter"]
fn distinct_grippers_publish_all_joints_and_target_only_the_named_environment() {
    use gz_sim_scene::{
        JointStateSample,
        gpu_scene::parallel_gripper::{observations, set_population_targets},
    };
    let mut sim = load(
        &population::xml(12),
        Some(&std::env::var("GZ_GPU_ADAPTER").unwrap()),
    )
    .unwrap();
    assert_eq!(execution_counts(&sim), Some((0, 12)));
    assert!(matches!(
        command(&mut sim, 99, [0.; 3]),
        Err(GpuError::UnknownEnvironment(99))
    ));
    let original: Vec<_> = observations(&sim).unwrap().map(|(_, l)| l.target).collect();
    assert!(matches!(
        set_population_targets(&mut sim, &[]),
        Err(GpuError::GripperActionCount { .. })
    ));
    let mut commands = vec![[0., 0., 0.]; 12];
    commands[11][0] = f64::NAN;
    assert!(matches!(
        set_population_targets(&mut sim, &commands),
        Err(GpuError::InvalidArmTarget(11))
    ));
    assert_eq!(
        original,
        observations(&sim)
            .unwrap()
            .map(|(_, l)| l.target)
            .collect::<Vec<_>>()
    );
    commands[11][0] = 0.;
    set_population_targets(&mut sim, &commands).unwrap();
    command(&mut sim, 2, [0.06, 0., 0.]).unwrap();
    for _ in 0..12 {
        sim.step_exact(1).unwrap();
    }
    let rows: Vec<_> = observations(&sim).unwrap().collect();
    for (i, (id, l)) in rows.iter().enumerate() {
        assert_eq!(*id, i as u64);
        if i == 2 {
            assert!(l.position[0] > 0.029);
        } else {
            assert_eq!(l.position[0], 0.028);
        }
    }
    let sensors: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<JointStateSample>())
        .collect();
    assert_eq!(sensors.len(), 12);
    assert!(
        sensors
            .iter()
            .all(|s| s.joints.len() == 3 && s.time.as_millis() == 12)
    );
    let extra = sim.world_mut().spawn(gz_sim_scene::EntityKind::Joint).id();
    assert!(matches!(
        set_population_targets(&mut sim, &commands),
        Err(GpuError::StaleBoundary)
    ));
    assert!(matches!(observations(&sim), Err(GpuError::StaleBoundary)));
    sim.world_mut().despawn(extra);
}

#[test]
#[ignore = "requires named f64 Vulkan adapter"]
fn streamed_actions_reproduce_the_authored_load_and_release_at_every_tick() {
    use gz_sim_scene::gpu_scene::parallel_gripper::{observations, set_population_targets};
    let xml = population::xml(8);
    let name = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut scheduled = load(&xml, Some(&name)).unwrap();
    let mut reversed = population::streamed(&xml);
    let nodes: Vec<_> = roxmltree::Document::parse(&reversed)
        .unwrap()
        .descendants()
        .filter(|n| n.has_tag_name("environment"))
        .map(|n| n.range())
        .collect();
    let order: String = nodes.iter().rev().map(|r| &reversed[r.clone()]).collect();
    reversed.replace_range(nodes[0].start..nodes.last().unwrap().end, &order);
    let mut live = load(&reversed, Some(&name)).unwrap();
    for tick in 1..=1000 {
        let actions: [_; 8] = std::array::from_fn(|i| population::action(i, tick));
        set_population_targets(&mut live, &actions).unwrap();
        scheduled.step_exact(1).unwrap();
        live.step_exact(1).unwrap();
        for ((a, l), (b, r)) in observations(&scheduled)
            .unwrap()
            .zip(observations(&live).unwrap())
        {
            assert_eq!(a, b);
            assert_eq!(l.position, r.position);
            assert_eq!(l.ball, r.ball);
            assert_eq!(l.speed, r.speed);
            assert_eq!(l.normal_impulse, r.normal_impulse);
            assert_eq!(l.tangent_impulse, r.tangent_impulse);
        }
    }
}
