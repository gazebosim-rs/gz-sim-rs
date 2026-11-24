// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "gpu-batch")]
use gz_physics_gpu::GpuError;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    Name, PhysicsEngine, WorldLoadOptions, WorldPose,
    gpu_scene::{
        ExecutionPlacement, environment_placement, execution_counts,
        planar_tool::{observe, set_targets},
    },
    load_world_on_adapter, reset_loaded_world,
};
const WORLD: &str = include_str!("../../../examples/gpu_planar_tool_world.sdf");
fn load(s: &str, name: Option<&str>) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_on_adapter(
        s,
        "gpu_sdf_tool.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        name,
    )
}
#[test]
fn inadmissible_tool_worlds_are_rejected_before_device_lookup() {
    assert!(
        load(WORLD, None)
            .err()
            .unwrap()
            .to_string()
            .contains("--gpu-adapter")
    );
    let cases = [
        ("0 0 -9.81", "1 0 -9.81"),
        ("0.001</max_step_size>", "0.002</max_step_size>"),
        ("<static>true</static>", "<static>false</static>"),
        ("0 0 1</normal>", "1 0 0</normal>"),
        (
            "<plane><normal>0 0 1</normal><size>2 1</size></plane>",
            "<box><size>2 1 1</size></box>",
        ),
        ("id=\"7\"", "id=\"x\""),
        ("plane=\"floor::base\"", "plane=\"absent\""),
        ("model=\"arm\"", "model=\"absent\""),
        ("id=\"7\" profile=", "id=\"7\" extra=\"x\" profile="),
        ("<drive joint=\"shoulder\"", "<drive joint=\"absent\""),
        ("stiffness=\"5\"", "stiffness=\"-1\""),
        ("damping=\"0.41\"", "damping=\"NaN\""),
        ("target=\"0.12\"", ""),
        ("tick=\"301\"", "tick=\"0\""),
        ("<target tick=", "<unknown tick="),
        ("<child>forearm</child>", "<child>upper</child>"),
        ("0 -1 0</xyz>", "0 1 0</xyz>"),
        (
            "<effort>8</effort>",
            "<effort>8</effort><velocity>5</velocity>",
        ),
        (
            "<joint name=\"elbow\" type=\"revolute\"",
            "<joint name=\"elbow\" type=\"prismatic\"",
        ),
        ("0.21 0 0 0 0 0", "0.21 0.01 0 0 0 0"),
        ("0.12 0 0 0 0 0", "0.24 0 0 0 0 0"),
        ("0 0 0 0 -0.48 0", "0 0 0 0.1 -0.48 0"),
        ("<mu2>0.30</mu2>", "<mu2>0.6</mu2>"),
        ("0 0 0.65 0 0 0", "0 0 1.65 0 0 0"),
    ];
    for (from, to) in cases {
        let s = WORLD.replace(from, to);
        assert_ne!(s, WORLD);
        let e = load(&s, Some("deliberately absent adapter"))
            .err()
            .expect("invalid SDF accepted")
            .to_string();
        assert!(!e.contains("was not found"), "{from}: {e}");
    }
}
#[test]
fn ownership_and_command_schedule_are_validated() {
    let start = WORLD.find("<environment ").unwrap();
    let end = WORLD.find("</environment>").unwrap() + "</environment>".len();
    let entry = &WORLD[start..end];
    let start_model = WORLD.find("<model name=\"arm\"").unwrap();
    let end_model = start_model + WORLD[start_model..].find("</model>").unwrap() + 8;
    let model = &WORLD[start_model..end_model];
    let examples = [
        WORLD.replace(
            "      </environment>",
            "        <target tick=\"301\" shoulder=\"0\" elbow=\"0\"/></environment>",
        ),
        WORLD.replace(
            "<drive joint=\"elbow\" stiffness=\"5\" damping=\"0.41\" target=\"0.12\"/>",
            "",
        ),
        WORLD.replace("</plugin>", &format!("{entry}</plugin>")),
        WORLD.replace(
            "</world>",
            &format!(
                "{}</world>",
                model.replace("name=\"arm\"", "name=\"second\"")
            ),
        ),
        WORLD
            .replace(
                "</plugin>",
                &format!(
                    "{}</plugin>",
                    entry
                        .replace("id=\"7\"", "id=\"8\"")
                        .replace("model=\"arm\"", "model=\"second\"")
                ),
            )
            .replace(
                "</world>",
                &format!(
                    "{}</world>",
                    model.replace("name=\"arm\"", "name=\"second\"")
                ),
            ),
    ];
    for s in examples {
        let e = load(&s, Some("deliberately absent adapter"))
            .err()
            .unwrap()
            .to_string();
        assert!(!e.contains("was not found"), "{e}");
    }
}
#[test]
#[ignore = "requires GZ_GPU_ADAPTER with f64 Vulkan"]
fn ordinary_sdf_load_target_changes_lifecycle_and_fixed_floor_are_consistent() {
    let mut sim = load(WORLD, Some(&std::env::var("GZ_GPU_ADAPTER").unwrap())).unwrap();
    assert_eq!(execution_counts(&sim), Some((0, 1)));
    assert_eq!(
        environment_placement(&sim, 7).unwrap(),
        ExecutionPlacement::Gpu
    );
    assert!(matches!(
        observe(&sim, 8),
        Err(GpuError::UnknownEnvironment(8))
    ));
    let expected = *observe(&sim, 7).unwrap();
    sim.step_exact(60).unwrap();
    let boundary = *observe(&sim, 7).unwrap();
    assert!(boundary.normal_impulse > 0.);
    assert!(set_targets(&mut sim, 7, [0., f64::NAN]).is_err());
    set_targets(&mut sim, 7, [0.4, -0.2]).unwrap();
    assert_eq!(observe(&sim, 7).unwrap().arm.angle, boundary.arm.angle);
    assert_ne!(observe(&sim, 7).unwrap().arm.target, boundary.arm.target);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(observe(&sim, 7).unwrap().arm.angle, expected.arm.angle);
    assert_eq!(observe(&sim, 7).unwrap().arm.target, expected.arm.target);
    sim.step_exact(60).unwrap();
    assert_eq!(
        observe(&sim, 7).unwrap().normal_impulse,
        boundary.normal_impulse
    );
    let e = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::floor::base")
        })
        .unwrap()
        .id();
    sim.world_mut().get_mut::<WorldPose>(e).unwrap().translation[2] += 0.2;
    assert!(sim.step_exact(1).is_err());
    assert!(matches!(
        set_targets(&mut sim, 7, [0.; 2]),
        Err(GpuError::StaleBoundary)
    ));
}

#[test]
#[ignore = "requires GZ_GPU_ADAPTER with f64 Vulkan"]
fn disjoint_arms_retain_logical_ids_and_refuse_a_replaced_population() {
    let start = WORLD.find("<model name=\"arm\"").unwrap();
    let end = start + WORLD[start..].find("</model>").unwrap() + 8;
    let model = WORLD[start..end]
        .replace("name=\"arm\"", "name=\"second\"")
        .replace("<pose>0 0 0.60", "<pose>0 1 0.60");
    let start = WORLD.find("<environment ").unwrap();
    let end = WORLD.find("</environment>").unwrap() + 14;
    let row = WORLD[start..end]
        .replace("id=\"7\"", "id=\"2\"")
        .replace("model=\"arm\"", "model=\"second\"");
    let sdf = WORLD
        .replace("</plugin>", &format!("{row}</plugin>"))
        .replace("</world>", &format!("{model}</world>"));
    let mut sim = load(&sdf, Some(&std::env::var("GZ_GPU_ADAPTER").unwrap())).unwrap();
    assert_eq!(execution_counts(&sim), Some((0, 2)));
    assert_eq!(
        observe(&sim, 2).unwrap().arm.target,
        observe(&sim, 7).unwrap().arm.target
    );
    set_targets(&mut sim, 2, [0.34, 0.47]).unwrap();
    sim.step_exact(60).unwrap();
    assert_eq!(observe(&sim, 2).unwrap().normal_impulse, 0.);
    assert!(observe(&sim, 7).unwrap().normal_impulse > 0.);
    sim.world_mut()
        .remove_resource::<gz_physics_gpu::PlanarToolBatch>();
    assert!(matches!(observe(&sim, 7), Err(GpuError::StaleBoundary)));
    assert!(reset_loaded_world(&mut sim, |_| ()).is_err());
}

#[test]
fn geometry_ownership_and_duplicate_geometry_are_diagnosed() {
    fn element<'a>(s: &'a str, open: &str, close: &str) -> &'a str {
        let a = s.find(open).unwrap();
        let b = s[a..].find(close).unwrap() + close.len();
        &s[a..a + b]
    }
    let floor = element(WORLD, "<model name=\"floor\"", "</model>");
    let ground = element(floor, "<collision name=\"plane\"", "</collision>");
    let tool = element(WORLD, "<collision name=\"round_tool\"", "</collision>");
    let elbow = element(WORLD, "<joint name=\"elbow\"", "</joint>");
    let cases = [
        (
            WORLD.replace(
                "</world>",
                &format!(
                    "{}</world>",
                    floor.replace("name=\"floor\"", "name=\"floor2\"")
                ),
            ),
            "one static floor",
        ),
        (
            WORLD.replace(
                floor,
                &floor.replace("</model>", "<link name=\"extra\"/></model>"),
            ),
            "one static link",
        ),
        (WORLD.replace(ground, ""), "one plane"),
        (
            WORLD.replace(
                ground,
                &ground.replace("<geometry>", "<pose>0.1 0 0 0 0 0</pose><geometry>"),
            ),
            "horizontal upward plane",
        ),
        (WORLD.replace(elbow, ""), "two revolute joints"),
        (
            WORLD.replace("<parent>upper</parent>", "<parent>world</parent>"),
            "world-rooted two-link",
        ),
        (WORLD.replace(tool, ""), "one spherical tool"),
        (
            WORLD.replace(
                tool,
                &tool.replace(
                    "<sphere><radius>0.018</radius></sphere>",
                    "<box><size>1 1 1</size></box>",
                ),
            ),
            "spherical tool",
        ),
    ];
    for (s, expected) in cases {
        let e = load(&s, Some("deliberately absent adapter"))
            .err()
            .unwrap()
            .to_string();
        assert!(e.contains(expected), "{e} expected {expected}");
    }
}
#[test]
#[ignore = "requires GZ_GPU_ADAPTER with f64 Vulkan"]
fn reversed_declaration_and_default_surface_preserve_physics_and_stale_bindings_are_rejected() {
    let shoulder = WORLD
        .lines()
        .find(|s| s.contains("<joint name=\"shoulder\""))
        .unwrap();
    let elbow = WORLD
        .lines()
        .find(|s| s.contains("<joint name=\"elbow\""))
        .unwrap();
    let other = WORLD
        .replace(shoulder, "HOLDER")
        .replace(elbow, shoulder)
        .replace("HOLDER", elbow)
        .replace(
            "<surface><friction><ode><mu>0.30</mu><mu2>0.30</mu2></ode></friction></surface>",
            "",
        );
    let mut sim = load(&other, Some(&std::env::var("GZ_GPU_ADAPTER").unwrap())).unwrap();
    assert_eq!(observe(&sim, 7).unwrap().friction, 0.5);
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::arm::elbow")
        })
        .unwrap()
        .id();
    let definition = sim
        .world_mut()
        .entity_mut(entity)
        .take::<gz_sim_scene::JointDefinition>()
        .unwrap();
    assert!(matches!(
        set_targets(&mut sim, 7, [0.; 2]),
        Err(GpuError::StaleBoundary)
    ));
    sim.world_mut().entity_mut(entity).insert(definition);
    set_targets(&mut sim, 7, [0.; 2]).unwrap();
    sim.world_mut()
        .remove_resource::<gz_sim_scene::gpu_scene::planar_tool::ToolBindings>();
    assert!(sim.step_exact(1).is_err());
}

const COMMAND_WORLD: &str = include_str!("../../../examples/gpu_commanded_tool_world.sdf");

#[test]
fn standard_tool_controllers_reject_conflicting_drives_and_invalid_limits() {
    for (from, to) in [
        ("damping=\"6\"", "damping=\"6\" stiffness=\"1\""),
        ("damping=\"6\"", "damping=\"0\""),
        ("damping=\"6\"", "damping=\"6\" target=\"0.1\""),
        ("<velocity>3</velocity>", "<velocity>0</velocity>"),
        ("<cmd_max>2.4</cmd_max>", "<cmd_max>-1</cmd_max>"),
        (
            "</environment>",
            "<target tick=\"2\" shoulder=\"1\" elbow=\"1\"/></environment>",
        ),
        (
            "<joint_name>elbow</joint_name><use_velocity_commands>",
            "<joint_name>shoulder</joint_name><use_velocity_commands>",
        ),
    ] {
        let error = load(&COMMAND_WORLD.replace(from, to), Some("absent adapter"))
            .err()
            .unwrap()
            .to_string();
        assert!(!error.contains("was not found"), "{from}: {error}");
    }
}

#[test]
#[ignore = "requires GZ_GPU_ADAPTER with f64 Vulkan"]
fn standard_commands_reach_contact_then_lift_and_reset_to_the_same_motion() {
    use gz_sim_scene::{
        JointStateSample, initial_joint_controller_configurations,
        joint_controller_binding_matches, set_joint_controller_command,
    };
    let mut sim = load(
        COMMAND_WORLD,
        Some(&std::env::var("GZ_GPU_ADAPTER").unwrap()),
    )
    .unwrap();
    let ids: Vec<_> = initial_joint_controller_configurations(&sim)
        .unwrap()
        .map(|(id, _)| id)
        .collect();
    assert_eq!(ids.len(), 2);
    assert!(
        ids.iter()
            .all(|id| joint_controller_binding_matches(&sim, *id))
    );
    assert!(set_targets(&mut sim, 7, [0.; 2]).is_err());
    assert!(set_joint_controller_command(&mut sim, ids[0], f64::INFINITY).is_err());
    assert!(set_joint_controller_command(&mut sim, 7, 1.).is_err());
    sim.step_exact(1).unwrap();
    let neutral = *observe(&sim, 7).unwrap();
    let mut trace = Vec::new();
    for episode in 0..2 {
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(observe(&sim, 7).unwrap().arm.angle, neutral.arm.angle);
        let mut first = None;
        let mut last = None;
        for step in 2..=600 {
            if step == 2 || step == 301 {
                let commands = if step == 2 { [-8., 0.12] } else { [8., 0.47] };
                for (i, id) in ids.iter().enumerate() {
                    set_joint_controller_command(&mut sim, *id, commands[i]).unwrap();
                }
            }
            sim.step_exact(1).unwrap();
            let lane = observe(&sim, 7).unwrap();
            for i in 0..2 {
                assert!(lane.velocity_target[i].abs() <= [2.4, 3.2][i]);
                assert!(lane.arm.effort[i].abs() <= [8., 2.8][i]);
            }
            if step == 2 || step == 301 {
                assert_eq!(lane.velocity_target[0], if step == 2 { -2.4 } else { 2.4 });
            }
            let state = sim
                .world()
                .iter_entities()
                .find_map(|e| e.get::<JointStateSample>())
                .unwrap();
            assert_eq!(state.joints.len(), 2);
            assert_eq!(state.time.as_millis(), step as u128);
            assert_eq!(state.joints[0].velocity, lane.arm.speed[0]);
            if lane.normal_impulse > 0. {
                first.get_or_insert(step);
                last = Some(step);
            }
            let point = (
                lane.arm.angle,
                lane.arm.speed,
                lane.arm.effort,
                lane.normal_impulse,
            );
            if episode == 0 {
                trace.push(point)
            } else {
                assert_eq!(trace[step - 2], point);
            }
        }
        assert!(first.unwrap() < 301 && last.unwrap() < 600);
        eprintln!("contact first {first:?}, last {last:?}");
    }
}

#[test]
#[ignore = "requires GZ_GPU_ADAPTER with f64 Vulkan"]
fn velocity_motor_saturates_resets_and_rejects_a_replaced_public_configuration() {
    use gz_sim_scene::{
        initial_joint_controller_configurations, joint_controller_binding_matches,
        joint_controller_binding_matches_configuration, set_joint_controller_command,
    };
    let scene=COMMAND_WORLD.replacen("gz::sim::systems::JointPositionController","gz::sim::systems::JointController",1)
        .replacen("gz-sim-joint-position-controller-system","gz-sim-joint-controller-system",1)
        .replace("<use_velocity_commands>true</use_velocity_commands><cmd_max>2.4</cmd_max>","<use_force_commands>false</use_force_commands><initial_velocity>0.4</initial_velocity>");
    let mut sim = load(&scene, Some(&std::env::var("GZ_GPU_ADAPTER").unwrap())).unwrap();
    let (id, cfg) = initial_joint_controller_configurations(&sim)
        .unwrap()
        .next()
        .map(|(id, c)| (id, c.clone()))
        .unwrap();
    assert!(cfg.topic.ends_with("/cmd_vel"));
    sim.step_exact(1).unwrap();
    assert_eq!(observe(&sim, 7).unwrap().velocity_target[0], 0.4);
    for value in [-30., 30.] {
        set_joint_controller_command(&mut sim, id, value).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(
            observe(&sim, 7).unwrap().velocity_target[0],
            value.signum() * 3.
        );
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(observe(&sim, 7).unwrap().velocity_target[0], 0.4);
    let e = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
    let mut different = cfg.clone();
    different.topic = "/wrong".into();
    sim.world_mut().entity_mut(e).insert(different.clone());
    assert!(!joint_controller_binding_matches(&sim, id));
    assert!(!joint_controller_binding_matches_configuration(
        &sim, id, &different
    ));
    assert!(joint_controller_binding_matches_configuration(
        &sim, id, &cfg
    ));
    assert!(set_joint_controller_command(&mut sim, id, 2.).is_err());
    assert!(sim.step_exact(1).is_err());
}

#[test]
fn controlled_tool_population_bounds_and_route_aliases_are_validated_before_adapter_selection() {
    let make = |count: usize, shared: bool| {
        let base = if shared {
            COMMAND_WORLD.replace(
                "<cmd_max>2.4</cmd_max>",
                "<cmd_max>2.4</cmd_max><topic>/shared</topic>",
            )
        } else {
            COMMAND_WORLD.to_owned()
        };
        let start = base.find("<model name=\"arm\"").unwrap();
        let end = start + base[start..].find("</model>").unwrap() + 8;
        let model = &base[start..end];
        let row =
            &base[base.find("<environment ").unwrap()..base.find("</environment>").unwrap() + 14];
        let mut models = String::new();
        let mut rows = String::new();
        for i in 1..count {
            let name = format!("arm{i}");
            models += &model
                .replace("name=\"arm\"", &format!("name=\"{name}\""))
                .replace("0 0 0.60", &format!("0 {i} 0.60"));
            rows += &row
                .replace("id=\"7\"", &format!("id=\"{}\"", 10 + i))
                .replace("model=\"arm\"", &format!("model=\"{name}\""));
        }
        base.replacen("</environment>", &format!("</environment>{rows}"), 1)
            .replace("</world>", &format!("{models}</world>"))
    };
    for (count, shared, reason) in [
        (2, true, "duplicate tool joint-controller topics"),
        (9, false, "multiple joint-controller chains"),
    ] {
        let error = load(&make(count, shared), Some("absent adapter"))
            .err()
            .unwrap()
            .to_string();
        assert!(error.contains(reason), "{error}");
    }
}

#[test]
#[ignore = "requires GZ_GPU_ADAPTER with f64 Vulkan"]
fn loaded_single_sided_shoulder_and_locked_elbow_survive_reset_without_floor_impulses() {
    use gz_sim_scene::{initial_joint_controller_configurations, set_joint_controller_command};
    let xml = include_str!("../../../examples/gpu_stopped_tool_world.sdf")
        .replace("<upper>0.02</upper>", "")
        .replace(
            "<lower>-0.15</lower><upper>1.2</upper>",
            "<lower>0</lower><upper>0</upper>",
        )
        .replace("<pose>0 0 0.65 0 0 0</pose>", "<pose>0 0 -2 0 0 0</pose>");
    let mut sim = load(&xml, Some(&std::env::var("GZ_GPU_ADAPTER").unwrap())).unwrap();
    let ids: Vec<_> = initial_joint_controller_configurations(&sim)
        .unwrap()
        .map(|(id, _)| id)
        .collect();
    let mut first = Vec::new();
    for repeat in 0..2 {
        if repeat == 1 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        let mut lower = false;
        let mut elbow = false;
        for tick in 1..=400 {
            if tick == 1 || tick == 220 {
                set_joint_controller_command(&mut sim, ids[0], if tick == 1 { -1. } else { 2. })
                    .unwrap();
                set_joint_controller_command(&mut sim, ids[1], if tick == 1 { 0.8 } else { -0.8 })
                    .unwrap();
            }
            sim.step_exact(1).unwrap();
            let l = observe(&sim, 7).unwrap();
            lower |= l.stop_impulse[0] > 0.;
            elbow |= l.stop_impulse[1] != 0.;
            assert_eq!(l.normal_impulse, 0.);
            assert!(l.joint_angles()[0] - 0.48 >= -0.36 - 1e-12);
            assert!((l.joint_angles()[1] + 0.62).abs() < 1e-12);
            assert!(l.arm.speed[1].abs() < 1e-10);
            if repeat == 0 {
                first.push((l.arm.angle, l.arm.speed, l.stop_impulse));
            } else {
                assert_eq!(first[tick - 1], (l.arm.angle, l.arm.speed, l.stop_impulse));
            }
        }
        assert!(lower && elbow);
        assert_eq!(observe(&sim, 7).unwrap().stop_impulse[0], 0.);
    }
}

#[test]
fn loaded_hinge_must_start_within_its_authored_travel() {
    let input = include_str!("../../../examples/gpu_stopped_tool_world.sdf")
        .replace("<lower>-0.36</lower>", "<lower>0.01</lower>");
    let reason = load(&input, Some("missing adapter"))
        .err()
        .unwrap()
        .to_string();
    assert!(reason.contains("outside travel"), "{reason}");
}
