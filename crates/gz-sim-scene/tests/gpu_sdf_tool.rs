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
