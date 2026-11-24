// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_physics_distributed::articulation::NativeArticulation;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{
    PhysicsEngine, distributed_hinge_spec, load_world_with_sdf_step, queue_joint_effort,
};

#[test]
fn child_authored_first_oblique_hinge_replays_the_ordinary_bevy_native_image()
-> Result<(), Box<dyn std::error::Error>> {
    let sdf = include_str!("../../../examples/multifidelity_articulation.sdf");
    let first = sdf.find("      <link name=\"base\">").unwrap();
    let second = sdf.find("      <link name=\"tool\">").unwrap();
    let after = sdf.find("      <joint name=\"hinge\"").unwrap();
    let swapped = [
        &sdf[..first],
        &sdf[second..after],
        &sdf[first..second],
        &sdf[after..],
    ]
    .concat()
    .replace(">0 0 1</xyz>", ">0.36 0.48 0.8</xyz>")
    .replace("0.07 -0.03 0 0 0 0", "0.07 -0.03 0 0.21 -0.12 0.36")
    .replace(
        "<iyy>0.12</iyy><izz>0.12</izz>",
        "<iyy>0.14</iyy><izz>0.13</izz><ixz>0.003</ixz>",
    );
    let spec = distributed_hinge_spec(&swapped, "child_first.sdf", "default")?;
    assert!(spec.links[0].id > spec.links[1].id);
    let mut ordinary = load_world_with_sdf_step(
        &swapped,
        "child_first.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut worker = NativeArticulation::new(&spec)?;
    for tick in 1..=288 {
        let effort = match tick {
            1..=44 => Some(1.375),
            91..=149 => Some(-2.125),
            _ => None,
        };
        if let Some(e) = effort {
            queue_joint_effort(&mut ordinary, spec.joint_id, e)?;
            worker.effort(spec.joint_id, e)?;
        }
        ordinary.step_exact(1)?;
        worker.advance()?;
        let native = ordinary
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .checkpoint_floating_revolute()?;
        let image = worker.capture()?;
        assert_eq!(spec.warm_payload(tick, &image)?, native, "tick {tick}");
        if tick % 37 == 0 {
            worker = NativeArticulation::restore(&spec, tick, &image)?;
        }
    }
    assert!(worker.observe().q.abs() > 0.001);
    Ok(())
}

#[test]
fn elevated_ground_authored_last_and_child_authored_first_recover_loaded_contacts()
-> Result<(), Box<dyn std::error::Error>> {
    let xml = include_str!("../../../examples/distributed_grounded_hinge.sdf");
    let floor = xml.find("    <model name=\"ground\">").unwrap();
    let robot = xml.find("    <model name=\"robot\">").unwrap();
    let base = xml.find("      <link name=\"base\">").unwrap();
    let tool = xml.find("      <link name=\"tool\">").unwrap();
    let joint = xml.find("      <joint name=\"hinge\"").unwrap();
    let end = xml.find("  </world>").unwrap();
    let xml = [
        &xml[..floor],
        &xml[robot..base],
        &xml[tool..joint],
        &xml[base..tool],
        &xml[joint..end],
        &xml[floor..robot],
        &xml[end..],
    ]
    .concat()
    .replace(
        "<static>true</static>",
        "<static>true</static><pose>0 0 0.10 0 0 0</pose>",
    );
    let spec = distributed_hinge_spec(&xml, "reordered.sdf", "default")?;
    assert!(spec.ground.unwrap().link_id > spec.links[0].id && spec.links[0].id > spec.links[1].id);
    let mut ordinary = load_world_with_sdf_step(
        &xml,
        "reordered.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut worker = NativeArticulation::new(&spec)?;
    let mut restored = 0;
    for tick in 1..=300 {
        ordinary.step_exact(1)?;
        worker.advance()?;
        let expected = ordinary
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .checkpoint_grounded_revolute()?;
        let image = worker.capture()?;
        assert_eq!(spec.warm_payload(tick, &image)?, expected, "tick {tick}");
        if tick % 17 == 0
            && worker
                .observe()
                .ground_reactions
                .unwrap()
                .iter()
                .any(|r| r.normal_impulse > 0.)
        {
            worker = NativeArticulation::restore(&spec, tick, &image)?;
            restored += 1;
        }
    }
    assert!(
        restored >= 4,
        "expected repeated loaded snapshots above a raised plane"
    );
    Ok(())
}
