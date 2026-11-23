// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Keep the SDF admission boundary for local scalar and spherical joints explicit.
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;

const WORLD: &str = include_str!("../../../examples/worlds/local_robot_joints.sdf");
const SLIDE_AXIS: &str =
    "<axis><xyz>1 0 0</xyz><limit><lower>-0.40</lower><upper>0.40</upper></limit></axis>";

#[test]
fn local_sdf_rejects_missing_slider_axis_and_invalid_ball_or_slider_coordinates() {
    let wrist_end = "<pose>0 0 0.23 0 0 0</pose></joint>";
    let replacements = [
        (SLIDE_AXIS, ""),
        (
            SLIDE_AXIS,
            "<axis><xyz>1 0 0</xyz></axis><axis2><xyz>0 1 0</xyz></axis2>",
        ),
        ("<lower>-0.40</lower>", "<lower>0.50</lower>"),
        ("<upper>0.40</upper>", "<upper>-0.50</upper>"),
        (
            wrist_end,
            "<pose>0 0 0.23 0 0 0</pose><axis><xyz>0 0 1</xyz></axis></joint>",
        ),
        (
            wrist_end,
            "<pose>0 0 0.23 0 0 0</pose><axis2><xyz>0 0 1</xyz></axis2></joint>",
        ),
    ];
    for (from, to) in replacements {
        assert!(WORLD.contains(from));
        let source = WORLD.replace(from, to);
        assert!(
            load_world_with_sdf_step(
                &source,
                "bad-robot.sdf",
                "robot_joints",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err(),
            "admitted {from} -> {to}"
        );
    }
}

#[test]
fn foreign_backends_keep_their_previous_joint_admission_domain() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        assert!(
            load_world_with_sdf_step(WORLD, "robot.sdf", "robot_joints", &NoResources, engine)
                .is_err()
        );
        // Remove the spherical joint entirely to check the slider policy independently.
        let start = WORLD.find("<joint name=\"wrist\"").unwrap();
        let stop = start + WORLD[start..].find("</joint>").unwrap() + 8;
        let just_slider = format!("{}{}", &WORLD[..start], &WORLD[stop..]);
        assert!(
            load_world_with_sdf_step(
                &just_slider,
                "slider.sdf",
                "robot_joints",
                &NoResources,
                engine
            )
            .is_err()
        );
    }
}
