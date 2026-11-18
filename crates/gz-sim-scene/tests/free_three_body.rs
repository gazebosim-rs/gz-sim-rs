// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! TPE refuses articulated models instead of silently world-anchoring them.
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;

fn xml(reverse: bool) -> String {
    let links = [("root", "0 0 0"), ("middle", "0 0 -2"), ("leaf", "10 0 0")]
        .map(|(name, pose)| format!("<link name='{name}'><pose>{pose} 0 0 0</pose><inertial><mass>1</mass><inertia><ixx>0.004</ixx><iyy>0.004</iyy><izz>0.004</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link>")).join("");
    let first = "<joint name='first' type='revolute'><parent>root</parent><child>middle</child><pose relative_to='__model__'>0 0 -1 0 0 0</pose><axis><xyz expressed_in='__model__'>0 1 0</xyz></axis></joint>";
    let second = "<joint name='second' type='revolute'><parent>middle</parent><child>leaf</child><pose relative_to='__model__'>0 0 0 0 0 0</pose><axis><xyz expressed_in='__model__'>0 1 0</xyz></axis></joint>";
    let joints = if reverse {
        format!("{second}{first}")
    } else {
        format!("{first}{second}")
    };
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='chain'>{links}{joints}</model></world></sdf>"
    )
}

#[test]
fn free_three_body_scope_keeps_unqualified_topologies_and_limits_closed() {
    let base = xml(false);
    {
        let engine = PhysicsEngine::Tpe;
        assert!(
            load_world_with_sdf_step(
                &base,
                "unsupported-engine.sdf",
                "default",
                &NoResources,
                engine
            )
            .is_err()
        );
    }
}
