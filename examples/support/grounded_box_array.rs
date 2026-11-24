// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One free-root star with independent hinged cuboids and up to four floor witnesses per cuboid.
use std::fmt::Write;

pub fn boxes(n: usize) -> String {
    let mut xml = String::from(
        "<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity><physics name='p' type='ignored'><max_step_size>0.001</max_step_size></physics><model name='floor'><static>true</static><link name='ground'><collision name='c'><geometry><plane><normal>0 0 1</normal><size>5 5</size></plane></geometry></collision></link></model><model name='feet'><self_collide>false</self_collide><plugin filename='gz-sim-switchable-articulation-system' name='gz::sim::systems::SwitchableArticulation'/>",
    );
    for i in 0..n {
        let (x, y) = ((i % 8) as f64 * 0.16, (i / 8) as f64 * 0.16);
        write!(&mut xml,"<link name='b{i}'><pose>{x} {y} 0.061 0 0 0</pose><inertial><mass>1</mass><inertia><ixx>0.0024</ixx><iyy>0.0024</iyy><izz>0.0024</izz></inertia></inertial><collision name='c'><geometry><box><size>0.12 0.12 0.12</size></box></geometry></collision></link>").unwrap();
    }
    for i in 1..n {
        write!(&mut xml,"<joint name='hinge{i}' type='revolute'><parent>b0</parent><child>b{i}</child><axis><xyz>0 1 0</xyz><limit><lower>-0.22</lower><upper>0.22</upper></limit></axis></joint>").unwrap();
    }
    xml.push_str("</model></world></sdf>");
    xml
}
