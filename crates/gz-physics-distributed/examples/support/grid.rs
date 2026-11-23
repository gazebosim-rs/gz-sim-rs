// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Parameterized ordinary Gazebo SDF with independently dropping two-box stacks.
use std::fmt::Write;
pub fn sdf(count: usize) -> String {
    assert!(count > 0 && count <= gz_physics_distributed::physics::MAX_PARTITION_BODIES);
    let columns = ((count.div_ceil(2) as f64).sqrt().ceil()) as usize;
    let rows = count.div_ceil(2).div_ceil(columns);
    let mut s = String::from(
        r#"<?xml version="1.0"?>
<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity>
<physics name="workers" type="ignored"><max_step_size>0.005</max_step_size><real_time_factor>0.2</real_time_factor></physics>
"#,
    );
    let width = 8 * columns + 1;
    let depth = 8 * rows + 1;
    writeln!(s,r#"<model name="ground"><static>true</static><link name="floor">
<collision name="soil"><geometry><plane><normal>0 0 1</normal><size>{width} {depth}</size></plane></geometry><surface><friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode></friction></surface></collision>
<visual name="surface"><pose>0 0 -0.025 0 0 0</pose><geometry><box><size>{width} {depth} 0.05</size></box></geometry><material><diffuse>0.20 0.27 0.32 1</diffuse></material></visual></link></model>"#).unwrap();
    for id in 0..count {
        let cell = id / 2;
        let x = ((cell % columns) as f64 - (columns - 1) as f64 * 0.5) * 8.;
        let y = ((cell / columns) as f64 - (rows - 1) as f64 * 0.5) * 8.;
        let top = id % 2 == 1;
        let z = if top {
            2.14 + 0.06 * (cell % 7) as f64
        } else {
            0.64 + 0.03 * (cell % 3) as f64
        };
        let mass = if top { 2.0 } else { 1.0 };
        let (ix, iz) = (
            mass * (0.55_f64.powi(2) + 0.4_f64.powi(2)) / 12.,
            mass * 0.55_f64.powi(2) / 6.,
        );
        let paint = if top {
            "0.95 0.44 0.09"
        } else {
            "0.03 0.73 0.70"
        };
        writeln!(s,r#"<model name="box_{id}"><pose>{x} {y} {z} 0 0 0</pose><link name="body">
<inertial><mass>{mass}</mass><inertia><ixx>{ix}</ixx><iyy>{ix}</iyy><izz>{iz}</izz></inertia></inertial>
<collision name="hull"><geometry><box><size>0.55 0.55 0.4</size></box></geometry></collision>
<visual name="paint"><geometry><box><size>0.55 0.55 0.4</size></box></geometry><material><diffuse>{paint} 1</diffuse></material></visual></link></model>"#).unwrap();
    }
    s.push_str("</world></sdf>\n");
    s
}
