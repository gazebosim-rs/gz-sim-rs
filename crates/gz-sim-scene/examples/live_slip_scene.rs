// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Write the live ordinary SDF with material point markers and native frame rails.
#[allow(dead_code)]
#[path = "support/inclined_sheet.rs"]
mod sheet;
#[allow(dead_code)]
#[path = "support/slip_transition.rs"]
mod transition;
use glam::{DQuat, DVec3, EulerRot};
use std::{error::Error, fmt::Write as _, io::Write};
fn main() -> Result<(), Box<dyn Error>> {
    let path = std::env::args_os()
        .nth(1)
        .ok_or("supply the new SDF path")?;
    let cfg = transition::Run {
        mu: 0.15,
        speed: -0.42,
        width: 250000,
        transition: Some(2000),
        yaw: 0.,
        extra_mass: 0.002,
    };
    let mut src = cfg.sdf();
    let sim = sheet::Incline::new(&src)?;
    let states = sim.states();
    let q = DQuat::from_array(states[0].orientation).conjugate();
    let origin = DVec3::from(states[0].position);
    let mut visuals = String::new();
    let mut bind = String::new();
    for i in 0..4 {
        writeln!(
            visuals,
            r#"        <visual name="cloth_corner_{i}"><geometry><sphere><radius>0.030</radius></sphere></geometry><material><diffuse>1 0.51 0.12 1</diffuse></material></visual>"#
        )?;
        writeln!(
            bind,
            r#"      <particle_visual material="material" particle="{i}" visual="default::gripper::palm::cloth_corner_{i}"/>"#
        )?;
    }
    for (i, j) in [(0, 1), (1, 2), (2, 3), (3, 0)] {
        let a = q * (DVec3::from(states[i].position) - origin);
        let b = q * (DVec3::from(states[j].position) - origin);
        let c = (a + b) * 0.5;
        let length = a.distance(b);
        let (y, p, r) =
            DQuat::from_rotation_arc(DVec3::X, (b - a) / length).to_euler(EulerRot::ZYX);
        writeln!(
            visuals,
            r#"        <visual name="rail_{i}_{j}"><pose>{} {} {} {r} {p} {y}</pose><geometry><box><size>{length} 0.030 0.030</size></box></geometry><material><diffuse>0.20 0.42 0.58 1</diffuse></material></visual>"#,
            c.x, c.y, c.z
        )?;
    }
    src = src
        .replacen("      </link>", &(visuals + "      </link>"), 1)
        .replace(
            "      <articulated_impact",
            &(bind + "      <articulated_impact"),
        );
    std::fs::File::create_new(path)?.write_all(src.as_bytes())?;
    Ok(())
}
