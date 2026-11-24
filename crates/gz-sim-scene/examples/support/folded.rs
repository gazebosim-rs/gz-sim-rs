// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
pub const FOLD: &str = include_str!("../../../../examples/articulated_folded_sheet.sdf");
pub const R: f64 = 0.12;
pub const SKIN: f64 = 0.006;
pub const WIDTH: f64 = 0.24;
pub const SLOPE: f64 = 0.5;
pub const IZ: f64 = 0.022;
pub fn sdf(mu: f64, width: u64, spin: bool) -> String {
    let mut src = FOLD
        .replace("friction=\"0\"", &format!("friction=\"{mu}\""))
        .replace(
            "max_contact_step_ns=\"250000\"",
            &format!("max_contact_step_ns=\"{width}\""),
        );
    if spin {
        src = src
            .replacen("mass=\"0.1\" velocity", "mass=\"0.21\" velocity", 2)
            .replace("velocity=\"0 0 0\"", "velocity=\"0.27 0 0\"");
    }
    src
}
pub fn normal(side: f64) -> DVec3 {
    DVec3::new(0., side * SLOPE, 1.).normalize()
}
pub fn cold_weights() -> (f64, f64) {
    let outer = (R + SKIN) * normal(1.).y / WIDTH;
    ((1. - outer) / 2., outer)
}
/// Independent force at t=0. Each triangle loads its own side node
/// and the two shared ridge nodes. Centered radial normal forces do
/// zero work on the sphere's axial degree of freedom.
pub fn cold_pressure() -> (f64, f64) {
    let (ridge, side) = cold_weights();
    let diag = 10. * (2. * ridge * ridge + side * side);
    let cross = 20. * ridge * ridge * normal(1.).dot(normal(-1.));
    let weight = 9.81 * normal(1.).z;
    (weight / (diag + cross), weight / diag)
}
pub const OBLIQUE: &str = include_str!("../../../../examples/articulated_ballistic_cloth.sdf");
pub fn oblique(mu: f64, width: u64) -> String {
    OBLIQUE
        .replace("friction=\"0.9\"", &format!("friction=\"{mu}\""))
        .replace(
            "detection=\"first_impact\" restitution=\"0.55\"",
            &format!(
                "detection=\"persistent_contact\" restitution=\"0\" max_contact_step_ns=\"{width}\""
            ),
        )
}
