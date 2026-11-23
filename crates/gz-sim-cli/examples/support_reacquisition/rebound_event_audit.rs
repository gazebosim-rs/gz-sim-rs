// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent pre-solve readback at each native event in the archived continuous runs.
use super::*;

#[derive(serde::Deserialize)]
struct Kinematics {
    position: [f64; 3],
    orientation: [f64; 4],
    velocity: [f64; 3],
    angular_velocity: [f64; 3],
}

#[derive(Clone, Copy)]
struct Surface {
    center: DVec3,
    rotation: DQuat,
    velocity: DVec3,
    omega: DVec3,
}

impl Surface {
    // Enumerate the vertices of the projected intersection independently of
    // the live polygon clip. Every extreme is either a sole vertex, a deck
    // vertex projected onto the sole plane, or a boundary-edge crossing.
    fn vertices(self) -> Vec<DVec3> {
        let [lx, ly, h] = FOOT_HALF;
        let [hx, hy, _] = PAD_HALF;
        let eps = 1e-12;
        let in_deck = |p: DVec3| p.x.abs() <= hx + eps && p.y.abs() <= hy + eps;
        let corners = [(-lx, -ly), (lx, -ly), (lx, ly), (-lx, ly)]
            .map(|(x, y)| self.center + self.rotation * DVec3::new(x, y, -h));
        let mut result: Vec<_> = corners.into_iter().filter(|p| in_deck(*p)).collect();
        let plane = self.rotation * DVec3::Z;
        for x in [-hx, hx] {
            for y in [-hy, hy] {
                let z = self.center.z
                    + (-h - plane.x * (x - self.center.x) - plane.y * (y - self.center.y))
                        / plane.z;
                let p = DVec3::new(x, y, z);
                let local = self.rotation.conjugate() * (p - self.center);
                if local.x.abs() <= lx + eps && local.y.abs() <= ly + eps {
                    result.push(p);
                }
            }
        }
        for i in 0..4 {
            let (a, b) = (corners[i], corners[(i + 1) % 4]);
            for (axis, half) in [(0, hx), (1, hy)] {
                let d = b[axis] - a[axis];
                if d == 0. {
                    continue;
                }
                for edge in [-half, half] {
                    let t = (edge - a[axis]) / d;
                    if (0.0..=1.0).contains(&t) {
                        let p = a + (b - a) * t;
                        if in_deck(p) {
                            result.push(p);
                        }
                    }
                }
            }
        }
        result
    }

    fn observe(self) -> Result<Value> {
        let points = self.vertices();
        let low = points
            .iter()
            .map(|p| p.z)
            .reduce(f64::min)
            .ok_or("event footprint missed the deck")?;
        let closing = |p: &DVec3| -(self.velocity.z + self.omega.cross(*p - self.center).z);
        let speed = points
            .iter()
            .filter(|p| p.z <= low + 1e-12)
            .map(closing)
            .fold(f64::NEG_INFINITY, f64::max);
        let penetrating = points
            .iter()
            .filter(|p| p.z <= PAD_HALF[2])
            .map(closing)
            .reduce(f64::max);
        let whole_low = self.center.z
            - (self.rotation.conjugate() * DVec3::Z)
                .abs()
                .dot(FOOT_HALF.into());
        Ok(
            json!({"supported_gap_m":low-PAD_HALF[2],"whole_box_gap_m":whole_low-PAD_HALF[2],
            "closing_at_lowest_m_s":speed,"fastest_penetrating_vertex_m_s":penetrating}),
        )
    }
}

pub(in super::super) fn write_event_audit(out: &Path) -> Result<()> {
    let runs: Vec<Value> = serde_json::from_slice(&fs::read(out.join("continuous.json"))?)?;
    let dt = STEP_US as f64 * 1e-6;
    let mut rows = Vec::new();
    for run in runs {
        for e in run["landings"].as_array().ok_or("missing native events")? {
            let f: Kinematics = serde_json::from_value(e["incoming_foot"].clone())?;
            let p: Kinematics = serde_json::from_value(e["incoming_pad"].clone())?;
            let incoming = Surface {
                center: DVec3::from(f.position) - DVec3::from(p.position),
                rotation: DQuat::from_array(f.orientation),
                velocity: DVec3::from(f.velocity) - DVec3::from(p.velocity),
                omega: f.angular_velocity.into(),
            };
            let end = Surface {
                center: incoming.center + incoming.velocity * dt
                    - DVec3::Z * (0.5 * GRAVITY_M_S2 * dt * dt),
                velocity: incoming.velocity - DVec3::Z * GRAVITY_M_S2 * dt,
                rotation: DQuat::from_scaled_axis(incoming.omega * dt) * incoming.rotation,
                ..incoming
            };
            let before = incoming.observe()?;
            let after = end.observe()?;
            let geometry_speed_candidate = after["fastest_penetrating_vertex_m_s"]
                .as_f64()
                .is_some_and(|c| c >= MIN_CLOSING_M_S);
            rows.push(json!({"deck":run["deck"],"switched":run["switched"],"policy":run["policy"],
                "cycle":e["cycle"],"tick":e["tick"],"peak_gap_m":e["peak_gap_m"],"native_first_ns":e["normal_ns"],
                "selected":e["selected"],"incoming":before,"ballistic_end":after,"end_geometry_and_speed_qualify":geometry_speed_candidate}));
        }
    }
    fs::write(
        out.join("event-audit.json"),
        serde_json::to_vec_pretty(&json!({"outer_us":STEP_US,
        "closing_positive_is_approaching":true,"minimum_closing_m_s":MIN_CLOSING_M_S,"release_witness_m":RELEASE_M,"events":rows}))?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn independent_half_space_vertices_recover_the_supported_clearance() {
        let pad = BodyState {
            position: [0., 0., -PAD_HALF[2]],
            orientation: DQuat::IDENTITY.to_array(),
            velocity: [0.; 3],
            angular_velocity: [0.; 3],
        };
        for x in [-0.6, -0.33, 0., 0.32, 0.5] {
            for y in [-0.20, -0.14, 0., 0.20] {
                for yaw in [0., 0.32, 1.55] {
                    let q = DQuat::from_rotation_z(yaw)
                        * DQuat::from_rotation_x(0.11)
                        * DQuat::from_rotation_y(-0.06);
                    let foot = BodyState {
                        position: [x, y, 0.06],
                        orientation: q.to_array(),
                        ..pad
                    };
                    let surface = Surface {
                        center: relative_center(foot, pad),
                        rotation: q,
                        omega: DVec3::ZERO,
                        velocity: DVec3::ZERO,
                    };
                    let independent = surface
                        .vertices()
                        .iter()
                        .map(|p| p.z - PAD_HALF[2])
                        .reduce(f64::min);
                    match (supported_gap(foot, pad), independent) {
                        (Some(a), Some(b)) => {
                            assert!((a - b).abs() < 1e-12, "x={x} y={y} yaw={yaw}")
                        }
                        (None, None) => (),
                        _ => panic!("disjointness differs at x={x} y={y} yaw={yaw}"),
                    }
                }
            }
        }
    }
}
