// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Prospective contact kinematics recovered from the airborne locked-foot state.
//! The driving pad is horizontal and translates vertically. All speeds here are
//! pre-impact rigid-body velocities, independent of subsequent solver outputs.
use super::*;

#[derive(Clone, Copy, Serialize)]
pub(super) struct Signature {
    pub(super) support_height_span_mm: f64,
    pub(super) prospective_tangent_m_s: f64,
    pub(super) supported_projected_area_fraction: f64,
    pub(super) spin_rad_s: f64,
    pub(super) recorded_gap_mm: f64,
}

#[derive(Deserialize)]
struct Incoming {
    com_m: [f64; 3],
    com_velocity_m_s: [f64; 3],
    foot_pose_xyzw: [f64; 4],
    foot_angular_velocity_rad_s: [f64; 3],
    gap_m: f64,
    normal_ns: f64,
}

fn projected_area(points: &[DVec3]) -> f64 {
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .map(|(a, b)| a.x * b.y - a.y * b.x)
        .sum::<f64>()
        .abs()
        * 0.5
}

fn clip(points: &[DVec3], axis: usize, edge: f64, keep_above: bool) -> Vec<DVec3> {
    let keep = |v: DVec3| {
        if keep_above {
            v[axis] >= edge
        } else {
            v[axis] <= edge
        }
    };
    let mut out = Vec::with_capacity(points.len() + 1);
    for (&a, &b) in points.iter().zip(points.iter().cycle().skip(1)) {
        if keep(a) != keep(b) {
            let fraction = (edge - a[axis]) / (b[axis] - a[axis]);
            out.push(a.lerp(b, fraction));
        }
        if keep(b) {
            out.push(b);
        }
    }
    out
}

pub(super) fn recover(raw: &Value, offset: f64) -> Result<Option<Signature>> {
    let donor: Incoming = serde_json::from_value(raw.clone())?;
    if donor.normal_ns != 0. {
        return Err("prospective classification requires an unloaded cut".into());
    }
    measure(
        donor.com_m,
        donor.com_velocity_m_s,
        donor.foot_pose_xyzw,
        donor.foot_angular_velocity_rad_s,
        donor.gap_m,
        offset,
    )
}
/// Endpoint material speed is kinematic. A tilted moving sole can have a low-speed point even in flight.
pub(super) fn measure(
    position: [f64; 3],
    velocity: [f64; 3],
    quaternion: [f64; 4],
    angular: [f64; 3],
    gap: f64,
    offset: f64,
) -> Result<Option<Signature>> {
    let orientation = DQuat::from_array(quaternion);
    let com = DVec3::from(position);
    let translation = DVec3::from(velocity);
    let spin = DVec3::from(angular);
    if !com.is_finite()
        || !translation.is_finite()
        || !spin.is_finite()
        || !orientation.is_finite()
        || (orientation.length() - 1.).abs() > 1e-8
        || !gap.is_finite()
    {
        return Err("invalid locked-foot state for kinematic classification".into());
    }
    // Dimensions and locked link separation from multifidelity_support_reacquisition.sdf.
    // Two equal masses with their centers 140 mm apart put the level sole 130 mm below COM.
    let sole = [(-0.18, -0.10), (0.18, -0.10), (0.18, 0.10), (-0.18, 0.10)]
        .map(|(x, y)| com + orientation * DVec3::new(x, y, -0.13));
    let full_area = projected_area(&sole);
    if full_area < 1e-10 || (orientation * DVec3::Z).z <= 0. {
        return Err("fixture sole is outside the upright projected-contact regime".into());
    }
    let mut polygon = sole.to_vec();
    for (axis, edge, above) in [
        (0, offset - 0.33, true),
        (0, offset + 0.33, false),
        (1, -0.18, true),
        (1, 0.18, false),
    ] {
        polygon = clip(&polygon, axis, edge, above);
    }
    let supported_area = projected_area(&polygon);
    if supported_area < 1e-14 {
        return Ok(None);
    }
    let low = polygon.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    let high = polygon
        .iter()
        .map(|p| p.z)
        .fold(f64::NEG_INFINITY, f64::max);
    // Co-lowest vertices on a face/edge can have different velocities. A 1 nm
    // coplanarity tolerance gives a reproducible representative material point.
    let mut low_points = Vec::<DVec3>::new();
    for &p in &polygon {
        if p.z <= low + 1e-9 && !low_points.iter().any(|q| p.distance(*q) < 1e-12) {
            low_points.push(p);
        }
    }
    let contact = low_points.iter().copied().sum::<DVec3>() / low_points.len() as f64;
    let material_velocity = translation + spin.cross(contact - com);
    Ok(Some(Signature {
        support_height_span_mm: (high - low) * 1000.,
        prospective_tangent_m_s: material_velocity.x.hypot(material_velocity.y),
        supported_projected_area_fraction: supported_area / full_area,
        spin_rad_s: spin.length(),
        recorded_gap_mm: gap * 1000.,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn tilted_sole_is_clipped_before_selecting_prospective_slip() {
        let theta = 0.1_f64;
        let q = DQuat::from_rotation_y(theta);
        let d = json!({"com_m":[0.,0.,0.2],"com_velocity_m_s":[1.,0.,-1.],
            "foot_pose_xyzw":q.to_array(),"foot_angular_velocity_rad_s":[0.,1.,0.],
            "normal_ns":0.,"gap_m":0.001});
        // Right edge of foot beyond left portion of pad at x = 0.10 m.
        let sg = recover(&d, -0.23).unwrap().unwrap();
        let x_min = -0.18 * theta.cos() - 0.13 * theta.sin();
        let z_low = 0.2 - 0.10 * theta.tan() - 0.13 / theta.cos();
        assert!((sg.support_height_span_mm - (0.10 - x_min) * theta.tan() * 1000.).abs() < 1e-9);
        assert!((sg.prospective_tangent_m_s - (1. + z_low - 0.2)).abs() < 1e-10);
        assert!(recover(&d, 5.).unwrap().is_none());
    }
}
