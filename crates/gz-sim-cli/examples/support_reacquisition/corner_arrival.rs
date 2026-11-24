// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Inspect the incoming, loaded side of the late rocking-corner impact.
use super::super::super::super::finite::load::MATERIAL;
use super::*;
use gz_physics::{FiniteBoxFlight, FinitePointSite, FlightBox, SeparatingFeature};

fn v(row: &Value) -> Result<DVec3> {
    let a: [f64; 3] = serde_json::from_value(row.clone())?;
    Ok(DVec3::from_array(a))
}
fn number(row: &Value) -> Result<f64> {
    row.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "missing finite scalar".into())
}
fn incoming(f: FiniteBoxFlight, geometry: &Value, mesh: u32) -> Result<Value> {
    let mut history = Value::Null;
    let (path, _) =
        complete::at_with_feature_history(f, geometry, 0.010, mesh, Some(&mut history))?;
    if path["end"] != "original_clock" {
        return Err("trajectory did not reach the first clock after corner impact".into());
    }
    let events = history["events"].as_array().ok_or("no subevent history")?;
    let [_, _, resident, corner] = events.as_slice() else {
        return Err("wrong incoming arc history".into());
    };
    if resident["feature"] != "EdgePair(0, 1)" || corner["feature"] != "SupportFace(2)" {
        return Err("unexpected corner transition".into());
    }
    // Record the endpoint from the completed incoming arc (before the selected
    // point impact) and independently cross-check it against the following arc.
    let arc = &resident["arc"];
    let elapsed = number(&history["start_s"])? + number(&corner["at_s"])?;
    let pre = FiniteBoxFlight {
        moving: FlightBox {
            center: v(&arc["foot_m"])?,
            orientation: DQuat::from_array(serde_json::from_value(
                arc["orientation_xyzw"].clone(),
            )?),
            ..f.moving
        },
        center_of_mass: v(&arc["com_m"])?,
        relative_velocity: v(&arc["velocity_m_s"])?,
        angular_velocity: v(&arc["spin_rad_s"])?,
        ..f
    };
    let material = MATERIAL;
    let site = FinitePointSite {
        feature: SeparatingFeature::EdgePair(0, 1),
        normal: v(&arc["normal"])?,
        point: v(&arc["point_m"])?,
    };
    // Mirror the constrained arc's force evaluation. The donor retains the
    // integrated velocity, including its independently bounded normal drift.
    let port = pre.relative_velocity + pre.angular_velocity.cross(site.point - pre.center_of_mass);
    let normal_drift = port.dot(site.normal);
    if normal_drift.abs() > 1e-9 {
        return Err(format!("excessive carrying-edge normal velocity drift {normal_drift}").into());
    }
    let load = FiniteBoxFlight {
        relative_velocity: pre.relative_velocity - normal_drift * site.normal,
        ..pre
    }
    .first_point_load(site, material, false)?
    .ok_or("no incoming edge reaction")?;
    let normal = v(&corner["normal"])?;
    let endpoints = [v(&corner["points_m"][0])?, v(&corner["points_m"][1])?];
    let loaded = corner["loaded_end"]
        .as_u64()
        .filter(|i| *i < 2)
        .ok_or("invalid corner endpoint")? as usize;
    let speeds = endpoints.map(|p| {
        normal.dot(pre.relative_velocity + pre.angular_velocity.cross(p - pre.center_of_mass))
    });
    let j_impact = v(&corner["j_ns"])?;
    let frame_j = v(&path["j"])? - v(&corner["arc"]["linear_impulse_ns"])? - j_impact;
    let mut integrated_j = v(&history["preceding_impulse_ns"])?;
    for event in &events[..events.len() - 1] {
        integrated_j += v(&event["j_ns"])? + v(&event["arc"]["linear_impulse_ns"])?;
    }
    let balance = (material.mass
        * (pre.relative_velocity - f.relative_velocity - elapsed * f.acceleration))
        .distance(integrated_j);
    let w_post = pre.angular_velocity
        + (endpoints[loaded] - pre.center_of_mass).cross(j_impact) / material.inertia;
    let v_post = pre.relative_velocity + j_impact / material.mass;
    let opening = normal.dot(v_post + w_post.cross(endpoints[1 - loaded] - pre.center_of_mass));
    if site.point.distance(endpoints[1 - loaded]) > 1e-8
        || speeds[loaded] >= 0.
        || normal.distance(DVec3::Z) > 1e-10
        || speeds[1 - loaded].abs() > 1e-5
        || load.force.dot(site.normal) <= 0.
        || opening <= 0.
        || (opening - number(&corner["opposite_speed_m_s"])?).abs() > 1e-10
        || balance > 1e-10
        || integrated_j.distance(frame_j) > 1e-10
    {
        let gap = site.point.distance(endpoints[1 - loaded]);
        return Err(format!(
            "corner state mismatch: speeds {speeds:?}, opposite gap {gap}, opening {opening}, impulse residual {balance}"
        ).into());
    }
    let into_foot = |p: DVec3| pre.moving.orientation.conjugate() * (p - pre.moving.center);
    Ok(json!({
        "mesh": mesh,
        "elapsed": elapsed,
        "com": pre.center_of_mass.to_array(),
        "foot": pre.moving.center.to_array(),
        "orientation": pre.moving.orientation.to_array(),
        "velocity": pre.relative_velocity.to_array(),
        "omega": pre.angular_velocity.to_array(),
        "j": integrated_j.to_array(),
        "line_points_m": corner["points_m"],
        "line_in_foot_m": endpoints.map(|p| into_foot(p).to_array()),
        "normal": corner["normal"],
        "corner_endpoint": loaded,
        "incoming_endpoint_normal_velocity_m_s": speeds,
        "resident_point_m": site.point.to_array(),
        "resident_normal": site.normal.to_array(),
        "resident_feature": resident["feature"],
        "resident_force_n": load.force.to_array(),
        "resident_normal_force_n": load.force.dot(site.normal),
        "resident_gap_m": arc["signed_gap_m"],
        "resident_point_velocity_m_s": port.to_array(),
        "resident_normal_speed_drift_m_s": normal_drift,
        "excluded_rigid_corner_impulse_ns": j_impact.to_array(),
        "rigid_post_opposite_opening_m_s": opening,
        "incoming_linear_momentum_residual_ns": balance,
    }))
}

pub(crate) fn write(dir: &Path) -> Result<()> {
    let case = survey::fixture(80);
    let cut = survey::cuts(case)?.0[1].ok_or("missing second landing")?;
    let (flight, _) = kinematics(case, &cut)?;
    let geom = geometry(flight)?;
    let a = incoming(flight, &geom, 2048)?;
    let b = incoming(flight, &geom, 8192)?;
    let dt = (number(&a["elapsed"])? - number(&b["elapsed"])?).abs();
    let delta = |key: &str| -> Result<f64> { Ok(v(&a[key])?.distance(v(&b[key])?)) };
    if dt > 1e-11
        || delta("com")? > 1e-10
        || delta("velocity")? > 1e-9
        || delta("omega")? > 1e-8
        || delta("resident_force_n")? > 1e-7
    {
        return Err(
            "incoming corner state did not converge between independent integrations".into(),
        );
    }
    fs::create_dir_all(dir)?;
    write_json(
        &dir.join("donor.json"),
        &json!({
            "case": 80, "stroke": 2,
            "coordinate_frame": "original support-translating frame",
            "public_restart_s": DT.as_secs_f64(),
            "drive": (command(case, cut.tick) * DVec3::Z).to_array(),
            "coarse": a, "fine": b,
            "convergence": {
                "clock_s": dt,
                "com_m": delta("com")?,
                "velocity_m_s": delta("velocity")?,
                "spin_rad_s": delta("omega")?,
                "resident_force_n": delta("resident_force_n")?
            }
        }),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_rocking_corner_arrives_while_the_opposite_end_still_carries_load() -> Result<()> {
        let out = tempfile::tempdir()?;
        write(out.path())?;
        let row: Value = serde_json::from_reader(fs::File::open(out.path().join("donor.json"))?)?;
        let corner = &row["fine"];
        let resumed = number(&corner["elapsed"])? - number(&row["public_restart_s"])?;
        assert!((resumed - 0.009242436127).abs() < 1e-11);
        let [edge, end]: [[f64; 3]; 2] = serde_json::from_value(corner["line_in_foot_m"].clone())?;
        for point in [edge, end] {
            assert!((point[1] - 0.10).abs() < 1e-11);
            assert!((point[2] + 0.06).abs() < 1e-11);
        }
        assert!(edge[0] > -0.18 && (end[0] - 0.18).abs() < 1e-11);
        assert!((10.89..10.90).contains(&number(&corner["resident_normal_force_n"])?));
        let speeds: [f64; 2] =
            serde_json::from_value(corner["incoming_endpoint_normal_velocity_m_s"].clone())?;
        assert!(speeds[0].abs() < 1e-10 && (-0.028..-0.027).contains(&speeds[1]));
        assert!((0.023..0.024).contains(&v(&corner["excluded_rigid_corner_impulse_ns"])?.z));
        assert!((0.002..0.0025).contains(&number(&corner["rigid_post_opposite_opening_m_s"])?));
        Ok(())
    }
}
