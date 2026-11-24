// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Last entirely separated public state of the second rising-deck landing.
use super::*;
use gz_physics::{FiniteBoxArrival, FiniteBoxFlight, FlightBox, SeparatingFeature};

fn vertices(f: FiniteBoxFlight, cut: &ReboundFrame, t: f64) -> Vec<DVec3> {
    let b = f.at(t);
    let s = BodyState {
        position: b.center.to_array(),
        orientation: b.orientation.to_array(),
        ..state(cut.foot)
    };
    rebound_gate::supported_vertices(s, state(cut.pad)).collect()
}

fn donor() -> Result<Value> {
    let fixture = survey::fixture(80);
    let cut = survey::cuts(fixture)?.0[1].ok_or("missing primary cut")?;
    let (f, rigid_velocity_defect) = kinematics(fixture, &cut)?;
    let restart = DT.as_secs_f64();
    let drive = command(fixture, cut.tick);
    let same_drive = (0..27).all(|i| command(fixture, cut.tick + i) == drive);
    let upper = FiniteBoxFlight {
        moving: FlightBox {
            center: cut.upper.position.into(),
            orientation: DQuat::from_array(cut.upper.orientation),
            half_extent: DVec3::new(0.05, 0.06, 0.06),
        },
        ..f
    };
    let deck_velocity = DVec3::from(cut.pad.velocity);
    if deck_velocity.distance(drive * DVec3::Z) > 1e-10
        || cut.normal_impulse_ns != 0.
        || cut.gap_m <= 0.
        || !same_drive
        || f.support.orientation.dot(DQuat::IDENTITY).abs() < 1. - 1e-14
        || rigid_velocity_defect > 1e-9
        || !matches!(upper.first_crossing(DT)?, FiniteBoxArrival::Separated)
    {
        return Err(format!("not a separated two-body landing on uniform deck motion, velocity defect {rigid_velocity_defect}").into());
    }
    let FiniteBoxArrival::Crossing(hit) = f.first_crossing(DT)? else {
        return Err("missing first collision".into());
    };
    let top = f.support.center.z + f.support.half_extent.z;
    let gap = |t: f64| -> Result<f64> {
        vertices(f, &cut, t)
            .iter()
            .map(|p| p.z - top)
            .reduce(f64::min)
            .ok_or_else(|| "empty foot projection".into())
    };
    let initial = gap(0.)?;
    let (mut lo, mut hi) = (hit.earliest.as_secs_f64(), hit.latest.as_secs_f64());
    if (initial - cut.gap_m).abs() > 1e-12
        || gap(lo)? <= 0.
        || gap(hi)? > 0.
        || hit.feature != SeparatingFeature::EdgePair(0, 1)
    {
        return Err("independent deck footprint disagrees with first box crossing".into());
    }
    for _ in 0..36 {
        let mid = 0.5 * (lo + hi);
        if gap(mid)? > 0. { lo = mid } else { hi = mid }
    }
    let b = f.at(hi);
    let mut corners = vertices(f, &cut, hi);
    corners.sort_by(|a, b| a.z.total_cmp(&b.z));
    let first = corners[0];
    let local = f.at(hi).orientation.conjugate() * (first - b.center);
    if corners.len() != 4
        || (local.y + f.moving.half_extent.y).abs() > 1e-10
        || (local.z + f.moving.half_extent.z).abs() > 1e-10
        || local.x.abs() >= f.moving.half_extent.x
        || (first.x - (f.support.center.x - f.support.half_extent.x)).abs() > 1e-10
        || corners[1].z - top < 1e-7
    {
        return Err("first site is not the sole's negative-Y edge alone".into());
    }
    let unclipped_bottom = |body: FlightBox| {
        (body.center.z
            - (0..3)
                .map(|i| {
                    (body.orientation * [DVec3::X, DVec3::Y, DVec3::Z][i])
                        .z
                        .abs()
                        * body.half_extent[i]
                })
                .sum::<f64>())
            - top
    };
    let upper_bound = unclipped_bottom(upper.moving)
        - restart
            * (f.relative_velocity.length()
                + restart * f.acceleration.length()
                + f.angular_velocity.length()
                    * (upper.moving.center.distance(f.center_of_mass)
                        + upper.moving.half_extent.length()));
    if upper_bound <= 0. {
        return Err("upper box could touch the deck during the primary arrival search".into());
    }
    let [moving_point, support_point] = hit.points.ok_or("missing supported OBB witness")?;
    let (time0, time1) = (hit.earliest.as_secs_f64(), hit.latest.as_secs_f64());
    let vertex_speed =
        f.relative_velocity + f.acceleration * hi + f.angular_velocity.cross(first - f.com_at(hi));
    Ok(
        json!({"case":80,"stroke":2,"coordinate_frame":"original support-translating frame",
      "public_restart_s":restart,"horizon_s":27.*restart,"drive":(DVec3::Z*drive).to_array(),
      "initial":{"elapsed":0.,"foot":f.moving.center.to_array(),"upper":upper.moving.center.to_array(),"com":f.center_of_mass.to_array(),
        "orientation":f.moving.orientation.to_array(),"velocity":f.relative_velocity.to_array(),"omega":f.angular_velocity.to_array(),"j":[0.,0.,0.]},
      "cut_tick":cut.tick,"cut_time_us":cut.time_us,"initial_foot_gap_m":initial,
      "initial_upper_gap_m":unclipped_bottom(upper.moving),"initial_normal_impulse_ns":cut.normal_impulse_ns,
      "upper_gap_lower_bound_until_primary_m":upper_bound,"initial_rigid_velocity_defect_m_s":rigid_velocity_defect,
      "drive_check":{"constant_from_tick":cut.tick,"constant_through_tick":cut.tick+26,"native_velocity_m_s":deck_velocity.to_array()},
      "deck":{"center_m":f.support.center.to_array(),"half_extents_m":f.support.half_extent.to_array(),"orientation_xyzw":f.support.orientation.to_array()},
      "foot_half_extents_m":f.moving.half_extent.to_array(),"upper_half_extents_m":upper.moving.half_extent.to_array(),
      "first_arrival":{"time_s":hi,"box_bracket_s":[time0,time1],"footprint_bracket_s":[lo,hi],
        "feature":"EdgePair(0, 1)","point_m":first.to_array(),"local_point_m":local.to_array(),
        "normal":hit.normal.to_array(),"incoming_normal_speed_m_s":hit.normal.dot(vertex_speed),
        "moving_witness_m":moving_point.to_array(),"support_witness_m":support_point.to_array(),
        "other_clipped_vertex_gaps_m":corners[1..].iter().map(|p|p.z-top).collect::<Vec<_>>()}}),
    )
}

pub(crate) fn write(dir: &Path) -> Result<()> {
    let data = donor()?;
    fs::create_dir_all(dir)?;
    write_json(&dir.join("donor.json"), &data)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn separated_fixture_reaches_only_negative_y_ridge_on_first_approach() -> Result<()> {
        let d = donor()?;
        let n = |key: &str| d[key].as_f64().unwrap();
        assert_eq!(d["initial_normal_impulse_ns"], 0.);
        assert!((0.00065..0.00068).contains(&n("initial_foot_gap_m")));
        assert!(n("upper_gap_lower_bound_until_primary_m") > 0.1);
        let at = d["first_arrival"]["time_s"].as_f64().unwrap();
        assert!((0.000499749..0.000499750).contains(&at));
        assert_eq!(d["drive"], json!([0., 0., 0.522]));
        Ok(())
    }
}
