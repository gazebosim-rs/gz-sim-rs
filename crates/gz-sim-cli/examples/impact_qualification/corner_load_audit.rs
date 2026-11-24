// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check moving contact geometry and resultant normal loads directly from the saved poses.
use super::{transfer::vector, *};
use glam::{DQuat, DVec3};
use gz_physics::ElasticLineFoundation;

struct Fixture {
    half: DVec3,
    deck: DVec3,
    deck_half: DVec3,
}

// Reconstruct the clipped material edge from the fixture and pose alone.
// Fixture 80 lands on the +Y, −Z longitudinal edge.
impl Fixture {
    fn edge(&self, frame: &Value) -> Result<([DVec3; 2], [f64; 2])> {
        let q = DQuat::from_array(serde_json::from_value(frame["orientation_xyzw"].clone())?);
        let foot = vector(&frame["foot_m"])?;
        if !q.is_normalized() {
            return Err("invalid foot pose".into());
        }
        let a = foot + q * DVec3::new(-self.half.x, self.half.y, -self.half.z);
        let ab = q * DVec3::X * (2. * self.half.x);
        let mut entry: f64 = 0.;
        let mut exit: f64 = 1.;
        for axis in 0..2 {
            let offset = self.deck[axis] - a[axis];
            if ab[axis].abs() <= f64::EPSILON {
                if offset.abs() > self.deck_half[axis] {
                    return Err("detached edge".into());
                }
            } else {
                let low = (offset - self.deck_half[axis]) / ab[axis];
                let high = (offset + self.deck_half[axis]) / ab[axis];
                entry = entry.max(low.min(high));
                exit = exit.min(low.max(high));
            }
        }
        if entry >= exit {
            return Err("empty projected edge".into());
        }
        Ok(([a + entry * ab, a + exit * ab], [entry, exit]))
    }
}

#[derive(Default)]
struct Resultant {
    force: f64,
    first_moment: f64, // Pressure weighted with fractional distance on the clipped segment.
    energy: f64,
}

fn integrate(
    length: f64,
    overlap: [f64; 2],
    material_u: [f64; 2],
    foot: [f64; 2],
    deck: f64,
) -> Result<Resultant> {
    if foot[0] == foot[1] {
        let r = ElasticLineFoundation::new(foot[0], deck)
            .and_then(|f| f.integrate_segment(length, overlap))
            .ok_or("invalid uniform foundation")?;
        return Ok(Resultant {
            force: r.normal_force_n,
            first_moment: r.normal_force_n * r.center_of_pressure_fraction.unwrap_or_default(),
            energy: r.energy_j,
        });
    }
    // For graded material, integrate independently with eight-point Gauss–Legendre
    // quadrature over the strictly positive region. The simulator uses midpoint cells.
    const NODES: [(f64, f64); 4] = [
        (0.1834346424956498, 0.362683783378362),
        (0.525532409916329, 0.3137066458778873),
        (0.7966664774136267, 0.2223810344533745),
        (0.9602898564975363, 0.1012285362903763),
    ];
    let [a, b] = overlap;
    let mut r = Resultant::default();
    if a.max(b) <= 0. {
        return Ok(r);
    }
    let lo = if a < 0. { a / (a - b) } else { 0. };
    let hi = if b < 0. { a / (a - b) } else { 1. };
    let mid = (lo + hi) * 0.5;
    let radius = (hi - lo) * 0.5;
    for (node, weight) in NODES {
        for sign in [-1., 1.] {
            let t = mid + sign * radius * node;
            let u = material_u[0] + (material_u[1] - material_u[0]) * t;
            let k = foot[0] + (foot[1] - foot[0]) * u;
            let sample = ElasticLineFoundation::new(k, deck)
                .and_then(|f| f.sample(a + (b - a) * t))
                .ok_or("invalid material")?;
            let dl = length * radius * weight;
            r.force += sample.line_load_n_m * dl;
            r.first_moment += t * sample.line_load_n_m * dl;
            r.energy += sample.energy_j_m * dl;
        }
    }
    Ok(r)
}

fn check_profile(fixture: &Fixture, p: &Value, material_profile: &Value) -> Result<Value> {
    let top = fixture.deck.z + fixture.deck_half.z;
    let foot: [f64; 2] =
        serde_json::from_value(material_profile["foot_endpoint_stiffness_n_m2"].clone())?;
    let deck = num(&material_profile["deck_stiffness_n_m2"])?;
    let frames = array(&p["frames"])?;
    let f0 = frames.first().ok_or("no incoming sample")?;
    let v0 = vector(&f0["v_m_s"])?;
    let t0 = num(&f0["original_time_s"])?;
    let mut rows = vec![];
    for frame in frames {
        let (ends, material) = fixture.edge(frame)?;
        let overlap = ends.map(|p| top - p.z);
        let geom_error = ends
            .iter()
            .enumerate()
            .map(|(i, p)| Ok(p.distance(vector(&frame["edge_endpoints_m"][i])?)))
            .collect::<Result<Vec<f64>>>()?
            .into_iter()
            .fold(0., f64::max);
        let r = integrate(ends[0].distance(ends[1]), overlap, material, foot, deck)?;
        let force = num(&frame["normal_force_n"])?;
        let moment = if force > 0. {
            force * num(&frame["center_of_pressure_fraction"])?
        } else {
            0.
        };
        let at = num(&frame["original_time_s"])?;
        let impulse = 2. * (vector(&frame["v_m_s"])? - v0 + 9.81 * (at - t0) * DVec3::Z);
        let momentum_error = impulse.distance(vector(&frame["support_frame_j_ns"])?);
        let force_error = (r.force - force).abs();
        let energy_error = (r.energy - num(&frame["strain_energy_j"])?).abs();
        let moment_error = (r.first_moment - moment).abs();
        if geom_error > 1e-10
            || force_error > 0.002
            || energy_error > 1e-8
            || moment_error > 0.002
            || momentum_error > 1e-9
        {
            return Err(format!("{} at {at} s: geometry {geom_error} m, force {force_error} N, first moment {moment_error} N, energy {energy_error} J and momentum {momentum_error} N s",p["name"]).into());
        }
        rows.push(json!({"original_time_s":at,"normal_force_n":r.force,"geometry_difference_m":geom_error,
                "force_difference_n":force_error,"fractional_first_moment_difference_n":moment_error,
                "stored_energy_difference_j":energy_error,"momentum_residual_ns":momentum_error}));
    }
    Ok(json!({"name":p["name"],"frames":rows}))
}

// At a common clock, gravity and the incoming total momentum cancel between
// branches. This also checks the rigid reaction used to connect different starts.
fn check_differences(early: &Value, late: &Value, recorded: &Value) -> Result<Value> {
    let late_frames = array(&late["frames"])?;
    let rows = array(recorded)?;
    if rows.len() != late_frames.len() || rows.is_empty() {
        return Err("missing paired clocks".into());
    }
    let mut max_j = 0_f64;
    let mut max_com = 0_f64;
    for (row, b) in rows.iter().zip(late_frames) {
        let t = num(&b["original_time_s"])?;
        let a = array(&early["frames"])?
            .iter()
            .find(|r| {
                r["original_time_s"]
                    .as_f64()
                    .is_some_and(|x| (t - x).abs() < 1e-13)
            })
            .ok_or("missing paired clock")?;
        if (t - num(&row["original_time_s"])?).abs() > 1e-13 {
            return Err("unmatched clock in measured difference".into());
        }
        let dj = 2. * (vector(&a["v_m_s"])? - vector(&b["v_m_s"])?);
        let dc = vector(&a["com_m"])? - vector(&b["com_m"])?;
        max_j = max_j.max(dj.distance(vector(&row["contact_impulse_shift_ns"])?));
        max_com = max_com.max(dc.distance(vector(&row["com_shift_m"])?));
    }
    if max_j > 1e-9 || max_com > 1e-12 {
        return Err("paired impulse or position differs from independent state readback".into());
    }
    Ok(
        json!({"name":early["name"],"common_clocks":rows.len(),"impulse_residual_ns":max_j,"com_residual_m":max_com}),
    )
}

pub(super) fn run(rigid: &Path, late_file: &Path, directory: &Path) -> Result<()> {
    let source = read(rigid)?;
    let study = read(&directory.join("reference.json"))?;
    if [source["case"].as_u64(), study["case"].as_u64()] != [Some(80); 2]
        || [source["stroke"].as_u64(), study["stroke"].as_u64()] != [Some(2); 2]
    {
        return Err("requires fixture 80, stroke 2".into());
    }
    let restart = num(&source["references"][0]["fine"]["elapsed"])?;
    let fixture = Fixture {
        half: vector(&source["geometry"]["foot_half_extents"])?,
        deck: vector(&source["frames"][0]["pad"]["position"])?
            - restart * vector(&source["drive"])?,
        deck_half: vector(&source["geometry"]["pad_half_extents"])?,
    };
    let late = read(late_file)?;
    let mut result = vec![];
    let mut pairs = vec![];
    let profiles = array(&study["profiles"])?;
    for p in profiles {
        result.push(check_profile(&fixture, p, p)?);
        let matched = array(&late["profiles"])?
            .iter()
            .find(|r| r["name"] == p["name"])
            .ok_or("unmatched material")?;
        pairs.push(check_differences(
            p,
            matched,
            &p["comparison_to_late_start"],
        )?);
    }
    let uniform = profiles
        .iter()
        .find(|r| r["name"] == "Uniform layers")
        .ok_or("missing uniform material")?;
    let earlier = &study["earlier_start_control"];
    let earlier_checks = check_profile(&fixture, earlier, uniform)?;
    pairs.push(check_differences(
        earlier,
        uniform,
        &earlier["comparison_to_corner_start"],
    )?);
    if result.len() != 3 {
        return Err("incomplete material sweep".into());
    }
    write(
        &directory.join("normal-load-audit.json"),
        &json!({"case":80,"stroke":2,"profiles":result,"earlier_control":earlier_checks,"matched_clock_comparisons":pairs}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compares_force_histories_from_a_common_incoming_momentum() {
        let sample = |t, x, v| json!({"original_time_s":t,"com_m":[x,0.,0.],"v_m_s":[v,0.,0.]});
        let early = json!({"frames":[sample(0.,0.,0.),sample(0.5,1.,0.75)]});
        let late = json!({"frames":[sample(0.5,1.25,0.25)]});
        let mut diff = json!([{"original_time_s":0.5, "com_shift_m":[-0.25,0.,0.],
            "contact_impulse_shift_ns":[1.,0.,0.]}]);
        assert!(check_differences(&early, &late, &diff).is_ok());
        diff[0]["contact_impulse_shift_ns"] = json!([0.75, 0., 0.]);
        assert!(check_differences(&early, &late, &diff).is_err());
    }

    #[test]
    fn partially_loaded_graded_ridge_recovers_the_uniform_limit() {
        let f = |layers| integrate(0.4, [-3e-4, 1e-4], [0.1, 1.], layers, 8e6).unwrap();
        let uniform = f([2e7, 2e7]);
        let nearly_uniform = f([2e7, 2e7 + 1e-6]);
        assert!((uniform.force - nearly_uniform.force).abs() < 1e-10);
        assert!((uniform.energy - nearly_uniform.energy).abs() < 1e-14);
        assert!((uniform.first_moment - nearly_uniform.first_moment).abs() < 1e-10);
    }
}
