// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::deformable::bodies;

const WORLD: &str = include_str!("../../../../examples/worlds/local_material_incline.sdf");
const MATERIALS: [&str; 2] = ["gripping", "sliding"];

pub(super) fn run(out: &Path) -> Result<()> {
    let mut recording = Recording::new(WORLD, "local_material_incline.sdf", "default")?;
    let downhill = DVec3::new(1., 0., -0.25).normalize();
    let normal = DVec3::new(0.25, 0., 1.).normalize();
    let mut frames = Vec::new();
    let mut trace = Vec::new();
    let mut final_distances = [0.; 2];
    let mut final_speeds = [0.; 2];
    let mut impulses = [DVec3::ZERO; 2];
    for pass in 0..2 {
        if pass == 1 {
            recording.sim.reset()?;
        }
        let mut origins = [DVec3::ZERO; 2];
        for tick in 0..=80 {
            if tick > 0 {
                if pass == 0 {
                    recording.advance()?;
                } else {
                    recording.sim.step_exact(1)?;
                }
            }
            let materials: std::collections::BTreeMap<_, _> = bodies(&recording.sim).collect();
            let mut points = Vec::new();
            let mut state = Vec::new();
            for (i, name) in MATERIALS.iter().enumerate() {
                let strip = materials
                    .get(name)
                    .ok_or("missing material on the incline")?;
                let p = strip.particles();
                let center = (DVec3::from(p[0].position) + DVec3::from(p[1].position)) / 2.;
                let velocity = (DVec3::from(p[0].velocity) + DVec3::from(p[1].velocity)) / 2.;
                if tick == 0 {
                    origins[i] = center;
                }
                final_distances[i] = (center - origins[i]).dot(downhill);
                final_speeds[i] = velocity.dot(downhill);
                let impulse = strip
                    .last_boundary_impulses()
                    .first()
                    .map_or(DVec3::ZERO, |j| -DVec3::from(j.linear));
                if pass == 0 {
                    impulses[i] += impulse;
                }
                points.extend(p.iter().map(|n| n.position));
                state.push((p.to_vec(), impulse));
            }
            if pass == 0 {
                trace.push(state);
                frames.push(json!({"time_ns":tick*10_000_000,"positions":points}));
                recording.capture()?;
            } else if state != trace[tick] {
                return Err("reset changed an incline position, velocity or reaction".into());
            }
        }
    }
    let expected = 9.81 * (-downhill.z - 0.1 * normal.z) * 0.8;
    if final_distances[0].abs() > 1e-9 || (final_speeds[1] - expected).abs() > 1e-9 {
        return Err(format!(
            "incline disagreement, held travel {} m and sliding speed {} m/s",
            final_distances[0], final_speeds[1]
        )
        .into());
    }
    recording.save(out)?;
    write_json(&out.join("particle-frames.json"), &json!(frames))?;
    let names: Vec<_> = MATERIALS
        .iter()
        .flat_map(|n| (0..2).map(move |i| format!("default::incline::surface::{n}_{i}")))
        .collect();
    write_json(
        &out.join("frames.json"),
        &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"particle_visuals":names,"captures":{"initial":0,"moving":30,"middle":50,"end":80}}),
    )?;
    write_json(
        &out.join("incline.json"),
        &json!({"slope":0.25, "static_plane_friction":[0.55,0.1],"sliding_friction":0.1, "step_ns":10_000_000, "reset_states":trace.len(),"distance_m":final_distances,"speed_m_s":final_speeds,"sliding_analytic_speed_m_s":expected,"support_impulse_ns":impulses.map(|j| j.to_array())}),
    )?;
    fs::write(out.join("world.sdf"), WORLD)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn the_authored_incline_sticks_slides_and_resets() -> Result<()> {
        let dir = tempfile::tempdir()?;
        run(dir.path())?;
        let r: Value = serde_json::from_slice(&fs::read(dir.path().join("incline.json"))?)?;
        assert_eq!(r["reset_states"], 81);
        for coefficient in ["-0.2", "0.05", "bad"] {
            let invalid = WORLD.replace(
                "static_plane_friction=\"0.55\"",
                &format!("static_plane_friction=\"{coefficient}\""),
            );
            assert!(Recording::new(&invalid, "invalid.sdf", "default").is_err());
        }
        assert!(r["distance_m"][1].as_f64().unwrap() > 0.44);
        Ok(())
    }
}
