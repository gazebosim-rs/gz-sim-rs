// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::deformable::bodies;

const WORLD: &str = include_str!("../../../../examples/worlds/local_material_incline.sdf");
const MATERIALS: [&str; 2] = ["gripping", "sliding"];

pub(super) fn run(out: &Path, substeps: u32, finite_board: bool) -> Result<()> {
    let world = WORLD.replace("substeps=\"1\"", &format!("substeps=\"{substeps}\""));
    let world = if finite_board {
        world.replace(
            r#"<plane point="0 0 0.32" normal="0.25 0 1"/>"#,
            r#"<box link="default::incline::surface" coupling="one_way"/>"#,
        )
    } else {
        world
    };
    record_world(out, &world, finite_board)
}

fn record_world(out: &Path, world: &str, finite_board: bool) -> Result<()> {
    let mut recording = Recording::new(world, "local_material_incline.sdf", "default")?;
    let downhill = DVec3::new(1., 0., -0.25).normalize();
    let normal = DVec3::new(0.25, 0., 1.).normalize();
    let step = recording.sim.step_duration();
    let substeps = MATERIALS.map(|name| {
        gz_sim_scene::deformable::material_snapshot(&recording.sim, name)
            .map(|strip| strip.config().substeps)
    });
    let mut frames = Vec::new();
    let mut trace = Vec::new();
    let mut final_distances = [0.; 2];
    let mut final_speeds = [0.; 2];
    let mut impulses = [DVec3::ZERO; 2];
    let end_tick = if finite_board { 120 } else { 80 };
    for pass in 0..2 {
        if pass == 1 {
            recording.sim.reset()?;
        }
        let mut origins = [DVec3::ZERO; 2];
        for tick in 0..=end_tick {
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
                if tick == 80 {
                    final_distances[i] = (center - origins[i]).dot(downhill);
                    final_speeds[i] = velocity.dot(downhill);
                }
                let impulse = strip
                    .last_boundary_impulses()
                    .first()
                    .map_or(DVec3::ZERO, |j| -DVec3::from(j.linear));
                if pass == 0 && tick <= 80 {
                    impulses[i] += impulse;
                }
                points.extend(p.iter().map(|n| n.position));
                state.push((p.to_vec(), impulse));
            }
            if pass == 0 {
                trace.push(state);
                frames.push(json!({"time_ns":tick as u128*step.as_nanos(),"positions":points}));
                recording.capture()?;
            } else if state != trace[tick] {
                return Err("reset changed an incline position, velocity or reaction".into());
            }
        }
    }
    let duration = step.as_secs_f64() * 80.;
    let expected = 9.81 * (-downhill.z - 0.1 * normal.z) * duration;
    if final_distances[0].abs() > 1e-9 || (final_speeds[1] - expected).abs() > 1e-9 {
        return Err(format!(
            "incline disagreement, held travel {} m and sliding speed {} m/s",
            final_distances[0], final_speeds[1]
        )
        .into());
    }
    let departure = if finite_board {
        let last = trace
            .iter()
            .rposition(|state| state[1].1 != DVec3::ZERO)
            .ok_or("board carried no load")?;
        let expected_change = DVec3::new(0., 0., -9.81) * step.as_secs_f64();
        let velocity = |state: &[(Vec<gz_physics_xpbd::Particle>, DVec3)]| {
            let nodes = &state[1].0;
            (DVec3::from(nodes[0].velocity) + DVec3::from(nodes[1].velocity)) * 0.5
        };
        let max_error = trace[last..]
            .windows(2)
            .map(|states| (velocity(&states[1]) - velocity(&states[0]) - expected_change).length())
            .fold(0., f64::max);
        let free = end_tick - last;
        if last <= 80 || free < 6 || max_error > 1e-9 {
            return Err("finite board did not release the sliding strip into free fall".into());
        }
        Some(json!({"last_loaded_tick":last,"unsupported_ticks":free,
            "max_free_velocity_change_error_m_s":max_error}))
    } else {
        None
    };
    recording.save(out)?;
    write_json(&out.join("particle-frames.json"), &json!(frames))?;
    let names: Vec<_> = MATERIALS
        .iter()
        .flat_map(|n| (0..2).map(move |i| format!("default::incline::surface::{n}_{i}")))
        .collect();
    write_json(
        &out.join("frames.json"),
        &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"particle_visuals":names,"captures":{"initial":0,"moving":30,"middle":50,"end":end_tick}}),
    )?;
    write_json(
        &out.join("incline.json"),
        &json!({"slope":0.25,"on_board_measurement_ticks":80,"departure":departure, "static_plane_friction":[0.55,0.1],"sliding_friction":0.1, "step_ns":step.as_nanos(),"material_substeps":substeps,"sliding_analytic_distance_m":0.5*expected*duration, "reset_states":trace.len(),"distance_m":final_distances,"speed_m_s":final_speeds,"sliding_analytic_speed_m_s":expected,"support_impulse_ns":impulses.map(|j| j.to_array())}),
    )?;
    fs::write(out.join("world.sdf"), world)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_sliding_control_and_incomplete_archives_cannot_report_a_qualified_hold() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let sliding = WORLD.replace(" static_plane_friction=\"0.55\"", "");
        let infinite = record_world(dir.path(), WORLD, true).unwrap_err();
        assert!(infinite.to_string().contains("free fall"));
        let e = record_world(dir.path(), &sliding, false).unwrap_err();
        assert!(e.to_string().contains("incline disagreement"));
        assert!(record_world(dir.path(), "<", false).is_err());
        let renamed = WORLD.replace("gripping", "other_strip");
        assert!(
            record_world(dir.path(), &renamed, false)
                .unwrap_err()
                .to_string()
                .contains("missing material")
        );
        for file in [
            "replay",
            "particle-frames.json",
            "frames.json",
            "incline.json",
            "world.sdf",
        ] {
            let dir = tempfile::tempdir()?;
            let path = dir.path().join(file);
            if file == "replay" {
                fs::write(path, "occupied")?;
            } else {
                fs::create_dir(path)?;
            }
            assert!(run(dir.path(), 1, false).is_err());
        }
        Ok(())
    }

    #[test]
    fn the_authored_incline_sticks_slides_and_resets() -> Result<()> {
        let dir = tempfile::tempdir()?;
        run(dir.path(), 1, false)?;
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

    #[test]
    fn sliding_distance_converges_at_fixed_public_clock() -> Result<()> {
        let dir = tempfile::tempdir()?;
        assert!(run(dir.path(), 0, false).is_err());
        for n in [1, 2, 4, 8] {
            let output = dir.path().join(n.to_string());
            run(&output, n, false)?;
            let r: Value = serde_json::from_slice(&fs::read(output.join("incline.json"))?)?;
            assert_eq!(r["step_ns"], 10_000_000);
            assert_eq!(r["material_substeps"], json!([n, n]));
            assert_eq!(r["reset_states"], 81);
            let distance = r["distance_m"][1].as_f64().unwrap();
            let reference = r["sliding_analytic_distance_m"].as_f64().unwrap();
            let speed = r["sliding_analytic_speed_m_s"].as_f64().unwrap();
            let kinetic = 0.1 * 9.81 / 1.0625_f64.sqrt();
            let euler_error = (0.5 * speed + 0.8 * kinetic) * 0.01 / f64::from(n);
            assert!((distance - reference - euler_error).abs() < 1e-9);
        }
        Ok(())
    }
    #[test]
    fn finite_board_carries_the_same_two_strips_as_the_analytic_plane() -> Result<()> {
        let dir = tempfile::tempdir()?;
        for n in [1, 8] {
            let output = dir.path().join(n.to_string());
            run(&output, n, true)?;
            let report: Value = serde_json::from_slice(&fs::read(output.join("incline.json"))?)?;
            let expected = report["sliding_analytic_distance_m"].as_f64().unwrap()
                + (report["sliding_analytic_speed_m_s"].as_f64().unwrap() * 0.005
                    + 0.008 * 0.1 * 9.81 / 1.0625_f64.sqrt()) / f64::from(n);
            assert!((report["distance_m"][1].as_f64().unwrap() - expected).abs() < 1e-9);
            assert_eq!(report["reset_states"], 121);
            assert!(report["departure"]["unsupported_ticks"].as_u64().unwrap() >= 6);
        }
        Ok(())
    }
}
