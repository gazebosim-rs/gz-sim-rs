// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay the same measured rigid jaw path from a complete, matched soft state.
use super::*;
use gz_physics_xpbd::{
    Boundary, ParticleBoundaryImpulse, PlaneImpulseProbe, RigidMobility, SoftBody,
    step_coupled_damped_pads,
};
use std::time::Duration;

#[path = "interval_view.rs"]
mod view;

fn paths(
    before: [BodyState; 2],
    after: [BodyState; 2],
    half: [DVec3; 2],
    dt: Duration,
    compliance: f64,
) -> ([Boundary; 3], [Option<RigidMobility>; 3]) {
    let jaws: [Boundary; 2] = std::array::from_fn(|i| Boundary::Cuboid {
        center: before[i].position,
        half_extents: half[i].to_array(),
        velocity: ((DVec3::from(after[i].position) - DVec3::from(before[i].position))
            / dt.as_secs_f64())
        .to_array(),
        angular_velocity: after[i].angular_velocity,
    });
    let mobility: [Option<RigidMobility>; 2] = std::array::from_fn(|i| {
        Some(RigidMobility::PrescribedCuboid {
            normal_compliance: compliance,
            start_orientation: before[i].orientation,
            end_orientation: after[i].orientation,
            start_angular_velocity: before[i].angular_velocity,
        })
    });
    (
        [
            jaws[0],
            jaws[1],
            Boundary::Plane {
                point: [0., 0., arm_sheet::BENCH],
                normal: [0., 0., 1.],
                velocity: [0.; 3],
            },
        ],
        [mobility[0], mobility[1], None],
    )
}

fn jaw_states(sim: &Simulation, ids: [u64; 2]) -> Result<[BodyState; 2]> {
    Ok([state(sim, ids[0])?, state(sim, ids[1])?])
}

fn table_readings(
    tick: usize,
    readings: &[ParticleBoundaryImpulse],
    table: DVec3,
    display_loads: &mut [f64],
) -> Result<Vec<serde_json::Value>> {
    let mut observed = DVec3::ZERO;
    let mut nodes = Vec::with_capacity(readings.len());
    for load in readings {
        let normal = -DVec3::from_array(load.normal);
        let tangent = -DVec3::from_array(load.tangent);
        observed += normal + tangent;
        display_loads[load.particle] += normal.z;
        nodes.push(json!({"node":load.particle,"normal_ns":normal.to_array(),"tangent_ns":tangent.to_array()}));
    }
    if !observed.is_finite()
        || !table.is_finite()
        || observed.distance(table) > 1e-13 * (1. + table.length())
    {
        return Err(
            format!("particle readings do not sum to the table impulse at tick {tick}").into(),
        );
    }
    Ok(nodes)
}

pub(super) fn run(
    out: &Path,
    xml: &str,
    damping: f64,
    [compliance, pad_viscosity]: [f64; 2],
    exchanges: usize,
    (first_tick, count): (usize, usize),
) -> Result<()> {
    let end = first_tick
        .checked_add(count.saturating_sub(1))
        .filter(|end| first_tick > 0 && count > 0 && *end <= 3400 * exchanges)
        .ok_or("interval is outside the 3.4 s grasp")?;
    let mut r = Recording::new(&view::world(xml), "sheet-interval.sdf", "default")?;
    let controller = arm::Controller::with_jaw_damping(&r.sim, damping)?;
    let ids = [
        entity_id(&r.sim, "default::arm::left")?,
        entity_id(&r.sim, "default::arm::right")?,
    ];
    let half = [
        arm_cable::half_size(&r.sdf, "arm", "left")?,
        arm_cable::half_size(&r.sdf, "arm", "right")?,
    ];
    let dt = r.sim.step_duration();
    let pad_settings = [(0, pad_viscosity), (1, pad_viscosity)];
    let viscosity: &[_] = if pad_viscosity > 0. {
        &pad_settings
    } else {
        &[]
    };
    let mut probe = PlaneImpulseProbe::default();
    let mut display_loads = [0.; arm_sheet::COUNT];
    let mut branches = Vec::<(u32, SoftBody)>::new();
    let mut samples = Vec::with_capacity(count);
    let stride = count.div_ceil(160);
    let mut frames = Vec::new();
    for tick in 1..=end {
        if (tick - 1).is_multiple_of(exchanges) {
            let actuator_tick = (tick - 1) / exchanges;
            arm_cable::lift_with_jaws(
                &controller,
                &mut r.sim,
                actuator_tick as f64 * 0.001,
                arm_sheet::JAW_STROKE,
            )?;
        }
        if tick < first_tick {
            r.sim.step_exact(1)?;
            continue;
        }
        if tick == first_tick {
            let seed = gz_sim_scene::deformable::material_snapshot(&r.sim, "sheet")
                .ok_or("missing matched material")?;
            let cfg = seed.config();
            for factor in [1, 2, 4, 8] {
                let steps = cfg
                    .substeps
                    .checked_mul(factor)
                    .ok_or("material resolution overflow")?;
                let mut b = seed.clone();
                b.set_resolution(steps, cfg.iterations)?;
                branches.push((steps, b));
            }
        }
        let pre = jaw_states(&r.sim, ids)?;
        r.sim.step_exact(1)?;
        let post = jaw_states(&r.sim, ids)?;
        let (geometry, mobility) = paths(pre, post, half, dt, compliance);
        let native = arm_sheet::cloth(&r.sim)?;
        let capture = (tick - first_tick).is_multiple_of(stride) || tick == end;
        let mut levels = Vec::new();
        let mut forces = [[0.; 2]; 4];
        for (index, (steps, branch)) in branches.iter_mut().enumerate() {
            if index == 0 {
                probe.step(branch, dt, &geometry, &mobility, viscosity)?;
            } else {
                step_coupled_damped_pads(&mut [branch], dt, &geometry, &mobility, viscosity)?;
            }
            if index == 0
                && (branch.particles() != native.particles()
                    || branch.last_boundary_impulses() != native.last_boundary_impulses())
            {
                return Err(format!("matched material failed to reproduce native particles or impulses at tick {tick}").into());
            }
            let mut normal = [0.; 2];
            for impulse in branch.last_pad_loads() {
                normal[impulse.boundary] += DVec3::from(impulse.normal).length() / dt.as_secs_f64();
            }
            forces[index] = normal;
            let peak = branch
                .particles()
                .iter()
                .zip(native.particles())
                .map(|(a, b)| DVec3::from(a.position).distance(b.position.into()))
                .fold(0., f64::max);
            let up: [f64; 2] = std::array::from_fn(|i| {
                -branch.last_boundary_impulses()[i].linear[2] / dt.as_secs_f64()
            });
            let particles = branch.particles();
            let (lowest_node, lowest) = particles
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.position[2].total_cmp(&b.position[2]))
                .ok_or("empty folded sheet")?;
            // Reactions are recorded on the boundary. Reverse their sign to
            // report the table's impulse on the sheet in world coordinates.
            let table = -DVec3::from(branch.last_boundary_impulses()[2].linear);
            let mut sample = json!({
                "substeps": steps, "normal_n": normal, "up_n": up, "max_pose_delta_m": peak,
                "table_impulse_ns": table.to_array(), "lowest_node": lowest_node,
                "min_table_gap_m": lowest.position[2] - arm_sheet::BENCH - arm_sheet::RADIUS,
            });
            if index == 0 {
                let nodes = table_readings(tick, probe.readings()?, table, &mut display_loads)?;
                if !nodes.is_empty() {
                    sample["table_particle_impulses"] = json!(nodes);
                }
            }
            if capture {
                sample["positions_m"] =
                    json!(particles.iter().map(|p| p.position).collect::<Vec<_>>());
                sample["velocities_m_s"] =
                    json!(particles.iter().map(|p| p.velocity).collect::<Vec<_>>());
            }
            levels.push(sample);
        }
        if capture {
            let time = native.sim_time().as_nanos();
            let positions = native
                .particles()
                .iter()
                .map(|p| p.position)
                .collect::<Vec<_>>();
            frames.push(json!({"time_ns":time,"positions":positions}));
            let strongest = display_loads
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.total_cmp(b.1))
                .and_then(|(i, up)| (*up > 0.).then_some(positions[i]));
            view::capture(&mut r, &forces, strongest)?;
            display_loads.fill(0.);
        }
        samples.push(json!({"end_tick":tick,"samples":levels}));
    }
    view::save(r, out, &frames)?;
    write_json(
        &out.join("interval.json"),
        &json!({"native_step_ns":dt.as_nanos(),"first_tick":first_tick,"native_matched_intervals":count,"actuator_period_ns":1_000_000,"intervals":samples}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_missing_sideways_impulse_cannot_pass_the_independent_total() {
        let normal = DVec3::Z * 0.0001;
        let bad = DVec3::splat(f64::NAN);
        for (observed, expected) in [
            (normal + DVec3::Y * 0.00001, normal),
            (bad, normal),
            (normal, bad),
        ] {
            let reading = ParticleBoundaryImpulse {
                boundary: 2,
                particle: 0,
                normal: (-observed).to_array(),
                tangent: [0.; 3],
            };
            assert!(table_readings(11, &[reading], expected, &mut [0.]).is_err());
        }
    }
    #[test]
    fn replay_matches_the_world_with_loaded_moving_jaws_and_checks_duration() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let xml = super::super::timed_scene(
            1.,
            0.001,
            Case {
                substeps: 1,
                ..Default::default()
            },
            1,
            true,
            0.001,
        )?;
        for t in [0, 3390, usize::MAX] {
            assert!(run(dir.path(), &xml, 12., [0.001, 0.], 1, (t, 16)).is_err());
        }
        run(dir.path(), &xml, 12., [0.001, 0.], 1, (1751, 16))?;
        let v: Value = serde_json::from_slice(&fs::read(dir.path().join("interval.json"))?)?;
        let steps = v["intervals"].as_array().unwrap();
        assert_eq!(steps.len(), 16);
        for step in steps {
            for (i, sample) in step["samples"].as_array().unwrap().iter().enumerate() {
                assert_eq!(sample["substeps"], 1 << i);
                assert_eq!(sample["table_impulse_ns"], json!([0., 0., 0.]));
                assert!(sample["min_table_gap_m"].as_f64().unwrap() > 0.1);
                let lowest = sample["lowest_node"].as_u64().unwrap() as usize;
                let nodes = sample["positions_m"].as_array().unwrap();
                assert_eq!(nodes.len(), arm_sheet::COUNT);
                assert_eq!(
                    sample["velocities_m_s"].as_array().unwrap().len(),
                    nodes.len()
                );
                let height = nodes[lowest][2].as_f64().unwrap();
                assert!(nodes.iter().all(|p| p[2].as_f64().unwrap() >= height));

                assert!(
                    sample["normal_n"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .all(|x| x.as_f64().unwrap() > 0.1)
                );
            }
            assert_eq!(step["samples"][0]["max_pose_delta_m"], 0.);
        }
        Ok(())
    }
}

#[cfg(test)]
mod viscous_tests {
    use super::*;
    #[test]
    fn the_native_and_replayed_pad_use_the_same_viscosity_and_source_clock() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let xml = super::super::damped_pads(
            super::super::timed_scene(
                1.,
                0.001,
                Case {
                    substeps: 1,
                    ..Default::default()
                },
                1,
                true,
                0.001,
            )?,
            0.2,
        )?;
        for bad in ["-1", "NaN", "1e1000", "bad"] {
            let wrong = xml.replace(
                "normal_damping=\"0.2\"",
                &format!("normal_damping=\"{bad}\""),
            );
            assert!(Recording::new(&wrong, "bad.sdf", "default").is_err());
        }
        let hard = xml.replace("normal_compliance=\"0.001\"", "normal_compliance=\"0\"");
        assert!(Recording::new(&hard, "hard.sdf", "default").is_err());
        assert!(super::super::damped_pads(xml.clone(), -1.).is_err());
        run(dir.path(), &xml, 12., [0.001, 0.2], 1, (1691, 16))?;
        let samples: Value = serde_json::from_slice(&fs::read(dir.path().join("interval.json"))?)?;
        assert_eq!(samples["native_matched_intervals"], 16);
        let frames: Value =
            serde_json::from_slice(&fs::read(dir.path().join("particle-frames.json"))?)?;
        assert_eq!(frames[0]["time_ns"], 1_691_000_000);
        assert_eq!(frames[15]["time_ns"], 1_706_000_000);
        Ok(())
    }
}

#[cfg(test)]
mod invalid_cut_tests {
    use super::*;
    #[test]
    fn unmatched_boundary_inputs_and_incomplete_scenes_refuse_the_cut() -> Result<()> {
        let xml = super::super::timed_scene(
            1.,
            0.001,
            Case {
                substeps: 1,
                ..Default::default()
            },
            1,
            true,
            0.,
        )?;
        let dir = tempfile::tempdir()?;
        let at = |s: &str, law| run(dir.path(), s, 12., law, 1, (1701, 1));
        assert!(
            at(&xml, [0.002, 0.])
                .unwrap_err()
                .to_string()
                .contains("failed to reproduce")
        );
        assert!(at("invalid XML", [0.001, 0.]).is_err());
        let renamed = xml.replace("close_left", "missing_drive");
        assert!(at(&renamed, [0.001, 0.]).is_err());
        let nomaterial = xml
            .replace("name=\"sheet\"", "name=\"absent\"")
            .replace("material=\"sheet\"", "material=\"absent\"");
        assert!(
            at(&nomaterial, [0.001, 0.])
                .unwrap_err()
                .to_string()
                .contains("missing matched material")
        );
        Ok(())
    }
}
