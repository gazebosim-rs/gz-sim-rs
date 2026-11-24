// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay the same measured rigid jaw path from a complete, matched soft state.
use super::*;
use gz_physics_xpbd::{Boundary, RigidMobility, SoftBody, step_coupled_rigid};
use std::time::Duration;

const LENGTH: usize = 16;

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

pub(super) fn run(
    out: &Path,
    xml: &str,
    damping: f64,
    compliance: f64,
    exchanges: usize,
    first_tick: usize,
) -> Result<()> {
    let end = first_tick
        .checked_add(LENGTH - 1)
        .filter(|end| first_tick > 0 && *end <= 3400 * exchanges)
        .ok_or("interval is outside the 3.4 s grasp")?;
    let mut r = Recording::new(xml, "sheet-interval.sdf", "default")?;
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
    let mut branches = Vec::<(u32, SoftBody)>::new();
    let mut samples = Vec::with_capacity(LENGTH);
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
        let mut levels = Vec::new();
        for (index, (steps, branch)) in branches.iter_mut().enumerate() {
            step_coupled_rigid(&mut [branch], dt, &geometry, &mobility)?;
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
            let peak = branch
                .particles()
                .iter()
                .zip(native.particles())
                .map(|(a, b)| DVec3::from(a.position).distance(b.position.into()))
                .fold(0., f64::max);
            let up: [f64; 2] = std::array::from_fn(|i| {
                -branch.last_boundary_impulses()[i].linear[2] / dt.as_secs_f64()
            });
            levels.push(
                json!({"substeps":steps,"normal_n":normal,"up_n":up,"max_pose_delta_m":peak}),
            );
        }
        samples.push(json!({"end_tick":tick,"samples":levels}));
    }
    write_json(
        &out.join("interval.json"),
        &json!({"native_step_ns":dt.as_nanos(),"first_tick":first_tick,"native_matched_intervals":LENGTH,"actuator_period_ns":1_000_000,"intervals":samples}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

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
            assert!(run(dir.path(), &xml, 12., 0.001, 1, t).is_err());
        }
        run(dir.path(), &xml, 12., 0.001, 1, 1751)?;
        let v: Value = serde_json::from_slice(&fs::read(dir.path().join("interval.json"))?)?;
        let steps = v["intervals"].as_array().unwrap();
        assert_eq!(steps.len(), LENGTH);
        for step in steps {
            for (i, sample) in step["samples"].as_array().unwrap().iter().enumerate() {
                assert_eq!(sample["substeps"], 1 << i);
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
