// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Accelerate a fixed-pivot wrist about an oblique axis, then brake it to rest.
use super::*;
use glam::DMat3;
use gz_sim_scene::{BallJointSample, locked_subtree_inertia};
use serde::Serialize;

const HALF: usize = 400;
const END: usize = 1000;
const DT: f64 = 0.001;

#[derive(Clone, Debug, PartialEq, Serialize)]
struct Sample {
    actual_xyzw: [f64; 4],
    velocity_rad_s: [f64; 3],
    torque_nm: [f64; 3],
    angle_error_rad: f64,
    velocity_error_rad_s: f64,
    inertia_error_kg_m2: f64,
    center_error_m: f64,
}

fn trajectory(tick: usize) -> (f64, f64, f64) {
    let half = HALF as f64 * DT;
    let time = tick as f64 * DT;
    let acceleration = 6.;
    if tick < HALF {
        (
            0.5 * acceleration * time * time,
            acceleration * time,
            acceleration,
        )
    } else if tick < 2 * HALF {
        let remaining = 2. * half - time;
        (
            acceleration * (half * half - 0.5 * remaining * remaining),
            acceleration * remaining,
            -acceleration,
        )
    } else {
        (acceleration * half * half, 0., 0.)
    }
}

fn reference(wrist: &Wrist, sim: &Simulation) -> (DMat3, DVec3) {
    let (pivot, carrier) = pose(sim, wrist.carrier);
    let axes = carrier * rotation([0.31, -0.42, 0.63]);
    let mut inertia = DMat3::ZERO;
    let mut first = DVec3::ZERO;
    for (entity, m, offset, diagonal) in [
        (wrist.tool, 2., DVec3::ZERO, DVec3::new(0.006, 0.008, 0.004)),
        (
            wrist.payload,
            1.,
            DVec3::new(0.016, -0.012, 0.005),
            DVec3::new(0.001, 0.0015, 0.0015),
        ),
    ] {
        let (link, q) = pose(sim, entity);
        let arm = axes.inverse() * (link + q * offset - pivot);
        let rotation = DMat3::from_quat(axes.inverse() * q);
        let outer = DMat3::from_cols(arm.x * arm, arm.y * arm, arm.z * arm);
        inertia += rotation * DMat3::from_diagonal(diagonal) * rotation.transpose()
            + m * (DMat3::from_diagonal(DVec3::splat(arm.length_squared())) - outer);
        first += m * arm;
    }
    (inertia, first / 3.)
}

fn episode(
    sim: &mut Simulation,
    mut capture: impl FnMut(usize, &Simulation) -> Result<()>,
) -> Result<Vec<[Sample; 2]>> {
    let wrists = ["held", "free"].map(|m| Wrist::new(sim, m));
    let entities = ["held", "free"].map(|m| named(sim, &format!("default::{m}::bearing")));
    let axis = DVec3::new(0.4, -0.18, 0.9).normalize();
    let mut trace = Vec::with_capacity(END + 1);
    for tick in 0..=END {
        if tick % 10 == 0 {
            capture(tick, sim)?;
        }
        let (angle, rate, acceleration) = trajectory(tick);
        let sample = |i: usize| -> Result<Sample> {
            let wrist = &wrists[i];
            let native = locked_subtree_inertia(sim, wrist.bearing)?;
            let real = DMat3::from_cols_array_2d(&native.inertia_kg_m2);
            let (independent, center) = reference(wrist, sim);
            let error = (real - independent)
                .to_cols_array()
                .into_iter()
                .map(f64::abs)
                .fold(0., f64::max);
            let center_error = center.distance(native.center_of_mass_m.into());
            if (native.mass_kg - 3.).abs() > 1e-12 || error > 1e-12 || center_error > 1e-12 {
                return Err(format!("downstream mass, tensor or center mismatch at tick {tick}: {native:?}, {error}, {center_error}").into());
            }
            let encoder = sim
                .world()
                .get::<BallJointSample>(entities[i])
                .ok_or("missing spherical encoder")?
                .state;
            let omega = DVec3::from(encoder.angular_velocity);
            let tensor = if i == 0 {
                real
            } else {
                DMat3::from_diagonal(DVec3::new(real.x_axis.x, real.y_axis.y, real.z_axis.z))
            };
            let gravity = DVec3::from(joint_gravity_load(sim, wrist.bearing)?.torque_nm);
            let torque = tensor * (axis * acceleration) + omega.cross(tensor * omega) - gravity;
            let residual =
                DQuat::from_array(encoder.orientation) * DQuat::from_axis_angle(axis, -angle);
            Ok(Sample {
                actual_xyzw: encoder.orientation,
                velocity_rad_s: encoder.angular_velocity,
                torque_nm: torque.to_array(),
                angle_error_rad: 2. * residual.xyz().length().atan2(residual.w.abs()),
                velocity_error_rad_s: omega.distance(rate * axis),
                inertia_error_kg_m2: error,
                center_error_m: center_error,
            })
        };
        let pair = [sample(0)?, sample(1)?];
        if tick < END {
            for i in 0..2 {
                queue_ball_joint_effort(sim, wrists[i].bearing, pair[i].torque_nm)?;
            }
            sim.step_exact(1)?;
        }
        trace.push(pair);
    }
    Ok(trace)
}

pub(super) fn record(out: &Path) -> Result<()> {
    if out.exists() {
        return Err("choose a new inertia recording directory".into());
    }
    let mut sim = load_world_with_sdf_step(
        XML,
        "gravity_wrist.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let sdf = sdformat::load_str(XML, "gravity_wrist.sdf", &sdformat::NoResources)?;
    let mut frames = Vec::new();
    let first = episode(&mut sim, |_, s| {
        frames.push(gz_sim_cli::primitive_frame(
            s,
            &sdf,
            "default",
            frames.len() as u64 + 1,
        )?);
        Ok(())
    })?;
    reset_loaded_world(&mut sim, |_| ())?;
    let second = episode(&mut sim, |_, _| Ok(()))?;
    if first != second {
        return Err("reset changed a loaded rotation or measured inertia".into());
    }
    let peak = |i: usize| {
        first
            .iter()
            .map(|s| s[i].angle_error_rad)
            .fold(0., f64::max)
    };
    let max_tensor = first
        .iter()
        .flatten()
        .map(|p| p.inertia_error_kg_m2)
        .fold(0., f64::max);
    let result = json!({"ticks":END,"dt_s":DT,"target_angle_rad":trajectory(END).0,"full_peak_angle_error_rad":peak(0),"diagonal_peak_angle_error_rad":peak(1),"max_tensor_error_kg_m2":max_tensor,"last":first[END],"reset_exact":true});
    fs::create_dir_all(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    for (name, value) in [
        ("comparison.json", result.clone()),
        ("trace.json", serde_json::to_value(first)?),
        (
            "frames.json",
            json!({"kind":"authored-archive-v1","dt_ns":1,"camera":"right","captures":{"accelerating":20,"peak_speed":40,"rest":80,"end":100}}),
        ),
    ] {
        fs::write(out.join(name), serde_json::to_vec_pretty(&value)?)?;
    }
    println!("{}", serde_json::to_string_pretty(&result)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_inertia_tracks_the_oblique_rotation_after_reset() -> Result<()> {
        let tmp = tempfile::tempdir()?;
        let out = tmp.path().join("wrist");
        record(&out)?;
        assert!(record(&out).is_err());
        let comparison: serde_json::Value =
            serde_json::from_slice(&fs::read(out.join("comparison.json"))?)?;
        assert!(comparison["full_peak_angle_error_rad"].as_f64().unwrap() < 0.015);
        assert!(
            comparison["diagonal_peak_angle_error_rad"]
                .as_f64()
                .unwrap()
                > 1.
        );
        assert_eq!(comparison["reset_exact"], true);
        Ok(())
    }

    #[test]
    fn interrupted_capture_or_invalid_bearing_cannot_qualify_a_loaded_wrist() -> Result<()> {
        let mut sim = load_world_with_sdf_step(
            XML,
            "gravity_wrist.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        assert!(episode(&mut sim, |_, _| Err("interrupted".into())).is_err());
        let sensor = named(&sim, "default::held::bearing");
        sim.world_mut()
            .entity_mut(sensor)
            .remove::<BallJointSample>();
        assert!(episode(&mut sim, |_, _| Ok(())).is_err());
        Ok(())
    }
    #[test]
    fn a_broken_clamp_or_solver_fault_cannot_enter_the_recording() -> Result<()> {
        for part in ["clamp", "bearing", "fault"] {
            let mut sim = load_world_with_sdf_step(
                XML,
                "gravity_wrist.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )?;
            if part == "fault" {
                sim.world()
                    .resource::<gz_sim_core::StepFault>()
                    .report(std::io::Error::other("solver unavailable"));
            } else {
                let entity = named(&sim, &format!("default::held::{part}"));
                let handle = sim
                    .world()
                    .get::<gz_sim_scene::JointHandle>(entity)
                    .unwrap()
                    .id();
                assert!(
                    sim.world_mut()
                        .resource_mut::<gz_physics::PhysicsBackend>()
                        .remove_joint(handle)
                );
            }
            assert!(episode(&mut sim, |_, _| Ok(())).is_err());
        }
        let dir = tempfile::tempdir()?;
        let file = dir.path().join("file");
        fs::write(&file, b"occupied")?;
        assert!(record(&file.join("recording")).is_err());
        assert_eq!(fs::read(file)?, b"occupied");
        Ok(())
    }
}
