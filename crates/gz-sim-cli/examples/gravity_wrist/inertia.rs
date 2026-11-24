// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Accelerate a fixed-pivot wrist about an oblique axis, then brake it to rest.
use super::*;
use glam::DMat3;
use gz_physics::joint_drive::SphericalDrive;
use gz_sim_scene::{BallJointSample, locked_subtree_inertia};
use serde::Serialize;

const HALF: usize = 400;
const END: usize = 1000;
const DT: f64 = 0.001;
const LIMIT_NM: f64 = 5.2;
const DISTURBANCE_NM: [f64; 3] = [8., -12., 6.];

#[derive(Clone, Debug, PartialEq, Serialize)]
struct Sample {
    actual_xyzw: [f64; 4],
    velocity_rad_s: [f64; 3],
    torque_nm: [f64; 3],
    angle_error_rad: f64,
    velocity_error_rad_s: f64,
    inertia_error_kg_m2: f64,
    center_error_m: f64,
    angular_momentum_world_kg_m2_s: [f64; 3],
    applied_torque_parent_nm: [f64; 3],
    gravity_moment_world_nm: [f64; 3],
    total_moment_world_nm: [f64; 3],
    kinetic_energy_j: f64,
    potential_energy_j: f64,
    kinetic_reference_error_j: f64,
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
    stabilize: bool,
    mut capture: impl FnMut(usize, &Simulation) -> Result<()>,
) -> Result<Vec<[Sample; 2]>> {
    let wrists = ["held", "free"].map(|m| Wrist::new(sim, m));
    let entities = ["held", "free"].map(|m| named(sim, &format!("default::{m}::bearing")));
    let axis = DVec3::new(0.4, -0.18, 0.9).normalize();
    let substeps = 1_000_000 / sim.step_duration().as_nanos();
    let drives = [
        SphericalDrive::new(24., 2.4, LIMIT_NM)?,
        SphericalDrive::new(0., 0., LIMIT_NM)?,
    ];
    let end = if stabilize { 2 * END } else { END };
    let mut trace = Vec::with_capacity(end + 1);
    for tick in 0..=end {
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
            let tensor = if i == 0 || stabilize {
                real
            } else {
                DMat3::from_diagonal(DVec3::new(real.x_axis.x, real.y_axis.y, real.z_axis.z))
            };
            let gravity = DVec3::from(joint_gravity_load(sim, wrist.bearing)?.torque_nm);
            let torque = tensor * (axis * acceleration) + omega.cross(tensor * omega) - gravity;
            let target = DQuat::from_axis_angle(axis, angle);
            let torque = if stabilize {
                drives[i].torque(
                    DQuat::from_array(encoder.orientation),
                    target,
                    omega,
                    rate * axis,
                    torque,
                )?
            } else {
                torque
            };
            let residual =
                DQuat::from_array(encoder.orientation) * DQuat::from_axis_angle(axis, -angle);
            let (pivot, carrier) = pose(sim, wrist.carrier);
            let axes = carrier * rotation([0.31, -0.42, 0.63]);
            let momentum =
                gz_sim_scene::subtree_momentum_about(sim, wrist.bearing, pivot.to_array())?;
            let disturbance = if stabilize && (450..470).contains(&tick) {
                DVec3::from_array(DISTURBANCE_NM)
            } else {
                DVec3::ZERO
            };
            let applied = torque + axes.inverse() * disturbance;
            Ok(Sample {
                angular_momentum_world_kg_m2_s: momentum.angular_kg_m2_s,
                applied_torque_parent_nm: applied.to_array(),
                gravity_moment_world_nm: (axes * gravity).to_array(),
                total_moment_world_nm: (axes * (applied + gravity)).to_array(),
                kinetic_energy_j: momentum.kinetic_energy_j,
                potential_energy_j: 3. * 9.81 * (pivot + axes * center).z,
                kinetic_reference_error_j: (momentum.kinetic_energy_j
                    - 0.5 * omega.dot(independent * omega))
                .abs(),
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
        if tick < end {
            for _ in 0..substeps {
                for i in 0..2 {
                    if stabilize && (450..470).contains(&tick) {
                        let id = sim
                            .world()
                            .get::<EntityId>(wrists[i].payload)
                            .unwrap()
                            .get();
                        gz_sim_scene::queue_link_world_wrench(sim, id, [0.; 3], DISTURBANCE_NM)?;
                    }
                    queue_ball_joint_effort(sim, wrists[i].bearing, pair[i].torque_nm)?;
                }
                sim.step_exact(1)?;
            }
        }
        trace.push(pair);
    }
    Ok(trace)
}

/// Work uses the committed quaternion increment of the moving joint. Momentum
/// balance uses the external moment about the fixed bearing, in world axes.
fn energy_balance(trace: &[[Sample; 2]], arm: usize) -> Value {
    let first = &trace[0][arm];
    let energy0 = first.kinetic_energy_j + first.potential_energy_j;
    let momentum0 = DVec3::from(first.angular_momentum_world_kg_m2_s);
    let mut work = 0.;
    let mut angular_impulse = DVec3::ZERO;
    let mut peak_work_residual = 0_f64;
    let mut peak_momentum_residual = 0_f64;
    let mut peak_kinetic_disagreement = first.kinetic_reference_error_j;
    for pair in trace.windows(2) {
        let (a, b) = (&pair[0][arm], &pair[1][arm]);
        let delta = DQuat::from_array(b.actual_xyzw) * DQuat::from_array(a.actual_xyzw).inverse();
        work += DVec3::from(a.applied_torque_parent_nm).dot(delta.to_scaled_axis());
        let gravity_delta =
            DVec3::from(b.gravity_moment_world_nm) - DVec3::from(a.gravity_moment_world_nm);
        angular_impulse += DT * (DVec3::from(a.total_moment_world_nm) + 0.5 * gravity_delta);
        let residual = b.kinetic_energy_j + b.potential_energy_j - energy0 - work;
        peak_work_residual = peak_work_residual.max(residual.abs());
        peak_momentum_residual = peak_momentum_residual.max(
            (DVec3::from(b.angular_momentum_world_kg_m2_s) - momentum0 - angular_impulse).length(),
        );
        peak_kinetic_disagreement = peak_kinetic_disagreement.max(b.kinetic_reference_error_j);
    }
    json!({"applied_work_j":work,"peak_work_residual_j":peak_work_residual,
        "peak_angular_impulse_residual_nm_s":peak_momentum_residual,"peak_kinetic_reference_error_j":peak_kinetic_disagreement})
}

pub(super) fn record(out: &Path, stabilize: bool, substeps: u8) -> Result<()> {
    if out.exists() {
        return Err("choose a new inertia recording directory".into());
    }
    if !matches!(substeps, 1 | 2 | 4) {
        return Err(
            "native substeps must evenly divide the actuator tick: choose 1, 2 or 4".into(),
        );
    }
    let native_dt = std::time::Duration::from_nanos(1_000_000 / u64::from(substeps));
    let mut sim = gz_sim_scene::load_world_with_engine(
        XML,
        "gravity_wrist.sdf",
        "default",
        native_dt,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let sdf = sdformat::load_str(XML, "gravity_wrist.sdf", &sdformat::NoResources)?;
    let mut frames = Vec::new();
    let first = episode(&mut sim, stabilize, |_, s| {
        frames.push(gz_sim_cli::primitive_frame(
            s,
            &sdf,
            "default",
            frames.len() as u64 + 1,
        )?);
        Ok(())
    })?;
    reset_loaded_world(&mut sim, |_| ())?;
    let second = episode(&mut sim, stabilize, |_, _| Ok(()))?;
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
    let end = first.len() - 1;
    let mut result = json!({"ticks":end,"dt_s":DT,"target_angle_rad":trajectory(END).0,"full_peak_angle_error_rad":peak(0),"diagonal_peak_angle_error_rad":peak(1),"max_tensor_error_kg_m2":max_tensor,"last":first[end],"reset_exact":true});
    if stabilize {
        let maximum_effort = |i: usize| {
            first
                .iter()
                .map(|p| DVec3::from(p[i].torque_nm).length())
                .fold(0., f64::max)
        };
        result = json!({"ticks":end, "dt_s":DT, "target_angle_rad":trajectory(end).0,
            "disturbance_ticks":[450,470],"world_disturbance_nm":DISTURBANCE_NM,
            "drive_limit_nm":LIMIT_NM, "max_torque_nm":[maximum_effort(0),maximum_effort(1)],
            "peak_angle_error_rad":[peak(0),peak(1)],"last":first[end], "reset_exact":true});
    }
    result["native_step_ns"] = json!(native_dt.as_nanos());
    result["balance"] = json!([energy_balance(&first, 0), energy_balance(&first, 1)]);
    let captures = if stabilize {
        json!({"before_pulse":45,"after_pulse":47,"braking":65,"hold":100,"end":end/10})
    } else {
        json!({"accelerating":20,"peak_speed":40,"rest":80,"end":end/10})
    };
    fs::create_dir_all(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    for (name, value) in [
        ("comparison.json", result.clone()),
        ("trace.json", serde_json::to_value(first)?),
        (
            "frames.json",
            json!({"kind":"authored-archive-v1","dt_ns":1,"camera":"perspective","captures":captures}),
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
        assert!(record(&out, false, 3).is_err());
        record(&out, false, 1)?;
        assert!(record(&out, false, 1).is_err());
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
        assert!(episode(&mut sim, false, |_, _| Err("interrupted".into())).is_err());
        for model in ["held", "free"] {
            let sensor = named(&sim, &format!("default::{model}::bearing"));
            let sample = sim
                .world_mut()
                .entity_mut(sensor)
                .take::<BallJointSample>()
                .unwrap();
            assert!(episode(&mut sim, false, |_, _| Ok(())).is_err());
            sim.world_mut().entity_mut(sensor).insert(sample);
        }
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
            assert!(episode(&mut sim, false, |_, _| Ok(())).is_err());
        }
        let dir = tempfile::tempdir()?;
        let file = dir.path().join("file");
        fs::write(&file, b"occupied")?;
        assert!(record(&file.join("recording"), false, 1).is_err());
        assert_eq!(fs::read(file)?, b"occupied");
        Ok(())
    }
    #[test]
    fn a_torque_limited_drive_recovers_from_the_same_external_disturbance() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let out = dir.path().join("disturbed");
        record(&out, true, 1)?;
        let value: serde_json::Value =
            serde_json::from_slice(&fs::read(out.join("comparison.json"))?)?;
        let peak = value["peak_angle_error_rad"][0].as_f64().unwrap();
        assert!(peak > 0.03 && peak < 0.08);
        assert!(value["last"][0]["angle_error_rad"].as_f64().unwrap() < 0.001);
        assert!(value["last"][1]["velocity_error_rad_s"].as_f64().unwrap() > 1.);
        for v in value["max_torque_nm"].as_array().unwrap() {
            assert!((v.as_f64().unwrap() - LIMIT_NM).abs() < 1e-12);
        }
        assert_eq!(value["reset_exact"], true);
        Ok(())
    }
    #[test]
    fn angular_impulse_and_energy_balance_converge_on_a_fixed_actuator_clock() -> Result<()> {
        let d = tempfile::tempdir()?;
        let mut previous: Option<Value> = None;
        for n in [1, 2, 4] {
            let out = d.path().join(n.to_string());
            record(&out, true, n)?;
            let current: Value = serde_json::from_slice(&fs::read(out.join("comparison.json"))?)?;
            assert_eq!(current["ticks"], 2000);
            assert_eq!(current["native_step_ns"], 1_000_000 / u64::from(n));
            for i in 0..2 {
                let row = &current["balance"][i];
                for metric in [
                    "peak_work_residual_j",
                    "peak_angular_impulse_residual_nm_s",
                    "peak_kinetic_reference_error_j",
                ] {
                    let now = row[metric].as_f64().unwrap();
                    if let Some(last) = &previous {
                        assert!(
                            now < 0.65 * last["balance"][i][metric].as_f64().unwrap(),
                            "{n} {i} {metric} {now}"
                        );
                    }
                }
                assert!(row["peak_work_residual_j"].as_f64().unwrap() < 0.005);
            }
            previous = Some(current);
        }
        Ok(())
    }
}
