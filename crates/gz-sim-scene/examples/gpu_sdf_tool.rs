// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify the checked-in SDF robot via the same loader used by gz-sim-rs.
#[allow(dead_code)]
#[path = "../../gz-physics-gpu/examples/support/planar_arm/native.rs"]
mod native;
#[path = "../../gz-physics-gpu/examples/support/planar_tool/native.rs"]
mod native_tool;
#[allow(dead_code)]
#[path = "../../gz-physics-gpu/examples/support/planar_arm/reference.rs"]
mod reference;
#[path = "../../gz-physics-gpu/examples/support/planar_tool/server.rs"]
mod server;
#[allow(dead_code)]
#[path = "../../gz-physics-gpu/examples/support/planar_tool/reference.rs"]
mod tool_reference;
use gz_physics_gpu::PlanarToolLane;
use gz_sim_scene::{
    EntityKind, JointSample, Name, PhysicsEngine, WorldLoadOptions, WorldPose,
    gpu_scene::planar_tool::observe, load_world_on_adapter,
};
use reference::turn;
use serde_json::json;
use server::{drive_velocity, reference_fixture};
use std::{error::Error, fs, path::Path};
const WORLD: &str = include_str!("../../../examples/gpu_planar_tool_world.sdf");
fn run(adapter: &str, out: &Path) -> Result<(), Box<dyn Error>> {
    let mut sim = load_world_on_adapter(
        WORLD,
        "gpu_planar_tool_world.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(adapter),
    )?;
    let mut frames = vec![];
    let mut l = reference_fixture();
    let mut cpu = reference::State::new(&l.arm);
    let mut native = native_tool::NativeTool::new(&l);
    let mut err = [0_f64; 2];
    let mut total = [0_f64; 3];
    let mut initial = [None; 3];
    let mut first = [0_f64; 3];
    let mut last = [0; 3];
    let mut joint_error = 0f64;
    for tick in 0..=600 {
        let mut cpu_row = tool_reference::Observation {
            tip: cpu.points(&l.arm)[1],
            ..Default::default()
        };
        let mut native_row = cpu_row;
        if tick > 0 {
            if tick == 301 {
                l.arm.target = [turn(0.82), turn(-0.15)];
            }
            sim.step_exact(1)?;
            cpu_row = tool_reference::fine_step(&mut cpu, &l);
            native_row = native.step(&l);
        }
        let device = observe(&sim, 7)?;
        let q = device.arm.angle.map(|p| p[1].atan2(p[0]));
        let tip = device.arm.points()[1];
        for (j, o) in [cpu_row, native_row].into_iter().enumerate() {
            err[j] = err[j].max((tip[0] - o.tip[0]).hypot(tip[1] - o.tip[1]));
        }
        let rows = [
            (
                device.normal_impulse,
                device.first_normal_impulse,
                (device.first_contact_offset >= 0.).then_some(device.first_contact_offset),
            ),
            (cpu_row.n, cpu_row.first_n, cpu_row.first),
            (native_row.n, native_row.first_n, native_row.first),
        ];
        for (j, (n, hit, t)) in rows.into_iter().enumerate() {
            total[j] += n;
            if let Some(t) = t {
                if initial[j].is_none() {
                    initial[j] = Some((tick - 1) as f64 * 0.001 + t);
                    first[j] = hit;
                }
                last[j] = tick;
            }
        }
        let mut links = serde_json::Map::new();
        let mut joints = serde_json::Map::new();
        for e in sim.world().iter_entities() {
            let (Some(name), Some(kind)) = (e.get::<Name>(), e.get::<EntityKind>()) else {
                continue;
            };
            if *kind == EntityKind::Link {
                let p = e.get::<WorldPose>().unwrap();
                links.insert(
                    name.0.clone(),
                    json!({"translation":p.translation,"rotation":p.rotation}),
                );
            }
            if let Some(sample) = e.get::<JointSample>() {
                let i = usize::from(name.0.ends_with("::elbow"));
                joint_error =
                    joint_error.max((sample.state.position - (q[i] - [0.48, -0.62][i])).abs());
                joints.insert(name.0.clone(),json!({"position_rad":sample.state.position,"velocity_rad_s":sample.state.velocity}));
            }
        }
        frames.push(json!({"time_ns":tick*1_000_000,"links":links,"joints":joints,"gpu_tip_m":[tip[0],0.,0.6+tip[1]],
            "gpu_normal_impulse_ns":device.normal_impulse,"gpu_tangent_impulse_ns":device.tangent_impulse,
            "cartesian_tip_m":[cpu_row.tip[0],0.,0.6+cpu_row.tip[1]],"rapier_tip_m":[native_row.tip[0],0.,0.6+native_row.tip[1]]}));
    }
    let result = json!({"adapter":adapter,"environment_id":7,"ticks":600,"step_ns":1_000_000,"fine_step_ns":62_500,"reference_order":["gpu","cartesian-rk4","rapier"],
        "first_contact_s":initial,"first_normal_impulse_ns":first,"total_normal_impulse_ns":total,"last_loaded_public_tick":last,"max_tip_error_m":err,"joint_publication_max_error_rad":joint_error});
    println!("{}", serde_json::to_string_pretty(&result)?);
    if err.iter().any(|e| *e > 0.001)
        || initial.iter().any(Option::is_none)
        || last.iter().any(|t| !(301..590).contains(t))
        || joint_error > 1e-10
    {
        return Err("SDF tool qualification failed".into());
    }
    fs::create_dir(out)?;
    fs::write(out.join("frames.json"), serde_json::to_vec(&frames)?)?;
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    Ok(())
}
#[derive(serde::Deserialize)]
struct Published {
    tick_ms: usize,
    position_rad: [f64; 2],
    command_rad: [f64; 2],
    normal_impulse_ns: f64,
    #[serde(default)]
    stop_impulse_nms: [f64; 2],
}
fn compare_commands(
    trace: &Path,
    out: &Path,
    stopped: bool,
    passes: Option<std::num::NonZeroUsize>,
) -> Result<(), Box<dyn Error>> {
    let frames: Vec<Published> = serde_json::from_slice(&fs::read(trace)?)?;
    if frames.len() != 600 {
        return Err("expected 600 consecutive transport observations".into());
    }
    let mut l = reference_fixture();
    l.arm.damping = [6., 1.5];
    l.velocity_mode = [1.; 2];
    let limits = if stopped {
        [Some([-0.36, 0.02]), Some([-0.15, 1.20])]
    } else {
        [None; 2]
    };
    let arm_at_limits = |limits| {
        let mut native = native_tool::NativeTool::new_with_limits(&l, limits);
        if let Some(passes) = passes {
            native
                .arm
                .world
                .integration_parameters
                .num_internal_stabilization_iterations = passes.get();
        }
        native
    };
    let mut native = arm_at_limits(limits);
    let stabilization = native
        .arm
        .world
        .integration_parameters
        .num_internal_stabilization_iterations;
    let mut first_stop = [[None; 2]; 2];
    let mut reference_residual = [0_f64; 2];
    let mut max_free_elbow = 0_f64;
    let mut controls = stopped.then(|| {
        [
            arm_at_limits([None, limits[1]]),
            arm_at_limits([limits[0].map(|[lo, _]| [lo, f64::MAX]), limits[1]]),
        ]
    });
    let mut native_departure = [None; 2];
    let mut combined = 0;
    let mut stop_impulse = [0_f64; 2];
    let mut peak_stop = [0_f64; 2];
    let mut travel_error = [0_f64; 2];
    let mut coordinate_error = [0_f64; 2];
    let mut total = [0_f64; 2];
    let mut first = [None; 2];
    let mut last = [0; 2];
    let mut max_tip = 0_f64;
    let origin = [0.48, -0.62];
    for (i, frame) in frames.iter().enumerate() {
        if frame.tick_ms != i + 1 {
            return Err("incomplete transport clock".into());
        }
        l.velocity_target = drive_velocity(frame.command_rad, native.arm.angles());
        let n = native.step(&l);
        let native_position = native.arm.angles();
        if let Some(controls) = &mut controls {
            for (k, control) in controls.iter_mut().enumerate() {
                let mut control_lane = l;
                control_lane.velocity_target =
                    drive_velocity(frame.command_rad, control.arm.angles());
                control.step(&control_lane);
                if control.arm.angles() != native_position
                    || control.arm.rates() != native.arm.rates()
                {
                    native_departure[k].get_or_insert(frame.tick_ms);
                }
            }
        }
        for axis in 0..2 {
            if let Some([lo, hi]) = limits[axis] {
                travel_error[0] = travel_error[0]
                    .max(lo - frame.position_rad[axis])
                    .max(frame.position_rad[axis] - hi);
                let native = native_position[axis] - origin[axis];
                travel_error[1] = travel_error[1].max(lo - native).max(native - hi);
                coordinate_error[axis] =
                    coordinate_error[axis].max((frame.position_rad[axis] - native).abs());
            }
            let p = frame.stop_impulse_nms[axis];
            if p != 0. {
                first_stop[axis][usize::from(p < 0.)].get_or_insert((
                    frame.tick_ms,
                    p,
                    native.limit_residual[axis],
                ));
            }
            reference_residual[axis] += native.limit_residual[axis];
            if axis == 1 {
                max_free_elbow = max_free_elbow.max(native.limit_residual[axis].abs());
            }
            stop_impulse[axis] += p;
            peak_stop[axis] = peak_stop[axis].max(p.abs());
        }
        if frame.normal_impulse_ns > 0. && frame.stop_impulse_nms.iter().any(|p| *p != 0.) {
            combined += 1
        }
        let a = frame.position_rad[0] + origin[0];
        let b = a + frame.position_rad[1] + origin[1];
        let x = 0.30 * a.cos() + 0.21 * b.cos();
        let z = 0.30 * a.sin() + 0.21 * b.sin();
        max_tip = max_tip.max((x - n.tip[0]).hypot(z - n.tip[1]));
        for (axis, impulse) in [frame.normal_impulse_ns, n.n].into_iter().enumerate() {
            total[axis] += impulse;
            if impulse > 0. {
                first[axis].get_or_insert((frame.tick_ms, impulse));
                last[axis] = frame.tick_ms;
            }
        }
    }
    let result = json!({"order":["published_gpu_trace","independent_native_rapier"],"first_loaded_public_tick_and_impulse_ns":first,"last_loaded_public_tick":last,"total_normal_impulse_ns":total,"max_tip_error_m":max_tip,"native_seconds_per_substep":0.001/16.,"native_stabilization_passes":stabilization,"authored_stops":stopped,"native_lower_upper_departure_tick":native_departure,"native_signed_limit_residual_nms":reference_residual,"max_native_free_elbow_residual_nms":max_free_elbow,"limits_rad":limits,"first_stop_tick_and_signed_impulse_nms":first_stop,"floor_and_stop_loaded_ticks":combined,"total_signed_stop_impulses_nms":stop_impulse,"max_abs_stop_impulse_nms":peak_stop,"max_travel_violation_rad":travel_error,"max_joint_difference_rad":coordinate_error});
    fs::write(out, serde_json::to_vec_pretty(&result)?)?;
    println!("{result}");
    Ok(())
}
#[path = "gpu_sdf_tool/landing.rs"]
mod landing;

fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if let [mode, trace, out] = args.as_slice()
        && mode == "--landing"
    {
        return landing::run(Path::new(trace), Path::new(out));
    }
    if let [mode, trace, out, options @ ..] = args.as_slice()
        && matches!(mode.as_str(), "--commands" | "--stops")
    {
        let passes = match options {
            [] => None,
            [flag, count] if flag == "--stabilization-passes" => Some(count.parse()?),
            _ => return Err("expected optional --stabilization-passes POSITIVE_COUNT".into()),
        };
        return compare_commands(Path::new(trace), Path::new(out), mode == "--stops", passes);
    }
    let [adapter, out] = args.as_slice() else {
        return Err("expected ADAPTER NEW_OUTPUT_DIRECTORY".into());
    };
    run(adapter, Path::new(out))
}
