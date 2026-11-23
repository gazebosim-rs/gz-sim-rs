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
#[allow(dead_code)]
#[path = "../../gz-physics-gpu/examples/support/planar_tool/reference.rs"]
mod tool_reference;
use gz_physics_gpu::{PlanarArmLane, PlanarToolLane};
use gz_sim_scene::{
    EntityKind, JointSample, Name, PhysicsEngine, WorldLoadOptions, WorldPose,
    gpu_scene::planar_tool::observe, load_world_on_adapter,
};
use reference::turn;
use serde_json::json;
use std::{error::Error, fs, path::Path};
const WORLD: &str = include_str!("../../../examples/gpu_planar_tool_world.sdf");
fn reference_fixture() -> PlanarToolLane {
    PlanarToolLane {
        arm: PlanarArmLane {
            angle: [turn(0.48), turn(-0.62)],
            speed: [0.; 2],
            length: [0.30, 0.21],
            center: [0.15, 0.12],
            mass: [0.8, 0.4],
            inertia: [0.0062, 0.0034],
            tip_mass: 0.,
            gravity: 9.81,
            target: [turn(-0.36), turn(-0.50)],
            stiffness: [16., 5.],
            damping: [2.3, 0.41],
            effort_limit: [8., 2.8],
            effort: [0.; 2],
        },
        ground_z: 0.65 - 0.60,
        radius: 0.018,
        friction: 0.30,
        normal_impulse: 0.,
        tangent_impulse: 0.,
        first_contact_offset: -1.,
        first_normal_impulse: 0.,
        first_tangent_impulse: 0.,
        turns: [0.; 2],
    }
}
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
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let [adapter, out] = args.as_slice() else {
        return Err("expected ADAPTER NEW_OUTPUT_DIRECTORY".into());
    };
    run(adapter, Path::new(out))
}
