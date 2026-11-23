// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify the planar arm profile on a named Vulkan GPU against Cartesian and native dynamics.
#[path = "support/planar_arm/capture.rs"]
mod capture;
#[path = "support/planar_arm/native.rs"]
mod native;
#[path = "support/planar_arm/reference.rs"]
mod reference;
use gz_physics_gpu::{
    PlanarArmBatch,
    simulation::{BatchTiming, planar_arm_simulation},
};
use reference::{State, fixture};
use serde_json::json;
use std::{error::Error, fs, hint::black_box, io, path::Path, time::Instant};
const TICKS: usize = 600;
const COUNT: usize = 2048;
const REVERSE: usize = 301;
fn second_target(i: usize) -> [f64; 2] {
    [-0.14 - 0.03 * (i % 3) as f64, 0.48 + 0.02 * (i % 5) as f64]
}
fn reverse_device(batch: &mut PlanarArmBatch) -> Result<(), gz_physics_gpu::GpuError> {
    for i in 0..batch.lanes().len() {
        batch.set_target_angles(i, second_target(i))?;
    }
    Ok(())
}
fn reverse_reference(lanes: &mut [gz_physics_gpu::PlanarArmLane]) {
    for (i, l) in lanes.iter_mut().enumerate() {
        l.target = second_target(i).map(reference::turn);
    }
}
fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn summary(mut v: Vec<u128>) -> serde_json::Value {
    v.sort_unstable();
    let n = v.len();
    json!({"p50_ns":v[(50*n).div_ceil(100)-1] as u64,"p95_ns":v[(95*n).div_ceil(100)-1] as u64})
}
fn run(name: &str, out: &Path) -> Result<(), Box<dyn Error>> {
    if out.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, out.display().to_string()).into());
    }
    let seeds = (0..COUNT).map(fixture).collect::<Vec<_>>();
    let mut sim = planar_arm_simulation(name, seeds.clone())?;
    let mut capture = capture::Capture::default();
    capture.sample(0, &seeds);
    let ids = (0..24).map(|k| k * 83).collect::<Vec<_>>();
    let mut cpu = seeds.iter().map(State::new).collect::<Vec<_>>();
    let mut fine = ids.iter().map(|&i| cpu[i]).collect::<Vec<_>>();
    let mut native = ids
        .iter()
        .map(|&i| native::Native::new(&seeds[i]))
        .collect::<Vec<_>>();
    let mut driven = seeds.clone();
    let mut discrete = 0_f64;
    let mut rate_error = 0_f64;
    let mut angle_error = 0_f64;
    let mut fine_error = [0_f64; 3];
    let mut native_error = [0_f64; 3];
    let mut native_fine = [0_f64; 3];
    let mut max_effort = [0_f64; 2];
    let mut max_excursion = 0_f64;
    for tick in 1..=TICKS {
        if tick == REVERSE {
            reverse_device(&mut sim.world_mut().resource_mut::<PlanarArmBatch>())?;
            reverse_reference(&mut driven);
        }
        sim.step_exact(1)?;
        let batch = sim.world().resource::<PlanarArmBatch>();
        let lanes = batch.lanes();
        for ((s, l), device) in cpu.iter_mut().zip(&driven).zip(lanes) {
            s.discrete(l);
            if !s.q.into_iter().chain(s.w).all(f64::is_finite) {
                return Err("nonfinite discrete reference".into());
            }
            for j in 0..2 {
                let [x, y] = device.angle[j];
                let [cx, cy] = reference::turn(s.q[j]);
                angle_error = angle_error.max((y * cx - x * cy).atan2(x * cx + y * cy).abs());
                rate_error = rate_error.max((s.w[j] - device.speed[j]).abs());
            }
            discrete = discrete.max(distance(s.points(l)[1], device.points()[1]));
            for (peak, u) in max_effort.iter_mut().zip(device.effort) {
                *peak = peak.max(u.abs());
            }
        }
        for (j, &i) in ids.iter().enumerate() {
            let l = &driven[i];
            fine[j].fine(l);
            native[j].step(l);
            let tip = lanes[i].points()[1];
            let ideal = fine[j].points(l)[1];
            let multibody = native[j].tip(l);
            if !ideal.into_iter().chain(multibody).all(f64::is_finite) {
                return Err("nonfinite reference".into());
            }
            let group = i % 3;
            fine_error[group] = fine_error[group].max(distance(tip, ideal));
            native_error[group] = native_error[group].max(distance(tip, multibody));
            native_fine[group] = native_fine[group].max(distance(ideal, multibody));
            max_excursion = max_excursion.max(distance(tip, l.points()[1]));
        }
        if tick % 10 == 0 {
            capture.sample((tick * 1_000_000) as u64, lanes);
        }
    }
    let final_snapshot = sim.world().resource::<PlanarArmBatch>().lanes().to_vec();
    sim.reset()?;
    let mut wall = vec![];
    let mut dispatch = vec![];
    let mut fence = vec![];
    let mut readback = 0;
    for tick in 1..=TICKS {
        let start = Instant::now();
        if tick == REVERSE {
            reverse_device(&mut sim.world_mut().resource_mut::<PlanarArmBatch>())?;
        }
        sim.step_exact(1)?;
        black_box(sim.world().resource::<PlanarArmBatch>().lanes());
        wall.push(start.elapsed().as_nanos());
        let timing = sim.world().resource::<BatchTiming>().last.as_ref().unwrap();
        if let Some(t) = timing.gpu_dispatch {
            dispatch.push(t.as_nanos());
        }
        fence.push(timing.wait_map.as_nanos());
        readback = timing.readback_bytes;
    }
    let batch = sim.world().resource::<PlanarArmBatch>();
    let info = batch.device_info();
    let same = bytemuck::cast_slice::<_, u8>(&final_snapshot)
        == bytemuck::cast_slice::<_, u8>(batch.lanes());
    let mut timing_cpu = vec![];
    let mut repeat = seeds.iter().map(State::new).collect::<Vec<_>>();
    let mut cpu_drives = seeds.clone();
    for tick in 1..=TICKS {
        let start = Instant::now();
        if tick == REVERSE {
            reverse_reference(&mut cpu_drives);
        }
        for (s, l) in repeat.iter_mut().zip(&cpu_drives) {
            s.discrete(l);
        }
        black_box(&repeat);
        timing_cpu.push(start.elapsed().as_nanos());
    }
    let result = json!({"adapter":info.name,"driver":info.driver_info,"backend":info.backend,"ticks":TICKS,"lanes":COUNT,
        "reverse_before_tick":REVERSE,"tick_ns":1_000_000,"reference_lane_ids":ids,"reference_substeps":8,"payloads_kg":[0.,0.1,0.25],
        "same_clock_max_angle_error_rad":angle_error,"same_clock_max_speed_error_rad_per_s":rate_error,"same_clock_max_tip_error_m":discrete,"fine_rk4_max_tip_error_m":fine_error,"fine_rapier_max_tip_error_m":native_error,
        "rk4_rapier_max_tip_difference_m":native_fine,"max_torque_nm":max_effort,"max_tip_travel_m":max_excursion,
        "byte_equal_after_reset":same,"dense_bevy_step":summary(wall),
        "gpu_dispatch":(!dispatch.is_empty()).then(||summary(dispatch)),"host_wait_map":summary(fence),
        "serial_cartesian_reference_step":summary(timing_cpu),"readback_bytes_per_tick":readback});
    println!("{}", serde_json::to_string_pretty(&result)?);
    if !same
        || discrete > 1e-10
        || angle_error > 1e-9
        || rate_error > 1e-8
        || fine_error.into_iter().any(|e| e > 1e-4)
        || native_error.into_iter().any(|e| e > 5e-4)
    {
        return Err("reset, same-clock or fine trajectory qualification failed".into());
    }
    let parent = out
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::tempdir_in(parent)?;
    capture.write(stage.path())?;
    fs::write(
        stage.path().join("comparison.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    fs::rename(stage.path(), out)?;
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let [name, out] = args.as_slice() else {
        return Err("expected ADAPTER NEW_OUTPUT_DIRECTORY".into());
    };
    run(name, Path::new(out))
}
