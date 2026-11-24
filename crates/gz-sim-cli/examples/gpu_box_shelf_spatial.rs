// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Freely rotating cuboid with spatial normal and tangential shelf contact.
#[path = "support/shelf_reference.rs"]
mod reference;
#[path = "../../gz-sim-scene/examples/box_spatial/visual_frame.rs"]
mod visual;
use glam::{DQuat, DVec3};
use gz_physics_gpu::{CarryGripperBatch, SpatialShelfLoads};
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_on_adapter, reset_loaded_world};
use reference::{MASS, Sample, Settings, WEIGHT, compare, cpu_tick, summarize};
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path, time::Instant};
const SDF: &str = include_str!("../../../examples/gpu_box_shelf_spatial.sdf");
struct Trial {
    result: Value,
    samples: Vec<(usize, Sample)>,
    loads: Vec<SpatialShelfLoads>,
}

fn run(adapter: &str, name: &str, src: &str, dir: &Path) -> Result<Trial, Box<dyn Error>> {
    let sdf = sdformat::load_str(src, name, &sdformat::NoResources)?;
    let mut sim = load_world_on_adapter(
        src,
        name,
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
    let mut frames = vec![gz_sim_cli::primitive_frame(&sim, &sdf, "default", 1)?];
    let mut poses = vec![visual::visual_frame(
        0,
        sim.world().resource::<CarryGripperBatch>(),
    )];
    let mut measured = Vec::new();
    let mut cpu_trace = Vec::new();
    let mut reference_trials: [Vec<(usize, Sample)>; 4] = std::array::from_fn(|_| Vec::new());
    let hard = Settings {
        stabilization: 0,
        ..Settings::default()
    };
    let alternatives = [
        Settings::default(),
        Settings {
            reversed_shelf_frame: true,
            ..Settings::default()
        },
        Settings {
            reversed_shelf_frame: true,
            ..hard
        },
        Settings { steps: 128, ..hard },
    ];
    let mut loads = Vec::new();
    let mut costs = Vec::new();
    let mut cpu_costs = Vec::new();
    let mut max_wrench_error = [0_f64; 2];
    let mut max_excess = 0_f64;
    let mut work = [0_f64; 2];
    let mut added_energy = 0_f64;
    let mut total = DVec3::ZERO;
    let mut angular = DVec3::ZERO;
    for epoch in 0..2 {
        if epoch > 0 {
            reset_loaded_world(&mut sim, |_| ()).map_err(|e| format!("{e:?}"))?;
        }
        let mut native = None;
        let mut variants = [const { None }; 4];
        let mut last: Option<(DVec3, DVec3)> = None;
        let end = if name == "spin_slick" { 2250 } else { 2650 };
        for tick in 1..=end {
            let start = Instant::now();
            if let Err(err) = sim.step_exact(1) {
                return Err(format!(
                    "{name} stopped at tick {tick}: {err:?}, {:?}",
                    sim.fault().map(ToString::to_string)
                )
                .into());
            }
            let elapsed = start.elapsed().as_secs_f64();
            let b = sim.world().resource::<CarryGripperBatch>();
            if epoch == 0 {
                poses.push(visual::visual_frame(tick, b));
                if tick % 80 == 0
                    || tick == end
                    || (2118..=2140).contains(&tick)
                    || (2165..=2180).contains(&tick)
                    || (2155..=2350).contains(&tick) && tick % 3 == 0
                {
                    frames.push(gz_sim_cli::primitive_frame(
                        &sim,
                        &sdf,
                        "default",
                        frames.len() as u64 + 1,
                    )?);
                }
            }
            let Some(body) = b.free_box() else { continue };
            let lane = b.lanes()[0];
            let sample = Sample::gpu(b);
            let receipt = b.spatial_shelf_loads().copied().unwrap_or_default();
            costs.push(elapsed);
            if epoch == 0 {
                let start = Instant::now();
                let reference = cpu_tick(&mut native, body, &lane, hard);
                cpu_costs.push(start.elapsed().as_secs_f64());
                for (((i, trial), solver), settings) in reference_trials
                    .iter_mut()
                    .enumerate()
                    .zip(&mut variants)
                    .zip(alternatives)
                {
                    if name == "center" || i == 0 || i == 3 {
                        trial.push((tick, cpu_tick(solver, body, &lane, settings)));
                    }
                }
                if let Some((v, m)) = last {
                    let dv = DVec3::from(body.center_velocity) - v + 0.00981 * DVec3::Z;
                    let dm = DVec3::from(body.momentum) - m;
                    let mut sum = DVec3::ZERO;
                    for impulse in receipt.world_impulses {
                        let j = DVec3::from(impulse);
                        assert!(j.z >= 0.);
                        max_excess = max_excess.max(j.x.hypot(j.y) - lane.shelf_friction * j.z);
                        sum += j;
                    }
                    max_wrench_error[0] = max_wrench_error[0].max((MASS * dv - sum).length());
                    max_wrench_error[1] = max_wrench_error[1]
                        .max((dm - DVec3::from(receipt.angular_impulse)).length());
                    for (a, b) in work.iter_mut().zip(receipt.contact_work) {
                        *a += b;
                    }
                    added_energy = added_energy.max(receipt.max_added_energy);
                    total += sum;
                    angular += DVec3::from(receipt.angular_impulse);
                }
                last = Some((
                    DVec3::from(body.center_velocity),
                    DVec3::from(body.momentum),
                ));
                measured.push((tick, sample));
                cpu_trace.push((tick, reference));
                loads.push(receipt);
            } else {
                assert_eq!((tick, sample), measured[tick - 2041]);
                assert_eq!(receipt, loads[tick - 2041]);
            }
        }
    }
    assert!(
        max_excess < 1e-12 && max_wrench_error[0] < 1e-11 && max_wrench_error[1] < 1e-11,
        "{max_excess} {max_wrench_error:?}"
    );
    let impulse = |samples: &[(usize, Sample)]| {
        let first = samples.iter().position(|(_, s)| s.impulse > 1e-10).unwrap();
        let start = samples[first - 1].1;
        let end = samples[first].1;
        let linear = MASS
            * (DVec3::from(end.center_velocity) - DVec3::from(start.center_velocity)
                + 0.00981 * DVec3::Z);
        let momentum = |s: Sample| {
            let q = DQuat::from_array(s.orientation);
            q * (reference::MOMENTS * (q.inverse() * DVec3::from(s.omega)))
        };
        (linear, momentum(end) - momentum(start))
    };
    let (jg, lg) = impulse(&measured);
    let (jr, lr) = impulse(&cpu_trace);
    let first_response = json!({"gpu_linear_ns":jg.to_array(),"gpu_angular_nms":lg.to_array(),"rapier_linear_ns":jr.to_array(),"rapier_angular_nms":lr.to_array(),"linear_error_ns":jg.distance(jr),"angular_error_nms":lg.distance(lr)});
    let errors = compare(&measured, &cpu_trace);
    let yaw = |s: Sample| {
        let x = DQuat::from_array(s.orientation) * DVec3::X;
        x.y.atan2(x.x).to_degrees()
    };
    let g = summarize(&measured);
    let c = summarize(&cpu_trace);
    let mean = g["last_200_mean_up_n"].as_f64().unwrap();
    let first = loads
        .iter()
        .position(|r: &SpatialShelfLoads| r.world_impulses.iter().any(|j| j[2] > 1e-9))
        .unwrap();
    let first_load = loads[first].world_impulses;
    let alternative_metrics:Vec<_>=reference_trials.iter().enumerate().filter(|(_,t)|!t.is_empty()).map(|(i,trial)|{
        let last=trial.last().unwrap().1;
        let label=["previous","previous_half_turn","hard_half_turn","hard_128_substeps"][i];
        json!({"mode":label,
            "yaw_degrees":yaw(last),"final_position_m":last.position,"first_impulse_ns":(summarize(trial)["first_up_ns"]),"errors_vs_gpu":compare(&measured,trial)})
    }).collect();
    let last = measured.last().unwrap().1;
    let gpu_yaw = yaw(last);
    let native_yaw = yaw(cpu_trace.last().unwrap().1);
    assert!(added_energy < 1e-12 && jg.z > 0. && jr.z > 0.);
    if name == "spin_slick" {
        assert!(work[1] == 0. && angular.z == 0.);
        assert!(last.omega[2] > 3. && gpu_yaw > 39. && (gpu_yaw - native_yaw).abs() < 0.01);
    } else {
        assert!((mean - WEIGHT).abs() < 0.001 && work[1] < 0.);
        assert!(g["begin_40ms_supported_rest_ms"].is_number());
        if matches!(name, "spin_plus" | "spin_minus") {
            let sign = if name == "spin_plus" { 1. } else { -1. };
            assert!(sign * gpu_yaw > 15. && sign * lg.z < -6e-5);
            assert!(errors[0] < 0.0001 && jg.distance(jr) < 3e-6 && lg.distance(lr) < 1e-8);
            assert!(DVec3::from(last.omega).length() < 0.001);
            assert!((gpu_yaw - native_yaw).abs() < 0.02);
            for v in &alternative_metrics {
                assert!((gpu_yaw - v["yaw_degrees"].as_f64().unwrap()).abs() < 0.2);
            }
        } else if name == "center" {
            assert!(gpu_yaw.abs() < 1e-9 && native_yaw.abs() < 0.005);
            assert!(
                alternative_metrics[2]["yaw_degrees"]
                    .as_f64()
                    .unwrap()
                    .abs()
                    < 0.005
            );
        }
    }
    let result = json!({"name":name,"matched_first_response":first_response,"native_control_settings":alternative_metrics,"normal_and_tangential_work_j":work,"largest_positive_contact_energy_j":added_energy,"gpu":g,"rapier":c,"mean_error_n":(mean-WEIGHT).abs(),"first_tick":measured[first].0,"first_vertex_impulses_ns":first_load,
        "total_impulse_ns":total.to_array(),"total_angular_impulse_nms":angular.to_array(),"max_coulomb_excess_ns":max_excess,"max_linear_angular_balance_error":max_wrench_error,
        "max_link_error_m":errors[0],"max_orientation_error_rad":errors[1],"final_link_error_m":errors[2],"final_orientation_error_rad":errors[3],
        "gpu_yaw_deg":yaw(measured.last().unwrap().1),"rapier_yaw_deg":yaw(cpu_trace.last().unwrap().1),"gpu_two_epochs":reference::rate(&mut costs),"cpu_reference_32_substeps":reference::rate(&mut cpu_costs)});
    gz_sim_cli::export_archive(
        &dir.join(format!("{name}-replay")),
        Default::default(),
        frames,
    )?;
    fs::write(
        dir.join(format!("{name}-poses.json")),
        serde_json::to_vec(&poses)?,
    )?;
    let impulses: Vec<_> = loads
        .iter()
        .map(|s| json!({"vertices":s.world_impulses,"angular":s.angular_impulse,"work_j":s.contact_work,"max_added_energy_j":s.max_added_energy}))
        .collect();
    fs::write(
        dir.join(format!("{name}-trace.json")),
        serde_json::to_vec(&json!({"gpu":measured,"rapier":cpu_trace,"impulses":impulses}))?,
    )?;
    Ok(Trial {
        result,
        samples: measured,
        loads,
    })
}

fn reflected(a: &Trial, b: &Trial) -> Value {
    assert_eq!(a.samples.len(), b.samples.len());
    let mut error = [0_f64; 4];
    let compare = |m: &mut f64, a: &[f64], b: &[f64], signs: &[f64]| {
        for ((x, y), sign) in a.iter().zip(b).zip(signs) {
            *m = m.max((x * sign - y).abs());
        }
    };
    let polar = [1., -1., 1.];
    let axial = [-1., 1., -1.];
    for (((x, y), p), q) in a.samples.iter().zip(&b.samples).zip(&a.loads).zip(&b.loads) {
        assert_eq!(x.0, y.0);
        compare(&mut error[0], &x.1.position, &y.1.position, &polar);
        compare(
            &mut error[1],
            &x.1.orientation,
            &y.1.orientation,
            &[-1., 1., -1., 1.],
        );
        for (i, j) in p.world_impulses.iter().enumerate() {
            compare(&mut error[2], j, &q.world_impulses[i ^ 2], &polar);
        }
        compare(
            &mut error[3],
            &p.angular_impulse,
            &q.angular_impulse,
            &axial,
        );
    }
    assert!(error.iter().all(|&e| e < 1e-10), "{error:?}");
    json!({"ticks":a.samples.len(),"max_position_m":error[0],"max_quaternion_component":error[1],"max_vertex_impulse_ns":error[2],"max_angular_impulse_nms":error[3]})
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().collect::<Vec<_>>();
    if !(3..=4).contains(&args.len()) {
        return Err("expected ADAPTER OUTPUT_DIRECTORY [CASE]".into());
    }
    let adapter = &args[1];
    let dir = Path::new(&args[2]);
    let only = args.get(3);
    fs::create_dir_all(dir)?;
    let mut all = Vec::new();
    let mut trials = std::collections::BTreeMap::new();
    for (name, j, p) in [
        ("center", "0.003 0 0.045", "0.007 0 0.004"),
        ("spin_plus", "0.008 0 0", "0.005 -0.008 -0.002"),
        ("spin_minus", "0.008 0 0", "0.005 0.008 -0.002"),
        ("spin_slick", "0.008 0 0", "0.005 -0.008 -0.002"),
        ("oblique_plus", "0.003 0.0012 0.045", "0.007 0.0015 0.004"),
        (
            "oblique_minus",
            "0.003 -0.0012 0.045",
            "0.007 -0.0015 0.004",
        ),
        ("yaw_plus", "0.003 0.006 0.045", "0.009 0.002 0.004"),
        ("yaw_minus", "0.003 -0.006 0.045", "0.009 -0.002 0.004"),
    ] {
        if only.is_some_and(|v| v != name) {
            continue;
        }
        let mut src = SDF
            .replace("0.008 0 0", j)
            .replace("0.005 -0.008 -0.002", p);
        if name == "spin_slick" {
            src = src.replace("<mu>0.45</mu><mu2>0.45</mu2>", "<mu>0</mu><mu2>0</mu2>");
        }
        let trial = run(adapter, name, &src, dir)?;
        println!("{}", serde_json::to_string_pretty(&trial.result)?);
        all.push(trial.result.clone());
        trials.insert(name, trial);
    }
    if all.is_empty() {
        return Err("unknown release case".into());
    }
    if only.is_none() {
        let pairs = [
            ("spin_plus", "spin_minus"),
            ("oblique_plus", "oblique_minus"),
            ("yaw_plus", "yaw_minus"),
        ];
        let symmetry: serde_json::Map<_, _> = pairs
            .into_iter()
            .map(|(a, b)| (a.to_owned(), reflected(&trials[a], &trials[b])))
            .collect();
        fs::write(
            dir.join("symmetry.json"),
            serde_json::to_vec_pretty(&symmetry)?,
        )?;
    }
    fs::write(
        dir.join("measurements.json"),
        serde_json::to_vec_pretty(&all)?,
    )?;
    Ok(())
}
