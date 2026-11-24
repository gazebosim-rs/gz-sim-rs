// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A physical two-corner shelf impact compared with an independent iterative impulse solve.
#[path = "support/spatial_box_reference.rs"]
mod reference;
#[path = "../../gz-sim-scene/examples/box_spatial/visual_frame.rs"]
mod visual;
use glam::{DMat3, DVec3};
use gz_physics_gpu::{CarryGripperBatch, GpuError};
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_on_adapter, reset_loaded_world};
use reference::{Body, I, MASS, OFFSET, TOP};
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path, time::Instant};
const SDF: &str = include_str!("../../../examples/gpu_box_shelf_edge.sdf");
const E: f64 = 0.65;
const DT: f64 = 0.001 / 64.;
#[derive(Clone, Copy)]
struct Arrival {
    t: f64,
    local: DVec3,
    body: Body,
}
fn roots(initial: Body) -> Vec<Arrival> {
    let mut crossings = Vec::new();
    for x in [-0.024, 0.024] {
        for y in [-0.012, 0.012] {
            for z in [-0.020, 0.020] {
                let local = DVec3::new(x, y, z) - OFFSET;
                let gap = |b: Body| b.p.z + (b.q * local).z - TOP;
                let mut b = initial;
                for i in 0..128 {
                    let end = b.free(DT);
                    if gap(b) > 0. && gap(end) <= 0. {
                        let (mut lo, mut hi) = (0., DT);
                        for _ in 0..38 {
                            let mid = (lo + hi) / 2.;
                            if gap(b.free(mid)) > 0. {
                                lo = mid;
                            } else {
                                hi = mid;
                            }
                        }
                        crossings.push(Arrival {
                            t: i as f64 * DT + hi,
                            local,
                            body: b.free(hi),
                        });
                        break;
                    }
                    b = end;
                }
            }
        }
    }
    crossings.sort_by(|a, b| a.t.total_cmp(&b.t));
    crossings
}
struct ReferenceImpact {
    body: Body,
    j: [f64; 2],
    points: [DVec3; 2],
    angular: DVec3,
    before: [f64; 2],
    after: [f64; 2],
}
fn rebound(arrivals: &[Arrival]) -> ReferenceImpact {
    let mut body = arrivals[0].body;
    let mut arms = [body.q * arrivals[0].local, body.q * arrivals[1].local];
    // Reference ordering is world coordinate ordering, independent of the device's corner enumeration.
    arms.sort_by(|a, b| a.y.total_cmp(&b.y).then_with(|| a.x.total_cmp(&b.x)));
    let points = arms.map(|r| body.p + r);
    let rotation = DMat3::from_quat(body.q);
    let inverse = (rotation * DMat3::from_diagonal(I) * rotation.transpose()).inverse();
    let unit_dw = arms.map(|r| inverse * r.cross(DVec3::Z));
    let before = arms.map(|r| (body.v + (body.q * body.w).cross(r)).z);
    let mut j = [0.; 2];
    let mut w = body.q * body.w;
    // Project normal rows iteratively, reversing order each sweep to test the
    // simultaneous response independently from the device's closed linear solve.
    for sweep in 0..128 {
        for k in 0..2 {
            let i = if sweep % 2 == 0 { k } else { 1 - k };
            let current = (body.v + w.cross(arms[i])).z;
            let response = (DVec3::Z / MASS + unit_dw[i].cross(arms[i])).z;
            let new = (j[i] + (-E * before[i] - current) / response).max(0.);
            let delta = new - j[i];
            j[i] = new;
            body.v += DVec3::Z * (delta / MASS);
            w += unit_dw[i] * delta;
        }
    }
    let after = arms.map(|r| (body.v + w.cross(r)).z);
    body.w = body.q.inverse() * w;
    let angular = arms[0].cross(DVec3::Z * j[0]) + arms[1].cross(DVec3::Z * j[1]);
    ReferenceImpact {
        body,
        j,
        points,
        angular,
        before,
        after,
    }
}
fn run(
    adapter: &str,
    src: &str,
    name: &str,
    dir: &Path,
    expect_hit: bool,
) -> Result<Value, Box<dyn Error>> {
    let load = || {
        load_world_on_adapter(
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
        )
    };
    let mut probe = load()?;
    let stop = (1_usize..3000)
        .find(|_| probe.step_exact(1).is_err())
        .ok_or("expected finite boundary")?;
    assert!(matches!(
        probe.fault().and_then(|e| e.downcast_ref::<GpuError>()),
        Some(GpuError::SpatialContactBoundary)
    ));
    let batch = probe.world().resource::<CarryGripperBatch>();
    let impact_tick = batch.shelf_impact().map(|h| h.tick as usize);
    assert_eq!(impact_tick.is_some(), expect_hit);
    let cut = impact_tick.unwrap_or(stop) - 1;
    drop(probe);
    let sdf = sdformat::load_str(src, name, &sdformat::NoResources)?;
    let mut sim = load()?;
    let mut frames = vec![gz_sim_cli::primitive_frame(&sim, &sdf, "default", 1)?];
    let mut trace = vec![visual::visual_frame(
        0,
        sim.world().resource::<CarryGripperBatch>(),
    )];
    let mut golden = vec![];
    let mut outputs = vec![];
    for epoch in 0..2 {
        if epoch > 0 {
            reset_loaded_world(&mut sim, |_| ()).map_err(|e| format!("{e:?}"))?;
        }
        let (mut report, mut native, mut fallback) = (None, None, None);
        let mut costs = vec![];
        let mut errors = [0_f64; 4];
        for tick in 1..stop {
            let wall = Instant::now();
            sim.step_exact(1)?;
            let elapsed = wall.elapsed().as_secs_f64();
            let b = sim.world().resource::<CarryGripperBatch>();
            if epoch == 0 {
                if tick % 80 == 0
                    || (1980..2120).contains(&tick) && tick % 12 == 0
                    || tick >= cut - 15
                {
                    frames.push(gz_sim_cli::primitive_frame(
                        &sim,
                        &sdf,
                        "default",
                        frames.len() as u64 + 1,
                    )?);
                }
                trace.push(visual::visual_frame(tick, b));
            }
            let Some(state) = b.free_box() else {
                continue;
            };
            costs.push(elapsed);
            let witnesses = b.shelf_witnesses().to_vec();
            if epoch == 0 {
                golden.push((state, witnesses.clone()));
            } else {
                assert_eq!((state, witnesses.clone()), golden[tick - 2041]);
            }
            if tick == cut {
                let seed = Body::from_gpu(state);
                assert!(seed.lowest().1 > 0.);
                let arrivals = roots(seed);
                assert!(arrivals.len() >= 2);
                let separation = arrivals[1].t - arrivals[0].t;
                let order = arrivals[..2]
                    .iter()
                    .map(|a| (a.local + OFFSET).to_array())
                    .collect::<Vec<_>>();
                fallback = Some(
                    json!({"cut_ms":cut,"unconstrained_arrival_separation_s":separation,"prospective_corner_order_m":order,
                    "first_arrival_time_s":cut as f64*0.001+arrivals[0].t}),
                );
                if expect_hit {
                    assert!(separation < 1.1e-10, "{name} {separation}");
                    native = Some((arrivals[0].t, rebound(&arrivals), state));
                } else {
                    assert!(separation > 1e-10, "{name} {}", separation);
                }
            }
            if let Some((tau, cpu, seed)) = native.as_mut()
                && tick > cut
            {
                let mut dt = 0.001;
                if tick == cut + 1 {
                    let h = b.shelf_impact().unwrap();
                    assert_eq!(witnesses.len(), 2);
                    let mut rows = witnesses;
                    rows.sort_by(|a, b| {
                        a.point_m[1]
                            .total_cmp(&b.point_m[1])
                            .then_with(|| a.point_m[0].total_cmp(&b.point_m[0]))
                    });
                    let points = [rows[0].point_m, rows[1].point_m];
                    let j = [rows[0].normal_impulse, rows[1].normal_impulse];
                    let point_errors = std::array::from_fn::<_, 2, _>(|i| {
                        DVec3::from(rows[i].point_m).distance(cpu.points[i])
                    });
                    let row_errors = std::array::from_fn::<_, 2, _>(|i| (j[i] - cpu.j[i]).abs());
                    let momentum = DVec3::from(state.momentum) - DVec3::from(seed.momentum);
                    let linear = (DVec3::from(state.center_velocity)
                        - DVec3::from(seed.center_velocity)
                        + DVec3::Z * 0.00981)
                        * MASS;
                    let residual = linear.distance(DVec3::Z * (j[0] + j[1]));
                    let torque_error = momentum.distance(cpu.angular);
                    let center = DVec3::from(seed.mass_center)
                        + DVec3::from(seed.center_velocity) * h.offset_s
                        - DVec3::Z * (4.905 * h.offset_s * h.offset_s);
                    let moment = rows
                        .iter()
                        .map(|row| {
                            (DVec3::from(row.point_m) - center).cross(DVec3::Z * row.normal_impulse)
                        })
                        .sum::<DVec3>();
                    let angular_balance = momentum.distance(moment);
                    assert!(angular_balance < 1e-15 && cpu.j.iter().all(|j| *j >= 0.));
                    assert!((0..2).all(
                        |i| (cpu.after[i] + E * cpu.before[i]).abs() < 1e-12 || cpu.j[i] == 0.
                    ));
                    assert!(
                        j.iter().all(|x| *x >= 0.)
                            && row_errors.iter().all(|x| *x < 3e-8)
                            && point_errors.iter().all(|x| *x < 2e-8)
                    );
                    assert!(
                        residual < 1e-15 && torque_error < 2e-9 && (*tau - h.offset_s).abs() < 2e-9
                    );
                    report = Some(
                        json!({"tick":tick,"offset_s":h.offset_s,"arrival_error_s":(*tau-h.offset_s).abs(),"point_m":points,
                        "normal_impulses_ns":j,"reference_impulses_ns":cpu.j,"row_errors_ns":row_errors,"point_errors_m":point_errors,
                        "reference_incoming_m_s":cpu.before,"reference_rebound_m_s":cpu.after,"linear_ns":h.linear_impulse,
                        "angular_nms":h.angular_impulse,"linear_balance_error_ns":residual,"angular_balance_error_nms":angular_balance,"angular_reference_error_nms":torque_error,"step_ms":elapsed*1000.}),
                    );
                    dt -= *tau;
                }
                let mut remaining = dt;
                while remaining > 0. {
                    let h = remaining.min(DT);
                    cpu.body = cpu.body.free(h);
                    remaining -= h;
                }
                let cur = Body::from_gpu(state);
                let dq = cpu.body.q.inverse() * cur.q;
                let delta = [
                    (cur.p - cur.q * OFFSET).distance(cpu.body.p - cpu.body.q * OFFSET),
                    2. * DVec3::new(dq.x, dq.y, dq.z).length().atan2(dq.w.abs()),
                    cur.v.distance(cpu.body.v),
                    (cur.q * cur.w).distance(cpu.body.q * cpu.body.w),
                ];
                for i in 0..4 {
                    errors[i] = errors[i].max(delta[i]);
                }
            }
        }
        assert!(errors[0] < 1e-6 && errors[1] < 1e-4);
        let sum = costs.iter().sum::<f64>();
        costs.sort_by(f64::total_cmp);
        outputs.push(json!({"epoch":epoch,"first_impact":report,"incoming":fallback.unwrap(),"last_safe_tick":stop-1,"post_ticks":if expect_hit{stop-1-cut}else{0},
            "max_post_link_error_m":errors[0],"max_post_angle_error_rad":errors[1],"max_post_com_speed_error_m_s":errors[2],
            "max_post_spin_error_rad_s":errors[3],"observed_steps_s":costs.len() as f64/sum,"p95_ms":1000.*costs[(costs.len()*95).div_ceil(100)-1]}));
    }
    gz_sim_cli::export_archive(
        &dir.join(format!("{name}-replay")),
        Default::default(),
        frames,
    )?;
    fs::write(
        dir.join(format!("{name}-poses.json")),
        serde_json::to_vec(&trace)?,
    )?;
    Ok(json!(outputs))
}
fn mirror(dir: &Path, plus: &str, minus: &str) -> Result<Value, Box<dyn Error>> {
    let poses = |name: &str| -> Result<Vec<Value>, Box<dyn Error>> {
        Ok(serde_json::from_slice(&fs::read(
            dir.join(format!("{name}-poses.json")),
        )?)?)
    };
    let a = poses(plus)?;
    let b = poses(minus)?;
    assert_eq!(a.len(), b.len());
    let mut position = 0_f64;
    let mut orientation = 0_f64;
    for (x, y) in a.iter().zip(&b) {
        let get =
            |frame: &Value, key: &str, i: usize| frame["poses"]["sphere"][key][i].as_f64().unwrap();
        assert_eq!(x["time_ns"], y["time_ns"]);
        for (i, sign) in [1., -1., 1.].into_iter().enumerate() {
            position =
                position.max((sign * get(x, "translation", i) - get(y, "translation", i)).abs());
        }
        for (i, sign) in [-1., 1., -1., 1.].into_iter().enumerate() {
            orientation =
                orientation.max((sign * get(x, "rotation", i) - get(y, "rotation", i)).abs());
        }
    }
    assert!(position < 1e-12 && orientation < 1e-11);
    Ok(
        json!({"frames":a.len(),"max_position_component_m":position,"max_quaternion_component":orientation}),
    )
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let [adapter, dir] = args.as_slice() else {
        return Err("expected GPU adapter and output directory".into());
    };
    let dir = Path::new(dir);
    fs::create_dir_all(dir)?;
    let mut result = serde_json::Map::new();
    // The lateral release is centered along X so the long edge reaches the shelf together.
    // Its 5 mm mass offset produces unequal normal loads along the 48 mm edge.
    for (name, impulse, point, accept) in [
        ("edge", "0.003 0 0.045", "0.007 0 0.004", true),
        (
            "near_plus",
            "0.003 0 0.045",
            "0.007 0.0000000000008 0.004",
            true,
        ),
        (
            "near_minus",
            "0.003 0 0.045",
            "0.007 -0.0000000000008 0.004",
            true,
        ),
        (
            "split_plus",
            "0.003 0 0.045",
            "0.007 0.0000001 0.004",
            false,
        ),
        (
            "split_minus",
            "0.003 0 0.045",
            "0.007 -0.0000001 0.004",
            false,
        ),
        (
            "lateral_plus",
            "0 0.017 0.045",
            "0.004999929122238637 0.002 0.004",
            true,
        ),
        (
            "lateral_minus",
            "0 -0.017 0.045",
            "0.004999929122238637 -0.002 0.004",
            true,
        ),
    ] {
        let src = SDF
            .replace("0.003 0 0.045", impulse)
            .replace("0.007 0 0.004", point);
        fs::write(dir.join(format!("{name}.sdf")), &src)?;
        result.insert(name.into(), run(adapter, &src, name, dir, accept)?);
    }
    let mut symmetry = serde_json::Map::new();
    for (a, b) in [
        ("near_plus", "near_minus"),
        ("split_plus", "split_minus"),
        ("lateral_plus", "lateral_minus"),
    ] {
        let mut measurement = mirror(dir, a, b)?;
        if let (Some(plus), Some(minus)) = (
            result[a][0]["first_impact"].as_object(),
            result[b][0]["first_impact"].as_object(),
        ) {
            let component = |v: &Value, i: usize| v[i].as_f64().unwrap();
            let change = (0..3)
                .map(|i| {
                    ([-1., 1., -1.][i] * component(&plus["angular_nms"], i)
                        - component(&minus["angular_nms"], i))
                    .abs()
                })
                .fold(0_f64, f64::max);
            let position = |v: &Value, i: usize| {
                DVec3::from_array(std::array::from_fn(|k| component(&v[i], k)))
            };
            let mirror = DVec3::new(1., -1., 1.);
            let mut force = 0_f64;
            let mut gap = 0_f64;
            let mut matched = [false; 2];
            for i in 0..2 {
                let reflected = position(&plus["point_m"], i) * mirror;
                let distances = [0, 1].map(|k| position(&minus["point_m"], k).distance(reflected));
                let other = usize::from(distances[1] < distances[0]);
                assert!(!matched[other]);
                matched[other] = true;
                gap = gap.max(distances[other]);
                force = force.max(
                    (component(&plus["normal_impulses_ns"], i)
                        - component(&minus["normal_impulses_ns"], other))
                    .abs(),
                );
            }
            assert!(gap < 1e-12 && force < 1e-12 && change < 1e-14);
            measurement["max_witness_reflection_error_m"] = json!(gap);
            measurement["max_normal_impulse_reflection_error_ns"] = json!(force);
            measurement["max_angular_impulse_reflection_error_nms"] = json!(change);
        }
        symmetry.insert(a.to_string(), measurement);
    }
    fs::write(
        dir.join("symmetry.json"),
        serde_json::to_vec_pretty(&symmetry)?,
    )?;
    fs::write(
        dir.join("measurements.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&result)?);
    Ok(())
}
