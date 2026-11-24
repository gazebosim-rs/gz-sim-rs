// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the first 3D shelf impact from the same last pre-impact SDF state.
#[path = "../../gz-sim-scene/examples/box_spatial/visual_frame.rs"]
mod visual;
use glam::{DMat3, DVec3};
use gz_physics_gpu::{CarryGripperBatch, GpuError, simulation::BatchTiming};
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_on_adapter, reset_loaded_world};
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path, time::Instant};
const SDF: &str = include_str!("../../../examples/gpu_box_shelf_impact.sdf");
#[path = "support/spatial_box_reference.rs"]
mod reference;
use reference::{Body, I, MASS, OFFSET};
impl Body {
    /// Use the change in witness velocity from one unit impulse and the complete
    /// world inertia matrix; body-axis RK4 continues independently after impact.
    pub fn rebound(&mut self, e: f64) -> (DVec3, DVec3, DVec3, f64, f64) {
        let r = self.lowest().0;
        let rotation = DMat3::from_quat(self.q);
        let inverse = (rotation * DMat3::from_diagonal(I) * rotation.transpose()).inverse();
        let unit_dw = inverse * r.cross(DVec3::Z);
        let witness_speed = |v: DVec3, w: DVec3| (v + w.cross(r)).z;
        let before = witness_speed(self.v, self.q * self.w);
        let dv = witness_speed(DVec3::Z / MASS, unit_dw);
        let j = -(1. + e) * before / dv;
        self.v += DVec3::Z * (j / MASS);
        self.w += self.q.inverse() * (unit_dw * j);
        (
            self.p + r,
            DVec3::Z * j,
            r.cross(DVec3::Z * j),
            before,
            witness_speed(self.v, self.q * self.w),
        )
    }
}
fn run(
    adapter: &str,
    source: &str,
    e: f64,
    control: bool,
    path: &Path,
) -> Result<Value, Box<dyn Error>> {
    let load = || {
        load_world_on_adapter(
            source,
            "gpu_box_shelf_impact.sdf",
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
    let stop = (1_usize..3500)
        .find(|_| probe.step_exact(1).is_err())
        .ok_or("missing finite contact boundary")?;
    assert!(matches!(
        probe.fault().and_then(|e| e.downcast_ref::<GpuError>()),
        Some(GpuError::SpatialContactBoundary)
    ));
    drop(probe);
    let mut sim = load()?;
    let sdf = sdformat::load_str(source, "gpu_box_shelf_impact.sdf", &sdformat::NoResources)?;
    let mut frames = vec![gz_sim_cli::primitive_frame(&sim, &sdf, "default", 1)?];
    let mut outputs = vec![];
    let mut trace = vec![visual::visual_frame(
        0,
        sim.world().resource::<CarryGripperBatch>(),
    )];
    let mut golden = vec![];
    for epoch in 0..2 {
        if epoch > 0 {
            reset_loaded_world(&mut sim, |_| ()).map_err(|e| format!("{e:?}"))?;
        }
        let mut cpu = None;
        let mut first = None;
        let mut report = None;
        let mut last = 0;
        let mut errors = [0_f64; 4];
        let mut costs = vec![];
        let mut post = 0;
        for tick in 1..stop {
            let matched = sim.world().resource::<CarryGripperBatch>().free_box();
            let wall = Instant::now();
            sim.step_exact(1)?;
            let elapsed = wall.elapsed().as_secs_f64();
            let batch = sim.world().resource::<CarryGripperBatch>();
            if epoch == 0 {
                if tick % 80 == 0
                    || (1980..2120).contains(&tick) && tick % 12 == 0
                    || tick >= stop.saturating_sub(27)
                {
                    frames.push(gz_sim_cli::primitive_frame(
                        &sim,
                        &sdf,
                        "default",
                        frames.len() as u64 + 1,
                    )?);
                }
                trace.push(visual::visual_frame(tick, batch));
            }
            last = tick;
            let Some(s) = batch.free_box() else {
                continue;
            };
            let receipt = batch.shelf_impact();
            if epoch == 0 {
                golden.push((s, receipt));
            } else {
                assert_eq!(golden[tick - 2041], (s, receipt));
            }
            costs.push(elapsed);
            if let Some(h) = receipt.filter(|h| h.tick == tick as u64) {
                assert!(first.is_none());
                first = Some(h);
                cpu = Some(Body::from_gpu(matched.unwrap()));
            }
            let Some(b) = cpu.as_mut() else { continue };
            for i in 0..32 {
                let dt = 0.001 / 32.;
                let end = b.free(dt);
                if report.is_none() && end.lowest().1 <= 0. {
                    let (mut lo, mut hi) = (0., dt);
                    for _ in 0..32 {
                        let mid = (lo + hi) / 2.;
                        if b.free(mid).lowest().1 <= 0. {
                            hi = mid;
                        } else {
                            lo = mid;
                        }
                    }
                    *b = b.free(hi);
                    let (point, j, angular, before, after) = b.rebound(e);
                    let t = i as f64 * dt + hi;
                    let h = first.unwrap();
                    let dt_error = (h.offset_s - t).abs();
                    let j_error = j.distance(h.linear_impulse.into());
                    let l_error = angular.distance(h.angular_impulse.into());
                    let p_error = point.distance(h.point_m.into());
                    assert!(
                        dt_error < 1e-9 && j_error < 2e-8 && l_error < 2e-9 && p_error < 1e-8,
                        "{dt_error} {j_error} {l_error} {p_error}"
                    );
                    let m = sim.world().resource::<BatchTiming>().last.unwrap();
                    report = Some(
                        json!({"tick":tick,"time_s":(tick-1)as f64*0.001+h.offset_s,"arrival_error_s":dt_error,
                        "point_m":h.point_m,"point_error_m":p_error,"signed_linear_ns":h.linear_impulse,"signed_angular_nms":h.angular_impulse,
                        "linear_impulse_error_ns":j_error,"angular_impulse_error_nms":l_error,
                        "incoming_corner_speed_m_s":before,"outgoing_corner_speed_m_s":after,"step_ms":elapsed*1000.,
                        "device_ns":m.gpu_dispatch.map(|d|d.as_nanos() as u64),"uploaded_bytes":m.uploaded_bytes,"read_bytes":m.readback_bytes}),
                    );
                    *b = b.free(dt - hi);
                } else {
                    *b = end;
                }
            }
            let actual = Body::from_gpu(s);
            let dq = b.q.inverse() * actual.q;
            let values = [
                (actual.p - actual.q * OFFSET).distance(b.p - b.q * OFFSET),
                2. * DVec3::new(dq.x, dq.y, dq.z).length().atan2(dq.w.abs()),
                actual.v.distance(b.v),
                (actual.q * actual.w).distance(b.q * b.w),
            ];
            for i in 0..4 {
                errors[i] = errors[i].max(values[i]);
            }
            post += 1;
        }
        if control {
            assert!(first.is_none() && last > 2100);
        } else {
            assert!(
                report.is_some() && post > 1 && errors[0] < 1e-6 && errors[1] < 1e-4,
                "{last} {report:?} {post} {errors:?}"
            );
        }
        let sum = costs.iter().sum::<f64>();
        costs.sort_by(f64::total_cmp);
        outputs.push(json!({"epoch":epoch,"impact":report,"last_safe_tick":last,"observed_spatial_ticks":costs.len(),"post_public_ticks":post,
            "max_link_error_m":errors[0],"max_angle_error_rad":errors[1],"max_velocity_error_m_s":errors[2],"max_spin_error_rad_s":errors[3],
            "observed_steps_s":costs.len() as f64/sum,"p95_ms":1000.*costs[(costs.len()*95).div_ceil(100)-1]}));
    }
    let archive = path.with_file_name(
        path.file_name()
            .unwrap()
            .to_str()
            .unwrap()
            .replace("-poses.json", "-replay"),
    );
    gz_sim_cli::export_archive(&archive, Default::default(), frames)?;
    fs::write(path, serde_json::to_vec(&trace)?)?;
    Ok(json!(outputs))
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let [adapter, dir] = args.as_slice() else {
        return Err("expected GPU adapter and output directory".into());
    };
    let dir = Path::new(dir);
    fs::create_dir_all(dir)?;
    let mut all = serde_json::Map::new();
    for (name, source, e, control) in [
        ("corner", SDF.to_owned(), 0.65, false),
        (
            "lower_restitution",
            SDF.replace("shelf_restitution=\"0.65\"", "shelf_restitution=\"0.35\""),
            0.35,
            false,
        ),
        (
            "shelf_miss",
            SDF.replace("0.18 0 0.007 0 0 0", "-0.18 0 0.007 0 0 0"),
            0.65,
            true,
        ),
    ] {
        fs::write(dir.join(format!("{name}.sdf")), &source)?;
        let results = run(
            adapter,
            &source,
            e,
            control,
            &dir.join(format!("{name}-poses.json")),
        )?;
        all.insert(name.to_owned(), results);
    }
    fs::write(
        dir.join("measurements.json"),
        serde_json::to_vec_pretty(&all)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&all)?);
    Ok(())
}
