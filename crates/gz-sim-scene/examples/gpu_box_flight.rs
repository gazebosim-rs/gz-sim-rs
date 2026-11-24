// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a true SDF pickup and spatial release with a separate Euler-top integration.
use glam::{DQuat, DVec3, DVec4};
use gz_physics_gpu::{CarryGripperBatch, simulation::BatchTiming};
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_on_adapter, reset_loaded_world};
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path, time::Instant};
const SDF: &str = include_str!("../../../examples/gpu_box_flight.sdf");
const MASS: f64 = 0.08;
const OFFSET: DVec3 = DVec3::new(0.005, 0., -0.002);
const I: DVec3 = DVec3::new(0.00001450666666666667, 0.00002602666666666667, 0.0000192);
struct Reference {
    p: DVec3,
    v: DVec3,
    q: DVec4,
    w: DVec3,
    l: DVec3,
    energy: f64,
}
impl Reference {
    fn new(l: &gz_physics_gpu::CarryGripperLane, impulse: DVec3, point: DVec3) -> Self {
        let h = l.hand;
        let q = DQuat::from_rotation_y(-h.angle);
        let spin = DVec3::new(0., -h.spin, 0.);
        let r = q * OFFSET;
        let w = q.inverse() * spin + (point - OFFSET).cross(q.inverse() * impulse) / I;
        Self {
            p: DVec3::new(h.ball[0], 0., h.ball[1]) + r,
            v: DVec3::new(h.velocity[0], 0., h.velocity[1]) + spin.cross(r) + impulse / MASS,
            q: DVec4::from_array(q.to_array()),
            w,
            l: q * (I * w),
            energy: 0.5 * w.dot(I * w),
        }
    }
    fn advance(&mut self) {
        // Euler's body-axis equations; independent of the device's world-L midpoint method.
        let slope = |q: DVec4, w: DVec3| {
            let qdot = DQuat::from_array(q.to_array()) * DQuat::from_xyzw(w.x, w.y, w.z, 0.);
            (
                DVec4::from_array(qdot.to_array()) * 0.5,
                -w.cross(I * w) / I,
            )
        };
        let dt = 0.001 / 32.;
        for _ in 0..32 {
            let (q, w) = (self.q, self.w);
            let (a, b) = slope(q, w);
            let (c, d) = slope(q + a * (dt / 2.), w + b * (dt / 2.));
            let (e, f) = slope(q + c * (dt / 2.), w + d * (dt / 2.));
            let (g, h) = slope(q + e * dt, w + f * dt);
            self.q = (q + (a + 2. * c + 2. * e + g) * (dt / 6.)).normalize();
            self.w = w + (b + 2. * d + 2. * f + h) * (dt / 6.);
        }
    }
}
fn visual_frame(tick: usize, batch: &CarryGripperBatch) -> Value {
    let lane = &batch.lanes()[0];
    let h = lane.hand;
    let pose = |p: DVec3, q: DQuat| json!({"translation":p.to_array(),"rotation":q.to_array()});
    let fixed = |x, y, z| pose(DVec3::new(x, y, z), DQuat::IDENTITY);
    let (p, q) = batch.free_box_link().map_or(
        (
            DVec3::new(h.ball[0], 0., h.ball[1]),
            DQuat::from_rotation_y(-h.angle),
        ),
        |b| (DVec3::from(b.position), DQuat::from_array(b.orientation)),
    );
    let [z, left, right] = h.position;
    json!({"time_ns":tick*1_000_000,"poses":{
        "floor":fixed(0.13,0.,-0.009),"track_guide":fixed(0.105,0.,0.263),"shelf":fixed(lane.shelf_center[0],0.,lane.shelf_center[1]),
        "cart":fixed(lane.track,0.,0.25),"guide":fixed(lane.track,0.,0.177),"carriage":fixed(lane.track,0.,z+0.08),
        "left":fixed(left,0.,z),"right":fixed(right,0.,z),"left_stem":fixed(left,0.,z+0.044),"right_stem":fixed(right,0.,z+0.044),
        "sphere":pose(p,q),"marker":pose(p+q*DVec3::new(0.014,0.015,0.),q)}})
}
fn run(
    adapter: &str,
    source: &str,
    impulse: DVec3,
    point: DVec3,
    path: &Path,
) -> Result<Value, Box<dyn Error>> {
    let load = || {
        load_world_on_adapter(
            source,
            "gpu_box_flight.sdf",
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
    let last_safe = (1..1500)
        .find(|_| probe.step_exact(1).is_err())
        .ok_or("expected a finite contact boundary")?
        - 1;
    if last_safe < 741
        || !matches!(
            probe
                .fault()
                .and_then(|e| e.downcast_ref::<gz_physics_gpu::GpuError>()),
            Some(gz_physics_gpu::GpuError::SpatialContactBoundary)
        )
    {
        return Err(format!("flight stopped at {last_safe} ms: {:?}", probe.fault()).into());
    }
    drop(probe);
    let mut sim = load()?;
    let mut trace = vec![visual_frame(0, sim.world().resource::<CarryGripperBatch>())];
    let mut baseline = Vec::new();
    let mut repetitions = Vec::new();
    for epoch in 0..2 {
        if epoch != 0 {
            reset_loaded_world(&mut sim, |_| ()).map_err(|e| format!("{e:?}"))?;
        }
        let mut reference = None;
        let mut errors = [0_f64; 5];
        let mut costs = Vec::new();
        let mut reads = 0;
        let mut uploads = 0;
        let mut last_link = None;
        for tick in 1..=last_safe {
            if tick == 741 {
                reference = Some(Reference::new(
                    &sim.world().resource::<CarryGripperBatch>().lanes()[0],
                    impulse,
                    point,
                ));
            }
            let t = Instant::now();
            sim.step_exact(1)?;
            let time = t.elapsed().as_secs_f64();
            let batch = sim.world().resource::<CarryGripperBatch>();
            let h = batch.lanes()[0].hand;
            if epoch == 0 {
                trace.push(visual_frame(tick, batch));
            }
            let Some(body) = batch.free_box() else {
                continue;
            };
            let r = reference.as_mut().unwrap();
            let dt = (tick - 740) as f64 * 0.001;
            r.advance();
            let q = DQuat::from_array(body.orientation);
            let native_q = DQuat::from_array(r.q.to_array());
            let p_expected = r.p + r.v * dt - DVec3::Z * (0.5 * 9.81 * dt * dt);
            let actual = batch.free_box_link().unwrap();
            let omega = native_q * r.w;
            let rotation = native_q.inverse() * q;
            let distances = [
                DVec3::from(body.mass_center).distance(p_expected),
                2. * DVec3::new(rotation.x, rotation.y, rotation.z)
                    .length()
                    .atan2(rotation.w.abs()),
                DVec3::from(actual.position).distance(p_expected - native_q * OFFSET),
                DVec3::from(actual.angular).distance(omega),
                (0.5 * DVec3::from(actual.angular).dot(DVec3::from(body.momentum)) - r.energy)
                    .abs(),
            ];
            assert!(DVec3::from(body.momentum).distance(r.l) < 2e-18);
            for i in 0..5 {
                errors[i] = errors[i].max(distances[i]);
            }
            if epoch == 0 {
                baseline.push((body, actual));
            } else {
                assert_eq!(baseline[tick - 741], (body, actual));
            }
            assert_eq!(h.normal_impulse, [0.; 3]);
            let m = sim.world().resource::<BatchTiming>().last.unwrap();
            reads += m.readback_bytes;
            uploads += m.uploaded_bytes;
            costs.push(time);
            last_link = Some(actual);
        }
        assert!(
            costs.len() > 50 && costs.len() < 700,
            "unexpected free interval"
        );
        assert!(
            errors
                .iter()
                .zip([1e-12, 2e-6, 2e-9, 2e-6, 1e-10])
                .all(|(&error, bound)| error < bound),
            "{errors:?}"
        );
        let elapsed = costs.iter().sum::<f64>();
        costs.sort_by(f64::total_cmp);
        let n = costs.len();
        let last = last_link.unwrap();
        repetitions.push(json!({"epoch":epoch,"observed_flight_ticks":n,"last_free_tick":740+n,"final_link_m":last.position,
            "final_quaternion":last.orientation,"max_com_error_m":errors[0],"max_orientation_error_rad":errors[1],
            "max_link_error_m":errors[2],"max_angular_speed_error_rad_s":errors[3],"max_energy_error_j":errors[4],
            "observed_steps_per_second":n as f64/elapsed,"p95_ms":costs[(n*95).div_ceil(100)-1]*1e3,
            "p99_ms":costs[(n*99).div_ceil(100)-1]*1e3,"read_bytes":reads,"uploaded_bytes":uploads}));
    }
    fs::write(path, serde_json::to_vec(&trace)?)?;
    Ok(json!(repetitions))
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let [adapter, out] = args.as_slice() else {
        return Err("expected GPU adapter and output directory".into());
    };
    let out = Path::new(out);
    fs::create_dir_all(out)?;
    let mut result = serde_json::Map::new();
    for (name, j, p) in [
        ("eccentric", [0.003, 0.027, 0.045], [0.007, 0.001, 0.004]),
        ("center_impulse", [0.003, 0.027, 0.045], OFFSET.to_array()),
        ("no_impulse", [0.; 3], OFFSET.to_array()),
    ] {
        let vector = |v: [f64; 3]| format!("{} {} {}", v[0], v[1], v[2]);
        let source = SDF
            .replace("0.003 0.027 0.045", &vector(j))
            .replace("0.007 0.001 0.004", &vector(p));
        fs::write(out.join(format!("{name}.sdf")), &source)?;
        let r = run(
            adapter,
            &source,
            DVec3::from(j),
            DVec3::from(p),
            &out.join(format!("{name}-poses.json")),
        )?;
        result.insert(name.to_string(), r);
    }
    fs::write(
        out.join("measurements.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&result)?);
    Ok(())
}
