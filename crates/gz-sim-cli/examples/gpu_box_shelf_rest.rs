// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A free box transitions to sustained shelf contact. A separate full 3D Rapier world
//! starts from the measured release and remains awake throughout the comparison.
#[path = "../../gz-sim-scene/examples/box_spatial/visual_frame.rs"]
mod visual;
use glam::{DQuat, DVec3};
use gz_physics_gpu::{CarryGripperBatch, CarryGripperLane, FreeBoxState};
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_on_adapter, reset_loaded_world};
use rapier3d_f64::prelude::*;
use serde::Serialize;
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path, time::Instant};
const SDF: &str = include_str!("../../../examples/gpu_box_shelf_rest.sdf");
const MOMENTS: DVec3 = DVec3::new(0.00001450666666666667, 0.00002602666666666667, 0.0000192);
const OFFSET: DVec3 = DVec3::new(0.005, 0., -0.002);
const MASS: f64 = 0.08;
const WEIGHT: f64 = 9.81 * MASS;

struct Reference {
    world: PhysicsWorld,
    body: RigidBodyHandle,
    shape: ColliderHandle,
    shelf: ColliderHandle,
}
impl Reference {
    fn new(body: FreeBoxState, l: &CarryGripperLane) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = -9.81 * Vector::Z;
        let p = &mut world.integration_parameters;
        p.dt = 0.001 / 32.;
        p.normalized_allowed_linear_error = 0.;
        p.normalized_prediction_distance = 0.;
        p.normalized_contact_recycle_distance = 0.;
        p.num_solver_iterations = 32;
        p.num_internal_pgs_iterations = 8;
        p.friction_model = FrictionModel::Coulomb;
        // Match the hard unilateral normal rows on the device.
        p.static_contact_softness.natural_frequency = 1e6;
        p.static_contact_softness.damping_ratio = 1.;
        p.normalized_max_corrective_velocity = 0.;
        let material = |c: ColliderBuilder, mu| {
            c.friction(mu)
                .friction_combine_rule(CoefficientCombineRule::Min)
                .restitution(0.)
        };
        let shelf = world.insert_collider(
            material(
                ColliderBuilder::cuboid(l.shelf_half[0], 0.105, l.shelf_half[1])
                    .translation(Vector::new(l.shelf_center[0], 0., l.shelf_center[1])),
                l.shelf_friction,
            ),
            None,
        );
        world.insert_collider(
            material(
                ColliderBuilder::cuboid(1., 1., 0.02)
                    .translation(Vector::Z * (l.hand.ground - 0.02)),
                l.hand.friction[2],
            ),
            None,
        );
        let q = DQuat::from_array(body.orientation);
        let pos = DVec3::from(body.mass_center) - q * OFFSET;
        let omega = q * ((q.inverse() * DVec3::from(body.momentum)) / MOMENTS);
        let id = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(Vector::from_array(pos.to_array()))
                .rotation(Vector::from_array(q.to_scaled_axis().to_array()))
                .can_sleep(false),
        );
        let shape = world.insert_collider(
            material(
                ColliderBuilder::cuboid(0.024, 0.012, 0.020).mass_properties(MassProperties::new(
                    Vector::from_array(OFFSET.to_array()),
                    MASS,
                    Vector::from_array(MOMENTS.to_array()),
                )),
                1.,
            ),
            Some(id),
        );
        world.bodies[id].set_linvel(Vector::from_array(body.center_velocity), true);
        world.bodies[id].set_angvel(Vector::from_array(omega.to_array()), true);
        Self {
            world,
            body: id,
            shape,
            shelf,
        }
    }
    fn advance(&mut self) -> f64 {
        let mut j = 0.;
        for _ in 0..32 {
            self.world.step();
            if let Some(pair) = self.world.narrow_phase.contact_pair(self.shelf, self.shape) {
                let sign = if pair.collider2 == self.shape {
                    1.
                } else {
                    -1.
                };
                for m in pair.solver_manifolds() {
                    // Narrow phase retains historical witnesses when a face replaces
                    // an edge. Read only the points admitted to this contact solve.
                    let normal: f64 = m
                        .data
                        .solver_contacts
                        .iter()
                        .map(|row| m.points[row.contact_indices()[0] as usize].data.impulse)
                        .sum();
                    j += sign * m.data.normal.z * normal;
                }
            }
        }
        j
    }
    fn state(&self, j: f64) -> Sample {
        let b = &self.world.bodies[self.body];
        let origin = b.translation();
        Sample {
            position: origin.to_array(),
            orientation: b.rotation().to_array(),
            center_velocity: b.linvel().to_array(),
            omega: b.angvel().to_array(),
            impulse: j,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
struct Sample {
    position: [f64; 3],
    orientation: [f64; 4],
    center_velocity: [f64; 3],
    omega: [f64; 3],
    impulse: f64,
}
impl Sample {
    fn gpu(b: &CarryGripperBatch) -> Self {
        let l = b.free_box_link().unwrap();
        Self {
            position: l.position,
            orientation: l.orientation,
            center_velocity: b.free_box().unwrap().center_velocity,
            omega: l.angular,
            impulse: b.lanes()[0].shelf_up,
        }
    }
}
fn compare(a: &[(usize, Sample)], b: &[(usize, Sample)]) -> [f64; 4] {
    let mut delta = [0_f64; 4];
    for ((_, g), (_, c)) in a.iter().zip(b) {
        delta[2] = DVec3::from(g.position).distance(c.position.into());
        delta[3] = DQuat::from_array(g.orientation).angle_between(DQuat::from_array(c.orientation));
        delta[0] = delta[0].max(delta[2]);
        delta[1] = delta[1].max(delta[3]);
    }
    delta
}
fn cpu_tick(
    reference: &mut Option<Reference>,
    state: FreeBoxState,
    lane: &CarryGripperLane,
) -> Sample {
    let impulse = match reference {
        Some(cpu) => cpu.advance(),
        None => {
            *reference = Some(Reference::new(state, lane));
            0.
        }
    };
    reference.as_ref().unwrap().state(impulse)
}
fn summarize(samples: &[(usize, Sample)]) -> Value {
    let first = samples
        .iter()
        .find(|(_, s)| s.impulse > 1e-10)
        .map(|(t, _)| t);
    let last = samples.last().unwrap().1;
    let window = &samples[samples.len() - 200..];
    let max = |f: fn(Sample) -> f64| window.iter().map(|(_, s)| f(*s)).fold(0., f64::max);
    let lo = window.first().unwrap().1;
    let mut episodes = Vec::new();
    let mut last_loaded = false;
    for (tick, s) in samples {
        let loaded = s.impulse > 1e-10;
        if loaded != last_loaded {
            episodes.push((*tick, loaded));
        }
        last_loaded = loaded;
    }
    let first_impulse = samples
        .iter()
        .find(|(_, s)| s.impulse > 1e-10)
        .map(|(_, s)| s.impulse);
    let settled = samples
        .windows(40)
        .find(|window| {
            window.iter().all(|(_, s)| {
                (1000. * s.impulse - WEIGHT).abs() < 0.005
                    && DVec3::from(s.center_velocity).length() < 0.001
                    && DVec3::from(s.omega).length() < 0.05
            })
        })
        .map(|w| w[0].0);
    json!({"first_contact_ms":first,"first_up_ns":first_impulse,"begin_40ms_supported_rest_ms":settled,"load_transitions":episodes, "last":last,"last_200_mean_up_n":window.iter().map(|(_,s)|s.impulse).sum::<f64>()/0.2,
        "last_200_max_load_error_n":max(|s|(1000.*s.impulse-WEIGHT).abs()),
        "last_200_max_com_speed_m_s":max(|s|DVec3::from(s.center_velocity).length()),"last_200_max_spin_rad_s":max(|s|DVec3::from(s.omega).length()),
        "last_200_link_drift_m":DVec3::from(last.position).distance(lo.position.into()),"last_200_angle_drift_rad":DQuat::from_array(last.orientation).angle_between(DQuat::from_array(lo.orientation))})
}
fn run(adapter: &str, name: &str, source: &str, dir: &Path) -> Result<Value, Box<dyn Error>> {
    let sdf = sdformat::load_str(source, name, &sdformat::NoResources)?;
    let mut sim = load_world_on_adapter(
        source,
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
    let mut gp = Vec::new();
    let mut cp = Vec::new();
    let mut receipts = Vec::new();
    let mut costs = Vec::new();
    let mut cpu_costs = Vec::new();
    for epoch in 0..2 {
        if epoch != 0 {
            reset_loaded_world(&mut sim, |_| ()).map_err(|e| format!("{e:?}"))?;
        }
        let mut cpu = None;
        let end = if name == "slide" { 4100 } else { 2850 };
        for tick in 1..=end {
            let t = Instant::now();
            sim.step_exact(1)?;
            let elapsed = t.elapsed().as_secs_f64();
            if epoch == 0
                && (tick % 80 == 0
                    || (2150..=2250).contains(&tick) && tick % 2 == 0
                    || tick == end
                    || tick == 2850
                    || (name == "slide" && tick >= 3300 && tick % 20 == 0))
            {
                frames.push(gz_sim_cli::primitive_frame(
                    &sim,
                    &sdf,
                    "default",
                    frames.len() as u64 + 1,
                )?);
            }
            let b = sim.world().resource::<CarryGripperBatch>();
            if epoch == 0 {
                poses.push(visual::visual_frame(tick, b));
            }
            let Some(state) = b.free_box() else { continue };
            let l = b.lanes()[0];
            assert_eq!(
                l.hand.normal_impulse[..2],
                [0.; 2],
                "{name} finger contact {tick}"
            );
            let sample = Sample::gpu(b);
            assert!(l.shelf_up >= -1e-12);
            costs.push(elapsed);
            if epoch == 0 {
                let start = Instant::now();
                let reference = cpu_tick(&mut cpu, state, &l);
                cpu_costs.push(start.elapsed().as_secs_f64());
                gp.push((tick, sample));
                cp.push((tick, reference));
                receipts.push((
                    tick,
                    l.point_impulse[3],
                    l.shelf_tangent,
                    l.hand.normal_impulse[2],
                ));
            } else {
                assert_eq!((tick, sample), gp[tick - 2041]);
                assert_eq!(
                    (
                        tick,
                        l.point_impulse[3],
                        l.shelf_tangent,
                        l.hand.normal_impulse[2]
                    ),
                    receipts[tick - 2041]
                );
            }
        }
    }
    let first = gp[0].1;
    assert!(first.center_velocity[2] > 0.);
    assert!(gp.iter().take(70).all(|(_, s)| s.impulse == 0.));
    let errors = compare(&gp, &cp);
    let g = gp.last().unwrap().1;
    let rate = |ts: &mut Vec<f64>| {
        ts.sort_by(f64::total_cmp);
        json!({"steps_s":ts.len() as f64/ts.iter().sum::<f64>(),"p95_ms":1000.*ts[(ts.len()*95).div_ceil(100)-1]})
    };
    let mut native_balance = 0_f64;
    if name == "rest" || name == "balanced" {
        for pair in cp.windows(2) {
            let impulse =
                MASS * (pair[1].1.center_velocity[2] - pair[0].1.center_velocity[2] + 0.00981);
            native_balance = native_balance.max((impulse - pair[1].1.impulse).abs());
        }
    }
    let mut balance = 0_f64;
    let mut friction_excess = 0_f64;
    for (pair, forces) in gp.windows(2).zip(receipts.iter().skip(1)) {
        let (_, before) = pair[0];
        let (_, after) = pair[1];
        let (tick, edge, tangent, ground) = forces;
        assert!(edge.iter().all(|j| *j >= 0.), "negative contact at {tick}");
        let gravity = -DVec3::Z * 0.00981;
        let dv = DVec3::from(after.center_velocity) - DVec3::from(before.center_velocity);
        // After a floor or shelf side contact, the reported normal no longer necessarily
        // points upwards. Only use the isolated top-surface portion in this impulse check.
        if *ground == 0. && *tick < 2851 {
            balance = balance
                .max((MASS * (dv - gravity) - DVec3::new(*tangent, 0., after.impulse)).length());
            let mu = if name == "rest" { 0.45 } else { 0. };
            friction_excess = friction_excess.max(tangent.abs() - mu * edge.iter().sum::<f64>());
        }
    }
    assert!(
        balance < 1e-12 && friction_excess < 1e-12 && native_balance < 1e-11,
        "{name} {balance} {friction_excess} {native_balance}"
    );
    let before_edge = |samples: &[(usize, Sample)]| summarize(&samples[..2850 - 2041 + 1]);
    let load = |samples: &[(usize, Sample)]| {
        samples
            .iter()
            .rev()
            .take(200)
            .map(|(_, s)| s.impulse)
            .sum::<f64>()
            / 0.2
    };
    let last_contact = |samples: &[(usize, Sample)]| {
        samples
            .iter()
            .rev()
            .find(|(_, s)| s.impulse > 1e-10)
            .map(|(t, _)| *t)
    };
    let landed = gp.iter().position(|(_, s)| s.impulse > 1e-10);
    let first_two_points = receipts
        .iter()
        .find(|(_, js, _, _)| js.iter().all(|j| *j > 1e-10))
        .map(|r| r.0);
    match name {
        "balanced" | "rest" => {
            assert!(landed.is_some() && first_two_points.is_some());
            assert!(gp[landed.unwrap()..].iter().all(|(_, s)| s.impulse >= 0.));
            for row in [&gp, &cp] {
                assert!((load(row) - WEIGHT).abs() < 0.0001);
                assert!(DVec3::from(row.last().unwrap().1.center_velocity).length() < 1e-4);
            }
            // The stationary face spans ±24 mm while its COM lies 5 mm to the right.
            // The two support reactions must balance both gravity and its moment.
            let expected = [
                WEIGHT * (0.024 - 0.005) / 0.048,
                WEIGHT * (0.024 + 0.005) / 0.048,
            ];
            for (_, endpoints, _, _) in receipts.iter().rev().take(200) {
                assert!((1000. * endpoints[0] - expected[0]).abs() < 1e-8);
                assert!((1000. * endpoints[1] - expected[1]).abs() < 1e-8);
            }
            if name == "balanced" {
                assert!(errors[0] < 0.00002 && errors[1] < 0.001 && errors[2] < 0.00001);
            }
        }
        "miss" => {
            assert!(landed.is_none() && last_contact(&cp).is_none());
            assert!(
                receipts
                    .iter()
                    .rev()
                    .take(200)
                    .all(|r| (r.3 - 0.001 * WEIGHT).abs() < 1e-9)
            );
        }
        _ => {
            let slide = &gp[2650 - 2041..2850 - 2041 + 1];
            assert!(
                slide.iter().all(|(_, s)| s.center_velocity[0] > 0.035
                    && (1000. * s.impulse - WEIGHT).abs() < 1e-8)
            );
            assert!(
                last_contact(&gp)
                    .unwrap()
                    .abs_diff(last_contact(&cp).unwrap())
                    < 10
            );
            assert!(load(&gp) == 0. && load(&cp) == 0. && g.position[0] > 0.24);
        }
    }
    let i=landed.map(|k|json!({"tick":gp[k].0,"gpu_total_ns":gp[k].1.impulse,"gpu_endpoints_ns":receipts[k].1,"gpu_tangent_ns":receipts[k].2}));
    let terminal_endpoints = receipts.last().unwrap().1.map(|j| 1000. * j);
    let result = json!({
        "name":name,"terminal_endpoint_forces_n":terminal_endpoints,"first_impulse":i,"first_two_endpoint_tick":first_two_points,
        "max_top_impulse_balance_error_ns":balance,"max_native_vertical_impulse_balance_error_ns":native_balance,"max_coulomb_bound_error_ns":friction_excess,
        "gpu_at_2850":before_edge(&gp),"rapier_at_2850":before_edge(&cp),"gpu":summarize(&gp),"rapier":summarize(&cp),
        "max_link_position_error_m":errors[0],"max_angle_error_rad":errors[1],
        "final_link_position_error_m":errors[2],"final_angle_error_rad":errors[3],
        "gpu_two_epochs":rate(&mut costs),"cpu_reference_32_substeps":rate(&mut cpu_costs)});
    gz_sim_cli::export_archive(
        &dir.join(format!("{name}-replay")),
        Default::default(),
        frames,
    )?;
    fs::write(
        dir.join(format!("{name}-poses.json")),
        serde_json::to_vec(&poses)?,
    )?;
    fs::write(
        dir.join(format!("{name}-trace.json")),
        serde_json::to_vec(&json!({"gpu":gp,"rapier":cp,"endpoint_impulses":receipts}))?,
    )?;
    Ok(result)
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().collect::<Vec<_>>();
    let [_, adapter, target] = args.as_slice() else {
        return Err("expected ADAPTER OUTPUT_DIRECTORY".into());
    };
    let dir = Path::new(target);
    fs::create_dir_all(dir)?;
    // The ordinary SDF compensates the measured horizontal momentum of the opened hand.
    // Restore the original sideways kick for the sliding and rough-shelf challenges.
    let slide = SDF.replace("0.000000354230582637 0 0.045", "0.003 0 0.045");
    let rough = slide.replace("<mu>0</mu><mu2>0</mu2>", "<mu>0.45</mu><mu2>0.45</mu2>");
    let miss = rough.replace("0.18 0 0.007", "-0.18 0 0.007");
    let mut results = Vec::new();
    for (name, src) in [
        ("rest", rough.as_str()),
        ("balanced", SDF),
        ("miss", miss.as_str()),
        ("slide", slide.as_str()),
    ] {
        let result = run(adapter, name, src, dir)?;
        println!("{}", serde_json::to_string_pretty(&result)?);
        results.push(result);
    }
    fs::write(
        dir.join("measurements.json"),
        serde_json::to_vec_pretty(&results)?,
    )?;
    Ok(())
}
