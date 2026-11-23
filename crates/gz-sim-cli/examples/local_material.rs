// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sliding spheres transition to rolling on three authored local Coulomb surfaces.
use glam::DVec3;
use gz_physics::{BodyId, BodyState, DynamicBackend, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, EntityKind, Name, PhysicsEngine, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    load_world_with_runtime, reset_loaded_world,
};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_local_friction.sdf");
const GRAVITY: f64 = 9.81;
const SPEED: f64 = 2.1;
const RADIUS: f64 = 0.1;
const INERTIA: f64 = 0.004;
const N: u64 = 1200;
fn expected(mu: f64, t: f64) -> [f64; 3] {
    let a = mu * GRAVITY;
    let slide = if a > 0. {
        t.min(SPEED / (a * (1. + RADIUS * RADIUS / INERTIA)))
    } else {
        t
    };
    let v = SPEED - a * slide;
    let w = a * RADIUS * slide / INERTIA;
    [
        -0.8 + SPEED * slide - 0.5 * a * slide * slide + v * (t - slide),
        v,
        w,
    ]
}
fn id(sim: &Simulation, link: &str) -> Result<BodyId> {
    #[allow(deprecated)]
    Ok(sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == link))
        .and_then(|e| e.get::<BodyHandle>())
        .ok_or("missing body")?
        .id())
}
fn frame(sim: &Simulation, ids: &[BodyId; 3]) -> [BodyState; 3] {
    ids.map(|id| {
        sim.world()
            .resource::<PhysicsBackend>()
            .state(id)
            .expect("registered sphere")
    })
}
/// Tangential velocity at the native solver's two lever arms after the solve.
fn contact_slip(sim: &Simulation, ids: [BodyId; 3], deck: BodyId) -> [Option<f64>; 3] {
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    let mut out = [None; 3];
    for p in native.rigid_contact_patches() {
        let (ball, sign) = if p.a == deck {
            (p.b, -1.)
        } else if p.b == deck {
            (p.a, 1.)
        } else {
            continue;
        };
        let Some(i) = ids.iter().position(|&b| b == ball) else {
            continue;
        };
        let states = [native.state(p.a).unwrap(), native.state(p.b).unwrap()];
        for q in p.points.into_iter().filter(|q| q.normal_impulse > 0.) {
            let at = |s: BodyState, r: [f64; 3]| {
                DVec3::from(s.velocity) + DVec3::from(s.angular_velocity).cross(DVec3::from(r))
            };
            out[i] = Some(sign * (at(states[0], q.lever_a) - at(states[1], q.lever_b)).x);
        }
    }
    out
}

fn run(sim: &mut Simulation, mut poses: Option<&mut Vec<Value>>) -> Result<Value> {
    let ids = [
        id(sim, "default::ice::ball")?,
        id(sim, "default::medium::ball")?,
        id(sim, "default::grip::ball")?,
    ];
    let dt = sim.step_duration().as_secs_f64();
    let mut traces = vec![];
    let deck = id(sim, "default::runway::deck")?;
    let mut starts = [None; 3];
    let mut native_band = [None; 3];
    let mut nominal_band = [None; 3];
    let mut errors = [[0.0f64; 3]; 3];
    for tick in 0..=N {
        if tick > 0 {
            sim.step_exact(1)?;
        }
        if let Some(frames) = poses.as_mut() {
            #[allow(deprecated)]
            let nodes = sim
                .world()
                .iter_entities()
                .filter_map(|e| {
                    if !matches!(
                        e.get::<EntityKind>()?,
                        EntityKind::Model | EntityKind::Link | EntityKind::Visual
                    ) {
                        return None;
                    }
                    let p = e.get::<WorldPose>()?;
                    Some((
                        e.get::<Name>()?.0.clone(),
                        json!({"translation":p.translation,"rotation":p.rotation}),
                    ))
                })
                .collect::<std::collections::BTreeMap<_, _>>();
            frames
                .push(json!({"step":tick,"time_ns":sim.elapsed().as_nanos() as u64,"poses":nodes}));
        }
        let t = tick as f64 * dt;
        let bodies = frame(sim, &ids);
        let actual = bodies.map(|s| [s.position[0], s.velocity[0], s.angular_velocity[1]]);
        let slip_at_point = contact_slip(sim, ids, deck);
        for (i, mu) in [0., 0.2, 0.6].into_iter().enumerate() {
            let exact = expected(mu, t);
            for k in 0..3 {
                errors[i][k] = errors[i][k].max((actual[i][k] - exact[k]).abs());
            }
            let slip = actual[i][1] - RADIUS * actual[i][2];
            if slip.abs() <= 1e-3 {
                nominal_band[i].get_or_insert(t);
            } else {
                nominal_band[i] = None;
            }
            if slip_at_point[i].is_some_and(|v| v.abs() <= 1e-4) {
                native_band[i].get_or_insert(t);
            } else {
                native_band[i] = None;
            }
            if slip.abs() < 1e-4 {
                starts[i].get_or_insert(t);
            }
        }
        traces.push(json!({"tick":tick,"time_ns":sim.elapsed().as_nanos() as u64,"native_contact_slip_m_s":slip_at_point,"states":bodies.map(|s|json!({"p":s.position,"q":s.orientation,"v":s.velocity,"w":s.angular_velocity})),"analytic_x_v_w":([0.,0.2,0.6].map(|mu|expected(mu,t)))}));
    }
    Ok(
        json!({"first_rolling_s":starts,"native_sustained_0_1mm_s":native_band,"nominal_sustained_1mm_s":nominal_band,"max_abs_x_v_w":errors,"traces":traces}),
    )
}
fn main() -> Result<()> {
    let out = PathBuf::from(std::env::args().nth(1).ok_or("output directory required")?);
    fs::create_dir_all(&out)?;
    for per_point in [false, true] {
        let mut sim = load_world_with_runtime(
            SDF,
            "local-friction.sdf",
            "default",
            &NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::MultiFidelity,
                step: None,
                noise_seed: [0; 32],
                distributed_worker: None,
            },
            WorldRuntimeOptions {
                rapier_per_point_friction: per_point,
                ..Default::default()
            },
        )?;
        let mut poses = vec![];
        let result = run(&mut sim, (!per_point).then_some(&mut poses))?;
        reset_loaded_world(&mut sim, |_| ())?;
        if run(&mut sim, None)? != result {
            return Err("reset velocities, slip or poses did not reproduce".into());
        }
        if !per_point {
            fs::write(out.join("poses.json"), serde_json::to_vec(&poses)?)?;
        }
        fs::write(
            out.join(if per_point {
                "per-point-study.json"
            } else {
                "study.json"
            }),
            serde_json::to_vec(&result)?,
        )?;
        println!(
            "{}",
            json!({"per_point":per_point,"native_sustained_0_1mm_s":result["native_sustained_0_1mm_s"],"nominal_sustained_1mm_s":result["nominal_sustained_1mm_s"],"first_rolling_s":result["first_rolling_s"],"max_abs_x_v_w":result["max_abs_x_v_w"],"ticks":N,"reset_exact":true})
        );
    }
    Ok(())
}
