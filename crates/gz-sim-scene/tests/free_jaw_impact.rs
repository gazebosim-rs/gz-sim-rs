// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, primary_contact::last_refinement,
};
use std::time::Duration;

const SDF: &str = include_str!("../../../examples/worlds/local_free_jaw_impact.sdf");
const EARLY: &str =
    "<early_face_step_ns>25</early_face_step_ns><early_face_max_steps>4096</early_face_max_steps>";
const TICK_NS: u32 = 1_000_000;
const PARCEL: f64 = 0.20;
const JAW: f64 = 0.08;
const SPEED: f64 = 0.75;
const GAP: f64 = 50e-6;

fn body(sim: &Simulation, name: &str) -> BodyId {
    #[allow(deprecated)]
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id()
}

#[test]
fn free_jaw_and_parcel_resolve_a_centered_plastic_strike_in_both_directions() {
    for sign in [1., -1.] {
        let xml = if sign > 0. {
            SDF.to_owned()
        } else {
            SDF.replace("-0.03905", "0.03905")
                .replace(">0.75 0 0<", ">-0.75 0 0<")
                .replace(">-x<", ">+x<")
        };
        let ideal_ns = GAP / SPEED * 1e9;
        let final_speed = sign * PARCEL * SPEED / (PARCEL + JAW);
        let expected_on_parcel = -JAW * final_speed;
        for (mode, step_ns) in [
            ("early", TICK_NS),
            ("uniform64", TICK_NS),
            ("100ns", 100),
            ("25ns", 25),
        ] {
            let mut scene = xml.clone();
            if step_ns < TICK_NS {
                // Run the ordinary native physics system at its own public timestep.
                let start = scene.find("    <plugin").unwrap();
                let end = start + scene[start..].find("</plugin>").unwrap() + "</plugin>".len();
                scene.replace_range(start..end, "");
                scene = scene.replace(">0.001<", &format!(">{:.9}<", f64::from(step_ns) * 1e-9));
            } else if mode == "uniform64" {
                scene = scene.replace(EARLY, "");
            }
            let mut sim = load_world_with_sdf_step(
                &scene,
                "free-jaw.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )
            .unwrap();
            let bodies = [
                body(&sim, "default::parcel::body"),
                body(&sim, "default::jaw::body"),
            ];
            let initial =
                bodies.map(|b| sim.world().resource::<PhysicsBackend>().state(b).unwrap());
            assert_eq!(initial[0].velocity, [sign * SPEED, 0., 0.]);
            assert_eq!(initial[1].velocity, [0.; 3]);
            let mut reaction_x = 0.;
            let mut first = None;
            for _ in 0..TICK_NS / step_ns {
                sim.step_exact(1).unwrap();
                if step_ns == TICK_NS {
                    let r = last_refinement(&sim).unwrap();
                    assert_eq!(
                        r.interval.native_steps,
                        if mode == "early" { 3808 } else { 64 }
                    );
                    let pair = r.requested_pairs[0];
                    assert_eq!((pair.foot, pair.support), (bodies[0], bodies[1]));
                    reaction_x = pair.normal_impulse_on_foot[0];
                    first = pair.first_positive_end;
                } else {
                    let native = sim
                        .world()
                        .resource::<PhysicsBackend>()
                        .downcast_ref::<MultiFidelityBackend>()
                        .unwrap();
                    for p in native.rigid_contact_patches() {
                        let dir = if p.a == bodies[0] { 1. } else { -1. };
                        for point in &p.points {
                            if point.normal_impulse > 0. {
                                first.get_or_insert(sim.elapsed());
                            }
                            reaction_x += dir * p.normal_on_a[0] * point.normal_impulse;
                        }
                    }
                }
            }
            assert_eq!(sim.elapsed(), Duration::from_millis(1));
            let end = bodies.map(|b| sim.world().resource::<PhysicsBackend>().state(b).unwrap());
            let momentum_x = PARCEL * end[0].velocity[0] + JAW * end[1].velocity[0];
            let first_ns = first.unwrap().as_nanos();
            let v = end.map(|s| s.velocity[0]);
            let spin = end.map(|s| {
                s.angular_velocity
                    .into_iter()
                    .map(|w| w * w)
                    .sum::<f64>()
                    .sqrt()
            });
            // A contact reported at the end of its native solve must enclose geometric closure.
            let hit_step = match mode {
                "early" => 25,
                "uniform64" => TICK_NS / 64,
                _ => step_ns,
            };
            assert_eq!(
                first_ns,
                (ideal_ns / f64::from(hit_step)).ceil() as u128 * u128::from(hit_step)
            );
            let flight_s = GAP / SPEED;
            let coast = final_speed * (sim.elapsed().as_secs_f64() - flight_s);
            let expected_x = [
                initial[0].position[0] + sign * SPEED * flight_s + coast,
                coast,
            ];
            let x_error = [0, 1].map(|i| end[i].position[0] - expected_x[i]);
            println!(
                "{}",
                serde_json::json!({
                    "sign": sign,
                    "mode": mode,
                    "first_loaded_end_ns": first_ns,
                    "signed_impulse_Ns": reaction_x,
                    "axial_velocity_mps": v,
                    "angular_speed_rad_s": spin,
                    "positions_m": end.map(|s| s.position),
                    "position_error_m": x_error,
                    "total_momentum_Ns": momentum_x,
                    "impact_time_error_ns": first_ns as f64 - ideal_ns,
                    "impulse_error_Ns": reaction_x - expected_on_parcel,
                })
            );
            assert!((reaction_x - expected_on_parcel).abs() < 1e-11);
            for i in 0..2 {
                assert!((v[i] - final_speed).abs() < 1e-10);
                assert!(x_error[i].abs() < 1e-9 && spin[i] < 1e-9);
                assert_eq!(end[i].velocity[1..], [0., 0.]);
            }
            assert!((momentum_x - sign * PARCEL * SPEED).abs() < 1e-10);
            assert!((PARCEL * (v[0] - sign * SPEED) - reaction_x).abs() < 1e-10);
            assert!((JAW * v[1] + reaction_x).abs() < 1e-10);
        }
    }
}
