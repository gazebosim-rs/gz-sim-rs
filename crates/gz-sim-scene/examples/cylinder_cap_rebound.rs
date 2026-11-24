// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Drop the same cylinder and capsule at several native contact iteration budgets.
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::entity_by_id;
use gz_sim_scene::{
    BodyHandle, PhysicsEngine, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    load_world_with_runtime, model_state, prepare_model_template, spawn_model_template,
};
use serde_json::{Value, json};
use std::error::Error;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORLD: &str = include_str!("../../../examples/worlds/local_curved_delivery.sdf");

fn drop_at(world: &str, passes: usize, per_point: bool) -> Result<Value> {
    let sdf = sdformat::load_str(world, "delivery.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_runtime(
        world,
        "delivery.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            step: None,
            engine: PhysicsEngine::MultiFidelity,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_per_point_friction: per_point,
            ..Default::default()
        },
    )?;
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<MultiFidelityBackend>()
        .unwrap()
        .set_contact_pgs_passes(passes)?;
    let mut templates = Vec::new();
    for (name, x) in [("cylinder", -0.15), ("capsule", 0.15)] {
        let model = sdf.worlds[0]
            .models
            .iter()
            .find(|m| m.name == name)
            .ok_or("missing delivery model")?;
        let template = prepare_model_template(&sim, &sdf, model)?;
        templates.push((template, x));
    }
    sim.step_exact(10)?;
    let mut parts = Vec::new();
    for (template, x) in templates {
        let id = spawn_model_template(
            &mut sim,
            &template,
            WorldPose {
                translation: [x, -0.12, 0.25],
                rotation: [0., 0., 0., 1.],
            },
        )?;
        let root = entity_by_id(sim.world(), id).unwrap();
        let link = sim
            .world()
            .get::<bevy_ecs::hierarchy::Children>(root)
            .unwrap()[0];
        parts.push((id, sim.world().get::<BodyHandle>(link).unwrap().id()));
    }
    let mut records: [Option<Value>; 2] = [None, None];
    let mut peaks = [0_f64; 2];
    for step in 1..=350 {
        sim.step_exact(1)?;
        let engine = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let patches = if records.iter().any(Option::is_none) {
            engine.rigid_contact_patches()
        } else {
            Vec::new()
        };
        for (i, &(id, body)) in parts.iter().enumerate() {
            let state = model_state(&sim, id)?;
            if records[i].is_some() {
                peaks[i] = peaks[i].max(state.translation[2]);
                continue;
            }
            for patch in patches.iter().filter(|p| p.a == body || p.b == body) {
                let impulse: f64 = patch.points.iter().map(|p| p.normal_impulse).sum();
                if impulse == 0. {
                    continue;
                }
                let points: Vec<_> = patch
                    .points
                    .iter()
                    .map(|p| {
                        json!({
                            "lever_m":if patch.a==body {p.lever_a} else {p.lever_b},
                            "normal_impulse_ns":p.normal_impulse
                        })
                    })
                    .collect();
                records[i] = Some(json!({
                    "flight_steps": step, "normal_impulse_ns": impulse,
                    "linear_velocity": state.linear_velocity, "angular_velocity": state.angular_velocity,
                    "points": points,
                }));
            }
        }
    }
    let [Some(cylinder), Some(capsule)] = records else {
        return Err("both parcels must hit the slab during the drop".into());
    };
    Ok(json!({
        "per_point_friction": per_point, "pgs_passes": passes,
        "cylinder": cylinder, "capsule": capsule, "apex_height_m": peaks,
    }))
}

fn main() -> Result<()> {
    let mut cases = Vec::new();
    for passes in [1, 2, 4, 8, 16, 32] {
        for per_point in [false, true] {
            cases.push(drop_at(WORLD, passes, per_point)?);
        }
    }
    println!("{}", serde_json::to_string_pretty(&cases)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_impacts_and_a_valid_native_budget_are_required_to_report_rebound() {
        assert!(drop_at(WORLD, 0, false).is_err());
        assert!(drop_at("<sdf", 1, false).is_err());
        assert!(
            drop_at(
                &WORLD.replace("name=\"default\"", "name=\"different\""),
                1,
                false
            )
            .is_err()
        );
        assert!(
            drop_at(
                &WORLD.replace("model name=\"capsule\"", "model name=\"other\""),
                1,
                false
            )
            .is_err()
        );
        let absent_floor = WORLD.replace(
            "<model name=\"floor\">",
            "<model name=\"floor\"><pose>0 0 -2 0 0 0</pose>",
        );
        assert!(drop_at(&absent_floor, 1, false).is_err());
        let with_sensor = WORLD.replacen(
            "<link name=\"body\">",
            "<link name=\"body\"><sensor name=\"imu\" type=\"imu\"><imu/></sensor>",
            1,
        );
        assert!(drop_at(&with_sensor, 1, false).is_err());
    }
}
