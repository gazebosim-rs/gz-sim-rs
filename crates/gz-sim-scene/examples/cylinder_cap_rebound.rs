// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Drop the same cylinder and capsule at several native contact iteration budgets.
use glam::DVec3;
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_physics_rapier::RigidContactPatch;
use gz_sim_core::entity_by_id;
use gz_sim_scene::{
    BodyHandle, PhysicsEngine, WorldLoadOptions, WorldPose, WorldRuntimeOptions,
    load_world_with_runtime, model_state, prepare_model_template, spawn_model_template,
};
use serde_json::{Value, json};
use std::error::Error;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORLD: &str = include_str!("../../../examples/worlds/local_curved_delivery.sdf");

fn drop_at(world: &str, passes: usize, per_point: bool, axial: bool) -> Result<Value> {
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
            rapier_axial_cap_impact: axial,
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
                let on_a = patch.a == body;
                let (normal_impulse, angular_normal_impulse, points) = normal_reaction(patch, on_a);
                let (floor_normal_impulse, floor_angular_impulse, floor_points) =
                    normal_reaction(patch, !on_a);
                records[i] = Some(json!({
                    "flight_steps": step, "normal_impulse_ns": impulse,
                    "linear_velocity": state.linear_velocity, "angular_velocity": state.angular_velocity,
                    "points": points, "normal_impulse_world_ns": normal_impulse.to_array(),
                    "angular_normal_impulse_at_com_nms": angular_normal_impulse.to_array(),
                    "floor_points": floor_points, "floor_normal_impulse_world_ns": floor_normal_impulse.to_array(),
                    "floor_angular_normal_impulse_at_origin_nms": floor_angular_impulse.to_array(),
                }));
            }
        }
    }
    let [Some(cylinder), Some(capsule)] = records else {
        return Err("both parcels must hit the slab during the drop".into());
    };
    Ok(json!({
        "per_point_friction": per_point, "pgs_passes": passes, "axial_cap_impact": axial,
        "cylinder": cylinder, "capsule": capsule, "apex_height_m": peaks,
    }))
}

// The moving side stores COM-relative levers. The fixed slab stores world anchors.
fn normal_reaction(patch: &RigidContactPatch, on_a: bool) -> (DVec3, DVec3, Vec<Value>) {
    let direction = DVec3::from_array(patch.normal_on_a) * if on_a { 1. } else { -1. };
    let mut linear = DVec3::ZERO;
    let mut angular = DVec3::ZERO;
    let mut points = Vec::new();
    for point in &patch.points {
        let lever = if on_a { point.lever_a } else { point.lever_b };
        let impulse = direction * point.normal_impulse;
        linear += impulse;
        angular += DVec3::from_array(lever).cross(impulse);
        points.push(json!({"lever_m": lever, "normal_impulse_ns": point.normal_impulse}));
    }
    (linear, angular, points)
}

fn main() -> Result<()> {
    let mut cases = Vec::new();
    for passes in [1, 2, 4, 8, 16, 32] {
        for per_point in [false, true] {
            for axial in [false, true] {
                cases.push(drop_at(WORLD, passes, per_point, axial)?);
            }
        }
    }
    println!("{}", serde_json::to_string_pretty(&cases)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn both_sides_report_equal_and_opposite_reactions_at_the_drop_location() {
        let run = drop_at(WORLD, 1, false, true).unwrap();
        let vector =
            |value: &Value| DVec3::from_array(serde_json::from_value(value.clone()).unwrap());
        for (name, x) in [("cylinder", -0.15), ("capsule", 0.15)] {
            let hit = &run[name];
            let body = vector(&hit["normal_impulse_world_ns"]);
            let floor = vector(&hit["floor_normal_impulse_world_ns"]);
            let floor_moment = vector(&hit["floor_angular_normal_impulse_at_origin_nms"]);
            assert!(body.z > 0.65);
            assert!((body + floor).length() < 1e-12);
            assert!((floor_moment - DVec3::new(x, -0.12, 0.).cross(floor)).length() < 1e-12);
        }
    }

    #[test]
    fn two_impacts_and_a_valid_native_budget_are_required_to_report_rebound() {
        assert!(drop_at(WORLD, 0, false, false).is_err());
        assert!(drop_at("<sdf", 1, false, false).is_err());
        assert!(
            drop_at(
                &WORLD.replace("name=\"default\"", "name=\"different\""),
                1,
                false,
                false
            )
            .is_err()
        );
        assert!(
            drop_at(
                &WORLD.replace("model name=\"capsule\"", "model name=\"other\""),
                1,
                false,
                false
            )
            .is_err()
        );
        let absent_floor = WORLD.replace(
            "<model name=\"floor\">",
            "<model name=\"floor\"><pose>0 0 -2 0 0 0</pose>",
        );
        assert!(drop_at(&absent_floor, 1, false, false).is_err());
        let with_sensor = WORLD.replacen(
            "<link name=\"body\">",
            "<link name=\"body\"><sensor name=\"imu\" type=\"imu\"><imu/></sensor>",
            1,
        );
        assert!(drop_at(&with_sensor, 1, false, false).is_err());
    }
}
