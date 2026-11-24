// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Finite translating receiver in the ordinary SDF material and rigid schedules.
use glam::DVec3;
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step,
    queue_link_world_velocity, reset_loaded_world,
};
use serde_json::json;
use std::{collections::BTreeMap, fs};
const SOURCE: &str = include_str!("../../../examples/worlds/local_cloth_conveyor.sdf");
const SURFACE: &str = "default::conveyor::deck";
const DT: f64 = 0.002;
const DRIFT: f64 = 0.4;
const END: usize = 240;

fn load(xml: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        xml,
        "conveyor.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn static_world() -> String {
    let a = SOURCE.find("    <plugin").unwrap();
    let b = SOURCE[a..].find("</plugin>").unwrap() + a + "</plugin>".len();
    let mut xml = SOURCE.to_string();
    xml.replace_range(a..b, "<static>true</static>");
    xml.replace("velocity=\"0.4 0 0\"", "velocity=\"0 0 0\"")
        .replace("velocity=\"1.65 0 0\"", "velocity=\"1.25 0 0\"")
}
#[allow(deprecated)]
fn deck(sim: &Simulation) -> (u64, gz_physics::BodyId) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == SURFACE))
        .unwrap();
    (
        e.get::<EntityId>().unwrap().get(),
        e.get::<BodyHandle>().unwrap().id(),
    )
}

#[test]
fn grip_slip_and_edge_departure_agree_in_both_deck_frames_and_after_reset() {
    let mut sim = load(SOURCE).unwrap();
    let mut control = load(&static_world()).unwrap();
    let (_, body) = deck(&sim);
    let mut reference = Vec::new();
    let mut first_epoch = Vec::new();
    let mut max_frame_error = 0_f64;
    let mut max_velocity_error = 0_f64;
    let mut max_support_error = 0_f64;
    let mut max_position_error = 0_f64;
    let mut max_moment_error = 0_f64;
    let mut last_load = 0;
    let mut measured_forces = [DVec3::ZERO; 2];
    let mut flight_velocity = DVec3::ZERO;
    for epoch in 0..2 {
        if epoch == 1 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            reset_loaded_world(&mut control, |_| ()).unwrap();
        }
        for tick in 0..=END {
            let t = tick as f64 * DT;
            if tick > 0 {
                sim.step_exact(1).unwrap();
                control.step_exact(1).unwrap();
            }
            let owner = sim
                .world()
                .resource::<PhysicsBackend>()
                .state(body)
                .unwrap();
            assert!((owner.position[0] - DRIFT * t).abs() < 1e-12);
            let pose = |p: DVec3| json!({"translation":p.to_array(),"rotation":[0.,0.,0.,1.]});
            let mut poses = BTreeMap::from([(
                format!("{SURFACE}::board"),
                pose(DVec3::new(0.06 + DRIFT * t, 0., 0.30)),
            )]);
            let mut states = Vec::new();
            for ((name, sheet), (control_name, stationary)) in bodies(&sim).zip(bodies(&control)) {
                assert_eq!(name, control_name);
                let carried = name == "carried";
                let relative_initial = if carried { 0. } else { 1.25 };
                let start_x = if carried { -0.12 } else { 0.12 };
                let mut position = DVec3::ZERO;
                let mut velocity = DVec3::ZERO;
                for (i, (p, c)) in sheet
                    .particles()
                    .iter()
                    .zip(stationary.particles())
                    .enumerate()
                {
                    let x = DVec3::from(p.position);
                    let v = DVec3::from(p.velocity);
                    max_frame_error = max_frame_error
                        .max((x - DVec3::X * (DRIFT * t)).distance(c.position.into()));
                    max_velocity_error =
                        max_velocity_error.max((v - DVec3::X * DRIFT).distance(c.velocity.into()));
                    position += 0.25 * x;
                    velocity += 0.25 * v;
                    poses.insert(format!("{SURFACE}::{name}_{i}"), pose(x));
                }
                let j = sheet
                    .last_boundary_impulses()
                    .first()
                    .copied()
                    .unwrap_or_default();
                let force = DVec3::from(j.linear) / DT;
                states.push((sheet.particles().to_vec(), j));
                if tick > 0 && (carried || t <= 0.15) {
                    let acceleration = if carried { 0. } else { -0.1 * 9.81 };
                    let analytic =
                        start_x + (DRIFT + relative_initial) * t + 0.5 * acceleration * t * t;
                    max_position_error = max_position_error.max((position.x - analytic).abs());
                    assert!(
                        (velocity.x - DRIFT - relative_initial - acceleration * t).abs() < 1e-9
                    );
                    if epoch == 0 && tick == 50 {
                        measured_forces[usize::from(!carried)] = force;
                    }
                    let expected = DVec3::new(-0.1 * acceleration, 0., -0.981);
                    max_support_error = max_support_error.max(force.distance(expected));
                    // World z torque at the board's initial collision center from a signed horizontal reaction.
                    let torque = DVec3::from(j.angular) / DT;
                    max_moment_error = max_moment_error
                        .max((torque.z + position.y * force.x).abs())
                        .max((torque.x - position.y * force.z).abs());
                    assert!((position.z - 0.328).abs() < 1e-10);
                }
                if !carried && tick > 0 {
                    if force.length() > 1e-8 {
                        last_load = tick;
                    }
                    if tick > 200 {
                        assert_eq!(j.linear, [0.; 3]);
                        if tick > 201 {
                            assert!(
                                (velocity - flight_velocity - DVec3::NEG_Z * (9.81 * DT)).length()
                                    < 1e-9
                            );
                        }
                        flight_velocity = velocity;
                    }
                }
            }
            if epoch == 0 {
                first_epoch.push(states);
                reference.push(json!({"time_ns":tick*2_000_000,"poses":poses}));
            } else {
                assert_eq!(states, first_epoch[tick]);
            }
        }
    }
    let receipt = json!({"ticks":END,"last_loaded_tick":last_load,"max_transformed_position_error_m":max_frame_error,
    "max_transformed_velocity_error_m_s":max_velocity_error,"max_supported_trajectory_error_m":max_position_error,
    "max_support_force_error_n":max_support_error,"max_support_moment_error_nm":max_moment_error,
    "co_moving_force_n":measured_forces[0].to_array(),"sliding_force_n":measured_forces[1].to_array(),"end_velocity_m_s":flight_velocity.to_array()});
    println!("{receipt}");
    assert!((80..=160).contains(&last_load));
    assert!(max_position_error < 0.00005 && max_support_error < 1e-8 && max_moment_error < 1e-8);
    assert!(max_frame_error < 1e-7 && max_velocity_error < 1e-5);
    if let Some(out) = std::env::var_os("GZ_CONVEYOR_MEASUREMENT") {
        let dir = std::path::Path::new(&out);
        fs::create_dir_all(dir).unwrap();
        fs::write(
            dir.join("measurements.json"),
            serde_json::to_vec_pretty(&receipt).unwrap(),
        )
        .unwrap();
        fs::write(
            dir.join("visual-reference.json"),
            serde_json::to_vec(&reference).unwrap(),
        )
        .unwrap();
    }
}

#[test]
fn rotation_refusal_preserves_the_previous_translation_and_clock() {
    let spin = SOURCE.replace("<angular_velocity>0 0 0", "<angular_velocity>0 0 1");
    assert!(
        load(&spin)
            .err()
            .unwrap()
            .to_string()
            .contains("linearly translating")
    );
    let mut sim = load(SOURCE).unwrap();
    let (id, body) = deck(&sim);
    sim.set_paused(true);
    queue_link_world_velocity(&mut sim, id, [0.1, 0., 0.], [0.; 3]).unwrap();
    assert!(queue_link_world_velocity(&mut sim, id, [0.7, 0., 0.], [0., 0., 0.1]).is_err());
    sim.tick().unwrap();
    assert!(sim.elapsed().is_zero());
    sim.step_exact(1).unwrap();
    assert!(
        (sim.world()
            .resource::<PhysicsBackend>()
            .state(body)
            .unwrap()
            .position[0]
            - 0.1 * DT)
            .abs()
            < 1e-12
    );
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_world_velocity(body, [0.; 3], [0., 0., 1.])
        .unwrap();
    assert!(sim.step_exact(1).is_err());
    assert_eq!(sim.elapsed(), std::time::Duration::from_millis(2));
    assert!(
        sim.fault()
            .unwrap()
            .to_string()
            .contains("linear translation")
    );
}

#[test]
fn centered_deck_and_offset_collision_give_the_same_material_motion() {
    let pose = "<pose>0.06 0 0.30 0 0 0</pose>";
    let centered = SOURCE.replace(pose, "").replace(
        "<link name=\"deck\">",
        &format!("<link name=\"deck\">{pose}"),
    );
    let mut offset = load(SOURCE).unwrap();
    let mut centered = load(&centered).unwrap();
    for _ in 0..END {
        offset.step_exact(1).unwrap();
        centered.step_exact(1).unwrap();
        for ((_, a), (_, b)) in bodies(&offset).zip(bodies(&centered)) {
            for (p, q) in a.particles().iter().zip(b.particles()) {
                assert!(DVec3::from(p.position).distance(q.position.into()) < 1e-9);
            }
            let ja = a.last_boundary_impulses()[0];
            let jb = b.last_boundary_impulses()[0];
            assert!(DVec3::from(ja.linear).distance(jb.linear.into()) < 1e-9);
            assert!(DVec3::from(ja.angular).distance(jb.angular.into()) < 1e-9);
        }
    }
    let guided = include_str!("../../../examples/multifidelity_curved_cable_guide.sdf");
    let curve = guided
        .split_once("<trajectory")
        .unwrap()
        .1
        .split_once("</trajectory>")
        .unwrap()
        .0;
    let curved = SOURCE.replace(
        "<linear_velocity>0.4 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity>",
        &format!("<trajectory{curve}</trajectory>"),
    );
    let error = load(&curved).err().unwrap().to_string();
    assert!(error.contains("linearly translating"), "{error}");
}
