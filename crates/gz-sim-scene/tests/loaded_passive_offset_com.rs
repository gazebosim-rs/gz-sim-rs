// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Loaded mass frames are separate from collider geometry and model-origin velocity.
use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, LocalPose, ModelState, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    model_state, prepare_model_template, queue_link_world_wrench, remove_model, reset_loaded_world,
    set_model_state, spawn_model_template,
};
use sdformat::NoResources;
const CENTER: [f64; 3] = [0.25, -0.125, 0.5];
fn geometry(kind: usize) -> &'static str {
    match kind % 4 {
        0 => "<box><size>1 1 1</size></box>",
        1 => "<sphere><radius>0.5</radius></sphere>",
        2 => "<cylinder><radius>0.5</radius><length>2</length></cylinder>",
        _ => "<capsule><radius>0.5</radius><length>2</length></capsule>",
    }
}
fn xml(
    count: usize,
    kind: usize,
    fixed: bool,
    density: f64,
    center: &str,
    link_pose: &str,
    posed: bool,
) -> String {
    let collisions=(0..count).map(|i|format!("<collision name='c{i}'><pose>{} 0 0 0 0 0</pose><density>{density}</density><geometry>{}</geometry></collision>",4.*i as f64+if posed {0.25}else{0.},geometry(kind+i))).collect::<String>();
    format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='body'><pose>0 0 10 0 0 0</pose><static>{fixed}</static><link name='link'><pose>{link_pose}</pose><inertial><pose>{center}</pose><mass>2</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial>{collisions}<visual name='view'><pose>0.1 0.2 0.3 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>"
    )
}
fn load(text: &str) -> Simulation {
    let mut sim = load_world_with_sdf_step(
        text,
        "loaded-offset-com",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    sim.initialize_schedules().unwrap();
    sim
}
fn entity(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .get::<EntityId>(entity(sim, name))
        .unwrap()
        .get()
}
fn handle(sim: &Simulation) -> BodyId {
    sim.world()
        .get::<BodyHandle>(entity(sim, "w::body::link"))
        .unwrap()
        .id()
}
fn state(sim: &Simulation) -> BodyState {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(handle(sim))
        .unwrap()
}
fn near(a: f64, b: f64) {
    assert!(
        a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
        "{a} != {b}"
    );
}
fn vector(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        near(a, b);
    }
}
fn center_world(s: BodyState) -> DVec3 {
    DVec3::from_array(s.position) + DQuat::from_array(s.orientation) * DVec3::from_array(CENTER)
}
#[test]
fn loaded_offset_identity_colliders_cover_four_native_constructors_and_compound_counts() {
    for count in [1, 2, 4, 16] {
        for kind in 0..4 {
            for fixed in [false, true] {
                let mut sim = load(&xml(
                    count,
                    kind,
                    fixed,
                    17.,
                    "0.25 -0.125 0.5 0 0 0",
                    "0 0 0 0 0 0",
                    false,
                ));
                let original = state(&sim);
                vector(original.position, [0., 0., 10.]);
                let model = id(&sim, "w::body");
                let link = entity(&sim, "w::body::link");
                assert_eq!(sim.world().get::<Children>(link).unwrap().len(), count + 1);
                let last = 4. * (count - 1) as f64;
                let ray = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .cast_ray([last + 2., 0., 10.], [-1., 0., 0.], 5., None)
                    .unwrap()
                    .unwrap();
                near(ray, 1.5);
                assert_eq!(
                    sim.world()
                        .resource::<PhysicsBackend>()
                        .cast_ray([last + 2., 0., 10.], [-1., 0., 0.], 5., Some(handle(&sim)))
                        .unwrap(),
                    None
                );
                sim.step_exact(2).unwrap();
                assert_eq!(state(&sim), original);
                assert_eq!(id(&sim, "w::body"), model);
            }
        }
    }
}
#[test]
fn loaded_offset_explicit_mass_and_principal_torques_ignore_collision_density() {
    for count in [1, 2, 4, 16] {
        for axis in 0..3 {
            let mut a = load(&xml(
                count,
                axis,
                false,
                0.001,
                "0.25 -0.125 0.5 0 0 0",
                "0 0 0 0 0 0",
                false,
            ));
            let mut b = load(&xml(
                count,
                axis,
                false,
                10000.,
                "0.25 -0.125 0.5 0 0 0",
                "0 0 0 0 0 0",
                false,
            ));
            let initial = center_world(state(&a));
            let mut torque = [0.; 3];
            torque[axis] = 6.;
            for sim in [&mut a, &mut b] {
                let link = id(sim, "w::body::link");
                queue_link_world_wrench(sim, link, [2., -4., 6.], torque).unwrap();
                sim.set_paused(true);
                sim.tick().unwrap();
                vector(state(sim).velocity, [0.; 3]);
                sim.set_paused(false);
                sim.step_exact(1).unwrap();
                vector(state(sim).velocity, [0.001, -0.002, 0.003]);
                let mut expected = [0.; 3];
                expected[axis] = 0.003;
                vector(state(sim).angular_velocity, expected);
                let delta = center_world(state(sim)) - initial;
                // Finite-step COM displacement is causal and bounded by final speed;
                // do not pretend Rapier's staged integration is one full Euler step.
                for (distance, speed) in delta.to_array().into_iter().zip(state(sim).velocity) {
                    assert!(distance * speed > 0. && distance.abs() <= speed.abs() * 0.001 + 1e-11);
                }
            }
            assert_eq!(state(&a), state(&b));
        }
    }
}
#[test]
fn loaded_offset_origin_velocity_six_axes_children_and_both_resets_preserve_mass_frame() {
    let mut sim = load(&xml(
        4,
        0,
        false,
        17.,
        "0.25 -0.125 0.5 0 0 0",
        "0 0 0 0 0 0",
        true,
    ));
    let model = id(&sim, "w::body");
    let mut ids: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(|v| v.get()))
        .collect();
    ids.sort_unstable();
    let initial = state(&sim);
    let requested = ModelState {
        translation: [3., -2., 8.],
        rotation: [0.5; 4],
        linear_velocity: [1., 2., 3.],
        angular_velocity: [0.4, -0.3, 0.2],
    };
    set_model_state(&mut sim, model, requested).unwrap();
    let observed = model_state(&sim, model).unwrap();
    vector(observed.translation, requested.translation);
    vector(observed.linear_velocity, requested.linear_velocity);
    vector(observed.angular_velocity, requested.angular_velocity);
    let q = DQuat::from_array(requested.rotation);
    let shift = DVec3::from_array(requested.angular_velocity).cross(q * DVec3::from_array(CENTER));
    vector(
        state(&sim).velocity,
        (DVec3::from_array(requested.linear_velocity) + shift).to_array(),
    );
    let link = entity(&sim, "w::body::link");
    for child in sim.world().get::<Children>(link).unwrap().iter() {
        let local = sim.world().get::<LocalPose>(child).unwrap().0;
        let world = sim.world().get::<WorldPose>(child).unwrap();
        vector(
            world.translation,
            (DVec3::from_array(requested.translation) + q * DVec3::from_array(local.translation))
                .to_array(),
        );
        let expected = q * DQuat::from_array(local.rotation);
        for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
            vector(
                (DQuat::from_array(world.rotation) * axis).to_array(),
                (expected * axis).to_array(),
            );
        }
    }
    sim.reset().unwrap();
    assert_eq!(state(&sim), initial);
    let old_link = link;
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(id(&sim, "w::body"), model);
    assert_ne!(entity(&sim, "w::body::link"), old_link);
    assert_eq!(state(&sim), initial);
    let mut now: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(|v| v.get()))
        .collect();
    now.sort_unstable();
    assert_eq!(ids, now);
}
#[test]
fn loaded_offset_full_link_rotation_is_a_body_pose_not_a_collider_pose() {
    let text = xml(
        2,
        1,
        false,
        17.,
        "0.25 -0.125 0.5 0 0 0",
        "1 2 3 0.3 -0.2 0.4",
        true,
    )
    .replacen(
        "<pose>0.25 0 0 0 0 0</pose>",
        "<pose>0.25 0 0 0.2 0.4 -0.3</pose>",
        1,
    );
    let mut sim = load(&text);
    let pose = *sim
        .world()
        .get::<WorldPose>(entity(&sim, "w::body::link"))
        .unwrap();
    vector(state(&sim).position, pose.translation);
    for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
        vector(
            (DQuat::from_array(state(&sim).orientation) * axis).to_array(),
            (DQuat::from_array(pose.rotation) * axis).to_array(),
        );
    }
    let q = DQuat::from_array(pose.rotation);
    let origin = DVec3::from_array(pose.translation) + q * DVec3::new(2.25, 0., 0.);
    let direction = -(q * DVec3::X);
    near(
        sim.world()
            .resource::<PhysicsBackend>()
            .cast_ray(origin.to_array(), direction.to_array(), 5., None)
            .unwrap()
            .unwrap(),
        1.5,
    );
    let child = sim
        .world()
        .get::<WorldPose>(entity(&sim, "w::body::link::c0"))
        .unwrap();
    let expected = q
        * (DQuat::from_rotation_z(-0.3)
            * DQuat::from_rotation_y(0.4)
            * DQuat::from_rotation_x(0.2));
    for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
        vector(
            (DQuat::from_array(child.rotation) * axis).to_array(),
            (expected * axis).to_array(),
        );
    }
    let link = id(&sim, "w::body::link");
    queue_link_world_wrench(&mut sim, link, [2., 0., 0.], [0.; 3]).unwrap();
    sim.step_exact(1).unwrap();
    vector(state(&sim).velocity, [0.001, 0., 0.]);
    // Existing world edits intentionally require identity LINK; loading does not broaden them.
    assert!(model_state(&sim, id(&sim, "w::body")).is_err());
}
#[test]
fn loaded_offset_sources_clone_through_existing_posed_and_compound_routes_and_retire_independently()
{
    for count in [1, 4] {
        let text = xml(
            count,
            1,
            false,
            17.,
            "0.25 -0.125 0.5 0 0 0",
            "0 0 0 0 0 0",
            true,
        );
        let mut sim = load(&text);
        let source = id(&sim, "w::body");
        let sdf = sdformat::load_str(&text, "loaded-offset-com", &NoResources).unwrap();
        let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
        let output = spawn_model_template(
            &mut sim,
            &template,
            WorldPose {
                translation: [100., 0., 10.],
                rotation: [0., 0., 0., 1.],
            },
        )
        .unwrap();
        remove_model(&mut sim, source).unwrap();
        assert!(entity_by_id(sim.world(), output).is_some());
        let output_state = model_state(&sim, output).unwrap();
        vector(output_state.translation, [100., 0., 10.]);
        sim.reset().unwrap();
        assert!(entity_by_id(sim.world(), output).is_some());
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert!(entity_by_id(sim.world(), output).is_none());
        assert_eq!(id(&sim, "w::body"), source);
    }
}
#[test]
fn loaded_offset_numeric_rotated_inertia_actuation_mesh_and_tpe_boundaries_are_closed() {
    for center in ["1e200 0 0 0 0 0", "0.25 0 0 0 0 0.2", "NaN 0 0 0 0 0"] {
        assert!(
            load_world_with_sdf_step(
                &xml(1, 1, false, 17., center, "0 0 0 0 0 0", false),
                "bad-com",
                "w",
                &NoResources,
                PhysicsEngine::Rapier
            )
            .is_err()
        );
    }
    let text = xml(
        1,
        1,
        false,
        17.,
        "0.25 -0.125 0.5 0 0 0",
        "0 0 0 0 0 0",
        false,
    );
    assert!(
        load_world_with_sdf_step(&text, "bad-com", "w", &NoResources, PhysicsEngine::Tpe).is_err()
    );
    let controlled=text.replace("</model>","<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/></model>");
    assert!(
        load_world_with_sdf_step(
            &controlled,
            "bad-com",
            "w",
            &NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
    let tiny = load(&xml(
        1,
        1,
        false,
        17.,
        "1e-13 0 0 0 0 0",
        "0 0 0 0 0 0",
        false,
    ));
    vector(state(&tiny).position, [0., 0., 10.]);
    // Invalid external Mesh resources remain refused; no implicit primitive fallback.
    let mesh = text.replace(
        geometry(1),
        "<mesh><uri>missing-offset-model.obj</uri></mesh>",
    );
    assert!(
        load_world_with_sdf_step(&mesh, "bad-com", "w", &NoResources, PhysicsEngine::Rapier)
            .is_err()
    );
}
#[test]
fn loaded_offset_falls_and_contacts_without_moving_static_geometry() {
    let model = xml(
        1,
        1,
        false,
        17.,
        "0.25 -0.125 0.5 0 0 0",
        "0 0 0 0 0 0",
        false,
    )
    .replace("<gravity>0 0 0</gravity>", "<gravity>0 0 -9.8</gravity>")
    .replace("0 0 10 0 0 0", "0 0 3 0 0 0");
    let ground = "<model name='ground'><static>true</static><link name='ground'><collision name='floor'><geometry><box><size>20 20 1</size></box></geometry></collision></link></model>";
    let mut sim = load(&model.replace("</world>", &format!("{ground}</world>")));
    let initial = state(&sim);
    sim.step_exact(10).unwrap();
    assert!(state(&sim).velocity[2] < 0.);
    assert!(state(&sim).position[2] < initial.position[2]);
    sim.step_exact(1490).unwrap();
    let after = state(&sim);
    assert!(
        after
            .position
            .iter()
            .chain(after.velocity.iter())
            .all(|v| v.is_finite())
    );
    assert!(after.position[2] > 0.8 && after.position[2] < 1.5);
    let ground = sim
        .world()
        .get::<BodyHandle>(entity(&sim, "w::ground::ground"))
        .unwrap()
        .id();
    vector(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(ground)
            .unwrap()
            .position,
        [0.; 3],
    );
}

#[test]
fn existing_automatic_mass_outputs_are_consumed_like_exact_resolved_explicit_inertia() {
    for kind in 0..3 {
        let explicit = xml(1, kind, false, 17., "0.25 0 0 0 0 0", "0 0 0 0 0 0", true);
        let start = explicit.find("<inertial>").unwrap();
        let end = explicit.find("</inertial>").unwrap() + "</inertial>".len();
        let auto = format!(
            "{}<inertial auto='true'/>{}",
            &explicit[..start],
            &explicit[end..]
        );
        let dom = sdformat::load_str(&auto, "resolved-auto", &NoResources).unwrap();
        let inertia = &dom.worlds[0].models[0].links[0].inertial;
        let c = inertia.pose.transform.translation;
        let [ixx, iyy, izz, ixy, ixz, iyz] = inertia.inertia;
        let resolved = format!(
            "<inertial><pose>{} {} {} 0 0 0</pose><mass>{}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz><ixy>{ixy}</ixy><ixz>{ixz}</ixz><iyz>{iyz}</iyz></inertia></inertial>",
            c[0], c[1], c[2], inertia.mass
        );
        let explicit = format!("{}{}{}", &explicit[..start], resolved, &explicit[end..]);
        let mut a = load(&auto);
        let mut b = load(&explicit);
        assert_eq!(state(&a), state(&b));
        for sim in [&mut a, &mut b] {
            let link = id(sim, "w::body::link");
            queue_link_world_wrench(sim, link, [2., -4., 6.], [0.; 3]).unwrap();
            sim.step_exact(1).unwrap();
            vector(
                state(sim).velocity,
                [
                    0.002 / inertia.mass,
                    -0.004 / inertia.mass,
                    0.006 / inertia.mass,
                ],
            );
        }
        assert_eq!(state(&a), state(&b));
    }
}
