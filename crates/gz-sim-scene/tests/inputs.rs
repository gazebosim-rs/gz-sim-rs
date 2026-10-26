// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    InputError, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    queue_link_world_velocity, queue_link_world_wrench, remove_model, reset_loaded_world,
};
use sdformat::NoResources;

fn load(engine: PhysicsEngine) -> Simulation {
    load_world_with_sdf_step("<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.01</max_step_size></physics><model name='robot'><link name='body'><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></world></sdf>", "inputs", "default", &NoResources, engine).unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn velocity(sim: &Simulation, id: u64) -> [f64; 3] {
    sim.world()
        .get::<Velocity>(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
        .linear
}

#[test]
fn pending_velocity_replaces_retains_through_pause_and_cancels_on_both_resets() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load(engine);
        let link = id(&sim, "default::robot::body");
        queue_link_world_velocity(&mut sim, link, [9.0; 3], [0.0; 3]).unwrap();
        queue_link_world_velocity(&mut sim, link, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
        assert_eq!(velocity(&sim, link), [0.0; 3]);
        sim.tick().unwrap();
        assert_eq!(velocity(&sim, link), [0.0; 3]);
        sim.step_exact(1).unwrap();
        assert_eq!(velocity(&sim, link), [1.0, 0.0, 0.0]);
        let entity = entity_by_id(sim.world(), link).unwrap();
        assert!(
            (sim.world().get::<WorldPose>(entity).unwrap().translation[0] - 0.01).abs() < 1e-12
        );
        queue_link_world_velocity(&mut sim, link, [9.0; 3], [0.0; 3]).unwrap();
        sim.reset().unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(velocity(&sim, link), [0.0; 3]);
        queue_link_world_velocity(&mut sim, link, [9.0; 3], [0.0; 3]).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(velocity(&sim, link), [0.0; 3]);
        queue_link_world_velocity(&mut sim, link, [9.0; 3], [0.0; 3]).unwrap();
        let model = id(&sim, "default::robot");
        remove_model(&mut sim, model).unwrap();
        assert!(matches!(
            queue_link_world_velocity(&mut sim, link, [0.0; 3], [0.0; 3]),
            Err(InputError::MissingEntity(_))
        ));
        sim.step_exact(1).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(velocity(&sim, link), [0.0; 3]);
    }
}

#[test]
fn admitted_force_is_one_tick_and_bad_inputs_leave_pending_requests_unchanged() {
    let mut sim = load(PhysicsEngine::Rapier);
    let link = id(&sim, "default::robot::body");
    queue_link_world_velocity(&mut sim, link, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
    queue_link_world_wrench(&mut sim, link, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
    queue_link_world_wrench(&mut sim, link, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
    assert!(matches!(
        queue_link_world_velocity(&mut sim, link, [9.0; 3], [f64::NAN; 3]),
        Err(InputError::InvalidValue)
    ));
    assert!(matches!(
        queue_link_world_wrench(&mut sim, link, [f64::INFINITY; 3], [0.0; 3]),
        Err(InputError::InvalidValue)
    ));
    sim.tick().unwrap();
    assert_eq!(velocity(&sim, link), [0.0; 3]);
    for _ in 0..3 {
        sim.step_exact(1).unwrap();
        assert!((velocity(&sim, link)[0] - 1.01).abs() < 1e-12);
    }
    queue_link_world_wrench(&mut sim, link, [2.0, 0.0, 0.0], [0.0; 3]).unwrap();
    sim.reset().unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(velocity(&sim, link), [0.0; 3]);
    let mut tpe = load(PhysicsEngine::Tpe);
    let link = id(&tpe, "default::robot::body");
    assert!(matches!(
        queue_link_world_wrench(&mut tpe, link, [2.0, 0.0, 0.0], [0.0; 3]),
        Err(InputError::Unsupported("TPE external wrench"))
    ));
    tpe.step_exact(1).unwrap();
    assert_eq!(velocity(&tpe, link), [0.0; 3]);
}

#[test]
fn fixed_body_commands_are_explicitly_unsupported_and_preserve_the_scene() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load_world_with_sdf_step(
            include_str!("fixtures/fall.sdf"),
            "fixed-input",
            "default",
            &NoResources,
            engine,
        )
        .unwrap();
        let fixed = id(&sim, "default::ground::base");
        assert!(matches!(
            queue_link_world_velocity(&mut sim, fixed, [1.0; 3], [0.0; 3]),
            Err(InputError::Unsupported(_))
        ));
        assert!(matches!(
            queue_link_world_wrench(&mut sim, fixed, [1.0; 3], [0.0; 3]),
            Err(InputError::Unsupported(_))
        ));
        sim.step_exact(1).unwrap();
        assert_eq!(velocity(&sim, fixed), [0.0; 3]);
    }
}

#[test]
fn native_world_velocity_and_wrench_controls_match_supported_observations() {
    let rows: Vec<serde_json::Value> =
        serde_json::from_str(include_str!("fixtures/physics_inputs.json")).unwrap();
    for row in rows {
        let native_engine = row["engine"].as_str().unwrap();
        let engine = if native_engine == "dartsim" {
            PhysicsEngine::Rapier
        } else {
            PhysicsEngine::Tpe
        };
        // Replace only native host-plugin selection with the explicit Rust backend API.
        let plugin = format!(
            "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'><engine><filename>gz-physics-{native_engine}-plugin</filename></engine></plugin>"
        );
        let xml = row["xml"].as_str().unwrap();
        assert_eq!(xml.matches(&plugin).count(), 1);
        let mut sim = load_world_with_sdf_step(
            &xml.replace(&plugin, ""),
            "native-inputs",
            "default",
            &NoResources,
            engine,
        )
        .unwrap();
        let link = id(&sim, "default::robot::body");
        let mode = row["mode"].as_str().unwrap();
        let linear = if matches!(mode, "velocity" | "both") {
            [1.0, 0.0, 0.0]
        } else {
            [0.0; 3]
        };
        let angular = if mode == "angular" {
            [0.0, 0.0, 1.0]
        } else {
            [0.0; 3]
        };
        if native_engine == "tpe" && row["frame"] != "identity" {
            assert!(matches!(
                queue_link_world_wrench(&mut sim, link, linear, angular),
                Err(InputError::Unsupported(_))
            ));
            continue; // Corrected velocity observations are checked separately in tpe_policy.rs.
        }
        if matches!(mode, "velocity" | "angular" | "both") {
            queue_link_world_velocity(&mut sim, link, linear, angular).unwrap();
        }
        if matches!(mode, "force" | "torque" | "both") {
            let force = if mode == "torque" {
                [0.0; 3]
            } else {
                [2.0, 0.0, 0.0]
            };
            let torque = if mode == "torque" {
                [0.0, 0.0, 0.2]
            } else {
                [0.0; 3]
            };
            let result = queue_link_world_wrench(&mut sim, link, force, torque);
            if native_engine == "tpe" {
                assert!(matches!(
                    result,
                    Err(InputError::Unsupported("TPE external wrench"))
                ));
            } else {
                result.unwrap();
            }
        }
        for native in row["native"].as_array().unwrap() {
            sim.step_exact(1).unwrap();
            let entity = entity_by_id(sim.world(), link).unwrap();
            let v = sim.world().get::<Velocity>(entity).unwrap();
            for (actual, key) in [
                (v.linear[0], "vx"),
                (v.linear[1], "vy"),
                (v.angular[2], "wz"),
            ] {
                assert!(
                    (actual - native[key].as_f64().unwrap()).abs() < 1e-12,
                    "{native_engine}/{}/{mode}/{key}: {actual} versus {}",
                    row["frame"],
                    native[key]
                );
            }
            let n = native["iteration"].as_u64().unwrap();
            assert_eq!(sim.elapsed(), std::time::Duration::from_millis(10 * n));
            let pose = sim.world().get::<WorldPose>(entity).unwrap();
            if native_engine == "dartsim" && matches!(mode, "force" | "both") {
                // One force pulse: a=1 m/s² for 10ms, then constant velocity.
                // Half an Euler tick bounds first-order position bias on both solvers.
                let analytic = linear[0] * n as f64 * 0.01 + 0.0001 * (n as f64 - 0.5);
                for actual in [pose.translation[0], native["x"].as_f64().unwrap()] {
                    assert!((actual - analytic).abs() <= 0.00005 + 1e-12);
                }
            } else {
                assert!((pose.translation[0] - native["x"].as_f64().unwrap()).abs() < 1e-12);
            }
            assert!((pose.translation[1] - native["y"].as_f64().unwrap()).abs() < 1e-12);
        }
    }
}
