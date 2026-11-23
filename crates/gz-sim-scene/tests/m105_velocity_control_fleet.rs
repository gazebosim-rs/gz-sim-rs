// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    Name, PhysicsEngine, Velocity, VelocityControlConfiguration, WorldPose,
    initial_velocity_control_configurations_with_identity, load_world_with_sdf_step, remove_model,
    reset_loaded_world, set_velocity_control_command, velocity_control_binding_matches,
};

fn xml(count: usize, compound: bool) -> String {
    let mut models = String::new();
    for i in 0..count {
        let collision = if compound {
            "<collision name='a'><pose>0 0 0 0.2 0.3 0.4</pose><geometry><box><size>1 1 1</size></box></geometry></collision><collision name='b'><pose>2 0 0 0 0 0</pose><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>"
        } else {
            "<collision name='a'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>"
        };
        models += &format!(
            "<model name='r{i}'><pose>{} 0 10 0 0 {}</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial>{collision}</link><plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'><initial_linear>{} 0 0</initial_linear></plugin></model>",
            i * 20,
            i as f64 * 0.1,
            i + 1
        );
    }
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn load(count: usize, engine: PhysicsEngine, compound: bool) -> Simulation {
    load_world_with_sdf_step(
        &xml(count, compound),
        "m105.sdf",
        "default",
        &sdformat::NoResources,
        engine,
    )
    .unwrap()
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
fn pose(sim: &Simulation, id: u64) -> WorldPose {
    *sim.world()
        .get::<WorldPose>(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
}
fn near(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        assert!(
            a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
            "{a} != {b}"
        );
    }
}
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = q;
    let [a, b, c] = v;
    [
        (1. - 2. * (y * y + z * z)) * a + 2. * (x * y - z * w) * b + 2. * (x * z + y * w) * c,
        2. * (x * y + z * w) * a + (1. - 2. * (x * x + z * z)) * b + 2. * (y * z - x * w) * c,
        2. * (x * z - y * w) * a + 2. * (y * z + x * w) * b + (1. - 2. * (x * x + y * y)) * c,
    ]
}
fn velocity(sim: &Simulation, link: u64) -> Velocity {
    *sim.world()
        .get::<Velocity>(entity_by_id(sim.world(), link).unwrap())
        .unwrap()
}
#[test]
fn m105_two_and_eight_owners_move_independently_on_both_backends() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for count in [2, 8] {
            let mut sim = load(count, engine, false);
            for _ in 0..20 {
                let before: Vec<_> = (0..count)
                    .map(|i| pose(&sim, id(&sim, &format!("default::r{i}"))))
                    .collect();
                sim.step_exact(1).unwrap();
                for (i, p) in before.into_iter().enumerate() {
                    let model = id(&sim, &format!("default::r{i}"));
                    let link = id(&sim, &format!("default::r{i}::body"));
                    let v = rotate(p.rotation, [(i + 1) as f64, 0., 0.]);
                    near(velocity(&sim, link).linear, v);
                    near(
                        pose(&sim, model).translation,
                        std::array::from_fn(|j| p.translation[j] + v[j] * 0.001),
                    );
                    assert!(velocity_control_binding_matches(&sim, model));
                }
            }
        }
    }
}
#[test]
fn m105_paused_latest_invalid_zero_and_poststep_latch_are_per_owner() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load(8, engine, false);
        for i in 0..8 {
            let m = id(&sim, &format!("default::r{i}"));
            set_velocity_control_command(&mut sim, m, [99.; 3], [0.; 3]).unwrap();
            set_velocity_control_command(&mut sim, m, [0., (i + 1) as f64, 0.], [0.; 3]).unwrap();
            assert!(set_velocity_control_command(&mut sim, m, [f64::NAN; 3], [0.; 3]).is_err());
        }
        for _ in 0..3 {
            sim.tick().unwrap();
        }
        sim.step_exact(1).unwrap();
        for i in 0..8 {
            let l = id(&sim, &format!("default::r{i}::body"));
            near(
                velocity(&sim, l).linear,
                rotate(pose(&sim, l).rotation, [(i + 1) as f64, 0., 0.]),
            );
        }
        sim.step_exact(1).unwrap();
        for i in 0..8 {
            let l = id(&sim, &format!("default::r{i}::body"));
            near(
                velocity(&sim, l).linear,
                rotate(pose(&sim, l).rotation, [0., (i + 1) as f64, 0.]),
            );
            let m = id(&sim, &format!("default::r{i}"));
            set_velocity_control_command(&mut sim, m, [0.; 3], [0.; 3]).unwrap();
        }
        sim.step_exact(2).unwrap();
        for i in 0..8 {
            near(
                velocity(&sim, id(&sim, &format!("default::r{i}::body"))).linear,
                [0.; 3],
            );
        }
    }
}
#[test]
fn m105_rotated_compound_owners_preserve_child_geometry_and_native_state() {
    let mut sim = load(8, PhysicsEngine::Rapier, true);
    for _ in 0..5 {
        sim.step_exact(1).unwrap();
    }
    for i in 0..8 {
        let model = id(&sim, &format!("default::r{i}"));
        let p = pose(&sim, model);
        let child = id(&sim, &format!("default::r{i}::body::b"));
        let arm = rotate(p.rotation, [2., 0., 0.]);
        near(
            pose(&sim, child).translation,
            std::array::from_fn(|j| p.translation[j] + arm[j]),
        );
        let backend = sim.world().resource::<gz_physics::PhysicsBackend>();
        let body = sim
            .world()
            .get::<gz_sim_scene::BodyHandle>(
                entity_by_id(sim.world(), id(&sim, &format!("default::r{i}::body"))).unwrap(),
            )
            .unwrap()
            .id();
        near(backend.state(body).unwrap().position, p.translation);
        let center = pose(&sim, child).translation;
        let hit = backend
            .cast_ray(
                [center[0], center[1], center[2] + 2.],
                [0., 0., -1.],
                3.,
                None,
            )
            .unwrap()
            .unwrap();
        assert!((hit - 1.75).abs() < 1e-11);
        assert!(
            backend
                .cast_ray(
                    [center[0], center[1], center[2] + 2.],
                    [0., 0., -1.],
                    3.,
                    Some(body)
                )
                .unwrap()
                .is_none()
        );
    }
}
#[test]
fn m105_full_and_loaded_resets_removal_restore_exact_original_fleet() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load(8, engine, false);
        let ids: Vec<_> = initial_velocity_control_configurations_with_identity(&sim)
            .unwrap()
            .map(|r| (r.id, r.configuration.clone()))
            .collect();
        sim.step_exact(5).unwrap();
        sim.reset().unwrap();
        for (m, c) in &ids {
            assert!(velocity_control_binding_matches(&sim, *m));
            near(
                pose(&sim, *m).translation,
                [(c.initial_linear[0] as usize - 1) as f64 * 20., 0., 10.],
            );
        }
        remove_model(&mut sim, ids[0].0).unwrap();
        sim.step_exact(2).unwrap();
        assert!(!velocity_control_binding_matches(&sim, ids[0].0));
        assert_eq!(
            initial_velocity_control_configurations_with_identity(&sim)
                .unwrap()
                .count(),
            8
        );
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(
            initial_velocity_control_configurations_with_identity(&sim)
                .unwrap()
                .map(|r| (r.id, r.configuration.clone()))
                .collect::<Vec<_>>(),
            ids
        );
        sim.step_exact(1).unwrap();
        for (m, _) in ids {
            assert!(velocity_control_binding_matches(&sim, m));
        }
    }
}
#[test]
fn m105_late_owner_drift_faults_before_any_native_velocity_replacement() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for fault in 0..3 {
            let mut sim = load(8, engine, false);
            let last = id(&sim, "default::r7");
            let e = entity_by_id(sim.world(), last).unwrap();
            let c = sim
                .world()
                .get::<VelocityControlConfiguration>(e)
                .unwrap()
                .clone();
            match fault {
                0 => {
                    sim.world_mut()
                        .entity_mut(e)
                        .remove::<VelocityControlConfiguration>();
                }
                1 => {
                    let mut c = c.clone();
                    c.topic.push_str("/foreign");
                    sim.world_mut().entity_mut(e).insert(c);
                }
                _ => {
                    let mut c = c.clone();
                    c.link = id(&sim, "default::r0::body");
                    sim.world_mut().entity_mut(e).insert(c);
                }
            }
            let before: Vec<_> = (0..8)
                .map(|i| {
                    let l = id(&sim, &format!("default::r{i}::body"));
                    (l, pose(&sim, l), velocity(&sim, l))
                })
                .collect();
            assert!(sim.step_exact(1).is_err());
            for (l, p, v) in before {
                assert_eq!(pose(&sim, l).translation, p.translation);
                near(velocity(&sim, l).linear, v.linear);
                near(velocity(&sim, l).angular, v.angular);
            }
        }
    }
}
#[test]
fn m105_capacity_and_closed_multilink_domains_remain_atomic() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        assert!(
            load_world_with_sdf_step(
                &xml(9, false),
                "nine",
                "default",
                &sdformat::NoResources,
                engine
            )
            .is_err()
        );
        let bad = xml(2, false).replacen("</link>", "</link><link name='extra'/>", 1);
        assert!(
            load_world_with_sdf_step(&bad, "multilink", "default", &sdformat::NoResources, engine)
                .is_err()
        );
    }
}
