// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Loaded sensor scans, named template edits, identity and reset across native backends.
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, LidarPublication, LidarSample, Name, PhysicsEngine, WorldPose,
    load_world_with_sdf_step, model_state, prepare_model_template, remove_model,
    reset_loaded_world, set_lidar_interest, set_model_state, spawn_model_template,
};
use sdformat::NoResources;
const XML: &str = include_str!("../../../examples/multifidelity_lidar_edits.sdf");
fn named(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn pose(p: [f64; 3]) -> WorldPose {
    WorldPose {
        translation: p,
        rotation: [0., 0., 0., 1.],
    }
}
#[test]
fn translated_scans_capture_spawn_edit_contact_remove_and_reset_against_rapier() {
    let start = XML.find("      <plugin").unwrap();
    let end = XML.find("</plugin>").unwrap() + "</plugin>".len();
    let native_xml = [&XML[..start], &XML[end..]].concat();
    let mut traces = Vec::new();
    for (engine, xml) in [
        (PhysicsEngine::MultiFidelity, XML),
        (PhysicsEngine::Rapier, native_xml.as_str()),
    ] {
        let sdf = sdformat::load_str(xml, "scene.sdf", &NoResources).unwrap();
        let mut sim =
            load_world_with_sdf_step(xml, "scene.sdf", "default", &NoResources, engine).unwrap();
        let scanner = entity_by_id(sim.world(), named(&sim, "default::scanner::body")).unwrap();
        if engine == PhysicsEngine::Rapier {
            let body = sim.world().get::<BodyHandle>(scanner).unwrap().id();
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .set_world_velocity(body, [0.2, 0., 0.], [0.; 3])
                .unwrap();
        }
        let mount_id = named(&sim, "default::scanner::body::laser");
        let laser = entity_by_id(sim.world(), mount_id).unwrap();
        assert_eq!(
            sim.world()
                .get::<LidarPublication>(laser)
                .unwrap()
                .config
                .samples,
            31
        );
        set_lidar_interest(&mut sim, [laser]);
        let get_template = |name: &str| {
            let m = sdf.worlds[0]
                .models
                .iter()
                .find(|m| m.name == name)
                .unwrap();
            prepare_model_template(&sim, &sdf, m)
        };
        assert!(get_template("scanner").is_err());
        let sphere = get_template("inspection_sphere").unwrap();
        let wall = get_template("backstop").unwrap();
        let mut rows = Vec::new();
        let scan = |s: &Simulation| {
            let scan = s.world().get::<LidarSample>(laser).unwrap();
            assert_eq!(scan.time, s.elapsed());
            scan.ranges.clone()
        };
        let mut read = |s: &mut Simulation, ticks| {
            s.step_exact(ticks).unwrap();
            let beams = scan(s);
            let mid = beams[15];
            rows.push(beams);
            mid
        };
        let first = read(&mut sim, 2);
        assert!((first - 2.948).abs() < 1e-10, "{first}");
        let first_free = next_entity_id(sim.world()).unwrap();
        let clock = (sim.epoch(), sim.elapsed());
        let ball = spawn_model_template(&mut sim, &sphere, pose([0., 0., 0.45])).unwrap();
        assert_eq!(ball, first_free);
        assert_eq!((sim.epoch(), sim.elapsed()), clock);
        sim.tick().unwrap(); // paused, still the previous completed scan
        assert_eq!(scan(&sim)[15], first);
        assert!((read(&mut sim, 2) - 1.096).abs() < 1e-10);
        let mut state = model_state(&sim, ball).unwrap();
        state.linear_velocity = [4., 0., 0.];
        set_model_state(&mut sim, ball, state).unwrap();
        for _ in 0..45 {
            read(&mut sim, 2);
        }
        let v = model_state(&sim, ball).unwrap().linear_velocity[0];
        assert!(
            v.abs() < 0.02,
            "the sphere must strike the fixed backstop, v={v}"
        );
        let r = remove_model(&mut sim, ball).unwrap();
        assert_eq!(r, (ball..ball + 4).collect::<Vec<_>>());
        let restored_wall = read(&mut sim, 2);
        assert!((restored_wall - (2.95 - 0.2 * sim.elapsed().as_secs_f64())).abs() < 1e-10);
        let barrier = spawn_model_template(&mut sim, &wall, pose([0.15, 0., 0.45])).unwrap();
        let mut state = model_state(&sim, barrier).unwrap();
        state.translation[0] = 0.3;
        set_model_state(&mut sim, barrier, state).unwrap();
        assert!((read(&mut sim, 2) - (1.6 - 0.2 * sim.elapsed().as_secs_f64())).abs() < 1e-10);
        set_lidar_interest(&mut sim, []);
        sim.step_exact(2).unwrap();
        assert!(sim.world().get::<LidarSample>(laser).is_none());
        let high = next_entity_id(sim.world()).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(next_entity_id(sim.world()).unwrap(), high);
        assert!(entity_by_id(sim.world(), barrier).is_none());
        let laser = entity_by_id(sim.world(), mount_id).unwrap();
        assert!(sim.world().get::<LidarSample>(laser).is_none());
        assert_eq!(sim.epoch(), clock.0 + 1);
        if engine == PhysicsEngine::MultiFidelity {
            set_lidar_interest(&mut sim, [laser]);
            sim.step_exact(2).unwrap();
            assert_eq!(
                sim.world().get::<LidarSample>(laser).unwrap().ranges[15],
                first
            );
        }
        traces.push(rows);
    }
    let (mut finite, mut misses, mut worst) = (0, 0, 0f64);
    for (a, b) in traces[0].iter().flatten().zip(traces[1].iter().flatten()) {
        if a.is_finite() {
            assert!(b.is_finite());
            finite += 1;
            worst = worst.max((a - b).abs());
        } else {
            assert_eq!(a, b);
            misses += 1;
        }
    }
    eprintln!("Compared {finite} finite returns, {misses} misses, maximum difference {worst:e} m");
    assert!(finite > 500 && misses > 200 && worst < 1e-10);
}
#[test]
fn unsupported_geometry_and_prescribed_templates_remain_outside_the_edit_domain() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples")
        .canonicalize()
        .unwrap();
    let uri = format!("file://{}", root.join("mvp_mesh_collisions.sdf").display());
    let mesh = include_str!("../../../examples/mvp_mesh_collisions.sdf");
    assert!(matches!(
        load_world_with_sdf_step(
            mesh,
            &uri,
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity
        ),
        Err(gz_sim_scene::LoadError::Unsupported {
            feature: "collision geometry",
            ..
        })
    ));
    let start = XML.find("        <sensor").unwrap();
    let end = XML.find("</sensor>").unwrap() + "</sensor>".len();
    let xml = [&XML[..start], &XML[end..]].concat();
    let sim = load_world_with_sdf_step(
        &xml,
        "prescribed.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let id = named(&sim, "default::scanner");
    let next = next_entity_id(sim.world()).unwrap();
    assert!(matches!(
        model_state(&sim, id),
        Err(gz_sim_scene::WorldEditError::UnsupportedModel)
    ));
    let sdf = sdformat::load_str(&xml, "prescribed.sdf", &NoResources).unwrap();
    assert!(matches!(
        prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]),
        Err(gz_sim_scene::WorldEditError::UnsupportedModel)
    ));
    assert_eq!(next_entity_id(sim.world()).unwrap(), next);
    let soft=XML.replace("</world>",r#"<plugin filename="gz-sim-deformables-system" name="gz::sim::systems::Deformables"/></world>"#);
    assert!(matches!(
        load_world_with_sdf_step(
            &soft,
            "soft.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity
        ),
        Err(gz_sim_scene::LoadError::Unsupported {
            feature: "CPU lidar on deformable geometry",
            ..
        })
    ));
}
