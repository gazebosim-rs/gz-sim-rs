// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{
    LoadError, PhysicsEngine, WorldPhysics, load_world_with_engine, load_world_with_sdf_step,
    reset_loaded_world,
};
use sdformat::{NoResources, Resource, ResourceResolver};
use std::{
    sync::atomic::{AtomicUsize, Ordering},
    time::Duration,
};

fn world(profiles: &str) -> String {
    format!("<sdf version='1.12'><world name='default'>{profiles}</world></sdf>")
}

fn profiles() -> &'static str {
    "<physics name='first' type='ode'><max_step_size>0.002</max_step_size><real_time_factor>0</real_time_factor></physics><physics name='marked' type='custom' default='true'><max_step_size>0.005</max_step_size></physics>"
}

#[test]
fn first_profile_and_explicit_override_are_owned_and_restored_on_both_engines() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let xml = world(profiles());
        for override_step in [None, Some(Duration::from_millis(3))] {
            let mut sim = match override_step {
                None => {
                    load_world_with_sdf_step(&xml, "profiles.sdf", "default", &NoResources, engine)
                }
                Some(step) => load_world_with_engine(
                    &xml,
                    "profiles.sdf",
                    "default",
                    step,
                    &NoResources,
                    engine,
                ),
            }
            .unwrap();
            let step = override_step.unwrap_or(Duration::from_millis(2));
            assert_eq!(sim.step_duration(), step);
            let metadata = sim.world().resource::<WorldPhysics>();
            assert_eq!(metadata.profiles.len(), 2);
            assert_eq!(metadata.selected().unwrap().name, "first");
            assert!(metadata.profiles[1].is_default);
            assert_eq!(metadata.step_override, override_step);
            assert_eq!(
                metadata.selected().unwrap().source.as_ref().unwrap().uri,
                "profiles.sdf"
            );
            assert!(
                metadata
                    .selected()
                    .unwrap()
                    .xml
                    .as_ref()
                    .unwrap()
                    .contains("0.002")
            );
            sim.world_mut()
                .resource_mut::<WorldPhysics>()
                .profiles
                .clear();
            assert!(sim.world().resource::<WorldPhysics>().selected().is_none());
            sim.step_exact(3).unwrap();
            assert_eq!(sim.elapsed(), step * 3);
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(sim.step_duration(), step);
            assert_eq!(sim.elapsed(), Duration::ZERO);
            let metadata = sim.world().resource::<WorldPhysics>();
            assert_eq!(metadata.profiles.len(), 2);
            assert_eq!(metadata.selected().unwrap().max_step_size, 0.002);
            assert_eq!(metadata.step_override, override_step);
            sim.step_exact(1).unwrap();
            assert_eq!(sim.elapsed(), step);
        }
    }
}

#[test]
fn used_step_is_truncated_validated_and_can_be_overridden_without_rewriting_metadata() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let implicit =
            load_world_with_sdf_step(&world(""), "implicit", "default", &NoResources, engine)
                .unwrap();
        assert_eq!(implicit.step_duration(), Duration::from_millis(1));
        assert!(
            implicit
                .world()
                .resource::<WorldPhysics>()
                .selected()
                .unwrap()
                .source
                .is_none()
        );
        let tiny = world(
            "<physics name='tiny' type='ode'><max_step_size>0.0000000019</max_step_size></physics>",
        );
        assert_eq!(
            load_world_with_sdf_step(&tiny, "tiny", "default", &NoResources, engine)
                .unwrap()
                .step_duration(),
            Duration::from_nanos(1)
        );
        for value in ["0", "-1", "0.0000000005", "1e300", "9223372036.854776"] {
            let xml = world(&format!(
                "<physics name='invalid' type='ode'><max_step_size>{value}</max_step_size></physics>"
            ));
            let error = load_world_with_sdf_step(&xml, "invalid", "default", &NoResources, engine)
                .err()
                .unwrap();
            assert!(matches!(error, LoadError::Unsupported { .. }), "{error}");
            assert!(error.to_string().contains("unrepresentable physics step"));
            let sim = load_world_with_engine(
                &xml,
                "overridden",
                "default",
                Duration::from_millis(1),
                &NoResources,
                engine,
            )
            .unwrap();
            assert_eq!(sim.step_duration(), Duration::from_millis(1));
            assert_eq!(
                sim.world()
                    .resource::<WorldPhysics>()
                    .selected()
                    .unwrap()
                    .max_step_size,
                value.parse::<f64>().unwrap()
            );
        }
        let limited =
            world("<physics name='limited' type='ode'><max_contacts>1</max_contacts></physics>");
        let error = load_world_with_sdf_step(&limited, "limited", "default", &NoResources, engine)
            .err()
            .unwrap();
        assert!(error.to_string().contains("physics maximum contacts"));
    }
}

struct CountedResource(AtomicUsize);
impl ResourceResolver for CountedResource {
    fn resolve(&self, uri: &str, _parent: &str) -> Result<Resource, String> {
        assert_eq!(uri, "model://robot");
        self.0.fetch_add(1, Ordering::Relaxed);
        Ok(Resource { canonical_uri: "file:///robot.sdf".into(), xml: "<sdf version='1.12'><model name='robot'><static>true</static><link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></sdf>".into() })
    }
}

#[test]
fn profile_selection_parses_includes_only_once_and_reset_needs_no_resolver() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let xml = world(&format!(
            "{}<include><uri>model://robot</uri></include>",
            profiles()
        ));
        let mut sim = {
            let resolver = CountedResource(AtomicUsize::new(0));
            let sim =
                load_world_with_sdf_step(&xml, "one-read", "default", &resolver, engine).unwrap();
            assert_eq!(resolver.0.load(Ordering::Relaxed), 1);
            sim
        };
        sim.step_exact(3).unwrap();
        sim.world_mut().remove_resource::<WorldPhysics>();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(
            sim.world()
                .resource::<WorldPhysics>()
                .selected()
                .unwrap()
                .name,
            "first"
        );
        sim.step_exact(1).unwrap();
        assert_eq!(sim.elapsed(), Duration::from_millis(2));
    }
}

#[test]
fn actual_native_server_clock_controls_match_both_rust_backends() {
    let rows: Vec<serde_json::Value> =
        serde_json::from_str(include_str!("fixtures/world_physics.json")).unwrap();
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for row in &rows {
            let mut sim = load_world_with_sdf_step(
                row["xml"].as_str().unwrap(),
                "native-profile",
                "default",
                &NoResources,
                engine,
            )
            .unwrap();
            sim.step_exact(3).unwrap();
            let native = &row["native"];
            assert_eq!(
                sim.elapsed().as_nanos(),
                u128::from(native["time_ns"].as_u64().unwrap()),
                "{}",
                row["name"]
            );
            assert_eq!(
                sim.step_duration().as_nanos(),
                u128::from(native["dt_ns"].as_u64().unwrap())
            );
            assert_eq!(
                sim.world().resource::<gz_sim_core::UpdateInfo>().iteration,
                native["iterations"].as_u64().unwrap()
            );
        }
    }
}
