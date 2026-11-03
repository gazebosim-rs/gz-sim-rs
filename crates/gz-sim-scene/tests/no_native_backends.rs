// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../build.rs"]
mod build_script;

#[cfg(not(feature = "native-backends"))]
use gz_sim_scene::{LoadError, PhysicsEngine, load_world_with_engine};
#[cfg(not(feature = "native-backends"))]
use sdformat::NoResources;
#[cfg(not(feature = "native-backends"))]
use std::time::Duration;

#[cfg(not(feature = "native-backends"))]
const XML: &str = "<sdf version='1.12'><world name='default'><gravity>0 0 -9.81</gravity><model name='ball'><pose>0 0 5 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>0.2</ixx><iyy>0.2</iyy><izz>0.2</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></world></sdf>";

#[test]
#[cfg(not(feature = "native-backends"))]
fn absent_native_backends_return_the_selected_engine_without_fallback() {
    for engine in [PhysicsEngine::Dart, PhysicsEngine::Mujoco] {
        let result = load_world_with_engine(
            XML,
            "no-native.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            engine,
        );
        let error = match result {
            Err(error) => error,
            Ok(_) => panic!("disabled native engine silently loaded a different backend"),
        };
        assert!(matches!(error, LoadError::BackendUnavailable(selected) if selected == engine));
        assert!(error.to_string().contains("native-backends"));
    }
}

#[test]
#[cfg(not(feature = "native-backends"))]
fn rapier_and_tpe_still_load_and_step_without_native_engines() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut simulation = load_world_with_engine(
            XML,
            "no-native.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            engine,
        )
        .unwrap();
        simulation.step_exact(10).unwrap();
        assert_eq!(simulation.elapsed(), Duration::from_millis(10));
        assert_eq!(simulation.epoch(), 0);
        let pose = simulation
            .world()
            .iter_entities()
            .find_map(|entity| {
                let name = entity.get::<gz_sim_scene::Name>()?;
                (name.0 == "default::ball::body")
                    .then(|| *entity.get::<gz_sim_scene::WorldPose>().unwrap())
            })
            .unwrap();
        if engine == PhysicsEngine::Rapier {
            assert!(pose.translation[2] < 5.0);
        } else {
            // TPE retains its existing model-motion proxy, without dynamic gravity.
            assert_eq!(pose.translation, [0., 0., 5.]);
        }
    }
}

#[test]
fn disabled_native_build_scripts_ignore_absent_or_ambient_native_metadata() {
    assert_eq!(build_script::configured_link_directives(false, None), "");
    assert_eq!(
        build_script::configured_link_directives(false, Some("/unselected/native/sdk".into())),
        ""
    );
}

#[test]
fn enabled_native_build_scripts_keep_verified_rpath_and_missing_metadata_failure() {
    assert_eq!(
        build_script::configured_link_directives(true, Some("/opt/dart/lib".into())),
        "cargo:rustc-link-arg=-Wl,--disable-new-dtags\ncargo:rustc-link-arg=-Wl,-rpath,/opt/dart/lib\n"
    );
    assert!(
        std::panic::catch_unwind(|| build_script::configured_link_directives(true, None)).is_err()
    );
}
