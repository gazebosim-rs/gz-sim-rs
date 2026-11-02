// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Loaded-scene controls for cold, atomic two-edge DART assembly.
use gz_physics::{PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, LoadError, Name, PhysicsEngine, WorldPose,
    load_world_with_sdf_step, queue_joint_effort, remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::{collections::BTreeMap, time::Duration};

fn xml(reverse: bool, external: bool) -> String {
    let link = |name: &str, z: usize| {
        format!(
            "<link name='{name}'><pose>0 0 {z} 0 0 0</pose><self_collide>false</self_collide>\
             <inertial><pose>0 0 0.1 0 0 0</pose><mass>1</mass>\
             <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>\
             <collision name='shape'><geometry><sphere><radius>0.6</radius></sphere>\
             </geometry></collision></link>"
        )
    };
    let first = "<joint name='first' type='revolute'><parent>root</parent><child>middle</child>\
                 <axis><xyz>1 0 0</xyz></axis></joint>";
    let second = "<joint name='second' type='revolute'><parent>middle</parent><child>leaf</child>\
                  <axis><xyz>1 0 0</xyz></axis></joint>";
    let joints = if reverse {
        format!("{second}{first}")
    } else {
        format!("{first}{second}")
    };
    // This model is deliberately constructed first. It stays external after
    // assembly, unlike the serial model's intended middle and leaf bodies.
    let external = if external {
        format!(
            "<model name='external'><pose>2 3 4 0.1 0.2 0.3</pose>{}</model>",
            link("body", 1)
        )
    } else {
        String::new()
    };
    format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity>{external}\
         <model name='chain'><pose>2 3 4 0.1 0.2 0.3</pose>{}{}{}{joints}</model>\
         </world></sdf>",
        link("root", 0),
        link("middle", 1),
        link("leaf", 2)
    )
}

fn identities(simulation: &Simulation) -> BTreeMap<String, u64> {
    let mut identities = BTreeMap::new();
    for entity in simulation.world().iter_entities() {
        match (entity.get::<Name>(), entity.get::<EntityId>()) {
            (Some(name), Some(id)) => {
                assert!(identities.insert(name.0.clone(), id.get()).is_none());
            }
            (None, None) => {
                // Bevy owns internal system entities outside the public scene.
                assert!(!entity.contains::<BodyHandle>());
                assert!(!entity.contains::<JointHandle>());
                assert!(!entity.contains::<WorldPose>());
            }
            _ => panic!("public scene identity must have both a name and an ID"),
        }
    }
    identities
}

fn close(a: impl IntoIterator<Item = f64>, b: impl IntoIterator<Item = f64>) {
    let a: Vec<_> = a.into_iter().collect();
    let b: Vec<_> = b.into_iter().collect();
    assert_eq!(a.len(), b.len());
    for (a, b) in a.into_iter().zip(b) {
        assert!((a - b).abs() <= 1e-12, "{a:.17e} != {b:.17e}");
    }
}

#[test]
fn intended_touching_links_assemble_in_either_declaration_order_and_reset() {
    for (reverse, rotated_joint) in [(false, false), (true, false), (false, true), (true, true)] {
        let xml = xml(reverse, false);
        // Both child-frame admission and the retained world-frame fallback
        // must preserve the same complete lifecycle and effort controls.
        let xml = if rotated_joint {
            xml.replace("<axis>", "<pose>0 0 0 0 0 0.25</pose><axis>")
        } else {
            xml
        };
        let parsed = sdformat::load_str(&xml, "serial.sdf", &NoResources).unwrap();
        let mut simulation =
            load_world_with_sdf_step(&xml, "serial.sdf", "w", &NoResources, PhysicsEngine::Dart)
                .unwrap();
        let original = identities(&simulation);
        assert_eq!(original.len(), 10);
        let ordered = if reverse {
            ["w::chain::second", "w::chain::first"]
        } else {
            ["w::chain::first", "w::chain::second"]
        };
        assert!(original[ordered[0]] < original[ordered[1]]);
        for epoch in 0..2 {
            assert_eq!(simulation.elapsed(), Duration::ZERO);
            assert_eq!(simulation.epoch(), epoch);
            assert_eq!(identities(&simulation), original);
            for name in ["w::chain::root", "w::chain::middle", "w::chain::leaf"] {
                let entity = entity_by_id(simulation.world(), original[name]).unwrap();
                let handle = simulation.world().get::<BodyHandle>(entity).unwrap();
                let state = simulation
                    .world()
                    .resource::<PhysicsBackend>()
                    .state(handle.id())
                    .unwrap();
                let expected = parsed.pose_of(name, "w").unwrap();
                close(state.position, expected.translation);
                close(state.orientation, expected.rotation);
                close(state.velocity, [0.0; 3]);
                close(state.angular_velocity, [0.0; 3]);
                let pose = simulation.world().get::<WorldPose>(entity).unwrap();
                close(pose.translation, state.position);
                close(pose.rotation, state.orientation);
            }
            for tick in 1..=10 {
                for (name, child, effort) in [
                    ("w::chain::first", "w::chain::middle", 0.25),
                    ("w::chain::second", "w::chain::leaf", -0.15),
                ] {
                    let id = original[name];
                    let entity = entity_by_id(simulation.world(), id).unwrap();
                    let joint = simulation.world().get::<JointHandle>(entity).unwrap();
                    assert_eq!(joint.child(), original[child]);
                    queue_joint_effort(&mut simulation, id, effort).unwrap();
                }
                simulation.step_exact(1).unwrap();
                assert_eq!(simulation.elapsed(), Duration::from_millis(tick));
                for name in ["w::chain::first", "w::chain::second"] {
                    let entity = entity_by_id(simulation.world(), original[name]).unwrap();
                    let sample = simulation.world().get::<JointSample>(entity).unwrap();
                    assert_eq!(sample.time, simulation.elapsed());
                    assert_eq!(sample.epoch, epoch);
                    assert!(sample.state.position.is_finite() && sample.state.velocity.is_finite());
                    if tick == 10 {
                        assert_ne!(sample.state.position, 0.0);
                        assert_ne!(sample.state.velocity, 0.0);
                    }
                }
            }
            reset_loaded_world(&mut simulation, |_| ()).unwrap();
        }
        remove_model(&mut simulation, original["w::chain"]).unwrap();
        assert!(simulation.world().iter_entities().all(|entity| {
            !entity.contains::<BodyHandle>() && !entity.contains::<JointHandle>()
        }));
        simulation.step_exact(1).unwrap();
        reset_loaded_world(&mut simulation, |_| ()).unwrap();
        assert_eq!(identities(&simulation), original);
    }
}

#[test]
fn final_contact_with_an_external_dynamic_body_remains_refused() {
    for reverse in [false, true] {
        let result = load_world_with_sdf_step(
            &xml(reverse, true),
            "serial-external.sdf",
            "w",
            &NoResources,
            PhysicsEngine::Dart,
        );
        assert!(matches!(
            result,
            Err(LoadError::Physics(PhysicsError::InvalidJoint))
        ));
    }
}
