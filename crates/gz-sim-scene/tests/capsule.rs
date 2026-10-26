// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::EntityId;
use gz_sim_scene::{
    ImuSample, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_engine, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

#[test]
fn centered_capsules_load_default_input_imu_and_loaded_reset() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for geometry in [
            "<capsule/>",
            "<capsule><radius>0.6</radius><length>2.8</length></capsule>",
        ] {
            let fixture = include_str!("fixtures/fall.sdf");
            let sphere = "<sphere><radius>0.5</radius></sphere>";
            assert_eq!(fixture.matches(sphere).count(), 1);
            let xml = fixture.replace(sphere, geometry);
            assert!(xml.contains(geometry));
            let mut sim = load_world_with_engine(
                &xml,
                "capsule.sdf",
                "default",
                Duration::from_millis(1),
                &NoResources,
                engine,
            )
            .unwrap();
            let mut original_id = None;
            for epoch in 0..2 {
                sim.step_exact(100).unwrap();
                let entity = sim
                    .world()
                    .iter_entities()
                    .find(|e| {
                        e.get::<Name>()
                            .is_some_and(|n| n.0 == "default::ball::body")
                    })
                    .unwrap();
                let id = entity.get::<EntityId>().unwrap().get();
                if let Some(original) = original_id {
                    assert_eq!(id, original);
                } else {
                    original_id = Some(id);
                }
                let pose = entity.get::<WorldPose>().unwrap();
                let velocity = entity.get::<Velocity>().unwrap();
                let z = if engine == PhysicsEngine::Tpe {
                    5.0
                } else {
                    4.95095
                };
                assert!((pose.translation[2] - z).abs() < 0.001);
                assert_eq!(&pose.translation[..2], &[0.0; 2]);
                assert!(
                    (velocity.linear[2]
                        - if engine == PhysicsEngine::Tpe {
                            0.0
                        } else {
                            -0.981
                        })
                    .abs()
                        < 1e-10
                );
                let sample = sim
                    .world_mut()
                    .query::<&ImuSample>()
                    .single(sim.world())
                    .unwrap();
                assert_eq!(sample.time, Duration::from_millis(100));
                assert_eq!(sample.epoch, epoch);
                let reading = sample.reading.as_ref().unwrap();
                assert_eq!(reading.angular_velocity, [0.0; 3]);
                let force = if engine == PhysicsEngine::Tpe {
                    9.81
                } else {
                    0.0
                };
                assert!((reading.specific_force[2] - force).abs() < 1e-10);
                assert_eq!(reading.orientation, [0.0, 0.0, 0.0, 1.0]);
                reset_loaded_world(&mut sim, |_| ()).unwrap();
                assert_eq!(sim.elapsed(), Duration::ZERO);
            }
        }
    }
}

fn contact_xml(mode: u64) -> String {
    let b_pose = match mode {
        1 => "1.2 0 0 0 0 0",
        2 => "1.201 0 0 0 0 0",
        3 => "0 0 4.0 0 0 0",
        4 => "0.5 0.2 0.1 0 0 0",
        5 => "0.5 0.2 0.1 0.3 -0.2 0.5",
        _ => "1 0 0 0 0 0",
    };
    let link_pose = if mode == 4 || mode == 5 {
        "0.25 0.1 0.2 0.4 0.2 0.7"
    } else {
        "0 0 0 0 0 0"
    };
    let link = |pose: &str| {
        format!(
            "<link name='body'><pose>{pose}</pose><collision name='c'><geometry><capsule><radius>0.6</radius><length>2.8</length></capsule></geometry></collision></link>"
        )
    };
    format!(
        "<sdf version='1.12'><world name='default'><model name='a'><static>{}</static>{}</model><model name='b'><static>{}</static><pose>{b_pose}</pose>{}</model></world></sdf>",
        mode == 6 || mode == 7,
        link("0 0 0 0 0 0"),
        mode == 6,
        link(link_pose)
    )
}

#[test]
fn loaded_capsule_contacts_match_eight_fresh_native_tpe_controls() {
    use gz_physics_tpe::Contacts;
    let reference: serde_json::Value =
        serde_json::from_str(include_str!("../../../planning/reference/capsule.json")).unwrap();
    let rows = reference["controls"]["rows"].as_array().unwrap();
    assert_eq!(rows.len(), 8);
    for (mode, row) in rows.iter().enumerate() {
        assert_eq!(row["mode"].as_u64().unwrap(), mode as u64);
        let mode = row["mode"].as_u64().unwrap();
        let mut sim = load_world_with_engine(
            &contact_xml(mode),
            "capsule-contacts.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            PhysicsEngine::Tpe,
        )
        .unwrap();
        sim.step_exact(1).unwrap();
        let actual = sim.world().resource::<Contacts>().as_slice();
        let expected = row["contacts"].as_array().unwrap();
        assert_eq!(actual.len(), expected.len(), "mode={mode}");
        for (actual, expected) in actual.iter().zip(expected) {
            for (actual, expected) in actual.point.iter().zip(expected.as_array().unwrap()) {
                assert!(
                    (actual - expected.as_f64().unwrap()).abs() < 1e-12,
                    "mode={mode}: {row}"
                );
            }
        }
        let ids: Vec<_> = sim
            .world()
            .iter_entities()
            .filter_map(|e| {
                let name = e.get::<Name>()?;
                (name.0 == "default::a" || name.0 == "default::b")
                    .then(|| (name.0.clone(), e.get::<EntityId>().unwrap().get()))
            })
            .collect();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        for (name, id) in ids {
            let entity = sim
                .world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap();
            assert_eq!(entity.get::<EntityId>().unwrap().get(), id);
        }
        let restored = sim.world().resource::<Contacts>().as_slice();
        assert_eq!(restored.len(), expected.len());
        for (contact, expected) in restored.iter().zip(expected) {
            for (actual, expected) in contact.point.iter().zip(expected.as_array().unwrap()) {
                assert!(
                    (actual - expected.as_f64().unwrap()).abs() < 1e-12,
                    "reset mode={mode}"
                );
            }
        }
    }
}
