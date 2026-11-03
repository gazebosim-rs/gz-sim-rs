// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::{EntityId, StepFault, assign_id};
use gz_sim_scene::{Name, VisualProperties};

fn rejected(sim: &Simulation, message: &str) {
    let before = (sim.epoch(), sim.elapsed(), sim.is_paused());
    let error = snapshot::extract(sim, "default", 0).err().unwrap();
    assert_eq!(error.kind(), io::ErrorKind::InvalidData);
    assert!(error.to_string().contains(message), "{error}");
    assert_eq!(before, (sim.epoch(), sim.elapsed(), sim.is_paused()));
}

#[test]
fn incomplete_visuals_never_produce_partial_snapshots() {
    let mut sim = simulation();
    let visual = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<EntityKind>() == Some(&EntityKind::Visual))
        .unwrap()
        .id();
    let initial = frame(&sim, 0);
    macro_rules! missing {
        ($component:ty, $message:literal) => {{
            let saved = sim
                .world_mut()
                .entity_mut(visual)
                .take::<$component>()
                .unwrap();
            rejected(&sim, $message);
            sim.world_mut().entity_mut(visual).insert(saved);
            assert_eq!(frame(&sim, 0), initial);
        }};
    }
    missing!(Name, "no name");
    missing!(WorldPose, "no world pose");
    missing!(VisualGeometry, "no geometry");
    missing!(VisualMaterial, "no material");
    missing!(VisualProperties, "no properties");
    // Removing identity also updates the ownership index; never move it back.
    sim.world_mut().entity_mut(visual).remove::<EntityId>();
    rejected(&sim, "no owned identity");
}

#[test]
fn world_identity_lights_and_faults_are_explicit_admission_failures() {
    let mut sim = simulation();
    let world = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<EntityKind>() == Some(&EntityKind::World))
        .unwrap()
        .id();
    let extra = sim.world_mut().spawn(EntityKind::World).id();
    assign_id(sim.world_mut(), extra).unwrap();
    rejected(&sim, "exactly one world");
    sim.world_mut().despawn(extra);
    for kind in [EntityKind::Light, EntityKind::LightVisual] {
        let light = sim.world_mut().spawn(kind).id();
        rejected(&sim, "does not lower SDF lights");
        sim.world_mut().despawn(light);
        assert_eq!(frame(&sim, 0)["visuals"].as_array().unwrap().len(), 21);
    }
    sim.world_mut().entity_mut(world).remove::<EntityId>();
    rejected(&sim, "world has no identity");
    sim.world_mut().entity_mut(world).remove::<EntityKind>();
    rejected(&sim, "no world identity");
    sim.world()
        .resource::<StepFault>()
        .report(io::Error::other("admission fixture fault"));
    rejected(&sim, "healthy named world");
}

#[test]
fn unsupported_materials_names_and_nonrepresentable_sizes_are_rejected() {
    let mut sim = simulation();
    let visual = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<EntityKind>() == Some(&EntityKind::Visual))
        .unwrap()
        .id();
    let material = sim.world().get::<VisualMaterial>(visual).unwrap().clone();
    let properties = sim.world().get::<VisualProperties>(visual).unwrap().clone();
    for field in 0..5 {
        let mut changed = material.clone();
        let mut flags = properties.clone();
        match field {
            0 => flags.0.transparency = 0.25,
            1 => flags.0.visibility_flags = 0,
            2 => {
                changed.0.script = Some(sdformat::MaterialScript {
                    uris: vec!["file://unsupported".into()],
                    name: "script".into(),
                    source: material.0.source.clone(),
                })
            }
            3 => {
                changed.0.pbr = Some(Box::new(sdformat::PbrMaterial {
                    metal: None,
                    specular: None,
                    source: material.0.source.clone(),
                }))
            }
            _ => changed.0.normal_map = "normal.png".into(),
        }
        sim.world_mut().entity_mut(visual).insert((changed, flags));
        rejected(&sim, "visible opaque diffuse primitives only");
    }
    sim.world_mut()
        .entity_mut(visual)
        .insert((material.clone(), properties));
    for color in [f32::NAN, -0.1, 1.1] {
        sim.world_mut()
            .get_mut::<VisualMaterial>(visual)
            .unwrap()
            .0
            .diffuse[0] = color;
        rejected(&sim, "invalid pose, name or opaque color");
    }
    sim.world_mut().entity_mut(visual).insert(material);
    let name = Name(sim.world().get::<Name>(visual).unwrap().0.clone());
    sim.world_mut()
        .entity_mut(visual)
        .insert(Name("é".repeat(129)));
    rejected(&sim, "invalid pose, name or opaque color");
    sim.world_mut().entity_mut(visual).insert(name);
    let geometry = sim.world().get::<VisualGeometry>(visual).unwrap().clone();
    for radius in [f64::MIN_POSITIVE, f64::MAX, f64::NAN] {
        sim.world_mut()
            .entity_mut(visual)
            .insert(VisualGeometry(Arc::new(sdformat::Geometry::Sphere {
                radius,
            })));
        rejected(&sim, "finite box and sphere visuals only");
    }
    sim.world_mut().entity_mut(visual).insert(geometry);
    assert_eq!(frame(&sim, 0)["visuals"].as_array().unwrap().len(), 21);
}

#[test]
fn visual_count_and_escaped_json_size_are_independent_hard_limits() {
    let mut sim = simulation();
    let visual = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<EntityKind>() == Some(&EntityKind::Visual))
        .unwrap();
    let pose = *visual.get::<WorldPose>().unwrap();
    let geometry = visual.get::<VisualGeometry>().unwrap().clone();
    let material = visual.get::<VisualMaterial>().unwrap().clone();
    let properties = visual.get::<VisualProperties>().unwrap().clone();
    for _ in 21..128 {
        let entity = sim
            .world_mut()
            .spawn((
                EntityKind::Visual,
                Name("extra".into()),
                pose,
                geometry.clone(),
                material.clone(),
                properties.clone(),
            ))
            .id();
        assign_id(sim.world_mut(), entity).unwrap();
    }
    let scene = snapshot::extract(&sim, "default", 0).unwrap();
    assert!(snapshot::encode(&scene).is_ok());
    assert_eq!(frame(&sim, 0)["visuals"].as_array().unwrap().len(), 128);
    let entities: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|entity| entity.get::<EntityKind>() == Some(&EntityKind::Visual))
        .map(|entity| entity.id())
        .collect();
    for entity in entities {
        sim.world_mut()
            .entity_mut(entity)
            .insert(Name("\\".repeat(256)));
    }
    let scene = snapshot::extract(&sim, "default", 0).unwrap();
    assert!(serde_json::to_string(&scene).unwrap().len() > snapshot::MAX_BYTES);
    let error = snapshot::encode(&scene).unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::InvalidData);
    assert!(error.to_string().contains("64 KiB"));
    let extra = sim
        .world_mut()
        .spawn((
            EntityKind::Visual,
            Name("extra".into()),
            pose,
            geometry,
            material,
            properties,
        ))
        .id();
    assign_id(sim.world_mut(), extra).unwrap();
    rejected(&sim, "exceeds 128 visuals");
}
