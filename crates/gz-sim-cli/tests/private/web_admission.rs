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
fn camera_fixture_visual_flags_keep_exact_owned_primitive_scene() {
    let source = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../examples/m76_camera_projection_offsets.sdf"
    ));
    let load = |source: &str| {
        gz_sim_scene::load_world(
            source,
            "m76_camera_projection_offsets.sdf",
            "default",
            Duration::from_millis(1),
            &sdformat::NoResources,
        )
        .unwrap()
    };
    let sim = load(source);
    let expected = [
        (
            "default::backdrop::body::surface",
            [6.0, 0.0, 1.0],
            [0.2, 16.0, 16.0],
            [0.0, 0.0, 1.0, 1.0],
            4,
        ),
        (
            "default::occluder_a::body::surface",
            [2.0, -2.0, 1.0],
            [0.4, 0.4, 0.4],
            [1.0, 0.0, 0.0, 1.0],
            1,
        ),
        (
            "default::occluder_b::body::surface",
            [3.0, 2.0, 1.0],
            [0.4, 0.4, 0.4],
            [0.0, 1.0, 0.0, 1.0],
            2,
        ),
    ];
    let mut rows = Vec::new();
    for (name, translation, size, color, flags) in expected {
        let matches: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|entity| {
                entity.get::<EntityKind>() == Some(&EntityKind::Visual)
                    && entity.get::<Name>().is_some_and(|value| value.0 == name)
            })
            .collect();
        assert_eq!(matches.len(), 1);
        let entity = &matches[0];
        let id = entity.get::<EntityId>().unwrap().get();
        assert_eq!(
            gz_sim_core::entity_by_id(sim.world(), id),
            Some(entity.id())
        );
        assert_eq!(
            entity.get::<VisualProperties>().unwrap().0.visibility_flags,
            flags
        );
        rows.push(json!({
            "id": id,
            "name": name,
            "translation": translation,
            "rotation": [0.0, 0.0, 0.0, 1.0],
            "shape": {"kind": "box", "size": size},
            "color": color,
        }));
    }
    rows.sort_unstable_by_key(|row| row["id"].as_u64().unwrap());
    assert!(rows.windows(2).all(|pair| pair[0]["id"] != pair[1]["id"]));
    assert_eq!(frame(&sim, 7)["visuals"], json!(rows));
    let all_visible = source
        .replace(
            "<visibility_flags>4</visibility_flags>",
            "<visibility_flags>4294967295</visibility_flags>",
        )
        .replace(
            "<visibility_flags>1</visibility_flags>",
            "<visibility_flags>4294967295</visibility_flags>",
        )
        .replace(
            "<visibility_flags>2</visibility_flags>",
            "<visibility_flags>4294967295</visibility_flags>",
        );
    assert_eq!(
        snapshot::encode(&snapshot::extract(&sim, "default", 7).unwrap()).unwrap(),
        snapshot::encode(&snapshot::extract(&load(&all_visible), "default", 7).unwrap()).unwrap()
    );
}

#[test]
fn nonzero_visibility_preserves_default_bytes_without_relaxing_opacity() {
    let mut sim = simulation();
    let visual = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<EntityKind>() == Some(&EntityKind::Visual))
        .unwrap()
        .id();
    let properties = sim.world().get::<VisualProperties>(visual).unwrap().clone();
    let material = sim.world().get::<VisualMaterial>(visual).unwrap().clone();
    let initial = snapshot::encode(&snapshot::extract(&sim, "default", 0).unwrap()).unwrap();
    // The viewer's implicit mask is all bits, unlike the bounded camera mask.
    for flags in [1, 2, 4, 0x0fff_ffff, 0x1000_0000, 0x8000_0000, u32::MAX] {
        sim.world_mut()
            .get_mut::<VisualProperties>(visual)
            .unwrap()
            .0
            .visibility_flags = flags;
        assert_eq!(
            snapshot::encode(&snapshot::extract(&sim, "default", 0).unwrap()).unwrap(),
            initial
        );
    }
    sim.world_mut()
        .get_mut::<VisualProperties>(visual)
        .unwrap()
        .0
        .visibility_flags = 0;
    rejected(&sim, "visible opaque diffuse primitives only");
    sim.world_mut()
        .get_mut::<VisualProperties>(visual)
        .unwrap()
        .0
        .visibility_flags = 1;
    sim.world_mut()
        .get_mut::<VisualProperties>(visual)
        .unwrap()
        .0
        .transparency = 0.25;
    rejected(&sim, "visible opaque diffuse primitives only");
    sim.world_mut()
        .get_mut::<VisualProperties>(visual)
        .unwrap()
        .0
        .transparency = 0.0;
    sim.world_mut()
        .get_mut::<VisualMaterial>(visual)
        .unwrap()
        .0
        .diffuse[3] = 0.5;
    rejected(&sim, "invalid pose, name or opaque color");
    sim.world_mut()
        .entity_mut(visual)
        .insert((properties, material));
    assert_eq!(
        snapshot::encode(&snapshot::extract(&sim, "default", 0).unwrap()).unwrap(),
        initial
    );
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
