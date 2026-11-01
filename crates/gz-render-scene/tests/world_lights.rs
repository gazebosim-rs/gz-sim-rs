// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_scene::{SnapshotError, extract, extract_camera};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    CameraSensor, EntityKind, Name, PhysicsEngine, WorldLight, load_world_with_sdf_step,
    reset_loaded_world,
};
use sdformat::NoResources;

fn scene(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "render-light.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap()
}
#[test]
fn every_loaded_light_type_and_disabled_light_refuses_rendering() {
    for kind in ["point", "spot", "directional"] {
        for on in ["true", "false"] {
            let xml = format!(
                "<sdf version='1.12'><world name='default'><light name='lamp' type='{kind}'><light_on>{on}</light_on></light></world></sdf>"
            );
            let sim = scene(&xml);
            let counts = sim.scheduled_system_counts();
            assert_eq!(
                extract(&sim, 1, [0.; 3]),
                Err(SnapshotError::UnsupportedLighting)
            );
            assert_eq!(sim.scheduled_system_counts(), counts);
            assert_eq!(
                extract(&sim, 0, [0.; 3]),
                Err(SnapshotError::InvalidRequest)
            );
        }
    }
}
#[test]
fn light_identity_or_metadata_remains_fail_closed_across_removal_and_reset() {
    let mut sim = scene(
        "<sdf version='1.12'><world name='default'><light name='lamp' type='point'/></world></sdf>",
    );
    let light = sim
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<WorldLight>())
        .unwrap()
        .id();
    let metadata = sim
        .world_mut()
        .entity_mut(light)
        .take::<WorldLight>()
        .unwrap();
    assert_eq!(
        extract(&sim, 1, [0.; 3]),
        Err(SnapshotError::UnsupportedLighting)
    );
    sim.world_mut()
        .entity_mut(light)
        .remove::<EntityKind>()
        .insert(metadata);
    assert_eq!(
        extract(&sim, 2, [0.; 3]),
        Err(SnapshotError::UnsupportedLighting)
    );
    sim.world_mut().entity_mut(light).despawn();
    assert!(extract(&sim, 3, [0.; 3]).is_ok());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        extract(&sim, 4, [0.; 3]),
        Err(SnapshotError::UnsupportedLighting)
    );
}
#[test]
fn actual_loaded_camera_also_refuses_world_lighting_before_snapshot_creation() {
    let base = include_str!("../../gz-sim-scene/tests/fixtures/camera-binding.sdf");
    let xml = base.replace("</world>", "<light name='sun' type='directional'/></world>");
    let sim = scene(&xml);
    let camera = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<CameraSensor>())
        .unwrap();
    assert_eq!(
        camera.get::<Name>().unwrap().0,
        "default::robot::body::camera"
    );
    let id = camera.get::<EntityId>().unwrap().get();
    assert_eq!(
        extract_camera(&sim, id, 1, [0.; 3]),
        Err(SnapshotError::UnsupportedLighting)
    );
}

#[test]
fn orphan_native_light_visual_identity_is_not_renderable_geometry() {
    let mut sim = scene("<sdf version='1.12'><world name='default'/></sdf>");
    sim.world_mut().spawn(EntityKind::LightVisual);
    assert_eq!(
        extract(&sim, 1, [0.; 3]),
        Err(SnapshotError::UnsupportedLighting)
    );
}
