// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{Geometry, Request};
use gz_render_scene::{SnapshotError, extract, extract_camera};
use gz_sim_core::assign_id;
use gz_sim_scene::{
    CameraSensor, EntityKind, PhysicsEngine, VisualGeometry, WorldPose, load_world_with_sdf_step,
};
use sdformat::{Camera, NoResources, Sensor, SensorConfig};
use std::sync::Arc;
fn scene() -> (gz_sim_core::Simulation, u64) {
    let xml = "<sdf version='1.12'><world name='default'><model name='object'><static>true</static><pose>4 0 0 0.2 0.3 0.4</pose><link name='body'><collision name='body'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='capsule'><geometry><capsule><radius>0.5</radius><length>2</length></capsule></geometry><material><diffuse>1 0 0 1</diffuse></material></visual></link></model></world></sdf>";
    let mut sim =
        load_world_with_sdf_step(xml, "capsule", "default", &NoResources, PhysicsEngine::Tpe)
            .unwrap();
    let sensor = Sensor {
        kind: "camera".into(),
        config: SensorConfig::Camera(Box::new(Camera {
            image_width: 64,
            image_height: 64,
            horizontal_fov: std::f64::consts::FRAC_PI_2,
            ..Default::default()
        })),
        ..Default::default()
    };
    let e = sim
        .world_mut()
        .spawn((
            EntityKind::Sensor,
            CameraSensor(Arc::new(sensor)),
            WorldPose {
                translation: [0.; 3],
                rotation: [0., 0., 0., 1.],
            },
        ))
        .id();
    let id = assign_id(sim.world_mut(), e).unwrap();
    (sim, id)
}
#[test]
fn capsule_extraction_keeps_exact_geometry_local_axis_and_visual_world_frame() {
    let (sim, id) = scene();
    let Request::CalibratedShapeSnapshot { visuals, .. } =
        extract_camera(&sim, id, 1, [0.; 3]).unwrap()
    else {
        panic!("shape route")
    };
    assert_eq!(visuals.len(), 1);
    assert_eq!(
        visuals[0].geometry,
        Geometry::Capsule {
            radius: 0.5,
            length: 2.
        }
    );
    assert_eq!(visuals[0].translation, [4., 0., 0.]);
    let visual = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<VisualGeometry>())
        .unwrap();
    assert_eq!(
        visuals[0].rotation,
        visual.get::<WorldPose>().unwrap().rotation
    );
    assert!(matches!(
        extract(&sim, 2, [0.; 3]),
        Err(SnapshotError::UnsupportedGeometry(_))
    ));
}
#[test]
fn capsule_invalid_geometry_is_validated_before_camera_visibility_selection() {
    let (mut sim, id) = scene();
    let camera = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
    {
        let mut sensor = sim.world_mut().get_mut::<CameraSensor>(camera).unwrap();
        let SensorConfig::Camera(camera) = &mut Arc::make_mut(&mut sensor.0).config else {
            panic!("camera")
        };
        camera.visibility_mask = 0;
    }
    let Request::CalibratedSnapshot { visuals, .. } = extract_camera(&sim, id, 2, [0.; 3]).unwrap()
    else {
        panic!("empty hidden route")
    };
    assert!(visuals.is_empty());
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<VisualGeometry>())
        .unwrap()
        .id();
    for (radius, length) in [
        (0., 1.),
        (-1., 1.),
        (1., 0.),
        (1., f64::INFINITY),
        (f64::MAX, 1.),
        (1e-300, 1.),
    ] {
        sim.world_mut().get_mut::<VisualGeometry>(e).unwrap().0 =
            Arc::new(sdformat::Geometry::Capsule { radius, length });
        assert!(matches!(
            extract_camera(&sim, id, 3, [0.; 3]),
            Err(SnapshotError::InvalidVisual(_))
        ));
    }
}
