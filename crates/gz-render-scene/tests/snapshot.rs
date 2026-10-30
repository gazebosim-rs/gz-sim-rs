// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use gz_render_protocol::{Request, Stamp};
use gz_render_scene::{SnapshotError, extract};
use gz_sim_core::{EntityId, Simulation, StepFault, assign_id};
use gz_sim_scene::{
    EntityKind, Name, PhysicsEngine, VisualGeometry, VisualMaterial, VisualProperties, WorldPose,
    load_world_with_sdf_step, remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::sync::Arc;

fn scene() -> Simulation {
    let xml = r#"<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity>
      <model name='robot'><static>true</static><pose>1000000000 2 0 0 0 1.5707963267948966</pose>
      <link name='link'><collision name='collision'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><visual name='view'><pose>1 0 0 0 0 0</pose>
      <geometry><box><size>1 2 3</size></box></geometry>
      <material><diffuse>0.2 0.4 0.6 1</diffuse></material>
      </visual></link></model></world></sdf>"#;
    load_world_with_sdf_step(xml, "snapshot", "default", &NoResources, PhysicsEngine::Tpe).unwrap()
}
fn named(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn parts(request: Request) -> (Stamp, Vec<gz_render_protocol::Visual>) {
    let Request::Snapshot { stamp, visuals, .. } = request else {
        panic!("expected snapshot")
    };
    (stamp, visuals)
}

#[test]
fn committed_source_lifecycle_and_frame_data() {
    let mut sim = scene();
    let counts = sim.scheduled_system_counts();
    let model = named(&sim, "default::robot");
    let model_id = sim.world().get::<EntityId>(model).unwrap().get();
    let visual = named(&sim, "default::robot::link::view");
    let id = sim.world().get::<EntityId>(visual).unwrap().get();
    let origin = [1_000_000_000.0, 0.0, 0.0];
    let request = extract(&sim, 1, origin).unwrap();
    assert!(request.wire_valid());
    let (stamp, visuals) = parts(request);
    assert_eq!(
        stamp,
        Stamp {
            epoch: 0,
            seconds: 0,
            nanoseconds: 0
        }
    );
    assert_eq!(visuals.len(), 1);
    assert_eq!(visuals[0].id, id);
    assert!((visuals[0].translation[0] - 1_000_000_000.0).abs() < 1e-7);
    assert!((visuals[0].translation[1] - 3.0).abs() < 1e-12);
    assert!((visuals[0].rotation[2] - 0.5_f64.sqrt()).abs() < 1e-12);
    assert_eq!(visuals[0].size, [1.0, 2.0, 3.0]);
    assert_eq!(visuals[0].color, [0.2, 0.4, 0.6]);
    assert_eq!(sim.scheduled_system_counts(), counts);
    sim.step_exact(1).unwrap();
    assert_eq!(
        parts(extract(&sim, 2, origin).unwrap()).0.nanoseconds,
        sim.elapsed().subsec_nanos()
    );
    remove_model(&mut sim, model_id).unwrap();
    assert!(parts(extract(&sim, 3, origin).unwrap()).1.is_empty());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let (stamp, visuals) = parts(extract(&sim, 4, origin).unwrap());
    assert_eq!(
        stamp,
        Stamp {
            epoch: 1,
            seconds: 0,
            nanoseconds: 0
        }
    );
    assert_eq!(visuals[0].id, id);
}

#[test]
fn refuses_bad_request_missing_components_and_faulted_state() {
    let mut sim = scene();
    assert_eq!(
        extract(&sim, 0, [0.0; 3]),
        Err(SnapshotError::InvalidRequest)
    );
    assert_eq!(
        extract(&sim, 1, [f64::NAN, 0.0, 0.0]),
        Err(SnapshotError::InvalidRequest)
    );
    let visual = named(&sim, "default::robot::link::view");
    let id = sim.world().get::<EntityId>(visual).unwrap().get();
    sim.world_mut()
        .entity_mut(visual)
        .remove::<VisualMaterial>();
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::MissingVisualData(id))
    );
    sim.world()
        .resource::<StepFault>()
        .report(std::io::Error::other("failure"));
    assert!(sim.step_exact(1).is_err());
    assert_eq!(extract(&sim, 1, [0.0; 3]), Err(SnapshotError::Faulted));
    sim.world_mut().insert_resource(StepFault::default());
    assert_eq!(extract(&sim, 1, [0.0; 3]), Err(SnapshotError::Faulted));
}

#[test]
fn rejects_unsupported_and_invalid_visuals() {
    let mut sim = scene();
    let entity = named(&sim, "default::robot::link::view");
    let id = sim.world().get::<EntityId>(entity).unwrap().get();
    sim.world_mut().get_mut::<VisualGeometry>(entity).unwrap().0 =
        Arc::new(sdformat::Geometry::Sphere { radius: 1.0 });
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedGeometry(id))
    );
    sim.world_mut().get_mut::<VisualGeometry>(entity).unwrap().0 =
        Arc::new(sdformat::Geometry::Box {
            size: [1.0, 0.0, 1.0],
        });
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::InvalidVisual(id))
    );
    sim.world_mut().get_mut::<VisualGeometry>(entity).unwrap().0 =
        Arc::new(sdformat::Geometry::Box { size: [1.0; 3] });
    sim.world_mut()
        .get_mut::<WorldPose>(entity)
        .unwrap()
        .rotation = [0.0; 4];
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::InvalidVisual(id))
    );
    sim.world_mut()
        .get_mut::<WorldPose>(entity)
        .unwrap()
        .rotation = [0.0, 0.0, 0.0, 1.0];
    sim.world_mut()
        .get_mut::<VisualProperties>(entity)
        .unwrap()
        .0
        .transparency = 0.5;
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedMaterial(id))
    );
    sim.world_mut()
        .get_mut::<VisualProperties>(entity)
        .unwrap()
        .0
        .transparency = 0.0;
    sim.world_mut()
        .get_mut::<VisualMaterial>(entity)
        .unwrap()
        .0
        .diffuse[3] = 0.5;
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedMaterial(id))
    );
}

#[test]
fn sorted_ids_limit_and_invalid_identity() {
    let mut sim = scene();
    let original = named(&sim, "default::robot::link::view");
    let geometry = sim.world().get::<VisualGeometry>(original).unwrap().clone();
    let material = sim.world().get::<VisualMaterial>(original).unwrap().clone();
    let properties = sim
        .world()
        .get::<VisualProperties>(original)
        .unwrap()
        .clone();
    let pose = *sim.world().get::<WorldPose>(original).unwrap();
    for _ in 0..127 {
        let e = sim
            .world_mut()
            .spawn((
                EntityKind::Visual,
                geometry.clone(),
                material.clone(),
                properties.clone(),
                pose,
            ))
            .id();
        assign_id(sim.world_mut(), e).unwrap();
    }
    let (_, visuals) = parts(extract(&sim, 1, [0.0; 3]).unwrap());
    assert_eq!(visuals.len(), 128);
    assert!(visuals.windows(2).all(|pair| pair[0].id < pair[1].id));
    let extra = sim
        .world_mut()
        .spawn((EntityKind::Visual, geometry, material, properties, pose))
        .id();
    assign_id(sim.world_mut(), extra).unwrap();
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::TooManyVisuals)
    );
    sim.world_mut().despawn(extra);
    sim.world_mut().spawn(EntityKind::Visual);
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::InvalidIdentity)
    );
}

#[test]
fn unsupported_material_channels_and_visual_properties_are_rejected() {
    let mut sim = scene();
    let entity = named(&sim, "default::robot::link::view");
    let id = sim.world().get::<EntityId>(entity).unwrap().get();
    let original = sim.world().get::<VisualMaterial>(entity).unwrap().clone();
    let mutations: [fn(&mut sdformat::Material); 11] = [
        |m| m.ambient[0] = 0.1,
        |m| m.specular[0] = 0.1,
        |m| m.emissive[0] = 0.1,
        |m| m.shininess = 1.0,
        |m| m.render_order = 1.0,
        |m| m.lighting = false,
        |m| m.double_sided = true,
        |m| m.shader = sdformat::ShaderType::Vertex,
        |m| m.normal_map = "normal.png".into(),
        |m| m.diffuse[0] = f32::NAN,
        |m| m.diffuse[1] = 1.1,
    ];
    for mutate in mutations {
        let mut changed = original.clone();
        mutate(&mut changed.0);
        *sim.world_mut().get_mut::<VisualMaterial>(entity).unwrap() = changed;
        assert_eq!(
            extract(&sim, 1, [0.0; 3]),
            Err(SnapshotError::UnsupportedMaterial(id))
        );
    }
    let mut scripted = original.clone();
    scripted.0.script = Some(sdformat::MaterialScript {
        uris: vec!["model://missing".into()],
        name: "material".into(),
        source: scripted.0.source.clone(),
    });
    *sim.world_mut().get_mut::<VisualMaterial>(entity).unwrap() = scripted;
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedMaterial(id))
    );
    *sim.world_mut().get_mut::<VisualMaterial>(entity).unwrap() = original;
    let original_properties = sim.world().get::<VisualProperties>(entity).unwrap().clone();
    for mutate in [
        (|p: &mut sdformat::VisualProperties| p.cast_shadows = false)
            as fn(&mut sdformat::VisualProperties),
        |p| p.visibility_flags = 1,
        |p| p.transparency = f32::NAN,
    ] {
        let mut changed = original_properties.clone();
        mutate(&mut changed.0);
        *sim.world_mut().get_mut::<VisualProperties>(entity).unwrap() = changed;
        assert_eq!(
            extract(&sim, 1, [0.0; 3]),
            Err(SnapshotError::UnsupportedMaterial(id))
        );
    }
}

#[test]
fn invalid_dimensions_and_out_of_range_relative_pose_are_rejected() {
    let mut sim = scene();
    let entity = named(&sim, "default::robot::link::view");
    let id = sim.world().get::<EntityId>(entity).unwrap().get();
    for value in [f64::NAN, f64::INFINITY, f64::MAX, f64::MIN_POSITIVE, -1.0] {
        sim.world_mut().get_mut::<VisualGeometry>(entity).unwrap().0 =
            Arc::new(sdformat::Geometry::Box {
                size: [value, 1.0, 1.0],
            });
        assert_eq!(
            extract(&sim, 1, [0.0; 3]),
            Err(SnapshotError::InvalidVisual(id))
        );
    }
    sim.world_mut().get_mut::<VisualGeometry>(entity).unwrap().0 =
        Arc::new(sdformat::Geometry::Box { size: [1.0; 3] });
    sim.world_mut()
        .get_mut::<WorldPose>(entity)
        .unwrap()
        .translation = [f64::MAX, 0.0, 0.0];
    assert_eq!(
        extract(&sim, 1, [-f64::MAX, 0.0, 0.0]),
        Err(SnapshotError::InvalidVisual(id))
    );
}

#[test]
fn all_required_components_are_checked_independently() {
    let mut sim = scene();
    let entity = named(&sim, "default::robot::link::view");
    let id = sim.world().get::<EntityId>(entity).unwrap().get();
    let pose = *sim.world().get::<WorldPose>(entity).unwrap();
    sim.world_mut().entity_mut(entity).remove::<WorldPose>();
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::MissingVisualData(id))
    );
    sim.world_mut().entity_mut(entity).insert(pose);
    let geometry = sim.world().get::<VisualGeometry>(entity).unwrap().clone();
    sim.world_mut()
        .entity_mut(entity)
        .remove::<VisualGeometry>();
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::MissingVisualData(id))
    );
    sim.world_mut().entity_mut(entity).insert(geometry);
    let properties = sim.world().get::<VisualProperties>(entity).unwrap().clone();
    sim.world_mut()
        .entity_mut(entity)
        .remove::<VisualProperties>();
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::MissingVisualData(id))
    );
    sim.world_mut().entity_mut(entity).insert(properties);
    assert!(extract(&sim, 1, [0.0; 3]).is_ok());
}

#[test]
fn empty_simulation_extraction_creates_no_scheduled_work() {
    let sim = Simulation::new(std::time::Duration::from_millis(1)).unwrap();
    let counts = sim.scheduled_system_counts();
    let (stamp, visuals) = parts(extract(&sim, 7, [0.0; 3]).unwrap());
    assert_eq!(
        stamp,
        Stamp {
            epoch: 0,
            seconds: 0,
            nanoseconds: 0
        }
    );
    assert!(visuals.is_empty());
    assert_eq!(sim.scheduled_system_counts(), counts);
}

#[test]
fn restored_identity_sorts_before_newer_ids_regardless_of_entity_order() {
    use gz_sim_core::{reserve_id, restore_id};
    let mut sim = scene();
    let first = named(&sim, "default::robot::link::view");
    let reserved = reserve_id(sim.world(), first).unwrap();
    let geometry = sim.world().get::<VisualGeometry>(first).unwrap().clone();
    let material = sim.world().get::<VisualMaterial>(first).unwrap().clone();
    let properties = sim.world().get::<VisualProperties>(first).unwrap().clone();
    let pose = *sim.world().get::<WorldPose>(first).unwrap();
    let second = sim
        .world_mut()
        .spawn((
            EntityKind::Visual,
            geometry.clone(),
            material.clone(),
            properties.clone(),
            pose,
        ))
        .id();
    let second_id = assign_id(sim.world_mut(), second).unwrap();
    sim.world_mut().despawn(first);
    let restored = sim
        .world_mut()
        .spawn((EntityKind::Visual, geometry, material, properties, pose))
        .id();
    restore_id(sim.world_mut(), restored, &reserved).unwrap();
    let (_, visuals) = parts(extract(&sim, 1, [0.0; 3]).unwrap());
    assert_eq!(
        visuals.iter().map(|visual| visual.id).collect::<Vec<_>>(),
        [reserved.get(), second_id]
    );
}
