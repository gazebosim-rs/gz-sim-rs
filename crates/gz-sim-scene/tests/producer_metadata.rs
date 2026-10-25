// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    LinkProperties, LoadError, Name, PhysicsEngine, VisualMaterial, load_world_with_sdf_step,
    remove_model, reset_loaded_world,
};
use sdformat::NoResources;

fn xml(flags: &str, material: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><model name='robot'><link name='body'>{flags}<collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><visual name='view'><geometry><sphere><radius>0.5</radius></sphere></geometry>{material}</visual></link></model></world></sdf>"
    )
}
fn named(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}

#[test]
fn material_and_link_properties_remain_owned_through_mutation_removal_and_reset() {
    let material = "<material><ambient>0.1 0.2 0.3 0.4</ambient><diffuse>0.5 0.6 0.7 0.8</diffuse><specular>0.2 0.3 0.4 0.5</specular><emissive>0.01 0.02 0.03 1</emissive><shininess>12</shininess><render_order>-2.5</render_order><lighting>false</lighting><double_sided>true</double_sided><shader type='normal_map_tangent_space'><normal_map>normal.png</normal_map></shader><script><uri>model://robot/materials/first</uri><uri>file:///second</uri><name>Robot/Metal</name></script></material>";
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let source = xml("", material);
        let mut sim =
            load_world_with_sdf_step(&source, "material-source", "default", &NoResources, engine)
                .unwrap();
        drop(source);
        let model = named(&sim, "default::robot");
        let model_id = sim.world().get::<EntityId>(model).unwrap().get();
        let link = named(&sim, "default::robot::body");
        let visual = named(&sim, "default::robot::body::view");
        let id = sim.world().get::<EntityId>(visual).unwrap().get();
        let properties = *sim.world().get::<LinkProperties>(link).unwrap();
        assert_eq!(properties.0, sdformat::LinkProperties::default());
        let initial = sim.world().get::<VisualMaterial>(visual).unwrap().0.clone();
        assert_eq!(initial.ambient, [0.1_f32, 0.2, 0.3, 0.4]);
        assert_eq!(initial.diffuse, [0.5_f32, 0.6, 0.7, 0.8]);
        assert_eq!(initial.specular, [0.2_f32, 0.3, 0.4, 0.5]);
        assert_eq!(initial.emissive, [0.01_f32, 0.02, 0.03, 1.0]);
        assert_eq!(initial.shininess, 12.0);
        assert_eq!(initial.render_order, -2.5_f32);
        assert!(!initial.lighting);
        assert!(initial.double_sided);
        assert_eq!(initial.shader, sdformat::ShaderType::NormalMapTangentSpace);
        assert_eq!(initial.normal_map, "normal.png");
        assert_eq!(
            initial.script.as_ref().unwrap().uris,
            ["model://robot/materials/first", "file:///second"]
        );
        assert_eq!(initial.script.as_ref().unwrap().name, "Robot/Metal");
        assert_eq!(initial.source.uri, "material-source");
        sim.world_mut()
            .get_mut::<LinkProperties>(link)
            .unwrap()
            .0
            .enable_gravity = false;
        sim.world_mut()
            .get_mut::<VisualMaterial>(visual)
            .unwrap()
            .0
            .script
            .as_mut()
            .unwrap()
            .uris
            .clear();
        sim.step_exact(1).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let link = named(&sim, "default::robot::body");
        let visual = named(&sim, "default::robot::body::view");
        assert_eq!(
            *sim.world().get::<LinkProperties>(link).unwrap(),
            properties
        );
        assert_eq!(
            sim.world().get::<VisualMaterial>(visual).unwrap().0,
            initial
        );
        assert_eq!(
            sim.world()
                .get::<EntityId>(named(&sim, "default::robot"))
                .unwrap()
                .get(),
            model_id
        );
        remove_model(&mut sim, model_id).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let visual = named(&sim, "default::robot::body::view");
        assert_eq!(sim.world().get::<EntityId>(visual).unwrap().get(), id);
        assert_eq!(
            sim.world().get::<VisualMaterial>(visual).unwrap().0,
            initial
        );
    }
}

#[test]
fn absent_material_stays_absent_and_nondefault_link_effects_are_typed_errors() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let sim = load_world_with_sdf_step(&xml("", ""), "absent", "default", &NoResources, engine)
            .unwrap();
        let visual = named(&sim, "default::robot::body::view");
        assert!(sim.world().get::<VisualMaterial>(visual).is_none());
        for (flag, expected) in [
            ("<enable_wind>true</enable_wind>", "link wind"),
            ("<gravity>false</gravity>", "disabled link gravity"),
            ("<kinematic>true</kinematic>", "kinematic links"),
            ("<self_collide>true</self_collide>", "link self collision"),
        ] {
            let error =
                load_world_with_sdf_step(&xml(flag, ""), "flags", "default", &NoResources, engine)
                    .err()
                    .unwrap();
            assert!(
                matches!(error,LoadError::Unsupported {feature,..} if feature==expected),
                "{error}"
            );
            assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
            assert!(sim.world().get::<VisualMaterial>(visual).is_none());
        }
    }
}
