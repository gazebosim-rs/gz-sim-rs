// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::relationship::RelationshipTarget;
use gz_sim_scene::{BodyHandle, PhysicsEngine};
use serde_json::{Value, json};

const ACTIONS: [&str; 4] = ["spawn", "remove", "get_state", "set_state"];

fn wire(action: &str) -> Value {
    let mut value = json!({"version":1,"kind":"lifecycle","request_id":7,"world_id":9,"epoch":3,"action":action});
    match action {
        "spawn" => {
            value["template_id"] = json!(2);
            value["translation"] = json!([1.0, 2.0, 3.0]);
            value["rotation"] = json!([0.0, 0.0, 1.0, 0.0]);
        }
        "remove" | "get_state" => value["model_id"] = json!(42),
        "set_state" => {
            value["model_id"] = json!(42);
            value["translation"] = json!([1.0, 2.0, 3.0]);
            value["rotation"] = json!([0.0, 0.0, 1.0, 0.0]);
            value["linear_velocity"] = json!([-1.0, 2.0, 3.0]);
            value["angular_velocity"] = json!([0.0, 0.0, -2.0]);
        }
        _ => panic!("unknown test action"),
    }
    value
}

fn refused(text: &str, previous: u64) {
    let error = match Request::decode(text, previous) {
        Ok(_) => panic!("unexpectedly decoded {text}"),
        Err(error) => error,
    };
    assert_eq!(error.kind(), io::ErrorKind::InvalidData, "{text}");
    assert_eq!(
        error.to_string(),
        "lifecycle request is outside the admitted contract"
    );
}

fn raw_field(mut value: Value, key: &str, raw: &str) -> String {
    assert!(value.as_object_mut().unwrap().remove(key).is_some());
    format!(
        "{{{}:{raw},{}",
        serde_json::to_string(key).unwrap(),
        &value.to_string()[1..]
    )
}

#[test]
fn all_four_wire_variants_preserve_exact_operands_and_state_units() {
    for action in ACTIONS {
        let request = Request::decode(&wire(action).to_string(), 6).unwrap();
        assert_eq!(
            (request.request_id, request.world_id, request.epoch),
            (7, 9, 3)
        );
        assert_eq!(request.action(), action);
        match request.operation {
            Operation::Spawn { template_id, pose } => {
                assert_eq!(action, "spawn");
                assert_eq!(template_id, 2);
                assert_eq!(pose.translation, [1.0, 2.0, 3.0]);
                assert_eq!(pose.rotation, [0.0, 0.0, 1.0, 0.0]);
            }
            Operation::Remove(id) => {
                assert_eq!(action, "remove");
                assert_eq!(id, 42);
            }
            Operation::GetState(id) => {
                assert_eq!(action, "get_state");
                assert_eq!(id, 42);
            }
            Operation::SetState(id, state) => {
                assert_eq!(action, "set_state");
                assert_eq!(id, 42);
                assert_eq!(
                    state,
                    ModelState {
                        translation: [1.0, 2.0, 3.0],
                        rotation: [0.0, 0.0, 1.0, 0.0],
                        linear_velocity: [-1.0, 2.0, 3.0],
                        angular_velocity: [0.0, 0.0, -2.0],
                    }
                );
            }
        }
    }
}

#[test]
fn every_required_field_rejects_missing_null_duplicates_and_inapplicable_keys() {
    for action in ACTIONS {
        let original = wire(action);
        for (key, field) in original.as_object().unwrap() {
            let mut missing = original.clone();
            missing.as_object_mut().unwrap().remove(key);
            refused(&missing.to_string(), 0);
            let mut null = original.clone();
            null[key] = Value::Null;
            refused(&null.to_string(), 0);
            // Keep the duplicate in raw text: a Value roundtrip would erase it.
            let duplicate = format!(
                "{{{}:{field},{}",
                serde_json::to_string(key).unwrap(),
                &original.to_string()[1..]
            );
            refused(&duplicate, 0);
        }
        for key in [
            "template_id",
            "model_id",
            "translation",
            "rotation",
            "linear_velocity",
            "angular_velocity",
            "xml",
            "uri",
            "name",
        ] {
            if original.get(key).is_none() {
                let mut extra = original.clone();
                extra[key] = Value::Null;
                refused(&extra.to_string(), 0);
            }
        }
    }
}

#[test]
fn envelope_integer_domains_and_monotonic_request_order_are_exact() {
    for action in ACTIONS {
        let original = wire(action);
        for (key, invalid) in [
            ("version", json!(0)),
            ("version", json!(2)),
            ("version", json!(256)),
            ("kind", json!("control")),
            ("kind", json!(true)),
            ("request_id", json!(0)),
            ("world_id", json!(0)),
            ("action", json!("reset")),
            ("action", json!("setState")),
        ] {
            let mut value = original.clone();
            value[key] = invalid;
            refused(&value.to_string(), 0);
        }
        let operand = if action == "spawn" {
            "template_id"
        } else {
            "model_id"
        };
        for key in ["version", "request_id", "world_id", "epoch", operand] {
            for raw in ["-1", "1.5", "\"7\"", "true", "18446744073709551616"] {
                refused(&raw_field(original.clone(), key, raw), 0);
            }
        }
        let mut zero_operand = original.clone();
        zero_operand[operand] = json!(0);
        refused(&zero_operand.to_string(), 0);
        refused(&original.to_string(), 7);
        refused(&original.to_string(), 8);
        let mut exhausted = original.clone();
        exhausted["request_id"] = json!(u64::MAX);
        assert_eq!(
            Request::decode(&exhausted.to_string(), u64::MAX - 1)
                .unwrap()
                .request_id,
            u64::MAX
        );
        refused(&exhausted.to_string(), u64::MAX);
        let mut first_epoch = original.clone();
        first_epoch["epoch"] = json!(0);
        assert_eq!(
            Request::decode(&first_epoch.to_string(), 0).unwrap().epoch,
            0
        );
    }
    for malformed in [
        "{",
        "null",
        "[]",
        "true",
        "\"request\"",
        "{}",
        "{} trailing",
    ] {
        refused(malformed, 0);
    }
}

#[test]
fn array_schema_and_json_numeric_errors_do_not_normalize_semantic_state() {
    for action in ["spawn", "set_state"] {
        let original = wire(action);
        for key in [
            "translation",
            "rotation",
            "linear_velocity",
            "angular_velocity",
        ] {
            if original.get(key).is_none() {
                continue;
            }
            let width = if key == "rotation" { 4 } else { 3 };
            for count in [width - 1, width + 1] {
                let mut wrong = original.clone();
                wrong[key] = json!(vec![0.0; count]);
                refused(&wrong.to_string(), 0);
            }
            for scalar in [
                "null", "true", "\"NaN\"", "1e309", "-1e309", "NaN", "Infinity",
            ] {
                let mut entries = vec!["0"; width];
                entries[0] = scalar;
                refused(
                    &raw_field(original.clone(), key, &format!("[{}]", entries.join(","))),
                    0,
                );
            }
        }
        // These finite f64 values are valid JSON/schema. Scene must refuse their
        // geometry/renderer semantics atomically; the decoder must not rewrite them.
        let mut semantic = original.clone();
        semantic["translation"] = json!([f64::MAX, 0.0, 0.0]);
        semantic["rotation"] = json!([0.0, 0.0, 0.0, 0.0]);
        match Request::decode(&semantic.to_string(), 0).unwrap().operation {
            Operation::Spawn { pose, .. } => {
                assert_eq!(pose.translation[0], f64::MAX);
                assert_eq!(pose.rotation, [0.0; 4]);
            }
            Operation::SetState(_, state) => {
                assert_eq!(state.translation[0], f64::MAX);
                assert_eq!(state.rotation, [0.0; 4]);
            }
            _ => panic!("wrong operation"),
        }
    }
}

fn catalog() -> Catalog {
    Catalog {
        version: 1,
        kind: "lifecycle_catalog",
        world_id: 1,
        epoch: 2,
        scene_sequence: 3,
        templates: vec![TemplateInfo {
            template_id: 1,
            label: "sphere".into(),
        }],
        models: vec![Model {
            model_id: 2,
            name: "w::a".into(),
            visual_ids: vec![3],
            can_remove: true,
            can_set_state: true,
        }],
    }
}

#[test]
fn catalog_counts_allow_the_declared_bounds_and_reject_each_overflow() {
    let mut full = catalog();
    full.models = (0..128)
        .map(|i| Model {
            model_id: 1000 + i,
            name: format!("w::m{i}"),
            visual_ids: vec![2000 + i],
            can_remove: i % 2 == 0,
            can_set_state: false,
        })
        .collect();
    full.templates = (1..=128)
        .map(|template_id| TemplateInfo {
            template_id,
            label: format!("t{template_id}"),
        })
        .collect();
    let encoded = full.encode().unwrap();
    assert!(encoded.len() <= snapshot::MAX_BYTES);
    let value: Value = serde_json::from_str(&encoded).unwrap();
    assert_eq!(value["kind"], "lifecycle_catalog");
    assert_eq!(
        (value["epoch"].as_u64(), value["scene_sequence"].as_u64()),
        (Some(2), Some(3))
    );
    assert_eq!(value["models"].as_array().unwrap().len(), 128);
    assert_eq!(value["templates"].as_array().unwrap().len(), 128);
    for (index, model) in value["models"].as_array().unwrap().iter().enumerate() {
        assert_eq!(model["model_id"], 1000 + index as u64);
        assert_eq!(model["visual_ids"], json!([2000 + index as u64]));
        assert_eq!(model["can_remove"], index % 2 == 0);
        assert_eq!(model["can_set_state"], false);
    }
    let mut models = full.clone();
    models.models.push(Model {
        model_id: 9999,
        name: "extra".into(),
        visual_ids: vec![9998],
        can_remove: false,
        can_set_state: false,
    });
    assert_eq!(
        models.encode().unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
    let mut templates = full.clone();
    templates.templates.push(TemplateInfo {
        template_id: 129,
        label: "extra".into(),
    });
    assert_eq!(
        templates.encode().unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
    let mut visuals = catalog();
    visuals.models[0].model_id = 1000;
    visuals.models[0].visual_ids = (10..139).collect();
    assert_eq!(
        visuals.encode().unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
}

#[test]
fn catalog_world_model_and_visual_aliases_names_and_encoded_byte_budget_are_refused() {
    type Mutation = fn(&mut Catalog);
    let mutations: &[(&str, Mutation)] = &[
        ("world zero", |c| c.world_id = 0),
        ("world rendering width", |c| {
            c.world_id = u32::MAX as u64 + 1
        }),
        ("model zero", |c| c.models[0].model_id = 0),
        ("model aliases world", |c| c.models[0].model_id = c.world_id),
        ("duplicate model", |c| c.models.push(c.models[0].clone())),
        ("empty owned visuals", |c| c.models[0].visual_ids.clear()),
        ("visual zero", |c| c.models[0].visual_ids[0] = 0),
        ("visual rendering width", |c| {
            c.models[0].visual_ids[0] = u32::MAX as u64 + 1
        }),
        ("duplicate visual within model", |c| {
            c.models[0].visual_ids.push(3)
        }),
        ("duplicate visual across owners", |c| {
            c.models.push(Model {
                model_id: 4,
                name: "w::b".into(),
                visual_ids: vec![3],
                can_remove: false,
                can_set_state: false,
            })
        }),
        ("visual aliases world", |c| {
            c.models[0].visual_ids[0] = c.world_id
        }),
        ("visual aliases model", |c| {
            c.models[0].visual_ids[0] = c.models[0].model_id
        }),
        ("name byte overflow", |c| c.models[0].name = "é".repeat(129)),
    ];
    for &(label, mutate) in mutations {
        let mut value = catalog();
        mutate(&mut value);
        assert_eq!(
            value.encode().unwrap_err().kind(),
            io::ErrorKind::InvalidData,
            "{label}"
        );
    }
    let mut edge = catalog();
    edge.models[0].name = "é".repeat(128);
    edge.models[0].model_id = u64::MAX;
    edge.models[0].visual_ids = vec![u32::MAX as u64];
    edge.world_id = u32::MAX as u64 - 1;
    assert!(edge.encode().is_ok());
    // Each label stays within the trusted constructor's 256-byte limit; JSON
    // escaping still makes the complete publication exceed its independent cap.
    let mut oversized = catalog();
    oversized.templates = (1..=128)
        .map(|template_id| TemplateInfo {
            template_id,
            label: "\"".repeat(256),
        })
        .collect();
    assert!(serde_json::to_string(&oversized).unwrap().len() > snapshot::MAX_BYTES);
    assert_eq!(
        oversized.encode().unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
}

fn loaded() -> (Simulation, sdformat::Sdf, snapshot::Scene) {
    let model = |name: &str, x, geometry: &str| {
        format!(
            "<model name='{name}'><pose>{x} 0 0 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='c'><geometry>{geometry}</geometry></collision><visual name='v'><geometry>{geometry}</geometry><material><diffuse>0.8 0.2 0.1 1</diffuse></material></visual></link></model>"
        )
    };
    let xml = format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity>{}{}</world></sdf>",
        model("a", 0, "<sphere><radius>0.5</radius></sphere>"),
        model("b", 10, "<box><size>1 1 1</size></box>")
    );
    let sdf = sdformat::load_str(&xml, "lifecycle-contract.sdf", &sdformat::NoResources).unwrap();
    let sim = gz_sim_scene::load_world_with_sdf_step(
        &xml,
        "lifecycle-contract.sdf",
        "w",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let scene = snapshot::extract(&sim, "w", 4).unwrap();
    (sim, sdf, scene)
}

#[test]
fn trusted_catalog_binds_real_model_visual_owners_and_refuses_missing_visible_metadata() {
    let (sim, sdf, scene) = loaded();
    let lifecycle = Lifecycle::new(&sim, &sdf, "w", &scene).unwrap();
    let encoded = lifecycle.encode_catalog(&sim, &scene).unwrap();
    let value: Value = serde_json::from_str(&encoded).unwrap();
    assert_eq!(
        value["templates"],
        json!([{"template_id":1,"label":"a"},{"template_id":2,"label":"b"}])
    );
    assert_eq!(value["world_id"], scene.world_id());
    assert_eq!(value["scene_sequence"], 4);
    for model in value["models"].as_array().unwrap() {
        let id = model["model_id"].as_u64().unwrap();
        let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
        assert_eq!(
            sim.world().get::<EntityKind>(entity),
            Some(&EntityKind::Model)
        );
        assert_eq!(model["name"], sim.world().get::<Name>(entity).unwrap().0);
        assert_eq!(model["can_remove"], true);
        assert_eq!(model["can_set_state"], true);
        let owned: Vec<_> = sim
            .world()
            .get::<Children>(entity)
            .unwrap()
            .iter()
            .flat_map(|link| sim.world().get::<Children>(link).unwrap().iter())
            .filter(|child| sim.world().get::<EntityKind>(*child) == Some(&EntityKind::Visual))
            .map(|visual| sim.world().get::<EntityId>(visual).unwrap().get())
            .collect();
        assert_eq!(model["visual_ids"], json!(owned));
    }
    assert_eq!(value["models"].as_array().unwrap().len(), 2);
    let mut missing = scene.clone();
    missing
        .project_remove(&[scene.visual_ids().next().unwrap()])
        .unwrap();
    assert!(Lifecycle::new(&sim, &sdf, "w", &missing).is_err());
    assert!(Lifecycle::new(&sim, &sdf, "missing-world", &scene).is_err());

    for corruption in 0..3 {
        let (mut sim, sdf, scene) = loaded();
        let lifecycle = Lifecycle::new(&sim, &sdf, "w", &scene).unwrap();
        let model = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "w::a"))
            .unwrap()
            .id();
        let visual = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "w::a::body::v"))
            .unwrap()
            .id();
        match corruption {
            0 => {
                sim.world_mut().entity_mut(model).remove::<EntityId>();
            }
            1 => {
                sim.world_mut().entity_mut(model).remove::<Name>();
            }
            _ => {
                sim.world_mut().entity_mut(visual).remove::<EntityId>();
            }
        }
        let bodies: Vec<_> = sim
            .world()
            .iter_entities()
            .filter_map(|e| e.get::<BodyHandle>().map(BodyHandle::id))
            .collect();
        let before: Vec<_> = bodies
            .iter()
            .map(|id| {
                sim.world()
                    .resource::<gz_physics::PhysicsBackend>()
                    .state(*id)
            })
            .collect();
        let next = next_entity_id(sim.world()).unwrap();
        let clock = (sim.elapsed(), sim.epoch(), sim.is_paused());
        assert_eq!(
            lifecycle.encode_catalog(&sim, &scene).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
        assert_eq!(next_entity_id(sim.world()).unwrap(), next);
        assert_eq!((sim.elapsed(), sim.epoch(), sim.is_paused()), clock);
        assert_eq!(
            bodies
                .iter()
                .map(|id| sim
                    .world()
                    .resource::<gz_physics::PhysicsBackend>()
                    .state(*id))
                .collect::<Vec<_>>(),
            before
        );
    }
}

#[test]
fn imu_model_stays_visible_readonly_while_only_the_free_box_becomes_a_template() {
    use gz_sim_scene::{ImuPublication, ImuSample, imu_active, imu_rate, set_imu_interest};

    let model = |name: &str, x: u8, sensor: &str| {
        format!(
            "<model name='{name}'><pose>{x} 0 0 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='c'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='v'><geometry><box><size>1 1 1</size></box></geometry><material><diffuse>0.2 0.4 0.6 1</diffuse></material></visual>{sensor}</link></model>"
        )
    };
    let xml = format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity>{}{}</world></sdf>",
        model(
            "instrumented",
            0,
            "<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>100</update_rate><topic>/m10/readonly/imu</topic><frame_id>instrumented/body/imu</frame_id><imu><enable_orientation>true</enable_orientation></imu></sensor>"
        ),
        model("free", 10, ""),
    );
    let sdf = sdformat::load_str(
        &xml,
        "lifecycle-imu-coexistence.sdf",
        &sdformat::NoResources,
    )
    .unwrap();
    let mut sim = gz_sim_scene::load_world_with_sdf_step(
        &xml,
        "lifecycle-imu-coexistence.sdf",
        "w",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let sensor = sim
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<ImuPublication>())
        .unwrap()
        .id();
    let sensor_id = sim.world().get::<EntityId>(sensor).unwrap().get();
    set_imu_interest(&mut sim, [sensor]);
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().get::<ImuSample>(sensor).unwrap().time,
        sim.elapsed()
    );
    let metadata = |sim: &Simulation| {
        let publication = sim.world().get::<ImuPublication>(sensor).unwrap();
        let rate = imu_rate(sim, sensor_id).unwrap();
        (
            publication.enable_orientation,
            publication.entity_name.clone(),
            publication.frame_id.clone(),
            publication.topic.clone(),
            (
                rate.sdf_max_hz,
                rate.rate_hz,
                imu_active(sim, sensor_id).unwrap(),
            ),
            format!("{:?}", sim.world().get::<ImuSample>(sensor)),
            format!("{:?}", sim.world().get::<WorldPose>(sensor)),
            sim.world().get::<ChildOf>(sensor).unwrap().parent(),
        )
    };
    let sensor_before = metadata(&sim);
    assert!(sensor_before.0);
    assert_eq!(sensor_before.2, "instrumented/body/imu");
    assert_eq!(sensor_before.3, "/m10/readonly/imu");
    assert_eq!(sensor_before.4, (100.0, 100.0, true));
    let identities = |sim: &Simulation| {
        let mut result: Vec<_> = sim
            .world()
            .iter_entities()
            .filter_map(|entity| entity.get::<EntityId>().map(|id| (id.get(), entity.id())))
            .collect();
        result.sort_unstable_by_key(|(id, _)| *id);
        result
    };
    let ids_before = identities(&sim);
    let next_before = next_entity_id(sim.world()).unwrap();
    let clock_before = (sim.elapsed(), sim.epoch(), sim.is_paused());
    let bodies: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<BodyHandle>().map(BodyHandle::id))
        .collect();
    assert_eq!(bodies.len(), 2);
    let native_before: Vec<_> = bodies
        .iter()
        .map(|id| {
            sim.world()
                .resource::<gz_physics::PhysicsBackend>()
                .state(*id)
                .unwrap()
        })
        .collect();
    let scene = snapshot::extract(&sim, "w", 17).unwrap();
    let scene_before = snapshot::encode(&scene).unwrap();

    let lifecycle = Lifecycle::new(&sim, &sdf, "w", &scene).unwrap();
    let catalog: Value =
        serde_json::from_str(&lifecycle.encode_catalog(&sim, &scene).unwrap()).unwrap();
    assert_eq!(catalog["world_id"], scene.world_id());
    assert_eq!(catalog["epoch"], sim.epoch());
    assert_eq!(catalog["scene_sequence"], 17);
    assert_eq!(
        catalog["templates"],
        json!([{"template_id":1,"label":"free"}])
    );
    assert_eq!(catalog["models"].as_array().unwrap().len(), 2);
    for (name, editable) in [("w::instrumented", false), ("w::free", true)] {
        let model = sim
            .world()
            .iter_entities()
            .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
            .unwrap();
        let model_id = model.get::<EntityId>().unwrap().get();
        let row = catalog["models"]
            .as_array()
            .unwrap()
            .iter()
            .find(|value| value["model_id"] == model_id)
            .unwrap();
        assert_eq!(row["name"], name);
        assert_eq!(row["can_remove"], editable);
        assert_eq!(row["can_set_state"], editable);
        let visual_name = format!("{name}::body::v");
        let visual = sim
            .world()
            .iter_entities()
            .find(|entity| {
                entity
                    .get::<Name>()
                    .is_some_and(|value| value.0 == visual_name)
            })
            .unwrap();
        assert_eq!(
            row["visual_ids"],
            json!([visual.get::<EntityId>().unwrap().get()])
        );
        assert_eq!(
            visual.get::<ChildOf>().unwrap().parent(),
            model.get::<Children>().unwrap()[0]
        );
    }
    assert_eq!(metadata(&sim), sensor_before);
    assert_eq!(identities(&sim), ids_before);
    for (id, entity) in ids_before {
        assert_eq!(gz_sim_core::entity_by_id(sim.world(), id), Some(entity));
    }
    assert_eq!(next_entity_id(sim.world()).unwrap(), next_before);
    assert_eq!((sim.elapsed(), sim.epoch(), sim.is_paused()), clock_before);
    assert_eq!(
        bodies
            .iter()
            .map(|id| sim
                .world()
                .resource::<gz_physics::PhysicsBackend>()
                .state(*id)
                .unwrap())
            .collect::<Vec<_>>(),
        native_before
    );
    assert_eq!(
        snapshot::encode(&snapshot::extract(&sim, "w", 17).unwrap()).unwrap(),
        scene_before
    );
}
