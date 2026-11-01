/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Sensor_TEST.FrameIdFromSdf precedence, projected onto the admitted camera path.
// Native SIM RenderUtil supplies the world-stripped scoped fallback before Load.
use super::*;
use std::sync::Arc;

fn loaded(parameters: &str) -> (Simulation, u64) {
    let source = include_str!("../../../gz-sim-scene/tests/fixtures/camera-binding.sdf")
        .replace("name=\"robot\"", "name=\"m1\"")
        .replace("name=\"body\"", "name=\"link1\"")
        .replace(
            "name=\"camera\" type=\"camera\"",
            "name=\"test\" type=\"camera\"",
        )
        .replace("<camera>", &format!("{parameters}<camera>"));
    let sim = load_world_with_sdf_step(
        &source,
        "loaded-camera.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<CameraSensor>())
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let entity = entity_by_id(sim.world(), id).unwrap();
    assert_eq!(
        sim.world().get::<Name>(entity).unwrap().0,
        "default::m1::link1::test"
    );
    assert_eq!(
        sim.world().get::<CameraSensor>(entity).unwrap().0.name,
        "test"
    );
    (sim, id)
}

fn configuration() -> CameraOutputConfig {
    CameraOutputConfig {
        origin: [1_000_000_000.0, 0.0, 0.0],
        frame_id: "operator_override".into(),
        image_key: "@/loaded-frame@/image".into(),
        info_key: "@/loaded-frame@/info".into(),
        worker_path: "/nonexistent/worker".into(),
        max_message_bytes: NonZeroUsize::new(16 * 1024 * 1024).unwrap(),
    }
}

#[test]
fn native_loaded_frame_precedence_keeps_scoped_fallback_and_present_empty_values() {
    // First three preserve the original Sensor_TEST precedence observations;
    // only the default changes through the source-audited SIM SetName step.
    for (parameters, expected) in [
        ("", "m1::link1::test"),
        ("<frame_id>custom_frame_id</frame_id>", "custom_frame_id"),
        (
            "<frame_id>custom_frame_id</frame_id><gz_frame_id>other_custom_frame_id</gz_frame_id>",
            "custom_frame_id",
        ),
        ("<gz_frame_id>legacy</gz_frame_id>", "legacy"),
        ("<gz_frame_id/>", ""),
        ("<gz_frame_id>  </gz_frame_id>", ""),
        ("<frame_id>  canonical  </frame_id>", "canonical"),
        ("<frame_id/><gz_frame_id>legacy</gz_frame_id>", "legacy"),
        ("<frame_id/>", "m1::link1::test"),
        ("<frame_id/><gz_frame_id/>", ""),
    ] {
        let (sim, id) = loaded(parameters);
        let config = configuration().with_loaded_frame_id(&sim, id).unwrap();
        assert_eq!(config.frame_id, expected, "{parameters}");
        assert_eq!(config.origin, [1_000_000_000.0, 0.0, 0.0]);
        assert_eq!(config.image_key, "@/loaded-frame@/image");
        assert_eq!(config.info_key, "@/loaded-frame@/info");
        assert_eq!(
            config.worker_path,
            std::path::Path::new("/nonexistent/worker")
        );
        assert_eq!(config.max_message_bytes.get(), 16 * 1024 * 1024);
    }
}

#[test]
fn derived_config_owns_frame_without_retaining_scene_metadata() {
    let (sim, id) = loaded("");
    let entity = entity_by_id(sim.world(), id).unwrap();
    let metadata = Arc::downgrade(&sim.world().get::<CameraSensor>(entity).unwrap().0);
    let owners = metadata.strong_count();
    let config = configuration().with_loaded_frame_id(&sim, id).unwrap();
    assert_eq!(metadata.strong_count(), owners);
    drop(sim);
    assert!(metadata.upgrade().is_none());
    assert_eq!(config.frame_id, "m1::link1::test");
}

#[test]
fn loaded_frame_rejects_missing_identity_kind_metadata_and_name_even_with_explicit_frame() {
    let (sim, id) = loaded("<frame_id>canonical</frame_id>");
    assert!(
        matches!(configuration().with_loaded_frame_id(&sim, id + 1000), Err(CameraOutputError::Snapshot(SnapshotError::MissingCamera(value))) if value == id + 1000)
    );
    for damage in 0..5 {
        let (mut sim, id) = loaded("<frame_id>canonical</frame_id>");
        let entity = entity_by_id(sim.world(), id).unwrap();
        match damage {
            0 => {
                sim.world_mut().entity_mut(entity).remove::<EntityId>();
            }
            1 => {
                sim.world_mut().entity_mut(entity).remove::<EntityKind>();
            }
            2 => {
                sim.world_mut().entity_mut(entity).insert(EntityKind::Link);
            }
            3 => {
                sim.world_mut().entity_mut(entity).remove::<CameraSensor>();
            }
            4 => {
                sim.world_mut().entity_mut(entity).remove::<Name>();
            }
            _ => unreachable!(),
        }
        assert!(
            matches!(configuration().with_loaded_frame_id(&sim, id), Err(CameraOutputError::Snapshot(SnapshotError::MissingCamera(value))) if value == id),
            "damage {damage}"
        );
    }
    for name in ["", "unscoped", "::sensor", "world::", "world::::sensor"] {
        let (mut sim, id) = loaded("<frame_id>canonical</frame_id>");
        let entity = entity_by_id(sim.world(), id).unwrap();
        sim.world_mut().entity_mut(entity).insert(Name(name.into()));
        assert!(
            matches!(configuration().with_loaded_frame_id(&sim, id), Err(CameraOutputError::Snapshot(SnapshotError::MissingCamera(value))) if value == id),
            "{name}"
        );
    }
}

fn published_frame(parameters: &str, derive: bool) -> String {
    let (sim, id) = loaded(parameters);
    let scheduled = sim.scheduled_system_counts();
    let (session, scope, _) = session();
    let subscriber = session
        .declare_subscriber("@/loaded-frame@/info")
        .wait()
        .unwrap();
    let mut config = configuration();
    if derive {
        config = config.with_loaded_frame_id(&sim, id).unwrap();
    }
    let expected = config.frame_id.clone();
    let mut output = CameraOutput::new(&sim, &session, &scope, id, config).unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::InfoPublished);
    assert!(output.client.is_none());
    assert_eq!(sim.scheduled_system_counts(), scheduled);
    let sample = subscriber
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap();
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.CameraInfo"
    );
    let info =
        gz_sim_proto::msgs::CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
    let header = info.header.unwrap();
    assert_eq!(header.data.len(), 1);
    assert_eq!(header.data[0].key, "frame_id");
    assert_eq!(header.data[0].value, std::slice::from_ref(&expected));
    assert_eq!(header.stamp.unwrap(), gz_sim_proto::msgs::Time::default());
    let image = image_packet(&vec![0u8; 64 * 64 * 4], Duration::ZERO, &expected, 0).unwrap();
    let image_header = image.header.unwrap();
    assert_eq!(image_header.data[0].key, "frame_id");
    assert_eq!(image_header.data[0].value, std::slice::from_ref(&expected));
    assert_eq!(image_header.data[1].key, "seq");
    assert_eq!(image_header.data[1].value, ["0"]);
    expected
}

#[test]
fn explicit_manual_override_and_opt_in_frame_reach_headers_without_starting_a_worker() {
    assert_eq!(published_frame("", true), "m1::link1::test");
    assert_eq!(
        published_frame("<frame_id>canonical</frame_id>", true),
        "canonical"
    );
    assert_eq!(
        published_frame("<gz_frame_id/>", false),
        "operator_override"
    );
}

#[test]
fn empty_native_frame_stays_empty_and_constructor_refuses_before_publication() {
    let (sim, id) = loaded("<gz_frame_id/>");
    let (session, scope, _) = session();
    let config = configuration().with_loaded_frame_id(&sim, id).unwrap();
    assert!(config.frame_id.is_empty());
    assert!(matches!(
        CameraOutput::new(&sim, &session, &scope, id, config),
        Err(CameraOutputError::InvalidFrameId)
    ));
    assert!(scope.entries.lock().unwrap().is_empty());
    assert_eq!(
        published_frame("<gz_frame_id/>", false),
        "operator_override"
    );
}
