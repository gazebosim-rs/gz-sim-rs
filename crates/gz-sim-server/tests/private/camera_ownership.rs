// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::net::TcpListener;

fn simulation(configured: bool, camera: bool) -> Simulation {
    let source = if camera {
        include_str!("../../../gz-sim-scene/tests/fixtures/camera-binding.sdf").to_owned()
    } else {
        "<sdf version='1.12'><world name='default'/></sdf>".into()
    };
    let source = if configured {
        source.replace("name=\"default\">", "name=\"default\"><plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>").replace("name='default'/>", "name='default'><plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/></world>")
    } else {
        source
    };
    gz_sim_scene::load_world_with_sdf_step(
        &source,
        "camera-ownership.sdf",
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Tpe,
    )
    .unwrap()
}
fn options() -> ServerOptions {
    ServerOptions {
        partition: format!("camera-boundary-{}", uuid::Uuid::new_v4()),
        namespace: "ns".into(),
        ..Default::default()
    }
}
fn config(endpoint: &str) -> zenoh::Config {
    let mut cfg = zenoh::Config::default();
    for (key, value) in [
        ("listen/endpoints", format!("[\"{endpoint}\"]")),
        ("scouting/multicast/enabled", "false".into()),
        ("scouting/gossip/enabled", "false".into()),
    ] {
        cfg.insert_json5(key, &value).unwrap();
    }
    cfg
}
fn camera_id(sim: &Simulation) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.contains::<gz_sim_scene::CameraSensor>())
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

#[test]
fn configured_camera_requires_an_external_owner_before_network_advertising() {
    let occupied = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = format!("tcp/{}", occupied.local_addr().unwrap());
    let result = Server::with_zenoh_config(
        simulation(true, true),
        "default",
        options(),
        config(&address),
        NonZeroUsize::new(1).unwrap(),
    );
    assert!(matches!(result, Err(Error::CameraOutputRequired)));
    assert!(matches!(
        Server::new(
            simulation(true, true),
            "default",
            options(),
            NonZeroUsize::new(1).unwrap()
        ),
        Err(Error::CameraOutputRequired)
    ));
}

#[test]
fn empty_configured_and_manual_metadata_worlds_preserve_normal_server() {
    for (configured, camera) in [(true, false), (false, true), (false, false)] {
        let options = options();
        let partition = options.partition.clone();
        let mut server = Server::with_zenoh_config(
            simulation(configured, camera),
            "default",
            options,
            config("tcp/127.0.0.1:0"),
            NonZeroUsize::new(1).unwrap(),
        )
        .unwrap();
        assert!(!server.is_shutdown());
        assert_eq!(server.options().partition, partition);
        assert_eq!(server.options().namespace, "ns");
        assert_eq!(
            server.options().topic_key("relative").unwrap(),
            format!("@/{partition}@/ns/relative")
        );
        server.shutdown();
        assert!(server.is_shutdown());
        server.shutdown();
        assert!(server.is_shutdown());
    }
}

#[test]
fn external_camera_boundary_validates_the_actual_one_camera_identity() {
    let sim = simulation(true, true);
    let id = camera_id(&sim);
    let mut server = Server::with_external_camera_output(
        sim,
        "default",
        options(),
        config("tcp/127.0.0.1:0"),
        NonZeroUsize::new(1).unwrap(),
        id,
    )
    .unwrap();
    // This tests the explicit trust boundary; actual external IO is verified by
    // the camera application's constructor/lifecycle tests.
    server.shutdown();
    for (configured, camera) in [(false, true), (true, false)] {
        assert!(matches!(
            Server::with_external_camera_output(
                simulation(configured, camera),
                "default",
                options(),
                config("tcp/127.0.0.1:0"),
                NonZeroUsize::new(1).unwrap(),
                id
            ),
            Err(Error::CameraIdentity)
        ));
    }
    assert!(matches!(
        Server::with_external_camera_output(
            simulation(true, true),
            "default",
            options(),
            config("tcp/127.0.0.1:0"),
            NonZeroUsize::new(1).unwrap(),
            u64::MAX
        ),
        Err(Error::CameraIdentity)
    ));
}

#[test]
fn preparation_observation_tracks_only_unprepared_reset_not_terminal_admission() {
    let mut server = Server::with_zenoh_config(
        simulation(false, false),
        "default",
        options(),
        config("tcp/127.0.0.1:0"),
        NonZeroUsize::new(1).unwrap(),
    )
    .unwrap();
    assert!(!server.io_preparation_required());
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(gz_sim_proto::msgs::WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(server.io_preparation_required());
    assert_eq!(server.simulation().epoch(), 0);
    server.prepare_io().unwrap();
    assert!(!server.io_preparation_required());
    assert_eq!(server.simulation().epoch(), 1);
    assert_eq!(server.tick().unwrap().epoch, 1);
    server.shutdown();
    assert!(server.is_shutdown());
    assert!(!server.io_preparation_required());
    assert!(matches!(server.prepare_io(), Err(Error::WorldClosed)));
}

fn rgb_pair_simulation(count: usize, configured: bool) -> Simulation {
    let source = include_str!("../../../gz-camera-output/tests/fixtures/m5-rgb.sdf");
    let begin = source.find("<model name=\"rig\">").unwrap();
    let end = begin + source[begin..].find("</model>").unwrap() + "</model>".len();
    let mut rigs = String::new();
    for index in 0..count {
        rigs.push_str(
            &source[begin..end]
                .replace("name=\"rig\"", &format!("name=\"rig_{index}\""))
                .replace("/m5/rgb/", &format!("/pair/{index}/"))
                .replace("rig/body/camera", &format!("rig_{index}/body/camera")),
        );
    }
    let mut source = source[..begin].to_owned() + &rigs + &source[end..];
    if !configured {
        source = source.replace(
            "<plugin name=\"gz::sim::systems::Sensors\" filename=\"gz-sim-sensors-system\"/>",
            "",
        );
    }
    gz_sim_scene::load_world_with_sdf_step(
        &source,
        "external-rgb-pair",
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Tpe,
    )
    .unwrap()
}

#[test]
fn external_camera_pair_matches_the_complete_rgb_set_before_network() {
    for reversed in [false, true] {
        let sim = rgb_pair_simulation(2, true);
        let mut ids: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
            .map(|node| node.get::<EntityId>().unwrap().get())
            .collect();
        ids.sort_unstable();
        if reversed {
            ids.reverse();
        }
        let mut server = Server::with_external_camera_outputs(
            sim,
            "default",
            options(),
            config("tcp/127.0.0.1:0"),
            NonZeroUsize::new(1).unwrap(),
            [ids[0], ids[1]],
        )
        .unwrap();
        assert!(!server.is_shutdown());
        assert_eq!(server.simulation().elapsed(), Duration::ZERO);
        server.shutdown();
    }
    // An occupied listener distinguishes pure admission errors from network setup.
    let occupied = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = format!("tcp/{}", occupied.local_addr().unwrap());
    for fault in [
        "no-intent",
        "empty",
        "one",
        "third",
        "duplicate-claim",
        "zero",
        "foreign",
        "wrong-kind",
        "missing-identity",
        "bad-width",
        "depth",
    ] {
        let count = match fault {
            "empty" => 0,
            "one" => 1,
            "third" => 3,
            _ => 2,
        };
        let mut sim = rgb_pair_simulation(count, fault != "no-intent");
        let cameras: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
            .map(|node| (node.id(), node.get::<EntityId>().unwrap().get()))
            .collect();
        let mut ids = [
            cameras.first().map_or(1, |row| row.1),
            cameras.get(1).map_or(2, |row| row.1),
        ];
        match fault {
            "duplicate-claim" => ids[1] = ids[0],
            "zero" => ids[0] = 0,
            "foreign" => ids[1] = u64::MAX,
            "wrong-kind" => {
                sim.world_mut()
                    .entity_mut(cameras[1].0)
                    .insert(gz_sim_scene::EntityKind::Link);
            }
            "missing-identity" => {
                sim.world_mut()
                    .entity_mut(cameras[1].0)
                    .remove::<EntityId>();
            }
            "bad-width" | "depth" => {
                let mut sensor = sim
                    .world()
                    .get::<gz_sim_scene::CameraSensor>(cameras[1].0)
                    .unwrap()
                    .0
                    .as_ref()
                    .clone();
                if fault == "depth" {
                    sensor.kind = "depth_camera".into();
                } else if let sdformat::SensorConfig::Camera(camera) = &mut sensor.config {
                    camera.image_width = 32;
                } else {
                    panic!("loaded RGB configuration");
                }
                sim.world_mut()
                    .entity_mut(cameras[1].0)
                    .insert(gz_sim_scene::CameraSensor(std::sync::Arc::new(sensor)));
            }
            _ => {}
        }
        let result = Server::with_external_camera_outputs(
            sim,
            "default",
            options(),
            config(&address),
            NonZeroUsize::new(1).unwrap(),
            ids,
        );
        assert!(
            matches!(result, Err(Error::CameraIdentity)),
            "fault={fault}"
        );
    }
    let sim = rgb_pair_simulation(2, true);
    let id = camera_id(&sim);
    assert!(
        matches!(
            Server::with_external_camera_output(
                sim,
                "default",
                options(),
                config(&address),
                NonZeroUsize::new(1).unwrap(),
                id,
            ),
            Err(Error::CameraIdentity)
        ),
        "the legacy scalar API still refuses two cameras"
    );
    assert!(matches!(
        Server::with_zenoh_config(
            rgb_pair_simulation(2, true),
            "default",
            options(),
            config(&address),
            NonZeroUsize::new(1).unwrap(),
        ),
        Err(Error::CameraOutputRequired)
    ));
}

fn depth_pair_simulation(count: usize, configured: bool) -> Simulation {
    let mut sim = rgb_pair_simulation(count, configured);
    let cameras: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
        .map(|node| node.id())
        .collect();
    for entity in cameras {
        let mut sensor = sim
            .world()
            .get::<gz_sim_scene::CameraSensor>(entity)
            .unwrap()
            .0
            .as_ref()
            .clone();
        sensor.kind = "depth_camera".into();
        sim.world_mut()
            .entity_mut(entity)
            .insert(gz_sim_scene::CameraSensor(std::sync::Arc::new(sensor)));
    }
    sim
}

#[test]
fn external_depth_pair_requires_its_own_complete_homogeneous_family() {
    for reversed in [false, true] {
        let sim = depth_pair_simulation(2, true);
        let mut ids: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
            .map(|node| node.get::<EntityId>().unwrap().get())
            .collect();
        ids.sort_unstable();
        if reversed {
            ids.reverse();
        }
        let mut server = Server::with_external_depth_camera_outputs(
            sim,
            "default",
            options(),
            config("tcp/127.0.0.1:0"),
            NonZeroUsize::new(1).unwrap(),
            [ids[0], ids[1]],
        )
        .unwrap();
        assert_eq!(server.simulation().elapsed(), Duration::ZERO);
        assert!(!server.is_shutdown());
        server.shutdown();
        assert!(server.is_shutdown());
    }
    let occupied = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = format!("tcp/{}", occupied.local_addr().unwrap());
    for scalar in [false, true] {
        let sim = depth_pair_simulation(2, true);
        let mut ids: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
            .map(|node| node.get::<EntityId>().unwrap().get())
            .collect();
        ids.sort_unstable();
        let result = if scalar {
            Server::with_external_camera_output(
                sim,
                "default",
                options(),
                config(&address),
                NonZeroUsize::new(1).unwrap(),
                ids[0],
            )
        } else {
            Server::with_external_camera_outputs(
                sim,
                "default",
                options(),
                config(&address),
                NonZeroUsize::new(1).unwrap(),
                [ids[0], ids[1]],
            )
        };
        assert!(
            matches!(result, Err(Error::CameraIdentity)),
            "legacy scalar and RGB pair APIs must refuse depth pairs before network setup"
        );
    }
}

#[test]
fn external_depth_pair_rejects_cardinality_identity_and_rgb_before_network() {
    let occupied = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = format!("tcp/{}", occupied.local_addr().unwrap());
    for fault in [
        "empty",
        "one",
        "third",
        "no-intent",
        "duplicate-claim",
        "zero",
        "foreign",
        "wrong-kind",
        "missing-identity",
        "rgb-pair",
        "mixed",
        "bad-width",
    ] {
        let count = match fault {
            "empty" => 0,
            "one" => 1,
            "third" => 3,
            _ => 2,
        };
        let mut sim = depth_pair_simulation(count, fault != "no-intent");
        let cameras: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
            .map(|node| (node.id(), node.get::<EntityId>().unwrap().get()))
            .collect();
        let mut ids = [
            cameras.first().map_or(1, |row| row.1),
            cameras.get(1).map_or(2, |row| row.1),
        ];
        match fault {
            "duplicate-claim" => ids[1] = ids[0],
            "zero" => ids[0] = 0,
            "foreign" => ids[1] = u64::MAX,
            "wrong-kind" => {
                sim.world_mut()
                    .entity_mut(cameras[1].0)
                    .insert(gz_sim_scene::EntityKind::Link);
            }
            "missing-identity" => {
                sim.world_mut()
                    .entity_mut(cameras[1].0)
                    .remove::<EntityId>();
            }
            "rgb-pair" | "mixed" | "bad-width" => {
                for (index, (entity, _)) in cameras.iter().enumerate() {
                    if fault != "rgb-pair" && index != 1 {
                        continue;
                    }
                    let mut sensor = sim
                        .world()
                        .get::<gz_sim_scene::CameraSensor>(*entity)
                        .unwrap()
                        .0
                        .as_ref()
                        .clone();
                    if fault == "bad-width" {
                        if let sdformat::SensorConfig::Camera(camera) = &mut sensor.config {
                            camera.image_width = 32;
                        } else {
                            panic!("loaded camera configuration");
                        }
                    } else {
                        sensor.kind = "camera".into();
                    }
                    sim.world_mut()
                        .entity_mut(*entity)
                        .insert(gz_sim_scene::CameraSensor(std::sync::Arc::new(sensor)));
                }
            }
            _ => {}
        }
        assert!(
            matches!(
                Server::with_external_depth_camera_outputs(
                    sim,
                    "default",
                    options(),
                    config(&address),
                    NonZeroUsize::new(1).unwrap(),
                    ids,
                ),
                Err(Error::CameraIdentity)
            ),
            "fault={fault}"
        );
    }
}

fn mixed_pair_simulation(count: usize, configured: bool) -> Simulation {
    let mut sim = rgb_pair_simulation(count, configured);
    let camera = sim
        .world()
        .iter_entities()
        .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
        .nth(1)
        .map(|node| node.id());
    if let Some(entity) = camera {
        let mut sensor = sim
            .world()
            .get::<gz_sim_scene::CameraSensor>(entity)
            .unwrap()
            .0
            .as_ref()
            .clone();
        sensor.kind = "depth_camera".into();
        sim.world_mut()
            .entity_mut(entity)
            .insert(gz_sim_scene::CameraSensor(std::sync::Arc::new(sensor)));
    }
    sim
}

#[test]
fn external_mixed_pair_uses_loaded_types_and_unordered_complete_ids() {
    for reversed in [false, true] {
        let sim = mixed_pair_simulation(2, true);
        let mut ids: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
            .map(|node| node.get::<EntityId>().unwrap().get())
            .collect();
        if reversed {
            ids.reverse();
        }
        let mut server = Server::with_external_mixed_camera_outputs(
            sim,
            "default",
            options(),
            config("tcp/127.0.0.1:0"),
            NonZeroUsize::new(1).unwrap(),
            [ids[0], ids[1]],
        )
        .unwrap();
        assert_eq!(server.simulation().elapsed(), Duration::ZERO);
        assert_eq!(server.simulation().epoch(), 0);
        assert!(!server.is_shutdown());
        server.shutdown();
        assert!(server.is_shutdown());
    }
}

#[test]
fn external_rgb_family_preserves_complete_ids_clock_reset_and_world_edit_boundary() {
    use gz_sim_proto::{msgs::Clock, prost::Message};
    for count in [3, 4] {
        for reversed in [false, true] {
            let sim = rgb_pair_simulation(count, true);
            let mut ids: Vec<_> = sim
                .world()
                .iter_entities()
                .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
                .map(|node| node.get::<EntityId>().unwrap().get())
                .collect();
            ids.sort_unstable();
            let expected_ids: BTreeSet<_> = ids.iter().copied().collect();
            if reversed {
                ids.reverse();
            }
            let mut server = Server::with_external_rgb_camera_family(
                sim,
                "default",
                options(),
                config("tcp/127.0.0.1:0"),
                NonZeroUsize::new(1).unwrap(),
                ids,
            )
            .unwrap();
            let camera_ids = |server: &Server| -> BTreeSet<u64> {
                server
                    .simulation()
                    .world()
                    .iter_entities()
                    .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
                    .map(|node| node.get::<EntityId>().unwrap().get())
                    .collect()
            };
            assert_eq!(camera_ids(&server), expected_ids);
            assert_eq!(server.simulation().elapsed(), Duration::ZERO);
            let model = server
                .simulation()
                .world()
                .iter_entities()
                .find(|node| {
                    node.get::<gz_sim_scene::EntityKind>() == Some(&gz_sim_scene::EntityKind::Model)
                })
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get();
            let before_entities = server.simulation().world().entities().len();
            assert!(matches!(
                server.remove_editable_model(model),
                Err(Error::CameraWorldEdit)
            ));
            assert_eq!(
                server.simulation().world().entities().len(),
                before_entities
            );
            assert_eq!(camera_ids(&server), expected_ids);
            assert_eq!(server.simulation().elapsed(), Duration::ZERO);

            // Observe the world clock through the actual shared IO owner, with
            // no external image writer or renderer required by this boundary.
            let session = server.zenoh_session();
            let (send, receive) = std::sync::mpsc::sync_channel(4);
            let key = server.options().topic_key("/world/default/clock").unwrap();
            let subscriber = session
                .declare_subscriber(key)
                .allowed_origin(zenoh::sample::Locality::SessionLocal)
                .callback(move |sample| {
                    let _ = send.try_send(sample);
                })
                .wait()
                .unwrap();
            assert!(server.apply_local_control(WorldControl {
                pause: false,
                ..Default::default()
            }));
            let advanced = server.update().unwrap();
            assert_eq!(advanced.sim_time, Duration::from_millis(1));
            let clock = Clock::decode(
                receive
                    .recv_timeout(Duration::from_secs(3))
                    .unwrap()
                    .payload()
                    .to_bytes()
                    .as_ref(),
            )
            .unwrap();
            assert_eq!(
                clock.sim,
                Some(gz_sim_proto::msgs::Time {
                    sec: 0,
                    nsec: 1_000_000
                })
            );
            assert_eq!(camera_ids(&server), expected_ids);
            assert!(server.apply_local_control(WorldControl {
                pause: true,
                reset: Some(gz_sim_proto::msgs::WorldReset {
                    all: true,
                    ..Default::default()
                }),
                ..Default::default()
            }));
            let reset = server.update().unwrap();
            assert_eq!((reset.epoch, reset.sim_time), (1, Duration::ZERO));
            assert_eq!(camera_ids(&server), expected_ids);
            drop(subscriber);
            server.shutdown();
            assert!(server.is_shutdown());
            assert!(matches!(server.update(), Err(Error::WorldClosed)));
        }
    }
}

#[test]
fn external_rgb_family_refuses_incomplete_or_forged_membership_before_network() {
    let occupied = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = format!("tcp/{}", occupied.local_addr().unwrap());
    for fault in [
        "empty",
        "two",
        "five",
        "no-intent",
        "too-few-claims",
        "too-many-claims",
        "zero",
        "duplicate-claim",
        "foreign",
        "wrong-kind",
        "missing-identity",
        "moved-identity",
        "bad-width",
        "depth",
        "missing-camera-config",
    ] {
        let count = match fault {
            "empty" => 0,
            "two" => 2,
            "five" => 5,
            _ => 4,
        };
        let mut sim = rgb_pair_simulation(count, fault != "no-intent");
        let cameras: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
            .map(|node| (node.id(), node.get::<EntityId>().unwrap().get()))
            .collect();
        let mut ids: Vec<_> = cameras.iter().map(|row| row.1).collect();
        if ids.is_empty() {
            ids = vec![1, 2, 3];
        }
        match fault {
            "too-few-claims" => {
                ids.pop();
            }
            "too-many-claims" => {
                sim.world_mut()
                    .entity_mut(cameras[3].0)
                    .remove::<gz_sim_scene::CameraSensor>();
            }
            "zero" => ids[3] = 0,
            "duplicate-claim" => ids[3] = ids[0],
            "foreign" => ids[3] = u64::MAX,
            "wrong-kind" => {
                sim.world_mut()
                    .entity_mut(cameras[3].0)
                    .insert(gz_sim_scene::EntityKind::Link);
            }
            "missing-identity" => {
                sim.world_mut()
                    .entity_mut(cameras[3].0)
                    .remove::<EntityId>();
            }
            "moved-identity" => {
                // Taking the immutable component retires its index entry;
                // reinsertion alone cannot recreate its original ownership.
                let identity = sim
                    .world_mut()
                    .entity_mut(cameras[3].0)
                    .take::<EntityId>()
                    .unwrap();
                sim.world_mut().entity_mut(cameras[3].0).insert(identity);
            }
            "bad-width" | "depth" | "missing-camera-config" => {
                let mut sensor = sim
                    .world()
                    .get::<gz_sim_scene::CameraSensor>(cameras[3].0)
                    .unwrap()
                    .0
                    .as_ref()
                    .clone();
                if fault == "depth" {
                    sensor.kind = "depth_camera".into();
                } else if fault == "missing-camera-config" {
                    sensor.config = sdformat::SensorConfig::Unconfigured;
                } else if let sdformat::SensorConfig::Camera(camera) = &mut sensor.config {
                    camera.image_width = 32;
                } else {
                    panic!("loaded RGB camera");
                }
                sim.world_mut()
                    .entity_mut(cameras[3].0)
                    .insert(gz_sim_scene::CameraSensor(std::sync::Arc::new(sensor)));
            }
            _ => {}
        }
        let result = Server::with_external_rgb_camera_family(
            sim,
            "default",
            options(),
            config(&address),
            NonZeroUsize::new(1).unwrap(),
            ids,
        );
        assert!(
            matches!(&result, Err(Error::CameraIdentity)),
            "fault={fault}, error={:?}",
            result.err()
        );
    }
}

#[test]
fn external_rgb_family_does_not_implicitly_expand_legacy_camera_constructors() {
    let occupied = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = format!("tcp/{}", occupied.local_addr().unwrap());
    for count in [3, 4] {
        for mode in 0..5 {
            let sim = rgb_pair_simulation(count, true);
            let ids: Vec<_> = sim
                .world()
                .iter_entities()
                .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
                .map(|node| node.get::<EntityId>().unwrap().get())
                .collect();
            let result = match mode {
                0 => Server::with_zenoh_config(
                    sim,
                    "default",
                    options(),
                    config(&address),
                    NonZeroUsize::new(1).unwrap(),
                ),
                1 => Server::with_external_camera_output(
                    sim,
                    "default",
                    options(),
                    config(&address),
                    NonZeroUsize::new(1).unwrap(),
                    ids[0],
                ),
                2 => Server::with_external_camera_outputs(
                    sim,
                    "default",
                    options(),
                    config(&address),
                    NonZeroUsize::new(1).unwrap(),
                    [ids[0], ids[1]],
                ),
                3 => Server::with_external_depth_camera_outputs(
                    sim,
                    "default",
                    options(),
                    config(&address),
                    NonZeroUsize::new(1).unwrap(),
                    [ids[0], ids[1]],
                ),
                _ => Server::with_external_mixed_camera_outputs(
                    sim,
                    "default",
                    options(),
                    config(&address),
                    NonZeroUsize::new(1).unwrap(),
                    [ids[0], ids[1]],
                ),
            };
            assert!(
                if mode == 0 {
                    matches!(&result, Err(Error::CameraOutputRequired))
                } else {
                    matches!(&result, Err(Error::CameraIdentity))
                },
                "mode={mode}, error={:?}",
                result.err()
            );
        }
    }
}

#[test]
fn external_mixed_pair_rejects_invalid_complete_sets_before_network() {
    let occupied = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = format!("tcp/{}", occupied.local_addr().unwrap());
    for fault in [
        "empty",
        "one",
        "third",
        "no-intent",
        "duplicate-claim",
        "zero",
        "foreign",
        "wrong-kind",
        "missing-identity",
        "rgb-pair",
        "depth-pair",
        "unsupported",
        "bad-width",
    ] {
        let count = match fault {
            "empty" => 0,
            "one" => 1,
            "third" => 3,
            _ => 2,
        };
        let mut sim = mixed_pair_simulation(count, fault != "no-intent");
        let cameras: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
            .map(|node| (node.id(), node.get::<EntityId>().unwrap().get()))
            .collect();
        let mut ids = [
            cameras.first().map_or(1, |row| row.1),
            cameras.get(1).map_or(2, |row| row.1),
        ];
        match fault {
            "duplicate-claim" => ids[1] = ids[0],
            "zero" => ids[0] = 0,
            "foreign" => ids[1] = u64::MAX,
            "wrong-kind" => {
                sim.world_mut()
                    .entity_mut(cameras[1].0)
                    .insert(gz_sim_scene::EntityKind::Link);
            }
            "missing-identity" => {
                sim.world_mut()
                    .entity_mut(cameras[1].0)
                    .remove::<EntityId>();
            }
            "rgb-pair" | "depth-pair" | "unsupported" | "bad-width" => {
                for (index, (entity, _)) in cameras.iter().enumerate() {
                    if fault != "rgb-pair" && fault != "depth-pair" && index != 1 {
                        continue;
                    }
                    let mut sensor = sim
                        .world()
                        .get::<gz_sim_scene::CameraSensor>(*entity)
                        .unwrap()
                        .0
                        .as_ref()
                        .clone();
                    match fault {
                        "rgb-pair" => sensor.kind = "camera".into(),
                        "depth-pair" => sensor.kind = "depth_camera".into(),
                        "unsupported" => sensor.kind = "wideanglecamera".into(),
                        _ => {
                            let sdformat::SensorConfig::Camera(camera) = &mut sensor.config else {
                                panic!("loaded camera configuration");
                            };
                            camera.image_width = 32;
                        }
                    }
                    sim.world_mut()
                        .entity_mut(*entity)
                        .insert(gz_sim_scene::CameraSensor(std::sync::Arc::new(sensor)));
                }
            }
            _ => {}
        }
        assert!(
            matches!(
                Server::with_external_mixed_camera_outputs(
                    sim,
                    "default",
                    options(),
                    config(&address),
                    NonZeroUsize::new(1).unwrap(),
                    ids,
                ),
                Err(Error::CameraIdentity)
            ),
            "fault={fault}"
        );
    }
    // The new explicit mode never broadens any legacy external-owner API.
    for mode in [0, 1, 2] {
        let sim = mixed_pair_simulation(2, true);
        let ids: Vec<_> = sim
            .world()
            .iter_entities()
            .filter(|node| node.contains::<gz_sim_scene::CameraSensor>())
            .map(|node| node.get::<EntityId>().unwrap().get())
            .collect();
        let result = match mode {
            0 => Server::with_external_camera_output(
                sim,
                "default",
                options(),
                config(&address),
                NonZeroUsize::new(1).unwrap(),
                ids[0],
            ),
            1 => Server::with_external_camera_outputs(
                sim,
                "default",
                options(),
                config(&address),
                NonZeroUsize::new(1).unwrap(),
                [ids[0], ids[1]],
            ),
            _ => Server::with_external_depth_camera_outputs(
                sim,
                "default",
                options(),
                config(&address),
                NonZeroUsize::new(1).unwrap(),
                [ids[0], ids[1]],
            ),
        };
        assert!(
            matches!(result, Err(Error::CameraIdentity)),
            "legacy mode={mode}"
        );
    }
}
