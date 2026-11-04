// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::{LidarPublication, LidarSample};
use serde_json::Value;
const XML: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../gz-sim-scene/tests/fixtures/planar_lidar.sdf"
));
fn simulation() -> Simulation {
    gz_sim_scene::load_world(
        XML,
        "scan.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap()
}
fn frame(sim: &Simulation) -> Value {
    serde_json::from_str(&scan::encode(sim).unwrap()).unwrap()
}
#[test]
fn measured_scan_payload_keeps_actual_ids_pose_units_nulls_and_reset_epochs() {
    let mut sim = simulation();
    assert!(frame(&sim)["scan"].is_null());
    let visual_count = sim
        .world()
        .iter_entities()
        .filter(|entity| {
            entity.get::<gz_sim_scene::EntityKind>() == Some(&gz_sim_scene::EntityKind::Visual)
        })
        .count();
    sim.step_exact(1).unwrap();
    let sample = frame(&sim);
    assert_eq!(sample["version"], 1);
    assert_eq!(sample["kind"], "scan");
    assert_eq!(sample["epoch"], 0);
    assert_eq!(sample["sim_time_ns"], 1_000_000_u64);
    assert_eq!(sample["scan"]["acquired_ns"], 1_000_000_u64);
    assert_eq!(
        sample["scan"]["sensor_position"],
        serde_json::json!([0.0, 0.0, 0.0])
    );
    assert!((sample["scan"]["ranges"][1].as_f64().unwrap() - 4.0).abs() < 1e-10);
    assert_eq!(sample["scan"]["ranges"].as_array().unwrap().len(), 3);
    let sensor = sim
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<LidarSample>())
        .unwrap()
        .id();
    let id = sim
        .world()
        .get::<gz_sim_core::EntityId>(sensor)
        .unwrap()
        .get();
    assert_eq!(sample["scan"]["sensor_id"], id);
    sim.world_mut()
        .get_mut::<LidarSample>(sensor)
        .unwrap()
        .ranges[0] = f64::INFINITY;
    assert!(frame(&sim)["scan"]["ranges"][0].is_null());
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|entity| entity.get::<gz_sim_scene::EntityKind>()
                == Some(&gz_sim_scene::EntityKind::Visual))
            .count(),
        visual_count
    );
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(frame(&sim)["epoch"], 1);
    assert!(frame(&sim)["scan"].is_null());
    sim.step_exact(1).unwrap();
    assert_eq!(frame(&sim)["scan"]["sensor_id"], id);
    assert_eq!(frame(&sim)["scan"]["acquired_ns"], 1_000_000_u64);
    gz_sim_scene::set_lidar_interest(&mut sim, []);
    assert!(frame(&sim)["scan"].is_null());
}
#[test]
fn corrupted_sample_values_are_refused_and_old_epochs_are_cleared() {
    for corruption in 0..12 {
        let mut sim = simulation();
        sim.step_exact(1).unwrap();
        let sensor = sim
            .world()
            .iter_entities()
            .find(|entity| entity.contains::<LidarSample>())
            .unwrap()
            .id();
        if corruption < 9 {
            let mut sample = sim.world_mut().get_mut::<LidarSample>(sensor).unwrap();
            match corruption {
                0 => sample.time = Duration::MAX,
                1 => sample.world_pose.translation[0] = f64::NAN,
                2 => sample.world_pose.rotation = [0.0; 4],
                3 => sample.ranges.clear(),
                4 => sample.ranges.push(1.0),
                5 => sample.ranges[0] = f64::NAN,
                6 => sample.ranges[0] = f64::NEG_INFINITY,
                7 => sample.ranges[0] = -1.0,
                _ => sample.ranges[0] = 11.0,
            }
        } else {
            let mut publication = sim.world_mut().get_mut::<LidarPublication>(sensor).unwrap();
            match corruption {
                9 => publication.config.range_min = -1.0,
                10 => publication.angle_step = 0.0,
                _ => publication.config.angle_max = f64::NAN,
            }
        }
        assert!(scan::encode(&sim).is_err(), "corruption {corruption}");
    }
    let mut sim = simulation();
    sim.step_exact(1).unwrap();
    let sensor = sim
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<LidarSample>())
        .unwrap()
        .id();
    sim.world_mut()
        .get_mut::<LidarSample>(sensor)
        .unwrap()
        .epoch = 9;
    assert!(frame(&sim)["scan"].is_null());
    assert!(scan::encode(&Simulation::new(Duration::from_millis(1)).unwrap()).is_err());
}
#[test]
fn real_opt_in_socket_gets_scene_before_measured_scan_and_invalid_opt_ins_fail() {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let address = listener.local_addr().unwrap();
    let options = gz_sim_server::ServerOptions {
        partition: format!("m7-scan-{}-{}", std::process::id(), address.port()),
        ..Default::default()
    };
    let mut sim = simulation();
    sim.set_paused(false);
    let mut server = Server::with_zenoh_config(
        sim,
        "default",
        options,
        zenoh::Config::default(),
        std::num::NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    let mut bridge = Bridge::with_listener(server.simulation(), "default", listener).unwrap();
    assert!(!bridge.scan_interested());
    let complete = Arc::new(AtomicBool::new(false));
    let done = complete.clone();
    let default_closed = Arc::new(AtomicBool::new(false));
    let client_closed = default_closed.clone();
    let (retired, retirement) = mpsc::sync_channel(1);
    let client = thread::spawn(move || {
        // Default clients retain the original Scene/control protocol. Do not
        // use a read-until-kind helper that could silently discard a scan.
        let (mut default_socket, _) = tungstenite::connect(format!("ws://{address}/")).unwrap();
        if let tungstenite::stream::MaybeTlsStream::Plain(tcp) = default_socket.get_mut() {
            tcp.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        }
        default_socket
            .send(tungstenite::Message::Text(
                serde_json::json!({"version":1,"kind":"control","request_id":1,"action":"resume"})
                    .to_string()
                    .into(),
            ))
            .unwrap();
        let mut saw_scene = false;
        let mut saw_ack = false;
        while !saw_scene || !saw_ack {
            if let tungstenite::Message::Text(text) = default_socket.read().unwrap() {
                let value: Value = serde_json::from_str(&text).unwrap();
                match value["kind"].as_str().unwrap() {
                    "scene" => saw_scene = true,
                    "ack" => {
                        assert_eq!(value["ok"], true);
                        saw_ack = true;
                    }
                    other => panic!("default client received unrequested frame {other}"),
                }
            }
        }
        default_socket
            .send(tungstenite::Message::Ping(vec![7, 11].into()))
            .unwrap();
        loop {
            match default_socket.read().unwrap() {
                tungstenite::Message::Text(text) => {
                    let value: Value = serde_json::from_str(&text).unwrap();
                    assert!(
                        matches!(value["kind"].as_str(), Some("scene" | "ack")),
                        "unrequested default-client frame: {value}"
                    );
                }
                tungstenite::Message::Pong(bytes) => {
                    assert_eq!(bytes.as_ref(), &[7, 11]);
                    break;
                }
                other => panic!("unexpected default-client FIFO barrier frame {other:?}"),
            }
        }
        let _ = default_socket.close(None);
        drop(default_socket);
        client_closed.store(true, Ordering::Release);
        retirement.recv_timeout(Duration::from_secs(3)).unwrap();
        for query in ["scan=0", "scan=1&scan=1", "scan"] {
            assert!(
                matches!(tungstenite::connect(format!("ws://{address}/?{query}")), Err(tungstenite::Error::Http(response)) if response.status() == 400)
            );
        }
        let (mut socket, _) = tungstenite::connect(format!("ws://{address}/?scan=1")).unwrap();
        if let tungstenite::stream::MaybeTlsStream::Plain(tcp) = socket.get_mut() {
            tcp.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        }
        let first: Value = loop {
            if let tungstenite::Message::Text(text) = socket.read().unwrap() {
                break serde_json::from_str(&text).unwrap();
            }
        };
        assert_eq!(first["kind"], "scene");
        let mut last_scene = first.clone();
        let scan: Value = loop {
            if let tungstenite::Message::Text(text) = socket.read().unwrap() {
                let value: Value = serde_json::from_str(&text).unwrap();
                if value["kind"] == "scene" {
                    last_scene = value;
                    continue;
                }
                if value["kind"] == "scan" && !value["scan"].is_null() {
                    break value;
                }
            }
        };
        assert_eq!(scan["world_id"], first["world_id"]);
        assert_eq!(scan["scan"]["ranges"].as_array().unwrap().len(), 3);
        assert!((scan["scan"]["ranges"][1].as_f64().unwrap() - 4.0).abs() < 1e-10);
        assert!(
            scan["scan"]["acquired_ns"].as_u64().unwrap() <= scan["sim_time_ns"].as_u64().unwrap()
        );
        socket
            .send(tungstenite::Message::Text(
                serde_json::json!({"version":1,"kind":"control","request_id":1,"action":"reset"})
                    .to_string()
                    .into(),
            ))
            .unwrap();
        let mut reset_ack = false;
        let mut reset_scan = false;
        while !reset_ack || !reset_scan {
            if let tungstenite::Message::Text(text) = socket.read().unwrap() {
                let value: Value = serde_json::from_str(&text).unwrap();
                match value["kind"].as_str().unwrap() {
                    "scene" => last_scene = value,
                    "scan" => {
                        assert_eq!(value["epoch"], last_scene["epoch"]);
                        assert_eq!(
                            value["sim_time_ns"].as_u64().unwrap(),
                            last_scene["seconds"].as_u64().unwrap() * 1_000_000_000
                                + last_scene["nanoseconds"].as_u64().unwrap()
                        );
                        if value["epoch"] == 1 {
                            assert!(value["scan"].is_null());
                            reset_scan = true;
                        }
                    }
                    "ack" => {
                        assert_eq!(value["request_id"], 1);
                        assert_eq!(value["ok"], true);
                        assert_eq!(value["epoch"], 1);
                        reset_ack = true;
                    }
                    other => panic!("unexpected frame {other}"),
                }
            }
        }
        let _ = socket.close(None);
        done.store(true, Ordering::Release);
    });
    let deadline = Instant::now() + Duration::from_secs(8);
    let mut retirement_sent = false;
    while !complete.load(Ordering::Acquire) && Instant::now() < deadline {
        if !retirement_sent
            && default_closed.load(Ordering::Acquire)
            && bridge.connected.load(Ordering::Acquire) == 0
            && !bridge.scan_interested()
        {
            assert!(!bridge.scan_interested());
            let _ = retired.try_send(());
            retirement_sent = true;
        }
        let controls = bridge.controls(&mut server);
        server.set_lidar_viewer_interest(bridge.scan_interested());
        server.prepare_io().unwrap();
        server.tick().unwrap();
        bridge
            .publish(server.simulation(), "default", controls)
            .unwrap();
        thread::sleep(Duration::from_millis(2));
    }
    assert!(
        complete.load(Ordering::Acquire),
        "bounded scan client did not finish"
    );
    client.join().unwrap();
    drop(bridge);
    server.shutdown();
}
