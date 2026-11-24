// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../../../tests/support/zenoh_peer.rs"]
mod selected_native;
use gz_sim_proto::{msgs::PointCloudPacked, prost::Message};
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use gz_sim_server::{Server, ServerOptions};
use std::{
    io::Read,
    num::NonZeroUsize,
    path::Path,
    process::{Child, Command, Stdio},
    thread::JoinHandle,
    time::{Duration, Instant},
};
use zenoh::Wait;
struct Peer {
    child: Child,
    out: Option<JoinHandle<Vec<u8>>>,
    err: Option<JoinHandle<Vec<u8>>>,
}
impl Drop for Peer {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        if let Some(t) = self.out.take() {
            let _ = t.join();
        }
        if let Some(t) = self.err.take() {
            let _ = t.join();
        }
    }
}
fn capture(stream: impl Read + Send + 'static, cap: u64) -> JoinHandle<Vec<u8>> {
    std::thread::spawn(move || {
        let mut bytes = Vec::new();
        stream.take(cap + 1).read_to_end(&mut bytes).unwrap();
        bytes
    })
}
#[test]
fn real_cpp_points_consumer_observes_native_field_layout_and_organized_sensor_coordinates() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let fixture = selected_native::NativePeer::build(&root, "lidar_points");
    let mut cfg = zenoh::Config::default();
    for (k, v) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        cfg.insert_json5(k, v).unwrap();
    }
    let observer = zenoh::open(cfg).wait().unwrap();
    let endpoints: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|x| x.to_string())
        .collect();
    assert_eq!(endpoints.len(), 1);
    assert_ne!(endpoints[0], "tcp/127.0.0.1:0");
    let json = serde_json::json!({"mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":endpoints,"timeout_ms":2000},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"transport":{"shared_memory":{"enabled":false}}});
    let dir = tempfile::tempdir().unwrap();
    let config = dir.path().join("client.json");
    std::fs::write(&config, json.to_string()).unwrap();
    let xml = "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='rig'><static>true</static><link name='body'><collision name='self'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision><sensor name='scan' type='lidar'><update_rate>100</update_rate><topic>/m114/scan</topic><frame_id>grid_frame</frame_id><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal><vertical><samples>3</samples><resolution>1</resolution><min_angle>-0.2</min_angle><max_angle>0.2</max_angle></vertical></scan><range><min>0</min><max>20</max></range></lidar></sensor></link></model><model name='wall'><static>true</static><pose>5 0 0 0 0 0</pose><link name='body'><collision name='wall'><geometry><box><size>2 100 100</size></box></geometry></collision></link></model></world></sdf>";
    let mut sim = load_world_with_sdf_step(
        xml,
        "points-native",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    sim.set_paused(false);
    let options = ServerOptions {
        partition: format!("m114-{}", uuid::Uuid::new_v4()),
        lidar_point_clouds: true,
        ..Default::default()
    };
    let mut server = Server::with_zenoh_config(
        sim,
        "default",
        options.clone(),
        zenoh::Config::from_json5(&json.to_string()).unwrap(),
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    let mut cmd = Command::new(&fixture.binary);
    fixture.configure_runtime(&mut cmd);
    cmd.env("GZ_PARTITION", &options.partition)
        .env("GZ_IP", "127.0.0.1")
        .env("ZENOH_CONFIG", &config)
        .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
        .env_remove("GZ_DISCOVERY_MSG_PORT")
        .env_remove("GZ_DISCOVERY_SRV_PORT")
        .env_remove("GZ_DISCOVERY_MULTICAST_IP")
        .env_remove("GZ_TRANSPORT_USERNAME")
        .env_remove("GZ_TRANSPORT_PASSWORD")
        .env_remove("GZ_RELAY")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = cmd.spawn().unwrap();
    let out = capture(child.stdout.take().unwrap(), 32768);
    let err = capture(child.stderr.take().unwrap(), 4096);
    let mut peer = Peer {
        child,
        out: Some(out),
        err: Some(err),
    };
    let end = Instant::now() + Duration::from_secs(20);
    let status = loop {
        if let Some(status) = peer.child.try_wait().unwrap() {
            break status;
        }
        assert!(Instant::now() < end, "native point observer deadline");
        server.update().unwrap();
        std::thread::sleep(Duration::from_millis(2));
    };
    let out = peer.out.take().unwrap().join().unwrap();
    let err = peer.err.take().unwrap().join().unwrap();
    assert!(out.len() <= 32768 && err.len() <= 4096);
    assert!(status.success(), "{}", String::from_utf8_lossy(&err));
    let text = std::str::from_utf8(&out).unwrap();
    let hex = text.trim_end().strip_prefix("PACKET ").unwrap();
    assert!(hex.len() <= 8192 && hex.len().is_multiple_of(2));
    let bytes: Vec<u8> = hex
        .as_bytes()
        .chunks_exact(2)
        .map(|p| {
            let digit = |x: u8| match x {
                b'0'..=b'9' => x - b'0',
                b'a'..=b'f' => x - b'a' + 10,
                _ => panic!("noncanonical hex"),
            };
            digit(p[0]) * 16 + digit(p[1])
        })
        .collect();
    assert!(bytes.len() <= 4096);
    let p = PointCloudPacked::decode(bytes.as_slice()).unwrap();
    assert_eq!(p.encode_to_vec(), bytes);
    assert_eq!(
        (p.width, p.height, p.point_step, p.row_step, p.data.len()),
        (3, 3, 18, 54, 162)
    );
    assert!(p.is_dense && !p.is_bigendian);
    assert_eq!(
        p.field
            .iter()
            .map(|f| (f.name.as_str(), f.offset, f.datatype, f.count))
            .collect::<Vec<_>>(),
        [
            ("x", 0, 6, 1),
            ("y", 4, 6, 1),
            ("z", 8, 6, 1),
            ("intensity", 12, 6, 1),
            ("ring", 16, 3, 1)
        ]
    );
    assert_eq!(p.header.as_ref().unwrap().data[0].value, ["grid_frame"]);
    for v in 0..3 {
        for h in 0..3 {
            let i = v * 3 + h;
            let yaw = -0.3 + h as f64 * 0.3;
            let pitch = -0.2 + v as f64 * 0.2;
            for (axis, expected) in [4., 4. * yaw.tan(), 4. * pitch.tan() / yaw.cos()]
                .into_iter()
                .enumerate()
            {
                let at = i * 18 + axis * 4;
                let actual = f32::from_le_bytes(p.data[at..at + 4].try_into().unwrap());
                assert!((f64::from(actual) - expected).abs() < 1e-6);
            }
            assert_eq!(&p.data[i * 18 + 12..i * 18 + 16], &0_f32.to_le_bytes());
            assert_eq!(
                u16::from_le_bytes(p.data[i * 18 + 16..i * 18 + 18].try_into().unwrap()),
                v as u16
            );
        }
    }
    server.shutdown();
    drop(observer);
}
