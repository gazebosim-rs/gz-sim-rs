// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::BodyHandle;
use std::io::{Read, Write};

const ROBOT: &str = include_str!("../../../../examples/local_robot_gripper.sdf");
const OBJ: &[u8] = include_bytes!("../../../../examples/meshes/m16-asymmetric.obj");
const MARKER: &str = "default::parcel::box::marker";
const TICKS: u64 = 180;

// Drop the independently colliding, marked parcel beside the hand onto the receiving table.
fn drop_world() -> String {
    let before = r#"<pose>0 0 0.171 0 0 0</pose>"#;
    let visual = r#"<visual name="box">"#;
    assert_eq!(ROBOT.matches(before).count(), 1);
    assert_eq!(ROBOT.matches(visual).count(), 1);
    ROBOT
        .replace(before, r#"<pose>0.36 0 0.23 0 0 0</pose>"#)
        .replace(visual, &format!(r#"<visual name="marker"><pose>-0.015 -0.012 0.05 0 0 0</pose><geometry><mesh><uri>marker.obj</uri><scale>0.03 0.012 0.01</scale></mesh></geometry></visual>{visual}"#))
}

fn mesh_bytes(uri: &str) -> Vec<u8> {
    let url = url::Url::parse(uri).unwrap();
    assert_eq!(url.host_str(), Some("127.0.0.1"));
    assert!(url.path().ends_with("/marker.obj.glb"));
    let mut socket = TcpStream::connect((Ipv4Addr::LOCALHOST, url.port().unwrap())).unwrap();
    socket.set_read_timeout(Some(TIMEOUT)).unwrap();
    socket.set_write_timeout(Some(TIMEOUT)).unwrap();
    write!(socket, "GET {} HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\nOrigin: http://127.0.0.1:8082\r\n\r\n", url.path()).unwrap();
    let mut response = Vec::new();
    socket.read_to_end(&mut response).unwrap();
    assert!(response.starts_with(b"HTTP/1.1 200"));
    let end = response.windows(4).position(|b| b == b"\r\n\r\n").unwrap();
    response.split_off(end + 4)
}

#[test]
fn replay_serves_the_moving_robot_payload_after_source_retirement() {
    let source = tempfile::tempdir().unwrap();
    let work = tempfile::tempdir().unwrap();
    let path = source.path().join("drop.sdf");
    let xml = drop_world();
    std::fs::write(&path, &xml).unwrap();
    std::fs::write(source.path().join("marker.obj"), OBJ).unwrap();
    let archive = std::env::var_os("GZ_LOCAL_MESH_REPLAY")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| work.path().join("capture"));
    record(&path, &archive, TICKS, 1_000_000);
    let mut sim = load_world_with_sdf_step(
        &xml,
        url::Url::from_file_path(&path).unwrap().as_str(),
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    source.close().unwrap();
    let body = |name| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .get::<BodyHandle>()
            .unwrap()
            .id()
    };
    let (parcel, table) = (body("default::parcel::box"), body("default::target::body"));
    let mut marker = Vec::new();
    let mut impact = (0, 0.0_f64);
    for tick in 0..=TICKS {
        if tick != 0 {
            sim.step_exact(1).unwrap();
            let backend = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            let impulse = backend
                .rigid_contact_patches()
                .iter()
                .filter(|p| (p.a == parcel && p.b == table) || (p.a == table && p.b == parcel))
                .flat_map(|p| &p.points)
                .map(|p| p.normal_impulse)
                .sum();
            if impulse > impact.1 {
                impact = (tick, impulse);
            }
        }
        marker.push(
            *sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == MARKER))
                .unwrap()
                .get::<WorldPose>()
                .unwrap(),
        );
    }
    assert!(
        (100..130).contains(&impact.0) && impact.1 > 0.1,
        "impact {impact:?}"
    );
    assert!(marker[0].translation[2] - marker[TICKS as usize].translation[2] > 0.055);

    let expected = gz_assets::mesh::obj_visual_glb(OBJ).unwrap();
    let (mut process, mut socket) = player(&archive);
    let mut frame = read_tuple(&mut socket, "publication_credit");
    let mut ordinal = 0;
    let mut asset = None;
    loop {
        ordinal += 1;
        assert_eq!(frame[2]["frame"], ordinal);
        let ns = frame[2]["recorded_nanoseconds"].as_u64().unwrap();
        assert!(ns.is_multiple_of(1_000_000) && ns <= TICKS * 1_000_000);
        let visual = frame[0]["visuals"]
            .as_array()
            .unwrap()
            .iter()
            .find(|v| v["name"] == MARKER)
            .unwrap();
        let pose = marker[(ns / 1_000_000) as usize];
        assert_eq!(visual["translation"], json!(pose.translation));
        assert_eq!(visual["rotation"], json!(pose.rotation));
        let uri = visual["shape"]["uri"].as_str().unwrap();
        assert_eq!(visual["shape"]["scale"], json!([0.03, 0.012, 0.01]));
        if let Some(url) = &asset {
            assert_eq!(uri, url);
        } else {
            assert_eq!(mesh_bytes(uri), expected);
            asset = Some(uri.to_owned());
            assert_eq!(ns, 0);
            socket
                .send(Message::text(
                    json!({"version":1,"kind":"control","request_id":1,"action":"resume"})
                        .to_string(),
                ))
                .unwrap();
            assert_eq!(read_tuple(&mut socket, "ack")[3]["ok"], true);
        }
        if frame[2]["finished"] == true {
            assert_eq!(ns, TICKS * 1_000_000);
            break;
        }
        socket
            .send(Message::text(
                json!({"version":1,"kind":"publication_ready","token":frame[3]["token"]})
                    .to_string(),
            ))
            .unwrap();
        frame = read_tuple(&mut socket, "publication_credit");
    }
    assert!(ordinal >= 2);
    socket
        .send(Message::text(
            json!({"version":1,"kind":"control","request_id":2,"action":"reset"}).to_string(),
        ))
        .unwrap();
    let rewind = read_tuple(&mut socket, "ack");
    assert_eq!(rewind[3]["ok"], true);
    assert_eq!(rewind[2]["frame"], 1);
    assert_eq!(mesh_bytes(asset.as_ref().unwrap()), expected);
    socket.close(None).unwrap();
    rustix::process::kill_process(
        rustix::process::Pid::from_child(process.0.as_ref().unwrap()),
        rustix::process::Signal::TERM,
    )
    .unwrap();
    assert!(process.wait(TIMEOUT).status.success());
    println!(
        "replayed {ordinal} frames, {} GLB bytes, peak contact {:.9} N s at {} ms",
        expected.len(),
        impact.1,
        impact.0
    );
}
