// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::net::TcpListener;
use tungstenite::protocol::Role;

// A rejected input must stop the capture before it can certify a view of the
// wrong object. Match actual CDP requests on a loopback browser connection.
#[test]
fn interrupted_camera_inputs_propagate_to_the_recorder() {
    let failures = [
        ("Page.bringToFront", "", "", 0, true),
        ("Input.dispatchMouseEvent", "mouseMoved", "", 743, true),
        ("Input.dispatchKeyEvent", "keyDown", "h", 0, true),
        ("Input.dispatchKeyEvent", "keyDown", "f", 0, true),
        ("Input.dispatchKeyEvent", "keyDown", "Shift", 0, true),
        ("Input.dispatchKeyEvent", "keyDown", "F", 0, true),
        ("Input.dispatchKeyEvent", "keyUp", "Shift", 0, true),
        ("Input.dispatchMouseEvent", "mouseWheel", "", 745, true),
        ("Input.dispatchMouseEvent", "mouseWheel", "", 745, false),
    ];
    thread::scope(|group| {
        for (method, event, key, x, follow_sheet) in failures {
            group.spawn(move || {
                let listener = TcpListener::bind(("127.0.0.1", 0)).unwrap();
                let client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
                let server = listener.accept().unwrap().0;
                let endpoint = thread::spawn(move || {
                    let mut peer = WebSocket::from_raw_socket(server, Role::Server, None);
                    loop {
                        let msg: Value =
                            serde_json::from_slice(&peer.read().unwrap().into_data()).unwrap();
                        let params = &msg["params"];
                        let reject = msg["method"] == method
                            && params["type"].as_str().unwrap_or("") == event
                            && params["key"].as_str().unwrap_or("") == key
                            && params["x"].as_u64().unwrap_or(0) == x;
                        let reply = if reject {
                            json!({"id": msg["id"], "error": {"message": "input refused"}})
                        } else {
                            json!({"id": msg["id"], "result": {}})
                        };
                        peer.send(Message::Text(reply.to_string().into())).unwrap();
                        if reject {
                            break;
                        }
                    }
                });
                let mut chrome = Chrome {
                    socket: WebSocket::from_raw_socket(client, Role::Client, None),
                    session: None,
                    next: 0,
                    oracle: Oracle::default(),
                };
                assert!(
                    chrome
                        .focus_visual(follow_sheet, true)
                        .unwrap_err()
                        .to_string()
                        .contains("input refused")
                );
                endpoint.join().unwrap();
            });
        }
    });
}

#[test]
fn zoom_rejects_an_unpainted_or_refused_wheel_event() {
    for interrupted in ["Input.dispatchMouseEvent", "Runtime.evaluate"] {
        let listener = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let server = listener.accept().unwrap().0;
        let endpoint = thread::spawn(move || {
            let mut peer = WebSocket::from_raw_socket(server, Role::Server, None);
            for method in ["Input.dispatchMouseEvent", "Runtime.evaluate"] {
                let msg: Value = serde_json::from_slice(&peer.read().unwrap().into_data()).unwrap();
                assert_eq!(msg["method"], method);
                if method == interrupted {
                    let reply = json!({"id":msg["id"],"error":{"message":"camera interrupted"}});
                    peer.send(Message::Text(reply.to_string().into())).unwrap();
                    break;
                }
                let reply = json!({"id":msg["id"],"result":{}});
                peer.send(Message::Text(reply.to_string().into())).unwrap();
            }
        });
        let mut chrome = Chrome {
            socket: WebSocket::from_raw_socket(client, Role::Client, None),
            session: None,
            next: 0,
            oracle: Oracle::default(),
        };
        assert!(
            chrome
                .zoom_out(1)
                .unwrap_err()
                .to_string()
                .contains("camera interrupted")
        );
        endpoint.join().unwrap();
    }
}
