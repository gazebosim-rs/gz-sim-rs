// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{config, connect_until, endpoint_valid};
use std::{
    io,
    net::{Shutdown, TcpListener, TcpStream},
    thread,
    time::{Duration, Instant},
};
use zenoh::Wait;

#[test]
fn delayed_handshake_uses_the_remaining_native_bootstrap_budget() -> io::Result<()> {
    let peer = zenoh::open(config(None)?)
        .wait()
        .map_err(io::Error::other)?;
    let address = peer
        .info()
        .locators()
        .wait()
        .iter()
        .map(ToString::to_string)
        .find(|locator| endpoint_valid(locator))
        .expect("peer must have a loopback listener");
    let proxy = TcpListener::bind(("127.0.0.1", 0))?;
    proxy.set_nonblocking(true)?;
    let endpoint = format!("tcp/{}", proxy.local_addr()?);
    let deadline = Instant::now() + Duration::from_secs(6);
    let connected_to_peer = thread::scope(|scope| -> io::Result<bool> {
        let opening = scope.spawn(|| connect_until(&endpoint, deadline));
        let client = loop {
            match proxy.accept() {
                Ok((stream, _)) => break stream,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock && Instant::now() < deadline => {
                    thread::sleep(Duration::from_millis(5));
                }
                Err(e) => return Err(e),
            }
        };
        // TCP has connected, but neither side sees Zenoh handshake bytes until
        // after the old one-second client connection timeout.
        thread::sleep(Duration::from_millis(1300));
        let server = TcpStream::connect(address.strip_prefix("tcp/").unwrap())?;
        thread::scope(|relay| {
            relay.spawn(|| io::copy(&mut &client, &mut &server));
            relay.spawn(|| io::copy(&mut &server, &mut &client));
            let opened = opening.join();
            let connected = opened.map(|session| {
                session.map(|session| session.info().peers_zid().wait().any(|id| id == peer.zid()))
            });
            let _ = client.shutdown(Shutdown::Both);
            let _ = server.shutdown(Shutdown::Both);
            connected.expect("connection thread panicked")
        })
    })?;
    assert!(
        connected_to_peer,
        "client must reach the intended live peer"
    );
    Ok(())
}

#[test]
fn stalled_handshake_exhausts_the_supplied_deadline() -> io::Result<()> {
    // The kernel accepts TCP but the listener never supplies a Zenoh handshake.
    let listener = TcpListener::bind(("127.0.0.1", 0))?;
    let endpoint = format!("tcp/{}", listener.local_addr()?);
    let start = Instant::now();
    let error = connect_until(&endpoint, start + Duration::from_millis(200))
        .expect_err("stalled peer completed its handshake");
    assert!(
        matches!(
            error.kind(),
            io::ErrorKind::TimedOut | io::ErrorKind::ConnectionAborted
        ),
        "{error}"
    );
    // Allows runtime teardown and timer jitter while catching a fresh six-second budget.
    assert!(start.elapsed() < Duration::from_secs(3));
    Ok(())
}

#[test]
fn expired_budget_and_invalid_endpoint_have_distinct_error_kinds() {
    let expired =
        connect_until("tcp/127.0.0.1:1", Instant::now()).expect_err("expired bootstrap accepted");
    assert_eq!(expired.kind(), io::ErrorKind::TimedOut);
    let invalid = connect_until("tcp/192.0.2.1:1", Instant::now() + Duration::from_secs(6))
        .expect_err("non-loopback endpoint accepted");
    assert_eq!(invalid.kind(), io::ErrorKind::InvalidInput);
}
