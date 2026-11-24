// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::Value;
use std::time::Instant;

fn until<T>(mut sample: impl FnMut() -> Option<T>) -> T {
    let start = Instant::now();
    loop {
        if let Some(value) = sample() {
            return value;
        }
        assert!(start.elapsed() < Duration::from_secs(5), "relay timed out");
        thread::sleep(INTERVAL);
    }
}

fn state(dir: &Path, predicate: impl Fn(&Value) -> bool) -> Value {
    until(|| {
        let value = serde_json::from_slice(&fs::read(dir.join("state.json")).ok()?).unwrap();
        predicate(&value).then_some(value)
    })
}

fn set_mode(dir: &Path, label: &str) {
    fs::write(dir.join("mode.next"), label).unwrap();
    fs::rename(dir.join("mode.next"), dir.join("mode")).unwrap();
    state(dir, |v| v["mode"] == label);
}

fn endpoints(addr: SocketAddr, listener: &TcpListener) -> (TcpStream, TcpStream) {
    let client = TcpStream::connect(addr).unwrap();
    let server = until(|| listener.accept().ok()).0;
    for socket in [&client, &server] {
        socket
            .set_read_timeout(Some(Duration::from_millis(300)))
            .unwrap();
    }
    (client, server)
}

fn silence(socket: &mut TcpStream) {
    let err = socket.read(&mut [0]).unwrap_err();
    assert!(matches!(
        err.kind(),
        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
    ));
}

fn expect_byte(socket: &mut TcpStream, byte: u8) {
    let mut got = [0];
    socket.read_exact(&mut got).unwrap();
    assert_eq!(got, [byte]);
}

#[test]
fn directional_cuts_suppress_bytes_and_fin_until_restoration() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path().to_owned();
    let upstream = TcpListener::bind("127.0.0.1:0").unwrap();
    upstream.set_nonblocking(true).unwrap();
    let args = Args {
        listen: "127.0.0.1:0".parse().unwrap(),
        upstream: upstream.local_addr().unwrap(),
        control_dir: dir.clone(),
    };
    let relay = thread::spawn(|| run(args));
    let ready: Value =
        until(|| serde_json::from_slice(&fs::read(dir.join("ready.json")).ok()?).ok());
    let addr = ready["listen"].as_str().unwrap().parse().unwrap();
    let (mut client, mut server) = endpoints(addr, &upstream);
    client.write_all(b"a").unwrap();
    expect_byte(&mut server, b'a');

    set_mode(&dir, "drop-up");
    client.write_all(b"x").unwrap();
    client.shutdown(Shutdown::Write).unwrap();
    state(&dir, |v| v["up"]["source_ends"] == 1);
    silence(&mut server);
    server.write_all(b"b").unwrap();
    expect_byte(&mut client, b'b');

    set_mode(&dir, "drop-down");
    assert_eq!(server.read(&mut [0]).unwrap(), 0);
    server.write_all(b"y").unwrap();
    server.shutdown(Shutdown::Write).unwrap();
    state(&dir, |v| v["down"]["source_ends"] == 1);
    silence(&mut client);

    // Connections created during a cut obey it from their first byte.
    set_mode(&dir, "drop-both");
    let (mut second, mut peer) = endpoints(addr, &upstream);
    second.write_all(b"z").unwrap();
    peer.write_all(b"q").unwrap();
    state(&dir, |v| {
        v["up"]["discarded_bytes"] == 2 && v["down"]["discarded_bytes"] == 2
    });
    silence(&mut second);
    silence(&mut peer);
    drop(second);
    state(&dir, |v| v["up"]["source_ends"] == 2);
    silence(&mut peer);

    set_mode(&dir, "pass");
    assert_eq!(client.read(&mut [0]).unwrap(), 0);
    assert_eq!(peer.read(&mut [0]).unwrap(), 0);
    drop(peer);
    let final_state = state(&dir, |v| v["closed"] == 2);
    for direction in ["up", "down"] {
        assert_eq!(final_state[direction]["forwarded_bytes"], 1);
        assert_eq!(final_state[direction]["discarded_bytes"], 2);
        assert_eq!(final_state[direction]["forwarded_ends"], 2);
    }

    let (mut fresh, mut healthy) = endpoints(addr, &upstream);
    fresh.write_all(b"r").unwrap();
    expect_byte(&mut healthy, b'r');
    healthy.write_all(b"s").unwrap();
    expect_byte(&mut fresh, b's');
    fs::write(dir.join("stop"), []).unwrap();
    assert!(relay.join().unwrap().is_ok());
    let last = state(&dir, |v| v["stopped"] == true);
    assert_eq!(last["accepted"], 3);
    assert_eq!(last["active"], 0);
    assert_eq!(last["connect_errors"], 0);
}
