// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_client::{Client, ClientError, ShutdownOutcome, TrustedWorker};
use std::{
    path::PathBuf,
    thread,
    time::{Duration, Instant},
};
fn main() {
    let path = PathBuf::from(std::env::args_os().nth(1).unwrap());
    let mut client = Client::start(TrustedWorker::from_operator_config(path).unwrap()).unwrap();
    let prefix = client.diagnostic_thread_prefix();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Err(ClientError::Io(error)) if error.kind() == std::io::ErrorKind::WouldBlock => break,
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("expected process or thread exhaustion: {other:?}"),
        }
    }
    assert_eq!(
        client.shutdown(Duration::from_secs(3)).unwrap(),
        ShutdownOutcome::Exited(None)
    );
    assert_eq!(client.worker_pid(), None);
    let remaining = std::fs::read_dir("/proc/self/task")
        .unwrap()
        .filter_map(Result::ok)
        .filter_map(|entry| std::fs::read_to_string(entry.path().join("comm")).ok())
        .filter(|name| name.starts_with(&prefix))
        .count();
    assert_eq!(remaining, 0);
}
