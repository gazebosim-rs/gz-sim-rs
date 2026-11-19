// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! M72-only adoption of the existing bounded canonical consumer custody.
//! Original M69 wrapper bodies/caps/deadlines remain untouched.
use super::{proc_identity, selected_native};
use std::{
    io::{BufRead, BufReader, Read},
    process::{Child, Command, Stdio},
    sync::{Arc, Mutex, mpsc},
    thread,
    time::{Duration, Instant},
};
#[derive(Default)]
struct Captured {
    bytes: Vec<u8>,
    error: Option<String>,
}
pub struct Native {
    child: Child,
    start: Option<u64>,
    owner: usize,
    lines: mpsc::Receiver<String>,
    stdout: Arc<Mutex<Captured>>,
    stderr: Arc<Mutex<Captured>>,
    readers: Vec<thread::JoinHandle<()>>,
    capture: std::path::PathBuf,
    waited: Option<std::process::ExitStatus>,
    saved: bool,
    _config: tempfile::TempDir,
}
impl Native {
    pub fn pid(&self) -> u32 {
        self.child.id()
    }
    pub fn actual_exit(&self) -> Option<i32> {
        self.waited
            .as_ref()
            .and_then(std::process::ExitStatus::code)
    }
    pub fn start(
        endpoint: &str,
        partition: &str,
        peer: &selected_native::NativePeer,
        owner: usize,
    ) -> Self {
        assert!(owner < 2);
        let config = tempfile::tempdir().unwrap();
        let path = config.path().join("client.json");
        std::fs::write(&path, serde_json::json!({"mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":[endpoint],"timeout_ms":2000},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"transport":{"shared_memory":{"enabled":false}}}).to_string()).unwrap();
        let mut command = Command::new(&peer.binary);
        peer.configure_runtime(&mut command);
        command
            .args(["explicit-intrinsics", &owner.to_string()])
            .env("GZ_IP", "127.0.0.1")
            .env("GZ_PARTITION", partition)
            .env("ZENOH_CONFIG", path)
            .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
            .env_remove("GZ_TRANSPORT_USERNAME")
            .env_remove("GZ_TRANSPORT_PASSWORD")
            .env_remove("GZ_RELAY")
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        let mut child = command.spawn().unwrap();
        let (tx, lines) = mpsc::sync_channel(8);
        let stdout = Arc::new(Mutex::new(Captured::default()));
        let stderr = Arc::new(Mutex::new(Captured::default()));
        let out = child.stdout.take().unwrap();
        let err = child.stderr.take().unwrap();
        let saved_out = Arc::clone(&stdout);
        let saved_err = Arc::clone(&stderr);
        let readers = vec![
            thread::spawn(move || {
                let mut reader = BufReader::new(out).take(65537);
                loop {
                    let mut line = Vec::new();
                    match reader.read_until(b'\n', &mut line) {
                        Ok(0) => break,
                        Ok(_) => {
                            let mut saved = saved_out.lock().unwrap();
                            if saved.bytes.len() + line.len() > 65536 {
                                // Preserve the bounded original prefix on failure.
                                let remaining = 65536 - saved.bytes.len();
                                saved
                                    .bytes
                                    .extend_from_slice(&line[..remaining.min(line.len())]);
                                saved.error =
                                    Some("canonical consumer stdout exceeded 64 KiB".into());
                                break;
                            }
                            saved.bytes.extend_from_slice(&line);
                            match String::from_utf8(line) {
                                Ok(line) => {
                                    if tx.try_send(line.trim_end_matches('\n').to_owned()).is_err()
                                    {
                                        saved.error =
                                            Some("bounded canonical phase queue exceeded".into());
                                        break;
                                    }
                                }
                                Err(error) => {
                                    saved.error = Some(error.to_string());
                                    break;
                                }
                            }
                        }
                        Err(error) => {
                            saved_out.lock().unwrap().error = Some(error.to_string());
                            break;
                        }
                    }
                }
            }),
            thread::spawn(move || {
                let mut bytes = Vec::new();
                let result = err.take(4097).read_to_end(&mut bytes);
                let mut saved = saved_err.lock().unwrap();
                if bytes.len() > 4096 {
                    bytes.truncate(4096);
                    saved.error = Some("canonical consumer stderr exceeded 4 KiB".into());
                } else if let Err(error) = result {
                    saved.error = Some(error.to_string());
                }
                saved.bytes = bytes;
            }),
        ];
        let mut owned = Self {
            child,
            start: None,
            owner,
            lines,
            stdout,
            stderr,
            readers,
            capture: peer.binary.parent().unwrap().to_owned(),
            waited: None,
            saved: false,
            _config: config,
        };
        let (start, state, parent) = proc_identity(owned.child.id()).unwrap();
        owned.start = Some(start);
        assert!(!matches!(state, 'Z' | 'X'));
        assert_eq!(parent, std::process::id());
        owned
    }
    pub fn line(&self) -> String {
        for saved in [&self.stdout, &self.stderr] {
            assert!(saved.lock().unwrap().error.is_none());
        }
        self.lines
            .recv_timeout(Duration::from_secs(20))
            .expect("bounded actual canonical explicit-K camera phase")
    }
    fn persist(&mut self, completed: bool) {
        if self.saved {
            return;
        }
        let out = self.stdout.lock().unwrap();
        let err = self.stderr.lock().unwrap();
        let save = |suffix: &str, bytes: &[u8]| {
            let path = self
                .capture
                .join(format!("m72-intrinsics-{}.{suffix}", self.owner));
            let mut file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(path)
                .unwrap();
            std::io::Write::write_all(&mut file, bytes).unwrap();
            file.sync_all().unwrap();
        };
        save("stdout", &out.bytes);
        save("stderr", &err.bytes);
        let record = serde_json::json!({"owner":self.owner,"pid":self.child.id(),"start_ticks":self.start,
            "actual_exit_code":self.waited.as_ref().and_then(std::process::ExitStatus::code),
            "actual_wait_reaped":self.waited.is_some(),"completed_canonical_checks":completed,
            "stdout_bytes":out.bytes.len(),"stderr_bytes":err.bytes.len(),
            "stdout_error":out.error,"stderr_error":err.error,"worker_exit_code":"UNKNOWN",
            "scope":"One actual owned upstream explicit-K camera consumer, full canonical Image/Info. Software pixels, not native rendering or Manager parity."});
        save("receipt.json", &serde_json::to_vec_pretty(&record).unwrap());
        self.saved = true;
    }
    pub fn finish(&mut self) -> usize {
        let until = Instant::now() + Duration::from_secs(5);
        loop {
            if let Some(status) = self.child.try_wait().unwrap() {
                self.waited = Some(status);
                break;
            }
            assert!(
                Instant::now() < until,
                "actual canonical consumer wait did not finish"
            );
            thread::yield_now();
        }
        while self.readers.iter().any(|r| !r.is_finished()) {
            assert!(
                Instant::now() < until,
                "actual consumer streams did not drain"
            );
            thread::yield_now();
        }
        for reader in self.readers.drain(..) {
            reader.join().unwrap();
        }
        let valid = {
            let out = self.stdout.lock().unwrap();
            let err = self.stderr.lock().unwrap();
            self.waited.as_ref().unwrap().success()
                && out.error.is_none()
                && err.error.is_none()
                && err.bytes.is_empty()
        };
        self.persist(valid);
        assert!(
            valid,
            "actual CPP owner{} wait={:?}; original streams retained",
            self.owner, self.waited
        );
        assert!(
            self.lines.try_recv().is_err(),
            "no unexamined canonical stdout phases"
        );
        self.stdout.lock().unwrap().bytes.len()
    }
}
impl Drop for Native {
    fn drop(&mut self) {
        if self.saved {
            return;
        }
        if matches!(self.child.try_wait(), Ok(None)) {
            let _ = self.child.kill();
        }
        let until = Instant::now() + Duration::from_secs(5);
        loop {
            match self.child.try_wait() {
                Ok(Some(status)) => {
                    self.waited = Some(status);
                    break;
                }
                Ok(None) if Instant::now() < until => thread::yield_now(),
                other => {
                    eprintln!("owned canonical wait UNKNOWN: {other:?}");
                    break;
                }
            }
        }
        while self.readers.iter().any(|r| !r.is_finished()) && Instant::now() < until {
            thread::yield_now();
        }
        for reader in self.readers.drain(..) {
            if reader.is_finished() {
                let _ = reader.join();
            }
        }
        // Panic already marks failure. Preserve partial channels and actual wait
        // without manufacturing completed packet checks or child exit0.
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.persist(false)));
    }
}
