// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::Server;
use crate::ServerOptions;
use gz_sim_core::EntityId;
use gz_sim_scene::{JointForceCommand, JointSample, Name, PhysicsEngine, load_world_with_sdf_step};
use nix::{
    errno::Errno,
    fcntl::{FcntlArg, OFlag, fcntl},
    sys::{
        signal::{Signal, killpg},
        wait::{Id, WaitPidFlag, WaitStatus, waitid},
    },
    unistd::Pid,
};
use sdformat::NoResources;
use std::{
    fs::File,
    io::{self, Read, Write},
    num::NonZeroUsize,
    os::{fd::AsFd, unix::process::CommandExt},
    path::Path,
    process::{Child, ChildStderr, ChildStdin, ChildStdout, Command, ExitStatus, Stdio},
    thread,
    time::{Duration, Instant},
};
use zenoh::Wait;
#[path = "../../../../tests/support/zenoh_peer.rs"]
mod zenoh_peer;

const XML: &str = include_str!("../fixtures/joint_force.sdf");
const PLUGIN: &str = "<plugin filename='gz-sim-apply-joint-force-system' name='gz::sim::systems::ApplyJointForce'><joint_name>hinge</joint_name></plugin>";

// Process-local configuration prevents concurrent tests from mutating global
// Zenoh settings. The kernel selects a port while the Rust session owns it;
// the native peer connects to that session's actual locator.
fn isolate(name: &str) -> bool {
    const CHILD: &str = "GZ_FORCE_ZENOH_CASE";
    if std::env::var(CHILD).as_deref() == Ok(name) {
        return false;
    }
    let directory = tempfile::tempdir().unwrap();
    let rust = directory.path().join("rust.json");
    let native = directory.path().join("native.json");
    let scouting = serde_json::json!({"multicast":{"enabled":false},"gossip":{"enabled":false}});
    std::fs::write(&rust, serde_json::to_vec(&serde_json::json!({"mode":"peer","listen":{"endpoints":["tcp/127.0.0.1:0"]},"scouting":scouting})).unwrap()).unwrap();
    std::fs::write(&native, serde_json::to_vec(&serde_json::json!({"mode":"peer","listen":{"endpoints":["tcp/127.0.0.1:0"]},"connect":{"endpoints":[],"timeout_ms":2000},"scouting":scouting,"transport":{"shared_memory":{"enabled":false}}})).unwrap()).unwrap();
    let mut child = Command::new(std::env::current_exe().unwrap());
    let module = module_path!().split_once("::").unwrap().1;
    child
        .args([
            "--exact",
            &format!("{module}::{name}"),
            "--nocapture",
            "--test-threads=1",
        ])
        .env(CHILD, name)
        .env("ZENOH_CONFIG", rust)
        .env("GZ_FORCE_ZENOH_NATIVE_CONFIG", native)
        .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
        .env_remove("LD_PRELOAD")
        .env_remove("LD_AUDIT");
    let mut process = OwnedProcess::spawn(&mut child, true).unwrap();
    let status = process.completed(Duration::from_secs(180)).unwrap();
    assert!(status.success(), "{}", process.diagnostics());
    assert!(String::from_utf8_lossy(&process.stdout_bytes).contains("1 passed"));
    true
}
fn force_owner(config: &ServerOptions) -> crate::world_io::WorldIo {
    let prefix = format!("@/native-force-owner-{}@", std::process::id());
    crate::world_io::WorldIo::new(
        format!("{prefix}/clock"),
        format!("{prefix}/control"),
        format!("{prefix}/poses"),
        config.capacity,
        config.max_message_bytes,
    )
    .unwrap()
}

const OUTPUT_CAP: usize = 4 * 1024 * 1024;
const LINE_CAP: usize = 65536;
const REAP_TIMEOUT: Duration = Duration::from_secs(3);

fn nonblocking(fd: impl AsFd) -> io::Result<()> {
    let flags = fcntl(&fd, FcntlArg::F_GETFL)
        .map_err(|error| io::Error::from_raw_os_error(error as i32))?;
    fcntl(
        &fd,
        FcntlArg::F_SETFL(OFlag::from_bits_retain(flags) | OFlag::O_NONBLOCK),
    )
    .map_err(|error| io::Error::from_raw_os_error(error as i32))?;
    Ok(())
}

fn drain(pipe: &mut impl Read, bytes: &mut Vec<u8>, file: &mut File) -> io::Result<bool> {
    let mut chunk = [0; 4096];
    loop {
        match pipe.read(&mut chunk) {
            Ok(0) => return Ok(true),
            Ok(count) => {
                if bytes.len() + count > OUTPUT_CAP {
                    return Err(io::Error::other("owned child output cap"));
                }
                file.write_all(&chunk[..count])?;
                bytes.extend_from_slice(&chunk[..count]);
            }
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => return Ok(false),
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => return Err(error),
        }
    }
}

// No blocking reader threads: inherited pipe writers cannot trap a reader join.
// The isolated Rust case owns a process group containing its native peer. The
// Source-bound C++ force peer launches no children and is killed directly.
struct OwnedProcess {
    child: Child,
    stdin: Option<ChildStdin>,
    stdout: Option<ChildStdout>,
    stderr: Option<ChildStderr>,
    stdout_file: File,
    stderr_file: File,
    stdout_bytes: Vec<u8>,
    stderr_bytes: Vec<u8>,
    stdout_eof: bool,
    stderr_eof: bool,
    status: Option<ExitStatus>,
    cursor: usize,
    group: bool,
    group_signalled: bool,
    exited_observed: bool,
    failed: bool,
    evidence: Option<tempfile::TempDir>,
}
impl OwnedProcess {
    fn spawn(command: &mut Command, group: bool) -> io::Result<Self> {
        let evidence = tempfile::tempdir()?;
        let stdout_file = File::create(evidence.path().join("stdout"))?;
        let stderr_file = File::create(evidence.path().join("stderr"))?;
        command
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        if group {
            command.process_group(0);
        }
        let mut child = command.spawn()?;
        let mut process = Self {
            stdin: child.stdin.take(),
            stdout: child.stdout.take(),
            stderr: child.stderr.take(),
            child,
            stdout_file,
            stderr_file,
            stdout_bytes: Vec::new(),
            stderr_bytes: Vec::new(),
            stdout_eof: false,
            stderr_eof: false,
            status: None,
            cursor: 0,
            group,
            group_signalled: false,
            exited_observed: false,
            failed: false,
            evidence: Some(evidence),
        };
        let configured = nonblocking(process.stdin.as_ref().unwrap())
            .and_then(|()| nonblocking(process.stdout.as_ref().unwrap()))
            .and_then(|()| nonblocking(process.stderr.as_ref().unwrap()));
        if let Err(error) = configured {
            process.failed = true;
            return Err(error);
        }
        Ok(process)
    }
    fn pump(&mut self) -> io::Result<()> {
        if let Some(stdout) = &mut self.stdout {
            self.stdout_eof |= drain(stdout, &mut self.stdout_bytes, &mut self.stdout_file)?;
        }
        if let Some(stderr) = &mut self.stderr {
            self.stderr_eof |= drain(stderr, &mut self.stderr_bytes, &mut self.stderr_file)?;
        }
        if self.status.is_none() {
            if self.group && !self.group_signalled {
                self.observe_group_exit()?;
            } else {
                self.status = self.child.try_wait()?;
            }
        }
        Ok(())
    }
    fn pid(&self) -> io::Result<Pid> {
        let raw = i32::try_from(self.child.id())
            .map_err(|_| io::Error::other("owned child PID range"))?;
        if raw <= 0 {
            return Err(io::Error::other("owned child PID is not positive"));
        }
        Ok(Pid::from_raw(raw))
    }
    fn observe_group_exit(&mut self) -> io::Result<()> {
        // WNOWAIT leaves even an exited leader unreaped, pinning the fresh
        // group ID until retire() has signalled its descendants.
        let pid = self.pid()?;
        let flags = WaitPidFlag::WEXITED | WaitPidFlag::WNOHANG | WaitPidFlag::WNOWAIT;
        match waitid(Id::Pid(pid), flags)
            .map_err(|error| io::Error::from_raw_os_error(error as i32))?
        {
            WaitStatus::StillAlive => {}
            WaitStatus::Exited(actual, _) | WaitStatus::Signaled(actual, _, _) if actual == pid => {
                self.exited_observed = true;
            }
            other => {
                return Err(io::Error::other(format!(
                    "unexpected owned child observation: {other:?}"
                )));
            }
        }
        Ok(())
    }
    fn diagnostics(&self) -> String {
        format!(
            "status={:?}, stdout={}\nstderr={}\nevidence={}",
            self.status,
            String::from_utf8_lossy(&self.stdout_bytes),
            String::from_utf8_lossy(&self.stderr_bytes),
            self.evidence.as_ref().unwrap().path().display()
        )
    }
    fn line(&mut self, timeout: Duration) -> Result<String, String> {
        let deadline = Instant::now() + timeout;
        loop {
            self.pump().map_err(|error| error.to_string())?;
            let tail = &self.stdout_bytes[self.cursor..];
            if let Some(end) = tail.iter().position(|byte| *byte == b'\n') {
                if end > LINE_CAP {
                    return Err("native protocol line cap".into());
                }
                let line = std::str::from_utf8(&tail[..end])
                    .map_err(|error| error.to_string())?
                    .to_owned();
                self.cursor += end + 1;
                return Ok(line);
            }
            if tail.len() > LINE_CAP || self.stdout_eof || Instant::now() >= deadline {
                return Err(format!(
                    "native protocol line missing: {}",
                    self.diagnostics()
                ));
            }
            thread::sleep(Duration::from_millis(2));
        }
    }
    fn send(&mut self, text: &str) -> Result<(), String> {
        let bytes = format!("{text}\n").into_bytes();
        if bytes.len() > LINE_CAP {
            return Err("native command cap".into());
        }
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut offset = 0;
        while offset < bytes.len() {
            self.pump().map_err(|error| error.to_string())?;
            if Instant::now() >= deadline {
                return Err("native command write deadline".into());
            }
            let stdin = self.stdin.as_mut().ok_or("native stdin closed")?;
            match stdin.write(&bytes[offset..]) {
                Ok(0) => return Err("native stdin stopped accepting bytes".into()),
                Ok(count) => offset += count,
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                    thread::sleep(Duration::from_millis(2))
                }
                Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                Err(error) => return Err(error.to_string()),
            }
        }
        Ok(())
    }
    fn completed(&mut self, timeout: Duration) -> Result<ExitStatus, String> {
        self.stdin.take();
        let deadline = Instant::now() + timeout;
        loop {
            if let Err(error) = self.pump() {
                self.failed = true;
                return Err(error.to_string());
            }
            if self.group && self.exited_observed && self.stdout_eof && self.stderr_eof {
                // Success still retires the owned group before the final reap.
                self.retire()?;
                return self
                    .status
                    .ok_or_else(|| "group leader was not actually reaped".into());
            }
            if let Some(status) = self.status
                && self.stdout_eof
                && self.stderr_eof
            {
                return Ok(status);
            }
            if Instant::now() >= deadline {
                self.failed = true;
                return Err(format!(
                    "owned child exit/pipe EOF deadline: {}",
                    self.diagnostics()
                ));
            }
            thread::sleep(Duration::from_millis(2));
        }
    }
    fn retire(&mut self) -> Result<(), String> {
        self.stdin.take();
        let mut errors = Vec::new();
        if self.group && !self.group_signalled {
            // Refuse signalling if waitid cannot confirm this still-waitable
            // owned leader; an ECHILD must not become an unpinned killpg.
            self.observe_group_exit()
                .map_err(|error| format!("owned leader pin unavailable: {error}"))?;
            // No group-owner path has reaped this Child: live or zombie, its
            // PID still pins the fresh group. Signal before any actual wait.
            let pid = self.pid().map_err(|error| error.to_string())?;
            if let Err(error) = killpg(pid, Signal::SIGKILL)
                && error != Errno::ESRCH
            {
                // Keep the unreaped identity on a failed signal; do not
                // turn a failed group cleanup into permission to reuse it.
                return Err(format!(
                    "owned group signal failed; leader not reaped: {error}"
                ));
            }
            self.group_signalled = true;
        }
        if self.status.is_none() {
            match self.child.try_wait() {
                Ok(Some(status)) => self.status = Some(status),
                Ok(None) if !self.group => {
                    if let Err(error) = self.child.kill() {
                        errors.push(error.to_string());
                    }
                }
                Ok(None) => {}
                Err(error) => errors.push(error.to_string()),
            }
        }
        let deadline = Instant::now() + REAP_TIMEOUT;
        while self.status.is_none() && Instant::now() < deadline {
            if let Err(error) = self.pump() {
                errors.push(error.to_string());
                // A stream/cap error must never prevent a subsequent actual reap.
                self.stdout.take();
                self.stderr.take();
            }
            match self.child.try_wait() {
                Ok(status) => self.status = status,
                Err(error) => {
                    errors.push(error.to_string());
                    break;
                }
            }
            thread::sleep(Duration::from_millis(2));
        }
        if self.status.is_none() {
            errors.push("UNKNOWN_NOT_REAPED within cleanup deadline".into());
        }
        if let Err(error) = self.pump() {
            errors.push(error.to_string());
        }
        self.stdout.take();
        self.stderr.take();
        for file in [&self.stdout_file, &self.stderr_file] {
            if let Err(error) = file.sync_all() {
                errors.push(error.to_string());
            }
        }
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors.join("; "))
        }
    }
}
impl Drop for OwnedProcess {
    fn drop(&mut self) {
        if let Err(error) = self.retire() {
            self.failed = true;
            let _ = writeln!(io::stderr().lock(), "owned force child cleanup: {error}");
        }
        if self.failed || thread::panicking() || self.status.is_some_and(|status| !status.success())
        {
            let _ = writeln!(io::stderr().lock(), "{}", self.diagnostics());
            if let Some(directory) = self.evidence.take() {
                let _ = writeln!(
                    io::stderr().lock(),
                    "retained native-force child outputs: {}",
                    directory.keep().display()
                );
            }
        }
    }
}

struct Peer {
    process: OwnedProcess,
}
impl Peer {
    fn start(
        native: &zenoh_peer::NativePeer,
        config: &ServerOptions,
        topic: &str,
        io: &crate::world_io::WorldIo,
    ) -> Self {
        let native_config = std::env::var_os("GZ_FORCE_ZENOH_NATIVE_CONFIG").unwrap();
        let mut settings: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&native_config).unwrap()).unwrap();
        let locators: Vec<_> = io
            .session
            .info()
            .locators()
            .wait()
            .into_iter()
            .map(|locator| locator.to_string())
            .collect();
        assert_eq!(locators.len(), 1, "expected the owned loopback listener");
        assert!(locators[0].starts_with("tcp/127.0.0.1:"));
        assert_ne!(locators[0], "tcp/127.0.0.1:0");
        settings["connect"]["endpoints"] = serde_json::json!(locators);
        std::fs::write(&native_config, serde_json::to_vec(&settings).unwrap()).unwrap();
        let mut command = Command::new(&native.binary);
        command
            .arg(topic)
            .env("GZ_PARTITION", &config.partition)
            .env("GZ_TRANSPORT_IMPLEMENTATION", "zenoh")
            .env("ZENOH_CONFIG", native_config)
            .env_remove("GZ_RELAY")
            .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
            .env_remove("LD_PRELOAD")
            .env_remove("LD_AUDIT")
            .env_remove("GZ_TRANSPORT_TOPIC_STATISTICS");
        native.configure_runtime(&mut command);
        Self {
            process: OwnedProcess::spawn(&mut command, false).unwrap(),
        }
    }
    fn line(&mut self, expected: &str) {
        assert_eq!(self.process.line(Duration::from_secs(8)).unwrap(), expected);
    }
    fn send(&mut self, text: &str) {
        self.process.send(text).unwrap();
        self.line("SENT");
    }
    fn finish(&mut self) {
        self.process.send("quit").unwrap();
        self.line("DONE");
        // Pinned Zenoh's close timeout is 10 seconds; DONE is not its exit.
        let status = self.process.completed(Duration::from_secs(15)).unwrap();
        assert!(status.success(), "{}", self.process.diagnostics());
        assert!(
            self.process.stderr_bytes.is_empty(),
            "{}",
            self.process.diagnostics()
        );
    }
}

#[test]
fn isolated_child_timeout_retains_partial_streams_and_reaps_owned_group() {
    let mut command = Command::new("/bin/sh");
    command.args([
        "-c",
        "printf 'partial-out\n'; printf 'partial-err\n' >&2; exec /bin/sleep 30",
    ]);
    let mut process = OwnedProcess::spawn(&mut command, true).unwrap();
    assert_eq!(process.line(Duration::from_secs(2)).unwrap(), "partial-out");
    assert!(process.completed(Duration::from_millis(50)).is_err());
    process.retire().unwrap();
    assert!(process.status.is_some());
    assert!(!process.status.unwrap().success());
    assert_eq!(process.stderr_bytes, b"partial-err\n");
    let directory = process.evidence.as_ref().unwrap().path();
    assert_eq!(
        std::fs::read(directory.join("stdout")).unwrap(),
        b"partial-out\n"
    );
    assert_eq!(
        std::fs::read(directory.join("stderr")).unwrap(),
        b"partial-err\n"
    );
}

#[test]
fn native_done_line_does_not_count_as_successful_process_retirement() {
    let mut command = Command::new("/bin/sh");
    command.args(["-c", "printf 'DONE\n'; exec /bin/sleep 30"]);
    let mut process = OwnedProcess::spawn(&mut command, true).unwrap();
    assert_eq!(process.line(Duration::from_secs(2)).unwrap(), "DONE");
    assert!(process.status.is_none());
    assert!(process.completed(Duration::from_millis(50)).is_err());
    process.retire().unwrap();
    assert!(process.status.is_some());
}

#[test]
fn exited_group_leader_stays_pinned_until_descendant_pipe_writer_is_retired() {
    let mut command = Command::new("/bin/sh");
    command.args([
        "-c",
        "/bin/sleep 30 & descendant=$!; printf 'DESCENDANT %s\n' \"$descendant\"; exit 0",
    ]);
    let mut process = OwnedProcess::spawn(&mut command, true).unwrap();
    let line = process.line(Duration::from_secs(2)).unwrap();
    let descendant: u32 = line.strip_prefix("DESCENDANT ").unwrap().parse().unwrap();
    let path = format!("/proc/{descendant}/stat");
    let initial = std::fs::read_to_string(&path).unwrap();
    let fields: Vec<_> = initial[initial.rfind(") ").unwrap() + 2..]
        .split_whitespace()
        .collect();
    assert_eq!(fields[2].parse::<u32>().unwrap(), process.child.id());
    let starttime = fields[19].to_owned();
    let deadline = Instant::now() + Duration::from_secs(2);
    while !process.exited_observed {
        process.pump().unwrap();
        assert!(Instant::now() < deadline, "group leader did not exit");
        thread::sleep(Duration::from_millis(2));
    }
    assert!(
        process.status.is_none(),
        "WNOWAIT must preserve the leader PID pin"
    );
    assert!(
        !process.stdout_eof,
        "the real descendant still owns the inherited pipe"
    );
    assert!(process.completed(Duration::from_millis(50)).is_err());
    process.retire().unwrap();
    assert!(process.group_signalled);
    assert!(
        process.status.unwrap().success(),
        "retain the leader's actual pre-kill exit zero"
    );
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        match std::fs::read_to_string(&path) {
            Ok(raw) => {
                let fields: Vec<_> = raw[raw.rfind(") ").unwrap() + 2..]
                    .split_whitespace()
                    .collect();
                if fields[19] != starttime || matches!(fields[0], "Z" | "X" | "x") {
                    break;
                }
            }
            Err(error) if error.kind() == io::ErrorKind::NotFound => break,
            Err(error) => panic!("descendant identity read: {error}"),
        }
        assert!(
            Instant::now() < deadline,
            "owned descendant remains live after group retirement"
        );
        thread::sleep(Duration::from_millis(2));
    }
    // A zombie is explicitly nonlive, not a descendant wait or an absent PID.
}

fn received(server: &mut Server, count: u64) {
    let deadline = Instant::now() + Duration::from_secs(3);
    while server.counters().force_messages_received < count {
        server.update().unwrap();
        assert!(Instant::now() < deadline, "force timeout");
        thread::sleep(Duration::from_millis(1));
    }
}
fn native_client() -> &'static zenoh_peer::NativePeer {
    static CLIENT: std::sync::OnceLock<zenoh_peer::NativePeer> = std::sync::OnceLock::new();
    CLIENT.get_or_init(|| {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        zenoh_peer::NativePeer::build(&root, "force")
    })
}

#[test]
fn real_cpp_publisher_controls_loaded_joint_force_on_all_dynamic_engines() {
    if isolate("real_cpp_publisher_controls_loaded_joint_force_on_all_dynamic_engines") {
        return;
    }
    let binary = native_client();
    for (index, engine) in [
        PhysicsEngine::Rapier,
        PhysicsEngine::Dart,
        PhysicsEngine::Mujoco,
    ]
    .into_iter()
    .enumerate()
    {
        let config = ServerOptions {
            partition: format!("force-cpp-{}-{index}", std::process::id()),
            ..ServerOptions::default()
        };
        let source = XML.replace("</model>", &format!("{PLUGIN}</model>"));
        let sim = load_world_with_sdf_step(&source, "force.sdf", "default", &NoResources, engine)
            .unwrap();
        let joint = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::rotor::hinge")
            })
            .unwrap()
            .id();
        let id = sim.world().get::<EntityId>(joint).unwrap().get();
        let mut server = Server::new(
            sim,
            "default",
            config.clone(),
            NonZeroUsize::new(2).unwrap(),
        )
        .unwrap();
        let mut peer = Peer::start(
            binary,
            &config,
            "/model/rotor/joint/hinge/cmd_force",
            &server.world_io,
        );
        peer.line("READY");
        for _ in 0..10 {
            server.update().unwrap();
        }
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            0.0
        );
        peer.send("0.001");
        received(&mut server, 1);
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            0.001
        );
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointSample>(joint)
                .unwrap()
                .state
                .velocity,
            0.0
        );
        server.set_paused(false);
        for _ in 0..10 {
            server.update().unwrap();
        }
        server.set_paused(true);
        let velocity = server
            .simulation()
            .world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .velocity;
        assert!((velocity - 0.000625).abs() < 1e-12, "{engine:?}:{velocity}");
        peer.send("nan");
        received(&mut server, 2);
        assert_eq!(server.counters().force_messages_rejected, 1);
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            0.001
        );
        peer.send("default");
        received(&mut server, 3);
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(joint)
                .unwrap()
                .torque(),
            0.0
        );
        assert_eq!(server.counters().joint_forces_applied, 2);
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<EntityId>(joint)
                .unwrap()
                .get(),
            id
        );
        peer.send("0.002");
        received(&mut server, 4);
        peer.finish();
        server.reset_scene().unwrap();
        let restored = gz_sim_core::entity_by_id(server.simulation().world(), id).unwrap();
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(restored)
                .unwrap()
                .torque(),
            0.0
        );
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointSample>(restored)
                .unwrap()
                .state
                .velocity,
            0.0
        );
        server.set_paused(false);
        for _ in 0..3 {
            server.update().unwrap();
        }
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointSample>(restored)
                .unwrap()
                .state
                .velocity,
            0.0
        );
        server.set_paused(true);
        let model = server
            .simulation()
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::rotor"))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        gz_sim_scene::remove_model(&mut server.simulation, model).unwrap();
        server.update().unwrap();
        assert!(gz_sim_core::entity_by_id(server.simulation().world(), id).is_none());
        server.reset_scene().unwrap();
        let restored = gz_sim_core::entity_by_id(server.simulation().world(), id).unwrap();
        assert_eq!(
            server
                .simulation()
                .world()
                .get::<JointForceCommand>(restored)
                .unwrap()
                .torque(),
            0.0
        );
        server.shutdown();
    }
}

#[test]
fn unread_native_force_queue_is_discarded_when_subscriptions_restart() {
    if isolate("unread_native_force_queue_is_discarded_when_subscriptions_restart") {
        return;
    }
    let config = ServerOptions {
        capacity: 1,
        partition: format!("force-unread-{}", std::process::id()),
        ..ServerOptions::default()
    };
    let source = XML.replace("</model>", &format!("{PLUGIN}</model>"));
    let mut sim = load_world_with_sdf_step(
        &source,
        "unread.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let definitions: Vec<_> = gz_sim_scene::initial_joint_force_configurations(&sim)
        .unwrap()
        .map(|(id, cfg)| (id, cfg.clone()))
        .collect();
    let io = force_owner(&config);
    let (mut forces, setup) =
        crate::forces::Forces::prepare(&config, definitions, &io.ingress_drops).unwrap();
    forces.bind(&sim).unwrap();
    io.install_forces(setup).unwrap();
    let mut peer = Peer::start(
        native_client(),
        &config,
        "/model/rotor/joint/hinge/cmd_force",
        &io,
    );
    peer.line("READY");
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let queue_drops = io.ingress_drops.load(std::sync::atomic::Ordering::Relaxed);
        if queue_drops >= 2 {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "native frames did not fill old bounded receiver: {queue_drops} drops"
        );
        // Native MS discovery precedes delivery readiness. Observable overflow
        // proves that actual wire samples reached the old bounded receiver.
        peer.send("1.0");
        thread::sleep(Duration::from_millis(10));
    }
    peer.finish();
    sim.reset().unwrap();
    forces.clear();
    let (replacement, setup) =
        crate::forces::Forces::restart(&config, &sim, &io.ingress_drops).unwrap();
    forces = replacement;
    io.install_forces(setup).unwrap();
    let mut counters = crate::Counters::default();
    forces
        .poll(&mut sim, NonZeroUsize::new(4).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(counters.force_messages_received, 0);
    sim.step_exact(2).unwrap();
    let joint = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<JointForceCommand>())
        .unwrap();
    assert_eq!(joint.get::<JointForceCommand>().unwrap().torque(), 0.0);
    assert_eq!(joint.get::<JointSample>().unwrap().state.velocity, 0.0);
    forces.shutdown();
}

// Copyright (C) 2019 Open Source Robotics Foundation
// Assertion port of ApplyJointForceTestFixture.JointVelocityCommand.
#[test]
fn original_ground_world_accepts_cpp_joint_force_with_original_sample_bounds() {
    if isolate("original_ground_world_accepts_cpp_joint_force_with_original_sample_bounds") {
        return;
    }
    let source = include_str!("../fixtures/apply_joint_force_original.sdf");
    let config = ServerOptions {
        partition: format!("original-force-cpp-{}", std::process::id()),
        ..ServerOptions::default()
    };
    let sim = load_world_with_sdf_step(
        source,
        "apply_joint_force.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Dart,
    )
    .unwrap();
    let joint = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|name| name.0 == "default::joint_force_test::j1")
        })
        .unwrap()
        .id();
    let mut server = Server::new(
        sim,
        "default",
        config.clone(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    // The tick-driven Rust server does not advance when constructed.
    assert_eq!(
        server
            .simulation()
            .world()
            .resource::<gz_sim_core::UpdateInfo>()
            .iteration,
        0
    );
    assert_eq!(server.pending_steps(), 0);
    server.set_paused(false);
    let mut samples = Vec::new();
    for _ in 0..10 {
        if let Some(command) = server.simulation().world().get::<JointForceCommand>(joint) {
            samples.push(command.torque());
        }
        server.update().unwrap();
        thread::sleep(Duration::from_millis(1));
    }
    assert_eq!(samples.len(), 10);
    for force in &samples {
        assert!(force.abs() <= 1e-4);
    }
    samples.clear();
    let mut peer = Peer::start(
        native_client(),
        &config,
        "/model/joint_force_test/joint/j1/cmd_force",
        &server.world_io,
    );
    peer.line("READY");
    peer.send("0.001");
    for _ in 0..1000 {
        if let Some(command) = server.simulation().world().get::<JointForceCommand>(joint) {
            samples.push(command.torque());
        }
        server.update().unwrap();
        thread::sleep(Duration::from_millis(1));
        if samples
            .last()
            .is_some_and(|force| (force - 0.001).abs() < 1e-6)
        {
            break;
        }
    }
    let actual = *samples.last().unwrap();
    assert!(actual.to_bits().abs_diff(0.001_f64.to_bits()) <= 4);
    peer.finish();
}
