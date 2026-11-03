// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::{
    fs,
    path::Path,
    process::{Child, Command, Output, Stdio},
    thread,
    time::{Duration, Instant},
};

#[cfg(feature = "rgb-camera")]
#[path = "../../gz-camera-output/tests/support/isolated_camera.rs"]
mod isolation;

const EMPTY: &str = "<sdf version='1.12'><world name='default'/></sdf>";
#[cfg(feature = "rgb-camera")]
const CAMERA: &str = include_str!("../../gz-camera-output/tests/fixtures/m5-rgb.sdf");
#[cfg(feature = "rgb-camera")]
const SENSORS: &str =
    "<plugin name=\"gz::sim::systems::Sensors\" filename=\"gz-sim-sensors-system\"/>";

struct Process(Option<Child>);

impl Drop for Process {
    fn drop(&mut self) {
        if let Some(mut child) = self.0.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

fn command(xml: &str, worker: Option<&Path>, args: &[&str]) -> Output {
    command_with_listener(xml, worker, args, "tcp/127.0.0.1:0")
}

fn command_with_listener(
    xml: &str,
    worker: Option<&Path>,
    args: &[&str],
    listener: &str,
) -> Output {
    command_network(xml, worker, args, Some(listener))
}

fn command_network(
    xml: &str,
    worker: Option<&Path>,
    args: &[&str],
    listener: Option<&str>,
) -> Output {
    let directory = tempfile::tempdir().unwrap();
    let source = directory.path().join("world.sdf");
    let network = directory.path().join("zenoh.json");
    let stdout = directory.path().join("stdout");
    let stderr = directory.path().join("stderr");
    fs::write(&source, xml).unwrap();
    if let Some(listener) = listener {
        fs::write(
            &network,
            serde_json::to_vec(&serde_json::json!({
                "mode":"peer", "listen":{"endpoints":[listener]},
                "connect":{"endpoints":[]},
                "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}
            }))
            .unwrap(),
        )
        .unwrap();
    }
    let mut command = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
    command
        .arg(source)
        .args(args)
        .env(
            "GZ_PARTITION",
            format!(
                "rgb-cli-{}",
                directory.path().file_name().unwrap().to_str().unwrap()
            ),
        )
        .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
        .env_remove("GZ_RELAY")
        .env_remove("SDF_PATH")
        .env_remove("GZ_SIM_RESOURCE_PATH")
        .stdin(Stdio::null())
        .stdout(fs::File::create(&stdout).unwrap())
        .stderr(fs::File::create(&stderr).unwrap());
    if listener.is_some() {
        command.env("ZENOH_CONFIG", network);
    } else {
        command.env_remove("ZENOH_CONFIG");
    }
    if let Some(worker) = worker {
        command.arg("--camera-worker").arg(worker);
    }
    let mut process = Process(Some(command.spawn().unwrap()));
    let deadline = Instant::now() + Duration::from_secs(10);
    let status = loop {
        if let Some(status) = process.0.as_mut().unwrap().try_wait().unwrap() {
            break status;
        }
        assert!(
            Instant::now() < deadline,
            "CLI timed out; stdout={} stderr={}",
            fs::read_to_string(&stdout).unwrap(),
            fs::read_to_string(&stderr).unwrap()
        );
        thread::sleep(Duration::from_millis(10));
    };
    process.0.take().unwrap().wait().unwrap();
    Output {
        status,
        stdout: fs::read(stdout).unwrap(),
        stderr: fs::read(stderr).unwrap(),
    }
}

#[test]
fn camera_worker_flag_requires_server_mode() {
    let directory = tempfile::tempdir().unwrap();
    let worker = directory.path().join("unselected-worker");
    let result = command(EMPTY, Some(&worker), &["--iterations", "0"]);
    assert_eq!(result.status.code(), Some(2), "{result:?}");
    assert!(result.stdout.is_empty());
    assert!(String::from_utf8_lossy(&result.stderr).contains("--server"));
    assert!(!worker.exists());
}

#[cfg(feature = "rgb-camera")]
#[test]
fn no_configured_camera_never_starts_the_operator_worker() {
    if isolation::child("no_configured_camera_never_starts_the_operator_worker", 45) {
        return;
    }
    let directory = tempfile::tempdir().unwrap();
    let worker = directory.path().join("absent-worker");
    assert!(!worker.exists());
    let sensors_only = format!("<sdf version='1.12'><world name='default'>{SENSORS}</world></sdf>");
    assert!(CAMERA.contains(SENSORS));
    let metadata_only = CAMERA.replacen(SENSORS, "", 1);
    for xml in [EMPTY, sensors_only.as_str(), metadata_only.as_str()] {
        let without = command(
            xml,
            None,
            &[
                "--physics-engine",
                "rapier",
                "--server",
                "--run",
                "--iterations",
                "3",
            ],
        );
        let with = command(
            xml,
            Some(&worker),
            &[
                "--physics-engine",
                "rapier",
                "--server",
                "--run",
                "--iterations",
                "3",
            ],
        );
        assert!(without.status.success(), "{without:?}");
        assert!(with.status.success(), "{with:?}");
        assert_eq!(with.stdout, without.stdout);
        assert!(
            String::from_utf8_lossy(&with.stdout)
                .starts_with("simulation_ns=3000000 iterations=3\n")
        );
        assert!(with.stderr.is_empty(), "{with:?}");
        assert!(!worker.exists());
    }
    // No loaded camera means the operator path is not validated or consumed.
    let relative = command(
        EMPTY,
        Some(Path::new("unselected-relative-worker")),
        &["--server", "--run", "--iterations", "3"],
    );
    assert!(relative.status.success(), "{relative:?}");
    assert_eq!(relative.stdout, b"simulation_ns=3000000 iterations=3\n");
    assert!(relative.stderr.is_empty(), "{relative:?}");
}

#[cfg(feature = "rgb-camera")]
#[test]
fn admitted_rgb_camera_without_image_interest_keeps_its_worker_lazy() {
    if isolation::child(
        "admitted_rgb_camera_without_image_interest_keeps_its_worker_lazy",
        20,
    ) {
        return;
    }
    let directory = tempfile::tempdir().unwrap();
    let worker = directory.path().join("absent-worker");
    assert!(!worker.exists());
    let result = command(
        CAMERA,
        Some(&worker),
        &[
            "--physics-engine",
            "rapier",
            "--server",
            "--run",
            "--iterations",
            "3",
        ],
    );
    assert!(result.status.success(), "{result:?}");
    assert_eq!(
        String::from_utf8_lossy(&result.stdout),
        "simulation_ns=3000000 iterations=3\ndefault::rig::body 0 0 0\ndefault::target::body 5 0 0\n"
    );
    assert!(result.stderr.is_empty(), "{result:?}");
    assert!(!worker.exists());
}

#[cfg(feature = "rgb-camera")]
#[test]
fn camera_worker_and_calibration_errors_fail_before_a_successful_run() {
    if isolation::child(
        "camera_worker_and_calibration_errors_fail_before_a_successful_run",
        45,
    ) {
        return;
    }
    let no_owner = command(
        CAMERA,
        None,
        &[
            "--physics-engine",
            "rapier",
            "--server",
            "--run",
            "--iterations",
            "1",
        ],
    );
    assert_eq!(no_owner.status.code(), Some(1), "{no_owner:?}");
    assert!(no_owner.stdout.is_empty());
    assert!(
        String::from_utf8_lossy(&no_owner.stderr).contains("camera application owner"),
        "{no_owner:?}"
    );
    let relative = command(
        CAMERA,
        Some(Path::new("relative-worker")),
        &[
            "--physics-engine",
            "rapier",
            "--server",
            "--run",
            "--iterations",
            "1",
        ],
    );
    assert_eq!(relative.status.code(), Some(1), "{relative:?}");
    assert!(relative.stdout.is_empty());
    assert!(
        String::from_utf8_lossy(&relative.stderr).contains("InvalidWorkerPath"),
        "{relative:?}"
    );
    let directory = tempfile::tempdir().unwrap();
    let worker = directory.path().join("absent-worker");
    assert!(CAMERA.contains("<width>64</width>"));
    let invalid = CAMERA.replacen("<width>64</width>", "<width>32</width>", 1);
    let rejected = command(
        &invalid,
        Some(&worker),
        &[
            "--physics-engine",
            "rapier",
            "--server",
            "--run",
            "--iterations",
            "1",
        ],
    );
    assert_eq!(rejected.status.code(), Some(1), "{rejected:?}");
    assert!(rejected.stdout.is_empty());
    assert_eq!(
        rejected.stderr,
        b"default::rig::body::camera: unsupported non-IMU sensors\n"
    );
    // Dimensions are checked by Scene admission. Keep the same admitted 64x64
    // metadata for a separate render-calibration refusal: this near clip is
    // finite, normal and positive in f64, but narrows to zero in f32.
    assert!(CAMERA.contains("<near>0.1</near>"));
    let unrepresentable = CAMERA.replacen("<near>0.1</near>", "<near>1e-50</near>", 1);
    let calibration = command(
        &unrepresentable,
        Some(&worker),
        &[
            "--physics-engine",
            "rapier",
            "--server",
            "--run",
            "--iterations",
            "1",
        ],
    );
    assert_eq!(calibration.status.code(), Some(1), "{calibration:?}");
    assert!(calibration.stdout.is_empty());
    let diagnostic = String::from_utf8_lossy(&calibration.stderr);
    let camera_id = diagnostic
        .strip_prefix("camera snapshot: camera sensor ")
        .and_then(|text| text.strip_suffix(" has unsupported or invalid calibration\n"))
        .unwrap_or_else(|| panic!("unexpected calibration refusal: {calibration:?}"));
    assert!(camera_id.parse::<u64>().is_ok(), "{diagnostic}");
    assert!(!worker.exists());
}

#[cfg(feature = "rgb-camera")]
#[test]
fn camera_and_server_use_distinct_listeners_and_release_the_fixed_server_address() {
    if isolation::child(
        "camera_and_server_use_distinct_listeners_and_release_the_fixed_server_address",
        20,
    ) {
        return;
    }
    let directory = tempfile::tempdir().unwrap();
    let worker = directory.path().join("absent-worker");
    let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let address = reservation.local_addr().unwrap();
    let listener = format!("tcp/{address}");
    // The child must really bind this address; there is no retry or alternate port.
    // The isolated test namespace has no unrelated listener contenders.
    drop(reservation);
    let result = command_with_listener(
        CAMERA,
        Some(&worker),
        &[
            "--physics-engine",
            "rapier",
            "--server",
            "--run",
            "--iterations",
            "3",
        ],
        &listener,
    );
    assert!(result.status.success(), "{result:?}");
    assert!(
        String::from_utf8_lossy(&result.stdout).starts_with("simulation_ns=3000000 iterations=3\n")
    );
    assert!(result.stderr.is_empty(), "{result:?}");
    assert!(!worker.exists());
    let _released = std::net::TcpListener::bind(address).unwrap();
}

#[cfg(feature = "rgb-camera")]
#[test]
fn camera_owner_shuts_down_when_viewer_setup_fails_and_its_listener_can_be_reused() {
    if isolation::child(
        "camera_owner_shuts_down_when_viewer_setup_fails_and_its_listener_can_be_reused",
        30,
    ) {
        return;
    }
    let directory = tempfile::tempdir().unwrap();
    let worker = directory.path().join("absent-worker");
    let reserved = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let address = reserved.local_addr().unwrap();
    let port = address.port().to_string();
    let args = [
        "--physics-engine",
        "rapier",
        "--server",
        "--run",
        "--iterations",
        "3",
        "--web-viewer",
        port.as_str(),
    ];
    let failed = command(CAMERA, Some(&worker), &args);
    assert_eq!(failed.status.code(), Some(1), "{failed:?}");
    assert!(failed.stdout.is_empty());
    assert!(
        String::from_utf8_lossy(&failed.stderr).contains("in use"),
        "{failed:?}"
    );
    assert!(!String::from_utf8_lossy(&failed.stderr).contains("gzweb-rs WebSocket:"));
    assert!(!worker.exists());
    drop(reserved);
    let succeeded = command(CAMERA, Some(&worker), &args);
    assert!(succeeded.status.success(), "{succeeded:?}");
    assert!(
        String::from_utf8_lossy(&succeeded.stdout)
            .starts_with("simulation_ns=3000000 iterations=3\n")
    );
    assert!(
        String::from_utf8_lossy(&succeeded.stderr)
            .contains(&format!("gzweb-rs WebSocket: ws://127.0.0.1:{port}/"))
    );
    let _released = std::net::TcpListener::bind(address).unwrap();
    assert!(!worker.exists());
}

#[cfg(feature = "rgb-camera")]
#[test]
fn configured_camera_without_zenoh_config_uses_the_default_network_lazily() {
    if isolation::child(
        "configured_camera_without_zenoh_config_uses_the_default_network_lazily",
        20,
    ) {
        return;
    }
    let directory = tempfile::tempdir().unwrap();
    let worker = directory.path().join("absent-worker");
    let result = command_network(
        CAMERA,
        Some(&worker),
        &[
            "--physics-engine",
            "rapier",
            "--server",
            "--run",
            "--iterations",
            "3",
        ],
        None,
    );
    assert!(result.status.success(), "{result:?}");
    assert_eq!(
        String::from_utf8_lossy(&result.stdout),
        "simulation_ns=3000000 iterations=3\ndefault::rig::body 0 0 0\ndefault::target::body 5 0 0\n"
    );
    assert!(result.stderr.is_empty(), "{result:?}");
    assert!(!worker.exists());
}

#[cfg(feature = "rgb-camera")]
struct CameraProcess {
    child: Child,
    stdout: std::path::PathBuf,
    stderr: std::path::PathBuf,
    complete: bool,
}

#[cfg(feature = "rgb-camera")]
impl Drop for CameraProcess {
    fn drop(&mut self) {
        if !self.complete {
            // The CLI and its operator fixture belong to this test's group.
            // This also cleans an orphan fixture if an exit/cleanup assertion fails.
            let group =
                rustix::process::Pid::from_raw(i32::try_from(self.child.id()).unwrap()).unwrap();
            let _ = rustix::process::kill_process_group(group, rustix::process::Signal::KILL);
        }
        let _ = self.child.wait();
        if thread::panicking() {
            eprintln!(
                "CLI stdout: {}",
                fs::read_to_string(&self.stdout).unwrap_or_default()
            );
            eprintln!(
                "CLI stderr: {}",
                fs::read_to_string(&self.stderr).unwrap_or_default()
            );
        }
    }
}

#[cfg(feature = "rgb-camera")]
#[test]
fn actual_image_interest_bounds_silent_worker_backpressure_and_propagates_spawn_failure() {
    use gz_sim_proto::{
        msgs::{Boolean, CameraInfo, Clock, WorldControl},
        prost::Message,
    };
    use std::{
        os::unix::{fs::PermissionsExt, process::CommandExt},
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
            mpsc,
        },
    };
    use zenoh::Wait;

    if isolation::child(
        "actual_image_interest_bounds_silent_worker_backpressure_and_propagates_spawn_failure",
        75,
    ) {
        return;
    }
    for (case, silent) in [true, false].into_iter().enumerate() {
        let directory = tempfile::tempdir().unwrap();
        // Bind the real observer first. Distinct fixed endpoints live only in
        // this test's private netns; no reservation is released or guessed.
        let endpoint = format!("tcp/127.0.0.2:{}", 39171 + case);
        let observer = zenoh::open(
            zenoh::Config::from_json5(
                &serde_json::json!({
                    "mode":"router", "listen":{"endpoints":[endpoint]}, "connect":{"endpoints":[]},
                    "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}
                })
                .to_string(),
            )
            .unwrap(),
        )
        .wait()
        .unwrap();
        let partition = format!(
            "rgb-failure-{}",
            directory.path().file_name().unwrap().to_str().unwrap()
        );
        let key = |topic: &str| format!("@/{partition}@{topic}");
        let images_seen = Arc::new(AtomicUsize::new(0));
        let seen = Arc::clone(&images_seen);
        let images = observer
            .declare_subscriber(key("/m5/rgb/image"))
            .callback(move |_| {
                seen.fetch_add(1, Ordering::Relaxed);
            })
            .wait()
            .unwrap();
        let infos = observer
            .declare_subscriber(key("/m5/rgb/camera_info"))
            .with(zenoh::handlers::FifoChannel::new(8))
            .wait()
            .unwrap();
        let (send_clock, receive_clock) = mpsc::sync_channel(1);
        // Only a startup-clock fence is needed. Later clocks never block the
        // observer's dispatch thread or claim a loss-free clock stream.
        let clocks = observer
            .declare_subscriber(key("/world/default/clock"))
            .callback(move |sample| {
                let _ = send_clock.try_send(sample);
            })
            .wait()
            .unwrap();
        let source = directory.path().join("world.sdf");
        let network = directory.path().join("zenoh.json");
        let worker = directory.path().join("worker");
        let pid_file = directory.path().join("worker.pid");
        fs::write(&source, CAMERA).unwrap();
        fs::write(
            &network,
            serde_json::to_vec(&serde_json::json!({
                "mode":"peer", "listen":{"endpoints":["tcp/127.0.0.1:0"]},
                "connect":{"endpoints":[endpoint]},
                "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}
            }))
            .unwrap(),
        )
        .unwrap();
        if silent {
            // This real operator executable emits no Ready, frame or protocol
            // result. exec keeps one CPU-light child for actual teardown checks.
            fs::write(
                &worker,
                "#!/bin/sh\nprintf '%s\\n' \"$$\" > \"$GZ_RGB_SILENT_PID\"\nexec /bin/sleep 60\n",
            )
            .unwrap();
            fs::set_permissions(&worker, fs::Permissions::from_mode(0o700)).unwrap();
        } else {
            assert!(!worker.exists());
        }
        let stdout = directory.path().join("stdout");
        let stderr = directory.path().join("stderr");
        // A live per-CLI watchdog still owns the subgroup if the outer namespace
        // watchdog kills this test before its Rust cleanup guard can run.
        let child = Command::new("timeout")
            .args(["--signal=KILL", "25s"])
            .arg(env!("CARGO_BIN_EXE_gz-sim-rs"))
            .arg(&source)
            .args([
                "--physics-engine",
                "rapier",
                "--server",
                "--iterations",
                "10000",
                "--camera-worker",
            ])
            .arg(&worker)
            .env("ZENOH_CONFIG", &network)
            .env("GZ_PARTITION", &partition)
            .env("GZ_RGB_SILENT_PID", &pid_file)
            .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
            .env_remove("GZ_RELAY")
            .env_remove("SDF_PATH")
            .env_remove("GZ_SIM_RESOURCE_PATH")
            .stdin(Stdio::null())
            .stdout(fs::File::create(&stdout).unwrap())
            .stderr(fs::File::create(&stderr).unwrap())
            .process_group(0)
            .spawn()
            .unwrap();
        let mut process = CameraProcess {
            child,
            stdout,
            stderr,
            complete: false,
        };
        let first_clock = receive_clock.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!(
            first_clock.attachment().unwrap().to_bytes().as_ref(),
            b"gz.msgs.Clock"
        );
        let clock = Clock::decode(first_clock.payload().to_bytes().as_ref())
            .unwrap()
            .sim
            .unwrap();
        assert_eq!((clock.sec, clock.nsec), (0, 0));
        // Observe actual camera publisher registrations before the single run
        // control. This is not a claim of camera data delivery/readiness.
        let registrations: Vec<_> = observer
            .liveliness()
            .get("@gz/**")
            .timeout(Duration::from_secs(1))
            .wait()
            .unwrap()
            .into_iter()
            .map(|reply| reply.into_result().unwrap().key_expr().to_string())
            .collect();
        for message in ["gz.msgs.Image", "gz.msgs.CameraInfo"] {
            assert!(registrations.iter().any(|name| name.contains("/MP/") && name.ends_with(&format!("/{message}/%/%"))), "{registrations:?}");
        }
        let replies = observer
            .get(key("/world/default/control"))
            .payload(
                WorldControl {
                    pause: false,
                    ..Default::default()
                }
                .encode_to_vec(),
            )
            .timeout(Duration::from_secs(5))
            .wait()
            .unwrap();
        let reply = replies
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .into_result()
            .unwrap();
        assert!(
            Boolean::decode(reply.payload().to_bytes().as_ref())
                .unwrap()
                .data
        );
        let mut observed_worker = None;
        if silent {
            let metadata = infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
            assert_eq!(
                metadata.attachment().unwrap().to_bytes().as_ref(),
                b"gz.msgs.CameraInfo"
            );
            let info = CameraInfo::decode(metadata.payload().to_bytes().as_ref()).unwrap();
            assert_eq!((info.width, info.height), (64, 64));
            let header = info.header.unwrap();
            assert_eq!(header.data[0].key, "frame_id");
            assert_eq!(header.data[0].value, ["rig/body/camera"]);
            let stamp = header.stamp.unwrap();
            assert!((0..1_000_000_000).contains(&stamp.nsec));
            let acquired = Duration::new(
                u64::try_from(stamp.sec).unwrap(),
                u32::try_from(stamp.nsec).unwrap(),
            );
            assert!(
                acquired > Duration::ZERO
                    && acquired + Duration::from_millis(100) < Duration::from_secs(10)
            );
            // Acquisition admission is not a process-start claim. Fence the
            // fixture's actual exec path independently before asserting teardown.
            let deadline = Instant::now() + Duration::from_secs(5);
            let worker_pid = loop {
                if let Some(pid) = fs::read_to_string(&pid_file)
                    .ok()
                    .and_then(|text| text.trim().parse::<u32>().ok())
                {
                    break pid;
                }
                assert!(
                    process.child.try_wait().unwrap().is_none(),
                    "CLI exited before fixture startup"
                );
                assert!(Instant::now() < deadline, "silent fixture did not start");
                thread::sleep(Duration::from_millis(1));
            };
            assert!(worker_pid > 0 && Path::new(&format!("/proc/{worker_pid}")).exists());
            observed_worker = Some(worker_pid);
        }
        let deadline = Instant::now() + Duration::from_secs(20);
        let status = loop {
            if let Some(status) = process.child.try_wait().unwrap() {
                break status;
            }
            assert!(
                Instant::now() < deadline,
                "CLI did not finish: {}",
                fs::read_to_string(&process.stderr).unwrap()
            );
            thread::sleep(Duration::from_millis(5));
        };
        let stdout = fs::read_to_string(&process.stdout).unwrap();
        let stderr = fs::read_to_string(&process.stderr).unwrap();
        if silent {
            assert!(status.success(), "{status}: {stdout} {stderr}");
            assert_eq!(
                stdout,
                "simulation_ns=10000000000 iterations=10000\ndefault::rig::body 0 0 0\ndefault::target::body 5 0 0\n"
            );
            assert_eq!(
                stderr.lines().collect::<Vec<_>>(),
                ["camera output backpressure: bounded worker busy; simulation continues"]
            );
            assert!(
                infos.try_recv().unwrap().is_none(),
                "Busy must not publish another acquisition"
            );
            let worker_pid = observed_worker.unwrap();
            assert!(
                !Path::new(&format!("/proc/{worker_pid}")).exists(),
                "silent worker was not reaped"
            );
        } else {
            assert_eq!(status.code(), Some(1), "{status}: {stdout} {stderr}");
            assert!(stdout.is_empty());
            assert!(
                stderr
                    .lines()
                    .any(|line| line.starts_with("renderer client: Io(")
                        && line.contains("NotFound")),
                "{stderr}"
            );
            assert!(!pid_file.exists());
        }
        assert_eq!(
            images_seen.load(Ordering::Relaxed),
            0,
            "neither fixture can produce an image"
        );
        process.complete = true;
        drop((images, infos, clocks));
        observer.close().wait().unwrap();
    }
}

#[cfg(feature = "rgb-camera")]
#[test]
fn admitted_depth_camera_without_image_interest_keeps_its_worker_lazy() {
    if isolation::child(
        "admitted_depth_camera_without_image_interest_keeps_its_worker_lazy",
        20,
    ) {
        return;
    }
    let depth = include_str!("../../gz-camera-output/tests/fixtures/m6-depth.sdf");
    let directory = tempfile::tempdir().unwrap();
    let worker = directory.path().join("absent-depth-worker");
    assert!(worker.is_absolute());
    assert!(!worker.exists());
    let result = command(
        depth,
        Some(&worker),
        &[
            "--physics-engine",
            "rapier",
            "--server",
            "--run",
            "--iterations",
            "3",
        ],
    );
    assert!(result.status.success(), "{result:?}");
    assert_eq!(
        String::from_utf8_lossy(&result.stdout),
        "simulation_ns=3000000 iterations=3\ndefault::rig::body 3 -2 1\ndefault::target::body 3 3 1\n"
    );
    assert!(result.stderr.is_empty(), "{result:?}");
    assert!(!worker.exists());
}
