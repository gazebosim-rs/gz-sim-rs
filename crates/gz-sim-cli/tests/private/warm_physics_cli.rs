// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Network, Process, Samples, listen_line};
use gz_sim_proto::{
    msgs::{Clock, WorldControl},
    prost::Message,
};
use std::{
    fs,
    path::Path,
    process::{Command, Stdio},
    time::{Duration, Instant},
};

const TIMEOUT: Duration = Duration::from_secs(20);
fn run(base: &impl Fn() -> Command, extra: &[&str], ok: bool) -> String {
    let child = base()
        .args(extra)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let result = Process(Some(child)).wait(TIMEOUT);
    assert_eq!(
        result.status.success(),
        ok,
        "{extra:?}: {}",
        String::from_utf8_lossy(&result.stderr)
    );
    String::from_utf8(if ok { result.stdout } else { result.stderr }).unwrap()
}

fn result(text: &str, steps: usize) -> &str {
    let (clock, poses) = text.split_once('\n').unwrap();
    assert_eq!(clock, format!("simulation_ns=600000000 iterations={steps}"));
    assert!(poses.contains("::") && poses.lines().all(|l| l.split_whitespace().count() == 4));
    poses
}

#[test]
#[ignore = "requires the headless graph's stock physics_worker"]
fn authored_worlds_resume_from_offline_and_operator_selected_warm_cuts() {
    let worker = std::env::var_os("GZ_MULTI_TEST_WORKER").expect("stock physics worker");
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    for (file, engine) in [
        ("multifidelity_curved_shuttle.sdf", "multifidelity"),
        ("distributed_rolling_sphere.sdf", "distributed"),
    ] {
        let dir = tempfile::tempdir().unwrap();
        let initial = dir.path().join("offline.cut");
        let paused = dir.path().join("paused.cut");
        let base = || {
            let mut cmd = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
            cmd.arg(root.join("examples").join(file))
                .args(["--physics-engine", engine, "--physics-worker"])
                .arg(&worker);
            if engine == "distributed" {
                cmd.args(["--physics-workers", "3"]);
            }
            cmd
        };
        let continuous = run(&base, &["--iterations", "120"], true);
        let expected = result(&continuous, 120);
        let file = initial.to_str().unwrap();
        let split = run(
            &base,
            &[
                "--checkpoint-at",
                "80",
                "--save-pool-checkpoint",
                file,
                "--iterations",
                "40",
            ],
            true,
        );
        assert_eq!(result(&split, 40), expected);
        let reopened = run(
            &base,
            &["--resume-pool-checkpoint", file, "--iterations", "40"],
            true,
        );
        assert_eq!(result(&reopened, 40), expected);
        let original = fs::read(&initial).unwrap();
        assert!(!original.is_empty());

        let mut network = Network::new();
        let mut service = base();
        service
            .args([
                "--server",
                "--resume-pool-checkpoint",
                file,
                "--live-pool-checkpoint",
            ])
            .arg(&paused);
        let mut server = Process(Some(network.spawn_server(&mut service)));
        assert!(listen_line(&mut server).contains("live pool checkpoint armed"));
        let session = network.session();
        let clock = Samples::new(&session, network.key("/world/default/clock"));
        network.control(
            &session,
            WorldControl {
                pause: true,
                step: true,
                ..Default::default()
            }
            .encode_to_vec(),
        );
        let deadline = Instant::now() + TIMEOUT;
        loop {
            let sample = clock.recv_timeout(deadline.saturating_duration_since(Instant::now()));
            let time = Clock::decode(sample.payload().to_bytes().as_ref())
                .unwrap()
                .sim
                .unwrap();
            assert_eq!(time.sec, 0);
            if time.nsec == 405_000_000 {
                break;
            }
            assert_eq!(time.nsec, 400_000_000);
        }
        let pid = rustix::process::Pid::from_child(server.0.as_ref().unwrap());
        rustix::process::kill_process(pid, rustix::process::Signal::USR1).unwrap();
        let saved = listen_line(&mut server);
        assert!(saved.contains("saved epoch=0 tick=81"), "{saved}");
        let sealed = fs::read(&paused).unwrap();
        assert_ne!(original, sealed);
        rustix::process::kill_process(pid, rustix::process::Signal::USR1).unwrap();
        let denied = listen_line(&mut server);
        assert!(denied.contains("refused epoch=0 tick=81"), "{denied}");
        assert_eq!(fs::read(&paused).unwrap(), sealed);
        rustix::process::kill_process(pid, rustix::process::Signal::TERM).unwrap();
        assert!(server.wait(TIMEOUT).status.success());
        let published = run(
            &base,
            &[
                "--resume-pool-checkpoint",
                paused.to_str().unwrap(),
                "--iterations",
                "39",
            ],
            true,
        );
        assert_eq!(result(&published, 39), expected);
        let torn = dir.path().join("incomplete.cut");
        fs::write(&torn, &sealed[..sealed.len() / 2]).unwrap();
        assert!(
            !run(
                &base,
                &[
                    "--resume-pool-checkpoint",
                    torn.to_str().unwrap(),
                    "--iterations",
                    "39"
                ],
                false
            )
            .is_empty()
        );
        assert_eq!(fs::read(initial).unwrap(), original);
    }
}
