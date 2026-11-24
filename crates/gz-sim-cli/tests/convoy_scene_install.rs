// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise the public operator supervisor across a rejected Bevy scene reinstall.
#![cfg(all(target_os = "linux", feature = "distributed-physics"))]

use gz_physics::PhysicsBackend;
use gz_sim_cli::live::{ConvoyDeployment, run_recoverable_convoy};
use gz_sim_scene::distributed_hinge::{DistributedMultiSphereBackend, HingeWorkerOptions};
use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
use serde_json::{Value, json};
use std::{
    cell::Cell,
    env,
    error::Error,
    fs::{self, File},
    net::{SocketAddr, TcpListener, TcpStream},
    path::{Path, PathBuf},
    process::{Child, Command},
    thread,
    time::{Duration, Instant},
};
use tungstenite::{Message, WebSocket};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
const CHILD: &str = "GZ_CONVOY_SCENE_INSTALL_CHILD";
const TEST: &str = "refused_scene_reinstall_parks_without_reissuing_authority";
const TICK: u64 = 150;

fn supervisor(root: &Path) -> Result<()> {
    let worker = PathBuf::from(env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing worker")?);
    let session = root.join("session");
    let calls = Cell::new(0);
    run_recoverable_convoy(
        ConvoyDeployment {
            xml: XML,
            source: "convoy.sdf",
            world: "default",
            worker: &worker,
            session: &session,
            options: HingeWorkerOptions {
                width: Some(3),
                ..Default::default()
            },
        },
        env::var("GZ_CONVOY_TEST_PORT")?.parse()?,
        |sim| {
            let call = calls.get() + 1;
            calls.set(call);
            if call == 1 {
                sim.step_exact(TICK)?;
                let backend = sim.world().resource::<PhysicsBackend>();
                let joint = backend
                    .downcast_ref::<DistributedMultiSphereBackend>()
                    .ok_or("missing initial backend")?
                    .pool()
                    .spec()
                    .hinge
                    .joint_id;
                gz_sim_scene::queue_joint_effort(sim, joint, -0.005)?;
            }
            let backend = sim.world().resource::<PhysicsBackend>();
            let pool = backend
                .downcast_ref::<DistributedMultiSphereBackend>()
                .ok_or("missing installed backend")?
                .pool();
            let owner = pool.owners()[0];
            let callback = json!({
                "epoch": sim.epoch(), "elapsed_ms": sim.elapsed().as_millis() as u64,
                "state": pool.observation(), "pending": pool.pending_joint_effort(),
                "owner": owner, "pid": pool.worker_pid(owner).ok_or("owner unavailable")?,
            });
            fs::write(
                root.join(format!("install-{call}.json")),
                serde_json::to_vec(&callback)?,
            )?;
            if call == 1 {
                Ok(())
            } else {
                Err("test scene setup refused guarded reinstall".into())
            }
        },
    )?;
    assert_eq!(calls.get(), 2);
    Ok(())
}

struct Supervisor(Child);
impl Drop for Supervisor {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn receive(peer: &mut WebSocket<TcpStream>, kind: &str) -> Result<Value> {
    let deadline = Instant::now() + Duration::from_secs(4);
    while Instant::now() < deadline {
        let msg = peer.read()?;
        let frame: Value = serde_json::from_str(msg.to_text()?)?;
        if frame["kind"] == "operator_restored" {
            return Err("scene published a successful handoff despite refusing setup".into());
        }
        if frame["kind"] == kind {
            return Ok(frame);
        }
    }
    Err(format!("missing {kind}").into())
}

fn connect_at(addr: SocketAddr, kind: &str) -> Result<(WebSocket<TcpStream>, Value)> {
    let deadline = Instant::now() + Duration::from_secs(20);
    while Instant::now() < deadline {
        if let Ok(stream) = TcpStream::connect_timeout(&addr, Duration::from_secs(1)) {
            stream.set_read_timeout(Some(Duration::from_secs(2)))?;
            stream.set_write_timeout(Some(Duration::from_secs(2)))?;
            if let Ok((mut peer, _)) =
                tungstenite::client(format!("ws://{addr}/?operator=1"), stream)
                && let Ok(frame) = receive(&mut peer, kind)
            {
                return Ok((peer, frame));
            }
        }
        thread::sleep(Duration::from_millis(20));
    }
    Err(format!("supervisor did not publish {kind} at {addr}").into())
}

fn send(peer: &mut WebSocket<TcpStream>, value: Value) -> Result<()> {
    Ok(peer.send(Message::Text(value.to_string().into()))?)
}

fn recover(status: &Value, id: u64) -> Value {
    json!({"version":1,"kind":"recover_workers","request_id":id,
        "epoch":status["epoch"],"tick":status["tick"],"token":status["token"]})
}

fn signal(raw_pid: u32, signal: Signal) -> Result<()> {
    let pid = Pid::from_raw(i32::try_from(raw_pid)?).ok_or("invalid pid")?;
    Ok(pidfd_send_signal(
        pidfd_open(pid, PidfdFlags::empty())?,
        signal,
    )?)
}

#[test]
#[ignore = "requires GZ_MULTI_TEST_WORKER pointing to the stock physics worker"]
fn refused_scene_reinstall_parks_without_reissuing_authority() -> Result<()> {
    if let Some(dir) = env::var_os(CHILD) {
        return supervisor(Path::new(&dir));
    }
    let temp = tempfile::tempdir()?;
    let root = temp.path();
    let session = root.join("session");
    let archive = session.join("barrier.gzmultisphere");
    let addr = TcpListener::bind("127.0.0.1:0")?.local_addr()?;
    let config = root.join("zenoh.json");
    fs::write(&config, br#"{"mode":"peer","listen":{"endpoints":["tcp/127.0.0.1:0"]},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}}"#)?;
    // Isolate the real supervisor's signal handlers and transport from the test harness.
    let mut child = Supervisor(
        Command::new(env::current_exe()?)
            .args(["--ignored", "--exact", TEST, "--nocapture"])
            .env(CHILD, root)
            .env("GZ_CONVOY_TEST_PORT", addr.port().to_string())
            .env("ZENOH_CONFIG", config)
            .env("GZ_PARTITION", format!("scene-install-{}", addr.port()))
            .spawn()?,
    );
    let (mut live, initial) = connect_at(addr, "scene")?;
    let installed: Value = serde_json::from_slice(&fs::read(root.join("install-1.json"))?)?;
    assert_eq!(initial["paused"], true);
    assert_eq!(installed["pending"], -0.005);
    assert_eq!(installed["elapsed_ms"], TICK);
    assert!(
        installed["state"]["sphere_reaction"]["normal_impulse"]
            .as_f64()
            .unwrap()
            > 0.
    );
    let original = fs::read(&archive)?;
    let owner = installed["owner"].as_u64().unwrap();
    signal(installed["pid"].as_u64().unwrap().try_into()?, Signal::KILL)?;
    let lease = File::open(session.join(format!("worker-{owner}.lock")))?;
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match lease.try_lock() {
            Ok(()) => break,
            Err(fs::TryLockError::WouldBlock) if Instant::now() < deadline => {
                thread::sleep(Duration::from_millis(5))
            }
            Err(e) => return Err(e.into()),
        }
    }
    send(
        &mut live,
        json!({"version":1,"kind":"control","action":"resume","request_id":1}),
    )?;
    // The numerical fault may close the old socket before its resume reply.
    // Only the separately rebound parked endpoint can authorize an operator restore.
    let (mut eligible, offer) = connect_at(addr, "operator_fence")?;
    let fenced_scene = receive(&mut eligible, "scene")?;
    assert_eq!(offer["eligible"], true);
    assert_eq!(offer["tick"], TICK);
    assert_eq!(offer["epoch"], installed["epoch"]);
    assert_eq!(fs::read(&archive)?, original);
    drop(lease);
    send(&mut eligible, recover(&offer, 1))?;
    assert_eq!(receive(&mut eligible, "operator_reply")?["accepted"], true);

    let (mut parked, status) = connect_at(addr, "operator_fence")?;
    let cached = receive(&mut parked, "scene")?;
    let reinstalled: Value = serde_json::from_slice(&fs::read(root.join("install-2.json"))?)?;
    for key in ["epoch", "elapsed_ms", "state", "pending", "owner"] {
        assert_eq!(installed[key], reinstalled[key], "restored {key}");
    }
    assert_ne!(installed["pid"], reinstalled["pid"]);
    assert_eq!(cached, fenced_scene);
    assert_eq!(cached["visuals"], initial["visuals"]);
    assert_eq!(status["eligible"], false);
    assert_eq!(status["tick"], TICK);
    assert_eq!(status["epoch"], offer["epoch"]);
    assert_ne!(status["token"], offer["token"]);
    assert!(
        status["reason"]
            .as_str()
            .unwrap()
            .contains("Scene installation failed")
    );
    for (id, token) in [(1, &offer), (2, &status)] {
        send(&mut parked, recover(token, id))?;
        let reply = receive(&mut parked, "operator_reply")?;
        assert_eq!(reply["request_id"], id);
        assert_eq!(reply["accepted"], false);
    }
    send(
        &mut parked,
        json!({"version":1,"kind":"operator_step","request_id":3,"epoch":status["epoch"],"tick":TICK}),
    )?;
    let denied = receive(&mut parked, "ack")?;
    assert_eq!(denied["ok"], false);
    assert_eq!(denied["nanoseconds"], TICK * 1_000_000);
    send(
        &mut parked,
        json!({"version":1,"kind":"control","action":"resume","request_id":4}),
    )?;
    let resume = receive(&mut parked, "ack")?;
    assert_eq!(resume["ok"], false);
    assert_eq!(resume["request_id"], 4);
    let (_, observer) = connect_at(addr, "operator_fence")?;
    assert_eq!(observer, status);
    assert_eq!(fs::read(&archive)?, original);
    for name in [
        "controller.lock",
        "worker-0.lock",
        "worker-1.lock",
        "worker-2.lock",
    ] {
        File::open(session.join(name))?.try_lock()?;
    }
    signal(child.0.id(), Signal::TERM)?;
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        if let Some(exit) = child.0.try_wait()? {
            assert!(exit.success(), "unclean supervisor exit {exit}");
            assert!(!root.join("install-3.json").exists());
            assert_eq!(fs::read(&archive)?, original);
            eprintln!(
                "Scene reinstall at tick {TICK} rejected after restoring the loaded contact and pending effort. Original archive, inspection-only port, control denials, released leases and clean SIGTERM verified."
            );
            return Ok(());
        }
        thread::sleep(Duration::from_millis(10));
    }
    Err("supervisor did not terminate".into())
}
