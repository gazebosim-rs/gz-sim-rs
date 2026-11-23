// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Launch an actual CLI and deliver real signals to its pidfd.
use super::*;
use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
use sha2::{Digest, Sha256};
use std::{
    fs,
    path::PathBuf,
    process::{Child, Command, Stdio},
    thread,
};

struct Cli(Child);
impl Drop for Cli {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}
fn line(cli: &mut Child, stderr: &Path, needle: &str, after: usize) -> Result<String> {
    let end = Instant::now() + Duration::from_secs(15);
    loop {
        let body = fs::read_to_string(stderr)?;
        if let Some(text) = body
            .get(after..)
            .and_then(|s| s.lines().find(|line| line.contains(needle)))
        {
            return Ok(text.to_owned());
        }
        if let Some(status) = cli.try_wait()? {
            return Err(format!("CLI exited {status}: {body}").into());
        }
        if Instant::now() >= end {
            return Err(format!("missing {needle}: {body}").into());
        }
        thread::sleep(Duration::from_millis(5));
    }
}

#[test]
#[ignore = "set GZ_OPERATOR_CLI to the qualified release gz-sim-rs binary"]
fn running_refusal_pause_ack_filesystem_retry_and_no_clobber_use_one_live_signal() -> Result<()> {
    let cli =
        PathBuf::from(std::env::var_os("GZ_OPERATOR_CLI").ok_or("GZ_OPERATOR_CLI is required")?);
    let dir = tempfile::tempdir()?;
    let requested = dir.path().join("operator-selected/new.gzscene");
    let sdf = dir.path().join("stack.sdf");
    fs::write(&sdf, SDF)?;
    let log = dir.path().join("cli.log");
    let (peer, client) = local_peer()?;
    let config = dir.path().join("client.json");
    fs::write(&config, client)?;
    let opts = ServerOptions {
        partition: format!("live-signal-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    let mut server = Cli(Command::new(cli)
        .arg(&sdf)
        .args([
            "--physics-engine",
            "distributed",
            "--physics-workers",
            "4",
            "--adaptive-contacts",
            "--physics-worker",
            env!("CARGO_BIN_EXE_physics_worker"),
            "--server",
            "--run",
            "--live-pool-checkpoint",
        ])
        .arg(&requested)
        .env("GZ_PARTITION", &opts.partition)
        .env("ZENOH_CONFIG", config)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(fs::File::create(&log)?)
        .spawn()?);
    let pid = Pid::from_child(&server.0);
    let fd = pidfd_open(pid, PidfdFlags::empty())?;
    line(&mut server.0, &log, "checkpoint armed", 0)?;
    pidfd_send_signal(&fd, Signal::USR1)?;
    let running = line(&mut server.0, &log, "checkpoint refused", 0)?;
    assert!(
        running.contains("paused") && !requested.exists(),
        "{running}"
    );
    let key = opts.topic_key("/world/default/control")?;
    let pause = peer
        .declare_querier(key)
        .timeout(Duration::from_secs(10))
        .wait()
        .map_err(|e| e.to_string())?;
    let end = Instant::now() + Duration::from_secs(10);
    while !pause
        .matching_status()
        .wait()
        .map_err(|e| e.to_string())?
        .matching()
    {
        assert!(Instant::now() < end, "live control endpoint did not match");
        thread::sleep(Duration::from_millis(2));
    }
    let reply = pause
        .get()
        .payload(
            WorldControl {
                pause: true,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .wait()
        .map_err(|e| e.to_string())?
        .recv_timeout(Duration::from_secs(10))
        .map_err(|e| e.to_string())?
        .expect("actual pause reply");
    assert!(
        Boolean::decode(
            reply
                .result()
                .map_err(|_| "control query failed")?
                .payload()
                .to_bytes()
                .as_ref()
        )?
        .data
    );

    let offset = fs::metadata(&log)?.len() as usize;
    pidfd_send_signal(&fd, Signal::USR1)?;
    let io = line(&mut server.0, &log, "checkpoint refused", offset)?;
    assert!(io.contains("file I/O") && !requested.exists(), "{io}");
    fs::create_dir(requested.parent().unwrap())?;
    pidfd_send_signal(&fd, Signal::USR1)?;
    let saved = line(&mut server.0, &log, "checkpoint saved", offset)?;
    let content = fs::read(&requested)?;
    let offset = fs::metadata(&log)?.len() as usize;
    pidfd_send_signal(&fd, Signal::USR1)?;
    let collision = line(&mut server.0, &log, "checkpoint refused", offset)?;
    assert!(collision.contains("exists"), "{collision}");
    assert_eq!(content, fs::read(&requested)?);
    pidfd_send_signal(&fd, Signal::TERM)?;
    assert!(server.0.wait()?.success());
    let restored = resume(&requested)?;
    assert!(restored.simulation().is_paused());
    let tick = restored
        .simulation()
        .world()
        .resource::<gz_sim_core::UpdateInfo>()
        .iteration;
    assert!(tick > 0 && saved.contains(&format!("tick={tick} ")));
    println!(
        "PID {} refused while running, acknowledged a real peer pause, refused at an absent directory, published on SIGUSR1 retry and refused overwrite. Restored committed tick {tick}, bytes {}, sha256={:x}.",
        pid.as_raw_nonzero(),
        content.len(),
        Sha256::digest(content)
    );
    Ok(())
}
