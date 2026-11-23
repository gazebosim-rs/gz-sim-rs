// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Start one stock convoy worker over SSH. Its control pipe and its Zenoh
//! loopback forward share the SSH process lifetime.
//! Set the deployment's top-level `worker_sha256` to the 64 lowercase hex
//! digits of the deployed `physics_worker` executable's SHA-256 digest.
//! `--check-sshd-liveness BUDGET_SECONDS` reads effective `sshd -T -C ...`
//! output on stdin for a prospective login. It checks the estimated idle
//! probe schedule. One-way loss, other traffic and OS scheduling can delay
//! detection. Run this operator check separately from launching a worker.
#[path = "convoy_ssh_worker/sshd_liveness.rs"]
mod sshd_liveness;

use serde::Deserialize;
use std::{
    env,
    ffi::OsString,
    fs::OpenOptions,
    io::{self, Read},
    num::NonZeroU16,
    path::{Path, PathBuf},
    process::Command,
};

const CONFIG_ENV: &str = "GZ_CONVOY_SSH_CONFIG";
const MAX_CONFIG: u64 = 64 * 1024;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Deployment {
    ssh_config: Option<PathBuf>,
    worker_sha256: String,
    slots: Vec<Slot>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Slot {
    host: String,
    worker: String,
    lease: String,
    remote_port: NonZeroU16,
    local_port: NonZeroU16,
}

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message.into())
}

fn remote_path(path: &str) -> bool {
    path.strip_prefix('/').is_some_and(|rest| {
        !rest.chars().any(char::is_control)
            && rest.split('/').all(|part| !matches!(part, "" | "." | ".."))
    })
}

// OpenSSH sends a command string to the remote login shell. Quote the two
// operator-supplied paths even though the local ssh invocation uses argv.
fn quoted(path: &str) -> String {
    format!("'{}'", path.replace('\'', "'\\''"))
}

impl Deployment {
    fn read(path: &Path) -> io::Result<Self> {
        if !path.is_absolute() {
            return Err(invalid("GZ_CONVOY_SSH_CONFIG must be an absolute path"));
        }
        let mut options = OpenOptions::new();
        options.read(true);
        #[cfg(target_os = "linux")]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.custom_flags(rustix::fs::OFlags::NONBLOCK.bits() as i32);
        }
        let file = options.open(path)?;
        if !file.metadata()?.is_file() {
            return Err(invalid("convoy SSH deployment must be a regular file"));
        }
        let mut bytes = Vec::new();
        file.take(MAX_CONFIG + 1).read_to_end(&mut bytes)?;
        if bytes.len() as u64 > MAX_CONFIG {
            return Err(invalid("convoy SSH deployment exceeds 64 KiB"));
        }
        let deployment: Self = serde_json::from_slice(&bytes)?;
        deployment.validate()?;
        Ok(deployment)
    }

    fn validate(&self) -> io::Result<()> {
        if self.worker_sha256.len() != 64
            || !self
                .worker_sha256
                .bytes()
                .all(|c| matches!(c, b'0'..=b'9' | b'a'..=b'f'))
        {
            return Err(invalid(
                "worker_sha256 must contain 64 lowercase hexadecimal digits",
            ));
        }
        if !(3..=8).contains(&self.slots.len()) {
            return Err(invalid("expected 3 through 8 convoy SSH slots"));
        }
        if let Some(config) = &self.ssh_config
            && (!config.is_absolute() || !config.is_file())
        {
            return Err(invalid("ssh_config must name an absolute regular file"));
        }
        for (i, slot) in self.slots.iter().enumerate() {
            if !slot
                .host
                .as_bytes()
                .first()
                .is_some_and(u8::is_ascii_alphanumeric)
                || !slot
                    .host
                    .bytes()
                    .all(|c| c.is_ascii_alphanumeric() || b"._-".contains(&c))
            {
                return Err(invalid(format!("slot {i} requires an SSH host alias")));
            }
            if !remote_path(&slot.worker) || !remote_path(&slot.lease) || slot.worker == slot.lease
            {
                return Err(invalid(format!(
                    "slot {i} requires distinct absolute worker and lease file paths"
                )));
            }
            for other in &self.slots[..i] {
                if other.local_port == slot.local_port {
                    return Err(invalid(format!("slot {i} repeats a local forwarding port")));
                }
                if other.host.eq_ignore_ascii_case(&slot.host)
                    && (other.remote_port == slot.remote_port
                        || other.lease == slot.lease
                        || other.worker == slot.lease
                        || slot.worker == other.lease)
                {
                    return Err(invalid(format!(
                        "slot {i} conflicts with another remote port or lease on {}",
                        slot.host
                    )));
                }
            }
        }
        Ok(())
    }

    fn command(&self, index: usize) -> io::Result<Command> {
        let slot = self
            .slots
            .get(index)
            .ok_or_else(|| invalid("slot missing from SSH deployment"))?;
        let mut ssh = Command::new("ssh");
        if let Some(config) = &self.ssh_config {
            ssh.arg("-F").arg(config);
        }
        ssh.args(["-T", "-a", "-x", "-e", "none", "-S", "none"]);
        for option in [
            "BatchMode=yes",
            "StrictHostKeyChecking=yes",
            "UpdateHostKeys=no",
            "ControlMaster=no",
            "ExitOnForwardFailure=yes",
            "ClearAllForwardings=no",
            "StdinNull=no",
            "ForkAfterAuthentication=no",
            "ServerAliveInterval=2",
            "ServerAliveCountMax=2",
            "ConnectTimeout=5",
            "ConnectionAttempts=1",
        ] {
            ssh.arg("-o").arg(option);
        }
        ssh.arg("-L")
            .arg(format!(
                "127.0.0.1:{}:127.0.0.1:{}",
                slot.local_port, slot.remote_port
            ))
            .arg("--")
            .arg(&slot.host)
            .arg(format!(
                "exec {} --managed-multisphere-ssh-v1 {index} {} {} {} {}",
                quoted(&slot.worker),
                quoted(&slot.lease),
                slot.remote_port,
                slot.local_port,
                self.worker_sha256
            ));
        Ok(ssh)
    }
}

fn managed_slot(args: &[OsString]) -> io::Result<usize> {
    let [flag, slot, lease] = args else {
        return Err(invalid(
            "expected --managed-multisphere-v2 SLOT LOCAL_LEASE",
        ));
    };
    let index = slot
        .to_str()
        .and_then(|n| n.parse::<usize>().ok())
        .ok_or_else(|| invalid("invalid convoy worker slot"))?;
    let lease = Path::new(lease);
    if flag != "--managed-multisphere-v2" || !lease.is_absolute() || lease.file_name().is_none() {
        return Err(invalid(
            "expected a managed convoy slot and its local lease path",
        ));
    }
    Ok(index)
}

fn main() -> io::Result<()> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    if let Some((flag, rest)) = args.split_first()
        && flag == sshd_liveness::FLAG
    {
        return sshd_liveness::check(rest, io::stdin().lock(), io::stdout().lock());
    }
    launch(&args)
}

#[cfg(unix)]
fn launch(args: &[OsString]) -> io::Result<()> {
    use std::os::unix::process::CommandExt;
    let index = managed_slot(args)?;
    let path = env::var_os(CONFIG_ENV).ok_or_else(|| invalid("missing GZ_CONVOY_SSH_CONFIG"))?;
    let deployment = Deployment::read(Path::new(&path))?;
    Err(deployment.command(index)?.exec())
}

#[cfg(not(unix))]
fn launch(_args: &[OsString]) -> io::Result<()> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "convoy SSH launch requires Unix exec",
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    const DIGEST: &str = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

    fn fixture() -> Value {
        json!({"worker_sha256": DIGEST, "slots": (0..3).map(|i| json!({"host":"node-a", "worker":"/opt/solver's worker",
            "lease":format!("/run/convoy/{i}"), "remote_port":7000+i, "local_port":8000+i})).collect::<Vec<_>>()})
    }

    fn deployment(value: Value) -> io::Result<Deployment> {
        let deployment: Deployment = serde_json::from_value(value)?;
        deployment.validate()?;
        Ok(deployment)
    }

    #[test]
    fn openssh_forwards_only_the_selected_slot_and_quotes_the_remote_paths() -> io::Result<()> {
        let config = deployment(fixture())?;
        let command = config.command(1)?;
        let args = command
            .get_args()
            .map(|a| a.to_str().unwrap())
            .collect::<Vec<_>>();
        assert!(
            args.windows(2)
                .any(|w| w == ["-o", "StrictHostKeyChecking=yes"])
        );
        let remote = format!(
            "exec '/opt/solver'\\''s worker' --managed-multisphere-ssh-v1 1 '/run/convoy/1' 7001 8001 {DIGEST}"
        );
        assert!(args.ends_with(&[
            "-L",
            "127.0.0.1:8001:127.0.0.1:7001",
            "--",
            "node-a",
            &remote
        ]));
        assert!(config.command(3).is_err());
        let args = ["--managed-multisphere-v2", "1", "/ledger/worker-1.lock"].map(OsString::from);
        assert_eq!(managed_slot(&args)?, 1);
        assert!(managed_slot(&args[1..]).is_err());
        Ok(())
    }

    #[test]
    fn requires_a_complete_lowercase_worker_digest() {
        let mut config = fixture();
        config.as_object_mut().unwrap().remove("worker_sha256");
        assert!(deployment(config.clone()).is_err());
        for digest in [
            DIGEST[..63].to_owned(),
            format!("{DIGEST}0"),
            DIGEST.to_uppercase(),
            "g".repeat(64),
        ] {
            config["worker_sha256"] = json!(digest);
            assert!(deployment(config.clone()).is_err(), "{digest}");
        }
    }

    #[test]
    fn rejects_ambiguous_or_invalid_deployments_including_unused_slots() {
        for (key, value) in [
            ("host", json!("-other")),
            ("lease", json!("/run/../lease")),
            ("lease", json!("/run/convoy/0")),
            ("worker", json!("relative")),
            ("remote_port", json!(7000)),
            ("local_port", json!(8001)),
            ("remote_port", json!(0)),
            ("local_port", json!(65536)),
            ("unknown", json!(true)),
        ] {
            let mut config = fixture();
            config["slots"][2][key] = value;
            assert!(deployment(config).is_err(), "{key}");
        }
        let mut separate = fixture();
        separate["slots"][2]["host"] = json!("node-b");
        separate["slots"][2]["remote_port"] = json!(7000);
        separate["slots"][2]["lease"] = json!("/run/convoy/0");
        assert!(deployment(separate).is_ok());
    }
}
