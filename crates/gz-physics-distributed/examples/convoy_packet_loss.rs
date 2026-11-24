// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Inject TCP packet loss on a dedicated Docker worker's SSH listener and
//! observe its real bind-mounted lease independently of the affected link.
use clap::{Parser, ValueEnum};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::{self, File, TryLockError},
    path::{Path, PathBuf},
    process::Command,
    thread,
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const PREF: &str = "41931";
const DEADLINE: Duration = Duration::from_secs(50);

#[derive(Clone, Copy, Debug, ValueEnum)]
enum Direction {
    Up,
    Down,
    Both,
}

#[derive(Clone, Copy, Debug, ValueEnum, PartialEq)]
enum Expect {
    /// Hold the cut through fencing and require the remote lease to become free.
    Release,
    /// Hold the cut through fencing and require the old lease to remain held.
    Held,
    /// Restore after a short interval, while the old worker still holds its lease.
    Transient,
    /// Restore after the specified time without notifying the coordinator.
    Automatic,
}

#[derive(Parser)]
struct Args {
    /// Output directory of the companion convoy_host_recovery process.
    #[arg(long)]
    trial: PathBuf,
    /// A dedicated test container with a clsact qdisc on eth0.
    #[arg(long)]
    container: String,
    #[arg(long)]
    ssh_port: std::num::NonZeroU16,
    /// Host path of the same bind-mounted lease file the remote solver locks.
    #[arg(long)]
    lease: PathBuf,
    #[arg(long)]
    slot: usize,
    #[arg(long, value_enum)]
    direction: Direction,
    #[arg(long, value_enum)]
    expect: Expect,
    /// Duration of a short or automatic-reopen cut.
    #[arg(
        long = "hold-ms",
        visible_alias = "transient-ms",
        default_value_t = 350
    )]
    hold_ms: u64,
}

fn tc(container: &str, args: &[&str]) -> Result<Vec<u8>> {
    let out = Command::new("docker")
        .args(["exec", container, "tc"])
        .args(args)
        .output()?;
    if !out.status.success() {
        return Err(format!("tc {args:?}: {}", String::from_utf8_lossy(&out.stderr)).into());
    }
    Ok(out.stdout)
}

struct Filter<'a> {
    container: &'a str,
    direction: &'static str,
    installed: bool,
}

impl Filter<'_> {
    fn install(&mut self, port: &str) -> Result<()> {
        let field = if self.direction == "ingress" {
            "dst_port"
        } else {
            "src_port"
        };
        tc(
            self.container,
            &[
                "filter",
                "add",
                "dev",
                "eth0",
                self.direction,
                "protocol",
                "ip",
                "pref",
                PREF,
                "handle",
                "1",
                "flower",
                "ip_proto",
                "tcp",
                field,
                port,
                "action",
                "drop",
            ],
        )?;
        self.installed = true;
        Ok(())
    }

    fn counts(&self) -> Result<Value> {
        let filters: Vec<Value> = serde_json::from_slice(&tc(
            self.container,
            &[
                "-j",
                "-s",
                "filter",
                "show",
                "dev",
                "eth0",
                self.direction,
                "pref",
                PREF,
            ],
        )?)?;
        let entry = filters
            .iter()
            .find(|f| f.pointer("/options/handle").and_then(Value::as_u64) == Some(1))
            .ok_or("missing packet filter")?;
        let stats = entry
            .pointer("/options/actions/0/stats")
            .ok_or("missing filter statistics")?;
        Ok(
            json!({"direction":self.direction,"bytes":stats["bytes"],"packets":stats["packets"],"drops":stats["drops"]}),
        )
    }

    fn remove(&mut self) -> Result<()> {
        if self.installed {
            tc(
                self.container,
                &[
                    "filter",
                    "del",
                    "dev",
                    "eth0",
                    self.direction,
                    "protocol",
                    "ip",
                    "pref",
                    PREF,
                    "handle",
                    "1",
                    "flower",
                ],
            )?;
            self.installed = false;
        }
        Ok(())
    }
}

impl Drop for Filter<'_> {
    fn drop(&mut self) {
        if let Err(error) = self.remove() {
            eprintln!("failed to remove {} test filter: {error}", self.direction);
        }
    }
}

fn lease_free(lease: &File) -> Result<bool> {
    match lease.try_lock() {
        Ok(()) => {
            lease.unlock()?;
            Ok(true)
        }
        Err(TryLockError::WouldBlock) => Ok(false),
        Err(TryLockError::Error(e)) => Err(e.into()),
    }
}

fn ready(path: &Path) -> Result<Value> {
    let start = Instant::now();
    loop {
        if path.try_exists()? {
            match serde_json::from_slice(&fs::read(path)?) {
                Ok(v) => return Ok(v),
                Err(e) if e.is_eof() => (),
                Err(e) => return Err(e.into()),
            }
        }
        if start.elapsed() > DEADLINE {
            return Err("missing driver fault cut".into());
        }
        thread::sleep(Duration::from_millis(20));
    }
}

fn main() -> Result<()> {
    let a = Args::parse();
    let network = Command::new("docker")
        .args([
            "inspect",
            "--type",
            "container",
            "--format",
            "{{.HostConfig.NetworkMode}}",
            "--",
            &a.container,
        ])
        .output()?;
    let mode = std::str::from_utf8(&network.stdout)?.trim();
    if !network.status.success()
        || mode.is_empty()
        || matches!(mode, "host" | "none")
        || mode.starts_with("container:")
    {
        return Err("packet loss requires a dedicated worker network namespace".into());
    }
    let cut = ready(&a.trial.join("fault-ready.json"))?;
    if cut["slot"].as_u64() != Some(a.slot as u64) {
        return Err("cut targets a different slot".into());
    }
    let lease = File::open(&a.lease)?;
    if lease_free(&lease)? {
        return Err("target lease must be held before injection".into());
    }
    let dirs: &[&str] = match a.direction {
        Direction::Up => &["ingress"],
        Direction::Down => &["egress"],
        Direction::Both => &["ingress", "egress"],
    };
    let mut filters = Vec::new();
    let start = Instant::now();
    for direction in dirs {
        let mut filter = Filter {
            container: &a.container,
            direction,
            installed: false,
        };
        filter.install(&a.ssh_port.to_string())?;
        filters.push(filter);
    }
    let all_filters_active_s = start.elapsed().as_secs_f64();
    fs::write(a.trial.join("fault-applied"), [])?;
    let mut first_free = None;
    let mut last_held = 0.;
    let transient = a.expect == Expect::Transient;
    let timed = transient || a.expect == Expect::Automatic;
    let witness = if a.expect == Expect::Automatic {
        let session = Path::new(cut["session"].as_str().ok_or("missing session path")?);
        let archive = session.join("barrier.gzmultisphere");
        let bytes = fs::read(&archive)?;
        use sha2::{Digest, Sha256};
        if cut["sha256"] != format!("{:x}", Sha256::digest(&bytes)) {
            return Err("starting archive differs from the queued input cut".into());
        }
        Some((File::open(session.join("controller.lock"))?, archive, bytes))
    } else {
        None
    };
    let mut checkpoint_samples = 0u32;
    let fenced = a.trial.join("fenced.json");
    loop {
        if lease_free(&lease)? {
            first_free.get_or_insert(start.elapsed().as_secs_f64());
        } else if first_free.is_none() {
            last_held = start.elapsed().as_secs_f64();
        }
        if let Some((controller, path, saved)) = &witness {
            if lease_free(controller)? || fs::read(path)? != *saved {
                return Err("writer lease or published checkpoint changed during partition".into());
            }
            checkpoint_samples += 1;
        }
        if timed && start.elapsed().as_secs_f64() >= all_filters_active_s + a.hold_ms as f64 / 1000.
        {
            break;
        }
        if !timed && fenced.try_exists()? && (a.expect == Expect::Held || first_free.is_some()) {
            break;
        }
        if start.elapsed() > DEADLINE {
            return Err("timed out waiting for fence or release".into());
        }
        thread::sleep(Duration::from_millis(20));
    }
    let counts = filters
        .iter()
        .map(Filter::counts)
        .collect::<Result<Vec<_>>>()?;
    // Reading kernel counters uses a separate management connection. Sample the
    // lease again immediately before healing so that time is part of the cut.
    let free_at_restore = lease_free(&lease)?;
    let restore_s = start.elapsed().as_secs_f64();
    if free_at_restore {
        first_free.get_or_insert(restore_s);
    } else if first_free.is_none() {
        last_held = restore_s;
    }
    for filter in &mut filters {
        filter.remove()?;
    }
    let healed = Instant::now();
    if !timed {
        let restored = Instant::now();
        while !lease_free(&lease)? {
            if restored.elapsed() > DEADLINE {
                return Err("lease still held after link restoration".into());
            }
            thread::sleep(Duration::from_millis(20));
        }
        fs::write(a.trial.join("link-restored"), [])?;
    }
    let link_to_completion_s = if a.expect == Expect::Automatic {
        while !a.trial.join("advance-completed").try_exists()? {
            if healed.elapsed() > DEADLINE {
                return Err("automatic advance never completed".into());
            }
            thread::sleep(Duration::from_millis(5));
        }
        Some(healed.elapsed().as_secs_f64())
    } else {
        None
    };
    let expected_lease = first_free.is_some()
        == matches!(a.expect, Expect::Release | Expect::Automatic)
        && free_at_restore == first_free.is_some();
    let packets_dropped = counts
        .iter()
        .all(|v| v["drops"].as_u64().is_some_and(|n| n > 0));
    fs::write(
        a.trial.join("packet-loss.json"),
        serde_json::to_vec_pretty(&json!({
            "direction":format!("{:?}",a.direction),"expect":format!("{:?}",a.expect),"slot":a.slot,
            "all_filters_active_seconds":all_filters_active_s,"last_observed_held_before_release_seconds":last_held,
            "first_observed_free_during_cut_seconds":first_free,"started_restoring_filters_seconds":restore_s,
            "lease_free_at_restoration":free_at_restore, "full_cut_seconds":restore_s-all_filters_active_s,
            "seconds_from_full_cut_to_first_free":first_free.map(|t|t-all_filters_active_s),
            "samples_with_unchanged_locked_archive":checkpoint_samples,
            "seconds_from_filter_removal_to_observed_completion":link_to_completion_s,
            "counters":counts,"expected_lease_observed":expected_lease,"packets_dropped":packets_dropped
        }))?,
    )?;
    if !expected_lease || !packets_dropped {
        return Err("packet or lease observation did not match trial".into());
    }
    Ok(())
}
