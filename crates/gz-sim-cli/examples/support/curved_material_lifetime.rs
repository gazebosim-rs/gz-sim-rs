// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Kill a complete SDF controller with active and stopped workers, including its idle slots.
//! Linux pidfds pin every owned test process until the comparison or explicit cleanup completes.
use super::*;
#[cfg(target_os = "linux")]
mod linux {
    use super::*;
    use rustix::{
        event::{PollFd, PollFlags, Timespec, poll},
        process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal},
    };
    use std::{
        os::fd::OwnedFd,
        process::Command,
        time::{Duration, Instant},
    };
    struct Watch {
        pid: u32,
        fd: OwnedFd,
    }
    impl Watch {
        fn exited(&self) -> Result<bool> {
            let mut p = [PollFd::new(&self.fd, PollFlags::IN)];
            Ok(poll(&mut p, Some(&Timespec::default()))? != 0)
        }
        fn signal(&self, s: Signal) -> Result<()> {
            Ok(pidfd_send_signal(&self.fd, s)?)
        }
    }
    impl Drop for Watch {
        fn drop(&mut self) {
            let _ = pidfd_send_signal(&self.fd, Signal::KILL);
        }
    }
    fn solver_children(server: u32, worker: &Path) -> Result<Vec<Watch>> {
        let target = worker.canonicalize()?;
        let mut found = std::collections::BTreeMap::new();
        for task in fs::read_dir(format!("/proc/{server}/task"))? {
            let Ok(list) = fs::read_to_string(task?.path().join("children")) else {
                continue;
            };
            for item in list.split_whitespace() {
                let pid = item.parse::<u32>()?;
                if found.contains_key(&pid) {
                    continue;
                }
                let Some(p) = Pid::from_raw(pid as i32) else {
                    continue;
                };
                let Ok(fd) = pidfd_open(p, PidfdFlags::empty()) else {
                    continue;
                };
                let dir = PathBuf::from(format!("/proc/{pid}"));
                if fs::read_link(dir.join("exe")).as_ref().ok() != Some(&target) {
                    continue;
                }
                let cmd = fs::read(dir.join("cmdline"))?;
                let args = cmd.split(|b| *b == 0).collect::<Vec<_>>();
                if args.get(1) == Some(&b"--managed-trajectory-material-v1".as_slice()) {
                    found.insert(pid, Watch { pid, fd });
                }
            }
        }
        Ok(found.into_values().collect())
    }
    fn child(
        exe: &Path,
        sdf: &Path,
        worker: &Path,
        dir: &Path,
        cut: Option<&Path>,
    ) -> Result<Actor> {
        let mut cmd = Command::new(exe);
        cmd.env("TMPDIR", dir)
            .arg("partition-child")
            .arg(sdf)
            .arg(worker);
        if let Some(c) = cut {
            cmd.arg(c);
        }
        Actor::from_command(cmd)
    }
    pub(super) fn stock(
        exe: &Path,
        worker: &Path,
        sdf: &Path,
        cut: &Path,
        port: u16,
        reader: &Path,
        reference: &Path,
    ) -> Result<()> {
        use rustix::process::kill_process;
        use std::process::Stdio;
        if cut.exists() {
            return Err("the stock witness needs a new immutable capture".into());
        }
        let mut child = Command::new(exe)
            .arg(sdf)
            .args(["--physics-engine", "multifidelity", "--physics-worker"])
            .arg(worker)
            .arg("--live-pool-checkpoint")
            .arg(cut)
            .args(["--server", "--web-lifecycle", "--web-viewer"])
            .arg(port.to_string())
            .stdout(Stdio::null())
            .spawn()?;
        let run = (|| -> Result<Value> {
            let until = Instant::now() + Duration::from_secs(15);
            while std::net::TcpStream::connect(("127.0.0.1", port)).is_err() {
                if Instant::now() >= until || child.try_wait()?.is_some() {
                    return Err("stock server did not start".into());
                }
                std::thread::sleep(Duration::from_millis(50));
            }
            let proof = Command::new(reader)
                .arg(reference)
                .arg(port.to_string())
                .args(["--until-ns", "90000000"])
                .output()?;
            if !proof.status.success() {
                return Err(String::from_utf8_lossy(&proof.stderr).into_owned().into());
            }
            let p = Pid::from_raw(child.id() as i32).ok_or("missing stock controller PID")?;
            kill_process(p, Signal::USR1)?;
            let until = Instant::now() + Duration::from_secs(8);
            while !cut.is_file() {
                if Instant::now() >= until {
                    return Err("stock checkpoint was not published".into());
                }
                std::thread::sleep(Duration::from_millis(20));
            }
            let restored = checkpoint::open(cut, worker)?;
            let i = restored.committed_update_info()?;
            let saved = partitions::state(&restored)?;
            let mut local = setup(&fs::read_to_string(sdf)?, None)?;
            for _ in 0..i.epoch {
                reset_loaded_world(&mut local, |_| ())?;
            }
            // The stock server samples IMUs only when their transport topics have subscribers.
            // This operator has a gz-web pose client and zero IMU subscriptions.
            gz_sim_scene::set_imu_interest(&mut local, std::iter::empty());
            local.step_exact(i.iteration)?;
            equal(&observation(&restored)?, &observation(&local)?)?;
            equal(&saved, &partitions::state(&local)?)?;
            drop(restored);
            let watches = solver_children(child.id(), worker)?;
            if watches.len() < 2 {
                return Err("stock expected multiple independent worker PIDs".into());
            }
            for w in &watches {
                w.signal(Signal::STOP)?;
            }
            let until = Instant::now() + Duration::from_secs(2);
            for w in &watches {
                while !fs::read_to_string(format!("/proc/{}/status", w.pid))?
                    .lines()
                    .any(|s| s.starts_with("State:") && s.contains("T (stopped)"))
                {
                    if Instant::now() >= until {
                        return Err("expected stopped stock worker".into());
                    }
                    std::thread::yield_now();
                }
            }
            let start = Instant::now();
            child.kill()?;
            let status = child.wait()?;
            use std::os::unix::process::ExitStatusExt;
            if status.signal() != Some(9) {
                return Err("stock operator was not killed".into());
            }
            let until = start + Duration::from_secs(1);
            while Instant::now() < until && !watches.iter().all(|w| w.exited().unwrap_or(false)) {
                std::thread::sleep(Duration::from_millis(2));
            }
            for w in &watches {
                if !w.exited()? {
                    return Err("stopped worker survived lost stock controller".into());
                }
            }
            Ok(
                json!({"worker_pids":watches.iter().map(|w|w.pid).collect::<Vec<_>>(),"stopped_workers":watches.len(),"controller_sigkill":true,
                "kernel_retirement_us":start.elapsed().as_micros(),"saved_time_ns":i.sim_time.as_nanos(),"tick":i.iteration,"epoch":i.epoch,
                "exact_material_and_native":saved,"live_reader":String::from_utf8(proof.stdout)?}),
            )
        })();
        let _ = child.kill();
        let _ = child.wait();
        println!("{}", run?);
        Ok(())
    }
    pub(super) fn inflight(
        worker: &Path,
        sdf: &Path,
        scratch: &Path,
        published: &Path,
    ) -> Result<()> {
        let exe = std::env::current_exe()?;
        if published.exists() {
            return Err("a new checkpoint destination is required".into());
        }
        let outer = tempfile::Builder::new()
            .prefix("gz-pending-cut-")
            .tempdir_in(scratch)?;
        let text = fs::read_to_string(sdf)?;
        let mut cases = Vec::new();
        for (case, modes) in [
            vec!["before"],
            vec!["solved"],
            vec!["partial"],
            vec!["partial", "solved"],
            vec!["partial", "partial"],
        ]
        .into_iter()
        .enumerate()
        {
            let dir = outer.path().join(case.to_string());
            fs::create_dir(&dir)?;
            let mut spec = Command::new(&exe);
            spec.arg("partition-child")
                .arg(sdf)
                .arg(&exe)
                .env("TMPDIR", &dir)
                .env("GZ_EXAMPLE_BREAKPOINTS", &dir)
                .env("GZ_EXAMPLE_REAL_WORKER", worker.canonicalize()?);
            let mut owner = Actor::from_command(spec)?;
            owner.read()?;
            let mut serial = setup(&text, None)?;
            let mut latest = Value::Null;
            let mut checks = 0;
            for i in 0..48 {
                if i == 32 && case >= 3 {
                    owner.call("move 0")?;
                    owner.call("move 1")?;
                }
                latest = owner.call("advance")?;
                serial.step_exact(1)?;
                equal(&latest["state"], &partitions::state(&serial)?)?;
                checks += 1;
            }
            let ids: Vec<u32> = serde_json::from_value(latest["pids"].clone())?;
            if ids.len() != 2
                || !latest["state"]["loads"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|x| x.as_f64().unwrap() > 0.)
            {
                return Err("need two loaded packed groups".into());
            }
            let saved = dir.join("retained.gzcurve");
            let receipt = owner.call(&format!("park-save {}", saved.display()))?;
            equal(&receipt["state"], &latest["state"])?;
            let original = fs::read(&saved)?;
            let relays = solver_children(owner.id(), &exe)?;
            let mut workers = Vec::new();
            for relay in &relays {
                workers.extend(solver_children(relay.pid, worker)?);
            }
            if relays.len() != workers.len() || relays.len() != if case >= 3 { 4 } else { 2 } {
                return Err("unexpected resident relay/solver population".into());
            }
            for (&pid, mode) in ids.iter().zip(&modes) {
                fs::write(dir.join(format!("hold-{pid}")), format!("49 {mode}"))?;
            }
            owner.begin("advance")?;
            let mut proofs = Vec::new();
            for (&pid, &mode) in ids.iter().zip(&modes) {
                let file = dir.join(format!("ready-{pid}"));
                let until = Instant::now() + Duration::from_secs(15);
                while !file.is_file() {
                    if Instant::now() > until {
                        return Err(format!("missing held group {pid} at {mode}").into());
                    }
                    std::thread::sleep(Duration::from_millis(2));
                }
                let proof: Value = serde_json::from_slice(&fs::read(file)?)?;
                if proof["tick"] != 49
                    || proof["site"] != mode
                    || !workers.iter().any(|w| proof["worker"] == w.pid)
                {
                    return Err("physical loss marker does not match lease".into());
                }
                proofs.push(proof);
            }
            // A partial pipe frame and a solved-but-unacknowledged peer coexist at this boundary.
            if case == 4 {
                for w in workers.iter().chain(&relays) {
                    w.signal(Signal::STOP)?;
                }
                let until = Instant::now() + Duration::from_secs(2);
                for w in workers.iter().chain(&relays) {
                    while !fs::read_to_string(format!("/proc/{}/status", w.pid))?
                        .lines()
                        .any(|s| s.starts_with("State:") && s.contains("T (stopped)"))
                    {
                        if Instant::now() >= until {
                            return Err("target worker hierarchy was not stopped".into());
                        }
                        std::thread::yield_now();
                    }
                }
            }
            let start = Instant::now();
            owner.abort()?;
            let until = start + Duration::from_secs(2);
            while Instant::now() < until
                && !workers
                    .iter()
                    .chain(&relays)
                    .all(|w| w.exited().unwrap_or(false))
            {
                std::thread::sleep(Duration::from_millis(2));
            }
            for w in workers.iter().chain(&relays) {
                if !w.exited()? {
                    return Err("a worker survived the interrupted controller".into());
                }
            }
            let elapsed = start.elapsed();
            if original != fs::read(&saved)? {
                return Err("in-flight exchange changed the retained checkpoint".into());
            }
            let mut restored = partitions::child(sdf, worker, &dir, Some(&saved))?;
            equal(&restored.read()?, &receipt)?;
            checks += 1;
            let mut digest = Sha256::new();
            for _ in 0..40 {
                serial.step_exact(1)?;
                let expected = partitions::state(&serial)?;
                equal(&restored.call("advance")?["state"], &expected)?;
                digest.update(expected["hash"].as_str().unwrap());
                checks += 1;
            }
            if case == 4 {
                let mut file = std::fs::OpenOptions::new()
                    .write(true)
                    .create_new(true)
                    .open(published)?;
                file.write_all(&original)?;
                file.sync_all()?;
            }
            cases.push(json!({"sites":proofs,"relays":relays.len(),"solvers":workers.len(),"all_stopped":case==4,
                "retirement_us":elapsed.as_micros(),"complete_comparisons":checks,"future_40_sha256":format!("{:x}",digest.finalize()),
                "retained_sha256":format!("{:x}",Sha256::digest(original)),"public_loads":receipt["state"]["loads"]}));
        }
        println!(
            "{}",
            json!({"controller_sigkills":cases.len(),"cases":cases})
        );
        Ok(())
    }
    pub(super) fn run(
        exe: &Path,
        worker: &Path,
        sdf: &Path,
        scratch: &Path,
        guarded: bool,
    ) -> Result<()> {
        let dir = tempfile::Builder::new()
            .prefix("gz-stopped-owner-")
            .tempdir_in(scratch)?;
        let mut results = Vec::new();
        for (case, name) in [
            "running",
            "one-stopped",
            "all-stopped",
            "idle-stopped",
            "all-eight-stopped",
        ]
        .into_iter()
        .enumerate()
        {
            let mut owner = child(exe, sdf, worker, dir.path(), None)?;
            let init = owner.read()?;
            let groups = init["owners"]
                .as_array()
                .ok_or("missing physical placement")?
                .len();
            if groups < 2 {
                return Err("need two or more independent groups".into());
            }
            let mut local = setup(&fs::read_to_string(sdf)?, None)?;
            let mut wanted = partitions::state(&local)?;
            let mut active = Vec::new();
            for t in 0..48 {
                if t == 32 && case >= 3 {
                    for g in 0..groups {
                        owner.call(&format!("move {g}"))?;
                    }
                }
                let r = owner.call("advance")?;
                active = serde_json::from_value::<Vec<u32>>(r["pids"].clone())?;
                local.step_exact(1)?;
                wanted = partitions::state(&local)?;
                equal(&r["state"], &wanted)?;
            }
            let cut = dir.path().join(format!("committed-{case}"));
            let saved = owner.call(&format!("save {}", cut.display()))?;
            equal(&saved["state"], &wanted)?;
            if !wanted["loads"]
                .as_array()
                .unwrap()
                .iter()
                .all(|v| v.as_f64().unwrap() > 0.)
            {
                return Err("expected a fully loaded source".into());
            }
            let watches = solver_children(owner.id(), worker)?;
            if watches.len() != groups * (if case >= 3 { 2 } else { 1 }) {
                return Err("unexpected resident lease population".into());
            }
            let frozen = |pid| match case {
                0 => false,
                1 => pid == active[0],
                2 | 4 => true,
                3 => !active.contains(&pid),
                _ => false,
            };
            for w in &watches {
                if frozen(w.pid) {
                    w.signal(Signal::STOP)?;
                }
            }
            // Confirm all selected children actually stopped before killing the controller.
            let until = Instant::now() + Duration::from_secs(2);
            for w in &watches {
                if frozen(w.pid) {
                    loop {
                        let state = fs::read_to_string(format!("/proc/{}/status", w.pid))?;
                        if state
                            .lines()
                            .any(|s| s.starts_with("State:") && s.contains("T (stopped)"))
                        {
                            break;
                        }
                        if Instant::now() > until {
                            return Err("worker did not enter stopped state".into());
                        }
                        std::thread::yield_now();
                    }
                }
            }
            let start = Instant::now();
            owner.abort()?;
            let until = start + Duration::from_millis(900);
            while Instant::now() < until && !watches.iter().all(|w| w.exited().unwrap_or(false)) {
                std::thread::sleep(Duration::from_millis(2));
            }
            let observed = start.elapsed();
            let mut survivors = Vec::new();
            let mut expected = Vec::new();
            let mut stopped = 0;
            for w in &watches {
                if !w.exited()? {
                    survivors.push(w.pid);
                }
                if frozen(w.pid) {
                    stopped += 1;
                    if !guarded {
                        expected.push(w.pid);
                    }
                }
            }
            // Cleanup uses the pre-opened owned pidfds even on a failed expectation.
            for w in &watches {
                if !w.exited()? {
                    w.signal(Signal::KILL)?;
                }
            }
            if survivors != expected {
                return Err(format!(
                    "unexpected stopped leases for {name}: {survivors:?} vs {expected:?}"
                )
                .into());
            }
            let mut restored = child(exe, sdf, worker, dir.path(), Some(&cut))?;
            equal(&restored.read()?, &saved)?;
            let mut digest = Sha256::new();
            for _ in 0..32 {
                local.step_exact(1)?;
                let want = partitions::state(&local)?;
                equal(&restored.call("advance")?["state"], &want)?;
                digest.update(want["hash"].as_str().unwrap());
            }
            results.push(json!({"case":name,"owned_workers":watches.len(),"stopped":stopped,"survivors_after_controller_death":survivors.len(),
                "retirement_observed_us":observed.as_micros(),"full_cuts_compared":81,"next_32_sha256":format!("{:x}",digest.finalize())}));
        }
        println!(
            "{}",
            json!({"kernel_parent_death_expected":guarded,"cases":results})
        );
        Ok(())
    }
}
pub(super) fn run(
    exe: &Path,
    worker: &Path,
    sdf: &Path,
    scratch: &Path,
    guarded: bool,
) -> Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::run(exe, worker, sdf, scratch, guarded)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (exe, worker, sdf, scratch, guarded);
        Err("stopped controller comparison requires Linux pidfds".into())
    }
}

pub(super) fn stock(
    exe: &Path,
    worker: &Path,
    sdf: &Path,
    cut: &Path,
    port: u16,
    reader: &Path,
    reference: &Path,
) -> Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::stock(exe, worker, sdf, cut, port, reader, reference)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (exe, worker, sdf, cut, port, reader, reference);
        Err("stock pidfd witness requires Linux".into())
    }
}

pub(super) fn inflight(worker: &Path, sdf: &Path, scratch: &Path, published: &Path) -> Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::inflight(worker, sdf, scratch, published)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (worker, sdf, scratch, published);
        Err("in-flight loss witness requires Linux pidfds".into())
    }
}
