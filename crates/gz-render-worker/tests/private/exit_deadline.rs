// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{
    os::unix::process::ExitStatusExt,
    process::{Child, Command, Stdio},
};

struct LiveChild(Option<Child>);
impl LiveChild {
    fn new() -> Self {
        Self(Some(
            Command::new("/bin/sleep")
                .arg("300")
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .spawn()
                .unwrap(),
        ))
    }
    fn pid(&mut self) -> Pid {
        let child = self.0.as_mut().unwrap();
        assert!(child.try_wait().unwrap().is_none());
        Pid::from_raw(i32::try_from(child.id()).unwrap()).unwrap()
    }
    fn reap_killed(&mut self) {
        let deadline = Instant::now() + Duration::from_secs(10);
        let status = loop {
            if let Some(status) = self.0.as_mut().unwrap().try_wait().unwrap() {
                break status;
            }
            assert!(
                Instant::now() < deadline,
                "owned signal target did not exit"
            );
            thread::sleep(Duration::from_millis(5));
        };
        assert_eq!(status.signal(), Some(Signal::KILL.as_raw()));
        self.0.take();
    }
}
impl Drop for LiveChild {
    fn drop(&mut self) {
        if let Some(mut child) = self.0.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

#[test]
fn idle_guard_cancels_and_joins_without_signaling_target() {
    let mut target = LiveChild::new();
    let guard = ExitDeadline::new(target.pid()).unwrap();
    drop(guard);
    assert!(target.0.as_mut().unwrap().try_wait().unwrap().is_none());
}

#[test]
fn elapsed_absolute_deadline_signals_owned_live_child_and_joins() {
    let mut target = LiveChild::new();
    let guard = ExitDeadline::new(target.pid()).unwrap();
    guard
        .trigger
        .as_ref()
        .unwrap()
        .send(Instant::now() - Duration::from_secs(1))
        .unwrap();
    target.reap_killed();
    drop(guard);
}

#[test]
fn armed_deadline_kills_owned_child_within_the_original_exit_window() {
    let mut target = LiveChild::new();
    let guard = ExitDeadline::new(target.pid()).unwrap();
    guard.arm();
    target.reap_killed();
    drop(guard);
}

#[test]
fn failed_handoffs_signal_only_owned_live_targets() {
    for closed in [true, false] {
        let mut target = LiveChild::new();
        let (sender, receiver) = mpsc::sync_channel(1);
        let receiver = if closed {
            drop(receiver);
            None
        } else {
            sender.try_send(Instant::now()).unwrap();
            Some(receiver)
        };
        signal_deadline(&sender, target.pid(), Instant::now());
        target.reap_killed();
        drop(receiver);
    }
}

#[cfg(target_os = "linux")]
#[test]
fn thread_admission_failure_is_reported_without_signaling_target() {
    if std::env::var_os("GZ_RENDER_EXIT_THREAD_QUOTA_CHILD").is_some() {
        use rustix::process::{Resource, Rlimit, setrlimit};
        let mut target = LiveChild::new();
        let pid = target.pid();
        setrlimit(
            Resource::Nproc,
            Rlimit {
                current: Some(1),
                maximum: Some(1),
            },
        )
        .unwrap();
        assert!(ExitDeadline::new(pid).is_err());
        assert!(target.0.as_mut().unwrap().try_wait().unwrap().is_none());
        return;
    }
    let output = Command::new("unshare")
        .args(["-Ur"])
        .arg(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "exit_deadline::tests::thread_admission_failure_is_reported_without_signaling_target",
            "--nocapture",
        ])
        .env("GZ_RENDER_EXIT_THREAD_QUOTA_CHILD", "1")
        .output()
        .unwrap();
    assert!(output.status.success(), "{output:?}");
}
