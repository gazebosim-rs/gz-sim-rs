// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Test client for controlling and terminating one complete ordinary SDF simulator.
use serde_json::Value;
use std::{
    error::Error,
    io::{BufRead, BufReader, Write},
    path::Path,
    process::{Child, ChildStdin, ChildStdout, Command, Stdio},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
pub struct Actor {
    child: Child,
    input: ChildStdin,
    output: BufReader<ChildStdout>,
}
impl Actor {
    pub fn new(worker: &Path, dir: &Path, seed: u64, cut: Option<&str>) -> Result<Self> {
        let mut spec = Command::new(std::env::current_exe()?);
        spec.env("TMPDIR", dir)
            .arg("actor")
            .arg(worker)
            .arg(dir)
            .arg(seed.to_string());
        if let Some(f) = cut {
            spec.arg(f);
        }
        Self::from_command(spec)
    }
    pub fn from_command(mut spec: Command) -> Result<Self> {
        let mut child = spec.stdin(Stdio::piped()).stdout(Stdio::piped()).spawn()?;
        let input = child.stdin.take().unwrap();
        let output = BufReader::new(child.stdout.take().unwrap());
        Ok(Self {
            child,
            input,
            output,
        })
    }
    #[allow(dead_code)] // Only Linux process-lifetime witnesses inspect the controller PID.
    pub fn id(&self) -> u32 {
        self.child.id()
    }
    pub fn abort(mut self) -> Result<()> {
        self.child.kill()?;
        let status = self.child.wait()?;
        #[cfg(unix)]
        if std::os::unix::process::ExitStatusExt::signal(&status) != Some(9) {
            return Err("controller was not killed by SIGKILL".into());
        }
        #[cfg(not(unix))]
        if status.success() {
            return Err("controller continued after termination".into());
        }
        Ok(())
    }
    pub fn read(&mut self) -> Result<Value> {
        let mut s = String::new();
        if self.output.read_line(&mut s)? == 0 {
            return Err("controller exited before its reply".into());
        }
        Ok(serde_json::from_str(&s)?)
    }
    pub fn begin(&mut self, cmd: &str) -> Result<()> {
        writeln!(self.input, "{cmd}")?;
        self.input.flush()?;
        Ok(())
    }
    pub fn call(&mut self, cmd: &str) -> Result<Value> {
        self.begin(cmd)?;
        self.read()
    }
}
impl Drop for Actor {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}
