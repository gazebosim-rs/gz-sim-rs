// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Length-prefixed local CPU/GPU worker protocol for the partition qualification example.
use super::super::Result;
use gz_physics_gpu::{PairBlockAssignment, PairBlockContinuation, PairBlockExecutor};
use std::{
    io::{self, BufRead, BufReader, Read, Write},
    path::Path,
    process::{Child, ChildStdin, ChildStdout, Command, Stdio},
};
pub(super) struct Process {
    child: Child,
    pub(super) input: ChildStdin,
    pub(super) output: BufReader<ChildStdout>,
}
impl Process {
    pub(super) fn start(mode: &str, adapter: &str, dir: &Path) -> Result<Self> {
        let mut child = Command::new(std::env::current_exe()?)
            .args([mode, adapter])
            .arg(dir)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        Ok(Self {
            input: child.stdin.take().unwrap(),
            output: BufReader::new(child.stdout.take().unwrap()),
            child,
        })
    }
    pub(super) fn line(&mut self) -> Result<String> {
        let mut s = String::new();
        if self.output.by_ref().take(16 * 1024).read_line(&mut s)? == 0 || !s.ends_with('\n') {
            return Err("missing complete coordinator acknowledgement".into());
        }
        Ok(s.trim_end().into())
    }
    pub(super) fn kill(&mut self) -> Result<()> {
        self.child.kill()?;
        if self.child.wait()?.success() {
            return Err("worker exited normally at forced death cut".into());
        }
        Ok(())
    }
    pub(super) fn send(&mut self, job: &[u8], hold: bool) -> Result<()> {
        self.input.write_all(&[u8::from(hold)])?;
        put_frame(&mut self.input, job)?;
        self.input.flush()?;
        Ok(())
    }
}
impl Drop for Process {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}
pub(super) fn get_number(r: &mut impl Read) -> Result<u64> {
    let mut b = [0; 8];
    r.read_exact(&mut b)?;
    Ok(u64::from_le_bytes(b))
}
pub(super) fn put_frame(w: &mut impl Write, b: &[u8]) -> Result<()> {
    w.write_all(&(b.len() as u64).to_le_bytes())?;
    w.write_all(b)?;
    Ok(())
}
pub(super) fn get_frame(r: &mut impl Read) -> Result<Vec<u8>> {
    let len = usize::try_from(get_number(r)?)?;
    if len > PairBlockAssignment::MAX_BYTES {
        return Err("oversized local pair worker frame".into());
    }
    let mut b = vec![0; len];
    r.read_exact(&mut b)?;
    Ok(b)
}
pub(crate) fn worker(adapter: &str) -> Result<()> {
    let mut solver = PairBlockExecutor::new(adapter);
    let mut input = io::stdin().lock();
    let mut out = io::BufWriter::new(io::stdout().lock());
    loop {
        let mut hold = [0];
        if input.read(&mut hold)? == 0 {
            return Ok(());
        }
        let data = get_frame(&mut input)?;
        let (receipt, metrics, reused) = if data.len() == PairBlockContinuation::BYTES {
            let cue = PairBlockContinuation::from_bytes(&data)?;
            let Some((r, m)) = solver.try_continue(cue)? else {
                out.write_all(b"M")?;
                out.flush()?;
                continue;
            };
            (r, m, true)
        } else {
            solver.execute(PairBlockAssignment::from_bytes(&data)?)?
        };
        if hold[0] == 1 {
            out.write_all(b"H")?;
            out.flush()?;
            // This worker never acknowledges. Its controller kills it, or EOF
            // ends it if the owner disappears at the same boundary.
            let mut b = [0];
            let _ = input.read(&mut b)?;
            return Ok(());
        }
        out.write_all(b"R")?;
        for n in [
            u64::from(reused),
            metrics.uploaded_bytes,
            metrics.readback_bytes,
            metrics.gpu_dispatch.unwrap_or_default().as_nanos() as u64,
        ] {
            out.write_all(&n.to_le_bytes())?;
        }
        put_frame(&mut out, &receipt.to_bytes()?)?;
        out.flush()?;
    }
}
