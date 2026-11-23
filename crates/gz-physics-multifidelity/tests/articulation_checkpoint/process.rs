// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{
    env, fs,
    io::{Read, Write},
    os::unix::process::ExitStatusExt,
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    time::{Instant, SystemTime},
};

const SEGMENTS: usize = 32;
const TICKS: usize = 32;
const DIRECTORY: &str = "GZ_ARTICULATION_NATIVE_TEST_DIR";
const SEGMENT: &str = "GZ_ARTICULATION_NATIVE_TEST_SEGMENT";

fn fixture() -> Result<(MultiFidelityBackend, [BodyId; 2], JointId)> {
    let unit = 14f64.sqrt().recip();
    build_with_axis([unit, 2. * unit, 3. * unit], true)
}
fn advance(b: &mut MultiFidelityBackend, j: JointId, segment: usize) -> Result<(Vec<u8>, Vec<u8>)> {
    let mut states = Vec::new();
    let mut cut = Vec::new();
    for local in 0..TICKS {
        let tick = segment * TICKS + local;
        if tick % 13 < 3 {
            b.add_joint_effort(j, 0.5625)?;
        }
        b.step(DT)?;
        if local == TICKS - 1 {
            let torque = if segment.is_multiple_of(2) {
                -0.4375
            } else {
                0.875
            };
            b.add_joint_effort(j, torque)?;
        }
        cut = b.checkpoint_floating_revolute()?;
        states.extend_from_slice(&(cut.len() as u16).to_le_bytes());
        states.extend_from_slice(&cut);
    }
    Ok((states, cut))
}
fn file(dir: &Path, segment: usize, kind: &str) -> PathBuf {
    dir.join(format!("{segment}.{kind}"))
}
fn save(path: &Path, data: &[u8]) -> Result<()> {
    let mut f = fs::File::create_new(path)?;
    f.write_all(data)?;
    f.sync_all()?;
    Ok(())
}

#[test]
#[ignore = "private subprocess role for the real process-death regression"]
fn native_child() -> Result<()> {
    let Some(dir) = env::var_os(DIRECTORY).map(PathBuf::from) else {
        return Ok(());
    };
    let segment = env::var(SEGMENT)?.parse::<usize>()?;
    let (mut b, _, j) = fixture()?;
    if segment != 0 {
        b.restore_floating_revolute(&fs::read(file(&dir, segment - 1, "cut"))?)?;
    }
    let (states, cut) = advance(&mut b, j, segment)?;
    save(&file(&dir, segment, "states"), &states)?;
    save(&file(&dir, segment, "cut"), &cut)?;
    save(&file(&dir, segment, "ready"), &[])?;
    // Exit on a lost harness so a failed parent cannot orphan this test process.
    std::io::stdin().lock().read_to_end(&mut Vec::new())?;
    Ok(())
}

struct Scratch(PathBuf);
impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}
struct Running(Child);
impl Drop for Running {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}
#[test]
fn hard_process_loss_resumes_from_prior_held_effort_across_32_cuts() -> Result<()> {
    let stamp = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)?
        .as_nanos();
    let dir = env::temp_dir().join(format!("gz-hinge-{}-{stamp}", std::process::id()));
    fs::create_dir(&dir)?;
    let scratch = Scratch(dir);
    let (mut control, _, j) = fixture()?;
    for segment in 0..SEGMENTS {
        let mut child = Running(
            Command::new(env::current_exe()?)
                .args([
                    "--exact",
                    "process::native_child",
                    "--ignored",
                    "--nocapture",
                ])
                .env(DIRECTORY, &scratch.0)
                .env(SEGMENT, segment.to_string())
                .stdin(Stdio::piped())
                .stdout(Stdio::null())
                .spawn()?,
        );
        let (states, cut) = advance(&mut control, j, segment)?;
        let deadline = Instant::now() + Duration::from_secs(20);
        while !file(&scratch.0, segment, "ready").exists() {
            if let Some(status) = child.0.try_wait()? {
                return Err(
                    format!("native process {segment} exited before sealing: {status}").into(),
                );
            }
            if Instant::now() >= deadline {
                return Err("native checkpoint child timed out".into());
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        child.0.kill()?;
        assert_eq!(child.0.wait()?.signal(), Some(9));
        assert_eq!(fs::read(file(&scratch.0, segment, "cut"))?, cut);
        assert_eq!(fs::read(file(&scratch.0, segment, "states"))?, states);
    }
    println!(
        "{SEGMENTS} independently created and killed native processes, 31 fresh file resumes, 1024 complete bit-identical native states, every cut with held torque"
    );
    Ok(())
}
