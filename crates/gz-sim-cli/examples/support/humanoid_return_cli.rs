// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, Simulation};
use gz_sim_scene::{EntityKind, Name, WorldPose};
use std::{collections::BTreeMap, ffi::OsString, fmt::Write, fs, process::Command};

pub(super) struct RobotCli {
    executable: OsString,
    directory: tempfile::TempDir,
    checkpoint: Vec<u8>,
}

impl RobotCli {
    pub(super) fn new(
        executable: OsString,
        bytes: &[u8],
        sdf: &str,
        changed: &str,
    ) -> Result<Self> {
        let directory = tempfile::tempdir()?;
        let root = directory.path();
        fs::write(root.join("robot.sdf"), sdf)?;
        fs::write(root.join("different.sdf"), changed)?;
        fs::write(root.join("barrier.gzgroundedrobot"), bytes)?;
        let runner = Self {
            executable,
            directory,
            checkpoint: bytes.to_vec(),
        };
        let mismatch = "--returned-robot-session requires --physics-engine multifidelity and its authored 1 ms step";
        for (world, engine, step, reason) in [
            (
                "different.sdf",
                "multifidelity",
                "1000000",
                "initial physics state belongs to a different backend",
            ),
            ("robot.sdf", "rapier", "1000000", mismatch),
            ("robot.sdf", "multifidelity", "2000000", mismatch),
        ] {
            let failure = runner.run(world, engine, step, 1)?;
            assert!(
                !failure.status.success()
                    && String::from_utf8_lossy(&failure.stderr).contains(reason),
                "{world} {engine} {step}"
            );
            runner.check_archive()?;
        }
        Ok(runner)
    }

    fn run(
        &self,
        world: &str,
        engine: &str,
        step: &str,
        intervals: u64,
    ) -> Result<std::process::Output> {
        let root = self.directory.path();
        Ok(Command::new(&self.executable)
            .arg(root.join(world))
            .args([
                "--physics-engine",
                engine,
                "--step-ns",
                step,
                "--returned-robot-session",
            ])
            .arg(root)
            .args(["--iterations", &intervals.to_string()])
            .output()?)
    }

    fn check_archive(&self) -> Result<()> {
        assert_eq!(
            fs::read(self.directory.path().join("barrier.gzgroundedrobot"))?,
            self.checkpoint
        );
        Ok(())
    }

    pub(super) fn check(&self, reference: &Simulation, intervals: u64) -> Result<()> {
        let output = self.run("robot.sdf", "multifidelity", "1000000", intervals)?;
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let mut expected = format!(
            "simulation_ns={} iterations={intervals}\n",
            reference.elapsed().as_nanos()
        );
        let mut poses = BTreeMap::new();
        for entity in reference.world().iter_entities() {
            if entity.get::<EntityKind>() == Some(&EntityKind::Link) {
                poses.insert(
                    &entity.get::<Name>().ok_or("missing link name")?.0,
                    entity
                        .get::<WorldPose>()
                        .ok_or("missing link pose")?
                        .translation,
                );
            }
        }
        assert!(poses.len() >= 31 && poses.keys().any(|name| name.ends_with("::humanoid::chest")));
        for (name, [x, y, z]) in poses {
            writeln!(expected, "{name} {x} {y} {z}")?;
        }
        assert_eq!(String::from_utf8(output.stdout)?, expected);
        self.check_archive()
    }
}
