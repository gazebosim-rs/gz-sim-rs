// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional measurements of the actual native solves and threads at a published barrier.
use super::{Island, SolveSchedule, WorldError};
use serde::Serialize;
use std::{
    fs::OpenOptions,
    io::{self, Write},
    path::PathBuf,
    sync::OnceLock,
    thread::{self, ThreadId},
    time::Instant,
};

#[derive(Serialize)]
struct WorldSolve {
    ids: Vec<u64>,
    fine: bool,
    helper: bool,
    prior_pairs: usize,
    prior_supports: usize,
    pairs: usize,
    supports: usize,
    solve_ns: u64,
}

pub(super) struct Capture {
    file: PathBuf,
    caller: ThreadId,
    rows: Vec<OnceLock<WorldSolve>>,
}

impl Capture {
    pub(super) fn from_env(worlds: usize) -> Option<Self> {
        let dir = std::env::var_os("GZ_SIM_LANE_TRACE_DIR")?;
        Some(Self {
            file: PathBuf::from(dir).join(format!("{}.jsonl", std::process::id())),
            caller: thread::current().id(),
            rows: (0..worlds).map(|_| OnceLock::new()).collect(),
        })
    }

    pub(super) fn measure(
        &self,
        index: usize,
        world: &mut Island,
        schedule: SolveSchedule<'_>,
        solve: impl FnOnce(&mut Island) -> Result<(), WorldError>,
    ) -> Result<(), WorldError> {
        let prior_pairs = world.committed.impulses.len();
        let prior_supports = world.committed.supports.len();
        let start = Instant::now();
        solve(world)?;
        let elapsed = start.elapsed();
        let s = &world.committed;
        let row = WorldSolve {
            ids: s.bodies.iter().map(|b| b.id).collect(),
            fine: schedule.fine(s.bodies[0].id),
            helper: thread::current().id() != self.caller,
            prior_pairs,
            prior_supports,
            pairs: s.impulses.len(),
            supports: s.supports.len(),
            solve_ns: elapsed.as_nanos().try_into().unwrap_or(u64::MAX),
        };
        assert!(self.rows[index].set(row).is_ok());
        Ok(())
    }

    fn write(self, tick: u64) -> io::Result<()> {
        #[derive(Serialize)]
        struct Barrier {
            tick: u64,
            worlds: Vec<WorldSolve>,
        }
        let worlds = self
            .rows
            .into_iter()
            .map(|r| {
                r.into_inner()
                    .expect("every published world completed its native solve")
            })
            .collect();
        let mut bytes = serde_json::to_vec(&Barrier { tick, worlds })?;
        bytes.push(b'\n');
        OpenOptions::new()
            .create(true)
            .append(true)
            .open(self.file)?
            .write_all(&bytes)
    }

    pub(super) fn publish(self, tick: u64) {
        if let Err(error) = self.write(tick) {
            // A diagnostics filesystem failure must never fault a physics worker.
            eprintln!("worker lane trace write failed at tick {tick}: {error}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{Body, GroundPlane, Partition, Primitive, SimulationStep, Snapshot, Tier};
    use serde_json::Value;
    use std::{error::Error, fs, process::Command};

    #[test]
    fn observed_threads_and_receipts_match_published_worlds() -> Result<(), Box<dyn Error>> {
        // Set the tracing environment only in a dedicated child test process.
        if std::env::var_os("GZ_TEST_TRACE_CHILD").is_none() {
            let dir = tempfile::tempdir()?;
            let status = Command::new(std::env::current_exe()?)
                .args([
                    "--exact",
                    concat!(
                        module_path!(),
                        "::observed_threads_and_receipts_match_published_worlds"
                    )
                    .split_once("::")
                    .unwrap()
                    .1,
                ])
                .env("GZ_TEST_TRACE_CHILD", "1")
                .env("GZ_SIM_LANE_TRACE_DIR", dir.path())
                .status()?;
            assert!(status.success());
            assert_eq!(dir.path().read_dir()?.count(), 1);
            return Ok(());
        }

        let bodies = (1..=3)
            .map(|id| Body {
                id,
                primitive: Primitive::Sphere,
                size: [0.2; 3],
                mass: 1.,
                tier: Tier::Dynamic,
                friction: 0.,
                position: [100. * id as f64, 0., 0.1],
                orientation: [0., 0., 0., 1.],
                linear: [0.; 3],
                angular: [0.; 3],
            })
            .collect();
        let mut world = Partition::restore(&Snapshot {
            bodies,
            ground: Some(GroundPlane::new(0.)?),
            step: SimulationStep::Ms10,
            ..Default::default()
        })?;
        world.helper_allowed = true;
        let g = [0., 0., -9.81];
        world.start_probe_in_mode(0, 2, true)?;
        world.step_scheduled(g, &[], SolveSchedule::Islands(&[1, 2, 3]))?;
        let first = world.snapshot();
        world.step_scheduled(g, &[], SolveSchedule::Islands(&[2]))?;
        assert!(
            world
                .step_scheduled(g, &[], SolveSchedule::Islands(&[4]))
                .is_err()
        );
        let measured = world.read_probe(0, 2).expect("complete scheduled probe");
        assert!(measured.iter().all(|row| row.elapsed_ns > 0));
        let file = PathBuf::from(std::env::var_os("GZ_SIM_LANE_TRACE_DIR").unwrap())
            .join(format!("{}.jsonl", std::process::id()));
        let rows = fs::read_to_string(file)?
            .lines()
            .map(serde_json::from_str::<Value>)
            .collect::<Result<Vec<_>, _>>()?;
        assert_eq!(rows.len(), 2);
        for (i, snapshot) in [first, world.snapshot()].iter().enumerate() {
            assert_eq!(rows[i]["tick"], snapshot.tick);
            let worlds = rows[i]["worlds"].as_array().unwrap();
            assert_eq!(worlds.len(), 3);
            for (j, w) in worlds.iter().enumerate() {
                let id = j as u64 + 1;
                assert_eq!(w["ids"], serde_json::json!([id]));
                assert_eq!(w["helper"], i == 0 && j == 0);
                assert_eq!(w["fine"], i == 0 || j == 1);
                assert_eq!(w["prior_supports"], i as u64);
                assert_eq!(w["supports"], 1);
                assert_eq!(w["pairs"], 0);
                assert_eq!(w["prior_pairs"], 0);
                assert!(snapshot.supports.iter().any(|s| s.body == id));
                assert!(w["solve_ns"].as_u64().unwrap() > 0);
            }
        }
        Ok(())
    }
}
