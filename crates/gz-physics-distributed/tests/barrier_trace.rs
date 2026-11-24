// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(all(target_os = "linux", feature = "barrier-trace"))]

#[allow(dead_code)]
#[path = "../examples/support/changing_worlds.rs"]
mod scene;
use gz_physics_distributed::{
    physics::{GroundPlane, SimulationStep},
    pool::WorkerPool,
};
use serde_json::Value;
use std::{fs, path::Path, process::Command};

#[test]
fn scoped_clocks_distinguish_reply_ready_from_collection() -> Result<(), Box<dyn std::error::Error>>
{
    const CHILD: &str = "GZ_BARRIER_CLOCK_TEST";
    if std::env::var_os(CHILD).is_some() {
        let bodies = scene::seeds(scene::Layout::Adjacent).concat();
        let mut pool = WorkerPool::spawn(
            Path::new(env!("CARGO_BIN_EXE_physics_worker")),
            35,
            vec![bodies, vec![], vec![], vec![]],
            SimulationStep::Ms10,
            Some(GroundPlane::new(0.)?),
            1024,
        )?;
        for _ in 0..3 {
            pool.advance_islands_in_field(scene::GRAVITY, &[])?;
        }
        return Ok(());
    }
    let dir = tempfile::tempdir()?;
    let status = Command::new(std::env::current_exe()?)
        .args([
            "--exact",
            "scoped_clocks_distinguish_reply_ready_from_collection",
        ])
        .env(CHILD, "1")
        .env("GZ_SIM_JOIN_TRACE", "1")
        .env("GZ_SIM_BARRIER_TRACE_DIR", dir.path())
        .status()?;
    assert!(status.success());
    // Tick 2 has a reused executor and each worker has continued through tick 3.
    let mut worker_rows = Vec::new();
    let mut coordinator = None;
    for f in fs::read_dir(dir.path())? {
        let f = f?;
        let name = f.file_name().into_string().unwrap();
        let content = fs::read_to_string(f.path())?;
        let r: Value = serde_json::from_str(content.lines().nth(1).unwrap())?;
        assert_eq!(r["tick"], 2);
        if name.starts_with("coord-") {
            coordinator = Some(r);
        } else {
            worker_rows.push((name.split('-').nth(1).unwrap().parse::<u64>()?, r));
        }
    }
    assert_eq!(worker_rows.len(), 4);
    let coordinator = coordinator.unwrap();
    fn stamp(row: &Value, name: &str, peer: Option<u64>) -> u64 {
        let e = row["events"]
            .as_array()
            .unwrap()
            .iter()
            .find(|e| e["name"] == name && e["peer"].as_u64() == peer)
            .unwrap();
        e["ns"].as_u64().unwrap()
    }
    for (id, w) in &worker_rows {
        let c = |name| stamp(&coordinator, name, Some(*id));
        let local = |name| stamp(w, name, None);
        let chain = [
            stamp(&coordinator, "api_start", None),
            c("dispatch_start"),
            c("submit"),
            local("request"),
            local("handle_start"),
            local("batch_start"),
            local("batch_end"),
            local("handle_end"),
            local("reply_submit"),
            c("ready"),
            c("received"),
            c("decoded"),
            stamp(&coordinator, "api_end", None),
        ];
        assert!(chain.is_sorted(), "{id} {chain:?}");
        assert!(c("collect_start") <= c("received"));
        let endpoints: Vec<_> = ["collect_start", "received", "decoded"]
            .iter()
            .map(|name| {
                coordinator["events"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .find(|e| e["name"] == *name && e["peer"] == *id)
                    .unwrap()
            })
            .collect();
        assert!(endpoints.windows(2).all(|pair| {
            pair[0]["cpu_ns"].as_u64().unwrap() <= pair[1]["cpu_ns"].as_u64().unwrap()
        }));
        // procfs may legitimately be unavailable in a restricted environment.
        if endpoints.iter().all(|e| e["sched"].is_object()) {
            for pair in endpoints.windows(2) {
                assert_eq!(pair[0]["sched"]["tid"], pair[1]["sched"]["tid"]);
                assert!(
                    pair[0]["sched"]["runqueue_ns"].as_u64().unwrap()
                        <= pair[1]["sched"]["runqueue_ns"].as_u64().unwrap()
                );
            }
        }
        let native: Vec<_> = w["events"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|e| e["name"] == "native_start")
            .collect();
        if *id == 0 {
            assert!(!native.is_empty());
        } else {
            assert!(native.is_empty());
        }
        let helper: Vec<_> = w["events"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|e| matches!(e["name"].as_str(), Some("helper_submit" | "helper_entry")))
            .collect();
        if *id == 0 && std::thread::available_parallelism().is_ok_and(|n| n.get() >= 2) {
            assert_eq!(helper.len(), 2);
            let events = w["events"].as_array().unwrap();
            let returned = events.iter().find(|e| e["name"] == "scope_return").unwrap();
            let done: Vec<_> = events.iter().filter(|e| e["name"] == "lane_done").collect();
            assert_eq!(done.len(), 2);
            for lane in [true, false] {
                let t = done.iter().find(|e| e["helper"] == lane).unwrap();
                let end = events
                    .iter()
                    .filter(|e| e["name"] == "native_end" && e["helper"] == lane)
                    .map(|e| e["ns"].as_u64().unwrap())
                    .max()
                    .unwrap();
                assert!(
                    [
                        end,
                        t["ns"].as_u64().unwrap(),
                        returned["ns"].as_u64().unwrap(),
                        local("batch_end")
                    ]
                    .is_sorted()
                );
                if let Some(tid) = returned["sched"]["tid"].as_u64() {
                    assert_eq!(t["sched"]["tid"].as_u64(), Some(tid));
                }
                assert_eq!(t["cpu_ns"].is_number(), !lane);
            }
            let submit = helper
                .iter()
                .find(|e| e["name"] == "helper_submit")
                .unwrap();
            let entry = helper.iter().find(|e| e["name"] == "helper_entry").unwrap();
            assert!(submit["ns"].as_u64().unwrap() <= entry["ns"].as_u64().unwrap());
            let first_world = native
                .iter()
                .filter(|e| e["helper"] == true)
                .map(|e| e["ns"].as_u64().unwrap())
                .min()
                .unwrap();
            assert!(entry["ns"].as_u64().unwrap() <= first_world);
            if Path::new("/proc/thread-self/schedstat").is_file() {
                assert_eq!(
                    submit["sched"]["tid"].as_u64().unwrap(),
                    entry["sched"]["tid"].as_u64().unwrap()
                );
                assert!(
                    submit["sched"]["runqueue_ns"].as_u64().unwrap()
                        <= entry["sched"]["runqueue_ns"].as_u64().unwrap()
                );
            }
        } else {
            assert!(helper.is_empty());
        }
        for start in native {
            let end = w["events"]
                .as_array()
                .unwrap()
                .iter()
                .find(|e| e["name"] == "native_end" && e["world"] == start["world"])
                .unwrap();
            assert_eq!(start["helper"], end["helper"]);
            assert!(
                [
                    local("batch_start"),
                    start["ns"].as_u64().unwrap(),
                    end["ns"].as_u64().unwrap(),
                    local("batch_end")
                ]
                .is_sorted()
            );
            assert!(start["cpu_ns"].as_u64().unwrap() <= end["cpu_ns"].as_u64().unwrap());
        }
    }
    Ok(())
}
