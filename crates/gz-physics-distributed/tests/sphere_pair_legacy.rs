// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reopen the previous coordinator's genuine native cuts under the sphere-pair policy.
use gz_physics_distributed::multi_sphere::{MultiSphereObservation, MultiSpherePool};
use serde::Deserialize;
use std::{
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

#[derive(Deserialize)]
struct Row {
    tick: u64,
    owners: [u8; 3],
    direct: MultiSphereObservation,
    queued_effort_nm: f64,
}

#[test]
#[ignore = "requires the retained v1 four-worker trajectory and cuts"]
fn original_four_cuts_continue_exactly_with_independent_sphere_contact()
-> Result<(), Box<dyn Error>> {
    let run = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../artifacts/multifidelity-two-sphere-hinge/four");
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let rows = BufReader::new(File::open(run.join("states.jsonl"))?)
        .lines()
        .map(|line| -> Result<Row, Box<dyn Error>> { Ok(serde_json::from_str(&line?)?) })
        .collect::<Result<Vec<_>, _>>()?;
    assert_eq!(rows.len(), 1801);
    for cut in [226, 466, 564, 1292] {
        let archive = "barrier.gzmultisphere";
        let seed = fs::read(run.join(format!("cuts/tick-{cut}")).join(archive))?;
        let dir = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
        let path = dir.path().join(archive);
        fs::write(&path, &seed)?;
        let mut pool = MultiSpherePool::resume(worker, dir.path())?;
        assert_eq!(pool.owners(), rows[cut].owners);
        assert!(pool.pair_image().is_none());
        assert_eq!(
            pool.pending_joint_effort(),
            Some(rows[cut].queued_effort_nm)
        );
        assert_eq!(fs::read(&path)?, seed);
        let mut first_pair = None;
        for row in rows.iter().skip(cut) {
            assert_eq!(
                bincode::serialize(pool.observation())?,
                bincode::serialize(&row.direct)?,
                "historical cut {cut}, tick {}",
                row.tick
            );
            let [hinge, a, b] = pool.owners();
            if a == b && a != hinge {
                assert!(pool.pair_image().is_some());
                first_pair.get_or_insert(row.tick);
            }
            if row.tick + 1 < rows.len() as u64 {
                if row.tick != cut as u64 {
                    pool.queue_effort(pool.spec().hinge.joint_id, row.queued_effort_nm)?;
                }
                pool.advance()?;
            }
        }
        assert!(first_pair.is_some());
        if cut == 1292 {
            assert_eq!(first_pair, Some(1293));
        }
        println!(
            "v1 cut {cut}: {} bit-identical observations through tick 1800, first separate pair at {}",
            rows.len() - cut,
            first_pair.unwrap()
        );
    }
    Ok(())
}
