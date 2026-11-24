// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::fs;

#[test]
fn bounds_checksum_and_selected_digest_are_checked_before_decode() -> io::Result<()> {
    let root = tempfile::tempdir()?;
    let path = root.path().join("checkpoint");
    for len in [0, 1, HEADER - 1, HEADER + CHECKSUM - 1] {
        fs::write(&path, vec![0; len])?;
        assert!(ConvoyCapsule::inspect(&path).is_err());
    }

    let mut bytes = [0; HEADER + CHECKSUM];
    bytes[..8].copy_from_slice(MAGIC);
    bytes[8..12].copy_from_slice(&u32::MAX.to_le_bytes());
    fs::write(&path, bytes)?;
    assert!(
        ConvoyCapsule::inspect(&path)
            .err()
            .unwrap()
            .to_string()
            .contains("lengths")
    );
    bytes[8..12].fill(0);
    fs::write(&path, bytes)?;
    assert!(
        ConvoyCapsule::inspect(&path)
            .err()
            .unwrap()
            .to_string()
            .contains("checksum")
    );
    assert!(
        ConvoyCapsule::open(&path, &"0".repeat(64))
            .err()
            .unwrap()
            .to_string()
            .contains("selected SHA-256")
    );
    fs::File::create(&path)?.set_len(MAX_PACKAGE as u64 + 1)?;
    assert!(ConvoyCapsule::inspect(&path).is_err());
    assert!(ConvoyCapsule::inspect(root.path()).is_err());
    Ok(())
}

#[cfg(unix)]
#[test]
fn substituted_fifo_is_rejected_without_waiting_for_a_writer() -> io::Result<()> {
    use rustix::fs::{CWD, Mode, mkfifoat};
    let root = tempfile::tempdir()?;
    let pipe = root.path().join("fifo");
    mkfifoat(CWD, &pipe, Mode::RUSR | Mode::WUSR)?;
    assert_eq!(
        ConvoyCapsule::inspect(&pipe).err().unwrap().kind(),
        io::ErrorKind::InvalidData
    );
    let link = root.path().join("link");
    std::os::unix::fs::symlink(&pipe, &link)?;
    assert!(ConvoyCapsule::inspect(&link).is_err());
    Ok(())
}

#[test]
fn drive_definition_round_trips_signed_effort_and_end_policies() -> io::Result<()> {
    let constant = JointTorqueDrive::constant(-0.0)?;
    let hydrated: JointTorqueDrive = serde_json::from_slice(&serde_json::to_vec(&constant)?)?;
    assert_eq!(
        hydrated.interval_effort(u64::MAX).to_bits(),
        (-0.0_f64).to_bits()
    );
    for interpolation in ["hold", "linear"] {
        for end in ["repeat", "hold", "zero"] {
            let text = format!(
                r#"{{"version":1,"interpolation":"{interpolation}","end":"{end}","knots":[{{"tick":0,"torque_nm":-0.03}},{{"tick":151,"torque_nm":0.012}},{{"tick":155,"torque_nm":0.02}}]}}"#
            );
            let original = JointTorqueDrive::read(text.as_bytes())?;
            let hydrated: JointTorqueDrive =
                serde_json::from_slice(&serde_json::to_vec(&original)?)?;
            for tick in [0, 149, 150, 151, 154, 155, 156, u64::MAX] {
                assert_eq!(
                    original.interval_effort(tick).to_bits(),
                    hydrated.interval_effort(tick).to_bits()
                );
            }
        }
    }
    assert!(
        serde_json::from_str::<JointTorqueDrive>(
            r#"{"scheduled":{"version":1,"interpolation":"hold","end":"repeat","knots":[]}}"#
        )
        .is_err()
    );
    Ok(())
}

#[cfg(unix)]
#[test]
#[ignore = "uses GZ_MULTI_TEST_WORKER pointing at a stock physics_worker"]
fn running_writer_capture_exact_input_noclobber_and_pinned_offline_read()
-> Result<(), Box<dyn std::error::Error>> {
    use gz_sim_scene::distributed_hinge::load_distributed_hinge_world;
    use std::thread;
    let exe = std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("set GZ_MULTI_TEST_WORKER")?;
    let sdf = include_str!("../../../../../examples/distributed_sphere_pair_island.sdf");
    let root = tempfile::tempdir()?;
    let session = root.path().join("session");
    let mut sim = load_distributed_hinge_world(
        sdf,
        "source.sdf",
        "default",
        Path::new(&exe),
        &session,
        Some(3),
    )?;
    let archive = session.join("barrier.gzmultisphere");
    let drive = JointTorqueDrive::constant(-0.01)?;
    let obtain = |sdf: &str, drive: &JointTorqueDrive| {
        ConvoyCapsule::capture(&archive, sdf, "default", drive)
    };
    let unqueued = obtain(sdf, &drive)?;
    let joint = unqueued.cut().cut.spec.hinge.joint_id;
    gz_sim_scene::queue_joint_effort(&mut sim, joint, 0.0)?;
    let positive_zero = obtain(sdf, &JointTorqueDrive::constant(0.0)?)?;
    assert_eq!(
        positive_zero.cut().cut.pending_effort.map(f64::to_bits),
        Some(0)
    );
    assert!(obtain(sdf, &JointTorqueDrive::constant(-0.0)?).is_err());
    gz_sim_scene::queue_joint_effort(&mut sim, joint, -0.01)?;
    let package = obtain(sdf, &drive)?;
    assert_ne!(package.sha256(), unqueued.sha256());
    assert_eq!(package.archive(), fs::read(&archive)?);
    assert_eq!(package.cut().cut.pending_effort, Some(-0.01));
    assert!(obtain(&sdf.replace("<mass>2</mass>", "<mass>3</mass>"), &drive).is_err());
    assert!(obtain(sdf, &JointTorqueDrive::constant(0.01)?).is_err());

    let out = root.path().join("recovery.gzcap");
    // Two independent publication calls can race at the same destination.
    let results = thread::scope(|scope| {
        let a = scope.spawn(|| package.publish(&out));
        let b = scope.spawn(|| unqueued.publish(&out));
        [a.join().unwrap(), b.join().unwrap()]
    });
    assert_eq!(results.iter().filter(|v| v.is_ok()).count(), 1);
    let winner = results.iter().find_map(|r| r.as_ref().ok()).unwrap();
    let loser = results.iter().find_map(|r| r.as_ref().err()).unwrap();
    assert_eq!(loser.kind(), io::ErrorKind::AlreadyExists);
    assert_eq!(format!("{:x}", Sha256::digest(fs::read(&out)?)), *winner);
    let copy = ConvoyCapsule::open(&out, winner)?;
    assert_eq!(copy.sha256(), winner);
    assert_eq!(copy.world(), "default");
    assert_eq!(copy.sdf(), sdf);
    assert_eq!(copy.origin(), package.origin());
    assert_eq!(copy.drive().interval_effort(0), -0.01);

    // Scene and physics remain inspectable when the originating session is offline.
    drop(sim);
    fs::rename(&session, root.path().join("offline"))?;
    assert_eq!(ConvoyCapsule::inspect(&out)?.archive(), copy.archive());

    let mut bytes = fs::read(&out)?;
    let index = bytes
        .windows(b"base_visual".len())
        .position(|v| v == b"base_visual")
        .unwrap();
    bytes[index] = b'c';
    fs::write(&out, &bytes)?;
    assert!(ConvoyCapsule::inspect(&out).is_err());
    let end = bytes.len() - CHECKSUM;
    let digest = Sha256::digest(&bytes[..end]);
    bytes[end..].copy_from_slice(&digest);
    fs::write(&out, &bytes)?;
    assert!(ConvoyCapsule::inspect(&out).is_ok());
    assert!(ConvoyCapsule::open(&out, winner).is_err());
    Ok(())
}
