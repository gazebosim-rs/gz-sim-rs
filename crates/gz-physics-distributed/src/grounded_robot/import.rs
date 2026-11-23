// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

impl GroundedRobotPool {
    /// Adopt a warm ordinary graph into a fresh durable session. The supplied clock and
    /// public IDs come from the exporting scene. The native image binds all solver construction
    /// data, coordinates, pending inputs and warm floor contacts in their native insertion order.
    /// A malformed image or an occupied ledger produces no new cut.
    pub fn create_from_native(
        exe: &Path,
        dir: &Path,
        width: u8,
        spec: GroundedRobotSpec,
        epoch: u64,
        tick: u64,
        image: &[u8],
    ) -> Result<Self> {
        if !(2..=8).contains(&width)
            || tick == 0
            || tick == u64::MAX
            || epoch == u64::MAX
            || !(8..=GROUNDED_ROBOT_IMAGE_LIMIT).contains(&image.len())
        {
            return Err(invalid("invalid robot source clock, width or image length"));
        }
        let ledger = Self::ledger(dir, true)?;
        if ledger.archive_path(FILE).try_exists()? {
            return Err(invalid("grounded robot session already exists"));
        }
        let mut local = native::RobotWorld::new(&spec, tick, None)?;
        local.install_native_image(image)?;
        let snapshot = local.capture()?;
        if snapshot.image.as_deref() != Some(image) {
            return Err(invalid("robot native image did not round-trip exactly"));
        }
        let cut = Cut {
            epoch,
            generation: epoch,
            width,
            owner: 0,
            posture: None,
            snapshot,
        };
        let pool = Self::open(exe, ledger, spec, cut, RobotGraphEncoding::Auto)?;
        boundary("robot-import-acknowledged");
        pool.publish(&pool.cut)
            .map_err(ArticulationError::Publication)?;
        boundary("after-sync");
        Ok(pool)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::floating_tree::crash_tests::terminate_probe;
    use std::{error::Error, fs};

    type Test<T = ()> = std::result::Result<T, Box<dyn Error>>;
    const CHILD: &str = "grounded_robot::pool::import::tests::initial_robot_handoff_child";

    fn adopt(exe: &Path, dir: &Path) -> Test<GroundedRobotPool> {
        let source = tempfile::tempdir()?;
        fs::copy(
            std::env::var("GZ_ROBOT_LOADED_CUT")?,
            source.path().join(FILE),
        )?;
        let l = GroundedRobotPool::ledger(source.path(), false)?;
        let (spec, cut) = GroundedRobotPool::read(&l)?;
        Ok(GroundedRobotPool::create_from_native(
            exe,
            dir,
            3,
            spec,
            1,
            cut.snapshot.state.tick,
            cut.snapshot
                .image
                .as_deref()
                .ok_or("source has no native image")?,
        )?)
    }

    #[test]
    #[ignore = "subprocess for hard SIGKILL during first robot import"]
    fn initial_robot_handoff_child() -> Test {
        let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
        let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
        let _pool = adopt(Path::new(&exe), Path::new(&dir))?;
        panic!("initial handoff passed the requested crash edge")
    }

    #[test]
    #[ignore = "requires a qualified loaded humanoid cut and real managed worker"]
    fn initial_robot_handoff_crash_preserves_the_warm_source_or_full_new_cut() -> Test {
        let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
        let exe = Path::new(&exe);
        for edge in [
            "robot-import-acknowledged",
            "before-rename",
            "after-rename",
            "after-sync",
        ] {
            let expected = tempfile::tempdir()?;
            let target = tempfile::tempdir()?;
            let mut direct = adopt(exe, expected.path())?;
            assert!(!direct.ground_patches().is_empty());
            let tick = direct.tick();
            terminate_probe(CHILD, target.path(), "initial-handoff", edge, tick)?;
            let published = matches!(edge, "after-rename" | "after-sync");
            let cut = target.path().join(FILE);
            assert_eq!(cut.try_exists()?, published);
            let mut recovered = if published {
                GroundedRobotPool::resume(exe, target.path())?
            } else {
                assert!(GroundedRobotPool::resume(exe, target.path()).is_err());
                adopt(exe, target.path())?
            };
            let expected_file = expected.path().join(FILE);
            assert_eq!(fs::read(&cut)?, fs::read(&expected_file)?);
            for _ in 0..32 {
                let joint = direct.spec().joints[0].joint_id;
                let cmd = [(joint, 0.075)];
                direct.advance_with_inputs_at(1, direct.tick(), &cmd, &[])?;
                recovered.advance_with_inputs_at(1, recovered.tick(), &cmd, &[])?;
                assert_eq!(fs::read(&cut)?, fs::read(&expected_file)?);
            }
            eprintln!(
                "edge={edge} source_tick={tick} published_before_kill={published} identical_continuation=32"
            );
        }
        Ok(())
    }
}
