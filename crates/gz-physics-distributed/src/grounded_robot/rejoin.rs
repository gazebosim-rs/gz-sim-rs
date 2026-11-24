// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reuse the same ledger and generation sequence after ordinary local stepping.
use super::*;

impl ReturnedGroundedRobot {
    /// Start managed workers from the last acknowledged local cut. Both the clock and
    /// the complete live native graph must still agree with the explicitly persisted image.
    pub fn rejoin_workers_at(
        &mut self,
        exe: &Path,
        epoch: u64,
        tick: u64,
        live_image: &[u8],
    ) -> Result<GroundedRobotPool> {
        if self.state != LocalAuthority::Local
            || (self.epoch(), self.tick()) != (epoch, tick)
            || self.native_image() != live_image
            || !self.cut.inputs.efforts.is_empty()
            || !self.cut.inputs.forces.is_empty()
        {
            return Err(invalid(
                "rejoin requires the current acknowledged local robot checkpoint",
            ));
        }
        let cut = Cut {
            posture: None,
            width: self.cut.width,
            owner: 0,
            epoch,
            generation: self
                .cut
                .generation
                .checked_add(1)
                .ok_or(invalid("robot generation exhausted"))?,
            snapshot: self.cut.snapshot.clone(),
        };
        let pool = GroundedRobotPool::open_locked(
            exe,
            Arc::clone(self.owned_ledger()),
            self.cut.spec.clone(),
            cut,
            RobotGraphEncoding::Auto,
        )?;
        boundary("robot-rejoin-destination-acknowledged");
        if let Err(err) = pool.publish(&pool.cut) {
            self.state = LocalAuthority::Uncertain;
            return Err(ArticulationError::Publication(err));
        }
        boundary("after-sync");
        self.ledger.take();
        self.state = LocalAuthority::Managed;
        Ok(pool)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::floating_tree::crash_tests::terminate_probe;
    use std::{error::Error, fs};
    type Test<T = ()> = std::result::Result<T, Box<dyn Error>>;
    const CHILD: &str =
        "grounded_robot::pool::local_owner::rejoin::tests::controller_rejoining_the_worker";

    fn rejoin(local: &mut ReturnedGroundedRobot, worker: &Path) -> Test<GroundedRobotPool> {
        let image = local.native_image().to_vec();
        Ok(local.rejoin_workers_at(worker, local.epoch(), local.tick(), &image)?)
    }

    #[test]
    #[ignore = "controller killed while installing an acknowledged local graph on a worker"]
    fn controller_rejoining_the_worker() -> Test {
        let target = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
        let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
        let mut local = ReturnedGroundedRobot::resume(Path::new(&target))?;
        let _active = rejoin(&mut local, Path::new(&worker))?;
        panic!("missed requested local-to-worker crash boundary")
    }

    #[test]
    #[ignore = "requires the qualified explicit local checkpoint and physical worker executable"]
    fn hard_process_death_during_rejoin_preserves_the_named_owner_and_exact_future() -> Test {
        let exe = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
        let exe = Path::new(&exe);
        let source = fs::read(std::env::var("GZ_ROBOT_LOCAL_CHECKPOINT")?)?;
        for edge in [
            "robot-rejoin-destination-acknowledged",
            "before-rename",
            "after-rename",
            "after-sync",
        ] {
            let pair = [tempfile::tempdir()?, tempfile::tempdir()?];
            let files = pair.each_ref().map(|d| d.path().join(FILE));
            for path in &files {
                fs::write(path, &source)?;
            }
            let mut local = ReturnedGroundedRobot::resume(pair[0].path())?;
            let epoch = local.epoch();
            let tick = local.tick();
            let image = local.native_image().to_vec();
            assert!(
                local
                    .rejoin_workers_at(exe, epoch, tick + 1, &image)
                    .is_err()
            );
            let mut bad = image.clone();
            bad[45] ^= 1;
            assert!(local.rejoin_workers_at(exe, epoch, tick, &bad).is_err());
            assert_eq!(fs::read(&files[0])?, source);
            let expected_generation = local.generation() + 1;
            let direct = rejoin(&mut local, exe)?;
            assert_eq!(direct.generation(), expected_generation);
            let accepted = fs::read(&files[0])?;
            assert!(local.checkpoint_native(epoch, tick + 1, &image).is_err());
            drop(direct);
            // A fenced handle remaining in memory cannot keep the active identity's file lock.
            let mut direct = GroundedRobotPool::resume(exe, pair[0].path())?;
            terminate_probe(CHILD, pair[1].path(), "rejoin", edge, tick)?;
            let published = matches!(edge, "after-rename" | "after-sync");
            assert_eq!(
                fs::read(&files[1])?,
                *(if published { &accepted } else { &source })
            );
            let mut control = if published {
                assert!(ReturnedGroundedRobot::resume(pair[1].path()).is_err());
                GroundedRobotPool::resume(exe, pair[1].path())?
            } else {
                assert!(GroundedRobotPool::resume(exe, pair[1].path()).is_err());
                let mut retry = ReturnedGroundedRobot::resume(pair[1].path())?;
                rejoin(&mut retry, exe)?
            };
            for _ in 0..32 {
                let effort = [(direct.spec().joints[5].joint_id, -0.09)];
                let force = [(direct.spec().links[3].id, [0.5, 0.11, 0.])];
                direct.advance_with_inputs_at(epoch, direct.tick(), &effort, &force)?;
                control.advance_with_inputs_at(epoch, control.tick(), &effort, &force)?;
                assert_eq!(fs::read(&files[0])?, fs::read(&files[1])?);
            }
            eprintln!(
                "rejoin edge={edge} already_published={published} physical_tick={tick} next_generation={expected_generation} eager_worker_reopen=true exact_accepted_cuts=32"
            );
        }
        Ok(())
    }
}
