// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Move a warm worker graph to a single ordinary process at a durable physical cut.
use super::*;
use std::sync::Arc;

#[path = "rejoin.rs"]
mod rejoin;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LocalAuthority {
    Local,
    Managed,
    Uncertain,
}

const LOCAL_ARCHIVE: [u8; 8] = *b"GZRBLC01";

/// Commands staged on the Bevy side between complete physical intervals.
/// Joint effort already present in a native image remains inside that image.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RobotLocalInputs {
    pub efforts: Vec<(u64, f64)>,
    pub forces: Vec<(u64, [f64; 3])>,
}

#[derive(Serialize, Deserialize)]
struct LocalCut {
    spec: GroundedRobotSpec,
    epoch: u64,
    generation: u64,
    width: u8,
    source_digest: [u8; 32],
    snapshot: Snapshot,
    inputs: RobotLocalInputs,
}

impl LocalCut {
    fn check(&self) -> Result<()> {
        self.spec.root()?;
        self.snapshot.check(&self.spec)?;
        if !(2..=8).contains(&self.width)
            || self.epoch >= self.generation
            || self.snapshot.state.tick == 0
            || self.snapshot.image.is_none()
            || self.inputs.efforts.len() > self.spec.joints.len()
            || self.inputs.forces.len() > self.spec.links.len()
        {
            return Err(invalid(
                "invalid returned robot clock, image or input count",
            ));
        }
        let mut seen = BTreeSet::new();
        for &(id, effort) in &self.inputs.efforts {
            let i = self
                .snapshot
                .state
                .joints
                .binary_search_by_key(&id, |j| j.joint_id)
                .map_err(|_| invalid("unknown returned robot effort"))?;
            if !seen.insert(id)
                || !(self.snapshot.state.joints[i]
                    .pending_effort
                    .unwrap_or_default()
                    + effort)
                    .is_finite()
            {
                return Err(invalid("duplicate or overflowing returned robot effort"));
            }
        }
        for &(id, force) in &self.inputs.forces {
            if !seen.insert(id)
                || self
                    .spec
                    .links
                    .binary_search_by_key(&id, |link| link.id)
                    .is_err()
                || !force.iter().all(|x| x.is_finite())
            {
                return Err(invalid(
                    "unknown, duplicate or nonfinite returned robot force",
                ));
            }
        }
        Ok(())
    }
}

/// Exclusive authority to continue the released cut in an ordinary process. A normal
/// managed-worker resume refuses this format. A crashed local process can reopen the
/// durable release cut; ticks advanced only in memory have ordinary process durability.
pub struct ReturnedGroundedRobot {
    ledger: Option<Arc<Ledger>>,
    cut: LocalCut,
    state: LocalAuthority,
}

impl ReturnedGroundedRobot {
    fn owned_ledger(&self) -> &Arc<Ledger> {
        self.ledger
            .as_ref()
            .expect("local owner holds the lease until rejoin")
    }

    /// Persist an accepted local interval under the retained exclusive lease.
    /// A later crash restores this cut, including native held effort and warm support.
    pub fn checkpoint_native(&mut self, epoch: u64, tick: u64, image: &[u8]) -> Result<()> {
        if self.state != LocalAuthority::Local {
            return Err(invalid(
                "reopen local robot after an uncertain checkpoint publication",
            ));
        }
        if (epoch, tick) <= (self.cut.epoch, self.cut.snapshot.state.tick)
            || tick == 0
            || tick == u64::MAX
            || epoch == u64::MAX
            || !(8..=GROUNDED_ROBOT_IMAGE_LIMIT).contains(&image.len())
        {
            return Err(invalid("invalid local robot checkpoint clock or image"));
        }
        let mut local = native::RobotWorld::new(&self.cut.spec, tick, None)?;
        local.install_native_image(image)?;
        let snapshot = local.capture()?;
        if snapshot.image.as_deref() != Some(image) {
            return Err(invalid(
                "local robot checkpoint failed exact native round trip",
            ));
        }
        let generation = self
            .cut
            .generation
            .checked_add(epoch - self.cut.epoch)
            .ok_or(invalid("local robot generation exhausted"))?;
        let cut = LocalCut {
            spec: self.cut.spec.clone(),
            epoch,
            generation,
            width: self.cut.width,
            source_digest: self.cut.source_digest,
            snapshot,
            inputs: RobotLocalInputs::default(),
        };
        cut.check()?;
        let bytes = codec(FILE_LIMIT)
            .serialize(&cut)
            .map_err(io::Error::other)?;
        if let Err(error) = self.owned_ledger().publish_bytes(
            &self.owned_ledger().archive_path(FILE),
            &LOCAL_ARCHIVE,
            &bytes,
            boundary,
        ) {
            self.state = LocalAuthority::Uncertain;
            return Err(ArticulationError::Publication(error));
        }
        boundary("after-sync");
        self.cut = cut;
        Ok(())
    }

    pub fn resume(directory: &Path) -> Result<Self> {
        let ledger = GroundedRobotPool::ledger(directory, false)?;
        let mut bytes = Vec::new();
        File::open(ledger.archive_path(FILE))?
            .take((FILE_LIMIT + 41) as u64)
            .read_to_end(&mut bytes)?;
        if !(40..=FILE_LIMIT + 40).contains(&bytes.len())
            || bytes[..8] != LOCAL_ARCHIVE
            || Sha256::digest(&bytes[40..])[..] != bytes[8..40]
        {
            return Err(invalid("invalid local-owner robot archive"));
        }
        let cut: LocalCut = codec(FILE_LIMIT)
            .deserialize(&bytes[40..])
            .map_err(io::Error::other)?;
        cut.check()?;
        let mut reconstructed = native::RobotWorld::new(&cut.spec, cut.snapshot.state.tick, None)?;
        reconstructed
            .install_native_image(cut.snapshot.image.as_deref().expect("checked warm image"))?;
        let bytes = |s: &Snapshot| codec(SNAP_LIMIT).serialize(s).map_err(io::Error::other);
        if bytes(&reconstructed.capture()?)? != bytes(&cut.snapshot)? {
            return Err(invalid(
                "returned robot observations differ from native graph",
            ));
        }
        Ok(Self {
            ledger: Some(Arc::new(ledger)),
            cut,
            state: LocalAuthority::Local,
        })
    }

    /// False after a disk error whose crash-visible outcome requires reopening the ledger.
    pub fn publication_certain(&self) -> bool {
        self.state != LocalAuthority::Uncertain
    }

    pub fn spec(&self) -> &GroundedRobotSpec {
        &self.cut.spec
    }
    pub fn epoch(&self) -> u64 {
        self.cut.epoch
    }
    pub fn tick(&self) -> u64 {
        self.cut.snapshot.state.tick
    }
    pub fn generation(&self) -> u64 {
        self.cut.generation
    }
    pub fn source_digest(&self) -> [u8; 32] {
        self.cut.source_digest
    }
    pub fn native_image(&self) -> &[u8] {
        self.cut
            .snapshot
            .image
            .as_deref()
            .expect("checked warm image")
    }
    pub fn inputs(&self) -> &RobotLocalInputs {
        &self.cut.inputs
    }
}

impl GroundedRobotPool {
    /// Fence the physical worker and durably release it to an ordinary local process.
    /// The caller must first reconstruct its destination from this exact cut and apply
    /// the supplied staged inputs. The returned guard must remain with the live simulator.
    pub fn release_to_local_at(
        &mut self,
        epoch: u64,
        tick: u64,
        source_digest: [u8; 32],
        inputs: RobotLocalInputs,
    ) -> Result<ReturnedGroundedRobot> {
        self.check_interval(epoch, tick)?;
        let cut = LocalCut {
            spec: self.spec.clone(),
            epoch,
            generation: self
                .generation()
                .checked_add(1)
                .ok_or(invalid("robot generation exhausted"))?,
            width: self.width(),
            source_digest,
            snapshot: self.cut.snapshot.clone(),
            inputs,
        };
        cut.check()?;
        let bytes = codec(FILE_LIMIT)
            .serialize(&cut)
            .map_err(io::Error::other)?;
        let digest = Sha256::digest(
            codec(SNAP_LIMIT)
                .serialize(&self.cut.snapshot)
                .map_err(io::Error::other)?,
        )
        .into();
        if let Err(err) = self.ack(self.owner(), tick, self.generation(), Op::Fence(digest)) {
            self.recover()?;
            if !matches!(err, ArticulationError::Io(_)) {
                return Err(err);
            }
            if let Err(err) = self.ack(self.owner(), tick, self.generation(), Op::Fence(digest)) {
                self.healthy = false;
                self.slots.clear();
                return Err(err);
            }
        }
        boundary("robot-return-source-fenced");
        self.healthy = false;
        self.slots.clear();
        self.owned_ledger()
            .publish_bytes(
                &self.owned_ledger().archive_path(FILE),
                &LOCAL_ARCHIVE,
                &bytes,
                boundary,
            )
            .map_err(ArticulationError::Publication)?;
        boundary("after-sync");
        Ok(ReturnedGroundedRobot {
            ledger: self.ledger.take(),
            cut,
            state: LocalAuthority::Local,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::floating_tree::crash_tests::terminate_probe;
    use std::{error::Error, fs};

    type Test<T = ()> = std::result::Result<T, Box<dyn Error>>;
    const CHILD: &str = "grounded_robot::pool::local_owner::tests::returning_robot_controller";

    fn release(p: &mut GroundedRobotPool) -> Test<ReturnedGroundedRobot> {
        let staged = RobotLocalInputs {
            efforts: vec![(p.spec().joints[3].joint_id, 0.135)],
            forces: vec![(p.spec().links[4].id, [0.2, -0.13, 0.01])],
        };
        Ok(p.release_to_local_at(p.epoch(), p.tick(), [15; 32], staged)?)
    }

    #[test]
    #[ignore = "child killed at physical owner release or checkpoint publication"]
    fn returning_robot_controller() -> Test {
        let dir = std::env::var("GZ_FLOATING_TREE_TEST_SESSION")?;
        let dir = Path::new(&dir);
        if std::env::var("GZ_FLOATING_TREE_TEST_ACTION")? == "return" {
            let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
            let mut p = GroundedRobotPool::resume(Path::new(&worker), dir)?;
            let _owned = release(&mut p)?;
        } else {
            let mut local = ReturnedGroundedRobot::resume(dir)?;
            local.checkpoint_native(
                local.epoch(),
                local.tick() + 1,
                &fs::read(dir.join("next-image"))?,
            )?;
        }
        panic!("missed expected robot owner publication boundary")
    }

    #[test]
    #[ignore = "requires a loaded robot archive and the actual physical worker"]
    fn local_owner_process_deaths_publish_one_authority_and_one_recovery_point() -> Test {
        let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
        let worker = Path::new(&worker);
        let source = fs::read(std::env::var("GZ_ROBOT_LOADED_CUT")?)?;
        let physical = tempfile::tempdir()?;
        fs::write(physical.path().join(FILE), &source)?;
        let mut ordinary = GroundedRobotPool::resume(worker, physical.path())?;
        let epoch = ordinary.epoch();
        let tick = ordinary.tick();
        let efforts = [(ordinary.spec().joints[3].joint_id, 0.135)];
        let forces = [(ordinary.spec().links[4].id, [0.2, -0.13, 0.01])];
        ordinary.advance_with_inputs_at(epoch, tick, &efforts, &forces)?;
        let next = ordinary
            .native_image()
            .ok_or("missing expected next physical image")?
            .to_vec();
        drop(ordinary);

        let reference = tempfile::tempdir()?;
        let path = reference.path().join(FILE);
        fs::write(&path, &source)?;
        let mut p = GroundedRobotPool::resume(worker, reference.path())?;
        assert!(
            p.release_to_local_at(epoch, tick + 1, [15; 32], RobotLocalInputs::default())
                .is_err()
        );
        let bad = RobotLocalInputs {
            efforts: vec![efforts[0], efforts[0]],
            forces: vec![],
        };
        assert!(p.release_to_local_at(epoch, tick, [15; 32], bad).is_err());
        assert_eq!(fs::read(&path)?, source);
        let mut local = release(&mut p)?;
        assert!(p.advance().is_err());
        assert!(p.reset().is_err());
        drop(p);
        let returned = fs::read(&path)?;
        assert!(!local.inputs().forces.is_empty());
        assert_eq!(local.tick(), tick);
        assert!(local.checkpoint_native(epoch, tick, &next).is_err());
        local.checkpoint_native(epoch, tick + 1, &next)?;
        let checkpoint = fs::read(&path)?;
        for (op, edges) in [
            (
                "return",
                &[
                    "robot-return-source-fenced",
                    "before-rename",
                    "after-rename",
                    "after-sync",
                ][..],
            ),
            (
                "checkpoint",
                &["before-rename", "after-rename", "after-sync"][..],
            ),
        ] {
            for edge in edges {
                let test = tempfile::tempdir()?;
                let cut = test.path().join(FILE);
                let prior = if op == "return" { &source } else { &returned };
                let expected = if op == "return" {
                    &returned
                } else {
                    &checkpoint
                };
                fs::write(&cut, prior)?;
                fs::write(test.path().join("next-image"), &next)?;
                terminate_probe(CHILD, test.path(), op, edge, tick)?;
                let published = matches!(*edge, "after-rename" | "after-sync");
                assert_eq!(fs::read(&cut)?, *(if published { expected } else { prior }));
                if op == "return" && !published {
                    assert!(ReturnedGroundedRobot::resume(test.path()).is_err());
                    let mut restored = GroundedRobotPool::resume(worker, test.path())?;
                    let _l = release(&mut restored)?;
                } else {
                    assert!(GroundedRobotPool::resume(worker, test.path()).is_err());
                }
                let mut recovered = ReturnedGroundedRobot::resume(test.path())?;
                if recovered.tick() == tick {
                    recovered.checkpoint_native(epoch, tick + 1, &next)?;
                }
                assert_eq!(recovered.native_image(), next);
                assert_eq!(fs::read(&cut)?, checkpoint);
                eprintln!(
                    "operation={op} edge={edge} published={published} single_correct_owner=true continued_cut={}",
                    tick + 1
                );
            }
        }
        Ok(())
    }
}
