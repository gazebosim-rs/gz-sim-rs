// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_distributed::{coordinator::CheckpointPeer, protocol::WorkerImage};

struct CapturingPeer {
    local: LocalPeer,
    image: Option<WorkerImage>,
    fail: Rc<Cell<u8>>,
    commits: Rc<Cell<usize>>,
}
impl Peer for CapturingPeer {
    fn checkpoint_peer(&mut self) -> Option<&mut dyn CheckpointPeer> {
        Some(self)
    }
    fn exchange(&mut self, r: &Request) -> Result<Reply, TransportError> {
        self.local.exchange(r)
    }
    fn restart(&mut self) -> io::Result<()> {
        self.local.restart()?;
        if let Some(im) = &self.image {
            self.local.worker =
                Worker::from_trusted_image(RUN, self.local.id, im).map_err(io::Error::other)?;
        }
        Ok(())
    }
}
impl CheckpointPeer for CapturingPeer {
    fn capture_committed(
        &mut self,
        sequence: u64,
        tick: u64,
    ) -> Result<WorkerImage, TransportError> {
        let state = self.local.worker.recovery_image(sequence, tick)?;
        match self.fail.replace(0) {
            1 => Err(TransportError::Unavailable(
                "injected image capture failure".into(),
            )),
            2 => {
                let mut json = serde_json::to_value(state).unwrap();
                json["digest"][0] = serde_json::json!(json["digest"][0].as_u64().unwrap() ^ 1);
                Ok(serde_json::from_value(json).unwrap())
            }
            3 => Ok(self.image.as_ref().unwrap().clone()),
            _ => Ok(state),
        }
    }
    fn retain_restart_image(&mut self, bytes: Vec<u8>) {
        self.image = Some(serde_json::from_slice(&bytes).unwrap());
        self.commits.set(self.commits.get() + 1);
    }
}

#[test]
fn interrupted_or_corrupt_second_capture_preserves_both_baselines_and_journal_suffixes() {
    let (local, links) = peers(initial());
    let peers = local.map(|local| CapturingPeer {
        local,
        image: None,
        fail: Default::default(),
        commits: Default::default(),
    });
    let fail = peers[1].fail.clone();
    let counts = peers.each_ref().map(|p| p.commits.clone());
    let mut c = Coordinator::new(RUN, peers, 16).unwrap();
    let mut oracle = group(128).0;
    assert_eq!(c.compact().unwrap().released_entries, [1, 1]);
    for mode in [1, 2, 3] {
        assert_eq!(c.advance().unwrap(), oracle.advance().unwrap());
        fail.set(mode);
        let before = c.retained_entries();
        assert!(c.compact().is_err());
        assert_eq!(c.retained_entries(), before);
        for count in &counts {
            assert_eq!(count.get(), 1);
        }
        for w in [0, 1] {
            c.recover(w).unwrap();
        }
        assert_eq!(c.frame(), oracle.frame());
    }
    let seq = links[1].requests.borrow().last().unwrap().sequence + 1;
    links[1].next_fault.set(Some((seq, Fault::LoseReply)));
    assert!(c.advance().is_err());
    assert!(matches!(c.compact(), Err(Error::Pending)));
    c.abort_pending().unwrap();
    assert_eq!(c.advance().unwrap(), oracle.advance().unwrap());
    links[1].fail_restart.set(true);
    assert!(c.recover(1).is_err());
    assert!(matches!(c.compact(), Err(Error::Offline(1))));
    c.recover(1).unwrap();
    assert!(c.compact().unwrap().bootstrap_bytes > 0);
    assert_eq!(c.retained_entries(), [0, 0]);
    for count in counts {
        assert_eq!(count.get(), 2);
    }
    assert_eq!(c.advance().unwrap(), oracle.advance().unwrap());
    assert!(matches!(oracle.compact(), Err(Error::Invalid(_))));
}
