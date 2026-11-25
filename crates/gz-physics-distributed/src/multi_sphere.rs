// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A floating hinge and two separately ownable spherical bodies on local worker processes.
#[cfg(test)]
mod crash_tests;
#[cfg(feature = "sphere-pair-trace")]
pub(crate) mod timing;

// Default builds compile the original expression without a timer or callback.
macro_rules! timed {
    ($field:ident, $operation:expr) => {{
        #[cfg(feature = "sphere-pair-trace")]
        {
            $crate::multi_sphere::timing::measure(|costs| &mut costs.$field, || $operation)
        }
        #[cfg(not(feature = "sphere-pair-trace"))]
        {
            $operation
        }
    }};
}
pub(crate) use timed;
mod disk;
mod native;
mod pool;
mod recovery;
pub use disk::{InspectedCut, inspect_published_cut, inspect_published_path, read_published_cut};
pub use native::{serve, serve_forwarded, validate_forwarded_slot};
pub use pool::MultiSpherePool;
pub use recovery::{FenceReceipt, RecoveryMonitor, RecoveryPhase, RecoveryStatus};

use crate::articulation::{
    ArticulationError, ArticulationLinkSpec, ArticulationLinkState, ArticulationObservation,
    ArticulationSpec, NativeError,
};
use crate::moving::{
    MovingReaction, MovingSpec,
    worker::{self, Peer},
};
use serde::{Deserialize, Serialize};
use std::{
    io,
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, ArticulationError>;
fn invalid(msg: &'static str) -> ArticulationError {
    ArticulationError::Invalid(msg)
}

const WORKER_FLAG: &str = "--managed-multisphere-v2";
pub(crate) const MAGIC: [u8; 8] = *b"GZ2SWK02";
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MultiSphereSpec {
    pub hinge: ArticulationSpec,
    pub spheres: [ArticulationLinkSpec; 2],
}
/// Public fields of one committed two-companion archive, readable while its
/// coordinator is running. The physical tick resides in `observation.hinge.tick`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct PublishedCut {
    pub spec: MultiSphereSpec,
    pub width: u8,
    pub epoch: u64,
    pub generation: u64,
    pub owners: [u8; 3],
    pub pending_effort: Option<f64>,
    pub observation: MultiSphereObservation,
}

impl MultiSphereSpec {
    fn single(&self, i: usize) -> MovingSpec {
        MovingSpec {
            hinge: self.hinge,
            sphere: self.spheres[i],
        }
    }
    pub fn validate(&self) -> Result<()> {
        if self.spheres[0].id >= self.spheres[1].id {
            return Err(invalid(
                "expected two distinct spheres in stable identity order",
            ));
        }
        self.single(0).validate()?;
        self.single(1).validate()
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MultiSphereObservation {
    pub hinge: ArticulationObservation,
    pub spheres: [ArticulationLinkState; 2],
    /// Impulse on [sphere index][hinge link index]'s hinge link from that sphere.
    pub reactions: [[MovingReaction; 2]; 2],
    /// Impulse on the first sphere from the second sphere.
    pub sphere_reaction: MovingReaction,
}

// ArticulationObservation uses an optional JSON-only field. The native wire
// schema stores the seven independent hinge velocities/poses in fixed binary fields.
#[derive(Clone, Serialize, Deserialize)]
// The fixed-width bincode schema encodes a byte sequence and a byte buffer
// identically. Use bulk copies for native graphs on the pipe and in the cut.
struct HingePart {
    #[serde(with = "serde_bytes")]
    image: Vec<u8>,
    sample: GraphSample,
}
#[derive(Clone, Serialize, Deserialize)]
struct GraphSample {
    links: [ArticulationLinkState; 2],
    q: f64,
    qdot: f64,
    mask: u8,
    spheres: [ArticulationLinkState; 2],
    reactions: [[MovingReaction; 2]; 2],
    sphere_reaction: MovingReaction,
}
#[derive(Clone, Serialize, Deserialize)]
struct PairPart {
    #[serde(with = "serde_bytes")]
    image: Vec<u8>,
    spheres: [ArticulationLinkState; 2],
    reaction: MovingReaction,
    drained: bool,
}
#[derive(Clone, Serialize, Deserialize)]
struct Cut {
    width: u8,
    tick: u64,
    epoch: u64,
    generation: u64,
    owners: [u8; 3],
    hinge: HingePart,
    spheres: [ArticulationLinkState; 2],
    pending: Option<f64>,
    // v1 stores this prefix alone. A v2 archive appends the independent pair
    // receipt to the same prefix, atomically with both workers' observations.
    #[serde(skip)]
    pair: Option<PairPart>,
}
impl Cut {
    fn state(&self, joint_id: u64) -> MultiSphereObservation {
        MultiSphereObservation {
            hinge: ArticulationObservation {
                tick: self.tick,
                joint_id,
                links: self.hinge.sample.links,
                q: self.hinge.sample.q,
                qdot: self.hinge.sample.qdot,
                ground_reactions: None,
            },
            spheres: self.spheres,
            reactions: self.hinge.sample.reactions,
            sphere_reaction: self
                .pair
                .as_ref()
                .map_or(self.hinge.sample.sphere_reaction, |p| p.reaction),
        }
    }
    fn validate(&self, spec: &MultiSphereSpec) -> Result<()> {
        if !(3..=8).contains(&self.width)
            || self.owners.iter().any(|s| *s >= self.width)
            || self.epoch > self.generation
            || self.hinge.sample.mask != owner_mask(self.owners)
            || pair_owned(self.owners) != self.pair.is_some()
            || self.pair.as_ref().is_some_and(|p| p.image.is_empty() || p.spheres != self.spheres)
            || self.spheres.each_ref().map(|s| s.id) != spec.spheres.map(|s| s.id)
            || self.hinge.sample.links.each_ref().map(|s| s.id) != spec.hinge.links.map(|s| s.id)
            // Included spheres have two copies in the public cut: the native
            // observation and the coordinator's per-role table. A detached
            // sphere intentionally diverges from its disabled graph slot.
            || self.spheres.iter().enumerate().any(|(i, s)| {
                includes(self.hinge.sample.mask, i) && *s != self.hinge.sample.spheres[i]
            })
            || self.pending.map(f64::to_bits)
                != gz_physics_multifidelity::MultiFidelityBackend::moving_checkpoint_effort(
                    &self.hinge.image,
                )?.map(f64::to_bits)
        {
            return Err(invalid("invalid multi-sphere cut"));
        }
        Ok(())
    }
    fn accept_hinge(&mut self, h: HingePart) -> Result<()> {
        if h.sample.mask != owner_mask(self.owners) {
            return Err(invalid("worker installed unexpected membership"));
        }
        for i in 0..2 {
            if includes(h.sample.mask, i) {
                self.spheres[i] = h.sample.spheres[i];
            }
        }
        self.hinge = h;
        Ok(())
    }
}
fn pair_owned([h, a, b]: [u8; 3]) -> bool {
    a == b && h != a
}
fn owner_mask(o: [u8; 3]) -> u8 {
    u8::from(o[1] == o[0]) | (u8::from(o[2] == o[0]) << 1)
}
fn includes(mask: u8, i: usize) -> bool {
    mask & (1 << i) != 0
}
#[derive(Serialize, Deserialize)]
enum Seed {
    Hinge(#[serde(with = "serde_bytes")] Option<Vec<u8>>),
    Sphere(usize, ArticulationLinkState),
    Pair {
        #[serde(with = "serde_bytes")]
        image: Option<Vec<u8>>,
        spheres: [ArticulationLinkState; 2],
    },
}
#[derive(Serialize, Deserialize)]
enum Command {
    Fence,
    Install(Seed),
    Step,
    Torque(f64),
    MergePair(Box<PairPart>),
    SplitPair,
    PreviewPair,
    Plan {
        mask: u8,
        spheres: [ArticulationLinkState; 2],
    },
}
impl Command {
    fn name(&self) -> &'static str {
        match self {
            Command::Fence => "fence",
            Command::Install(_) => "install",
            Command::Step => "step",
            Command::Torque(_) => "torque",
            Command::MergePair(_) => "merge_pair",
            Command::SplitPair => "split_pair",
            Command::PreviewPair => "preview",
            Command::Plan { .. } => "plan",
        }
    }
}
#[derive(Serialize, Deserialize)]
enum Reply {
    Empty,
    Hinge(Box<HingePart>),
    Sphere(ArticulationLinkState),
    Pair(Box<PairPart>),
    SplitPair(Box<(HingePart, PairPart)>),
    PairEndpoint([[f64; 3]; 2]),
}

#[cfg(test)]
mod codec_tests {
    use super::{Seed, worker};
    use bincode::Options;

    #[test]
    fn native_image_buffers_keep_the_v2_bytes_and_frame_limit() {
        // Legacy Seed::Hinge is variant zero and its optional image is an
        // ordinary length-prefixed sequence of individual bytes.
        for image in [None, Some(vec![]), Some((0..=255).collect::<Vec<u8>>())] {
            let legacy = worker::encode(&(0_u32, &image)).unwrap();
            let Seed::Hinge(restored) = worker::codec().deserialize(&legacy).unwrap() else {
                panic!("wrong seed variant");
            };
            assert_eq!(restored, image);
            assert_eq!(worker::encode(&Seed::Hinge(image)).unwrap(), legacy);
        }

        let mut truncated = worker::encode(&Seed::Hinge(Some(vec![1, 2, 3]))).unwrap();
        truncated.pop();
        assert!(worker::codec().deserialize::<Seed>(&truncated).is_err());
        assert!(worker::encode(&Seed::Hinge(Some(vec![0; worker::LIMIT]))).is_err());
    }
}
