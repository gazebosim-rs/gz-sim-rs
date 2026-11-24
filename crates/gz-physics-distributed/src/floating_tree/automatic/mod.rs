// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A managed revolute tree and an XPBD material resolve their first geometric contact within a public interval.
use super::*;
use bincode::Options;
use gz_physics_xpbd::SoftBody;
use std::{io, path::Path, time::Duration};
pub(super) const WIRE: [u8; 8] = *b"GZATWK01";
const FILE: &str = crate::articulation::ledger::AUTOMATIC_MATERIAL_FILE;
const MAGIC: [u8; 8] = *b"GZATCU01";
const FILE_LIMIT: usize = 2 * 1024 * 1024;
pub(super) const NATIVE_LIMIT: usize = 32 * 1024;
mod construction;
mod pool;
mod rotor;
mod soft;
pub use construction::AutomaticConstruction;
pub use pool::{AutomaticMaterialPool, AutomaticPool, RotorMaterialPool};
pub use rotor::{AutomaticRotorMaterialSpec, RotorPadSpec};
pub(super) fn encode<T: Serialize>(v: &T, limit: usize) -> Result<Vec<u8>> {
    Ok(bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(limit as u64)
        .reject_trailing_bytes()
        .serialize(v)
        .map_err(io::Error::other)?)
}
pub(super) fn decode<T: serde::de::DeserializeOwned>(bytes: &[u8], limit: usize) -> Result<T> {
    Ok(bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(limit as u64)
        .reject_trailing_bytes()
        .deserialize(bytes)
        .map_err(io::Error::other)?)
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AutomaticMaterialSpec {
    pub tree: ContactTreeSpec,
    pub material_name: String,
    pub step_ns: u64,
    /// Spherical native receivers in authored sweep order.
    pub links: Vec<u64>,
    pub restitution: f64,
}
impl AutomaticMaterialSpec {
    pub(super) fn validate_tree(&self) -> Result<()> {
        use profile::sealed::Recipe;
        self.tree.validate_configuration()?;
        self.tree.tree.root()?;
        if !(1..=10_000_000).contains(&self.step_ns)
            || self.material_name.is_empty()
            || self.material_name.len() > 256
            || self.links.is_empty()
            || self.links.len() > contact::MAX_SITES
            || !(0.0..=1.0).contains(&self.restitution)
            || self.links.iter().collect::<BTreeSet<_>>().len() != self.links.len()
            || self
                .links
                .iter()
                .any(|id| self.tree.tree.link_index(*id).is_err())
        {
            return Err(invalid(
                "invalid automatic material interval, sweep receiver, name or restitution",
            ));
        }
        Ok(())
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AutomaticImpactReceipt {
    /// Tick before the public interval containing the event.
    pub input_tick: u64,
    pub head_ns: u64,
    pub tail_ns: u64,
    pub continuous_seconds: f64,
    pub rounding_distance: f64,
    /// Each receiver index and the material vertex IDs of the selected edge/face/point.
    pub features: Vec<(usize, Vec<usize>)>,
    /// Actual native link poses and twists at the event, before either solver's impulse.
    pub head_links: Vec<ArticulationLinkState>,
    /// Actual XPBD positions and velocities at the same private head, in material node order.
    pub head_particles: Vec<([f64; 3], [f64; 3])>,
    pub contact: MaterialContactReceipt,
}
#[derive(Clone, Serialize, Deserialize)]
pub(in crate::floating_tree) struct TimedCut {
    pub state: FloatingTreeObservation,
    pub last_ns: u64,
    pub consumed: bool,
    pub image: Option<Vec<u8>>,
}
impl TimedCut {
    pub(super) fn check(&self, s: &impl AutomaticConstruction) -> Result<()> {
        if !self
            .state
            .links
            .iter()
            .map(|l| l.id)
            .eq(s.link_specs().iter().map(|l| l.id))
            || !self
                .state
                .joints
                .iter()
                .map(|j| j.joint_id)
                .eq(s.sorted_joints())
            || self
                .state
                .joints
                .iter()
                .any(|j| !j.q.is_finite() || !j.qdot.is_finite() || j.pending_effort.is_some())
            || self.state.links.iter().any(|l| {
                !l.position
                    .iter()
                    .chain(&l.orientation)
                    .chain(&l.velocity)
                    .chain(&l.angular_velocity)
                    .all(|x| x.is_finite())
            })
            || !match &self.image {
                None => self.state.tick == 0 && self.last_ns == 0 && !self.consumed,
                Some(b) => {
                    self.state.tick > 0
                        && (1..=s.interval_ns()).contains(&self.last_ns)
                        && s.image_admitted(b)
                }
            }
        {
            return Err(invalid("invalid time-bound native material tree snapshot"));
        }
        Ok(())
    }
}
#[derive(Serialize, Deserialize)]
pub(in crate::floating_tree) enum Subrequest {
    Motion,
    Head {
        nanos: u64,
    },
    Sample {
        stamp: [u8; 32],
        sites: Vec<ArticulatedPointSite>,
    },
    Apply {
        stamp: [u8; 32],
        sites: Vec<ArticulatedPointSite>,
        seal: [u8; 32],
        impulses: Vec<[f64; 3]>,
    },
}
#[derive(Serialize, Deserialize)]
pub(in crate::floating_tree) enum Subreply {
    Motion {
        geometry: Vec<([f64; 3], f64)>,
        velocity: [f64; 3],
    },
    Head {
        geometry: Vec<([f64; 3], f64)>,
        state: FloatingTreeObservation,
        stamp: [u8; 32],
    },
    Sample {
        basis: ContactPointBasis,
        seal: [u8; 32],
    },
    Applied(ContactImpulseReceipt),
    Orbit {
        geometry: Vec<([f64; 3], f64)>,
        pivot: [f64; 3],
        axis: [f64; 3],
        angular_speed: f64,
    },
}
impl Subreply {
    fn into_shapes(
        self,
        rotor: bool,
    ) -> Result<Vec<gz_physics_xpbd::articulated_impact::SweptSphere>> {
        use gz_physics_xpbd::articulated_impact::{ImpactSphere, SphereMotion, SweptSphere};
        let (geometry, motion) = match self {
            Self::Motion { geometry, velocity } => (geometry, SphereMotion::Translate { velocity }),
            Self::Orbit {
                geometry,
                pivot,
                axis,
                angular_speed,
            } if rotor => (
                geometry,
                SphereMotion::Revolve {
                    pivot,
                    axis,
                    angular_speed,
                },
            ),
            _ => {
                return Err(invalid(
                    "native swept-sphere reply differs from negotiated construction",
                ));
            }
        };
        Ok(geometry
            .into_iter()
            .map(|(center, radius)| SweptSphere {
                sphere: ImpactSphere { center, radius },
                motion,
            })
            .collect())
    }
}
pub fn serve(slot: u8, lease: &Path) -> io::Result<()> {
    serve_as::<AutomaticMaterialSpec>(slot, lease)
}
pub fn serve_rotor(slot: u8, lease: &Path) -> io::Result<()> {
    serve_as::<AutomaticRotorMaterialSpec>(slot, lease)
}
fn serve_as<S: AutomaticConstruction>(slot: u8, lease: &Path) -> io::Result<()> {
    if slot >= 8 || !lease.is_absolute() {
        return Err(io::Error::other("invalid timed tree worker slot"));
    }
    crate::moving::worker::serve_profile(slot, lease, S::WIRE, |s: S| {
        s.validate_native()?;
        let mut state = crate::paged_native::State::<native::TimedWorld<S>>::default();
        Ok(move |tick, req, steps: &mut u64| state.handle(&s, (), tick, req, steps))
    })
}

#[cfg(test)]
mod crash_tests;
