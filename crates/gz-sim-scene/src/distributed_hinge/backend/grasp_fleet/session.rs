// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Preserve a common Bevy clock and public control surface across isolated and fully coupled fleets.
use super::*;
use gz_physics_distributed::floating_tree::{
    AdaptiveGraspFleetPool, CoupledGraspFleetPool, FloatingTreeObservation, GraspFleetMerge,
    GraspFleetPairReaction, GraspReplayBudget,
};
use gz_physics_multifidelity::MovingReaction;
type Result<T> = std::result::Result<T, ArticulationError>;

/// The independent layouts enforce clearance between different assemblies before publication.
/// Coupled placement admits physical contacts across both complete assemblies.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub enum GraspFleetLayout {
    Packed,
    RoundRobin,
    Coupled,
    Adaptive,
    Warm,
    Replay { max_intervals: u32, max_events: u32 },
}
impl From<GraspFleetPlacement> for GraspFleetLayout {
    fn from(p: GraspFleetPlacement) -> Self {
        match p {
            GraspFleetPlacement::Packed => Self::Packed,
            GraspFleetPlacement::RoundRobin => Self::RoundRobin,
        }
    }
}
impl GraspFleetLayout {
    fn merge(self) -> Option<GraspFleetMerge> {
        match self {
            Self::Adaptive => Some(GraspFleetMerge::ClearMembers),
            Self::Warm => Some(GraspFleetMerge::WarmMembers),
            Self::Replay {
                max_intervals,
                max_events,
            } => Some(GraspFleetMerge::VerifiedReplay(GraspReplayBudget {
                max_intervals,
                max_events,
            })),
            _ => None,
        }
    }
    fn independent(self) -> Option<GraspFleetPlacement> {
        match self {
            Self::Packed => Some(GraspFleetPlacement::Packed),
            Self::RoundRobin => Some(GraspFleetPlacement::RoundRobin),
            Self::Coupled | Self::Adaptive | Self::Warm | Self::Replay { .. } => None,
        }
    }
    pub fn archive(self) -> &'static str {
        match self {
            Self::Coupled => "barrier.gzgraspcluster",
            Self::Adaptive | Self::Warm | Self::Replay { .. } => "barrier.gzadaptivegraspfleet",
            Self::Packed | Self::RoundRobin => "barrier.gzgraspfleet",
        }
    }
}
pub enum ManagedGraspFleet {
    Independent(Box<GraspFleetPool>),
    Coupled(Box<CoupledGraspFleetPool>),
    Adaptive(Box<AdaptiveGraspFleetPool>),
}
macro_rules! call {
    ($self:expr,$p:ident => $body:expr) => {
        match $self {
            Self::Independent($p) => $body,
            Self::Coupled($p) => $body,
            Self::Adaptive($p) => $body,
        }
    };
}
impl ManagedGraspFleet {
    pub fn open_scene(
        exe: &Path,
        dir: &Path,
        width: Option<u8>,
        spec: GraspFleetSpec,
        layout: GraspFleetLayout,
    ) -> Result<Self> {
        if let Some(policy) = layout.merge() {
            return Ok(Self::Adaptive(Box::new(
                AdaptiveGraspFleetPool::open_scene_with_policy(exe, dir, width, spec, policy)?,
            )));
        }
        Ok(match layout.independent() {
            Some(p) => Self::Independent(Box::new(GraspFleetPool::open_scene(
                exe, dir, width, spec, p,
            )?)),
            None => Self::Coupled(Box::new(CoupledGraspFleetPool::open_scene(
                exe, dir, width, spec,
            )?)),
        })
    }
    pub fn create(
        exe: &Path,
        dir: &Path,
        width: u8,
        spec: GraspFleetSpec,
        layout: GraspFleetLayout,
    ) -> Result<Self> {
        if let Some(policy) = layout.merge() {
            return Ok(Self::Adaptive(Box::new(
                AdaptiveGraspFleetPool::create_with_policy(exe, dir, width, spec, policy)?,
            )));
        }
        Ok(match layout.independent() {
            Some(p) => {
                Self::Independent(Box::new(GraspFleetPool::create(exe, dir, width, spec, p)?))
            }
            None => Self::Coupled(Box::new(CoupledGraspFleetPool::create(
                exe, dir, width, spec,
            )?)),
        })
    }
    pub fn resume(exe: &Path, dir: &Path) -> Result<Self> {
        if dir
            .join(GraspFleetLayout::Adaptive.archive())
            .try_exists()?
        {
            return Ok(Self::Adaptive(Box::new(AdaptiveGraspFleetPool::resume(
                exe, dir,
            )?)));
        }
        if dir.join(GraspFleetLayout::Coupled.archive()).try_exists()? {
            Ok(Self::Coupled(Box::new(CoupledGraspFleetPool::resume(
                exe, dir,
            )?)))
        } else {
            Ok(Self::Independent(Box::new(GraspFleetPool::resume(
                exe, dir,
            )?)))
        }
    }
    pub fn spec(&self) -> &GraspFleetSpec {
        call!(self,p=>p.spec())
    }
    pub fn owners(&self) -> &[u8] {
        call!(self,p=>p.owners())
    }
    pub fn width(&self) -> u8 {
        call!(self,p=>p.width())
    }
    pub fn tick(&self) -> u64 {
        call!(self,p=>p.tick())
    }
    pub fn epoch(&self) -> u64 {
        call!(self,p=>p.epoch())
    }
    pub fn generation(&self) -> u64 {
        call!(self,p=>p.generation())
    }
    pub fn layout(&self) -> GraspFleetLayout {
        match self {
            Self::Independent(p) => p.placement().into(),
            Self::Coupled(_) => GraspFleetLayout::Coupled,
            Self::Adaptive(p) => match p.merge_policy() {
                GraspFleetMerge::ClearMembers => GraspFleetLayout::Adaptive,
                GraspFleetMerge::WarmMembers => GraspFleetLayout::Warm,
                GraspFleetMerge::VerifiedReplay(b) => GraspFleetLayout::Replay {
                    max_intervals: b.max_intervals,
                    max_events: b.max_events,
                },
            },
        }
    }
    pub fn is_coupled(&self) -> bool {
        match self {
            Self::Independent(_) => false,
            Self::Coupled(_) => true,
            Self::Adaptive(p) => p.is_coupled(),
        }
    }
    pub fn joined_at(&self) -> Option<u64> {
        match self {
            Self::Adaptive(p) => p.joined_at(),
            _ => None,
        }
    }
    pub fn observation(&self, id: u64) -> Option<&FloatingTreeObservation> {
        call!(self,p=>p.observation(id))
    }
    pub fn payload_state(&self, id: u64) -> Option<&ArticulationLinkState> {
        call!(self,p=>p.payload_state(id))
    }
    pub fn payload_reactions(&self, id: u64) -> Option<&[MovingReaction]> {
        call!(self,p=>p.payload_reactions(id))
    }
    pub fn cross_reactions(&self) -> &[GraspFleetPairReaction] {
        match self {
            Self::Independent(_) => &[],
            Self::Coupled(p) => p.cross_reactions(),
            Self::Adaptive(p) => p.cross_reactions(),
        }
    }
    pub fn coupled_image(&self) -> Option<&[u8]> {
        match self {
            Self::Independent(_) => None,
            Self::Coupled(p) => p.image(),
            Self::Adaptive(p) if p.is_coupled() => {
                p.image(p.spec().assemblies[0].payload.tree.model_id)
            }
            Self::Adaptive(_) => None,
        }
    }
    pub fn velocity_motor(&self, id: u64) -> Option<JointVelocityMotor> {
        call!(self,p=>p.velocity_motor(id))
    }
    pub fn pending_joint_effort(&self, id: u64) -> Option<f64> {
        call!(self,p=>p.pending_joint_effort(id))
    }
    pub fn set_velocity_motor(&mut self, id: u64, m: Option<JointVelocityMotor>) -> Result<()> {
        call!(self,p=>p.set_velocity_motor(id,m))
    }
    pub fn queue_effort(&mut self, id: u64, e: f64) -> Result<()> {
        call!(self,p=>p.queue_effort(id,e))
    }
    pub fn check_interval(&self, epoch: u64, tick: u64) -> Result<()> {
        call!(self,p=>p.check_interval(epoch,tick))
    }
    pub fn advance_with_inputs_at(
        &mut self,
        epoch: u64,
        tick: u64,
        efforts: &[(u64, f64)],
        forces: &[(u64, [f64; 3])],
    ) -> Result<()> {
        call!(self,p=>p.advance_with_inputs_at(epoch,tick,efforts,forces))
    }
    pub fn transfer(&mut self, model: Option<u64>, dest: u8) -> Result<()> {
        match (self, model) {
            (Self::Adaptive(p), Some(id)) => p.transfer(id, dest),
            (Self::Adaptive(p), None) if p.is_coupled() => {
                p.transfer(p.spec().assemblies[0].payload.tree.model_id, dest)
            }
            (Self::Adaptive(_), None) => Err(ArticulationError::Invalid(
                "independent fleet transfer requires a model ID",
            )),
            (Self::Independent(p), Some(id)) => p.transfer(id, dest),
            (Self::Coupled(p), None) => p.transfer(dest),
            (Self::Independent(_), None) => Err(ArticulationError::Invalid(
                "fleet transfer requires a model ID",
            )),
            (Self::Coupled(_), Some(_)) => Err(ArticulationError::Invalid(
                "coupled transfer moves both models together",
            )),
        }
    }
    pub fn replace_worker(&mut self, slot: u8) -> Result<()> {
        call!(self,p=>p.replace_worker(slot))
    }
    pub fn reset(&mut self) -> Result<()> {
        call!(self,p=>p.reset())
    }
}
