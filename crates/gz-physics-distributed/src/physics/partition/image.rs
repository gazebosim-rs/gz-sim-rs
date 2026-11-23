// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct PartitionImage {
    #[serde(default, skip_serializing_if = "NativeSolverProfile::is_standard")]
    pub(crate) profile: NativeSolverProfile,
    pub(crate) tick: u64,
    solvers: Vec<WarmCheckpoint>,
    pub(crate) pending: Vec<PendingImpulse>,
    #[serde(default, skip_serializing_if = "SimulationStep::is_default")]
    step: SimulationStep,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    ground: Option<GroundPlane>,
}

impl PartitionImage {
    pub(crate) fn native_wire_payloads(&mut self) -> impl ExactSizeIterator<Item = &mut Vec<u8>> {
        self.solvers
            .iter_mut()
            .map(WarmCheckpoint::native_wire_payload)
    }
}

impl Partition {
    pub(crate) fn image(&self) -> Result<PartitionImage, WorldError> {
        self.healthy()?;
        let solvers = self
            .solvers
            .iter()
            .map(|s| {
                s.solver_image(&[], self.profile)
                    .map_err(|e| WorldError::ImageCapture {
                        members: s.index.keys().copied().collect(),
                        source: Box::new(e),
                    })
            })
            .collect::<Result<_, _>>()?;
        Ok(PartitionImage {
            profile: self.profile,
            tick: self.committed.tick,
            step: self.committed.step,
            ground: self.committed.ground,
            solvers,
            pending: self.pending.clone(),
        })
    }

    pub(crate) fn from_image(image: &PartitionImage) -> Result<Self, WorldError> {
        if image.solvers.len() > MAX_PARTITION_BODIES || image.pending.len() > MAX_PENDING_IMPULSES
        {
            return Err(WorldError::Checkpoint);
        }
        let mut members = std::collections::BTreeSet::new();
        let mut solvers = Vec::with_capacity(image.solvers.len());
        for solver in &image.solvers {
            if solver.snapshot.ground != image.ground
                || solver.snapshot.step != image.step
                || solver.snapshot.tick != image.tick
                || !solver.pending.is_empty()
                || solver.snapshot.bodies.iter().any(|b| !members.insert(b.id))
                || members.len() > MAX_PARTITION_BODIES
            {
                return Err(WorldError::Checkpoint);
            }
            solvers.push(Island::restore_solver_with_profile(solver, image.profile)?);
        }
        let mut partition = Self {
            profile: image.profile,
            solvers,
            components: Vec::new(),
            committed: Snapshot {
                step: image.step,
                ground: image.ground,
                ..Default::default()
            },
            pending: Vec::new(),
            fault: None,
            probe: None,
            helper_allowed: false,
            helper: None,
        };
        partition.publish(image.tick);
        for p in &image.pending {
            partition.queue_impulse(p.id, p.impulse)?;
        }
        Ok(partition)
    }
}
