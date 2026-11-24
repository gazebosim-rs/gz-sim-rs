// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::adaptive::Resolution;
use crate::physics::{BodyWrench, ForceMotion, StepField, field_motion};

impl<P: Peer> Coordinator<P> {
    /// Inspect the selected barrier width without moving bodies or dispatching workers.
    pub fn adaptive_substeps(&self, gravity: [f64; 3], inputs: &[BodyWrench]) -> Result<u8, Error> {
        crate::adaptive::for_workers(&self.frame.workers, &self.queued, gravity, inputs)
    }

    /// Stable IDs of bodies that would receive ten solves at the next barrier.
    pub fn refined_island_bodies(
        &self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<Vec<u64>, Error> {
        Ok(self.island_routes(gravity, inputs)?.1)
    }

    pub(crate) fn force_motion(
        &self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<Vec<ForceMotion>, Error> {
        Self::force_motion_in(&self.frame, gravity, inputs)
    }

    fn force_motion_in(
        frame: &Frame,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<Vec<ForceMotion>, Error> {
        if inputs.is_empty() && gravity == [0.; 3] {
            return Ok(Vec::new());
        }
        let bodies: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|o| o.snapshot.bodies.iter().cloned())
            .collect();
        field_motion(frame.step(), &bodies, gravity, inputs)
            .map_err(|_| Error::Invalid("invalid native step wrenches"))
    }

    /// Check all pending force/torque contributions, current queued impulses,
    /// and prospective component placements without mutating a worker.
    pub fn wrench_routes(&self, inputs: &[BodyWrench]) -> Result<Vec<BodyRoute>, Error> {
        self.field_routes([0.; 3], inputs)
    }

    pub(crate) fn field_routes(
        &self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<Vec<BodyRoute>, Error> {
        self.plan_routes(&self.queued, None, &self.force_motion(gravity, inputs)?)
    }

    pub(super) fn validate_edited_owner(
        &self,
        owner: usize,
        observation: Observation,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<(), Error> {
        let mut frame = self.frame.clone();
        frame.workers[owner] = observation;
        let forced = Self::force_motion_in(&frame, gravity, inputs)?;
        self.plan_routes_in(&frame, &self.queued, None, &forced)
            .map(|_| ())
    }

    pub(crate) fn validate_edited_body(
        &self,
        body: &Body,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<(), Error> {
        let (owner, _) = self
            .frame
            .body(body.id)
            .ok_or(Error::Invalid("unknown edited body"))?;
        let owner = usize::from(owner);
        let mut observation = self.frame.workers[owner].clone();
        let i = observation
            .snapshot
            .bodies
            .binary_search_by_key(&body.id, |b| b.id)
            .unwrap();
        observation.snapshot.bodies[i] = body.clone();
        self.validate_edited_owner(owner, observation, gravity, inputs)
    }
    /// Hold each supplied COM force and spatial torque for all substeps of
    /// exactly one advancing barrier. Free flight uses a directional force bound.
    /// Co-location completes before either worker sees a wrench.
    pub fn advance_routed_with_wrenches(&mut self, inputs: &[BodyWrench]) -> Result<&Frame, Error> {
        self.advance_in_field([0.; 3], inputs)
    }

    /// Advance one barrier in a uniform world acceleration field with optional
    /// held COM forces and torques. Coordinates and accelerations are in SI units.
    pub fn advance_in_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<&Frame, Error> {
        if inputs.is_empty() && gravity == [0.; 3] {
            return self.advance_routed();
        }
        let (end, cursor, commands) = self.begin_field(gravity, inputs)?;
        self.complete_step(end, cursor, commands)
    }

    pub(super) fn begin_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<(u64, [usize; 2], [Command; 2]), Error> {
        self.begin_selected_field(gravity, inputs, false)
    }

    pub(super) fn begin_selected_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        refined: bool,
    ) -> Result<(u64, [usize; 2], [Command; 2]), Error> {
        let resolution = if refined {
            Resolution::Fine
        } else {
            Resolution::Coarse
        };
        self.begin_uniform_field(gravity, inputs, resolution)
    }

    pub(super) fn begin_uniform_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        resolution: Resolution,
    ) -> Result<(u64, [usize; 2], [Command; 2]), Error> {
        if !self.pending.as_ref().is_some_and(|p| {
            matches!(
                p.action,
                Action::Forced(_)
                    | Action::Field(_)
                    | Action::Refined(_)
                    | Action::Islands(_, _)
                    | Action::SceneWithSeparation(_, _)
            )
        }) {
            let routes = self.field_routes(gravity, inputs)?;
            self.execute_routes(routes, [1, 1])?;
        }
        self.begin_resolved_field(gravity, inputs, resolution)
    }

    pub(super) fn begin_island_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
    ) -> Result<(u64, [usize; 2], [Command; 2]), Error> {
        let ids = if let Some(Pending {
            action: Action::Islands(_, ids),
            ..
        }) = &self.pending
        {
            ids.clone()
        } else {
            let (routes, ids) = self.island_routes(gravity, inputs)?;
            self.execute_routes(routes, [1, 1])?;
            ids
        };
        self.begin_resolved_field(gravity, inputs, Resolution::Islands(ids))
    }

    pub(super) fn begin_resolved_field(
        &mut self,
        gravity: [f64; 3],
        inputs: &[BodyWrench],
        resolution: Resolution,
    ) -> Result<(u64, [usize; 2], [Command; 2]), Error> {
        let tick = self.frame.tick;
        let end = tick.checked_add(1).ok_or(Error::Capacity)?;
        let field = StepField {
            gravity,
            wrenches: inputs.to_vec(),
        };
        let action = match &resolution {
            Resolution::Fine => Action::Refined(field),
            Resolution::SceneWithSeparation { fine } => Action::SceneWithSeparation(field, *fine),
            Resolution::Islands(ids) => Action::Islands(field, ids.clone()),
            Resolution::Coarse if gravity == [0.; 3] => Action::Forced(field.wrenches),
            Resolution::Coarse => Action::Field(field),
        };
        let cursor = self.begin(action, [1, 1])?;
        let commands = [0, 1].map(|i| {
            let wrenches = inputs
                .iter()
                .filter(|w| self.frame.workers[i].generations.contains_key(&w.id))
                .cloned()
                .collect();
            resolution.command(
                &self.frame.workers[i],
                StepField { gravity, wrenches },
                |field| {
                    if gravity == [0.; 3] {
                        Command::StepWithWrenches {
                            tick,
                            wrenches: field.wrenches,
                        }
                    } else {
                        Command::StepInField { tick, field }
                    }
                },
            )
        });
        Ok((end, cursor, commands))
    }
}
