// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::prelude::Resource;
use sha2::{Digest, Sha256};
use std::str::FromStr;

#[derive(Clone, Copy, Debug, PartialEq, Resource, Serialize)]
pub(super) struct Pull {
    first_ms: usize,
    duration_ms: usize,
    downward_n: f64,
}
impl FromStr for Pull {
    type Err = &'static str;
    fn from_str(text: &str) -> std::result::Result<Self, Self::Err> {
        let mut fields = text.split(',');
        let first_ms = fields
            .next()
            .and_then(|v| v.parse::<usize>().ok())
            .ok_or("expected first pull tick")?;
        let duration_ms = fields
            .next()
            .and_then(|v| v.parse::<usize>().ok())
            .ok_or("expected duration in ms")?;
        let downward_n = fields
            .next()
            .and_then(|v| v.parse::<f64>().ok())
            .ok_or("expected downward force in N")?;
        if fields.next().is_some()
            || !(MOVE_START..=MOVE_END).contains(&first_ms)
            || duration_ms == 0
            || duration_ms > MOVE_END - first_ms + 1
            || !downward_n.is_finite()
            || downward_n < 0.
        {
            return Err(
                "supply first_ms,duration_ms,downward_n within the carry interval with finite nonnegative force",
            );
        }
        Ok(Self {
            first_ms,
            duration_ms,
            downward_n,
        })
    }
}
impl Pull {
    pub(super) fn install(self, sim: &mut Simulation, trial: Trial) -> Result<()> {
        if !trial.carry() {
            return Err("a parcel pull requires a monitored carry trial".into());
        }
        let dt_ns = sim.step_duration().as_nanos();
        if !(((self.first_ms - 1) as u128 * 1_000_000).is_multiple_of(dt_ns)
            && (self.duration_ms as u128 * 1_000_000).is_multiple_of(dt_ns))
        {
            return Err("align the physical pull start and duration to the SDF solve clock".into());
        }
        sim.world_mut().insert_resource(self);
        Ok(())
    }
}

#[derive(Debug, PartialEq, Serialize)]
pub(super) struct Measurements {
    pub(super) command: Pull,
    force_ticks: usize,
    incoming: Option<ContactState>,
    first_alarm: Option<ContactState>,
    repeated_alarm: Option<ContactState>,
}
impl Measurements {
    pub(super) fn new(command: Pull) -> Self {
        Self {
            command,
            force_ticks: 0,
            incoming: None,
            first_alarm: None,
            repeated_alarm: None,
        }
    }
    pub(super) fn before_step(
        &mut self,
        sim: &mut Simulation,
        tick: usize,
        parcel: Part,
        rail: Part,
    ) -> Result<()> {
        if tick == self.command.first_ms && self.incoming.is_none() {
            self.incoming = Some(ContactState::read(sim, parcel, rail)?);
        }
        if (self.command.first_ms..self.command.first_ms + self.command.duration_ms).contains(&tick)
            && self.command.downward_n > 0.
        {
            gz_sim_scene::queue_link_world_wrench(
                sim,
                parcel.id,
                [0., 0., -self.command.downward_n],
                [0.; 3],
            )?;
            self.force_ticks += 1;
        }
        Ok(())
    }
    pub(super) fn after_alarm(
        &mut self,
        sim: &Simulation,
        parcel: Part,
        rail: Part,
        first: bool,
        repeated: bool,
    ) -> Result<()> {
        if first {
            self.first_alarm = Some(ContactState::read(sim, parcel, rail)?);
        }
        if repeated {
            self.repeated_alarm = Some(ContactState::read(sim, parcel, rail)?);
        }
        Ok(())
    }
    pub(super) fn boundary(&self, tick: usize) -> bool {
        tick == self.command.first_ms - 1
            || tick == self.command.first_ms
            || tick == self.command.first_ms + self.command.duration_ms - 1
    }
}

#[derive(Debug, PartialEq, Serialize)]
struct ContactState {
    elapsed_ms: u128,
    parcel_position_m: [f64; 3],
    rail_position_m: f64,
    bodies: usize,
    contact_points: usize,
    body_and_contact_sha256: String,
}
impl ContactState {
    fn read(sim: &Simulation, parcel: Part, rail: Part) -> Result<Self> {
        let physics = sim.world().resource::<PhysicsBackend>();
        let solver = physics
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing carry contact state")?;
        let mut handles = sim
            .world()
            .iter_entities()
            .filter_map(|e| e.get::<BodyHandle>().map(|h| h.id()))
            .collect::<Vec<_>>();
        handles.sort_unstable();
        let mut digest = Sha256::new();
        digest.update((handles.len() as u64).to_le_bytes());
        for &id in &handles {
            let s = physics.state(id).ok_or("missing native link state")?;
            digest.update(id.raw().to_le_bytes());
            for value in s
                .position
                .into_iter()
                .chain(s.orientation)
                .chain(s.velocity)
                .chain(s.angular_velocity)
            {
                digest.update(value.to_le_bytes());
            }
        }
        let mut points = 0;
        for patch in solver.rigid_contact_patches() {
            digest.update(patch.a.raw().to_le_bytes());
            digest.update(patch.b.raw().to_le_bytes());
            digest.update((patch.points.len() as u64).to_le_bytes());
            for n in patch.normal_on_a {
                digest.update(n.to_le_bytes());
            }
            for point in patch.points {
                points += 1;
                for v in point
                    .lever_a
                    .into_iter()
                    .chain(point.lever_b)
                    .chain([point.normal_impulse])
                {
                    digest.update(v.to_le_bytes());
                }
                digest.update([u8::from(point.tangent_impulse_on_a.is_some())]);
                for v in point.tangent_impulse_on_a.into_iter().flatten() {
                    digest.update(v.to_le_bytes());
                }
            }
        }
        Ok(Self {
            elapsed_ms: sim.elapsed().as_millis(),
            parcel_position_m: parcel.pose(sim)?.translation,
            rail_position_m: sim
                .world()
                .get::<JointSample>(rail.entity)
                .ok_or("missing rail pulse sample")?
                .state
                .position,
            bodies: handles.len(),
            contact_points: points,
            body_and_contact_sha256: format!("{:x}", digest.finalize()),
        })
    }
}

#[cfg(test)]
#[path = "pull/tests.rs"]
mod tests;
