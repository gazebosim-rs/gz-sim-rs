// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::Result;
use bevy_ecs::prelude::{Res, ResMut, Resource};
use clap::ValueEnum;
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::{ContactIntervalReceipt, MultiFidelityBackend};
use gz_sim_core::{Phase, Simulation, UpdateInfo};
use gz_sim_scene::primary_contact;
use serde::Serialize;
use std::time::Duration;

pub const DT: Duration = Duration::from_micros(200);
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum, Serialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum Grid {
    Adaptive,
    Fine,
}

#[derive(Resource, Default)]
struct FineTick(ContactIntervalReceipt);

pub fn configure(sim: &mut Simulation, grid: Grid) -> Result<()> {
    if grid == Grid::Fine {
        assert_eq!(
            sim.remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)?,
            1
        );
        sim.world_mut().init_resource::<FineTick>();
        sim.add_systems(
            Phase::PhysicsStep,
            |t: Res<UpdateInfo>, mut p: ResMut<PhysicsBackend>, mut receipt: ResMut<FineTick>| {
                receipt.0 = p
                    .downcast_mut::<MultiFidelityBackend>()
                    .unwrap()
                    .step_contact_interval(t.dt, 200, 4)
                    .unwrap();
            },
        );
    }
    Ok(())
}

pub fn last(sim: &Simulation) -> Option<&ContactIntervalReceipt> {
    sim.world()
        .get_resource::<FineTick>()
        .map(|f| &f.0)
        .or_else(|| {
            primary_contact::last_refinement(sim)
                .filter(|r| r.sim_time == sim.elapsed())
                .map(|r| &r.interval)
        })
}
