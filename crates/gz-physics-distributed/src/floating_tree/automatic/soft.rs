// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::{
    prelude::{ResMut, Resource, World},
    schedule::Schedule,
};
use gz_physics_xpbd::{Particle, XpbdError};
#[derive(Resource)]
struct Material {
    body: SoftBody,
    origin: Vec<Particle>,
}
#[derive(Resource, Default)]
struct Interval {
    dt: Duration,
    start: Option<Duration>,
    failed: Option<XpbdError>,
}
fn solve(mut m: ResMut<Material>, mut interval: ResMut<Interval>) {
    let Material { body, origin } = &mut *m;
    let result = match interval.start {
        Some(start) => start
            .checked_add(interval.dt)
            .ok_or(XpbdError::InvalidStep)
            .and_then(|end| body.advance_articulated_flight_from(origin, start, end)),
        None => body.step(interval.dt, &[]),
    };
    interval.failed = result.err();
}
/// Each private subinterval runs its actual material physics system in a Bevy schedule.
/// Published time and the native counterpart remain owned by the globally committed cut.
pub(super) struct MaterialWorld {
    world: World,
    schedule: Schedule,
}
impl MaterialWorld {
    pub fn at(cold: &SoftBody, image: Option<&[u8]>) -> Result<Self> {
        let mut body = cold.clone();
        if let Some(b) = image {
            body.restore_articulated_material(b)?;
        }
        let mut world = World::new();
        world.insert_resource(Material {
            body,
            origin: cold.particles().to_vec(),
        });
        world.insert_resource(Interval::default());
        let mut schedule = Schedule::default();
        schedule.add_systems(solve);
        Ok(Self { world, schedule })
    }
    pub fn body(&self) -> &SoftBody {
        &self.world.resource::<Material>().body
    }
    pub fn body_mut(&mut self) -> &mut SoftBody {
        &mut self.world.resource_mut::<Material>().into_inner().body
    }
    pub fn advance(&mut self, dt: Duration, start: Option<Duration>) -> Result<()> {
        if dt.is_zero() {
            return Ok(());
        }
        {
            let mut i = self.world.resource_mut::<Interval>();
            i.dt = dt;
            i.start = start;
        }
        self.schedule.run(&mut self.world);
        if let Some(e) = self.world.resource_mut::<Interval>().failed.take() {
            return Err(e.into());
        }
        Ok(())
    }
}
