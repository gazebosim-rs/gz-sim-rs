// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::EntityKind;
use parry3d_f64::{
    math::{Pose, Rotation, Vector},
    query::{self, ShapeCastOptions},
    shape::Cuboid,
};
use std::collections::BTreeMap;

struct BoxPart {
    name: String,
    collision: Entity,
    body: BodyId,
    shape: Cuboid,
}
impl BoxPart {
    fn gap(&self, pose: &Pose, wall: &BoxPart, wall_pose: &Pose) -> Result<f64> {
        let gap = query::distance(pose, &self.shape, wall_pose, &wall.shape)?.distance;
        if gap == 0.
            && let Some(contact) = query::contact(pose, &self.shape, wall_pose, &wall.shape, 0.)?
        {
            return Ok(contact.dist);
        }
        Ok(gap)
    }
    fn pose(&self, sim: &Simulation) -> Result<Pose> {
        let frame = sim
            .world()
            .get::<WorldPose>(self.collision)
            .ok_or("missing obstacle-route collision pose")?;
        Ok(Pose::from_parts(
            Vector::from_array(frame.translation),
            Rotation::from_array(frame.rotation),
        ))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(super) struct SweepHit {
    tick: usize,
    part: String,
    travel_m: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(super) struct PartClearance {
    pub(super) minimum_gap_m: f64,
    pub(super) nearest_tick: usize,
    pub(super) peak_normal_force_n: f64,
    pub(super) normal_impulse_ns: f64,
    pub(super) peak_force_vector_n: [f64; 3],
}
#[derive(Debug, Default, PartialEq, Serialize)]
pub(super) struct PassageOutcome {
    #[serde(skip_serializing_if = "is_zero")]
    pub(super) separated_box_rechecks: usize,
    pub(super) parts: BTreeMap<String, PartClearance>,
    pub(super) first_sweep_hit: Option<SweepHit>,
    pub(super) first_obstacle_reaction_tick: Option<usize>,
    pub(super) abort_tick: Option<usize>,
    pub(super) abort_on_measured_contact: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub(super) unsecured_dispatch_tick: Option<usize>,
    pub(super) post_abort_ticks: usize,
    pub(super) both_pads_supporting_after_abort_ticks: usize,
    pub(super) peak_carriage_travel_m: f64,
}

fn is_zero(value: &usize) -> bool {
    *value == 0
}

pub(super) struct Passage {
    wall: BoxPart,
    moving: Vec<BoxPart>,
    pub(super) outcome: PassageOutcome,
}
impl Passage {
    pub(super) fn body(&self) -> BodyId {
        self.wall.body
    }
    pub(super) fn bind(sim: &Simulation, sdf: &sdformat::Sdf) -> Result<Self> {
        let world = sdf
            .worlds
            .iter()
            .find(|world| world.name == "default")
            .ok_or("missing obstacle route world")?;
        let mut wall = None;
        let mut moving = Vec::new();
        for model in &world.models {
            let obstacle = model.name == "obstacle";
            if !obstacle && !matches!(model.name.as_str(), "parcel" | "gripper") {
                continue;
            }
            for link in &model.links {
                for c in &link.collisions {
                    let sdformat::Geometry::Box { size } = c.geometry else {
                        return Err("route requires cuboid collision geometry".into());
                    };
                    let name = format!("{}::{}", link.scoped_name, c.name);
                    let collision = sim
                        .world()
                        .iter_entities()
                        .find(|e| {
                            e.get::<EntityKind>() == Some(&EntityKind::Collision)
                                && e.get::<Name>().is_some_and(|n| n.0 == name)
                        })
                        .ok_or("missing obstacle-route collision")?
                        .id();
                    let box_part = BoxPart {
                        name: link.scoped_name.clone(),
                        collision,
                        body: part(sim, &format!("{}::{}", model.name, link.name))?.body(sim)?,
                        shape: Cuboid::new(Vector::from_array(size) * 0.5),
                    };
                    if obstacle {
                        if !model.is_static || wall.is_some() {
                            return Err("route requires one fixed obstacle box".into());
                        }
                        wall = Some(box_part);
                    } else {
                        if moving
                            .iter()
                            .any(|known: &BoxPart| known.body == box_part.body)
                        {
                            return Err("route requires one contact box per moving link".into());
                        }
                        moving.push(box_part);
                    }
                }
            }
        }
        let wall = wall.ok_or("missing fixed route obstacle")?;
        if moving.is_empty() {
            return Err("no moving collision boxes on route".into());
        }
        let pose = wall.pose(sim)?;
        let mut outcome = PassageOutcome::default();
        for part in &moving {
            let gap = part.gap(&part.pose(sim)?, &wall, &pose)?;
            outcome.parts.insert(
                part.name.clone(),
                PartClearance {
                    minimum_gap_m: gap,
                    nearest_tick: 0,
                    peak_normal_force_n: 0.,
                    normal_impulse_ns: 0.,
                    peak_force_vector_n: [0.; 3],
                },
            );
        }
        Ok(Self {
            wall,
            moving,
            outcome,
        })
    }
    pub(super) fn recheck(&mut self, sim: &mut Simulation) -> Result<()> {
        let dt = sim.step_duration();
        let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
        let solver = backend
            .downcast_mut::<MultiFidelityBackend>()
            .ok_or("missing local solver")?;
        for part in &self.moving {
            self.outcome.separated_box_rechecks +=
                usize::from(solver.recheck_separated_boxes(part.body, self.wall.body, dt)?);
        }
        Ok(())
    }

    pub(super) fn aborted(&self) -> bool {
        self.outcome.abort_tick.is_some()
    }

    pub(super) fn record_grip(
        &mut self,
        tick: usize,
        travel: f64,
        pad: [f64; 2],
        on_support: bool,
    ) {
        let carried = !on_support && pad.iter().all(|n| *n > 0.1);
        if tick == MOVE_START && !carried {
            self.outcome.unsecured_dispatch_tick = Some(tick);
            self.outcome.abort_tick.get_or_insert(tick);
        }
        self.outcome.peak_carriage_travel_m = self.outcome.peak_carriage_travel_m.max(travel);
        if self.aborted() {
            self.outcome.post_abort_ticks += 1;
            self.outcome.both_pads_supporting_after_abort_ticks += usize::from(carried);
        }
    }
    pub(super) fn observe(
        &mut self,
        sim: &Simulation,
        solver: &MultiFidelityBackend,
        tick: usize,
        remaining: Option<f64>,
        contact_stop: bool,
        dt: f64,
    ) -> Result<()> {
        let wall = self.wall.pose(sim)?;
        let sweep = remaining.filter(|span| *span != 0. && !self.aborted());
        let mut first: Option<(&str, f64)> = None;
        for moving in &self.moving {
            let pose = moving.pose(sim)?;
            // Measure the committed oriented solids, including a signed depth if they intersect.
            let gap = moving.gap(&pose, &self.wall, &wall)?;
            let reaction = solver
                .normal_reaction_on(moving.body, self.wall.body)?
                .ok_or("missing obstacle contact observation")?;
            let impulse = reaction.magnitude;
            let row = self
                .outcome
                .parts
                .get_mut(&moving.name)
                .expect("registered collision");
            if gap < row.minimum_gap_m {
                row.minimum_gap_m = gap;
                row.nearest_tick = tick;
            }
            if impulse / dt > row.peak_normal_force_n {
                row.peak_normal_force_n = impulse / dt;
                row.peak_force_vector_n = reaction.impulse.map(|value| value / dt);
            }
            row.normal_impulse_ns += impulse;
            if impulse > 0. {
                self.outcome
                    .first_obstacle_reaction_tick
                    .get_or_insert(tick);
                if self.outcome.abort_tick.is_none() {
                    self.outcome.abort_tick = Some(tick);
                    self.outcome.abort_on_measured_contact = true;
                }
            }
            if let Some(span) = sweep
                && let Some(hit) = query::cast_shapes(
                    &pose,
                    Vector::Y * span.signum(),
                    &moving.shape,
                    &wall,
                    Vector::ZERO,
                    &self.wall.shape,
                    ShapeCastOptions::with_max_time_of_impact(span.abs()),
                )?
                && first.is_none_or(|(_, d)| hit.time_of_impact < d)
            {
                first = Some((&moving.name, hit.time_of_impact));
            }
        }
        if let Some((part, travel)) = first {
            self.outcome
                .first_sweep_hit
                .get_or_insert_with(|| SweepHit {
                    tick,
                    part: part.to_owned(),
                    travel_m: travel,
                });
            if !contact_stop {
                self.outcome.abort_tick.get_or_insert(tick);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "obstacle/tests.rs"]
mod tests;
