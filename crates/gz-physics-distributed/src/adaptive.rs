// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Contact forecasts for scene-wide and island-local native solve schedules.
use crate::{
    coordinator::Error,
    physics::{
        Body, BodyWrench, ComponentInfo, ForceMotion, GroundPlane, PendingImpulse, Primitive,
        SimulationStep, Snapshot, Tier, field_motion,
    },
    protocol::Observation,
};
use std::collections::BTreeMap;

mod ground;
mod pairs;

pub(crate) fn for_workers(
    workers: &[Observation],
    pending: &[PendingImpulse],
    gravity: [f64; 3],
    inputs: &[BodyWrench],
) -> Result<u8, Error> {
    let bodies: Vec<_> = workers
        .iter()
        .flat_map(|w| w.snapshot.bodies.iter().cloned())
        .collect();
    let world = &workers[0].snapshot;
    select(bodies, world.step, world.ground, pending, gravity, inputs)
}

fn length(v: [f64; 3]) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}

/// Bound every attitude, including the admitted quaternion normalization tolerance.
fn outer_radius(b: &Body) -> f64 {
    let radius = match b.primitive {
        Primitive::Box => 0.5 * length(b.size),
        Primitive::Sphere => 0.5 * b.size[0],
        Primitive::Capsule => 0.5 * b.size[0] + 0.5 * b.size[2],
        Primitive::Cylinder => 0.5 * b.size[0].hypot(b.size[2]),
    };
    radius * (1. + 8e-12)
}

fn may_clamp(body: &Body, change: [f64; 3]) -> bool {
    let limit = gz_physics_multifidelity::partition_linear_speed_limit();
    let initial = length(body.linear);
    let final_speed = length(std::array::from_fn(|i| body.linear[i] + change[i]));
    body.tier == Tier::Dynamic
        && (initial > limit
            || (change != [0.; 3] && initial.max(final_speed) >= limit * (1. - 32. * f64::EPSILON)))
}

struct SweptBody {
    body: Body,
    force: Option<ForceMotion>,
    lo: [f64; 3],
    hi: [f64; 3],
    at_limit: bool,
}

impl SweptBody {
    fn new(body: Body, force: Option<ForceMotion>, dt: f64) -> Result<Self, Error> {
        let radius =
            outer_radius(&body) + gz_physics_multifidelity::partition_prediction_distance();
        let change = force.map_or([0.; 3], |f| f.delta_velocity);
        let limit = gz_physics_multifidelity::partition_linear_speed_limit();
        let at_limit = may_clamp(&body, change);
        let mut lo = [0.; 3];
        let mut hi = [0.; 3];
        for axis in 0..3 {
            let center = body.position[axis];
            let travel = body.linear[axis] * dt;
            let acceleration = change[axis] * dt;
            let slack = 128.
                * f64::EPSILON
                * (1. + center.abs() + travel.abs() + acceleration.abs() + radius);
            let (min, max) = if at_limit {
                (-limit * dt, limit * dt)
            } else {
                (
                    travel.min(0.) + acceleration.min(0.),
                    travel.max(0.) + acceleration.max(0.),
                )
            };
            lo[axis] = center + min - radius - slack;
            hi[axis] = center + max + radius + slack;
            if !lo[axis].is_finite() || !hi[axis].is_finite() {
                return Err(Error::Invalid("invalid adaptive sweep bounds"));
            }
        }
        Ok(Self {
            body,
            force,
            lo,
            hi,
            at_limit,
        })
    }

    fn needs_refinement(&self, other: &Self, step: SimulationStep) -> Result<bool, Error> {
        if (self.body.tier != Tier::Dynamic && other.body.tier != Tier::Dynamic)
            || (0..3).any(|k| self.hi[k] < other.lo[k] || other.hi[k] < self.lo[k])
        {
            return Ok(false);
        }
        if self.at_limit || other.at_limit {
            return Ok(true);
        }
        if self.body.primitive != Primitive::Box || other.body.primitive != Primitive::Box {
            return pairs::curved_candidate(self, other, step.duration().as_secs_f64());
        }
        if self.force.is_some_and(|f| f.has_torque) || other.force.is_some_and(|f| f.has_torque) {
            return Ok(true);
        }
        let dv = |s: &Self| s.force.map_or([0.; 3], |f| f.delta_velocity);
        let (a, b) = (dv(self), dv(other));
        if a == b {
            // A common uniform acceleration cancels from relative translation.
            return self
                .body
                .predicts_box_contact(&other.body, step)
                .map_err(|_| Error::Invalid("invalid adaptive contact forecast"));
        }
        // The acceleration offset is bounded throughout the interval by
        // |relative acceleration| * dt^2, including staged native integration.
        // Expand uniformly in the box frame so the bound survives rotation.
        let d = (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]) * step.duration().as_secs_f64();
        let mut padded = self.body.clone();
        padded.size = padded.size.map(|x| x + 2. * d);
        padded
            .predicts_box_contact(&other.body, step)
            .map_err(|_| Error::Invalid("invalid forced adaptive forecast"))
    }
}

/// Select one 10 ms solve or ten 1 ms solves in a ground-free world of centered primitives
/// with a dynamic member. Inputs and twists are in world coordinates. Apply
/// queued impulses in submission order before forecasting. This predicts
/// free motion under the given field. An earlier impact can alter a later trajectory.
pub fn adaptive_substeps(
    bodies: &[Body],
    step: SimulationStep,
    pending: &[PendingImpulse],
    gravity: [f64; 3],
    inputs: &[BodyWrench],
) -> Result<u8, Error> {
    adaptive_substeps_with_ground(bodies, step, None, pending, gravity, inputs)
}

/// Select one common resolution for dynamic primitive pairs and contacts with the shared halfspace.
pub fn adaptive_substeps_with_ground(
    bodies: &[Body],
    step: SimulationStep,
    ground: Option<GroundPlane>,
    pending: &[PendingImpulse],
    gravity: [f64; 3],
    inputs: &[BodyWrench],
) -> Result<u8, Error> {
    // Direct callers can construct portable bodies themselves. The worker
    // entry point already has receipts from validated native bodies.
    for b in bodies {
        if !b.linear.iter().chain(&b.angular).all(|v| v.is_finite())
            || !b.friction.is_finite()
            || b.friction < 0.
            || (b.tier == Tier::Fixed
                && (b.mass.to_bits() != 0 || b.linear != [0.; 3] || b.angular != [0.; 3]))
            || (b.tier != Tier::Fixed && (!b.mass.is_finite() || b.mass <= 0.))
        {
            return Err(Error::Invalid("invalid adaptive body state"));
        }
        b.bounds()
            .map_err(|_| Error::Invalid("invalid adaptive body geometry"))?;
    }
    select(bodies.to_vec(), step, ground, pending, gravity, inputs)
}

fn select(
    mut bodies: Vec<Body>,
    step: SimulationStep,
    ground: Option<GroundPlane>,
    pending: &[PendingImpulse],
    gravity: [f64; 3],
    inputs: &[BodyWrench],
) -> Result<u8, Error> {
    if step != SimulationStep::Ms10 {
        return Err(Error::Invalid(
            "adaptive contact requires a 10 ms public step",
        ));
    }
    let forces: BTreeMap<_, _> = if inputs.is_empty() && gravity == [0.; 3] {
        BTreeMap::new()
    } else {
        field_motion(step, &bodies, gravity, inputs)
            .map_err(|_| Error::Invalid("invalid adaptive gravity or wrench"))?
            .into_iter()
            .map(|f| (f.id, f))
            .collect()
    };
    bodies.sort_unstable_by_key(|b| b.id);
    select_group(&mut bodies, step, ground, pending, &forces)
}

// The broadphase already joins warm solver members and everything reachable
// by a secondary collision. Apply the narrow forecast inside each such group.
// Independent contacts with the replicated plane do not create a graph edge.
pub(crate) fn group_refinements(
    scene: &Snapshot,
    components: &[ComponentInfo],
    groups: &[Vec<usize>],
    pending: &[PendingImpulse],
    motion: &[ForceMotion],
) -> Result<Vec<u64>, Error> {
    if scene.step != SimulationStep::Ms10 {
        return Err(Error::Invalid(
            "island refinement requires a 10 ms public step",
        ));
    }
    let forces: BTreeMap<_, _> = motion.iter().map(|&f| (f.id, f)).collect();
    let body = |id| {
        scene
            .bodies
            .binary_search_by_key(&id, |b| b.id)
            .map(|index| &scene.bodies[index])
            .map_err(|_| Error::Invalid("unknown refinement group member"))
    };
    let mut refined = Vec::new();
    let mut bodies = Vec::new();
    let mut local = Vec::new();
    for group in groups {
        if let [component] = group.as_slice()
            && let [id] = components[*component].ids.as_slice()
        {
            let Some(plane) = scene.ground else {
                continue;
            };
            if !pending.iter().any(|p| p.id == *id) {
                if ground::needs_refinement(
                    body(*id)?,
                    forces.get(id).copied(),
                    plane,
                    scene.step.duration().as_secs_f64(),
                )? {
                    refined.push(*id);
                }
                continue;
            }
        }
        bodies.clear();
        for &index in group {
            for &id in &components[index].ids {
                bodies.push(body(id)?.clone());
            }
        }
        bodies.sort_unstable_by_key(|b| b.id);
        local.clear();
        local.extend(
            pending
                .iter()
                .filter(|p| bodies.binary_search_by_key(&p.id, |b| b.id).is_ok())
                .cloned(),
        );
        if select_group(&mut bodies, scene.step, scene.ground, &local, &forces)? == 10 {
            for &i in group {
                refined.extend_from_slice(&components[i].ids);
            }
        }
    }
    refined.sort_unstable();
    Ok(refined)
}

fn select_group(
    bodies: &mut Vec<Body>,
    step: SimulationStep,
    ground: Option<GroundPlane>,
    pending: &[PendingImpulse],
    forces: &BTreeMap<u64, ForceMotion>,
) -> Result<u8, Error> {
    if bodies.windows(2).any(|p| p[0].id == p[1].id) {
        return Err(Error::Invalid("duplicate adaptive body ID"));
    }
    for p in pending {
        let index = bodies
            .binary_search_by_key(&p.id, |b| b.id)
            .map_err(|_| Error::Invalid("unknown queued body in adaptive forecast"))?;
        let b = &mut bodies[index];
        if b.tier != Tier::Dynamic {
            return Err(Error::Invalid(
                "queued adaptive impulse requires dynamic body",
            ));
        }
        for (v, j) in b.linear.iter_mut().zip(p.impulse) {
            *v += j * b.mass.recip();
        }
        let speed = b.linear.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !speed.is_finite() || speed > gz_physics_multifidelity::partition_linear_speed_limit() {
            return Err(Error::Invalid("invalid queued adaptive impulse"));
        }
    }
    let dt = step.duration().as_secs_f64();
    let mut sweeps = Vec::with_capacity(bodies.len());
    for body in bodies.drain(..) {
        let force = forces.get(&body.id).copied();
        if let Some(plane) = ground
            && ground::needs_refinement(&body, force, plane, dt)?
        {
            return Ok(10);
        }
        sweeps.push(SweptBody::new(body, force, dt)?);
    }
    if sweeps.len() < 2 {
        return Ok(1);
    }
    let spread = |axis: usize| {
        let (mut lo, mut hi, mut widths) = (f64::INFINITY, f64::NEG_INFINITY, 0.);
        for b in &sweeps {
            lo = lo.min(b.lo[axis]);
            hi = hi.max(b.hi[axis]);
            widths += b.hi[axis] - b.lo[axis];
        }
        (hi - lo) / widths
    };
    let axis = (0..3)
        .map(|i| (i, spread(i)))
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .unwrap()
        .0;
    sweeps.sort_unstable_by(|a, b| {
        a.lo[axis]
            .total_cmp(&b.lo[axis])
            .then(a.body.id.cmp(&b.body.id))
    });
    for (i, a) in sweeps.iter().enumerate() {
        for b in &sweeps[i + 1..] {
            if b.lo[axis] > a.hi[axis] {
                break;
            }
            if a.needs_refinement(b, step)? {
                return Ok(10);
            }
        }
    }
    Ok(1)
}

/// One policy selection for an entire distributed barrier.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Resolution {
    Coarse,
    Fine,
    Islands(Vec<u64>),
    /// One rate for the scene; allow native worlds to split at the completed barrier.
    SceneWithSeparation { fine: bool },
}

impl Resolution {
    pub(crate) fn command(
        &self,
        obs: &Observation,
        field: crate::physics::StepField,
        coarse: impl FnOnce(crate::physics::StepField) -> crate::protocol::Command,
    ) -> crate::protocol::Command {
        use crate::protocol::Command;
        let tick = obs.snapshot.tick;
        match self {
            Self::Coarse => coarse(field),
            Self::Fine => Command::StepRefined { tick, field },
            Self::SceneWithSeparation { fine } => Command::StepIslands {
                tick,
                field,
                refined: if *fine {
                    obs.generations.keys().copied().collect()
                } else {
                    Vec::new()
                },
            },
            Self::Islands(ids) => {
                let refined: Vec<_> = obs
                    .generations
                    .keys()
                    .copied()
                    .filter(|id| ids.binary_search(id).is_ok())
                    .collect();
                Command::StepIslands {
                    tick,
                    field,
                    refined,
                }
            }
        }
    }
}

pub(crate) fn valid_last_interval(
    tick: u64,
    step: SimulationStep,
    fine: bool,
    ids: Option<&[u64]>,
    width: usize,
) -> bool {
    (!(fine || ids.is_some()) || (tick > 0 && step == SimulationStep::Ms10))
        && ids.is_none_or(|ids| {
            fine != ids.is_empty()
                && ids.len() <= width * crate::protocol::MAX_BODY_IDS
                && ids.windows(2).all(|a| a[0] < a[1])
        })
}
