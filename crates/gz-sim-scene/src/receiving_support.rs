// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Detect sustained load transfer between two named rigid links after a local physics solve.
use crate::{
    BodyHandle, EntityKind, InputError, Velocity, WorldPose, initial, world_spawn::RuntimeBodyOwner,
};
use bevy_ecs::{prelude::*, world::WorldId};
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{UpdateInfo, entity_by_id};
use std::{collections::VecDeque, time::Duration};

#[derive(Clone, Copy, Debug)]
pub struct SupportCriteria {
    /// Positive expected weight in newtons and required uninterrupted qualifying time.
    pub weight_n: f64,
    pub dwell: Duration,
    /// World-space upward direction, normalized at construction.
    pub up: [f64; 3],
    /// Minimum projected / total compressive impulse, in [0, 1].
    pub minimum_alignment: f64,
    pub minimum_sample_fraction: f64,
    pub mean_weight_fraction: [f64; 2],
    pub max_relative_normal_speed_m_s: f64,
    pub max_receiver_speed_m_s: f64,
    pub max_receiver_spin_rad_s: f64,
}
impl SupportCriteria {
    /// Parameters qualified on a stationary horizontal receiving shelf.
    pub fn for_weight(weight_n: f64, dwell: Duration) -> Self {
        Self {
            weight_n,
            dwell,
            up: [0., 0., 1.],
            minimum_alignment: 0.999,
            minimum_sample_fraction: 0.2,
            mean_weight_fraction: [0.94, 1.25],
            max_relative_normal_speed_m_s: 0.01,
            max_receiver_speed_m_s: 0.002,
            max_receiver_spin_rad_s: 0.01,
        }
    }
    fn validate(&mut self) -> Result<(), InputError> {
        let up = DVec3::from(self.up)
            .try_normalize()
            .ok_or(InputError::InvalidValue)?;
        let [low, high] = self.mean_weight_fraction;
        if self.dwell.is_zero()
            || !self.weight_n.is_finite()
            || self.weight_n <= 0.
            || !(0.0..=1.).contains(&self.minimum_alignment)
            || ![
                self.minimum_sample_fraction,
                low,
                high,
                self.max_relative_normal_speed_m_s,
                self.max_receiver_speed_m_s,
                self.max_receiver_spin_rad_s,
            ]
            .into_iter()
            .all(|v| v.is_finite() && v >= 0.)
            || low == 0.
            || low > high
            || self.minimum_sample_fraction > high
            || !(high * self.weight_n).is_finite()
        {
            return Err(InputError::InvalidValue);
        }
        self.up = up.to_array();
        Ok(())
    }
}

#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct SupportReading {
    /// False before the first solve or when an imported checkpoint lacks a signed receipt.
    pub measurement_available: bool,
    /// Signed compressive force along the configured upward direction.
    pub normal_force_n: f64,
    pub relative_normal_speed_m_s: f64,
    pub face_loaded: bool,
    pub receiver_moving: bool,
    /// Time represented in the current contiguous window, capped at the dwell.
    pub sustained: Duration,
    /// Present once the complete dwell is covered by qualifying samples.
    pub mean_force_n: Option<f64>,
    pub ready: bool,
}

#[derive(Component)]
pub struct ReceivingSupport {
    owner: WorldId,
    bindings: [(u64, BodyId); 2],
    last_receiver_pose: Option<(DVec3, DQuat)>,
    criteria: SupportCriteria,
    history: VecDeque<(Duration, f64)>,
    span: Duration,
    impulse: f64,
    rounding: f64,
    stamp: Option<(u64, u64)>,
    last: SupportReading,
}

fn binding(world: &World, id: u64) -> Result<(Entity, BodyId), InputError> {
    let e = entity_by_id(world, id).ok_or(InputError::MissingEntity(id))?;
    let b = world
        .get::<BodyHandle>(e)
        .ok_or(InputError::InvalidPhysics)?
        .id();
    if world.get::<EntityKind>(e) != Some(&EntityKind::Link)
        || !(initial::owns_loaded_body(world, id, b)
            || world
                .get::<RuntimeBodyOwner>(e)
                .is_some_and(|owner| owner.matches(id, b)))
    {
        return Err(InputError::InvalidPhysics);
    }
    Ok((e, b))
}

impl ReceivingSupport {
    /// Bind public link entity IDs in this world. Sampling never actuates either link.
    pub fn new(
        world: &World,
        payload: u64,
        receiver: u64,
        mut criteria: SupportCriteria,
    ) -> Result<Self, InputError> {
        criteria.validate()?;
        if !initial::matches_physics_owner(world)
            || world
                .get_resource::<PhysicsBackend>()
                .and_then(|b| b.downcast_ref::<MultiFidelityBackend>())
                .is_none()
        {
            return Err(InputError::InvalidPhysics);
        }
        if payload == receiver {
            return Err(InputError::InvalidValue);
        }
        let bindings = [
            (payload, binding(world, payload)?.1),
            (receiver, binding(world, receiver)?.1),
        ];
        Ok(Self {
            owner: world.id(),
            bindings,
            last_receiver_pose: None,
            criteria,
            history: VecDeque::new(),
            span: Duration::ZERO,
            impulse: 0.,
            rounding: 0.,
            stamp: None,
            last: SupportReading::default(),
        })
    }

    fn forget(&mut self) {
        self.history.clear();
        self.span = Duration::ZERO;
        self.impulse = 0.;
        self.rounding = 0.;
    }
    // Compensated moving integral. Each sample enters and leaves the deque once.
    fn add_impulse(&mut self, value: f64) {
        let adjusted = value - self.rounding;
        let next = self.impulse + adjusted;
        self.rounding = (next - self.impulse) - adjusted;
        self.impulse = next;
    }

    /// Read once after the local physics step and pose publication. Duplicate
    /// reads of a committed tick are idempotent. Reset, skipped ticks, invalid
    /// bindings and motion of the receiver invalidate the previous dwell.
    pub fn sample(&mut self, world: &World) -> Result<SupportReading, InputError> {
        let result = self.read(world);
        if result.is_err() {
            self.forget();
            self.stamp = None;
            self.last_receiver_pose = None;
            self.last = SupportReading::default();
        }
        result
    }

    fn read(&mut self, world: &World) -> Result<SupportReading, InputError> {
        if self.owner != world.id() || !initial::matches_physics_owner(world) {
            return Err(InputError::InvalidPhysics);
        }
        let clock = world
            .get_resource::<UpdateInfo>()
            .ok_or(InputError::InvalidPhysics)?;
        let resolve = |(id, expected)| -> Result<_, InputError> {
            let e = entity_by_id(world, id).ok_or(InputError::MissingEntity(id))?;
            if world.get::<BodyHandle>(e).map(BodyHandle::id) != Some(expected) {
                return Err(InputError::InvalidPhysics);
            }
            Ok((e, expected))
        };
        let [(p, pid), (s, sid)] = [resolve(self.bindings[0])?, resolve(self.bindings[1])?];
        let solver = world
            .get_resource::<PhysicsBackend>()
            .and_then(|b| b.downcast_ref::<MultiFidelityBackend>())
            .ok_or(InputError::InvalidPhysics)?;
        let state = |e| -> Result<_, InputError> {
            let pos = world
                .get::<WorldPose>(e)
                .ok_or(InputError::InvalidPhysics)?;
            let v = world.get::<Velocity>(e).ok_or(InputError::InvalidPhysics)?;
            Ok((
                DVec3::from(pos.translation),
                DQuat::from_array(pos.rotation),
                DVec3::from(v.linear),
                DVec3::from(v.angular),
            ))
        };
        let (p0, _, pv, _) = state(p)?;
        let (s0, sq, sv, sw) = state(s)?;
        let c = self.criteria;
        let up = DVec3::from(c.up);
        let normal_speed = (pv - sv - sw.cross(p0 - s0)).dot(up);
        let stamp = (clock.epoch, clock.iteration);
        let same = self.stamp == Some(stamp);
        let continuous = self.stamp.is_some_and(|(epoch, tick)| {
            epoch == clock.epoch && tick.checked_add(1) == Some(clock.iteration)
        });
        let h = if same { 0. } else { clock.dt.as_secs_f64() };
        let moved = (same || continuous)
            && self.last_receiver_pose.is_some_and(|(p, q)| {
                let turn = q.conjugate() * sq;
                let angle = 2.
                    * DVec3::new(turn.x, turn.y, turn.z)
                        .length()
                        .atan2(turn.w.abs());
                s0.distance(p) > c.max_receiver_speed_m_s * h + 1e-12
                    || angle > c.max_receiver_spin_rad_s * h + 1e-12
            });
        let moving = moved
            || sv.length() > c.max_receiver_speed_m_s
            || sw.length() > c.max_receiver_spin_rad_s;
        if !normal_speed.is_finite() || !sq.is_normalized() || !sv.is_finite() || !sw.is_finite() {
            return Err(InputError::InvalidPhysics);
        }
        let mut reaction = if clock.iteration != 0 && !clock.dt.is_zero() {
            solver.normal_reaction_on(pid, sid)?
        } else {
            None
        };
        if let Some(r) = &mut reaction
            && let Some(interval) = crate::primary_contact::current_interval(world, clock)
        {
            let key = (pid.min(sid), pid.max(sid));
            if let Ok(i) = interval.pairs.binary_search_by_key(&key, |p| (p.a, p.b)) {
                let pair = &interval.pairs[i];
                let sign = if pair.a == pid { 1. } else { -1. };
                r.impulse = pair.normal_impulse_on_a.map(|v| sign * v);
                r.magnitude = pair.normal_impulse;
                // A transient impact can deliver impulse and already be unloaded at tick end.
                r.loaded_at_end &= pair.active_at_end;
            } else {
                *r = Default::default();
            }
        }
        if same
            && !moving
            && normal_speed.abs() <= c.max_relative_normal_speed_m_s
            && self.last.measurement_available == reaction.is_some()
        {
            return Ok(self.last);
        }
        if !continuous {
            self.forget();
        }
        self.stamp = Some(stamp);
        self.last_receiver_pose = Some((s0, sq));
        let mut reading = SupportReading {
            relative_normal_speed_m_s: normal_speed,
            receiver_moving: moving,
            ..SupportReading::default()
        };
        if let Some(r) = reaction {
            reading.measurement_available = true;
            let projected = DVec3::from(r.impulse).dot(up);
            reading.normal_force_n = projected / clock.dt.as_secs_f64();
            reading.face_loaded = r.loaded_at_end
                && r.magnitude > 0.
                && projected >= c.minimum_alignment * r.magnitude;
            if reading.face_loaded
                && !moving
                && normal_speed.abs() <= c.max_relative_normal_speed_m_s
                && reading.normal_force_n >= c.minimum_sample_fraction * c.weight_n
            {
                self.history.push_back((clock.dt, reading.normal_force_n));
                self.span += clock.dt;
                self.add_impulse(projected);
                while self
                    .history
                    .front()
                    .is_some_and(|(dt, _)| self.span - *dt >= c.dwell)
                {
                    let (time, force) = self.history.pop_front().unwrap();
                    self.span -= time;
                    self.add_impulse(-time.as_secs_f64() * force);
                }
                reading.sustained = self.span.min(c.dwell);
                if self.span >= c.dwell {
                    let excess = self.span - c.dwell;
                    let mean = (self.impulse - excess.as_secs_f64() * self.history[0].1)
                        / c.dwell.as_secs_f64();
                    reading.mean_force_n = Some(mean);
                    reading.ready = (c.mean_weight_fraction[0] * c.weight_n
                        ..=c.mean_weight_fraction[1] * c.weight_n)
                        .contains(&mean);
                }
            } else {
                self.forget();
            }
        } else {
            self.forget();
        }
        self.last = reading;
        Ok(reading)
    }
}
