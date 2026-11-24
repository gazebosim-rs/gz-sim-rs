// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A closed-form translation interval for worlds containing only prescribed
//! centered primitives. Each edited or late-added body starts its own interval.
//! Materialize all live poses before entry into the native contact solver.
use super::*;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum FleetClock {
    Disabled,
    Coupled,
    Translating(Duration),
}

impl RapierBackend {
    pub fn new_multifidelity(gravity: [f64; 3]) -> Result<Self, PhysicsError> {
        let mut backend = Self::new(gravity)?;
        backend.fleet_clock = FleetClock::Translating(Duration::ZERO);
        backend.allow_isolation = true;
        backend.fleet_dispatch = fleet_isolation::Dispatch::Candidate;
        Ok(backend)
    }

    pub(super) fn current_translation(&self, handle: RigidBodyHandle, body: &RigidBody) -> Vector {
        if let fleet_isolation::Dispatch::Active(i) = &self.fleet_dispatch {
            return i.translation(handle, body, &self.world);
        }
        if let FleetClock::Translating(now) = self.fleet_clock {
            if self.curves.authored()
                && let Some((pos, _)) = self.curves.at_handle(handle)
            {
                return pos;
            }
            let dt = now
                - self
                    .translation_epochs
                    .get(&handle)
                    .copied()
                    .unwrap_or_default();
            if !dt.is_zero() {
                return body.translation() + body.linvel() * dt.as_secs_f64();
            }
        }
        body.translation()
    }

    // Construction poses and twists stay immutable for reset. The native body
    // holds the current motion segment's origin, and only segments that start
    // after t=0 need an epoch entry.
    pub(super) fn record_translation_start(&mut self, id: BodyId, handle: RigidBodyHandle) {
        let FleetClock::Translating(epoch) = self.fleet_clock else {
            return;
        };
        self.translation_rays.take();
        if !epoch.is_zero() {
            self.translation_epochs.insert(handle, epoch);
        }
        if let Some(rows) = self.translation_rows.get_mut() {
            let row = TranslationRow::new(id, epoch, &self.world.bodies[handle]);
            if rows.last().is_none_or(|r| r.id < id) {
                rows.push(row);
            } else {
                let index = rows
                    .binary_search_by_key(&id, |r| r.id)
                    .expect("live translation row");
                rows[index] = row;
            }
        }
    }

    pub(super) fn materialize_fleet(&mut self) {
        if let FleetClock::Translating(now) = self.fleet_clock {
            if !now.is_zero() {
                for (handle, body) in self.world.bodies.iter_mut() {
                    if self.curves.authored()
                        && let Some((p, v)) = self.curves.at_handle(handle)
                    {
                        body.set_translation(p, true);
                        body.set_linvel(v, true);
                        continue;
                    }
                    let dt = now
                        - self
                            .translation_epochs
                            .get(&handle)
                            .copied()
                            .unwrap_or_default();
                    if !dt.is_zero() {
                        body.set_translation(
                            body.translation() + body.linvel() * dt.as_secs_f64(),
                            true,
                        );
                    }
                }
            }
            self.translation_epochs.clear();
            self.translation_rows.take();
            self.translation_rays.take();
            self.fleet_clock = FleetClock::Coupled;
        }
    }

    pub(super) fn restore_fleet_clock(&mut self) {
        self.fleet_resume_positions.clear();
        self.curves.reset(&self.bodies);
        self.fleet_dispatch = if self.allow_isolation {
            fleet_isolation::Dispatch::Candidate
        } else {
            fleet_isolation::Dispatch::Disabled
        };
        self.translation_rows.take();
        self.translation_rays.take();
        self.translation_epochs.clear();
        if self.fleet_clock == FleetClock::Disabled {
            return;
        }
        if self.curves.authored() {
            self.fleet_dispatch = fleet_isolation::Dispatch::Disabled;
        }
        self.fleet_clock = if self.bodies.values().all(|(_, spec, rotation, inertia, _)| {
            spec.kinematic
                && spec.can_sleep // Distributed boxes always use native integration.
                && rotation.angular_velocity == [0.0; 3]
                && inertia.is_none_or(|i| i.center == [0.0; 3])
        }) {
            FleetClock::Translating(Duration::ZERO)
        } else {
            FleetClock::Coupled
        };
    }
}

/// One immutable row at the start of the current translation interval.
/// Sorted live IDs keep memory proportional to live bodies after remove/spawn.
pub(super) struct TranslationRow {
    id: BodyId,
    epoch: Duration,
    start: BodyState,
}
pub(super) fn native_state(body: &RigidBody) -> BodyState {
    BodyState {
        position: body.translation().to_array(),
        velocity: body.linvel().to_array(),
        orientation: body.rotation().to_array(),
        angular_velocity: body.angvel().to_array(),
    }
}
impl TranslationRow {
    fn new(id: BodyId, epoch: Duration, body: &RigidBody) -> Self {
        Self {
            id,
            epoch,
            start: native_state(body),
        }
    }
}

impl RapierBackend {
    pub(super) fn state_from_entry(&self, (handle, spec, _, inertia, _): &BodyEntry) -> BodyState {
        let body = &self.world.bodies[*handle];
        if self.curves.authored()
            && matches!(self.fleet_clock, FleetClock::Translating(_))
            && let Some((p, v)) = self.curves.at_handle(*handle)
        {
            return BodyState {
                position: p.to_array(),
                velocity: v.to_array(),
                ..native_state(body)
            };
        }
        let mut origin = self.current_translation(*handle, body);
        if let (Some(_), Some(inertia)) = (spec.articulation, inertia) {
            origin -= *body.rotation() * Vector::from_array(inertia.center);
        }
        BodyState {
            position: origin.to_array(),
            velocity: match &self.fleet_dispatch {
                fleet_isolation::Dispatch::Active(i)
                    if spec.kinematic
                        && !self.world.colliders[body.colliders()[0]].is_enabled() =>
                {
                    i.velocity.to_array()
                }
                _ => body.linvel().to_array(),
            },
            orientation: body.rotation().to_array(),
            angular_velocity: body.angvel().to_array(),
        }
    }

    pub(super) fn read_body_states(
        &self,
        visit: &mut dyn FnMut(&mut dyn gz_physics::BodyStateLookup),
    ) {
        if let fleet_isolation::Dispatch::Active(i) = &self.fleet_dispatch {
            i.read(&self.world, visit);
            return;
        }
        let FleetClock::Translating(elapsed) = self.fleet_clock else {
            // Most scene archetypes traverse successive live IDs. Restart the
            // ordered cursor when the caller switches range or asks out of order.
            let mut entries = self.bodies.range(..).peekable();
            visit(&mut |id| {
                if entries.peek().is_none_or(|(key, _)| **key != id) {
                    entries = self.bodies.range(id..).peekable();
                }
                let (_, entry) = entries.next_if(|(key, _)| **key == id)?;
                Some(self.state_from_entry(entry))
            });
            return;
        };
        let rows = self.translation_rows.get_or_init(|| {
            self.bodies
                .iter()
                .map(|(&id, (handle, ..))| {
                    TranslationRow::new(
                        id,
                        self.translation_epochs
                            .get(handle)
                            .copied()
                            .unwrap_or_default(),
                        &self.world.bodies[*handle],
                    )
                })
                .collect()
        });
        let seconds = elapsed.as_secs_f64();
        let mut next = 0;
        let mut curves = self.curves.cursor();
        visit(&mut |id| {
            if rows.get(next).is_none_or(|r| r.id != id) {
                next = rows.binary_search_by_key(&id, |row| row.id).ok()?;
            }
            let row = &rows[next];
            next += 1;
            let mut state = row.start;
            if let Some((p, v)) = curves.as_mut().and_then(|c| c.get(id)) {
                state.position = p.to_array();
                state.velocity = v.to_array();
                return Some(state);
            }
            if row.epoch != elapsed {
                let dt = if row.epoch.is_zero() {
                    seconds
                } else {
                    (elapsed - row.epoch).as_secs_f64()
                };
                for (p, v) in state.position.iter_mut().zip(state.velocity) {
                    *p += v * dt;
                }
            }
            Some(state)
        });
    }
}
