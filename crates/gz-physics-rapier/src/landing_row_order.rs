// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Controlled permutation of existing rows for an isolated local landing audit.
use super::*;
use rapier3d_f64::geometry::{NEW_CONTACT_BIT, SolverContact};
use std::sync::atomic::{AtomicU64, Ordering};

#[derive(Clone, Copy, Debug)]
pub enum LandingRowOrder {
    /// Enable the same contact modifier but preserve every native row in place.
    Unchanged,
    /// Sort by the signed foot-local (x,y) quadrant, then by local position.
    SoleQuadrants,
    /// Exchange the two positive-x quadrants in that ordering.
    ExchangePositiveX,
}

pub(super) struct LandingRows {
    foot: ColliderHandle,
    pad: ColliderHandle,
    order: LandingRowOrder,
    visits: AtomicU64,
    permutations: AtomicU64,
}

impl LandingRows {
    pub(super) fn apply(&self, ctx: &mut ContactModificationContext) {
        let first = ctx.collider1 == self.foot && ctx.collider2 == self.pad;
        if !first && (ctx.collider2 != self.foot || ctx.collider1 != self.pad) {
            return;
        }
        let Some(contact) = ctx.rigid_mut() else {
            return;
        };
        if contact.solver_contacts.is_empty() {
            return;
        }
        self.visits.fetch_add(1, Ordering::Relaxed);
        if matches!(self.order, LandingRowOrder::Unchanged) {
            return;
        }
        let local = |row: &SolverContact| {
            let i = (row.contact_id[0] & !NEW_CONTACT_BIT) as usize;
            let point = &contact.manifold.points[i];
            if first {
                point.local_p1
            } else {
                point.local_p2
            }
        };
        let rank = |p: Vector| {
            let q = 2 * u8::from(p.x > 0.) + u8::from(p.y > 0.);
            if q >= 2 && matches!(self.order, LandingRowOrder::ExchangePositiveX) {
                q ^ 1
            } else {
                q
            }
        };
        let compare = |a: &SolverContact, b: &SolverContact| {
            let a = local(a);
            let b = local(b);
            rank(a)
                .cmp(&rank(b))
                .then_with(|| a.x.total_cmp(&b.x))
                .then_with(|| a.y.total_cmp(&b.y))
                .then_with(|| a.z.total_cmp(&b.z))
        };
        // Move whole rows. Each point ID, skin-adjusted anchor, material,
        // multiplier and counterpart manifold point keeps its native identity.
        if !contact
            .solver_contacts
            .is_sorted_by(|a, b| compare(a, b).is_le())
        {
            contact.solver_contacts.sort_by(compare);
            self.permutations.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LandingRowCounts {
    pub nonempty_manifolds_visited: u64,
    pub ordered_manifolds_changed: u64,
}

impl RapierBackend {
    /// Observe one qualified detached landing interval with a named solver-row
    /// ordering. Neither collision queries nor warm-start multipliers are reset.
    pub fn observe_landing_row_order(
        &mut self,
        pair: (BodyId, BodyId),
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        order: LandingRowOrder,
        observer: impl FnMut(ContactSubstep<'_>),
    ) -> Result<(ContactIntervalReceipt, LandingRowCounts), PhysicsError> {
        self.with_landing_rows(pair, dt, calls, iterations, order, |world| {
            world.observe_contact_interval(dt, calls, iterations, observer)
        })
    }

    /// Advance the identical diagnostic path without a per-substep observer.
    pub fn step_landing_row_order(
        &mut self,
        pair: (BodyId, BodyId),
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        order: LandingRowOrder,
    ) -> Result<(ContactIntervalReceipt, LandingRowCounts), PhysicsError> {
        self.with_landing_rows(pair, dt, calls, iterations, order, |world| {
            world.step_contact_interval(dt, calls, iterations)
        })
    }

    fn with_landing_rows(
        &mut self,
        pair: (BodyId, BodyId),
        dt: Duration,
        calls: u32,
        iterations: ContactSolverIterations,
        order: LandingRowOrder,
        advance: impl FnOnce(&mut Self) -> Result<ContactIntervalReceipt, PhysicsError>,
    ) -> Result<(ContactIntervalReceipt, LandingRowCounts), PhysicsError> {
        self.validate_contact_interval(dt, calls, iterations)?;
        self.airborne_restart_state(pair.0, pair.1)?;
        if dt.is_zero() {
            return Ok((
                Default::default(),
                LandingRowCounts {
                    nonempty_manifolds_visited: 0,
                    ordered_manifolds_changed: 0,
                },
            ));
        }
        let sole = self.world.bodies[self.bodies[&pair.0].0].colliders()[0];
        let support = self.world.bodies[self.bodies[&pair.1].0].colliders()[0];
        let original = self.world.colliders[sole].active_hooks();
        self.world.colliders[sole].set_active_hooks(original | ActiveHooks::MODIFY_SOLVER_CONTACTS);
        self.landing_rows = Some(LandingRows {
            foot: sole,
            pad: support,
            order,
            visits: AtomicU64::new(0),
            permutations: AtomicU64::new(0),
        });
        let receipt = advance(self);
        let rows = self
            .landing_rows
            .take()
            .expect("scoped landing observation");
        self.world.colliders[sole].set_active_hooks(original);
        Ok((
            receipt?,
            LandingRowCounts {
                nonempty_manifolds_visited: rows.visits.into_inner(),
                ordered_manifolds_changed: rows.permutations.into_inner(),
            },
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hooked_pair_keeps_anchors_and_applies_each_requested_order() {
        let mut incoming = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let make = |z, half_extents| BodySpec {
            position: [0., 0., z],
            shape: Shape::Box { half_extents },
            mass: Some(1.),
            velocity: [0.; 3],
        };
        let foot = incoming
            .spawn_articulation_link(
                make(0.06002, [0.18, 0.10, 0.06]),
                RotationSpec::default(),
                None,
            )
            .unwrap();
        let upper = incoming
            .spawn_articulation_link(
                make(0.20002, [0.05, 0.06, 0.06]),
                RotationSpec::default(),
                None,
            )
            .unwrap();
        incoming
            .add_limited_revolute(Some(foot), upper, [0., 0., 0.13], [1., 0., 0.], [0.; 2])
            .unwrap();
        incoming.enable_exact_revolute_locks(foot).unwrap();
        let pad = incoming
            .spawn_velocity_boundary(
                make(-0.06, [0.33, 0.18, 0.06]),
                RotationSpec::default(),
                None,
            )
            .unwrap();
        incoming
            .set_world_velocity(pad, [0., 0., 0.8], [0.; 3])
            .unwrap();
        incoming.step(Duration::from_nanos(1)).unwrap();
        let mut anchors = None;
        for (order, expected) in [
            (LandingRowOrder::SoleQuadrants, [0, 1, 2, 3]),
            (LandingRowOrder::ExchangePositiveX, [0, 1, 3, 2]),
        ] {
            let mut world = incoming.copy_airborne_landing(foot, pad, false).unwrap();
            let collider = world.world.bodies[world.bodies[&foot].0].colliders()[0];
            let hooks = world.world.colliders[collider].active_hooks();
            let dt = Duration::from_micros(500);
            assert!(
                world
                    .observe_landing_row_order((foot, pad), dt, 0, 4.into(), order, |_| panic!(
                        "invalid grid"
                    ))
                    .is_err()
            );
            assert_eq!(world.world.colliders[collider].active_hooks(), hooks);
            let mut at_first = None;
            let (receipt, visits) = world
                .observe_landing_row_order((foot, pad), dt, 8, 4.into(), order, |s| {
                    let rows = s.landing_solver_points(foot, pad).unwrap();
                    if at_first.is_none() && rows.iter().any(|r| r.impulse > 0.) {
                        let q = |p: [f64; 3]| 2 * u8::from(p[0] > 0.) + u8::from(p[1] > 0.);
                        assert_eq!(
                            rows.iter().map(|r| q(r.local_foot)).collect::<Vec<_>>(),
                            expected
                        );
                        let mut points = rows
                            .iter()
                            .map(|r| {
                                (
                                    q(r.local_foot),
                                    r.local_foot,
                                    r.local_pad,
                                    r.normal_on_foot,
                                    r.geometric_gap,
                                )
                            })
                            .collect::<Vec<_>>();
                        points.sort_by_key(|r| r.0);
                        at_first = Some(points);
                    }
                })
                .unwrap();
            assert_eq!(world.world.colliders[collider].active_hooks(), hooks);
            assert!(world.landing_rows.is_none());
            assert!(receipt.pairs.iter().any(|p| p.normal_impulse > 0.));
            assert!(visits.nonempty_manifolds_visited > 0);
            if let Some(points) = &anchors {
                assert_eq!(at_first.as_ref(), Some(points));
            }
            anchors = at_first;
            assert!(anchors.is_some());
        }
    }
}
