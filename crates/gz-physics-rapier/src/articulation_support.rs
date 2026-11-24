// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Keep the native tracked points of a settled link on independently loaded stationary supports.
//! No cached impulse is injected as a force or generalized velocity change.
#[path = "articulation_support/registered.rs"]
mod registered;
use super::*;
use rapier3d_f64::geometry::ContactPair;
pub(crate) use registered::ParkedSharedSupport;

struct Surface {
    handle: ColliderHandle,
    parent: RigidBodyHandle,
    pose: Pose,
    shape: SharedShape,
    material: ColliderMaterial,
    skin: f64,
}

impl Surface {
    fn capture(world: &PhysicsWorld, handle: ColliderHandle) -> Option<Self> {
        let c = world.colliders.get(handle)?;
        let parent = c.parent()?;
        let pose = world.bodies.get(parent)?.position() * c.position_wrt_parent()?;
        Some(Self {
            handle,
            parent,
            pose,
            shape: c.shared_shape().clone(),
            material: *c.material(),
            skin: c.contact_skin(),
        })
    }

    fn current_pose(&self, world: &PhysicsWorld) -> Option<Pose> {
        let c = world.colliders.get(self.handle)?;
        if c.parent() != Some(self.parent)
            || !Arc::ptr_eq(&self.shape.0, &c.shared_shape().0)
            || self.material != *c.material()
            || self.skin != c.contact_skin()
        {
            return None;
        }
        Some(world.bodies.get(self.parent)?.position() * c.position_wrt_parent()?)
    }
}

/// First condition that invalidated the demotion-time support during a prescribed interval.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SupportExpiry {
    SurfaceChanged,
    SolverChanged,
    FootprintMoved,
    AnchorOutsideFace,
    NoLoadedPoint,
    CompetingContact,
}

/// Instantaneous qualification readback for a captured link–support pair.
#[derive(Clone, Copy, Debug)]
pub struct HeldSupportStatus {
    pub expiry: Option<SupportExpiry>,
    /// Number of originally loaded points close enough to a box face's edges to cross.
    pub edge_anchors: usize,
    /// Signed maximum distance past a face boundary for those original link-local points.
    /// Positive means outside, negative means still on the face.
    pub max_face_overhang_m: Option<f64>,
}

struct EdgeAnchor {
    link_point: Vector,
    normal_axis: usize,
    at_capture: Vector,
}

impl EdgeAnchor {
    fn tangent_max(&self, offset: Vector) -> f64 {
        offset[(self.normal_axis + 1) % 3].max(offset[(self.normal_axis + 2) % 3])
    }

    fn overhang(&self, link_in_ground: &Pose, extents: Vector) -> f64 {
        self.tangent_max((*link_in_ground * self.link_point).abs() - extents)
    }

    fn crosses_face(&self, link_in_ground: &Pose, extents: Vector, roundoff: f64) -> bool {
        // Anchors are geometric witnesses from the previous solve. A contact
        // skin or solver residual can already put the projected link point
        // just beyond the nominal face at capture. Count only a new crossing
        // (or further outward motion) on that same side, separately per axis.
        let current = *link_in_ground * self.link_point;
        let distance =
            (current - extents.max(self.at_capture)).max((-extents).min(self.at_capture) - current);
        self.tangent_max(distance) > roundoff
    }
}

pub(super) struct SupportHold {
    link: Surface,
    ground: Surface,
    gravity: Vector,
    integration: IntegrationParameters,
    radius_about_link_origin: f64,
    expiry: Option<SupportExpiry>,
    edge_anchors: Vec<EdgeAnchor>,
    edge_roundoff: f64,
}

impl SupportHold {
    pub(super) fn expire_moving_support(&mut self, body: RigidBodyHandle) {
        if self.ground.parent == body {
            self.expiry.get_or_insert(SupportExpiry::SurfaceChanged);
        }
    }

    pub(super) fn is_pair(&self, link: ColliderHandle, floor: ColliderHandle) -> bool {
        self.link.handle == link && self.ground.handle == floor
    }

    fn capture(world: &PhysicsWorld, handle: ColliderHandle, pair: &ContactPair) -> Option<Self> {
        let ground = if pair.collider1 == handle {
            pair.collider2
        } else {
            pair.collider1
        };
        let floor = &world.colliders[ground];
        if !support_loss::stationary_support(&world.bodies[floor.parent()?])
            || !pair
                .solver_manifolds()
                .iter()
                .any(|m| m.points.iter().any(|p| p.data.warmstart_impulse > 0.))
        {
            return None;
        }
        let link = Surface::capture(world, handle)?;
        let sphere = link.shape.compute_local_bounding_sphere();
        let ground = Surface::capture(world, ground)?;
        let mut edge_anchors = Vec::new();
        let coordinate_scale = link
            .pose
            .translation
            .abs()
            .max(ground.pose.translation.abs())
            .max_element();
        let edge_roundoff =
            8. * f64::EPSILON * (1. + coordinate_scale + sphere.center.length() + sphere.radius);
        if let Some(cuboid) = ground.shape.as_cuboid() {
            let foot_first = pair.collider1 == handle;
            let link_in_ground = ground.pose.inv_mul(&link.pose);
            for manifold in pair.solver_manifolds() {
                let (normal, subshape) = if foot_first {
                    (manifold.local_n2, manifold.subshape_pos1())
                } else {
                    (manifold.local_n1, manifold.subshape_pos2())
                };
                let normal_axis = normal.abs().max_position();
                for point in manifold
                    .points
                    .iter()
                    .filter(|p| p.data.warmstart_impulse > 0.)
                {
                    // A non-face (edge or corner) normal has no certified planar support.
                    if 1. - normal[normal_axis].abs() > 1e-12 {
                        return None;
                    }
                    let local = if foot_first {
                        point.local_p1
                    } else {
                        point.local_p2
                    };
                    let link_point = subshape.map_or(local, |pose| *pose * local);
                    let anchor = EdgeAnchor {
                        link_point,
                        at_capture: link_in_ground * link_point,
                        normal_axis,
                    };
                    // Interior points cannot reach a lip before the drift guard expires.
                    let overhang = anchor.overhang(&link_in_ground, cuboid.half_extents);
                    let tolerance = world.integration_parameters.allowed_linear_error();
                    if overhang >= -tolerance - edge_roundoff {
                        if edge_roundoff >= tolerance
                            || overhang > link.skin + ground.skin + tolerance + edge_roundoff
                        {
                            return None;
                        }
                        edge_anchors.push(anchor);
                    }
                }
            }
        }
        Some(Self {
            link,
            ground,
            gravity: world.gravity,
            integration: world.integration_parameters,
            radius_about_link_origin: sphere.center.length() + sphere.radius,
            expiry: None,
            edge_anchors,
            edge_roundoff,
        })
    }

    fn overhang(&self, pose: &Pose, floor: &Pose) -> Option<f64> {
        if self.edge_anchors.is_empty() {
            return None;
        }
        let extents = self.ground.shape.as_cuboid()?.half_extents;
        let link_in_ground = floor.inv_mul(pose);
        self.edge_anchors
            .iter()
            .map(|anchor| anchor.overhang(&link_in_ground, extents))
            .reduce(f64::max)
    }

    fn check(&self, world: &PhysicsWorld) -> Result<(), SupportExpiry> {
        let pose = self
            .link
            .current_pose(world)
            .ok_or(SupportExpiry::SurfaceChanged)?;
        let floor = self
            .ground
            .current_pose(world)
            .ok_or(SupportExpiry::SurfaceChanged)?;
        let collider = &world.colliders[self.ground.handle];
        if floor != self.ground.pose
            || !support_loss::stationary_support(&world.bodies[self.ground.parent])
            || !collision::can_exchange_impulses(
                &world.colliders[self.link.handle],
                collider,
                world.bodies[self.ground.parent].body_type(),
            )
        {
            return Err(SupportExpiry::SurfaceChanged);
        }
        if self.gravity != world.gravity || self.integration != world.integration_parameters {
            return Err(SupportExpiry::SolverChanged);
        }
        if !self.edge_anchors.is_empty() {
            let extents = self
                .ground
                .shape
                .as_cuboid()
                .expect("captured cuboid face")
                .half_extents;
            let link_in_ground = floor.inv_mul(&pose);
            if self
                .edge_anchors
                .iter()
                .any(|a| a.crosses_face(&link_in_ground, extents, self.edge_roundoff))
            {
                return Err(SupportExpiry::AnchorOutsideFace);
            }
        }
        let delta = self.link.pose.inv_mul(&pose);
        let drift = delta.translation.length()
            + 2. * delta.rotation.xyz().length() * self.radius_about_link_origin;
        if drift > world.integration_parameters.allowed_linear_error() {
            return Err(SupportExpiry::FootprintMoved);
        }
        if !world
            .narrow_phase
            .contact_pair(self.link.handle, self.ground.handle)
            .is_some_and(|p| {
                p.solver_manifolds()
                    .iter()
                    .any(|m| m.points.iter().any(|p| p.data.warmstart_impulse > 0.))
            })
        {
            return Err(SupportExpiry::NoLoadedPoint);
        }
        Ok(())
    }
}

fn competing_contact(world: &PhysicsWorld, handle: ColliderHandle, holds: &[SupportHold]) -> bool {
    world.narrow_phase.contact_pairs_with(handle).any(|pair| {
        pair.has_any_active_contact()
            && !holds
                .iter()
                .any(|h| pair.collider1 == h.ground.handle || pair.collider2 == h.ground.handle)
    })
}

pub(super) fn refresh_contact_rows(collider: &mut Collider) {
    // In Rapier 0.36.1 a hook change alone does not force a full narrow-phase
    // query. GROUPS invalidates recycled rows without changing the final mask.
    let groups = collider.solver_groups();
    collider.set_solver_groups(InteractionGroups::none());
    collider.set_solver_groups(groups);
}

impl RapierBackend {
    pub(super) fn step_with_held_supports(&mut self) {
        self.step_with_box_features(None);
    }
    pub(super) fn step_with_box_features(
        &mut self,
        fresh: Option<&crate::box_features::BoxFeatures>,
    ) {
        self.step_with_registered_features(fresh, None, None);
    }
    pub(super) fn step_with_registered_features(
        &mut self,
        fresh: Option<&crate::box_features::BoxFeatures>,
        approach: Option<(ColliderHandle, ColliderHandle)>,
        opponent: Option<ColliderHandle>,
    ) {
        let row = approach.map(|(foot, pad)| collision::SuppressedRows {
            foot,
            pad,
            second: opponent,
        });
        self.step_with_suppressed_rows(fresh, row.as_slice());
    }
    pub(crate) fn step_with_suppressed_rows(
        &mut self,
        fresh: Option<&crate::box_features::BoxFeatures>,
        rows: &[collision::SuppressedRows],
    ) {
        self.parked_shared.clear();
        self.solved_shared_groups.clear();
        let trial = if self.parked_speed.is_empty() {
            None
        } else {
            let ordinary = fresh.is_none()
                && rows.is_empty()
                && self.held_supports.is_empty()
                && self.cold_support_pairs.is_empty()
                && !self.joints.authored_welds
                && self.welded_projection.is_none();
            #[cfg(feature = "landing-audit")]
            let ordinary = ordinary && self.landing_rows.is_none();
            #[cfg(feature = "partition-checkpoint")]
            let ordinary = ordinary && !self.join_queries_pending;
            self.parked_speed_trial(ordinary)
        };
        if self.joints.authored_welds || self.welded_projection.is_some() {
            let eligible = fresh.is_none()
                && rows.is_empty()
                && self.held_supports.is_empty()
                && self.cold_support_pairs.is_empty();
            #[cfg(feature = "landing-audit")]
            let eligible = eligible && self.landing_rows.is_none();
            #[cfg(feature = "partition-checkpoint")]
            let eligible = eligible && !self.join_queries_pending;
            if self.step_welded_projection(eligible) {
                return;
            }
        }
        let mut activate = |row: &collision::SuppressedRows| {
            let c = &mut self.world.colliders[row.foot];
            let old = c.active_hooks();
            c.set_active_hooks(old | ActiveHooks::FILTER_CONTACT_PAIRS);
            refresh_contact_rows(c);
            (row.foot, old)
        };
        // The ordinary single-pair path needs no allocation.
        let (first, rest) = rows
            .split_first()
            .map_or((None, &[][..]), |(r, tail)| (Some(activate(r)), tail));
        let extra: Vec<_> = rest.iter().map(activate).collect();
        let cold = if self.cold_support_pairs.is_empty() {
            None
        } else {
            // Re-arm a required full query even if filtering or sleep prevented
            // the previous native step from ever visiting this pair.
            for &(foot, _) in &self.cold_support_pairs {
                if let Some(body) = self.world.bodies.get(foot)
                    && let [handle] = body.colliders()
                {
                    refresh_contact_rows(&mut self.world.colliders[*handle]);
                }
            }
            // A box-join dispatcher can only exist on a newly staged world
            // without articulations. Capturing loaded supports needs a solved tree.
            #[cfg(feature = "partition-checkpoint")]
            debug_assert!(!self.join_queries_pending);
            let discard = crate::contact_dispatcher::DiscardPairs::new(std::mem::take(
                &mut self.cold_support_pairs,
            ));
            Some(discard)
        };
        if let Some(selected) = fresh {
            selected.request_queries(&mut self.world.colliders);
        }
        if cold.is_some() || fresh.is_some() {
            self.world.narrow_phase.set_query_dispatcher(
                crate::contact_dispatcher::NativeContacts(crate::box_features::PrepareContacts {
                    cold: cold.clone(),
                    fresh: fresh.cloned(),
                }),
            );
        }
        self.solve_mounted_interval(rows);
        if let Some(before) = trial
            && self.speed_park_invalidated()
        {
            // An incoming contact woke the island during this interval. Repeat from
            // its unmodified manifolds with the original signed speed limits active.
            self.world = before;
            self.resume_parked_speed_groups();
            self.solve_mounted_interval(rows);
        }
        self.advance_parked_reference_time();
        for (a, old) in first.into_iter().chain(extra) {
            let c = &mut self.world.colliders[a];
            c.set_active_hooks(old);
            refresh_contact_rows(c);
        }
        if cold.is_some() || fresh.is_some() {
            self.world
                .narrow_phase
                .set_query_dispatcher(rapier3d_f64::parry::query::DefaultQueryDispatcher);
        }
        if let Some(discard) = cold {
            // Filtered or sleeping pairs may skip a full geometric query. Leave
            // the reset armed while their original native points still exist.
            let mut pending = discard.remaining();
            pending.retain(|&(a, b)| {
                let (Some(left), Some(right)) =
                    (self.world.bodies.get(a), self.world.bodies.get(b))
                else {
                    return false;
                };
                let ([x], [y]) = (left.colliders(), right.colliders()) else {
                    return false;
                };
                self.world
                    .narrow_phase
                    .contact_pair(*x, *y)
                    .is_some_and(|pair| pair.manifolds().iter().any(|m| !m.points.is_empty()))
            });
            self.cold_support_pairs.extend(pending);
        }
        self.support_watch.awaiting_solve.clear();
        self.support_watch.update(&self.world, None);
        // A prescribed foot can leave a valid footprint and return before
        // promotion. Rapier may still match its old points by feature ID.
        // Keep the pair dormant but permanently disqualify its seeds from
        // the next dynamic solve as soon as any advancing native step fails.
        for holds in self.held_supports.values_mut() {
            let other = competing_contact(&self.world, holds[0].link.handle, holds);
            for hold in holds {
                if hold.expiry.is_none() {
                    hold.expiry = if other {
                        Some(SupportExpiry::CompetingContact)
                    } else {
                        hold.check(&self.world).err()
                    };
                }
            }
        }
    }

    fn solve_mounted_interval(&mut self, rows: &[collision::SuppressedRows]) {
        self.joints
            .prepare_universal_axes(&mut self.world, &self.bodies);
        self.joints
            .prepare_speed_caps(&mut self.world, &self.bodies, &self.parked_speed);
        self.joints.prepare_screws(&mut self.world, &self.bodies);
        let caps = self.begin_axial_cap_impacts();
        self.world.step_with_events(
            &collision::SupportedGroups(
                &self.held_supports,
                rows,
                &caps,
                #[cfg(feature = "landing-audit")]
                self.landing_rows.as_ref(),
            ),
            &(),
        );
        self.end_axial_cap_impacts(&caps);
    }

    /// The single captured support, when the link has exactly one.
    pub fn held_support_status(&self, link: BodyId) -> Option<HeldSupportStatus> {
        let [hold] = self.held_supports.get(&link)?.as_slice() else {
            return None;
        };
        Some(self.support_status(hold))
    }

    /// Read the independently latched status for the requested foot and support.
    pub fn held_support_pair_status(
        &self,
        link: BodyId,
        support: BodyId,
    ) -> Option<HeldSupportStatus> {
        let floor = self.bodies.get(&support)?.0;
        let hold = self
            .held_supports
            .get(&link)?
            .iter()
            .find(|h| h.ground.parent == floor)?;
        Some(self.support_status(hold))
    }

    fn support_status(&self, hold: &SupportHold) -> HeldSupportStatus {
        HeldSupportStatus {
            expiry: hold.expiry,
            edge_anchors: hold.edge_anchors.len(),
            max_face_overhang_m: hold.link.current_pose(&self.world).and_then(|p| {
                hold.ground
                    .current_pose(&self.world)
                    .and_then(|f| hold.overhang(&p, &f))
            }),
        }
    }

    pub(super) fn retire_support_body(&mut self, retired: RigidBodyHandle) {
        self.parked_shared.retain(|p| !p.contains_body(retired));
        self.solved_shared_groups.clear();
        self.cold_support_pairs
            .retain(|&(a, b)| a != retired && b != retired);
        self.held_supports.retain(|_, holds| {
            let link = holds[0].link.handle;
            holds.retain(|h| h.link.parent != retired && h.ground.parent != retired);
            if !holds.is_empty() {
                return true;
            }
            if let Some(collider) = self.world.colliders.get_mut(link) {
                let mut hooks = collider.active_hooks();
                hooks.remove(ActiveHooks::MODIFY_SOLVER_CONTACTS);
                collider.set_active_hooks(hooks);
            }
            false
        });
    }

    pub(super) fn retain_stationary_support(
        &mut self,
        id: BodyId,
        body: RigidBodyHandle,
        target: ArticulationTier,
        dt: Duration,
    ) -> bool {
        let [handle] = *self.world.bodies[body].colliders() else {
            return false;
        };
        if target == ArticulationTier::Prescribed
            && self.parked_shared.iter().any(|p| p.owns_foot(body))
        {
            return false;
        }
        let keep = match target {
            ArticulationTier::Prescribed => {
                if self.world.gravity == Vector::ZERO
                    || self.world.integration_parameters.dt != dt.as_secs_f64()
                {
                    return false;
                }
                let Some(holds) = self
                    .world
                    .narrow_phase
                    .contact_pairs_with(handle)
                    .filter(|p| p.has_any_active_contact())
                    .map(|p| SupportHold::capture(&self.world, handle, p))
                    .collect::<Option<Vec<_>>>()
                else {
                    return false;
                };
                if holds.is_empty() {
                    return false;
                }
                // Selective manifold updates identify bodies by Rapier's generational
                // parent handles. Compound/multi-collider supports stay on the
                // existing whole-link cold path until individual subshape histories qualify.
                if holds.len() > 1
                    && holds.iter().any(|h| {
                        self.world.bodies[h.ground.parent].colliders().len() != 1
                            || !h.ground.shape.is_convex()
                            || !h.link.shape.is_convex()
                    })
                {
                    return false;
                }
                self.held_supports.insert(id, holds);
                true
            }
            ArticulationTier::Dynamic => {
                let Some(mut holds) = self.held_supports.remove(&id) else {
                    return false;
                };
                let keep = !competing_contact(&self.world, handle, &holds)
                    && holds[0].integration.dt == dt.as_secs_f64();
                if keep {
                    for hold in &mut holds {
                        if hold.expiry.is_none() {
                            hold.expiry = hold.check(&self.world).err();
                        }
                    }
                }
                let retained = keep && holds.iter().any(|h| h.expiry.is_none());
                // A wholly cold foot uses the existing collider rebuild. Only a
                // mixed decision needs a per-pair deferred native requery.
                if retained {
                    for hold in holds.iter().filter(|h| h.expiry.is_some()) {
                        let pair = (body, hold.ground.parent);
                        if !self.cold_support_pairs.contains(&pair) {
                            self.cold_support_pairs.push(pair);
                        }
                    }
                }
                retained
            }
        };
        let collider = &mut self.world.colliders[handle];
        let mut hooks = collider.active_hooks();
        hooks.set(
            ActiveHooks::MODIFY_SOLVER_CONTACTS,
            target == ArticulationTier::Prescribed,
        );
        collider.set_active_hooks(hooks);
        if keep {
            refresh_contact_rows(collider);
        }
        self.world.bodies[body].wake_up(true);
        keep
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const DT: Duration = Duration::from_millis(1);

    fn settled() -> (RapierBackend, BodyId, BodyId, BodyId, JointId) {
        settled_on(5.)
    }
    fn settled_on(right_edge: f64) -> (RapierBackend, BodyId, BodyId, BodyId, JointId) {
        settled_with(right_edge, |b, spec| b.spawn(spec).unwrap())
    }

    fn settled_with(
        right_edge: f64,
        spawn_pad: impl FnOnce(&mut RapierBackend, BodySpec) -> BodyId,
    ) -> (RapierBackend, BodyId, BodyId, BodyId, JointId) {
        let mut b = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
        let spec = |position, half_extents, mass| BodySpec {
            position,
            velocity: [0.; 3],
            mass,
            shape: Shape::Box { half_extents },
        };
        let [root, tip] = [0., 0.65].map(|y| {
            b.spawn_articulation_link(
                spec([0., y, 0.24], [0.14; 3], Some(1.)),
                RotationSpec::default(),
                None,
            )
            .unwrap()
        });
        let hinge = b
            .add_reduced_revolute(Some(root), tip, [0., 0.65, 0.24], [0., 0., 1.])
            .unwrap();
        let floor = spawn_pad(
            &mut b,
            spec([right_edge - 5., 0., -0.1], [5., 5., 0.1], None),
        );
        for _ in 0..2000 {
            b.step(DT).unwrap();
        }
        (b, root, tip, floor, hinge)
    }

    fn impulse(b: &RapierBackend, a: BodyId, c: BodyId) -> f64 {
        b.rigid_contact_observations()
            .iter()
            .filter(|p| (p.a == a && p.b == c) || (p.a == c && p.b == a))
            .map(|p| p.normal_impulse)
            .sum()
    }
    #[test]
    fn tracked_floor_seeds_wait_then_feed_the_native_dynamic_solver() {
        let run = |coefficient| {
            let (mut b, root, tip, floor, _) = settled();
            let before = [root, tip].map(|id| b.contact_warmstart(id, floor).unwrap());
            assert!(before.iter().all(|p| p.normal_ns > 0. && p.points >= 3));
            b.switch_articulation(root, ArticulationTier::Prescribed, DT)
                .unwrap();
            for _ in 0..40 {
                b.step(DT).unwrap();
            }
            for (id, source) in [root, tip].into_iter().zip(&before) {
                let held = b.contact_warmstart(id, floor).unwrap();
                assert_eq!(held.normal_ns, source.normal_ns);
                assert_eq!(held.tangent_on_a_ns, source.tangent_on_a_ns);
                assert_eq!(impulse(&b, id, floor), 0.);
            }
            b.switch_articulation(root, ArticulationTier::Dynamic, DT)
                .unwrap();
            b.switch_articulation(root, ArticulationTier::Dynamic, DT)
                .unwrap();
            assert!(b.held_supports.is_empty());
            for (id, seed) in [root, tip].into_iter().zip(before) {
                let current = b.contact_warmstart(id, floor).unwrap();
                assert_eq!(current.normal_ns, seed.normal_ns);
                assert_eq!(current.tangent_on_a_ns, seed.tangent_on_a_ns);
            }
            b.world.integration_parameters.warmstart_coefficient = coefficient;
            b.step(DT).unwrap();
            let weight_per_step = 2. * 9.81 * DT.as_secs_f64();
            let solved = impulse(&b, root, floor) + impulse(&b, tip, floor);
            eprintln!(
                "coefficient={coefficient}: first solved impulse={solved:.12} N s, static weight impulse={weight_per_step:.12} N s"
            );
            (
                b.state(tip).unwrap().velocity,
                (solved - weight_per_step).abs(),
            )
        };
        // Same contacts and retained manifold. The native solver's own warm-start
        // coefficient changes the first dynamic result, independently of geometry.
        let (warm_velocity, warm_error) = run(1.);
        let (cold_velocity, cold_error) = run(0.);
        assert_ne!(warm_velocity, cold_velocity);
        assert!(warm_error < 1e-7 && cold_error > 1e-6);
    }

    #[test]
    fn moved_replaced_or_retired_floor_and_changed_step_drop_the_seed() {
        for change in 0..7 {
            let (mut b, root, tip, floor, _) = settled();
            let h = b.bodies[&floor].0;
            b.switch_articulation(root, ArticulationTier::Prescribed, DT)
                .unwrap();
            b.step(DT).unwrap();
            let mut dt = DT;
            match change {
                0 => b.world.bodies[h].set_translation(Vector::new(0., 0., -1.), true),
                1 => {
                    let co = b.world.bodies[h].colliders()[0];
                    let original = b.world.remove_collider(co).unwrap();
                    b.world.insert_collider(original, Some(h));
                }
                2 => dt = Duration::from_millis(2),
                3 => {
                    let co = b.world.bodies[h].colliders()[0];
                    b.world.colliders[co].set_friction(0.01);
                }
                4 => {
                    b.world.integration_parameters.num_solver_iterations += 1;
                }
                5 => {
                    let co = b.world.bodies[h].colliders()[0];
                    b.world.colliders[co].set_contact_skin(0.001);
                }
                _ => {
                    let link = b.bodies[&root].0;
                    let p = b.world.bodies[link].translation() + Vector::X * 0.01;
                    b.world.bodies[link].set_translation(p, true);
                    let link = b.bodies[&tip].0;
                    let p = b.world.bodies[link].translation() + Vector::X * 0.01;
                    b.world.bodies[link].set_translation(p, true);
                }
            }
            b.switch_articulation(root, ArticulationTier::Dynamic, dt)
                .unwrap();
            for id in [root, tip] {
                assert_eq!(b.contact_warmstart(id, floor).unwrap().normal_ns, 0.);
            }
        }
    }
    #[test]
    fn retiring_floor_or_articulation_member_releases_hooks_on_survivors() {
        for retire in 0..3 {
            let (mut b, root, tip, floor, hinge) = settled();
            b.switch_articulation(root, ArticulationTier::Prescribed, DT)
                .unwrap();
            b.step(DT).unwrap();
            assert_eq!(b.held_supports.len(), 2);
            for group in [Some(9), None] {
                let h = b.bodies[&root].0;
                collision::assign(&mut b.world, h, group);
                let flags = b.world.colliders[b.world.bodies[h].colliders()[0]].active_hooks();
                assert!(flags.contains(ActiveHooks::MODIFY_SOLVER_CONTACTS));
                assert_eq!(
                    flags.contains(ActiveHooks::FILTER_CONTACT_PAIRS),
                    group.is_some()
                );
            }
            assert!(match retire {
                0 => b.remove(floor),
                1 => b.remove(tip),
                _ => b.remove_joint(hinge),
            });
            assert!(b.held_supports.is_empty());
            for h in b.world.bodies[b.bodies[&root].0].colliders() {
                assert!(
                    !b.world.colliders[*h]
                        .active_hooks()
                        .contains(ActiveHooks::MODIFY_SOLVER_CONTACTS)
                );
            }
        }
    }

    #[test]
    fn rocking_away_then_returning_cannot_restore_an_expired_footprint() {
        for endpoint_only in [true, false] {
            let (mut b, root, tip, floor, _) = settled();
            let initial = b.state(root).unwrap();
            assert!(b.contact_warmstart(root, floor).unwrap().normal_ns > 0.);
            b.switch_articulation(root, ArticulationTier::Prescribed, DT)
                .unwrap();
            let index = b.articulation_index(root).unwrap();
            // Drive the prescribed generalized pitch velocity, so Rapier itself
            // advances and matches the points during both halves of the excursion.
            for direction in [1., -1.] {
                b.world
                    .multibody_joints
                    .get_multibody_mut(index)
                    .unwrap()
                    .generalized_velocity_mut()[4] = direction;
                for _ in 0..32 {
                    b.step(DT).unwrap();
                    assert_eq!(impulse(&b, root, floor), 0.);
                }
                let q = b.state(root).unwrap().orientation;
                let alignment: f64 = q.iter().zip(initial.orientation).map(|(a, b)| a * b).sum();
                let angle = 2. * alignment.abs().min(1.).acos();
                eprintln!(
                    "rock direction={direction} angle={angle} seed={:?}",
                    b.contact_warmstart(root, floor).unwrap()
                );
                if direction > 0. {
                    assert!(angle > 0.03);
                    assert_eq!(
                        b.held_supports[&root][0].check(&b.world),
                        Err(SupportExpiry::FootprintMoved)
                    );
                } else {
                    assert!(angle < 1e-5);
                    assert!(b.held_supports[&root][0].check(&b.world).is_ok());
                    assert_eq!(
                        b.held_support_status(root).unwrap().expiry,
                        Some(SupportExpiry::FootprintMoved)
                    );
                    let seeds = b.contact_warmstart(root, floor).unwrap();
                    assert!(
                        seeds.normal_ns > 0. && Vector::from(seeds.tangent_on_a_ns).length() > 0.
                    );
                }
            }
            if endpoint_only {
                // Recreate the former endpoint-only decision on the identical
                // world. No poses, manifolds, features or seeds have been changed.
                for hold in b.held_supports.values_mut().flatten() {
                    hold.expiry = None;
                }
            }
            b.switch_articulation(root, ArticulationTier::Dynamic, DT)
                .unwrap();
            for id in [root, tip] {
                let seeds = b.contact_warmstart(id, floor).unwrap();
                eprintln!("endpoint_only={endpoint_only} link={id:?} cache at promotion={seeds:?}");
                if endpoint_only {
                    assert!(
                        seeds.normal_ns > 0. && Vector::from(seeds.tangent_on_a_ns).length() > 0.
                    );
                } else {
                    assert_eq!(seeds, ContactWarmstart::default());
                }
            }
            b.step(DT).unwrap();
            assert!(impulse(&b, root, floor) > 0.);
        }
    }

    #[test]
    fn a_loaded_toe_crossing_a_finite_face_expires_below_the_drift_tolerance() {
        let lip = 0.14005;
        let (mut b, root, _, floor, _) = settled_on(lip);
        let foot = b.bodies[&root].0;
        let toe_x =
            |b: &RapierBackend| (b.world.bodies[foot].position() * Vector::new(0.14, 0., -0.14)).x;
        assert!(toe_x(&b) < lip);
        let index = b.articulation_index(root).unwrap();
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        let captured = b.held_support_status(root).unwrap();
        assert!(captured.edge_anchors > 0);
        assert!(captured.max_face_overhang_m.unwrap() < 0.);
        b.world
            .multibody_joints
            .get_multibody_mut(index)
            .unwrap()
            .generalized_velocity_mut()[4] = -1.;
        for _ in 0..2 {
            b.step(DT).unwrap();
        }
        let overhang = toe_x(&b) - lip;
        assert!(overhang > 0. && overhang < b.world.integration_parameters.allowed_linear_error());
        let held = b.contact_warmstart(root, floor).unwrap();
        eprintln!("toe overhang={overhang:.9} m, dormant cache={held:?}");
        assert!(held.normal_ns > 0.);
        let crossed = b.held_support_status(root).unwrap();
        assert_eq!(crossed.expiry, Some(SupportExpiry::AnchorOutsideFace));
        assert!(crossed.max_face_overhang_m.unwrap() > 0.);
        // The toe left the face, but the rest of the sole still touches.
        assert_eq!(b.pending_support_loss(root), None);
        assert!(!b.has_pending_support_loss());
        b.switch_articulation(root, ArticulationTier::Dynamic, DT)
            .unwrap();
        assert_eq!(
            b.contact_warmstart(root, floor).unwrap(),
            ContactWarmstart::default()
        );
    }

    #[test]
    fn an_existing_witness_offset_on_a_rotated_face_expires_only_after_further_motion() {
        let floor = Pose::from_parts(
            Vector::new(100_000., -500_000., 23.),
            Rotation::from_rotation_y(0.29) * Rotation::from_rotation_z(0.71),
        );
        let extents = Vector::new(0.2, 0.1, 0.02);
        let foot = |shift: f64| floor * Pose::translation(0.12 + shift, 0., 0.06);
        let relative = floor.inv_mul(&foot(0.));
        let local = Vector::new(0.08002, 0., -0.04);
        let anchor = EdgeAnchor {
            normal_axis: 2,
            link_point: local,
            at_capture: relative * local,
        };
        let roundoff = 8. * f64::EPSILON * floor.translation.abs().max_element();
        assert!(anchor.overhang(&relative, extents) > 0.);
        assert!(!anchor.crosses_face(&relative, extents, roundoff));
        assert!(!anchor.crosses_face(&floor.inv_mul(&foot(-0.0001)), extents, roundoff));
        assert!(anchor.crosses_face(&floor.inv_mul(&foot(0.0001)), extents, roundoff));
    }

    #[test]
    fn selectively_discarded_pair_is_cold_on_its_first_native_requery() {
        use crate::contact_dispatcher::{DiscardPairs, Prepare};
        let (b, root, tip, floor, _) = settled();
        let (foot, neighbor, support) = (b.bodies[&root].0, b.bodies[&tip].0, b.bodies[&floor].0);
        let manifolds = |body: RigidBodyHandle| {
            let a = b.world.bodies[body].colliders()[0];
            let ground = b.world.bodies[support].colliders()[0];
            b.world
                .narrow_phase
                .contact_pair(a, ground)
                .unwrap()
                .manifolds()
                .to_vec()
        };
        let mut discarded = manifolds(foot);
        let mut untouched = manifolds(neighbor);
        let shape = b.world.colliders[b.world.bodies[foot].colliders()[0]].shape();
        let seeds = |m: &[rapier3d_f64::geometry::ContactManifold]| {
            m.iter()
                .flat_map(|m| &m.points)
                .map(|p| {
                    (
                        p.data.warmstart_impulse,
                        p.data.warmstart_tangent_world,
                        p.data.warmstart_tangent_impulse,
                        p.data.warmstart_twist_impulse,
                    )
                })
                .collect::<Vec<_>>()
        };
        let other = seeds(&untouched);
        let original = seeds(&discarded);
        assert!(original.iter().any(|p| p.0 > 0. && p.1.length() > 0.));
        assert!(other.iter().any(|p| p.0 > 0. && p.1.length() > 0.));
        let discard = DiscardPairs::new(vec![(foot, support)]);
        // Both pairs deliberately receive the same Shape references. Only their
        // persisted manifold owners can distinguish these queries.
        discard.prepare(&Pose::IDENTITY, shape, shape, &mut untouched);
        assert_eq!(seeds(&untouched), other);
        discard.prepare(&Pose::IDENTITY, shape, shape, &mut discarded);
        assert!(seeds(&discarded).iter().all(|p| p.0 == 0.
            && p.1 == Vector::ZERO
            && p.2.iter().all(|v| *v == 0.)
            && p.3 == 0.));
        // Model the next substep by reinstalling newly solved data. The old
        // discard cannot erase a fresh solve, even for the same pair.
        let mut fresh = manifolds(foot);
        discard.prepare(&Pose::IDENTITY, shape, shape, &mut fresh);
        assert_eq!(seeds(&fresh), original);
        assert!(discard.remaining().is_empty());
    }

    #[test]
    fn removing_one_of_two_live_supports_preserves_the_sibling_pair_and_hooks() {
        let (mut b, root, tip, floor, _) = settled();
        assert!(b.remove(floor));
        let half_pad = |x| BodySpec {
            position: [x, 0., -0.1],
            velocity: [0.; 3],
            mass: None,
            shape: Shape::Box {
                half_extents: [0.06, 1., 0.1],
            },
        };
        let [left, right] = [-0.08, 0.08].map(|x| b.spawn(half_pad(x)).unwrap());
        for _ in 0..1000 {
            b.step(DT).unwrap();
        }
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        let before = [root, tip].map(|id| b.contact_warmstart(id, left).unwrap());
        for id in [root, tip] {
            assert!(b.held_support_status(id).is_none());
            for support in [left, right] {
                assert!(b.held_support_pair_status(id, support).is_some());
                assert!(b.native_contact_warmstart(id, support).unwrap().normal_ns > 0.);
            }
        }
        assert!(!b.has_pending_support_loss());
        assert!(b.remove(right));
        let loss = Some(SupportLoss {
            link: root,
            support: right,
            reason: SupportLossReason::Retired,
        });
        assert_eq!(b.pending_support_loss(root), loss);
        assert_eq!(b.pending_support_loss(tip), None);
        b.step(DT).unwrap();
        assert_eq!(
            b.articulation_tier(root).unwrap(),
            ArticulationTier::Prescribed
        );
        assert_eq!(b.pending_support_loss(root), loss);
        b.switch_articulation(root, ArticulationTier::Dynamic, DT)
            .unwrap();
        assert!(!b.has_pending_support_loss());
        for (id, expected) in [root, tip].into_iter().zip(before) {
            assert_eq!(b.native_contact_warmstart(id, left).unwrap(), expected);
            assert!(b.contact_warmstart(id, right).is_err());
        }
        b.step(DT).unwrap();
        assert!(impulse(&b, root, left) + impulse(&b, tip, left) > 0.);
    }

    #[test]
    fn cold_demotion_still_tracks_solved_support_and_preserves_retired_identity() {
        let (mut b, root, _, floor, hinge) = settled();
        assert!(impulse(&b, root, floor) > 0.);
        // A dt mismatch denies native warm-start retention during this switch.
        b.switch_articulation(
            root,
            ArticulationTier::Prescribed,
            Duration::from_micros(750),
        )
        .unwrap();
        assert!(b.held_support_status(root).is_none());
        b.step(DT).unwrap();
        assert!(!b.has_pending_support_loss());
        let pad = BodySpec {
            position: [4., 0., -0.1],
            velocity: [0.; 3],
            mass: None,
            shape: Shape::Box {
                half_extents: [0.1; 3],
            },
        };
        let unrelated = b.spawn(pad).unwrap();
        assert!(b.remove(unrelated));
        assert!(!b.has_pending_support_loss());
        assert!(b.remove(floor));
        let replacement = b.spawn(pad).unwrap();
        assert_ne!(floor, replacement);
        assert_eq!(
            b.pending_support_loss(root),
            Some(SupportLoss {
                link: root,
                support: floor,
                reason: SupportLossReason::Retired,
            })
        );
        assert!(b.remove_joint(hinge));
        assert!(!b.has_pending_support_loss());
    }

    #[test]
    fn moving_a_loaded_pad_reports_only_proven_separation_and_reset_clears_it() {
        let (mut b, root, _, floor, _) = settled();
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        let original = b.state(floor).unwrap();
        let mut moved = original;
        moved.position[0] += 0.01;
        b.set_body_state(floor, moved).unwrap();
        assert!(!b.has_pending_support_loss());
        moved.position[0] += 20.;
        b.set_body_state(floor, moved).unwrap();
        let loss = Some(SupportLoss {
            link: root,
            support: floor,
            reason: SupportLossReason::Separated,
        });
        assert_eq!(b.pending_support_loss(root), loss);
        b.set_body_state(floor, original).unwrap();
        assert_eq!(b.pending_support_loss(root), loss);
        b.reset().unwrap();
        assert!(!b.has_pending_support_loss());
    }

    #[test]
    fn a_filtered_support_cannot_admit_a_stale_reaction_and_latches_if_already_held() {
        let (mut b, root, _, floor, _) = settled();
        let pad = b.world.bodies[b.bodies[&floor].0].colliders()[0];
        let groups = b.world.colliders[pad].solver_groups();
        b.world.colliders[pad].set_solver_groups(InteractionGroups::none());
        assert!(impulse(&b, root, floor) > 0.);
        assert!(matches!(
            b.switch_articulation(root, ArticulationTier::Prescribed, DT),
            Err(PhysicsError::TransitionDeferred { .. })
        ));
        assert_eq!(
            b.articulation_tier(root).unwrap(),
            ArticulationTier::Dynamic
        );
        assert!(!b.has_pending_support_loss());
        b.world.colliders[pad].set_solver_groups(groups);
        b.step(DT).unwrap();
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        b.world.colliders[pad].set_solver_groups(InteractionGroups::none());
        b.step(DT).unwrap();
        assert_eq!(
            b.pending_support_loss(root),
            Some(SupportLoss {
                link: root,
                support: floor,
                reason: SupportLossReason::Separated
            })
        );
    }

    #[test]
    fn captured_skin_allows_clearance_and_prepared_reset_rearms_observation() {
        let (mut b, root, _, floor, _) = settled();
        let initial = b.initial_state();
        let pad = b.world.bodies[b.bodies[&floor].0].colliders()[0];
        b.world.colliders[pad].set_contact_skin(0.01);
        for _ in 0..1000 {
            b.step(DT).unwrap();
        }
        assert!(impulse(&b, root, floor) > 0.);
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        let mut moved = b.state(floor).unwrap();
        // The sole remains geometrically clear after lifting the pad 5 mm
        // into its previously loaded 10 mm contact-skin envelope.
        moved.position[2] += 0.005;
        b.set_body_state(floor, moved).unwrap();
        assert!(!b.has_pending_support_loss());
        moved.position[2] -= 0.02;
        b.set_body_state(floor, moved).unwrap();
        assert_eq!(b.pending_support_loss(root).unwrap().support, floor);
        let prepared = b.prepare_initial(&initial).unwrap();
        b.apply_initial(prepared).unwrap();
        assert!(!b.has_pending_support_loss());
        for _ in 0..2000 {
            b.step(DT).unwrap();
        }
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        assert!(!b.has_pending_support_loss());
        assert!(b.remove(floor));
        assert_eq!(
            b.pending_support_loss(root).unwrap().reason,
            SupportLossReason::Retired
        );
    }

    #[test]
    fn committed_prescribed_liftoff_latches_a_geometric_loss() {
        let (mut b, root, tip, floor, _) = settled();
        let index = b.articulation_index(root).unwrap();
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        // Drive a prescribed lift independently of the tiny residual spin of
        // the settled fixture. A spinning tree transports its COM velocity
        // into root coordinates again at each native boundary.
        b.spinning_assemblies.remove(&root);
        b.world
            .multibody_joints
            .get_multibody_mut(index)
            .unwrap()
            .generalized_velocity_mut()[2] = 0.5;
        assert!(!b.has_pending_support_loss());
        for _ in 0..40 {
            b.step(DT).unwrap();
            if b.has_pending_support_loss() {
                break;
            }
        }
        let loss = b
            .pending_support_loss(root)
            .expect("prescribed lift lost the floor");
        assert_eq!(loss.reason, SupportLossReason::Separated);
        assert_eq!(loss.support, floor);
        assert!([root, tip].contains(&loss.link));
        let collider = |id| &b.world.colliders[b.world.bodies[b.bodies[&id].0].colliders()[0]];
        let sole = collider(loss.link);
        let pad = collider(floor);
        let clearance = rapier3d_f64::parry::query::distance(
            sole.position(),
            sole.shape(),
            pad.position(),
            pad.shape(),
        )
        .unwrap()
        .distance;
        assert!(clearance > 0., "native sole–floor clearance {clearance}");
    }

    fn velocity_pad(b: &mut RapierBackend, spec: BodySpec) -> BodyId {
        b.spawn_velocity_boundary(spec, RotationSpec::default(), None)
            .unwrap()
    }

    #[test]
    fn stationary_velocity_pad_rearms_on_each_landing_and_after_reset() {
        let (mut b, root, tip, pad, _) = settled_with(5., velocity_pad);
        let initial = b.initial_state();
        for cycle in 0..3 {
            let loaded = [root, tip].map(|id| b.contact_warmstart(id, pad).unwrap());
            assert!(loaded.into_iter().all(|c| c.points > 0 && c.normal_ns > 0.));
            b.switch_articulation(root, ArticulationTier::Prescribed, DT)
                .unwrap();
            b.step(DT).unwrap();
            assert_eq!([root, tip].map(|id| impulse(&b, id, pad)), [0.; 2]);
            assert_eq!(
                [root, tip].map(|id| b.contact_warmstart(id, pad).unwrap()),
                loaded
            );
            assert!(!b.has_pending_support_loss());
            // A motion command latches at this boundary. The later integrated
            // flight is established independently with an actual unloaded gap.
            let start = b.state(pad).unwrap();
            b.set_world_velocity(pad, [0., 0., -1.], [0.; 3]).unwrap();
            assert_eq!(b.state(pad).unwrap().position, start.position);
            assert_eq!(
                b.pending_support_loss(root),
                Some(SupportLoss {
                    link: root,
                    support: pad,
                    reason: SupportLossReason::MotionStarted
                })
            );
            b.switch_articulation(root, ArticulationTier::Dynamic, DT)
                .unwrap();
            assert!(!b.has_pending_support_loss());
            assert_eq!(
                [root, tip].map(|id| b.native_contact_warmstart(id, pad).unwrap().points),
                [0; 2]
            );
            for _ in 0..30 {
                b.step(DT).unwrap();
            }
            assert_eq!([root, tip].map(|id| impulse(&b, id, pad)), [0.; 2]);
            let sole = &b.world.colliders[b.world.bodies[b.bodies[&root].0].colliders()[0]];
            let floor = &b.world.colliders[b.world.bodies[b.bodies[&pad].0].colliders()[0]];
            let gap = rapier3d_f64::parry::query::distance(
                sole.position(),
                sole.shape(),
                floor.position(),
                floor.shape(),
            )
            .unwrap()
            .distance;
            assert!(gap > 0.01, "airborne gap on cycle {cycle}: {gap}");
            assert!(
                b.switch_articulation(root, ArticulationTier::Prescribed, DT)
                    .is_err()
            );
            b.set_world_velocity(pad, [0., 0., 0.25], [0.; 3]).unwrap();
            let mut landing = None;
            for tick in 1..=120 {
                b.step(DT).unwrap();
                let normal = impulse(&b, root, pad) + impulse(&b, tip, pad);
                if normal > 0. {
                    landing.get_or_insert((tick, normal));
                }
            }
            let (tick, impulse) = landing.expect("moving pad did not catch the assembly");
            assert!(tick < 120 && impulse > 0.02, "cycle {cycle}: {landing:?}");
            assert!(
                b.switch_articulation(root, ArticulationTier::Prescribed, DT)
                    .is_err()
            );
            b.set_world_velocity(pad, [0.; 3], [0.; 3]).unwrap();
            for _ in 0..1800 {
                b.step(DT).unwrap();
            }
        }
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        b.set_world_velocity(pad, [0.; 3], [0., 0.2, 0.]).unwrap();
        assert_eq!(
            b.pending_support_loss(root).unwrap().reason,
            SupportLossReason::MotionStarted
        );
        let prepared = b.prepare_initial(&initial).unwrap();
        b.apply_initial(prepared).unwrap();
        assert!(!b.has_pending_support_loss());
        for _ in 0..2000 {
            b.step(DT).unwrap();
        }
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        b.set_world_velocity(pad, [0.001, 0., 0.], [0.; 3]).unwrap();
        assert_eq!(b.pending_support_loss(root).unwrap().support, pad);
    }

    #[test]
    fn stopped_velocity_command_waits_for_fresh_solve_even_with_matching_old_impulses() {
        let (mut b, root, _, pad, _) = settled_with(5., velocity_pad);
        let reaction = impulse(&b, root, pad);
        assert!(reaction > 0.);
        b.set_world_velocity(pad, [0., 0., -0.0000001], [0.; 3])
            .unwrap();
        assert!(
            b.switch_articulation(root, ArticulationTier::Prescribed, DT)
                .is_err()
        );
        // Stop at the same boundary. The original solved points still exist,
        // so admission must reject solely because their provenance is stale.
        let mut stop = b.state(pad).unwrap();
        stop.velocity = [0.; 3];
        b.set_body_state(pad, stop).unwrap();
        b.set_world_velocity(pad, [0.; 3], [0.; 3]).unwrap();
        let stopped = b.state(pad).unwrap();
        assert!(impulse(&b, root, pad) > 0.);
        assert!(b.support_watch.awaiting_solve.contains(&b.bodies[&pad].0));
        for _ in 0..2 {
            let decision = b.switch_articulation(root, ArticulationTier::Prescribed, DT);
            assert!(
                matches!(
                    decision,
                    Err(PhysicsError::TransitionDeferred {
                        reason: "support needs a solved stationary velocity boundary"
                    })
                ),
                "demotion attempted before a fresh solve: {decision:?}"
            );
            b.step(Duration::ZERO).unwrap();
        }
        b.step(DT).unwrap();
        b.set_body_state(pad, stopped).unwrap(); // an identical pose and twist preserve the fresh reaction
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        assert!(!b.has_pending_support_loss());
        let mut turning = stopped;
        turning.angular_velocity = [0., 0., 0.1];
        b.set_body_state(pad, turning).unwrap();
        assert_eq!(
            b.pending_support_loss(root).unwrap().reason,
            SupportLossReason::MotionStarted
        );
    }

    #[test]
    fn editing_a_parked_pad_pose_wakes_its_foot_and_stale_reactions_retire_with_the_pad() {
        let (mut b, root, _, pad, _) = settled_with(5., velocity_pad);
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        let original = b.state(pad).unwrap();
        let mut shifted = original;
        shifted.position[0] += 0.005; // Still underneath the entire foot.
        b.set_body_state(pad, shifted).unwrap();
        b.set_body_state(pad, original).unwrap();
        b.set_world_velocity(pad, [0.; 3], [0.; 3]).unwrap();
        assert_eq!(
            b.pending_support_loss(root).unwrap().reason,
            SupportLossReason::MotionStarted
        );
        b.switch_articulation(root, ArticulationTier::Dynamic, DT)
            .unwrap();
        assert_eq!(b.native_contact_warmstart(root, pad).unwrap().points, 0);
        let h = b.bodies[&pad].0;
        assert!(b.support_watch.awaiting_solve.contains(&h));
        assert!(b.remove(pad));
        assert!(!b.support_watch.awaiting_solve.contains(&h));
    }

    #[test]
    fn authored_velocity_pad_dwell_cannot_qualify_until_its_path_is_overridden_and_solved() {
        use gz_physics::trajectory::{PathEnd, TranslationSpline, Waypoint};
        let (mut b, root, _, pad, _) = settled_with(5., |b, spec| {
            let pad = velocity_pad(b, spec);
            let waypoint = |ms, z| Waypoint {
                time: Duration::from_millis(ms),
                displacement: [0., 0., z],
                velocity: [0.; 3],
            };
            let path = TranslationSpline::new(
                &[waypoint(0, 0.), waypoint(3000, 0.), waypoint(3500, -0.1)],
                PathEnd::Hold,
            )
            .unwrap();
            b.set_initial_translation_path(pad, path).unwrap();
            pad
        });
        assert!(impulse(&b, root, pad) > 0.);
        assert_eq!(b.state(pad).unwrap().velocity, [0.; 3]);
        assert!(matches!(
            b.switch_articulation(root, ArticulationTier::Prescribed, DT),
            Err(PhysicsError::TransitionDeferred {
                reason: "support needs a solved stationary velocity boundary"
            })
        ));
        b.set_world_velocity(pad, [0.; 3], [0.; 3]).unwrap();
        assert!(
            b.switch_articulation(root, ArticulationTier::Prescribed, DT)
                .is_err()
        );
        for _ in 0..2000 {
            b.step(DT).unwrap();
        }
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
    }

    #[test]
    fn reset_discards_retained_features_and_support_hooks() {
        let (mut b, root, tip, floor, _) = settled();
        b.switch_articulation(root, ArticulationTier::Prescribed, DT)
            .unwrap();
        b.step(DT).unwrap();
        assert_eq!(b.held_supports.len(), 2);
        b.reset().unwrap();
        assert!(b.held_supports.is_empty());
        for id in [root, tip] {
            let co = b.world.bodies[b.bodies[&id].0].colliders()[0];
            assert!(
                !b.world.colliders[co]
                    .active_hooks()
                    .contains(ActiveHooks::MODIFY_SOLVER_CONTACTS)
            );
            assert_eq!(b.contact_warmstart(id, floor).unwrap().normal_ns, 0.);
        }
    }
}
