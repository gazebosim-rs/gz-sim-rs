// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Stationary prescribed pose for an unloaded robot with one fixed world mount.
//! Sleeping the complete native island keeps its constraints, finite collision mass,
//! and warm starts intact. Native collision and commanded loads wake it before solving.
use super::*;
use crate::ArticulationTier;

// Maximum velocity discarded by a stationary cut (SI). Both are checked on every link.
pub(super) const LINEAR_REST: f64 = 0.00025;
pub(super) const ANGULAR_REST: f64 = 0.001;

fn defer(reason: &'static str) -> PhysicsError {
    PhysicsError::TransitionDeferred { reason }
}

// A stop supports only the signed load directed into its active face.
fn supported_coordinate(
    position: f64,
    limits: Option<[f64; 2]>,
    spring: Option<RobotSpring>,
    gravity_effort: f64,
    gravity_scale: f64,
    unstable_gravity_stiffness: Option<f64>,
) -> bool {
    let elastic = spring.map_or(0., |s| s.stiffness * (s.reference - position));
    let load = gravity_effort + elastic;
    if !load.is_finite() || !gravity_scale.is_finite() {
        return false;
    }
    if load.abs() <= 1e-8 * (gravity_scale + elastic.abs()) {
        // At a gravity maximum, damping cannot restore a balanced free axis.
        return unstable_gravity_stiffness.is_none_or(|required| {
            spring.is_some_and(|s| s.stiffness > required)
                || limits.is_some_and(|[lo, hi]| lo == hi && (position - lo).abs() <= 2e-5)
        });
    }
    limits.is_some_and(|[lo, hi]| (position - if load < 0. { lo } else { hi }).abs() <= 2e-5)
}

/// A hanging universal is a stable gravity minimum in both free coordinates when its
/// axes span the horizontal plane. Bound its available angular displacement by the
/// gravitational and spring potential above their simultaneous minima. This also
/// accepts tiny residual oscillations in the native solver's settled pose.
fn hanging_wrist_at_rest(
    weight: Vector,
    moment: Vector,
    downward_lever: f64,
    axes_normal: Vector,
    twice_spring_energy: f64,
) -> bool {
    const MAX_REST_ANGLE: f64 = 2e-5; // radians, the mounted stop tolerance
    if downward_lever <= 0. || weight.dot(axes_normal).abs() < 0.9 * weight.length() {
        return false;
    }
    // With a center below its pivot, |moment| / lever is tan(tilt). Its square
    // bounds twice the gravitational potential excess divided by the downward lever.
    (moment.length() / downward_lever).powi(2) + twice_spring_energy / downward_lever
        <= MAX_REST_ANGLE.powi(2)
}

impl RapierBackend {
    fn robot_parent_joint(&self, link: BodyId) -> Option<(JointId, &Joint)> {
        self.joints
            .records
            .iter()
            .find(|(_, j)| j.initial.robot && j.initial.child == link)
            .map(|(id, j)| (*id, j))
    }
    pub(crate) fn mounted_robot_root(&self, root: BodyId) -> bool {
        self.bodies.get(&root).is_some_and(|e| {
            self.world.multibody_joints.rigid_body_link(e.0).is_none()
                || !self.joints.records.values().any(|j| {
                    matches!(j.handle, Handle::Reduced(_))
                        && (j.initial.child == root || j.initial.parent == Some(root))
                })
        }) && self
            .robot_parent_joint(root)
            .is_some_and(|(_, j)| j.initial.parent.is_none())
    }
    pub(crate) fn robot_member(&self, root: BodyId, mut body: BodyId) -> bool {
        // Impulse graphs can be made programmatically as well as through the SDF tree loader.
        for _ in 0..=self.joints.records.len() {
            if body == root {
                return true;
            }
            let Some(parent) = self
                .robot_parent_joint(body)
                .and_then(|(_, j)| j.initial.parent)
            else {
                return false;
            };
            body = parent;
        }
        false
    }
    pub(crate) fn mounted_members(&self, root: BodyId) -> Vec<BodyId> {
        self.joints
            .records
            .values()
            .filter(|j| j.initial.robot && self.robot_member(root, j.initial.child))
            .map(|j| j.initial.child)
            .collect()
    }
    pub(crate) fn mounted_tier(&self, root: BodyId) -> ArticulationTier {
        if self.parked_robots.get(&root).is_some_and(|hs| {
            hs.iter()
                .all(|h| self.world.bodies.get(*h).is_some_and(|b| b.is_sleeping()))
        }) {
            ArticulationTier::Prescribed
        } else {
            ArticulationTier::Dynamic
        }
    }
    pub(crate) fn retire_woken_robots(&mut self) {
        self.parked_robots.retain(|_, hs| {
            hs.iter()
                .all(|h| self.world.bodies.get(*h).is_some_and(|b| b.is_sleeping()))
        });
    }

    // Evaluate free directions on the authored joint manifold. Solver pivot drift
    // can tilt a nominally horizontal unloaded slide by microradians.
    fn mounted_axis(&self, joint: Construction, local_axis: Vector) -> Vector {
        let mut axis = joint.frame1.rotation * local_axis;
        let mut parent = joint.parent;
        while let Some(id) = parent {
            let (_, j) = self.robot_parent_joint(id).expect("mounted ancestor");
            let c = j.initial;
            axis = c.frame2.rotation.inverse() * axis;
            match c.motion {
                JointMotion::Angular | JointMotion::Screw { .. } if !c.authored_weld() => {
                    axis = Rotation::from_rotation_x(j.position) * axis;
                }
                JointMotion::Ball => {
                    let (parent_frame, child_frame) =
                        c.world_joint_frames(&self.world, &self.bodies);
                    axis = (parent_frame.inverse() * child_frame) * axis;
                }
                JointMotion::Universal(_) => {
                    let [first, second] = j.universal_angles;
                    axis = Rotation::from_rotation_x(first)
                        * (Rotation::from_rotation_y(second) * axis);
                }
                _ => {}
            }
            axis = c.frame1.rotation * axis;
            parent = c.parent;
        }
        axis
    }

    pub(crate) fn validate_mounted_park(
        &self,
        root: BodyId,
        dt: Duration,
    ) -> Result<Vec<RigidBodyHandle>, PhysicsError> {
        if dt.is_zero() {
            return Err(PhysicsError::InvalidValue);
        }
        let members = self.mounted_members(root);
        if members.iter().collect::<BTreeSet<_>>().len() != members.len()
            || self.joints.records.values().any(|j| {
                let c = j.initial;
                (members.contains(&c.child) || c.parent.is_some_and(|p| members.contains(&p)))
                    && (!c.robot
                        || !members.contains(&c.child)
                        || c.parent.is_some_and(|p| !members.contains(&p)))
            })
        {
            return Err(defer("mounted robot has an external or duplicate joint"));
        }
        let handles: Vec<_> = members.iter().map(|id| self.bodies[id].0).collect();
        for &h in &handles {
            let body = &self.world.bodies[h];
            for c in body.colliders() {
                if self.world.narrow_phase.contact_pairs_with(*c).any(|pair| {
                    let other = if pair.collider1 == *c {
                        pair.collider2
                    } else {
                        pair.collider1
                    };
                    (pair.total_impulse_magnitude() > 0.
                        || pair
                            .solver_manifolds()
                            .iter()
                            .any(|m| m.data.solver_contacts.iter().any(|c| c.dist <= 0.)))
                        && !self.world.colliders[other]
                            .parent()
                            .is_some_and(|p| handles.contains(&p))
                }) {
                    return Err(defer("mounted robot has an external contact"));
                }
            }
        }
        for (&id, &h) in members.iter().zip(&handles) {
            let body = &self.world.bodies[h];
            if !body.is_dynamic()
                || body.linvel().length() > LINEAR_REST
                || body.angvel().length() > ANGULAR_REST
                || self.pending_wrenches.contains(&id)
            {
                return Err(defer("mounted robot has motion or a pending wrench"));
            }
            let (joint_id, joint) = self
                .robot_parent_joint(id)
                .expect("collected attached link");
            let c = joint.initial;
            let Handle::Impulse(handle) = joint.handle else {
                return Err(defer("expected an impulse joint"));
            };
            if !self
                .world
                .impulse_joints
                .get(handle)
                .expect("registered joint")
                .data
                .motor_axes
                .is_empty()
                || self.joints.efforts.contains_key(&joint_id)
                || self.joints.angular_efforts.contains_key(&joint_id)
                || (0..2).any(|axis| self.joints.universal_motors.contains_key(&(joint_id, axis)))
            {
                return Err(defer("mounted robot has a pending input or active motor"));
            }
            if c.authored_weld() {
                continue;
            }
            let frame = self.parent_joint_pose(c);
            let (mut force, mut moment) = (Vector::ZERO, Vector::ZERO);
            let (mut force_scale, mut moment_scale) = (0., 0.);
            let mut gravity_lever = 0.;
            for (&descendant, &dh) in members.iter().zip(&handles) {
                if self.robot_member(id, descendant) {
                    let (weight, torque, lever) = self.gravity_about(dh, frame.translation);
                    gravity_lever += lever;
                    force += weight;
                    moment += torque;
                    force_scale += weight.length();
                    moment_scale += torque.length();
                }
            }
            let axis = self.mounted_axis(c, Vector::X);
            let stable = match c.motion {
                JointMotion::Universal(settings) => {
                    let q = joint.universal_angles;
                    let second = self.mounted_axis(c, Rotation::from_rotation_x(q[0]) * Vector::Y);
                    let springs = self.joints.universal_springs.get(&joint_id);
                    let twice_spring_energy = (0..2)
                        .map(|i| {
                            springs.and_then(|rows| rows[i]).map_or(0., |(law, _)| {
                                law.stiffness * (law.reference - q[i]).powi(2)
                            })
                        })
                        .sum();
                    let axes_normal = axis.cross(second);
                    let torque = moment.length();
                    // For the orthogonal universal axes, |r| mg bounds the
                    // negative gravitational curvature. The rotating second axis
                    // adds a mixed derivative bounded by the current moment.
                    let gravity_stiffness = (gravity_lever < 0. && torque <= -2e-5 * gravity_lever)
                        .then(|| gravity_lever.hypot(torque) + torque);
                    hanging_wrist_at_rest(
                        force,
                        moment,
                        gravity_lever,
                        axes_normal,
                        twice_spring_energy,
                    ) || [axis, second].into_iter().enumerate().all(|(i, a)| {
                        let spring = springs.and_then(|rows| rows[i].map(|(law, _)| law));
                        supported_coordinate(
                            q[i],
                            settings.range(i),
                            spring,
                            moment.dot(a),
                            moment_scale,
                            gravity_stiffness,
                        )
                    })
                }
                // The potential of the complete descendant assembly is minimal
                // when its center of mass hangs below the spherical pivot.
                // A center exactly at the pivot or zero gravity is neutral.
                JointMotion::Ball => {
                    gravity_lever.is_finite()
                        && gravity_lever >= 0.
                        && moment_scale.is_finite()
                        && moment.length() <= 1e-8 * moment_scale.max(gravity_lever)
                }
                _ => {
                    let q = self
                        .world_joint_state(joint_id)
                        .expect("scalar joint")
                        .position;
                    let (load, scale) = match c.motion {
                        JointMotion::Prismatic => (force, force_scale),
                        // Helical virtual work per radian includes axial translation.
                        JointMotion::Screw { lead, .. } => (
                            moment + lead * force,
                            moment_scale + lead.abs() * force_scale,
                        ),
                        _ => (moment, moment_scale),
                    };
                    supported_coordinate(q, c.limits, c.spring, load.dot(axis), scale, None)
                }
            };
            if !stable {
                return Err(defer(
                    "gravity or spring preload can move an unrestrained mounted coordinate",
                ));
            }
        }
        if self.joints.speed_caps.iter().any(|(id, caps)| {
            members.contains(&self.joints.records[id].initial.child)
                && caps.iter().flatten().any(|cap| cap.limit() > 0.)
        }) && self.foreign_body_moving(&handles)
        {
            return Err(defer(
                "speed-limited parking needs stationary external bodies",
            ));
        }
        // The first native solve establishes narrow-phase contact pairs and processes
        // new restraints. Until then an apparently unloaded mount can already touch a load.
        if self.joints.construction_open() {
            return Err(defer(
                "mounted parking needs an initial committed contact solve",
            ));
        }
        if !self.can_sleep_mounted_group(&handles) {
            return Err(defer("mounted robot does not own a complete native island"));
        }
        Ok(handles)
    }
    pub(crate) fn switch_mounted_robot(
        &mut self,
        root: BodyId,
        target: ArticulationTier,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        match target {
            ArticulationTier::Prescribed if self.mounted_tier(root) == target => {}
            ArticulationTier::Prescribed => {
                let handles = self.validate_mounted_park(root, dt)?;
                self.park_speed_references(root, &handles)?;
                self.parked_robots.insert(root, handles);
            }
            ArticulationTier::Dynamic => {
                self.resume_speed_references(root);
                if let Some(handles) = self.parked_robots.remove(&root) {
                    for h in handles {
                        self.world.islands.wake_up(&mut self.world.bodies, h, true);
                    }
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "../../tests/private/mounted_ball_axis.rs"]
mod mounted_ball_axis;

#[cfg(test)]
mod hanging_wrist_tests {
    use super::*;
    #[test]
    fn a_free_neighbor_across_a_small_gap_keeps_its_mass_and_delivers_the_same_first_impact() {
        let run = |park: bool| {
            let dt = Duration::from_millis(1);
            let mut b = RapierBackend::new([0.; 3]).unwrap();
            let mut ball = |x| {
                b.spawn(BodySpec {
                    shape: Shape::Sphere { radius: 0.05 },
                    position: [x, 0., 0.],
                    velocity: [0.; 3],
                    mass: Some(1.),
                })
                .unwrap()
            };
            let root = ball(0.);
            let tip = ball(0.3);
            let parcel = ball(0.4005);
            b.add_robot_joint(
                None,
                root,
                [0.; 3],
                RobotJoint::Revolute {
                    axis: [0., 0., 1.],
                    limits: None,
                },
            )
            .unwrap();
            b.add_robot_joint(
                Some(root),
                tip,
                [0.3, 0., 0.],
                RobotJoint::Prismatic {
                    axis: [1., 0., 0.],
                    limits: None,
                },
            )
            .unwrap();
            b.step(dt).unwrap();
            let handles = [b.bodies[&root].0, b.bodies[&tip].0];
            let foreign = b.bodies[&parcel].0;
            assert!(
                !b.world
                    .islands
                    .can_sleep_connected_group(&b.world.bodies, &handles),
                "the native predictive edge must actually connect the actors"
            );
            if park {
                b.switch_articulation(root, ArticulationTier::Prescribed, dt)
                    .unwrap();
                assert_eq!(b.parked_speed[&root].separated.len(), 1);
                assert!(b.world.islands.active_bodies().any(|h| h == foreign));
            }
            for _ in 0..5 {
                b.step(dt).unwrap();
            }
            if park {
                assert_eq!(b.mounted_tier(root), ArticulationTier::Prescribed);
            }
            assert_eq!(b.state(parcel).unwrap().velocity, [0.; 3]);
            b.apply_impulse(parcel, [-1., 0., 0.], [0.; 3]).unwrap();
            for tick in 1..20 {
                b.step(dt).unwrap();
                assert_eq!(b.mounted_tier(root), ArticulationTier::Dynamic);
                if let Some(c) = b.rigid_contact_observations().iter().find(|c| {
                    ([c.a, c.b] == [parcel, tip] || [c.b, c.a] == [parcel, tip])
                        && c.normal_impulse > 0.
                }) {
                    return (
                        tick,
                        c.normal_impulse,
                        b.state(tip).unwrap(),
                        b.state(parcel).unwrap(),
                    );
                }
            }
            panic!("missing first impact");
        };
        let control = run(false);
        let switched = run(true);
        assert_eq!(control.0, switched.0);
        assert!(
            (control.1 - switched.1).abs() < 1e-10,
            "{control:?} {switched:?}"
        );
        assert_eq!((control.2, control.3), (switched.2, switched.3));
    }

    #[test]
    fn retiring_a_prediction_refuses_event_pairs_penetration_and_incomplete_cuts_atomically() {
        let mut w = PhysicsWorld::default();
        w.gravity = Vector::ZERO;
        let actors = [0., 0.1005, -0.09, 2., 3., 3.1005].map(|x| {
            let h = w.insert_body(RigidBodyBuilder::dynamic().translation(Vector::X * x));
            let c = w.insert_collider(ColliderBuilder::ball(0.05), Some(h));
            (h, c)
        });
        let [
            (robot, a),
            (neighbor, b),
            (contacting, c),
            (unrelated, d),
            (isolated, e),
            (_, f),
        ] = actors;
        w.step();
        let invalid = ColliderHandle::invalid();
        // These colliders exist publicly before the next advancing detection pass.
        let stale = w.insert_collider(ColliderBuilder::ball(0.01), Some(unrelated));
        w.remove_collider(stale).unwrap();
        let unparented = w.insert_collider(ColliderBuilder::ball(0.01), None);
        let fresh_left = w.insert_collider(ColliderBuilder::ball(0.01), Some(robot));
        let fresh_right = w.insert_collider(ColliderBuilder::ball(0.01), Some(neighbor));
        let new_body = w.insert_body(RigidBodyBuilder::dynamic());
        let unregistered = w.insert_collider(ColliderBuilder::ball(0.01), Some(new_body));
        for pairs in [
            vec![],
            vec![(invalid, b)],
            vec![(a, invalid)],
            vec![(stale, b)],
            vec![(a, stale)],
            vec![(unparented, b)],
            vec![(a, unparented)],
            vec![(fresh_left, b)],
            vec![(a, fresh_right)],
            vec![(unregistered, b)],
            vec![(b, a)],
            vec![(a, d)],
            vec![(a, c)],
            vec![(a, b)],
            vec![(a, b), (a, c)],
        ] {
            assert!(!w.islands.can_sleep_with_separated_predictions(
                &w.bodies,
                &[robot],
                &w.colliders,
                &w.narrow_phase,
                &pairs
            ));
            assert!(!w.islands.sleep_with_separated_predictions(
                &mut w.bodies,
                &[robot],
                &w.colliders,
                &mut w.narrow_phase,
                &pairs
            ));
            assert!(
                w.narrow_phase
                    .contact_pair(a, b)
                    .unwrap()
                    .has_any_active_contact()
            );
            assert!(w.islands.active_bodies().any(|h| h == robot));
        }
        // Both nodes already have live contact edges to a, but do not touch one another.
        assert!(w.narrow_phase.contact_pair(c, b).is_none());
        assert!(!w.islands.can_sleep_with_separated_predictions(
            &w.bodies,
            &[contacting],
            &w.colliders,
            &w.narrow_phase,
            &[(c, b)]
        ));
        // The separate isolated pair has a positive gap with zero load. Two
        // observations of its edge still identify a single valid cut.
        assert!(w.islands.can_sleep_with_separated_predictions(
            &w.bodies,
            &[isolated],
            &w.colliders,
            &w.narrow_phase,
            &[(e, f), (e, f)]
        ));
        w.colliders[b].set_active_events(ActiveEvents::COLLISION_EVENTS);
        assert!(!w.islands.can_sleep_with_separated_predictions(
            &w.bodies,
            &[robot],
            &w.colliders,
            &w.narrow_phase,
            &[(a, b)]
        ));
    }

    #[test]
    fn a_removed_bridge_can_split_at_handoff_but_a_live_contact_or_joint_cannot() {
        let mut w = PhysicsWorld::default();
        w.gravity = Vector::ZERO;
        // Contacts connect a to its satellite at the left, and d to its
        // satellite at the right. Two rotor assemblies share one removable bridge.
        let centers = [0., 0.08, -0.5, 1.0, 1.08, 1.5, 2.];
        let bodies = centers.map(|x| {
            let h = w.insert_body(RigidBodyBuilder::dynamic().translation(Vector::X * x));
            w.insert_collider(ColliderBuilder::ball(0.05), Some(h));
            h
        });
        let [a, b, c, d, e, f, bridge] = bodies;
        for (parent, child) in [(0, 2), (3, 5), (2, 6), (6, 5)] {
            w.insert_impulse_joint(
                bodies[parent],
                bodies[child],
                FixedJointBuilder::new()
                    .local_anchor1(Vector::X * (centers[child] - centers[parent])),
            );
        }
        w.step();
        let island = w.islands.persistent_island_of(&w.bodies, a).unwrap();
        assert!(
            bodies
                .iter()
                .all(|h| w.islands.persistent_island_of(&w.bodies, *h) == Some(island))
        );
        w.remove_body(bridge).unwrap();
        assert_eq!(w.islands.persistent_island_of(&w.bodies, d), Some(island));
        // The current graph still requires b (live contact) and c (live joint).
        for incomplete in [vec![a, c], vec![a, b]] {
            assert!(!w.islands.can_sleep_connected_group(&w.bodies, &incomplete));
            assert!(!w.islands.sleep_connected_group(
                &mut w.bodies,
                &mut w.narrow_phase,
                &incomplete
            ));
            assert_eq!(w.islands.persistent_island_of(&w.bodies, d), Some(island));
            assert!(!w.bodies[a].is_sleeping());
        }
        assert!(w.islands.can_sleep_connected_group(&w.bodies, &[a, b, c]));
        assert!(
            w.islands
                .sleep_connected_group(&mut w.bodies, &mut w.narrow_phase, &[a, b, c])
        );
        assert_ne!(
            w.islands.persistent_island_of(&w.bodies, a),
            w.islands.persistent_island_of(&w.bodies, d)
        );
        assert!([d, e, f].iter().all(|h| !w.bodies[*h].is_sleeping()));
        assert!([a, b, c].iter().all(|h| w.bodies[*h].is_sleeping()));
        w.islands.wake_up(&mut w.bodies, b, true);
        assert!([a, b, c].iter().all(|h| !w.bodies[*h].is_sleeping()));
    }

    #[test]
    fn sleeping_a_complete_group_updates_solver_membership_and_wakes_the_whole_graph() {
        let mut w = PhysicsWorld::default();
        let a = w.insert_body(RigidBodyBuilder::dynamic().additional_mass(1.));
        let b = w.insert_body(RigidBodyBuilder::dynamic().additional_mass(1.));
        let c = w.insert_body(RigidBodyBuilder::dynamic().additional_mass(1.));
        w.insert_impulse_joint(a, b, FixedJointBuilder::new());
        w.step();
        assert!(
            !w.islands
                .can_sleep_connected_group(&w.bodies, &[RigidBodyHandle::invalid()])
        );
        assert!(
            !w.islands
                .sleep_connected_group(&mut w.bodies, &mut w.narrow_phase, &[a])
        );
        assert!(w.islands.active_bodies().any(|v| v == a));
        assert!(
            w.islands
                .sleep_connected_group(&mut w.bodies, &mut w.narrow_phase, &[a, b])
        );
        assert!(
            w.islands
                .sleep_connected_group(&mut w.bodies, &mut w.narrow_phase, &[b, a])
        );
        assert_eq!(w.islands.active_bodies().collect::<Vec<_>>(), vec![c]);
        w.step();
        assert_eq!(w.bodies[a].linvel(), Vector::ZERO);
        assert_eq!(w.bodies[b].linvel(), Vector::ZERO);
        w.islands.wake_up(&mut w.bodies, b, true);
        assert!(w.islands.active_bodies().any(|v| v == a));
        assert!(!w.bodies[a].is_sleeping());
        w.step();
        assert!(w.bodies[a].linvel().y < 0.);
    }

    #[test]
    fn only_a_near_vertical_stably_suspended_wrist_with_small_stored_energy_can_park() {
        let admits = |tilt_moment, lever, normal, spring| {
            hanging_wrist_at_rest(
                -10. * Vector::Z,
                tilt_moment * Vector::X,
                lever,
                normal,
                spring,
            )
        };
        assert!(admits(1e-5, 1., Vector::Z, 1e-10));
        assert!(!admits(3e-5, 1., Vector::Z, 0.));
        assert!(!admits(0., -1., Vector::Z, 0.));
        assert!(!admits(0., 0., Vector::Z, 0.));
        assert!(!admits(0., 1., Vector::X, 0.));
        assert!(!admits(0., 1., Vector::Z, 1e-8));
    }
}
