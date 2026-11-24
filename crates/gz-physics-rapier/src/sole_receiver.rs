// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native state exchange for an independently integrated coating between a foot and finite support faces.
use super::*;

/// Native bodies assigned to a coating owner. A compound contains two boxes on one body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SoleFoot {
    Box(BodyId),
    Compound(BodyId),
    Weld { lower: BodyId, upper: BodyId },
}
impl From<BodyId> for SoleFoot {
    fn from(id: BodyId) -> Self {
        Self::Box(id)
    }
}
impl SoleFoot {
    pub fn lower(self) -> BodyId {
        match self {
            Self::Box(id) | Self::Compound(id) => id,
            Self::Weld { lower, .. } => lower,
        }
    }
}
fn unsupported() -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "rapier",
        capability: "coated sole requires its declared free box, two-box compound or floating two-link weld and its enrolled box supports",
    }
}
impl RapierBackend {
    /// Read force and moment about the foot assembly COM, without consuming the input.
    pub fn sole_wrench(
        &self,
        binding: impl Into<SoleFoot>,
        supports: impl ExactSizeIterator<Item = BodyId> + Clone,
    ) -> Result<([f64; 3], [f64; 3]), PhysicsError> {
        let binding = binding.into();
        let foot = binding.lower();
        let upper = if let SoleFoot::Weld { upper, .. } = binding {
            Some(upper)
        } else {
            None
        };
        if supports.len() == 0
            || self.bodies.len() != 1 + usize::from(upper.is_some()) + supports.len()
            || self.world.colliders.len() != self.bodies.len()
            || self.curves.authored()
        {
            return Err(unsupported());
        }
        let body = |id, compound| {
            let (handle, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
            let rb = &self.world.bodies[*handle];
            let [c] = rb.colliders() else {
                return Err(unsupported());
            };
            let shape = self.world.colliders[*c].shape();
            let valid = if compound {
                shape.as_compound().is_some_and(|s| {
                    s.shapes().len() == 2 && s.shapes().iter().all(|(_, p)| p.as_cuboid().is_some())
                })
            } else {
                shape.as_cuboid().is_some()
            };
            if !valid {
                return Err(unsupported());
            }
            Ok(rb)
        };
        for (i, id) in supports.clone().enumerate() {
            let rb = body(id, false)?;
            if id == foot
                || Some(id) == upper
                || supports.clone().take(i).any(|p| p == id)
                || self.has_joint(id) != Some(false)
                || rb.is_dynamic()
                || (supports.len() > 1 && rb.angvel() != Vector::ZERO)
            {
                return Err(unsupported());
            }
        }
        let rb = body(foot, matches!(binding, SoleFoot::Compound(_)))?;
        if !rb.is_dynamic() {
            return Err(unsupported());
        }
        if let Some(other) = upper {
            let _ = body(other, false)?;
            let a = self.welded_assembly_state(foot)?;
            let index = |id| {
                self.world
                    .multibody_joints
                    .rigid_body_link(self.bodies[&id].0)
                    .map(|l| l.multibody)
            };
            if other == foot || a.links != 2 || index(foot) != index(other) {
                return Err(unsupported());
            }
            Ok((a.force, a.torque))
        } else if self.has_joint(foot) == Some(false) {
            Ok((rb.user_force().to_array(), rb.user_torque().to_array()))
        } else {
            Err(unsupported())
        }
    }

    /// Prepare all endpoints before publishing any. Only a committed step
    /// consumes the held wrench, including when an adaptive trial failed.
    pub fn commit_sole_panels(
        &mut self,
        foot: (impl Into<SoleFoot>, BodyState),
        supports: &[(BodyId, BodyState)],
    ) -> Result<(), PhysicsError> {
        let binding = foot.0.into();
        self.sole_wrench(binding, supports.iter().map(|p| p.0))?;
        let prepare = |(id, state)| -> Result<_, PhysicsError> {
            let (handle, pose) = body_state::prepare(self, id, state)?;
            Ok((id, handle, pose, state))
        };

        let prepare_deck = |update: &(BodyId, BodyState)| {
            if supports.len() > 1 && update.1.angular_velocity != [0.; 3] {
                return Err(PhysicsError::InvalidValue);
            }
            prepare(*update)
        };
        let publish_foot = |this: &mut Self| {
            if let SoleFoot::Weld { lower, upper } = binding {
                this.commit_welded_sole(lower, upper, foot.1)
            } else {
                let id = binding.lower();
                let (handle, pose) = body_state::prepare(this, id, foot.1)?;
                this.publish_sole_endpoint((id, handle, pose, foot.1));
                this.consume_sole_wrench(id, handle);
                Ok(())
            }
        };
        if let [support] = supports {
            let deck = prepare_deck(support)?;
            publish_foot(self)?;
            self.publish_sole_endpoint(deck);
        } else {
            let decks = supports
                .iter()
                .map(prepare_deck)
                .collect::<Result<Vec<_>, _>>()?;
            publish_foot(self)?;
            for deck in decks {
                self.publish_sole_endpoint(deck);
            }
        }
        self.world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut self.world.colliders);
        Ok(())
    }

    // The candidate pose uses the lower link frame and the velocity is at aggregate COM.
    // Prepare all finite poses and twists before publishing the six generalized rates.
    fn commit_welded_sole(
        &mut self,
        lower: BodyId,
        upper: BodyId,
        end: BodyState,
    ) -> Result<(), PhysicsError> {
        let aggregate = self.welded_assembly_state(lower)?;
        let lower_state = self.state(lower).expect("validated welded lower link");
        let new_rotation = rotation(end.orientation)?;
        let turn = (new_rotation * rotation(lower_state.orientation)?.inverse()).normalize();
        let point = |p: Vector| {
            Vector::from(end.position) + turn * (p - Vector::from(lower_state.position))
        };
        let center = point(Vector::from(aggregate.center));
        let spin = Vector::from(end.angular_velocity);
        let links = [lower, upper].map(|id| {
            let h = self.bodies[&id].0;
            let rb = &self.world.bodies[h];
            let pose = Pose::from_parts(
                point(rb.translation()),
                (turn * (*rb.rotation())).normalize(),
            );
            let velocity =
                Vector::from(end.velocity) + spin.cross(point(rb.center_of_mass()) - center);
            (id, h, pose, velocity)
        });
        for &(_, h, pose, v) in &links {
            let collider = &self.world.colliders[self.world.bodies[h].colliders()[0]];
            let bounds = collider
                .shape()
                .compute_aabb(&(pose * collider.position_wrt_parent().unwrap()));
            if !bounds.mins.is_finite()
                || !bounds.maxs.is_finite()
                || !v.is_finite()
                || !spin.is_finite()
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        let (_, root, pose, velocity) =
            *links.iter().find(|(id, ..)| *id == aggregate.root).unwrap();
        self.release_welded_projection();
        if self.joints.construction_open() {
            self.joints.initialize_velocities(&mut self.world);
            self.joints.close();
        }
        self.world.bodies[root].set_position(pose, true);
        let tree_id = self
            .world
            .multibody_joints
            .rigid_body_link(root)
            .unwrap()
            .multibody;
        let tree = self
            .world
            .multibody_joints
            .get_multibody_mut(tree_id)
            .unwrap();
        tree.forward_kinematics(&self.world.bodies, true);
        tree.update_rigid_bodies(&mut self.world.bodies, true);
        let mut qdot = tree.generalized_velocity_mut();
        qdot.as_mut_slice()[..3].copy_from_slice(&velocity.to_array());
        qdot.as_mut_slice()[3..].copy_from_slice(&spin.to_array());
        for (id, h, _, v) in links {
            let body = &mut self.world.bodies[h];
            body.set_linvel(v, true);
            body.set_angvel(spin, true);
            self.consume_sole_wrench(id, h);
        }
        Ok(())
    }

    fn consume_sole_wrench(&mut self, id: BodyId, handle: RigidBodyHandle) {
        if self.pending_wrenches.remove(&id) {
            let body = &mut self.world.bodies[handle];
            body.reset_forces(false);
            body.reset_torques(false);
        }
    }

    fn publish_sole_endpoint(
        &mut self,
        (id, handle, pose, state): (BodyId, RigidBodyHandle, Pose, BodyState),
    ) {
        let body = &mut self.world.bodies[handle];
        body.set_position(pose, true);
        body.set_linvel(Vector::from(state.velocity), true);
        body.set_angvel(Vector::from(state.angular_velocity), true);
        self.record_translation_start(id, handle);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn invalid_last_panel_keeps_every_pose_and_the_held_wrench() {
        let mut r = RapierBackend::new([0.; 3]).unwrap();
        let spec = |x, mass| BodySpec {
            shape: Shape::Box {
                half_extents: [0.1; 3],
            },
            position: [x, 0., 0.],
            velocity: [0.; 3],
            mass,
        };
        let foot = r.spawn(spec(0., Some(2.))).unwrap();
        let decks = [
            r.spawn(spec(-1., None)).unwrap(),
            r.spawn(spec(1., None)).unwrap(),
        ];
        let foot_start = r.state(foot).unwrap();
        let mut foot_end = foot_start;
        foot_end.position[2] += 0.01;
        r.add_world_wrench(foot, [1., 2., 3.], [3., 2., 1.])
            .unwrap();
        let starts = decks.map(|id| r.state(id).unwrap());
        let ends = decks.map(|id| (id, r.state(id).unwrap()));
        let mut bad = ends;
        bad[1].1.orientation = [0.; 4];
        assert!(r.commit_sole_panels((foot, foot_end), &bad).is_err());
        assert_eq!(decks.map(|id| r.state(id).unwrap()), starts);
        assert_eq!(r.state(foot).unwrap(), foot_start);
        assert_eq!(
            r.sole_wrench(foot, decks.into_iter()).unwrap(),
            ([1., 2., 3.], [3., 2., 1.])
        );
        assert!(r.sole_wrench(foot, [decks[0]; 2].into_iter()).is_err());
        r.commit_sole_panels((foot, foot_end), &ends).unwrap();
        assert_eq!(r.state(foot).unwrap(), foot_end);
        assert_eq!(
            r.sole_wrench(foot, decks.into_iter()).unwrap(),
            ([0.; 3], [0.; 3])
        );
    }
}
