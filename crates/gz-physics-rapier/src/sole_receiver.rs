// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native state exchange for an independently integrated coating between a foot and finite support faces.
use super::*;

impl RapierBackend {
    /// Read the held wrench only while the registered foot and panels own every collider.
    pub fn sole_wrench(
        &self,
        foot: BodyId,
        supports: impl ExactSizeIterator<Item = BodyId> + Clone,
    ) -> Result<([f64; 3], [f64; 3]), PhysicsError> {
        let reject = || PhysicsError::Unsupported {
            engine: "rapier",
            capability: "coated sole requires an isolated free box and fixed or translating box",
        };
        if supports.len() == 0
            || self.bodies.len() != 1 + supports.len()
            || self.world.colliders.len() != self.bodies.len()
            || self.curves.authored()
        {
            return Err(reject());
        }
        let body = |id| {
            let (handle, ..) = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
            let b = &self.world.bodies[*handle];
            let [c] = b.colliders() else {
                return Err(reject());
            };
            if self.has_joint(id) != Some(false)
                || self.world.colliders[*c].shape().as_cuboid().is_none()
            {
                return Err(reject());
            }
            Ok(b)
        };
        for (i, id) in supports.clone().enumerate() {
            let deck = body(id)?;
            if id == foot
                || supports.clone().take(i).any(|other| other == id)
                || deck.is_dynamic()
                || (supports.len() > 1 && deck.angvel() != Vector::ZERO)
            {
                return Err(reject());
            }
        }
        let foot = body(foot)?;
        if !foot.is_dynamic() {
            return Err(reject());
        }
        Ok((foot.user_force().to_array(), foot.user_torque().to_array()))
    }

    /// Prepare all endpoints before publishing any. Only a committed step
    /// consumes the held wrench, including when an adaptive trial failed.
    pub fn commit_sole_panels(
        &mut self,
        foot: (BodyId, BodyState),
        supports: &[(BodyId, BodyState)],
    ) -> Result<(), PhysicsError> {
        self.sole_wrench(foot.0, supports.iter().map(|p| p.0))?;
        let prepare = |(id, state)| -> Result<_, PhysicsError> {
            let (handle, pose) = body_state::prepare(self, id, state)?;
            Ok((id, handle, pose, state))
        };
        let prepared_foot = prepare(foot)?;
        let prepare_deck = |update: &(BodyId, BodyState)| {
            if supports.len() > 1 && update.1.angular_velocity != [0.; 3] {
                return Err(PhysicsError::InvalidValue);
            }
            prepare(*update)
        };
        // The single support hot path does not allocate.
        if let [support] = supports {
            let deck = prepare_deck(support)?;
            self.publish_sole_endpoint(prepared_foot);
            self.publish_sole_endpoint(deck);
        } else {
            let decks = supports
                .iter()
                .map(prepare_deck)
                .collect::<Result<Vec<_>, _>>()?;
            self.publish_sole_endpoint(prepared_foot);
            for deck in decks {
                self.publish_sole_endpoint(deck)
            }
        }
        if self.pending_wrenches.remove(&foot.0) {
            let body = &mut self.world.bodies[prepared_foot.1];
            body.reset_forces(false);
            body.reset_torques(false);
        }
        self.world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut self.world.colliders);
        Ok(())
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
