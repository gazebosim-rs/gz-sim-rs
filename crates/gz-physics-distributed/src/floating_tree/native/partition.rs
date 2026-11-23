// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::floating_tree::placement::Box3;
use crate::floating_tree::profile::sealed::Recipe;
use gz_physics::{BodyState, PhysicsError};

impl TreeWorld {
    pub(in crate::floating_tree) fn member_packet(
        &self,
        s: &GraspTreeSpec,
        warm: bool,
    ) -> Result<Option<Vec<u8>>> {
        let native = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned native tree");
        let (_, part) = self
            .payload
            .ok_or(invalid("missing independent workpiece"))?;
        let root = self.links[s.payload.tree.root()?].1;
        if warm && let Some(packet) = native.checkpoint_warm_grasp_member(root, part)? {
            return Ok(Some(packet));
        }
        Ok(native.checkpoint_clear_grasp_member(root, part)?)
    }

    pub(in crate::floating_tree) fn detach_grasp(
        &self,
        setup: &GraspTreeSpec,
        previous: &Snapshot,
    ) -> Result<Option<Snapshot>> {
        let native = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned native tree");
        let (_, payload) = self.payload.ok_or(invalid("missing coupled payload"))?;
        let bytes = match native.detached_grasp_tree_image(payload) {
            Ok(bytes) => bytes,
            Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(e) => return Err(e.into()),
        };
        let bare = setup.bare_motor_spec();
        let image = Image::MotorWarm {
            native: bytes,
            motors: previous
                .image
                .motors()
                .ok_or(invalid("missing grasp settings"))?
                .to_vec(),
        };
        let digest = Snapshot::seal(bare.digest(), self.tick(), &image);
        Ok(Some(Snapshot {
            state: previous.state.clone(),
            image,
            digest,
        }))
    }
    pub(in crate::floating_tree) fn combine_grasp(
        setup: &GraspTreeSpec,
        tree: &Snapshot,
        part: &ArticulationLinkState,
    ) -> Result<Self> {
        let mut world = Self::restore(&setup.bare_motor_spec(), tree)?;
        let mut backend = world.sim.world_mut().resource_mut::<PhysicsBackend>();
        let native = backend
            .downcast_mut::<MultiFidelityBackend>()
            .expect("owned native tree");
        let spec = &setup.payload.payload;
        if part.id != spec.id {
            return Err(invalid("incorrect released payload identity"));
        }
        let id = spec.spawn_dynamic(native)?;
        native.keep_body_awake(id)?;
        native.place_detached_sphere(id, as_body(part))?;
        world.recipe = setup.digest();
        world.payload = Some((spec.id, id));
        Ok(world)
    }
    pub(in crate::floating_tree) fn collision_bounds(&self) -> Result<(Vec<Box3>, Option<Box3>)> {
        let native = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned tree");
        let bound = |id| native.grasp_collision_bounds(id, DT).map(Box3::from);
        let tree = self
            .links
            .iter()
            .map(|&(_, id)| bound(id))
            .collect::<std::result::Result<_, _>>()?;
        let part = self.payload.map(|(_, id)| bound(id)).transpose()?;
        Ok((tree, part))
    }
}
pub(in crate::floating_tree) fn as_body(s: &ArticulationLinkState) -> BodyState {
    BodyState {
        position: s.position,
        orientation: s.orientation,
        velocity: s.velocity,
        angular_velocity: s.angular_velocity,
    }
}
