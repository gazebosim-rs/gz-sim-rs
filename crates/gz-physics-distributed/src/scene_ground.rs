// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::PlaneSpec;

/// The immovable environment has a scene handle but consumes no mobile owner slot.
pub(super) fn handle() -> BodyId {
    BodyId::from_raw(u64::MAX)
}

#[derive(Clone, Copy)]
pub(super) struct SceneGround {
    pub(super) spec: PlaneSpec,
    pub(super) surface: crate::physics::GroundPlane,
}

impl SceneBackend {
    /// Whether this is the immutable environment shared by all workers.
    pub fn is_shared_ground(&self, id: BodyId) -> bool {
        id == handle() && self.ground.is_some()
    }
    pub fn spawn_ground_surface(
        &mut self,
        plane: PlaneSpec,
        friction: f64,
    ) -> Result<BodyId, PhysicsError> {
        let q = plane.orientation;
        let [x, y, z, w] = q;
        let n = [
            2. * (x * z + w * y),
            2. * (y * z - w * x),
            1. - 2. * (x * x + y * y),
        ];
        let norm = n[0].hypot(n[1]).hypot(n[2]);
        let ground =
            crate::physics::GroundPlane::oriented(plane.position, n.map(|c| c / norm), friction)?;
        self.spawn_ground_geometry(plane, ground)
    }

    /// Attach a shared geometry to its immutable scene link. An offset collision
    /// keeps the SDF link and its public handle at their own original pose.
    pub fn spawn_ground_geometry(
        &mut self,
        plane: PlaneSpec,
        ground: crate::physics::GroundPlane,
    ) -> Result<BodyId, PhysicsError> {
        if self.live.is_some() || self.pool.is_some() || self.ground.is_some() {
            return Err(unsupported("one shared ground fixed before startup"));
        }
        if !plane.position.into_iter().all(f64::is_finite)
            || !plane.size.into_iter().all(|s| s.is_finite() && s > 0.)
            || !plane.orientation.into_iter().all(f64::is_finite)
            || (plane.orientation.iter().map(|x| x * x).sum::<f64>() - 1.).abs() > 1e-12
        {
            return Err(PhysicsError::InvalidValue);
        }
        for b in self.loading.values() {
            ground.admits(b).map_err(native)?;
        }
        self.ground = Some(SceneGround {
            spec: plane,
            surface: ground,
        });
        Ok(handle())
    }
}
