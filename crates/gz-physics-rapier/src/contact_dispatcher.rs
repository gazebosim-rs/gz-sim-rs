// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Use native Parry queries, optionally preparing a contact manifold before matching.
use rapier3d_f64::prelude::RigidBodyHandle;
use rapier3d_f64::{
    geometry::{ContactData, ContactManifold, ContactManifoldData},
    math::{Pose, Real, Vector},
    parry::{
        query::{
            self as q, DefaultQueryDispatcher as Native, PersistentQueryDispatcher,
            QueryDispatcher, Unsupported,
        },
        shape::Shape,
    },
};

pub(super) trait Prepare: Send + Sync {
    fn prepare(
        &self,
        pose: &Pose,
        a: &dyn Shape,
        b: &dyn Shape,
        manifolds: &mut Vec<ContactManifold>,
    );
}
pub(super) struct NativeContacts<T>(pub T);
type BodyPair = (RigidBodyHandle, RigidBodyHandle);

#[derive(Clone)]
pub(super) struct DiscardPairs(std::sync::Arc<std::sync::Mutex<Vec<BodyPair>>>);

impl DiscardPairs {
    pub(super) fn new(pairs: Vec<BodyPair>) -> Self {
        Self(std::sync::Arc::new(std::sync::Mutex::new(pairs)))
    }
    pub(super) fn remaining(self) -> Vec<BodyPair> {
        std::mem::take(&mut *self.0.lock().expect("contact discard queue"))
    }
}

impl Prepare for DiscardPairs {
    fn prepare(
        &self,
        _: &Pose,
        _: &dyn Shape,
        _: &dyn Shape,
        manifolds: &mut Vec<ContactManifold>,
    ) {
        // Multi-support capture is restricted to one convex collider per body.
        // Use generational parent identities even if pads share immutable geometry.
        let Some((a, b)) = manifolds
            .iter()
            .find_map(|m| m.data.rigid_body1.zip(m.data.rigid_body2))
        else {
            return;
        };
        let mut queue = self.0.lock().expect("contact discard queue");
        let Some(index) = queue
            .iter()
            .position(|&(x, y)| (a, b) == (x, y) || (a, b) == (y, x))
        else {
            return;
        };
        queue.swap_remove(index);
        drop(queue);
        // Clear before Parry matches the old points or the native solver reads
        // their guesses. A later substep must keep any impulses solved since.
        for m in manifolds {
            for point in &mut m.points {
                point.data = ContactData::default();
            }
        }
    }
}

impl<T: Prepare> PersistentQueryDispatcher<ContactManifoldData, ContactData> for NativeContacts<T> {
    fn contact_manifolds(
        &self,
        pose: &Pose,
        a: &dyn Shape,
        b: &dyn Shape,
        prediction: Real,
        manifolds: &mut Vec<ContactManifold>,
        workspace: &mut Option<q::ContactManifoldsWorkspace>,
    ) -> Result<(), Unsupported> {
        self.0.prepare(pose, a, b, manifolds);
        Native.contact_manifolds(pose, a, b, prediction, manifolds, workspace)
    }
    fn contact_manifold_convex_convex(
        &self,
        pose: &Pose,
        a: &dyn Shape,
        b: &dyn Shape,
        ca: Option<&dyn q::details::NormalConstraints>,
        cb: Option<&dyn q::details::NormalConstraints>,
        prediction: Real,
        manifold: &mut ContactManifold,
    ) -> Result<(), Unsupported> {
        Native.contact_manifold_convex_convex(pose, a, b, ca, cb, prediction, manifold)
    }
}

// Ordinary geometric queries use the maintained Parry implementation.
impl<T: Prepare> QueryDispatcher for NativeContacts<T> {
    fn intersection_test(
        &self,
        p: &Pose,
        a: &dyn Shape,
        b: &dyn Shape,
    ) -> Result<q::ShapeIntersection, Unsupported> {
        Native.intersection_test(p, a, b)
    }
    fn distance(
        &self,
        p: &Pose,
        a: &dyn Shape,
        b: &dyn Shape,
    ) -> Result<q::ShapeDistance, Unsupported> {
        Native.distance(p, a, b)
    }
    fn contact(
        &self,
        p: &Pose,
        a: &dyn Shape,
        b: &dyn Shape,
        prediction: Real,
    ) -> Result<Option<q::Contact>, Unsupported> {
        Native.contact(p, a, b, prediction)
    }
    fn closest_points(
        &self,
        p: &Pose,
        a: &dyn Shape,
        b: &dyn Shape,
        dist: Real,
    ) -> Result<q::ClosestPoints, Unsupported> {
        Native.closest_points(p, a, b, dist)
    }
    fn cast_shapes(
        &self,
        p: &Pose,
        v: Vector,
        a: &dyn Shape,
        b: &dyn Shape,
        opts: q::ShapeCastOptions,
    ) -> Result<Option<q::ShapeCastHit>, Unsupported> {
        Native.cast_shapes(p, v, a, b, opts)
    }
    fn cast_shapes_nonlinear(
        &self,
        ma: &q::NonlinearRigidMotion,
        a: &dyn Shape,
        mb: &q::NonlinearRigidMotion,
        b: &dyn Shape,
        start: Real,
        end: Real,
        stop: bool,
    ) -> Result<Option<q::ShapeCastHit>, Unsupported> {
        Native.cast_shapes_nonlinear(ma, a, mb, b, start, end, stop)
    }
}
