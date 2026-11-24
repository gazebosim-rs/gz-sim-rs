// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Restore donor-local geometry and warm impulses on Rapier's first actual pair query.
use crate::contact_dispatcher::Prepare;
use rapier3d_f64::{geometry::ContactManifold, parry::shape::Shape};
use std::{collections::BTreeMap, sync::Mutex};

pub(super) type Cache = BTreeMap<(usize, usize), ContactManifold>;
pub(super) fn identity(shape: &dyn Shape) -> usize {
    // The owning native colliders hold these distinct Arc allocations across this
    // one step. This process-local lookup key never enters a snapshot or the wire.
    std::ptr::from_ref(shape).cast::<()>().addr()
}
pub(super) struct RestorePairs(pub Mutex<Cache>);

impl Prepare for RestorePairs {
    fn prepare(&self, _: &rapier3d_f64::math::Pose, a: &dyn Shape, b: &dyn Shape, manifolds: &mut Vec<ContactManifold>) {
        let (a_id, b_id) = (identity(a), identity(b));
        let mut cache = self.0.lock().expect("native manifold restoration");
        let direct = cache.remove(&(a_id, b_id));
        let saved = direct.or_else(|| {
            cache.remove(&(b_id, a_id)).map(|mut m| {
                std::mem::swap(&mut m.local_n1, &mut m.local_n2);
                for p in &mut m.points {
                    std::mem::swap(&mut p.local_p1, &mut p.local_p2);
                    std::mem::swap(&mut p.fid1, &mut p.fid2);
                    std::mem::swap(&mut p.data.solver_dp1, &mut p.data.solver_dp2);
                    // Rapier uses the world-space tangent on body one to warm-start
                    // 3D friction. Reversing the pair reverses that vector.
                    p.data.warmstart_tangent_world = -p.data.warmstart_tangent_world;
                }
                m
            })
        });
        drop(cache);
        if let Some(saved) = saved {
            debug_assert!(manifolds.is_empty());
            manifolds.clear();
            manifolds.push(saved);
        }
    }
}
