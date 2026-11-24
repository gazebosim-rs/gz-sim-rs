// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Select and check the shared rigid-top integrator on an isolated native span.
use super::*;
use crate::free_rotation::{RotationUpdate, attitude_interval};

pub(crate) struct FreeRotation {
    update: RotationUpdate,
    pub(super) slices: u16,
}
impl FreeRotation {
    pub(super) fn prepare(c: &Candidate, e: &RapierBackend, duration: Duration) -> Option<Self> {
        let b = &e.world.bodies[c.foot];
        if c.inertia.is_none()
            || b.is_sleeping()
            || !b.gyroscopic_forces_enabled()
            || b.angvel() == Vector::ZERO
        {
            return None;
        }
        let props = &b.mass_properties().local_mprops;
        let axes = props.principal_inertia_local_frame;
        let inverse = props.inv_principal_inertia;
        let initial = *b.rotation() * axes;
        let w = initial.inverse() * b.angvel();
        let local_l = inverse.recip() * w;
        let energy = w.dot(local_l);
        let momentum = initial * local_l;
        let evaluate = |count| {
            attitude_interval(
                initial,
                momentum,
                Vector::ZERO,
                inverse,
                duration.as_secs_f64(),
                count,
            )
        };
        let mut coarse = evaluate(1);
        for count in [2, 4, 8, 16, 32] {
            let fine = evaluate(count);
            if let (Ok(a), Ok(b)) = (&coarse, &fine) {
                let delta = *a * b.inverse();
                let angle = 2. * delta.xyz().length().atan2(delta.w.abs());
                let end_l = b.inverse() * momentum;
                let next_w = inverse * end_l;
                // Decline unresolved high-spin increments. Energy and orientation
                // are evaluated independently from the native contact solver.
                if angle <= 1.5e-9 && (next_w.dot(end_l) - energy).abs() <= 2e-12 * energy {
                    let q = (*b * axes.inverse()).normalize();
                    return Some(Self {
                        update: RotationUpdate::new(c.foot, q, *b * next_w),
                        slices: count as u16,
                    });
                }
            }
            coarse = fine;
        }
        None
    }
}
/// Corrections prepared from the actual start of one certified collision-free solve.
#[derive(Default)]
pub(crate) struct GuardedMotion {
    pub(super) rotation: Option<FreeRotation>,
    translation: Option<(RigidBodyHandle, Vector, Vector)>,
}
impl GuardedMotion {
    pub(super) fn prepare(
        c: &Candidate,
        e: &RapierBackend,
        h: Duration,
        rotate: bool,
        ballistic: bool,
    ) -> Self {
        let b = &e.world.bodies[c.foot];
        let rotation = rotate.then(|| FreeRotation::prepare(c, e, h)).flatten();
        let acceleration = e.world.gravity * c.gravity_scale;
        let mut translation = None;
        if ballistic
            && acceleration != Vector::ZERO
            && !b.is_sleeping()
            && (rotation.is_some() || c.inertia.is_none() || b.angvel() == Vector::ZERO)
        {
            let t = h.as_secs_f64();
            let position = center(b) + (b.linvel() + 0.5 * t * acceleration) * t;
            let velocity = b.linvel() + acceleration * t;
            if position.is_finite() && velocity.is_finite() {
                translation = Some((c.foot, position, velocity));
            }
        }
        Self {
            rotation,
            translation,
        }
    }
    pub(super) fn slices(&self) -> u16 {
        self.rotation.as_ref().map_or(0, |r| r.slices)
    }
    pub(super) fn ballistic(&self) -> bool {
        self.translation.is_some()
    }
    pub(super) fn present(&self) -> bool {
        self.rotation.is_some() || self.ballistic()
    }
    pub(crate) fn commit(self, e: &mut RapierBackend) {
        match (self.rotation, self.translation) {
            (Some(r), None) => r.update.commit(&mut e.world),
            (rotation, Some((h, center, velocity))) => {
                let motion = rotation.map_or_else(
                    || {
                        let body = &e.world.bodies[h];
                        RotationUpdate::new(h, *body.rotation(), body.angvel())
                    },
                    |r| r.update,
                );
                motion.commit_with_translation(&mut e.world, Some((center, velocity)));
            }
            _ => {}
        }
    }
}
