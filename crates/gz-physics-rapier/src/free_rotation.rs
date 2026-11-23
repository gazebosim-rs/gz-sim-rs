// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Angular-momentum and energy conserving rotation of one isolated rigid body.
use super::*;

pub(super) struct RotationUpdate {
    handle: RigidBodyHandle,
    attitude: Rotation,
    angular_velocity: Vector,
}
impl RotationUpdate {
    pub(super) fn new(
        handle: RigidBodyHandle,
        attitude: Rotation,
        angular_velocity: Vector,
    ) -> Self {
        Self {
            handle,
            attitude,
            angular_velocity,
        }
    }
    pub(super) fn commit(self, world: &mut PhysicsWorld) {
        self.commit_with_translation(world, None);
    }
    /// A verified collision-free COM arc can supply translation while the
    /// orientation is composed about that same, possibly offset, inertial center.
    pub(super) fn commit_with_translation(
        self,
        world: &mut PhysicsWorld,
        ballistic: Option<(Vector, Vector)>,
    ) {
        let body = &mut world.bodies[self.handle];
        let local_com = body.mass_properties().local_mprops.local_com;
        if let Some((center, velocity)) = ballistic {
            body.set_position(
                Pose::from_parts(center - self.attitude * local_com, self.attitude),
                false,
            );
            body.set_linvel(velocity, false);
        } else if local_com == Vector::ZERO {
            body.set_rotation(self.attitude, false);
        } else {
            // Preserve native translation when the link frame is offset from the COM.
            let center = *body.position() * local_com;
            body.set_position(
                Pose::from_parts(center - self.attitude * local_com, self.attitude),
                false,
            );
        }
        body.set_angvel(self.angular_velocity, false);
        world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut world.colliders);
    }
}
impl RapierBackend {
    /// Opt in one unconstrained, centered dynamic body with no other colliders.
    /// The ordinary native solve takes over whenever another collider or a joint
    /// appears. This local integrator never replaces a native contact impulse.
    pub fn enable_momentum_preserving_rotation(&mut self, id: BodyId) -> Result<(), PhysicsError> {
        let handle = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?.0;
        if !self.admits_free_rotation(id, handle) {
            return Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "momentum-preserving rotation requires one isolated free centered body with finite principal inertia and zero angular damping",
            });
        }
        self.conserved_rotor = Some(id);
        Ok(())
    }

    fn admits_free_rotation(&self, id: BodyId, handle: RigidBodyHandle) -> bool {
        let b = &self.world.bodies[handle];
        let props = &b.mass_properties().local_mprops;
        let inverse = props.inv_principal_inertia;
        self.world.colliders.len() == 1
            && b.colliders().len() == 1
            && self.has_joint(id) == Some(false)
            && b.is_dynamic()
            && b.is_enabled()
            && b.locked_axes().is_empty()
            && b.angular_damping() == 0.
            && props.local_com == Vector::ZERO
            && inverse.is_finite()
            && inverse.min_element() > 0.
            && inverse.recip().is_finite()
    }
    pub(super) fn free_rotation_step(
        &self,
        dt: Duration,
    ) -> Result<Option<RotationUpdate>, PhysicsError> {
        let Some(id) = self.conserved_rotor else {
            return Ok(None);
        };
        let Some((handle, ..)) = self.bodies.get(&id) else {
            return Ok(None);
        };
        if !self.admits_free_rotation(id, *handle) {
            return Ok(None);
        }
        let b = &self.world.bodies[*handle];
        let omega = b.angvel();
        let torque = b.user_torque();
        if omega == Vector::ZERO && torque == Vector::ZERO {
            return Ok(None);
        }
        let m = &b.mass_properties().local_mprops;
        let axes = m.principal_inertia_local_frame;
        let world_frame = *b.rotation() * axes;
        let (attitude, angular_velocity) = integrate(
            world_frame,
            omega,
            torque,
            m.inv_principal_inertia,
            dt.as_secs_f64(),
        )?;
        Ok(Some(RotationUpdate::new(
            *handle,
            (attitude * axes.inverse()).normalize(),
            angular_velocity,
        )))
    }
}

// Integrate body-frame Euler motion by implicit midpoint. Its two quadratic
// invariants are the squared momentum and the kinetic energy. The Cayley step
// for orientation transports that same momentum back into the world. A pair
// of half impulses applies the authored constant world torque symmetrically.
fn integrate(
    q: Rotation,
    omega: Vector,
    torque: Vector,
    inverse: Vector,
    dt: f64,
) -> Result<(Rotation, Vector), PhysicsError> {
    let initial = q * (inverse.recip() * (q.inverse() * omega));
    let target = initial + torque * dt;
    let rate = inverse.max_element() * initial.length().max(target.length());
    // Bound both midpoint iteration contraction and orientation error.
    let count = (dt * rate / 0.10).ceil().max(1.);
    if !count.is_finite() || count > 4096. || !target.is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    let q = attitude_interval(q, initial, torque, inverse, dt, count as u32)?;
    let w = q * (inverse * (q.inverse() * target));
    if !q.is_finite() || !w.is_finite() {
        return Err(PhysicsError::InvalidValue);
    }
    Ok((q, w))
}
/// Shared fourth-order midpoint/Cayley map in principal coordinates.
/// The legacy isolated rotor and the certified inter-contact rotor supply their
/// separate eligibility bounds and subdivision policies.
pub(super) fn attitude_interval(
    mut q: Rotation,
    initial: Vector,
    torque: Vector,
    inverse: Vector,
    dt: f64,
    count: u32,
) -> Result<Rotation, PhysicsError> {
    let h = dt / f64::from(count);
    // The symmetric midpoint/Cayley map is self-adjoint. Three signed stages
    // cancel its cubic local error (2a+b=1, 2a³+b³=0). Each still preserves
    // both free-body invariants. Sample the torque on this same signed clock.
    const OUTER: f64 = 1.3512071919596578; // 1 / (2 - cbrt(2)).
    const INNER: f64 = 1. - 2. * OUTER;
    for i in 0..count {
        let t = h * f64::from(i);
        for (weight, center) in [
            (OUTER, 0.5 * OUTER),
            (INNER, 0.5),
            (OUTER, 1. - 0.5 * OUTER),
        ] {
            let world_mid = initial + torque * (t + h * center);
            q = midpoint(q, world_mid, inverse, h * weight)?;
        }
    }
    Ok(q)
}

fn midpoint(
    q: Rotation,
    world_mid: Vector,
    inverse: Vector,
    h: f64,
) -> Result<Rotation, PhysicsError> {
    let start = q.inverse() * world_mid;
    let mut mid = start;
    let tolerance = 16. * f64::EPSILON * start.length();
    for _ in 0..24 {
        let next = start + (0.5 * h) * mid.cross(inverse * mid);
        if (next - mid).length() <= tolerance {
            let step = 0.5 * h * (inverse * next);
            let cayley = Rotation::from_xyzw(step.x, step.y, step.z, 1.).normalize();
            return Ok((q * cayley).normalize());
        }
        mid = next;
    }
    Err(PhysicsError::InvalidValue)
}
