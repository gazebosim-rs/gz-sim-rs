// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use rapier3d_f64::parry::{
    bounding_volume::{Aabb, BoundingVolume},
    shape::Shape,
};

/// A free body's linear velocity is at its COM. The force is already staged
/// by PhysicsPrepare, whereas Rapier derives effective gravity during step.
pub(crate) fn free_sweep(
    body: &RigidBody,
    shape: &dyn Shape,
    pose: &Pose,
    gravity: Vector,
    speed_cap: f64,
    dt: f64,
) -> (Vector, f64) {
    if body.is_sleeping() {
        return (Vector::ZERO, 0.);
    }
    let props = body.mass_properties();
    let acceleration = (body.user_force()
        + gravity * props.effective_mass() * body.gravity_scale())
        * props.effective_inv_mass;
    let linear = body.linvel();
    // Relative to the initial-velocity chord, semi-implicit integration of
    // constant acceleration adds at most |a|dt², for any substep partition.
    // Damping and Rapier's per-substep speed cap can shorten that chord.
    let speed = linear.length();
    let mut margin = acceleration.length() * dt * dt
        + speed * (body.linear_damping() * dt).min(1.) * dt
        + (speed - speed_cap).max(0.) * dt;

    let sphere = shape.compute_bounding_sphere(pose);
    let offset = (sphere.center - body.center_of_mass()).length();
    let reach = if shape.as_ball().is_some() {
        offset
    } else {
        offset + sphere.radius
    };
    if reach != 0. && (body.angvel() != Vector::ZERO || body.user_torque() != Vector::ZERO) {
        let inverse = props.local_mprops.inv_principal_inertia;
        let torque_delta = body.user_torque().length() * inverse.max_element() * dt;
        let angle = if !body.gyroscopic_forces_enabled() {
            (body.angvel().length() + torque_delta) * dt
        } else if inverse.min_element() > 0. {
            // Rapier preserves |Iω| in each gyroscopic update. Its subsequent
            // rotation is about ω, so |Iω| is also unchanged across pose updates.
            // Each staged torque increment can add at most I_max |Δω|.
            let frame = *body.rotation() * props.local_mprops.principal_inertia_local_frame;
            let momentum = inverse.recip() * (frame.inverse() * body.angvel());
            inverse.max_element() * (momentum.length() + torque_delta / inverse.min_element()) * dt
        } else {
            2.
        };
        margin += reach * angle.min(2.);
    }
    (linear, margin)
}

pub(crate) fn swept_aabb(
    shape: &dyn Shape,
    pose: &Pose,
    velocity: Vector,
    margin: f64,
    dt: f64,
) -> Aabb {
    let aabb = shape.compute_aabb(pose);
    aabb.merged(&aabb.translated(velocity * dt))
        .loosened(margin)
}

pub(crate) fn deflected_aabb(
    body: &RigidBody,
    shape: &dyn Shape,
    pose: &Pose,
    travel: f64,
) -> Aabb {
    let sphere = shape.compute_bounding_sphere(pose);
    let com = body.center_of_mass();
    let radius = (sphere.center - com).length() + sphere.radius + travel;
    Aabb::from_half_extents(com, Vector::splat(radius))
}

/// A halfspace's native AABB is infinite. Test the actual plane, including skin.
fn fixed_intersections(
    fixed: &Collider,
    pose: &Pose,
    free: &Aabb,
    reachable: &Aabb,
) -> (bool, bool) {
    if let Some(plane) = fixed.shape().as_halfspace() {
        let normal = pose.rotation * plane.normal;
        let intersects = |a: &Aabb| {
            normal.dot(a.center() - pose.translation)
                <= normal.abs().dot(a.half_extents()) + fixed.contact_skin()
        };
        (intersects(free), intersects(reachable))
    } else {
        let bounds = fixed
            .shape()
            .compute_aabb(pose)
            .loosened(fixed.contact_skin());
        (free.intersects(&bounds), reachable.intersects(&bounds))
    }
}

impl RapierBackend {
    /// A third contact can change the payload velocity during the same tick.
    /// Enter this scan only if such a deflection could reach the selected arm.
    /// Until its first outside contact the payload follows its free-flight tube.
    pub(super) fn possible_payload_deflection(
        &self,
        tree: MultibodyIndex,
        payload: RigidBodyHandle,
        dt: Duration,
    ) -> Result<bool, PhysicsError> {
        let seconds = dt.as_secs_f64();
        let params = &self.world.integration_parameters;
        let cap = params.max_linear_velocity();
        let payload_body = &self.world.bodies[payload];
        let mut planar_support = false;
        let mut reachable_neighbor = false;
        for handle in payload_body.colliders() {
            let collider = &self.world.colliders[*handle];
            if !collider.is_enabled() || collider.is_sensor() {
                continue;
            }
            let pose =
                payload_body.position() * collider.position_wrt_parent().expect("attached payload");
            let (v, margin) = free_sweep(
                payload_body,
                collider.shape(),
                &pose,
                self.world.gravity,
                cap,
                seconds,
            );
            let skin = collider.contact_skin()
                + params.prediction_distance()
                + payload_body.soft_ccd_prediction();
            let free = swept_aabb(collider.shape(), &pose, v, margin + skin, seconds);
            let reachable =
                deflected_aabb(payload_body, collider.shape(), &pose, cap * seconds + skin);

            let mut attached_shapes = 0;
            for (id, (body_handle, ..)) in &self.bodies {
                let body = &self.world.bodies[*body_handle];
                attached_shapes += body.colliders().len();
                if *body_handle == payload
                    || self
                        .world
                        .multibody_joints
                        .rigid_body_link(*body_handle)
                        .is_some_and(|link| link.multibody == tree)
                {
                    continue;
                }
                for h in body.colliders() {
                    let third = &self.world.colliders[*h];
                    if !collision::can_exchange_impulses(collider, third, body.body_type()) {
                        continue;
                    }
                    let pose =
                        body.position() * third.position_wrt_parent().expect("attached collider");
                    let hit = if body.is_fixed() {
                        fixed_intersections(third, &pose, &free, &reachable)
                    } else {
                        let bounds = if self.has_joint(*id) == Some(true) {
                            // External joint reactions are not bounded by the isolated-body speed cap.
                            return Ok(true);
                        } else if body.is_dynamic() {
                            deflected_aabb(body, third.shape(), &pose, cap * seconds)
                        } else {
                            let (linear, deviation) =
                                self.curves.chord(*id, dt)?.unwrap_or((body.linvel(), 0.));
                            let (v, rotation) = collider_chord(
                                third.shape(),
                                &pose,
                                body.center_of_mass(),
                                linear,
                                body.angvel(),
                                seconds,
                            );
                            swept_aabb(third.shape(), &pose, v, deviation + rotation, seconds)
                        }
                        .loosened(third.contact_skin() + body.soft_ccd_prediction());
                        (free.intersects(&bounds), reachable.intersects(&bounds))
                    };
                    if hit.0 {
                        if !body.is_fixed()
                            || planar_support
                            || !self.single_planar_support(
                                tree, *handle, *h, &pose, &reachable, seconds,
                            )
                        {
                            return Ok(true);
                        }
                        planar_support = true;
                    } else {
                        reachable_neighbor |= hit.1;
                    }
                    if planar_support && reachable_neighbor {
                        // Once supported, the payload can deviate from its free chord before
                        // hitting this additional body. Do not rely on the original free sweep.
                        return Ok(true);
                    }
                }
            }
            // Normal SDF geometry has a public rigid parent. Count attached
            // shapes during the scan so compound models incur no second pass.
            if attached_shapes < self.world.colliders.len() {
                for (_, third) in self
                    .world
                    .colliders
                    .iter()
                    .filter(|(_, c)| c.parent().is_none())
                {
                    if collision::can_exchange_impulses(collider, third, RigidBodyType::Fixed) {
                        let (free_hit, after_support) =
                            fixed_intersections(third, third.position(), &free, &reachable);
                        if free_hit || (planar_support && after_support) {
                            return Ok(true);
                        }
                    }
                }
            }
        }
        Ok(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nearby_striker_deflects_stationary_payload_into_the_promoted_arm() {
        let mut backend = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let sphere = |position, velocity| BodySpec {
            position,
            velocity,
            mass: Some(1.),
            shape: gz_physics::Shape::Sphere { radius: 0.1 },
        };
        let [root, tip] = [0., 1.].map(|y| {
            backend
                .spawn_articulation_link(
                    sphere([0., y, 0.], [0.; 3]),
                    RotationSpec::default(),
                    None,
                )
                .unwrap()
        });
        backend
            .add_limited_revolute(Some(root), tip, [0., 0.5, 0.], [0., 0., 1.], [-1., 1.])
            .unwrap();
        let payload = backend.spawn(sphere([-0.225, 1., 0.], [0.; 3])).unwrap();
        let dt = Duration::from_millis(1);
        backend
            .switch_articulation(root, ArticulationTier::Prescribed, dt)
            .unwrap();
        assert!(
            !backend
                .predicts_articulation_contact(root, payload, dt)
                .unwrap()
        );
        backend
            .spawn(sphere([-80., 1., 0.], [70., 0., 0.]))
            .unwrap();
        assert!(
            !backend
                .predicts_articulation_contact(root, payload, dt)
                .unwrap()
        );

        let striker = backend
            .spawn(sphere([-0.426, 1., 0.], [70., 0., 0.]))
            .unwrap();
        assert!(
            backend
                .predicts_articulation_contact(root, payload, dt)
                .unwrap()
        );
        backend
            .switch_articulation(root, ArticulationTier::Dynamic, dt)
            .unwrap();
        let [striker, payload, tip] = [striker, payload, tip]
            .map(|id| backend.world.bodies[backend.bodies[&id].0].colliders()[0]);
        let mut first_strike = None;
        let mut first_arm_impulse = None;
        for tick in 1..=6 {
            backend.step(dt).unwrap();
            for (a, b, first) in [
                (striker, payload, &mut first_strike),
                (payload, tip, &mut first_arm_impulse),
            ] {
                if backend
                    .world
                    .narrow_phase
                    .contact_pair(a, b)
                    .is_some_and(|pair| pair.total_impulse_magnitude() > 1e-6)
                {
                    first.get_or_insert(tick);
                }
            }
        }
        assert_eq!(first_strike, Some(1));
        assert_eq!(first_arm_impulse, Some(2));
    }

    #[test]
    fn tumbling_payload_sweep_encloses_native_corners_under_force_and_torque() {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        let extent = Vector::new(0.35, 0.12, 0.08);
        let (h, c) = world.insert(
            RigidBodyBuilder::dynamic(),
            ColliderBuilder::cuboid(extent.x, extent.y, extent.z).mass(2.),
        );
        world.step();
        world.gravity = Vector::new(0., 0., -9.81);
        let dt = 0.02;
        world.integration_parameters.dt = dt / 4.;
        let body = &mut world.bodies[h];
        body.set_linvel(Vector::new(1., -3., 0.5), true);
        body.set_angvel(Vector::new(-2., 1., -1.), true);
        body.add_force(Vector::new(1., 3., 6.), true);
        body.add_torque(Vector::new(0.02, -0.04, 0.03), true);
        let start = *world.colliders[c].position();
        let (v, margin) = free_sweep(
            body,
            world.colliders[c].shape(),
            &start,
            world.gravity,
            world.integration_parameters.max_linear_velocity(),
            dt,
        );
        assert!(
            margin < 0.12,
            "a slow spin should retain local clearance: {margin}"
        );
        let mut greatest_deviation = 0_f64;
        for i in 1..=4 {
            world.step();
            let end = *world.colliders[c].position();
            for x in [-extent.x, extent.x] {
                for y in [-extent.y, extent.y] {
                    for z in [-extent.z, extent.z] {
                        let corner = Vector::new(x, y, z);
                        let deviation = (end * corner
                            - (start * corner + v * (dt * f64::from(i) / 4.)))
                            .length();
                        assert!(
                            deviation <= margin,
                            "{deviation} exceeds {margin} at sample {i}"
                        );
                        greatest_deviation = greatest_deviation.max(deviation);
                    }
                }
            }
        }
        assert!(greatest_deviation > 0.005);
    }
}
