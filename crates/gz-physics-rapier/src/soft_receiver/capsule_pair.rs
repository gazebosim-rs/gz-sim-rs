// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use rapier3d_f64::parry::{query, shape::Capsule};

fn conflict() -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability: "two finite soft capsule receivers require exclusive free centered capsules and separated native contact paths",
    }
}

impl RapierBackend {
    /// Exactly two distinct enrolled capsules may exchange soft impulses if
    /// their native bodies remain separated throughout the free and corrected paths.
    pub fn soft_capsule_pair_inertia(
        &self,
        ids: [BodyId; 2],
    ) -> Result<[SoftRigidInertia; 2], PhysicsError> {
        if ids[0] == ids[1] || self.world.colliders.len() != 2 {
            return Err(conflict());
        }
        Ok([
            self.receiver_inertia(ids[0], ReceiverShape::Capsule, true)?,
            self.receiver_inertia(ids[1], ReceiverShape::Capsule, true)?,
        ])
    }
    /// Each tuple contains native identity, start of public tick, and fully
    /// coupled end state. State validation and pair exclusion precede all writes.
    pub fn commit_soft_capsule_pair(
        &mut self,
        updates: [(BodyId, BodyState, BodyState); 2],
    ) -> Result<(), PhysicsError> {
        self.soft_capsule_pair_inertia(updates.map(|(id, _, _)| id))?;
        let prepare = |(id, start, end)| {
            let (h, a) = body_state::prepare(self, id, start)?;
            let (_, b) = body_state::prepare(self, id, end)?;
            let body = &self.world.bodies[h];
            let c = body.colliders()[0];
            if self
                .world
                .narrow_phase
                .contact_pairs_with(c)
                .flat_map(|p| p.solver_manifolds())
                .flat_map(|m| m.points.iter())
                .any(|p| p.data.impulse != 0.)
            {
                return Err(conflict());
            }
            Ok((
                a,
                *body.position(),
                b,
                *self.world.colliders[c]
                    .shape()
                    .as_capsule()
                    .expect("admitted capsule"),
            ))
        };
        let (a0, an, a1, ca) = prepare(updates[0])?;
        let (b0, bn, b1, cb) = prepare(updates[1])?;
        if !separated(&ca, &cb, [a0, an], [b0, bn])?
            || !separated(&ca, &cb, [an, a1], [bn, b1])?
            || !separated(&ca, &cb, [a0, a1], [b0, b1])?
        {
            return Err(conflict());
        }
        for (id, _, end) in updates {
            self.set_body_state(id, end)?;
        }
        Ok(())
    }
}

/// A midpoint distance minus half the maximum relative point travel bounds
/// the separation on the entire interval. Axial spin leaves a capsule fixed.
/// Return false on exhausted resolution as well as proven contact.
fn separated(a: &Capsule, b: &Capsule, pa: [Pose; 2], pb: [Pose; 2]) -> Result<bool, PhysicsError> {
    let rotation = |p: [Pose; 2]| {
        let mut q = (p[1].rotation * p[0].rotation.inverse()).normalize();
        if q.w < 0. {
            q = -q;
        }
        q.to_scaled_axis()
    };
    let ra = rotation(pa);
    let rb = rotation(pb);
    let linear = (pa[1].translation - pa[0].translation) - (pb[1].translation - pb[0].translation);
    let speed = linear.length()
        + ra.cross(pa[0].rotation * a.segment.a).length()
        + rb.cross(pb[0].rotation * b.segment.a).length();
    let scale = pa
        .iter()
        .chain(pb.iter())
        .map(|p| p.translation.abs().max_element())
        .fold(a.radius.max(b.radius), f64::max);
    let tol = 128. * f64::EPSILON * scale;
    let at = |p: [Pose; 2], r: Vector, t: f64| {
        Pose::from_parts(
            p[0].translation.lerp(p[1].translation, t),
            Rotation::from_scaled_axis(t * r) * p[0].rotation,
        )
    };
    let mut stack = vec![(0., 1., 0u32)];
    let mut queries = 0;
    while let Some((l, h, depth)) = stack.pop() {
        let t = 0.5 * (l + h);
        let d = query::distance(&at(pa, ra, t), a, &at(pb, rb, t), b)
            .map_err(|_| conflict())?
            .distance;
        if !d.is_finite() || d <= tol {
            return Ok(false);
        }
        if d > 0.5 * (h - l) * speed + tol {
            continue;
        }
        queries += 1;
        if depth == 28 || queries == 4096 {
            return Ok(false);
        }
        stack.extend([(t, h, depth + 1), (l, t, depth + 1)]);
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn pair() -> (RapierBackend, [BodyId; 2]) {
        let mut r = RapierBackend::new([0.; 3]).unwrap();
        let ids = [-0.35, 0.35].map(|x| {
            r.spawn(BodySpec {
                shape: Shape::Capsule {
                    radius: 0.05,
                    half_length: 0.4,
                },
                position: [x, 0., 0.],
                velocity: [0.; 3],
                mass: Some(0.2),
            })
            .unwrap()
        });
        (r, ids)
    }
    #[test]
    fn valid_pair_commits_atomically_and_invalid_or_crossing_second_correction_keeps_both_states() {
        let (mut r, ids) = pair();
        assert!(r.soft_capsule_inertia(ids[0]).is_err());
        assert!(r.soft_capsule_pair_inertia([ids[0]; 2]).is_err());
        let inertia = r.soft_capsule_pair_inertia(ids).unwrap();
        assert!((inertia[0].inverse_mass - 5.).abs() < 1e-12);
        let start = ids.map(|id| r.state(id).unwrap());
        let mut end = start;
        end[0].position[2] += 0.02;
        end[1].velocity[0] = 0.3;
        let apply = |r: &mut RapierBackend, start: [BodyState; 2], end: [BodyState; 2]| {
            r.commit_soft_capsule_pair([(ids[0], start[0], end[0]), (ids[1], start[1], end[1])])
        };
        apply(&mut r, start, end).unwrap();
        assert_eq!(ids.map(|id| r.state(id).unwrap()), end);
        let mut bad = end;
        bad[0].velocity[0] += 1.;
        bad[1].orientation = [f64::NAN; 4];
        assert!(apply(&mut r, end, bad).is_err());
        assert_eq!(ids.map(|id| r.state(id).unwrap()), end);
        let mut cross = end;
        cross[0].position[0] = 0.36;
        cross[1].position[0] = -0.36;
        assert!(apply(&mut r, end, cross).is_err());
        assert_eq!(ids.map(|id| r.state(id).unwrap()), end);
        // Bodies end where they began but the proposed public start traverses the pair.
        assert!(apply(&mut r, cross, end).is_err());
        r.spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.01 },
            position: [4., 0., 0.],
            velocity: [0.; 3],
            mass: None,
        })
        .unwrap();
        assert!(r.soft_capsule_pair_inertia(ids).is_err());
    }
    #[test]
    fn rotating_capsule_between_clear_endpoints_is_excluded() {
        let a = Capsule::new_z(0.8, 0.05);
        let b = Capsule::new_z(0.03, 0.025);
        let p = |x, y, z, angle| {
            Pose::from_parts(Vector::new(x, y, z), Rotation::from_rotation_y(angle))
        };
        let obstacle = p(0.725, 0., 0., 0.);
        let ends = [p(0., 0., 0., 0.45), p(0., 0., 0., 2.70)];
        for pose in ends {
            assert!(query::distance(&pose, &a, &obstacle, &b).unwrap().distance > 0.45);
        }
        assert!(!separated(&a, &b, ends, [obstacle; 2]).unwrap());
        let near = p(0.725, 0.076, 0., 0.);
        assert!(separated(&a, &b, ends, [near; 2]).unwrap());
    }
    #[test]
    fn curved_pair_clearance_matches_independent_nonlinear_casts() {
        let mut state = 97u64;
        let mut rand = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            2. * ((state >> 11) as f64 / (1u64 << 53) as f64) - 1.
        };
        let a = Capsule::new_z(0.30, 0.07);
        let b = Capsule::new_z(0.22, 0.045);
        let mut hits = 0;
        let mut misses = 0;
        for _ in 0..600 {
            let mut trajectory = || {
                let p = Pose::from_parts(
                    Vector::new(rand(), rand(), rand()) * 0.35,
                    Rotation::from_scaled_axis(Vector::new(rand(), rand(), rand())),
                );
                let v = Vector::new(rand(), rand(), rand()) * 0.6;
                let w = Vector::new(rand(), rand(), rand()) * 1.4;
                let motion = query::NonlinearRigidMotion::new(p, Vector::ZERO, v, w);
                (
                    [
                        p,
                        Pose::from_parts(
                            p.translation + v,
                            Rotation::from_scaled_axis(w) * p.rotation,
                        ),
                    ],
                    motion,
                )
            };
            let (pa, ma) = trajectory();
            let (pb, mb) = trajectory();
            let reference = query::cast_shapes_nonlinear(&ma, &a, &mb, &b, 0., 1., true)
                .unwrap()
                .is_some();
            assert_eq!(!separated(&a, &b, pa, pb).unwrap(), reference);
            if reference {
                hits += 1
            } else {
                misses += 1;
            }
        }
        assert!(hits > 50 && misses > 50);
        println!(
            "600 independent nonlinear capsule-pair shape casts: {hits} contacts and {misses} clear paths"
        );
    }
}
