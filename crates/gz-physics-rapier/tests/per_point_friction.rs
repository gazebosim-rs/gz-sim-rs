// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, BodyState, DynamicBackend, Shape};
use gz_physics_rapier::RapierBackend;
use rapier3d_f64::prelude::*;
use std::time::Duration;
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const G: [f64; 3] = [0., 0., -9.81];
struct Native {
    world: PhysicsWorld,
    ids: [RigidBodyHandle; 2],
}
impl Native {
    fn new(point: bool) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::from(G);
        world.integration_parameters.contact_recycling = false;
        if point {
            world.integration_parameters.friction_model = FrictionModel::Coulomb;
        }
        let ids = [0, 1].map(|i| {
            let (size, z, mass, mu) = shapes(i);
            let body = if i == 0 {
                RigidBodyBuilder::kinematic_velocity_based()
            } else {
                RigidBodyBuilder::dynamic()
            };
            let [x, y, z_half] = size;
            let (h, _) = world.insert(
                body.can_sleep(false)
                    .translation(Vector::new(0., 0., z))
                    .linvel(Vector::X),
                ColliderBuilder::cuboid(x, y, z_half)
                    .mass(mass)
                    .friction(mu)
                    .friction_combine_rule(CoefficientCombineRule::Min),
            );
            world.bodies[h].recompute_mass_properties_from_colliders(&world.colliders);
            h
        });
        Self { world, ids }
    }
    fn state(&self, i: usize) -> BodyState {
        let b = &self.world.bodies[self.ids[i]];
        BodyState {
            position: b.translation().to_array(),
            orientation: b.rotation().to_array(),
            velocity: b.linvel().to_array(),
            angular_velocity: b.angvel().to_array(),
        }
    }
}
fn shapes(i: usize) -> ([f64; 3], f64, f64, f64) {
    if i == 0 {
        ([4., 1.5, 0.2], -0.2, 10., 0.65)
    } else {
        ([0.4, 0.2, 0.1], 0.15, 2., 0.35)
    }
}
fn configured(point: bool) -> Result<(RapierBackend, [BodyId; 2])> {
    let mut engine = if point {
        RapierBackend::new_per_point_coulomb(G)?
    } else {
        RapierBackend::new(G)?
    };
    let spawn = |engine: &mut RapierBackend, i| {
        let (size, z, m, mu) = shapes(i);
        engine.spawn_partition_box(
            BodySpec {
                position: [0., 0., z],
                velocity: [1., 0., 0.],
                mass: Some(m),
                shape: Shape::Box { half_extents: size },
            },
            Default::default(),
            i == 0,
            mu,
        )
    };
    let ids = [spawn(&mut engine, 0)?, spawn(&mut engine, 1)?];
    Ok((engine, ids))
}
fn bits(s: BodyState) -> Vec<u64> {
    s.position
        .into_iter()
        .chain(s.orientation)
        .chain(s.velocity)
        .chain(s.angular_velocity)
        .map(f64::to_bits)
        .collect()
}
#[test]
fn moving_table_combined_slide_and_yaw_matches_the_independent_solver_every_substep() -> Result<()>
{
    for point in [false, true] {
        let (mut backend, ids) = configured(point)?;
        let mut native = Native::new(point);
        let initial = backend.initial_state();
        assert_eq!(backend.per_point_friction_enabled(), point);
        let mut steps = 0;
        for tick in 1..=120 {
            if tick == 31 {
                backend.apply_impulse(ids[1], [4.8, 0., 0.], [0., 0., 0.96])?;
                let b = &mut native.world.bodies[native.ids[1]];
                b.apply_impulse(Vector::new(4.8, 0., 0.), true);
                b.apply_torque_impulse(Vector::new(0., 0., 0.96), true);
            }
            let count = if tick <= 8 { 1 } else { 10 };
            let dt = Duration::from_millis(10 / count);
            for _ in 0..count {
                backend.step(dt)?;
                native.world.integration_parameters.dt = dt.as_secs_f64();
                native.world.step();
                for (i, id) in ids.into_iter().enumerate() {
                    assert_eq!(
                        bits(backend.state(id).unwrap()),
                        bits(native.state(i)),
                        "mode={point}, native substep={steps}, body={i}"
                    );
                }
                steps += 1;
            }
        }
        assert_eq!(steps, 1128);
        let b = backend.state(ids[1]).unwrap();
        // Final frames independently captured by the coupled point-pressure study.
        let expected: [f64; 3] = if point {
            [2.3299118191315, 1.0246990599418284, 0.07224129921781511]
        } else {
            [2.04149377983829, 0.9999969488096963, 2.1746592222069687e-14]
        };
        assert_eq!(
            [b.position[0], b.velocity[0], b.angular_velocity[2]].map(f64::to_bits),
            expected.map(f64::to_bits)
        );
        backend.reset()?;
        assert_eq!(backend.per_point_friction_enabled(), point);
        backend.apply_initial(backend.prepare_initial(&initial)?)?;
        assert_eq!(backend.per_point_friction_enabled(), point);
    }
    Ok(())
}
#[cfg(feature = "partition-checkpoint")]
#[test]
fn partitions_and_native_snapshots_refuse_the_standalone_per_point_solver() -> Result<()> {
    use gz_physics_rapier::NativeSolverProfile;
    let (point, _) = configured(true)?;
    let (grouped, _) = configured(false)?;
    for profile in [
        NativeSolverProfile::Standard32,
        NativeSolverProfile::Extended64,
    ] {
        assert!(point.checkpoint_solver_with_profile(profile).is_err());
        assert!(
            point
                .join_box_solvers_with_profile(&[&grouped], profile)
                .is_err()
        );
        assert!(
            grouped
                .join_box_solvers_with_profile(&[&point], profile)
                .is_err()
        );
    }
    assert!(point.checkpoint_floating_revolute().is_err());
    Ok(())
}
