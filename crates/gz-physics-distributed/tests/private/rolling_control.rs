// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, PhysicsBackend, RotationSpec, Shape};
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, Primitive},
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured, reset_loaded_world};
use sdformat::NoResources;
use std::{error::Error, path::Path, time::Duration};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const RADIUS: f64 = 0.3;
const HALF_LENGTH: f64 = 0.45;

fn check_ray(
    scene: &PhysicsBackend,
    native: &MultiFidelityBackend,
    origin: [f64; 3],
    direction: [f64; 3],
    expected: f64,
) -> Result<f64> {
    let limit = expected + 0.2;
    let hit = scene
        .cast_ray(origin, direction, limit, None)?
        .expect("finite roller hit");
    let reference = native
        .cast_ray(origin, direction, limit, None)?
        .expect("native roller hit");
    assert_eq!(hit.to_bits(), reference.to_bits());
    assert!(hit > 0.);
    assert!(
        scene
            .cast_ray(origin, direction, limit, Some(BodyId::from_raw(0)))?
            .is_none()
    );
    Ok((hit - expected).abs())
}
fn sample_rays(scene: &PhysicsBackend, native: &MultiFidelityBackend, b: &Body) -> Result<f64> {
    let mut largest = 0f64;
    if b.primitive == Primitive::Sphere {
        for z in [-0.24, 0., 0.27] {
            let expected = 2. - (RADIUS * RADIUS - z * z).sqrt();
            largest = largest.max(check_ray(
                scene,
                native,
                [b.position[0], -2., b.position[2] + z],
                [0., 1., 0.],
                expected,
            )?);
        }
    } else {
        let [x, y, z, w] = b.orientation;
        let axis = [
            2. * (x * z + w * y),
            2. * (y * z - w * x),
            1. - 2. * (x * x + y * y),
        ];
        // The ray stays horizontal and perpendicular to the actual centerline.
        // Its three slices hit the cylinder and both caps, clear of the floor.
        let length = axis[0].hypot(axis[1]);
        let direction = [-axis[1] / length, axis[0] / length, 0.];
        let normal = [
            -axis[2] * direction[1],
            axis[2] * direction[0],
            axis[0] * direction[1] - axis[1] * direction[0],
        ];
        if b.primitive == Primitive::Cylinder {
            // One barrel chord and two off-axis end-cap hits. A capsule sharing
            // the cylinder's radius and centerline would give different cap TOFs.
            let offset = 0.13_f64;
            let origin = std::array::from_fn(|i| {
                b.position[i] + 0.34 * axis[i] + offset * normal[i] - direction[i]
            });
            largest = largest.max(check_ray(
                scene,
                native,
                origin,
                direction,
                1. - (RADIUS * RADIUS - offset * offset).sqrt(),
            )?);
            for sign in [-1., 1.] {
                let ray = axis.map(|v| sign * v);
                let origin = std::array::from_fn(|i| b.position[i] + 0.14 * direction[i] - ray[i]);
                largest = largest.max(check_ray(scene, native, origin, ray, 1. - HALF_LENGTH)?);
            }
            return Ok(largest);
        }
        for (height, offset) in [(0.12_f64, 0.10_f64), (0.63, 0.08), (-0.71, 0.06)] {
            let origin = std::array::from_fn(|i| {
                b.position[i] + height * axis[i] + offset * normal[i] - direction[i]
            });
            let cap = (height.abs() - HALF_LENGTH).max(0.);
            let expected = 1. - (RADIUS * RADIUS - offset * offset - cap * cap).sqrt();
            largest = largest.max(check_ray(scene, native, origin, direction, expected)?);
        }
    }
    Ok(largest)
}
pub fn qualify(xml: &str, primitive: Primitive) -> Result<()> {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let mut sim = load_world_configured(
        xml,
        "rolling.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?;
    let frame = |s: &gz_sim_core::Simulation| {
        s.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .unwrap()
            .frame()
            .unwrap()
            .clone()
    };
    let seed = frame(&sim);
    let dt = Duration::from_millis(5);
    let mut native = MultiFidelityBackend::new([2.1, 0., -9.81])?;
    native.set_partition_step(dt)?;
    native.set_partition_ground_material(0.25, 0.25)?;
    let (shape, inertia_ratio) = match primitive {
        Primitive::Sphere => (Shape::Sphere { radius: RADIUS }, 0.4),
        Primitive::Cylinder => (
            Shape::Cylinder {
                radius: RADIUS,
                half_height: HALF_LENGTH,
            },
            0.5,
        ),
        Primitive::Capsule => {
            let caps_mass = 2. * RADIUS / (3. * HALF_LENGTH + 2. * RADIUS);
            (
                Shape::Capsule {
                    radius: RADIUS,
                    half_length: HALF_LENGTH,
                },
                0.5 * (1. - caps_mass) + 0.4 * caps_mass,
            )
        }
        _ => unreachable!(),
    };
    for (i, (shape, position, mass, friction)) in [
        (shape, [-3., 0., 0.65], 1., 0.72),
        (
            Shape::Box {
                half_extents: [0.2; 3],
            },
            [3., 0., 0.50],
            2.,
            0.80,
        ),
    ]
    .into_iter()
    .enumerate()
    {
        let orientation = seed.body(i as u64).unwrap().1.orientation;
        let handle = native.spawn_partition_primitive(
            BodySpec {
                shape,
                position,
                velocity: [0.; 3],
                mass: Some(mass),
            },
            RotationSpec {
                orientation,
                ..Default::default()
            },
            false,
            friction,
        )?;
        assert_eq!(handle.raw(), i as u64);
    }
    assert_eq!(seed.body(0).unwrap().1.primitive, primitive);
    assert_ne!(seed.body(0).unwrap().0, seed.body(1).unwrap().0);
    let (mut join, mut impact) = (None, None);
    let mut samples = [[0.; 3]; 2];
    let mut supported = 0;
    let mut ray_error = 0f64;
    for tick in 1..=740 {
        sim.step_exact(1)?;
        native.step(dt)?;
        let f = frame(&sim);
        let (roller_owner, roller) = f.body(0).unwrap();
        let (block_owner, block) = f.body(1).unwrap();
        if roller_owner == block_owner && join.is_none() {
            assert!(supported > 1);
            join = Some(tick);
        }
        for b in [roller, block] {
            let n = native.state(BodyId::from_raw(b.id)).unwrap();
            assert_eq!(
                (
                    b.position.map(f64::to_bits),
                    b.orientation.map(f64::to_bits),
                    b.linear.map(f64::to_bits),
                    b.angular.map(f64::to_bits)
                ),
                (
                    n.position.map(f64::to_bits),
                    n.orientation.map(f64::to_bits),
                    n.velocity.map(f64::to_bits),
                    n.angular_velocity.map(f64::to_bits)
                ),
                "body {}, tick {tick}",
                b.id
            );
        }
        let mut ground: Vec<_> = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.supports)
            .map(|p| (p.body, p.normal_impulse.to_bits()))
            .collect();
        ground.sort();
        let expected: Vec<_> = native
            .partition_ground_contacts()
            .into_iter()
            .filter(|p| p.active)
            .map(|p| (p.body.raw(), p.normal_impulse.to_bits()))
            .collect();
        assert_eq!(ground, expected);
        supported += usize::from(ground.len() == 2);
        let pairs: Vec<_> = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(|p| (p.a, p.b, p.normal_impulse.to_bits()))
            .collect();
        let expected: Vec<_> = native
            .rigid_contact_observations()
            .into_iter()
            .filter(|p| p.active)
            .map(|p| (p.a.raw(), p.b.raw(), p.normal_impulse.to_bits()))
            .collect();
        assert_eq!(pairs, expected, "pair impulse at tick {tick}");
        if pairs.iter().any(|p| f64::from_bits(p.2) > 0.) {
            impact.get_or_insert(tick);
        }
        if matches!(tick, 200 | 400) {
            samples[tick / 200 - 1] = [roller.linear[0], roller.angular[1], block.position[0]];
        }
        if tick <= 400 {
            ray_error = ray_error.max(sample_rays(
                sim.world().resource::<PhysicsBackend>(),
                &native,
                roller,
            )?);
        }
    }
    let acceleration = samples[1][0] - samples[0][0];
    let spin = samples[1][1] - samples[0][1];
    let ideal = 2.1 / (1. + inertia_ratio);
    println!(
        "{primitive:?} and block: 740 exact frames and both native impulse streams, warm join {join:?}, impact {impact:?}, supported together {supported} ticks; linear acceleration {acceleration:.9} m/s² vs {ideal:.9} analytically, angular acceleration {spin:.9} rad/s², 1200 strictly positive native and analytical ray chords, maximum error {ray_error:.3e} m"
    );
    assert!(join.is_some() && impact.is_some() && join < impact);
    assert!((samples[1][2] - samples[0][2]).abs() < 1e-6);
    assert!((acceleration - ideal).abs() < 0.001 && (spin - ideal / RADIUS).abs() < 0.01);
    assert!(ray_error < 2e-13);
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(frame(&sim), seed);
    Ok(())
}
