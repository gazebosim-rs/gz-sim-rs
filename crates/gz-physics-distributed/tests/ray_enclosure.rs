// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Positive, missed, clipped and excluded rays before and after normal worker startup.
use gz_physics::{BodySpec, DynamicBackend, RotationSpec, Shape};
use gz_physics_distributed::{SceneBackend, physics::Tier};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::{error::Error, path::Path, time::Duration};

#[test]
fn mixed_shape_rays_match_an_exhaustive_native_world_at_both_scene_epochs()
-> Result<(), Box<dyn Error>> {
    let step = Duration::from_millis(5);
    let mut world = SceneBackend::new(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        [0.; 3],
        step,
    )?
    .with_worker_count(4)?;
    let mut native = MultiFidelityBackend::new([0.; 3])?;
    let mut bodies = vec![];
    for i in 0..16 {
        let shape = match i % 4 {
            0 => Shape::Box {
                half_extents: [0.24, 0.7, 0.045],
            },
            1 => Shape::Sphere { radius: 0.35 },
            2 => Shape::Capsule {
                radius: 0.17,
                half_length: 0.72,
            },
            _ => Shape::Cylinder {
                radius: 0.49,
                half_height: 0.035,
            },
        };
        let offset = if i < 8 { 0. } else { 1.0e7 };
        let position = [
            offset + 3.4 * (i % 4) as f64,
            -offset + 4.8 * (i / 4) as f64,
            1.2,
        ];
        let spec = BodySpec {
            shape,
            position,
            velocity: [0.; 3],
            mass: Some(1.),
        };
        let angle = i as f64 * 0.14;
        let rotation = RotationSpec {
            orientation: [0., angle.sin(), 0., angle.cos()],
            ..Default::default()
        };
        let id = world.spawn_primitive_link(spec, rotation, None, Tier::Dynamic, 0.)?;
        assert_eq!(
            id,
            native.spawn_partition_primitive(spec, rotation, false, 0.)?
        );
        bodies.push((id, position));
    }
    for epoch in 0..2 {
        if epoch == 1 {
            world.initialize()?;
            world.step(step)?;
            native.step(step)?;
        }
        let (mut positive, mut misses, mut zero) = (0, 0, 0);
        for (id, center) in &bodies {
            for i in 1..=64 {
                let u = i as f64;
                let axis = [(u * 0.27).cos(), (u * 0.431).sin(), (u * 0.137).cos()];
                let length = axis[0].hypot(axis[1]).hypot(axis[2]);
                let d = axis.map(|v| v / length);
                for shift in [0., 0.48, 1.4] {
                    let origin = [
                        center[0] - d[0] * 2. + shift,
                        center[1] - d[1] * 2.,
                        center[2] - d[2] * 2.,
                    ];
                    let hit = native.cast_ray(origin, d, 6., None)?;
                    let t = hit.unwrap_or(2.);
                    for (limit, exclude) in [
                        (6., None),
                        (t.next_down().max(f64::MIN_POSITIVE), None),
                        (t.max(f64::MIN_POSITIVE), None),
                        (6., Some(*id)),
                    ] {
                        let r = native.cast_ray(origin, d, limit, exclude)?;
                        assert_eq!(
                            world.cast_ray(origin, d, limit, exclude)?.map(f64::to_bits),
                            r.map(f64::to_bits)
                        );
                        if r.is_some() {
                            positive += 1
                        } else {
                            misses += 1
                        }
                    }
                }
                let inside = world.cast_ray(*center, d, 0.005, None)?;
                assert_eq!(inside, Some(0.));
                zero += 1;
            }
        }
        assert!(positive > 2000 && misses > 2000);
        println!(
            "epoch {epoch}: {positive} positive, {misses} missed or excluded native rays, {zero} starts inside; world origins span 10,000 km"
        );
    }
    Ok(())
}
