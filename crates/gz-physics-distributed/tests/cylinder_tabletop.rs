// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, PhysicsBackend, RotationSpec, Shape};
use gz_physics_distributed::{
    SceneBackend,
    coordinator::Frame,
    physics::{Primitive, Tier},
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured, reset_loaded_world};
use sdformat::NoResources;
use std::{error::Error, path::Path, time::Duration};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/distributed_cylinder_tabletop.sdf");
const TOP: f64 = 1.;
const R: f64 = 0.75;
const BALL: f64 = 0.12;
const MASS: f64 = 0.6;
const G: [f64; 3] = [1.25, 0., -9.81];

fn frame(s: &Simulation) -> &Frame {
    s.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
        .frame()
        .unwrap()
}

fn static_rays(s: &PhysicsBackend, native: &MultiFidelityBackend, ball: BodyId) -> Result<f64> {
    let mut error = 0.0f64;
    let mut ray = |start, dir, limit, expected: Option<f64>| -> Result<()> {
        let a = s.cast_ray(start, dir, limit, Some(ball))?;
        let b = native.cast_ray(start, dir, limit, Some(ball))?;
        assert_eq!(
            a.map(f64::to_bits),
            b.map(f64::to_bits),
            "ray {start:?} {dir:?}"
        );
        match (a, expected) {
            (Some(a), Some(b)) => {
                error = error.max((a - b).abs());
                assert!((a - b).abs() < 4. * f64::EPSILON, "TOF {a} vs {b}");
            }
            _ => assert!(a.is_none() && expected.is_none()),
        }
        Ok(())
    };
    // All three downward rays see the same top. A spherical cap sharing
    // this apex and radius falls over 0.50 m at the outer radial sample.
    for x in [0., 0.45, 0.749] {
        ray([x, 0., 2.], [0., 0., -1.], 1.5, Some(1.))?;
    }
    ray([0.751, 0., 2.], [0., 0., -1.], 1.5, None)?;
    // A true finite cylindrical barrel has fixed radius all the way up to
    // the cap. Above the cap the same ray misses, and the underside is flat.
    ray(
        [2., 0.45, 0.999],
        [-1., 0., 0.],
        2.,
        Some(2. - (R * R - 0.45f64.powi(2)).sqrt()),
    )?;
    ray([2., 0.45, 1.001], [-1., 0., 0.], 2., None)?;
    ray([0.51, 0., 0.20], [0., 0., 1.], 1., Some(0.20))?;
    Ok(error)
}

#[test]
fn finite_cylinder_supports_a_sliding_sphere_then_releases_it_at_the_rim() -> Result<()> {
    for millis in [1, 2, 5] {
        qualify(millis)?;
    }
    Ok(())
}

fn qualify(millis: u64) -> Result<()> {
    let dt = Duration::from_millis(millis);
    let h = dt.as_secs_f64();
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let mut s = load_world_configured(
        XML,
        "tabletop.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: Some(dt),
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?;

    let mut native = MultiFidelityBackend::new(G)?;
    native.set_partition_step(dt)?;
    native.set_partition_ground_material(0., 0.)?;
    let table = native.spawn_partition_primitive(
        BodySpec {
            shape: Shape::Cylinder {
                radius: R,
                half_height: 0.30,
            },
            position: [0., 0., 0.70],
            velocity: [0.; 3],
            mass: None,
        },
        RotationSpec::default(),
        false,
        0.,
    )?;
    let sphere = native.spawn_partition_primitive(
        BodySpec {
            shape: Shape::Sphere { radius: BALL },
            position: [-0.55, 0.12, 1.42],
            velocity: [0.; 3],
            mass: Some(MASS),
        },
        RotationSpec::default(),
        false,
        0.,
    )?;
    let (t, b) = (table.raw(), sphere.raw());
    let initial = frame(&s).clone();
    assert_eq!(initial.body(t).unwrap().1.primitive, Primitive::Cylinder);
    assert_eq!(initial.body(t).unwrap().1.tier, Tier::Fixed);
    assert_eq!(initial.body(b).unwrap().1.primitive, Primitive::Sphere);
    let mut ray_error = static_rays(s.world().resource::<PhysicsBackend>(), &native, sphere)?;
    s.set_paused(true);
    s.tick()?;
    assert_eq!(frame(&s), &initial);
    s.set_paused(false);
    let mut first = None;
    let mut last = None;
    let mut landing = None;
    let mut carry = 0;
    let mut peak = 0f64;
    let mut impact_sum = 0.;
    let mut before_impact = 0.;
    let mut after_impact = 0.;
    let mut impact_ticks = 0;
    let mut previous_vz = 0.;
    let mut mean = 0.;
    let mut total = 0;
    let mut min_height = f64::INFINITY;
    let mut max_height = f64::NEG_INFINITY;
    let mut ballistic_error = 0f64;
    let mut quadrature_error = 0.0f64;
    let mut horizontal_error = 0.0f64;
    let mut owner_join = None;
    let separate = initial.body(b).unwrap().0 != initial.body(t).unwrap().0;
    let mut checkpoint = None;
    let mut last_state = None;
    for tick in 1..=2100 / millis {
        s.step_exact(1)?;
        native.step(dt)?;
        let f = frame(&s);
        if f.body(t).unwrap().0 == f.body(b).unwrap().0 {
            owner_join.get_or_insert(tick);
        }
        for id in [table, sphere] {
            let actual = f.body(id.raw()).unwrap().1;
            let want = native.state(id).unwrap();
            assert_eq!(
                (
                    actual.position.map(f64::to_bits),
                    actual.orientation.map(f64::to_bits),
                    actual.linear.map(f64::to_bits),
                    actual.angular.map(f64::to_bits)
                ),
                (
                    want.position.map(f64::to_bits),
                    want.orientation.map(f64::to_bits),
                    want.velocity.map(f64::to_bits),
                    want.angular_velocity.map(f64::to_bits)
                ),
                "state at tick {tick}, body {}",
                id.raw()
            );
        }
        let actual: Vec<_> = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(|q| (q.a, q.b, q.normal_impulse.to_bits()))
            .collect();
        let expect: Vec<_> = native
            .rigid_contact_observations()
            .iter()
            .filter(|q| q.active)
            .map(|q| (q.a.raw(), q.b.raw(), q.normal_impulse.to_bits()))
            .collect();
        assert_eq!(actual, expect, "native finite contact at tick {tick}");
        let ground: Vec<_> = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.supports)
            .map(|q| (q.body, q.normal_impulse.to_bits()))
            .collect();
        let support: Vec<_> = native
            .partition_ground_contacts()
            .iter()
            .filter(|q| q.active)
            .map(|q| (q.body.raw(), q.normal_impulse.to_bits()))
            .collect();
        assert_eq!(ground, support, "native shared ground at {tick}");
        let ball = f.body(b).unwrap().1;
        let x = ball.position[0];
        let z = ball.position[2];
        let j = actual.first().map_or(0., |q| f64::from_bits(q.2));
        if j > 0. {
            if first.is_none() {
                first = Some(tick);
                before_impact = previous_vz;
            }
            last = Some(tick);
            peak = peak.max(j);
            last_state = Some((x, z, ball.linear[2]));
        }
        if first.is_some_and(|i| tick - i < 20 / millis) {
            impact_sum += j;
            impact_ticks += 1;
            after_impact = ball.linear[2];
        }
        previous_vz = ball.linear[2];
        if (0.15..0.50).contains(&x) {
            let elapsed = tick as f64 * h;
            let analytic_x = -0.55 + 0.5 * G[0] * elapsed * elapsed + G[0] * elapsed * h / 8.;
            horizontal_error = horizontal_error.max((x - analytic_x).abs());
            total += 1;
            carry += usize::from(j > 0.);
            mean += j / dt.as_secs_f64();
            min_height = min_height.min(z);
            max_height = max_height.max(z);
        }
        if let Some(t0) = last
            && tick > t0
            && ground.is_empty()
        {
            let (_, z0, v0) = last_state.unwrap();
            let elapsed = (tick - t0) as f64 * dt.as_secs_f64();
            let prediction = z0 + v0 * elapsed + 0.5 * G[2] * elapsed * elapsed;
            ballistic_error = ballistic_error.max((z - prediction).abs());
            // Four native semi-implicit substeps add g*t*h/8 to the
            // continuous ballistic position while retaining its exact speed.
            quadrature_error =
                quadrature_error.max((z - (prediction + G[2] * elapsed * h / 8.)).abs());
        }
        if ground.iter().any(|(_, j)| f64::from_bits(*j) > 0.) {
            landing.get_or_insert(tick);
        }
        if tick == 700 / millis {
            checkpoint = Some(f.clone());
        }
        if tick == 1200 / millis || tick == 1700 / millis {
            ray_error = ray_error.max(static_rays(
                s.world().resource::<PhysicsBackend>(),
                &native,
                sphere,
            )?);
        }
    }
    let (first, last, landing) = (first.unwrap(), last.unwrap(), landing.unwrap());
    let release = last_state.unwrap();
    let crossing = ((R * R - 0.12f64.powi(2)).sqrt() + 0.55)
        .mul_add(2. / G[0], 0.)
        .sqrt();
    let fall = (2. * (1.42 - TOP - BALL) / -G[2]).sqrt();
    let floor_time =
        (release.2 + (release.2.powi(2) + 2. * (-G[2]) * (release.1 - BALL)).sqrt()) / (-G[2]);
    let excess = impact_sum - MASS * (-G[2]) * h * impact_ticks as f64;
    println!(
        "{millis} ms: first {first} last {last} landing {landing}, seeded separate {separate}, owner union {owner_join:?}; release x/z/vz {release:?}; height [{min_height:.9},{max_height:.9}], {carry}/{total} plateau impulses, mean {:.9} N vs weight {:.6} N; peak single-tick {peak:.8} N s; first 20 ms total {impact_sum:.9} N s, excess over weight {excess:.9} N s; continuous ballistic error {ballistic_error:e} m, native quadrature error {quadrature_error:e} m; plateau x error {horizontal_error:e} m; analytic-ray error {ray_error:e} m",
        mean / total as f64,
        MASS * 9.81
    );
    assert_eq!(carry, total);
    assert!(total > 25 && (mean / total as f64 - MASS * 9.81).abs() < 0.01);
    assert!(min_height > TOP + BALL - 0.001 && max_height < TOP + BALL + 0.001);
    assert!((first as f64 * h - fall).abs() < 2. * h);
    assert!((last as f64 * h - crossing).abs() < 3. * h);
    assert!(((landing - last) as f64 * h - floor_time).abs() < 2. * h);
    assert!((excess - MASS * (after_impact - before_impact)).abs() < 1e-9);
    assert!(
        after_impact.abs() < 0.01
            && (excess - MASS * (-G[2]) * fall).abs() < 2. * MASS * (-G[2]) * h
    );
    assert!(first < last && last < landing && quadrature_error < 1e-8 && horizontal_error < 1e-8);
    assert!(separate && owner_join.is_some_and(|t| t < first));
    reset_loaded_world(&mut s, |_| ())?;
    assert_eq!(frame(&s).body(b).unwrap().1, initial.body(b).unwrap().1);
    s.step_exact(700 / millis)?;
    for id in [t, b] {
        assert_eq!(
            frame(&s).body(id).unwrap().1,
            checkpoint.as_ref().unwrap().body(id).unwrap().1
        );
    }
    Ok(())
}
