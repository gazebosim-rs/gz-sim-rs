// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/pendulum.rs"]
mod pendulum;

use gz_sim_scene::reset_loaded_world;
use pendulum::{Assembly, Result, STOCK, load, with_lower_inertial};
use std::time::Duration;

#[test]
fn stock_links_release_under_gravity_and_repeat_after_reset() -> Result<()> {
    for stiff in [false, true] {
        let mut sim = load(STOCK, stiff)?;
        let mut first = None;
        for epoch in 0..=1 {
            let assemblies = [Assembly::new(&mut sim, 0)?, Assembly::new(&mut sim, 1)?];
            assert_eq!(sim.epoch(), epoch);
            assert_eq!(sim.elapsed(), Duration::ZERO);
            let mut history = vec![];
            for tick in 0..=600 {
                if tick > 0 {
                    sim.step_exact(10)?;
                }
                for assembly in &assemblies {
                    let s = assembly.sample(&sim, 1., 1.)?;
                    assert_eq!(s.tick, tick * 10);
                    assert!(s.anchor_error_m < 0.005);
                    if tick == 0 {
                        assert!(s.upper_q.abs() < 1e-12 && s.lower_q.abs() < 1e-12);
                        assert_eq!(s.kinetic_j, 0.);
                    }
                    if tick == 50 {
                        assert!(s.upper_q < -0.2 && s.kinetic_j > 1.);
                        assert!(s.lower_q.abs() > 0.01);
                    }
                    history.push(s);
                }
            }
            if let Some(previous) = &first {
                assert_eq!(&history, previous);
            } else {
                first = Some(history);
                reset_loaded_world(&mut sim, |_| ())?;
            }
        }
    }
    Ok(())
}

#[test]
fn invalid_lower_link_inertia_fails_during_load() -> Result<()> {
    for invalid in [
        "<mass>-1</mass>",
        "<inertia><ixx>-1</ixx><iyy>1</iyy><izz>1</izz></inertia>",
        "<inertia><ixx>1</ixx><iyy>1</iyy><izz>3</izz></inertia>",
        "<inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>2</ixy></inertia>",
    ] {
        assert!(
            load(&with_lower_inertial(invalid)?, false).is_err(),
            "accepted {invalid}"
        );
    }
    Ok(())
}

#[test]
fn kinetic_energy_matches_motion_of_centers_and_link_orientations() -> Result<()> {
    use glam::{DQuat, DVec3};
    use gz_sim_core::Simulation;
    use gz_sim_scene::{Name, WorldPose};

    let mut sim = load(STOCK, false)?;
    let assembly = Assembly::new(&mut sim, 0)?;
    let world = sim.world_mut();
    let mut names = world.query::<(bevy_ecs::entity::Entity, &Name)>();
    let links = ["base", "upper_link", "lower_link"].map(|name| {
        let scope = format!("default::double_pendulum_with_base0::{name}");
        names.iter(world).find(|(_, n)| n.0 == scope).unwrap().0
    });
    let poses = |sim: &Simulation| links.map(|e| *sim.world().get::<WorldPose>(e).unwrap());
    sim.step_exact(399)?;
    let before = poses(&sim);
    sim.step_exact(1)?;
    let measured = assembly.sample(&sim, 1., 1.)?;
    sim.step_exact(1)?;
    let after = poses(&sim);
    let span = 2. * sim.step_duration().as_secs_f64();
    let mut kinetic = 0.;
    for (i, (a, b)) in before.iter().zip(after).enumerate() {
        let orientation = |p: &WorldPose| DQuat::from_array(p.rotation);
        let offset = [0., 0.5, 0.5][i] * DVec3::Z;
        let center = |p: &WorldPose| DVec3::from_array(p.translation) + orientation(p) * offset;
        let speed = center(&b).distance(center(a)) / span;
        let angular_speed = orientation(&b).angle_between(orientation(a)) / span;
        kinetic += 0.5 * ([100., 1., 1.][i] * speed * speed + angular_speed * angular_speed);
    }
    assert!(kinetic > 3.);
    assert!(
        (kinetic - measured.kinetic_j).abs() < 0.015,
        "finite difference {kinetic} J vs public velocities {} J",
        measured.kinetic_j
    );
    Ok(())
}

/// Sample the independent DART trace at the observed peak angle and energy differences.
/// The full 10 ms comparison for both assemblies is retained in pendulum-reference.tar.xz.
#[test]
fn stiff_support_stays_within_the_native_pendulum_budgets() -> Result<()> {
    let mut sim = load(STOCK, true)?;
    let arms = [Assembly::new(&mut sim, 0)?, Assembly::new(&mut sim, 1)?];
    let initial_energy = arms[0].sample(&sim, 1., 1.)?.total_j;
    // Individual pose snapshots give joint orientation modulo a full turn.
    let orientation_error = |a: f64, b: f64| (a - b).sin().atan2((a - b).cos()).abs();
    let mut previous = 0;
    for (tick, native) in [
        (
            870,
            [
                (-1.967114871, 1.314108985, 39.121630736),
                (-1.967119772, 1.314118211, 39.126732224),
            ],
        ),
        (
            3590,
            [
                (-1.672653189, -5.295374569, 39.134117049),
                (-1.672648757, -5.295435133, 39.135754666),
            ],
        ),
        (
            5380,
            [
                (-1.058897369, -12.201491541, 39.136643967),
                (-1.058893712, -12.201448253, 39.136181461),
            ],
        ),
        (
            5940,
            [
                (-0.864244399, -14.665177527, 39.120639922),
                (-0.864236679, -14.665117993, 39.125297461),
            ],
        ),
    ] {
        sim.step_exact(tick - previous)?;
        previous = tick;
        for (assembly, (upper, lower, energy)) in arms.iter().zip(native) {
            let state = assembly.sample(&sim, 1., 1.)?;
            assert!(
                orientation_error(state.upper_q, upper) < 0.06,
                "upper angle at tick {tick}"
            );
            assert!(
                orientation_error(state.lower_q, lower) < 0.06,
                "lower angle at tick {tick}"
            );
            assert!(
                (state.total_j - energy).abs() < 0.02 * initial_energy,
                "energy at tick {tick}"
            );
        }
    }
    Ok(())
}
