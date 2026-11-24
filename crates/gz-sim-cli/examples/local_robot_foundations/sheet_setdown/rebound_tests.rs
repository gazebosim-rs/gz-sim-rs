// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn point_seed() -> Seed {
    let bench = Boundary::TranslatingCuboid {
        center: [0., 0., -0.01],
        half_extents: [0.1, 0.1, 0.01],
        orientation: [0., 0., 0., 1.],
        velocity: [0.; 3],
    };
    let config = gz_physics_xpbd::SolverConfig {
        substeps: 1,
        particle_radius: arm_sheet::RADIUS,
        ..Default::default()
    };
    let material = SoftBody::new(
        vec![Particle {
            position: [0., 0., arm_sheet::RADIUS + 0.00045],
            velocity: [0., 0., -0.99],
            inverse_mass: 9.81 / WEIGHT,
        }],
        vec![],
        config,
    )
    .unwrap();
    Seed {
        material,
        bench,
        top: 0.,
        time_ns: 0,
    }
}

#[test]
fn a_single_inelastic_particle_cannot_pass_for_a_rebounding_sheet() {
    let s = point_seed();
    for rate in [16, 64, 4000] {
        let m = point_control(&s, rate).unwrap();
        assert!(m["first_impulse_time_ns"].as_u64().unwrap() <= 500_000);
        assert!(m["rebound_m"].as_f64().unwrap() < 1e-15);
    }
    assert!(trial(&s, Clock::uniform(16)).is_err());
}

#[test]
fn the_seed_requires_a_stopped_receiver_and_supported_incoming_clock() {
    let d = tempfile::tempdir().unwrap();
    assert!(run(d.path(), 0).is_err());
    let mut r = Recording::new(&world(16, true, false).unwrap(), "rebound.sdf", "default").unwrap();
    let bench = entity_id(&r.sim, "default::source::body").unwrap();
    gz_sim_scene::queue_link_world_velocity(&mut r.sim, bench, [0., 0., -0.5], [0.; 3]).unwrap();
    r.advance().unwrap();
    assert!(seed(&r).is_err());
}
