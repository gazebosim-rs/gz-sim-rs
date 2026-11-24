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

#[test]
fn a_missed_receiver_and_unusable_boundary_cannot_supply_impact_or_visual_evidence() {
    let mut s = point_seed();
    s.bench = Boundary::StaticCuboid {
        center: [5., 0., -0.01],
        half_extents: [0.1, 0.1, 0.01],
        orientation: [0., 0., 0., 1.],
    };
    assert!(point_control(&s, 16).is_err());
    let dir = tempfile::tempdir().unwrap();
    assert!(view::save(dir.path(), &s, &[]).is_err());
    s.bench = Boundary::StaticCuboid {
        center: [0.; 3],
        half_extents: [-0.1, 0.1, 0.01],
        orientation: [0., 0., 0., 1.],
    };
    assert!(trial(&s, Clock::uniform(16)).is_err());
    assert!(point_control(&s, 16).is_err());
}

#[test]
fn missing_native_material_and_geometry_prevent_capture() {
    let fixture = world(16, true, false).unwrap();
    let mut r = Recording::new(&fixture, "invalid.sdf", "default").unwrap();
    let deck = entity_id(&r.sim, "default::source::body").unwrap();
    let e = entity_by_id(r.sim.world(), deck).unwrap();
    r.sim.world_mut().entity_mut(e).remove::<BodyHandle>();
    assert!(seed(&r).is_err());
    r.sim.world_mut().entity_mut(e).remove::<Name>();
    assert!(seed(&r).is_err());
    let bare = r#"<sdf version="1.12"><world name="default"><model name="source"><static>true</static><link name="body"><collision name="box"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></world></sdf>"#;
    let r = Recording::new(bare, "bare.sdf", "default").unwrap();
    assert!(seed(&r).is_err());
    let round = bare.replace(
        "<box><size>1 1 1</size></box>",
        "<sphere><radius>0.5</radius></sphere>",
    );
    let r = Recording::new(&round, "round.sdf", "default").unwrap();
    assert!(seed(&r).is_err());
}

#[test]
fn a_complete_trajectory_requires_writable_archive_and_measurement_paths() {
    let s = point_seed();
    let p = s.material.particles()[0];
    let lanes: Vec<_> = (0..3)
        .map(|_| {
            (0..=HORIZON_MS)
                .map(|i| sample(&[p; arm_sheet::COUNT], s.top, i * MS, 0., true))
                .collect()
        })
        .collect();
    let f = tempfile::NamedTempFile::new().unwrap();
    assert!(view::save(f.path(), &s, &lanes).is_err());
    let d = tempfile::tempdir().unwrap();
    fs::create_dir(d.path().join("particle-frames.json")).unwrap();
    assert!(view::save(d.path(), &s, &lanes).is_err());
}
