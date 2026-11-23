// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{PhysicsEngine, deformable::bodies, load_world_with_sdf_step};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{env, fs};

const SDF: &str = include_str!("../../../examples/worlds/local_soft_static_bench.sdf");

#[test]
fn cloth_rests_and_leaves_the_authored_finite_offset_bench() {
    let mut sim = load_world_with_sdf_step(
        SDF,
        "bench.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let mut frames = Vec::new();
    let mut last_slide = 0;
    let mut first_rest = 0;
    let mut peak_lateral = 0f64;
    let mut last_rest = 0.;
    let mut final_slide_top = 0.;
    for tick in 0..=320 {
        if tick > 0 {
            sim.step_exact(1).unwrap();
        }
        let mut poses = serde_json::Map::new();
        let mut reactions = serde_json::Map::new();
        poses.insert(
            "default::bench::deck::top".into(),
            json!({"translation":[0.1,0.,0.4],"rotation":[0.,0.,0.,1.]}),
        );
        for (name, soft) in bodies(&sim) {
            let j = soft
                .last_boundary_impulses()
                .first()
                .map_or([0.; 3], |j| j.linear);
            let side = j[0].hypot(j[1]);
            if name == "rest" {
                if j[2] < -1e-10 && first_rest == 0 {
                    first_rest = tick;
                }
                if tick >= 285 {
                    last_rest = -j[2] * 1000.;
                    assert!((last_rest - 9. * 0.004 * 9.81).abs() < 1e-7);
                    assert!(
                        soft.particles()
                            .iter()
                            .all(|p| (p.position[2] - 0.452).abs() < 1e-8)
                    );
                }
            } else {
                if j.iter().any(|a| a.abs() > 1e-10) {
                    last_slide = tick;
                }
                if soft.particles().iter().all(|p| p.position[0] < 0.26) {
                    assert!(side <= 0.1 * (-j[2]) + 1e-10);
                    peak_lateral = peak_lateral.max(side * 1000.);
                }
                if tick == 320 {
                    final_slide_top = soft
                        .particles()
                        .iter()
                        .map(|p| p.position[2])
                        .fold(f64::NEG_INFINITY, f64::max);
                    assert!(
                        soft.particles()
                            .iter()
                            .all(|p| p.position[0] > 0.272 && p.position[2] < 0.23)
                    );
                }
            }
            for (i, p) in soft.particles().iter().enumerate() {
                poses.insert(
                    format!("default::bench::deck::{name}_{i}"),
                    json!({"translation":p.position,"rotation":[0.,0.,0.,1.]}),
                );
            }
            reactions.insert(name.to_string(), json!(j));
        }
        frames
            .push(json!({"time_ns":tick*1_000_000,"poses":poses,"impulse_on_bench_ns":reactions}));
    }
    assert!((100..=130).contains(&first_rest));
    assert!((40..=140).contains(&last_slide));
    assert!(peak_lateral > 0.03);
    let report = json!({"first_rest_contact_ms":first_rest,"last_slide_contact_ms":last_slide,"rest_weight_n":last_rest,"peak_slide_lateral_force_n":peak_lateral,"departed_sheet_highest_z_m":final_slide_top,"bench_top_z_m":0.44});
    println!("{report}");
    if let Some(out) = env::var_os("GZ_STATIC_BENCH_MEASUREMENT") {
        let out = std::path::Path::new(&out);
        fs::create_dir_all(out).unwrap();
        fs::write(
            out.join("comparison.json"),
            serde_json::to_vec_pretty(&report).unwrap(),
        )
        .unwrap();
        fs::write(
            out.join("frames.json"),
            serde_json::to_vec(&Value::Array(frames)).unwrap(),
        )
        .unwrap();
    }
}

#[test]
fn dynamic_and_offset_wrench_boxes_keep_their_existing_requirements() {
    for (sdf, why) in [
        (
            SDF.replacen("<static>true</static>", "<static>false</static>", 1),
            "centered geometry",
        ),
        (
            SDF.replace("coupling=\"one_way\"", "coupling=\"articulated_wrench\""),
            "centered geometry",
        ),
        (
            SDF.replacen("<static>true</static>", "<static>false</static>", 1)
                .replace("<pose>0.05 0 0 0 0 -1.5707963267948966</pose>", ""),
            "require a static",
        ),
    ] {
        let err = load_world_with_sdf_step(
            &sdf,
            "bench.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .err()
        .expect("invalid admission");
        assert!(err.to_string().contains(why), "{err}");
    }
}

#[test]
fn finite_static_deck_can_share_the_coupled_solver_with_dynamic_surface_pads() {
    let original = include_str!("../../../examples/folded_surface_pad_transport.sdf");
    let bench = r#"<model name="table"><static>true</static><pose>0 0 -0.015 0 0 0</pose><link name="deck"><collision name="surface"><geometry><box><size>1 1 0.03</size></box></geometry></collision></link></model>"#;
    let xml = original
        .replace("  <plugin ", &format!("  {bench}\n  <plugin "))
        .replace(
            "<plane point=\"0 0 0\" normal=\"0 0 1\"/>",
            "<box link=\"default::table::deck\" coupling=\"one_way\"/>",
        );
    let mut finite = load_world_with_sdf_step(
        &xml,
        "table.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let mut plane = load_world_with_sdf_step(
        original,
        "plane.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    for _ in 0..30 {
        finite.step_exact(1).unwrap();
        plane.step_exact(1).unwrap();
        let (_, cloth) = bodies(&finite).next().unwrap();
        let (_, reference) = bodies(&plane).next().unwrap();
        assert_eq!(cloth.last_boundary_impulses().len(), 3);
        for (a, b) in cloth.particles().iter().zip(reference.particles()) {
            assert!(glam::DVec3::from(a.position).distance(glam::DVec3::from(b.position)) < 1e-6);
        }
    }
}
