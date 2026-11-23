// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../../../examples/support/grounded_box_array.rs"]
mod grounded_box_array;

use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_distributed::grounded_robot::{GroundedRobotPool, RobotGraphEncoding};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, PhysicsEngine, distributed_hinge::distributed_grounded_robot_spec,
    load_world_with_sdf_step,
};
use serde_json::json;
use std::{error::Error, fs, io::Write, path::Path, process::Command, time::Instant};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn native_id(sim: &Simulation, public: u64) -> BodyId {
    sim.world()
        .get::<BodyHandle>(entity_by_id(sim.world(), public).unwrap())
        .unwrap()
        .id()
}

#[test]
fn maximum_loaded_ground_patches_continue_exactly_over_full_and_packed_pages() -> Result<()> {
    let file = env!("CARGO_BIN_EXE_physics_worker");
    let exe = Path::new(file);
    let xml = grounded_box_array::boxes(32);
    let spec = distributed_grounded_robot_spec(&xml, "array.sdf", "default")?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "array.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let root = native_id(&sim, spec.links[0].id);
    let floor = native_id(&sim, spec.ground.link_id);
    let public = spec.links[19].id;
    let driven = native_id(&sim, public);
    let directory = tempfile::tempdir()?;
    let mut pools = Vec::new();
    for (name, encoding) in [
        ("raw", RobotGraphEncoding::FullPages),
        ("packed", RobotGraphEncoding::Packed),
    ] {
        let path = directory.path().join(name);
        let pool = GroundedRobotPool::create_with_encoding(exe, &path, 3, spec.clone(), encoding)?;
        pools.push((path, encoding, pool, 0u64, 0u64));
    }
    let output = std::env::var_os("GZ_ROBOT_CAPACITY_CAPTURE").map(std::path::PathBuf::from);
    if let Some(dir) = &output {
        fs::create_dir_all(dir)?;
    }
    let mut out = output
        .as_ref()
        .map(|d| fs::File::create(d.join("timeline.jsonl")))
        .transpose()?;
    let mut full_ticks = 0;
    let mut largest = 0;
    let mut first = None;
    let mut disturbed = None;
    let mut rest_load = 0.;
    for tick in 0..160u64 {
        if tick == 64 || tick == 100 {
            for (_, _, p, _, _) in &mut pools {
                p.transfer((p.owner() + 1) % 3)?;
                let pid = p.worker_pid(p.owner()).unwrap();
                assert!(
                    Command::new("kill")
                        .args(["-KILL", &pid.to_string()])
                        .status()?
                        .success()
                );
            }
        }
        if tick == 89 || tick == 123 {
            for (dir, encoding, p, uploaded, downloaded) in std::mem::take(&mut pools) {
                let traffic = p.traffic();
                drop(p);
                let resumed = GroundedRobotPool::resume_with_encoding(exe, &dir, encoding)?;
                pools.push((
                    dir,
                    encoding,
                    resumed,
                    uploaded + traffic.uploaded_bytes,
                    downloaded + traffic.downloaded_bytes,
                ));
            }
        }
        let mut forces = vec![];
        if (84..106).contains(&tick) {
            let f = [4., -2., -27.];
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .add_world_wrench(driven, f, [0.; 3])?;
            forces.push((public, f));
        }
        sim.step_exact(1)?;
        let be = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let native = be.checkpoint_grounded_robot_tree(root, floor)?;
        let mut times = Vec::new();
        for (_, _, p, _, _) in &mut pools {
            let start = Instant::now();
            p.advance_with_inputs_at(0, tick, &[], &forces)?;
            times.push(start.elapsed().as_micros() as u64);
            assert_eq!(p.tick(), tick + 1);
            assert!(
                p.native_image() == Some(&native[..]),
                "native graph differs at {tick}"
            );
        }
        largest = largest.max(native.len());
        let a = &pools[0].2;
        let b = &pools[1].2;
        assert_eq!(
            bincode::serialize(a.observation())?,
            bincode::serialize(b.observation())?
        );
        assert_eq!(
            bincode::serialize(a.ground_reactions())?,
            bincode::serialize(b.ground_reactions())?
        );
        assert_eq!(
            bincode::serialize(a.ground_patches())?,
            bincode::serialize(b.ground_patches())?
        );
        let ordinary_patches = be.fixed_ground_patches(floor)?;
        assert_eq!(a.ground_patches().len(), ordinary_patches.len());
        for patch in a.ground_patches() {
            let id = native_id(&sim, patch.link_id);
            let reference = ordinary_patches.iter().find(|p| p.body == id).unwrap();
            assert_eq!(
                reference.normal_on_body.map(f64::to_bits),
                patch.normal_on_body.map(f64::to_bits)
            );
            assert_eq!(
                bincode::serialize(&reference.points)?,
                bincode::serialize(&patch.points)?
            );
        }
        let solved = a
            .ground_patches()
            .iter()
            .flat_map(|p| &p.points)
            .filter(|p| p.normal_impulse > 0.)
            .count();
        let load = a
            .ground_patches()
            .iter()
            .flat_map(|p| &p.points)
            .map(|p| 1000. * p.normal_impulse)
            .sum::<f64>();
        if solved == 128 {
            full_ticks += 1;
            first.get_or_insert(tick + 1);
        }
        if tick == 105 {
            disturbed = Some(load);
        }
        if tick >= 140 {
            rest_load += load / 20.;
        }
        if let Some(out) = out.as_mut() {
            serde_json::to_writer(
                &mut *out,
                &json!({"tick":tick+1,"links":a.observation().links,"patches":a.ground_patches(),"force_N":forces,"loaded_points":solved,"normal_N":load,"graph_bytes":native.len(),"durable_us":times}),
            )?;
            writeln!(out)?;
        }
    }
    assert!(full_ticks >= 50, "{full_ticks} full-contact cuts");
    assert!(
        (rest_load - 32. * 9.81).abs() < 1.5,
        "resting load {rest_load}"
    );
    assert!(
        disturbed.unwrap() > rest_load + 20.,
        "should react to the external downward load"
    );
    assert!(largest > 300_000);
    for (_, mode, p, upload, download) in &pools {
        let tr = p.traffic();
        println!(
            "mode={mode:?} uploaded_graph_bytes={} downloaded_graph_bytes={} full_128_point_ticks={full_ticks} first_full_tick={} maximum_graph_bytes={largest} rest_N={rest_load} loaded_N={}",
            upload + tr.uploaded_bytes,
            download + tr.downloaded_bytes,
            first.unwrap(),
            disturbed.unwrap()
        );
    }
    Ok(())
}
