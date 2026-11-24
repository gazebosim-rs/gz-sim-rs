// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record normal and sliding impulses from three ordinary local cable materials.
use glam::DVec3;
use gz_sim_scene::{Name, PhysicsEngine, WorldPose, deformable::bodies, load_world_with_sdf_step};
use serde_json::json;
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Duration,
};

const WORLD: &str = include_str!("../../../examples/worlds/local_floor_impulses.sdf");

fn main() -> Result<(), Box<dyn Error>> {
    let output = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply a new output directory")?,
    );
    fs::create_dir(&output)?;
    record(WORLD, &output)
}

fn record(world: &str, output: &Path) -> Result<(), Box<dyn Error>> {
    let sdf = sdformat::load_str(world, "local_floor_impulses.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        world,
        "local_floor_impulses.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut readings = BufWriter::new(fs::File::create_new(output.join("impulses.jsonl"))?);
    let gauges: BTreeMap<_, _> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| {
            let key = entity
                .get::<Name>()?
                .0
                .strip_prefix("default::table::deck::gauge_")?;
            Some((
                key.to_owned(),
                (entity.id(), entity.get::<WorldPose>()?.translation),
            ))
        })
        .collect();
    let mut frames = vec![gz_sim_cli::primitive_frame(&sim, &sdf, "default", 1)?];
    let mut peak_error = 0_f64;
    let mut observations = 0;
    let mut impulse_sums = BTreeMap::<String, DVec3>::new();
    for tick in 1..=100 {
        let previous: BTreeMap<_, _> = bodies(&sim)
            .map(|(name, s)| (name.to_string(), s.particles().to_vec()))
            .collect();
        sim.step_exact(1)?;
        let dt = sim.step_duration().as_secs_f64();
        let time = sim.elapsed().as_nanos() as u64;
        let mut gauge_positions = Vec::new();
        for (name, material) in bodies(&sim) {
            let floor = material
                .last_plane_impulses()
                .ok_or("floor sensor disabled")??;
            for (node, (p0, p1)) in previous[name].iter().zip(material.particles()).enumerate() {
                let j = floor.iter().find(|j| j.particle == node);
                let normal = j.map_or([0.; 3], |j| j.normal);
                let tangent = j.map_or([0.; 3], |j| j.tangent);
                let surface = DVec3::from(normal) + DVec3::from(tangent);
                let (gauge, origin) = gauges
                    .get(&format!("{name}_{node}"))
                    .ok_or("missing floor gauge")?;
                // Display only. Up is the downward normal reaction on the floor;
                // horizontal displacement shows the signed floor friction impulse.
                gauge_positions.push((
                    *gauge,
                    [
                        origin[0] + 3. * tangent[0],
                        origin[1] + 3. * tangent[1],
                        origin[2] - 0.8 * normal[2],
                    ],
                ));
                let expected = (DVec3::from(p0.velocity) - DVec3::from(p1.velocity)
                    + DVec3::new(0., 0., -10. * dt))
                    / p1.inverse_mass;
                let pad = material
                    .last_pad_loads()
                    .iter()
                    .filter(|j| j.particle == node)
                    .map(|j| DVec3::from(j.normal) + DVec3::from(j.tangent))
                    .sum::<DVec3>();
                peak_error = peak_error.max((surface + pad).distance(expected));
                *impulse_sums.entry(name.to_owned()).or_default() += surface;
                serde_json::to_writer(
                    &mut readings,
                    &json!({"time_ns":time,"material":name,"particle":node,"position_m":p1.position,"normal_ns":normal,"tangent_ns":tangent,"pad_ns":pad.to_array(),"momentum_reference_ns":expected.to_array()}),
                )?;
                writeln!(readings)?;
                observations += 1;
            }
        }
        for (entity, position) in gauge_positions {
            sim.world_mut()
                .get_mut::<WorldPose>(entity)
                .expect("authored gauge")
                .translation = position;
        }
        frames.push(gz_sim_cli::primitive_frame(
            &sim,
            &sdf,
            "default",
            tick + 1,
        )?);
    }
    readings.flush()?;
    if !peak_error.is_finite() || peak_error > 2e-11 {
        return Err(format!("per-particle impulse discrepancy {peak_error:e} N·s").into());
    }
    let ticks = frames.len() - 1;
    gz_sim_cli::export_archive_at(
        &output.join("replay"),
        BTreeMap::new(),
        0,
        Duration::ZERO,
        frames,
    )?;
    let sums: BTreeMap<_, _> = impulse_sums
        .into_iter()
        .map(|(n, v)| (n, v.to_array()))
        .collect();
    let summary = json!({"step_ns":sim.step_duration().as_nanos() as u64,"ticks":ticks,"observations":observations,"peak_node_impulse_error_ns":peak_error,"total_plane_impulse_ns":sums});
    fs::write(
        output.join("summary.json"),
        serde_json::to_string_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn physical_controls_and_failed_outputs_cannot_publish_a_qualification() {
        let tmp = tempfile::tempdir().unwrap();
        let check = |xml: &str| {
            let output = tempfile::tempdir_in(tmp.path()).unwrap();
            let e = record(xml, output.path()).unwrap_err().to_string();
            assert!(!output.path().join("summary.json").exists());
            e
        };
        assert!(!check("<sdf>").is_empty());
        assert!(
            check(&WORLD.replace("gauge_falling_0", "missing_marker"))
                .contains("missing floor gauge")
        );
        assert!(check(&WORLD.replace("normal=\"0 0 1\"", "normal=\"0 0 0\"")).contains("normal"));
        assert!(
            check(&WORLD.replace("plane_impulses=\"true\"", "plane_impulses=\"false\""))
                .contains("disabled")
        );
        assert!(
            check(&WORLD.replace("0 0 -10</gravity>", "0 0 -1</gravity>")).contains("discrepancy")
        );
        let dir = tempfile::tempdir_in(tmp.path()).unwrap();
        fs::create_dir(dir.path().join("replay")).unwrap();
        assert!(record(WORLD, dir.path()).is_err());
        assert!(!dir.path().join("summary.json").exists());
        fs::remove_dir(dir.path().join("replay")).unwrap();
        assert!(record(WORLD, dir.path()).is_err()); // Refuse the previous trace.
        let dir = tempfile::tempdir_in(tmp.path()).unwrap();
        fs::create_dir(dir.path().join("summary.json")).unwrap();
        assert!(record(WORLD, dir.path()).is_err());
    }
}
