// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::multi_sphere::MultiSpherePool;
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use serde_json::json;
use std::{error::Error, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error + Send + Sync>>;
const SDF: &str = include_str!("../../../../examples/distributed_sphere_pair_island.sdf");

/// Two three- and four-slot convoy runs agree physically through a loaded
/// migration at 150 ms. The candidate then receives one additional queued
/// torque followed by positive drive, giving a known first divergent step.
pub fn generate(worker: &Path, output: &Path) -> Result<()> {
    fs::create_dir(output)?;
    let temp = tempfile::tempdir_in(output)?;
    let reference = temp.path().join("three");
    let candidate = temp.path().join("four");
    let spec = distributed_multi_sphere_spec(SDF, "inspection.sdf", "default")?;
    let joint = spec.hinge.joint_id;
    let mut a = MultiSpherePool::create(worker, &reference, 3, spec)?;
    let mut b = MultiSpherePool::create(worker, &candidate, 4, spec)?;
    let mut rows = Vec::new();
    let mut capture = |label: &str| -> Result<()> {
        let first = format!("reference-{label}.gzmultisphere");
        let second = format!("candidate-{label}.gzmultisphere");
        fs::copy(reference.join("barrier.gzmultisphere"), output.join(&first))?;
        fs::copy(
            candidate.join("barrier.gzmultisphere"),
            output.join(&second),
        )?;
        rows.push(json!({"reference":first,"candidate":second}));
        Ok(())
    };
    for tick in 0..200 {
        if tick == 149 {
            capture("149")?;
        }
        if tick == 150 {
            assert!(a.observation().sphere_reaction.normal_impulse > 0.);
            b.transfer_pair(b.owners()[0])?;
            capture("150-migrated")?;
            b.queue_effort(joint, 0.02)?;
            capture("150-queued")?;
        }
        a.advance_with_effort_at(0, tick, joint, -0.01)?;
        b.advance_with_effort_at(0, tick, joint, if tick <= 150 { -0.01 } else { 0.01 })?;
        if [151, 155, 175, 200].contains(&(tick + 1)) {
            capture(&(tick + 1).to_string())?;
        }
    }
    fs::write(
        output.join("series.json"),
        serde_json::to_vec_pretty(&rows)?,
    )?;
    Ok(())
}
