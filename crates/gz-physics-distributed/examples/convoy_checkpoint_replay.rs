// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Export validated physical checkpoints as a native primitive replay with measured clocks.
#[allow(dead_code)]
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
use clap::Parser;
use gz_physics_distributed::multi_sphere::{MultiSphereSpec, inspect_published_path};
use primitive_replay_view::View;
use serde::Serialize;
use serde_json::json;
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Duration,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// New directory for the finalized replay and source index.
    #[arg(long)]
    output: PathBuf,
    /// Published archives in measured time order. An identical repeat is coalesced.
    #[arg(required = true)]
    archives: Vec<PathBuf>,
}

// Replay entity IDs occupy a separate model/visual namespace. The source index
// maps each of these pairs to the stable physical ID in the original recipe.
const ROLES: [(&str, u64, [f64; 4]); 4] = [
    ("hinge_base", 10, [0.14, 0.48, 0.83, 1.]),
    ("hinge_child", 20, [0.96, 0.43, 0.12, 1.]),
    ("sphere_a", 30, [0.70, 0.12, 0.55, 1.]),
    ("sphere_b", 40, [0.16, 0.78, 0.35, 1.]),
];

#[derive(Serialize)]
struct Frame {
    sequence: u64,
    epoch: u64,
    tick: u64,
    sha256: String,
    sources: Vec<PathBuf>,
}

fn export(args: Args) -> Result<()> {
    if args.output.try_exists()? {
        return Err("replay output already exists".into());
    }
    let mut sources = Vec::<Frame>::new();
    let mut frames = Vec::new();
    let mut recipe: Option<MultiSphereSpec> = None;
    for path in args.archives {
        let inspected = inspect_published_path(&path)?;
        let c = inspected.cut;
        if let Some(spec) = recipe {
            if serde_json::to_vec(&spec)? != serde_json::to_vec(&c.spec)? {
                return Err(format!("different world recipe in {}", path.display()).into());
            }
        } else {
            recipe = Some(c.spec);
        }
        let tick = c.observation.hinge.tick;
        if let Some(previous) = sources.last_mut() {
            if c.epoch != previous.epoch || tick < previous.tick {
                return Err("mixed epoch or backwards checkpoint time".into());
            }
            if tick == previous.tick {
                if inspected.sha256 != previous.sha256 {
                    return Err(format!(
                        "conflicting checkpoints at epoch {} tick {tick}",
                        c.epoch
                    )
                    .into());
                }
                previous.sources.push(path);
                continue;
            }
        }
        let mut view = View::default();
        let specs = c.spec.hinge.links.iter().chain(&c.spec.spheres);
        let states = c
            .observation
            .hinge
            .links
            .iter()
            .chain(&c.observation.spheres);
        for ((spec, state), (role, id, color)) in specs.zip(states).zip(ROLES) {
            view.add(
                id,
                role,
                state.position,
                state.orientation,
                json!({"kind":"sphere","radius":spec.radius}),
                color,
            );
        }
        let sequence = sources.len() as u64 + 1;
        frames.push(view.finish_stamp(sequence, c.epoch, Duration::from_millis(tick), true));
        sources.push(Frame {
            sequence,
            epoch: c.epoch,
            tick,
            sha256: inspected.sha256,
            sources: vec![path],
        });
    }
    let spec = recipe.ok_or("a replay requires at least one checkpoint")?;
    let first = sources.first().unwrap();
    let parent = args
        .output
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let stage = tempfile::tempdir_in(parent)?;
    gz_sim_cli::export_archive_at(
        &stage.path().join("replay"),
        Default::default(),
        first.epoch,
        Duration::from_millis(first.tick),
        frames,
    )?;
    let mapping: Vec<_> = spec
        .hinge
        .links
        .iter()
        .chain(&spec.spheres)
        .zip(ROLES)
        .map(|(body, (role, model_id, _))| {
            json!({
                "role": role,
                "source_body_id": body.id,
                "replay_model_id": model_id,
                "replay_visual_id": model_id + 1,
                "radius_m": body.radius,
            })
        })
        .collect();
    fs::write(
        stage.path().join("index.json"),
        serde_json::to_vec_pretty(&json!({"bodies":mapping,"frames":sources}))?,
    )?;
    publish(stage, &args.output)
}

// Reserve the destination only after full archive validation and index creation.
fn publish(stage: tempfile::TempDir, destination: &Path) -> Result<()> {
    #[cfg(target_os = "linux")]
    {
        use rustix::fs::{CWD, RenameFlags, renameat_with};
        renameat_with(CWD, stage.path(), CWD, destination, RenameFlags::NOREPLACE)?;
        // The old temporary name is no longer ours.
        let _ = stage.keep();
    }
    #[cfg(not(target_os = "linux"))]
    {
        fs::create_dir(destination)?;
        if let Err(e) = fs::rename(stage.path().join("replay"), destination.join("replay")) {
            fs::remove_dir(destination)?;
            return Err(e.into());
        }
        if let Err(e) = fs::rename(
            stage.path().join("index.json"),
            destination.join("index.json"),
        ) {
            fs::rename(destination.join("replay"), stage.path().join("replay"))?;
            fs::remove_dir(destination)?;
            return Err(e.into());
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    export(Args::parse())
}

#[cfg(test)]
mod tests {
    use super::*;
    use gz_physics_distributed::multi_sphere::MultiSpherePool;
    use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;

    #[test]
    #[ignore = "requires GZ_MULTI_TEST_WORKER pointing to the stock physics worker"]
    fn exact_duplicate_nonzero_epoch_and_rejections() -> Result<()> {
        let worker =
            PathBuf::from(std::env::var_os("GZ_MULTI_TEST_WORKER").ok_or("missing worker")?);
        let dir = tempfile::tempdir()?;
        let root = dir.path();
        let spec = distributed_multi_sphere_spec(
            include_str!("../../../examples/distributed_sphere_pair_island.sdf"),
            "fixture.sdf",
            "default",
        )?;
        let archive = root.join("session/barrier.gzmultisphere");
        let mut pool = MultiSpherePool::create(&worker, &root.join("session"), 3, spec)?;
        let before = root.join("before");
        fs::copy(&archive, &before)?;
        pool.reset()?;
        pool.advance()?;
        let source = root.join("source");
        fs::copy(&archive, &source)?;
        let output = root.join("result");
        export(Args {
            output: output.clone(),
            archives: vec![source.clone(), source.clone()],
        })?;
        let index: serde_json::Value =
            serde_json::from_slice(&fs::read(output.join("index.json"))?)?;
        assert_eq!(index["frames"].as_array().unwrap().len(), 1);
        assert_eq!(index["frames"][0]["sources"].as_array().unwrap().len(), 2);
        assert_eq!(index["frames"][0]["epoch"], 1);
        assert_eq!(index["frames"][0]["tick"], 1);
        assert!(
            export(Args {
                output,
                archives: vec![source.clone()]
            })
            .is_err()
        );
        pool.queue_effort(spec.hinge.joint_id, 0.02)?;
        let reject = |name: &str, paths: Vec<PathBuf>, reason: &str| {
            let target = root.join(name);
            let err = export(Args {
                output: target.clone(),
                archives: paths,
            })
            .unwrap_err();
            assert!(err.to_string().contains(reason), "{err}");
            assert!(!target.exists());
        };
        reject(
            "same-clock",
            vec![source.clone(), archive.clone()],
            "conflicting checkpoints",
        );
        reject("mixed-epoch", vec![before, archive.clone()], "mixed epoch");
        pool.advance()?;
        reject("backwards", vec![archive, source.clone()], "backwards");
        let mut other_spec = spec;
        other_spec.spheres[1].velocity[0] += 0.01;
        let _other = MultiSpherePool::create(&worker, &root.join("other"), 3, other_spec)?;
        reject(
            "recipe",
            vec![source, root.join("other/barrier.gzmultisphere")],
            "different world recipe",
        );
        Ok(())
    }
}
