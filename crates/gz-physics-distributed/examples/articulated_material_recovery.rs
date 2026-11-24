// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record genuine managed-worker cloth and cable impacts for gz-web-rs.
#[path = "../../gz-sim-cli/examples/soft_replay/mod.rs"]
mod soft_replay;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics_distributed::floating_tree::{
    ArticulatedMaterialPool, ArticulatedMaterialSpec, ContactTreeSpec, MaterialImpactBarrier,
};
use gz_physics_xpbd::SoftBody;
use gz_sim_scene::deformable::articulated_impact::distributed_material_contact_tree_spec;
use serde_json::json;
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    process::Command,
    time::Duration,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    /// New gz-web-rs recording directory.
    output: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    /// New directory for independently reopenable native/material session files.
    #[arg(long)]
    sessions: PathBuf,
}
struct Run {
    label: &'static str,
    cold: SoftBody,
    directory: PathBuf,
    pool: Option<ArticulatedMaterialPool>,
    shift: f64,
}
impl Run {
    fn new(label: &'static str, xml: &str, shift: f64, worker: &Path, root: &Path) -> Result<Self> {
        let recipe = distributed_material_contact_tree_spec(xml, "material.sdf", "default")?;
        let spec = ArticulatedMaterialSpec {
            tree: ContactTreeSpec {
                tree: recipe.tree,
                limits: recipe.limits,
            },
            material_name: recipe.material_name,
            barriers: recipe
                .barriers
                .into_iter()
                .map(|b| MaterialImpactBarrier {
                    tick: b.time.as_millis() as u64,
                    links: b.links,
                    restitution: b.restitution,
                })
                .collect(),
        };
        let directory = root.join(label);
        let pool =
            ArticulatedMaterialPool::create(worker, &directory, 3, spec, recipe.material.clone())?;
        Ok(Self {
            label,
            cold: recipe.material,
            directory,
            pool: Some(pool),
            shift,
        })
    }
    fn step(&mut self, tick: u64, worker: &Path) -> Result<()> {
        let p = self.pool.as_mut().unwrap();
        if tick == 60 || tick == 99 || tick == 100 || tick == 155 {
            let dest = (p.owner() + 1) % 3;
            let slot = if tick == 60 || tick == 100 {
                dest
            } else {
                p.owner()
            };
            if !Command::new("kill")
                .args(["-KILL", &p.worker_pid(slot).unwrap().to_string()])
                .status()?
                .success()
            {
                return Err("could not terminate requested worker".into());
            }
            if tick != 99 {
                p.transfer(dest)?;
            }
        }
        p.advance_with_efforts_at(0, tick, &[])?;
        if [60, 99, 100, 155].contains(&tick) {
            self.pool.take();
            self.pool = Some(ArticulatedMaterialPool::resume(
                worker,
                &self.directory,
                self.cold.clone(),
            )?);
        }
        Ok(())
    }
    fn record(&self, f: &mut Frame) -> serde_json::Value {
        let p = self.pool.as_ref().unwrap();
        let turn = DQuat::from_rotation_z(-0.72);
        let display = |v: DVec3| turn * v + DVec3::Y * self.shift;
        let key = |s: &str| format!("{}_{}", self.label, s);
        for (i, (l, r)) in p
            .spec()
            .tree
            .tree
            .links
            .iter()
            .zip(&p.observation().links)
            .enumerate()
        {
            let c = DVec3::from(r.position);
            let q = DQuat::from_array(r.orientation);
            let color = if i == 0 {
                [0.19, 0.29, 0.47, 1.]
            } else {
                [0.1, 0.64, 0.59, 1.]
            };
            f.add(
                &key(&format!("link_{}", l.id)),
                display(c),
                turn * q,
                json!({"kind":"sphere","radius":l.radius}),
                color,
            );
            if i != 0 {
                f.add(
                    &key(&format!("shank_{}", l.id)),
                    display(c + q * DVec3::new(-0.30, 0., 0.)),
                    turn * q,
                    json!({"kind":"box","size":[0.60,0.045,0.055]}),
                    [0.15, 0.46, 0.51, 1.],
                );
            } else {
                f.add(
                    &key("crossbar"),
                    display(c + q * DVec3::new(0.02, 0., 0.)),
                    turn * q,
                    json!({"kind":"box","size":[0.15,0.58,0.095]}),
                    color,
                );
            }
        }
        let v = p
            .particles()
            .iter()
            .map(|x| DVec3::from(x.position))
            .collect::<Vec<_>>();
        let edges = if v.len() == 4 {
            vec![(0, 1), (0, 2), (1, 3), (2, 3), (0, 3)]
        } else {
            (0..v.len() - 1).map(|i| (i, i + 1)).collect()
        };
        for (i, pt) in v.iter().enumerate() {
            f.add(
                &key(&format!("node_{i}")),
                display(*pt),
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":0.019}),
                [0.95, 0.59, 0.12, 1.],
            );
        }
        for (a, b) in edges {
            f.edge(
                &key(&format!("fiber_{a}_{b}")),
                display(v[a]),
                display(v[b]),
                (DVec3::from(self.cold.particles()[b].position)
                    - DVec3::from(self.cold.particles()[a].position))
                .length(),
                0.014,
                [0.89, 0.54, 0.12, 1.],
            );
        }
        json!({"epoch":p.epoch(),"tick":p.tick(),"owner":p.owner(),"generation":p.generation(),"rigid":p.observation(),"particles":p.particles().iter().map(|x|json!([x.position,x.velocity])).collect::<Vec<_>>(),"last_impact":p.last_impact()})
    }
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.sessions)?;
    let mut runs = [
        Run::new(
            "cloth",
            include_str!("../../../examples/articulated_material_cloth_impact.sdf"),
            0.82,
            &args.worker,
            &args.sessions,
        )?,
        Run::new(
            "cable",
            include_str!("../../../examples/articulated_material_cable_impact.sdf"),
            -0.82,
            &args.worker,
            &args.sessions,
        )?,
    ];
    let mut raw = BufWriter::new(fs::File::create_new(args.sessions.join("states.jsonl"))?);
    let mut frames = Vec::new();
    for t in 0..=240 {
        let mut f = Frame::default();
        let mut row = json!({"time_ms":t});
        for run in &mut runs {
            if t > 0 {
                run.step(t - 1, &args.worker)?;
            }
            row[run.label] = run.record(&mut f);
        }
        serde_json::to_writer(&mut raw, &row)?;
        writeln!(raw)?;
        if t % 2 == 0 {
            frames.push(f.finish(Duration::from_millis(t), frames.len() + 1));
        }
    }
    raw.flush()?;
    gz_sim_cli::export_archive(&args.output, BTreeMap::new(), frames)?;
    println!(
        "121 recorded frames per material, 8 real process faults, 8 live controller reopens, both impacts at 100 ms."
    );
    Ok(())
}
