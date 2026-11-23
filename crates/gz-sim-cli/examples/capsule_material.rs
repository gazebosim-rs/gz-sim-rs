// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare particle-only support and swept material-feature support in the actual Bevy scene.
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, PhysicsBackend};
use gz_sim_core::Simulation;
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};
mod soft_replay;
use soft_replay::Frame;
#[derive(Parser)]
struct Args {
    /// New gz-web-rs wire archive.
    output: PathBuf,
    /// Use the four-vertex sheet supported by two endcaps.
    #[arg(long)]
    cloth: bool,
    /// Use a freely tumbling finite-mass receiver in zero gravity.
    #[arg(long)]
    finite: bool,
    /// Save every Bevy tick, including topology-attributed support impulses.
    #[arg(long)]
    measurements: Option<PathBuf>,
}
struct Run {
    sim: Simulation,
    name: &'static str,
    shift: DVec3,
    color: [f64; 4],
    guides: Vec<BodyId>,
    rest: Vec<DVec3>,
    finite: bool,
    initial_rigid_velocity: DVec3,
    nodal_momentum: Vec<DVec3>,
}
impl Run {
    fn new(cloth: bool, material: bool, finite: bool) -> Result<Self, Box<dyn Error>> {
        let xml = match (cloth, finite) {
            (false, false) => include_str!("../../../examples/material_capsule_cable.sdf"),
            (true, false) => include_str!("../../../examples/material_capsule_cloth.sdf"),
            (false, true) => include_str!("../../../examples/finite_material_cable.sdf"),
            (true, true) => include_str!("../../../examples/finite_material_cloth.sdf"),
        };
        let xml = if material {
            xml.to_owned()
        } else {
            xml.replace(
                "capsule_contact=\"material\"",
                "capsule_contact=\"particles\"",
            )
        };
        let sim = load_world_with_sdf_step(
            &xml,
            "material_capsule.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let mut ids = sim
            .world()
            .iter_entities()
            .filter_map(|e| {
                let n = e.get::<Name>()?;
                n.0.ends_with("::guide")
                    .then(|| (n.0.clone(), e.get::<BodyHandle>().unwrap().id()))
            })
            .collect::<Vec<_>>();
        ids.sort_by(|a, b| a.0.cmp(&b.0));
        let (name, offset, color) = if material {
            ("material", -0.63, [0.09, 0.77, 0.76, 1.])
        } else {
            ("particles", 0.63, [0.94, 0.43, 0.15, 1.])
        };
        let initial_rigid_velocity = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(ids[0].1)
            .unwrap()
            .velocity
            .into();
        let rest: Vec<_> = bodies(&sim)
            .next()
            .unwrap()
            .1
            .particles()
            .iter()
            .map(|p| DVec3::from(p.position))
            .collect();
        Ok(Self {
            finite,
            initial_rigid_velocity,
            nodal_momentum: vec![DVec3::ZERO; rest.len()],
            rest,
            sim,
            name,
            shift: DVec3::Y * offset,
            color,
            guides: ids.into_iter().map(|(_, i)| i).collect(),
        })
    }
    fn sample(&mut self, cloth: bool, f: &mut Frame) -> Value {
        let n = self.name;
        let blue = [0.16, 0.35, 0.61, 1.];
        let gold = [1., 0.76, 0.18, 1.];
        let mut guides = vec![];
        for (id, handle) in self.guides.iter().enumerate() {
            let s = self
                .sim
                .world()
                .resource::<PhysicsBackend>()
                .state(*handle)
                .unwrap();
            let q = DQuat::from_array(s.orientation);
            let center = DVec3::from(s.position) + self.shift;
            guides.push(json!({"center":s.position,"orientation":s.orientation,"velocity":s.velocity,"spin":s.angular_velocity}));
            for (side, sign) in [("negative", -1.), ("positive", 1.)] {
                f.add(
                    &format!("{n}_guide_{id}_{side}_cap"),
                    center + q * (sign * 0.15 * DVec3::Z),
                    q,
                    json!({"kind":"sphere","radius":0.08}),
                    blue,
                );
            }
            for j in 0..12 {
                let o = q * DQuat::from_rotation_z(std::f64::consts::TAU * j as f64 / 12.);
                f.add(
                    &format!("{n}_guide_{id}_barrel_{j}"),
                    center + o * (DVec3::X * 0.077),
                    o,
                    json!({"kind":"box","size":[0.006,0.16*(std::f64::consts::PI/12.).tan(),0.30]}),
                    if self.finite && j == 0 { gold } else { blue },
                );
            }
        }
        let sample = bodies(&self.sim).next().unwrap().1;
        let ps = sample.particles();
        let mut loads = vec![];
        let mut per_node = vec![DVec3::ZERO; ps.len()];
        for l in sample.last_capsule_material_loads() {
            let vertices = l.feature.vertices();
            for (&idx, j) in vertices.iter().zip(l.nodal) {
                per_node[idx] += DVec3::from(j);
            }
            loads.push(json!({"boundary":l.boundary,"vertices":vertices,"region":format!("{:?}",l.region),"first_time":l.first_time,"linear":l.linear,"angular":l.angular,"nodal":l.nodal}));
        }
        if self.finite {
            for (total, j) in self.nodal_momentum.iter_mut().zip(&per_node) {
                *total += *j;
            }
            for (row, load) in loads.iter_mut().zip(sample.last_capsule_material_loads()) {
                row["two_way"] = json!(load.two_way);
            }
        }
        let mut particles = vec![];
        for (i, p) in ps.iter().enumerate() {
            particles.push(
                json!({"position":p.position,"velocity":p.velocity,"mass":p.inverse_mass.recip()}),
            );
            let pos = DVec3::from(p.position) + self.shift;
            f.add(
                &format!("{n}_node_{i}"),
                pos,
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":0.033}),
                self.color,
            );
            // A gold indicator rises above its material vertex under load.
            // The scale saturates at that vertex's full gravitational weight.
            let fraction = if self.finite {
                (self.nodal_momentum[i].length() / 0.12).min(1.)
            } else {
                (per_node[i].z * p.inverse_mass / (9.81 * 0.005)).clamp(0., 1.)
            };
            f.add(
                &format!("{n}_node_load_{i}"),
                pos + DVec3::Z * (0.058 * fraction),
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":0.015}),
                gold,
            );
        }
        let edges: &[(usize, usize)] = if cloth {
            &[(0, 1), (1, 3), (3, 2), (2, 0), (0, 3)]
        } else {
            &[(0, 1)]
        };
        for &(a, b) in edges {
            let x = DVec3::from(ps[a].position) + self.shift;
            let y = DVec3::from(ps[b].position) + self.shift;
            f.edge(
                &format!("{n}_material_edge_{a}_{b}"),
                x,
                y,
                self.rest[a].distance(self.rest[b]),
                if a == 0 && b == 3 { 0.012 } else { 0.02 },
                self.color,
            );
        }
        let reactions = sample.last_boundary_impulses();
        if self.finite {
            let end = self
                .sim
                .world()
                .resource::<PhysicsBackend>()
                .state(self.guides[0])
                .unwrap();
            let rigid_impulse = 0.4 * (DVec3::from(end.velocity) - self.initial_rigid_velocity);
            let a = self.shift + DVec3::new(0.42, 0.24, 0.10);
            let b = a - DVec3::Y * 0.48;
            f.edge(
                &format!("{n}_impulse_scale"),
                a,
                b,
                0.48,
                0.011,
                [0.66, 0.68, 0.70, 1.],
            );
            f.add(
                &format!("{n}_received_impulse"),
                a.lerp(b, (rigid_impulse.length() / 0.20).clamp(0., 1.)),
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":0.030}),
                self.color,
            );
            return json!({"guides":guides,"particles":particles,"loads":loads,"reactions":reactions.iter().map(|r|json!({"boundary":r.boundary,"linear":r.linear,"angular":r.angular})).collect::<Vec<_>>(),"native_impulse":rigid_impulse.to_array(),"nodal_impulses":self.nodal_momentum.iter().map(|j|j.to_array()).collect::<Vec<_>>()});
        }
        let weight = ps.iter().map(|p| 9.81 / p.inverse_mass).sum::<f64>();
        let vertical = -reactions
            .iter()
            .take(self.guides.len())
            .map(|v| v.linear[2])
            .sum::<f64>()
            / 0.005;
        // A bounded display scale shows both static support and impact without
        // drawing the transient peak outside the world. The measurement keeps it in full.
        let a = self.shift + DVec3::new(0.4, 0.25, 0.20);
        let b = a - DVec3::Y * 0.5;
        f.edge(
            &format!("{n}_weight_scale"),
            a,
            b,
            0.5,
            0.01,
            [0.66, 0.68, 0.70, 1.],
        );
        f.add(
            &format!("{n}_weight_marker"),
            a.lerp(b, (vertical / weight).clamp(0., 1.)),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.027}),
            self.color,
        );
        f.add(
            &format!("{n}_ground"),
            self.shift + DVec3::new(0., 0., 0.10),
            DQuat::IDENTITY,
            json!({"kind":"box","size":[0.9,1.05,0.04]}),
            [0.23, 0.25, 0.27, 1.],
        );
        json!({"guides":guides,"particles":particles,"loads":loads,"reactions":reactions.iter().map(|r|json!({"boundary":r.boundary,"linear":r.linear,"angular":r.angular})).collect::<Vec<_>>(),"guide_force_z":vertical,"material_weight":weight})
    }
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let mut runs = [
        Run::new(args.cloth, false, args.finite)?,
        Run::new(args.cloth, true, args.finite)?,
    ];
    let mut out = args
        .measurements
        .map(File::create_new)
        .transpose()?
        .map(BufWriter::new);
    let mut frames = vec![];
    let final_tick = if args.finite { 350 } else { 200 };
    for tick in 0..=final_tick {
        let mut f = Frame::default();
        let mut row = json!({"tick":tick,"frame":null});
        for run in &mut runs {
            if tick > 0 {
                run.sim.step_exact(1)?;
            }
            row[run.name] = run.sample(args.cloth, &mut f);
        }
        if tick % 2 == 0 || (!args.finite && (35..=41).contains(&tick)) {
            frames.push(f.finish(runs[0].sim.elapsed(), frames.len() + 1));
            row["frame"] = json!(frames.len());
        }
        if let Some(w) = &mut out {
            serde_json::to_writer(&mut *w, &row)?;
            writeln!(w)?;
        }
        if tick == final_tick && args.finite {
            println!(
                "0.7 s, changes in native linear momentum (N s), particle contact {} and material contact {}.",
                row["particles"]["native_impulse"], row["material"]["native_impulse"]
            );
        }
        if tick == final_tick && !args.finite {
            println!(
                "1 s, guide support ordinary/material = {} / {} N for {} N of material.",
                row["particles"]["guide_force_z"],
                row["material"]["guide_force_z"],
                row["material"]["material_weight"]
            );
        }
    }
    if let Some(w) = &mut out {
        w.flush()?;
    }
    gz_sim_cli::export_archive(&args.output, BTreeMap::new(), frames)?;
    Ok(())
}
