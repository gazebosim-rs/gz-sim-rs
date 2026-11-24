// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reacting folded material on one finite sphere and matched oblique resolution controls.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/articulated_mobility.rs"]
mod gripper;
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/rotating_pad.rs"]
mod rotor;
mod soft_replay;
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/folded.rs"]
mod source;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::BodyState;
use gz_sim_scene::deformable::{self, DeformableSample};
use serde_json::{Value, json};
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
    time::Duration,
};

enum Owner {
    Fold(rotor::Rotor),
    Oblique(gripper::Gripper),
}
impl Owner {
    fn sample(&self) -> &DeformableSample {
        match self {
            Self::Fold(w) => w.material(),
            Self::Oblique(w) => deformable::bodies(&w.sim).next().unwrap().1,
        }
    }
    fn time(&self) -> Duration {
        match self {
            Self::Fold(w) => w.sim.elapsed(),
            Self::Oblique(w) => w.sim.elapsed(),
        }
    }
    fn next(&mut self) -> Result<(), Box<dyn Error>> {
        match self {
            Self::Fold(w) => w.advance(1),
            Self::Oblique(w) => w
                .sim
                .step_exact(1)
                .map_err(|e| format!("{e}: {:?}", w.sim.fault()).into()),
        }
    }
    fn bodies(&self) -> Vec<(BodyState, u64, f64, f64)> {
        match self {
            Self::Fold(w) => vec![(w.state(), w.link, 0.12, 0.75)],
            Self::Oblique(w) => w
                .states()
                .into_iter()
                .enumerate()
                .map(|(i, p)| (p, w.ids[i].0, [0.14, 0.1, 0.1][i], gripper::MASS[i]))
                .collect(),
        }
    }
    fn slot(&self, source: usize, feature: &[usize], n: DVec3) -> usize {
        match self {
            Self::Oblique(_) => source,
            Self::Fold(_) => {
                if feature.contains(&1) {
                    0
                } else if feature.contains(&2) {
                    1
                } else {
                    usize::from(n.y > 0.)
                }
            }
        }
    }
}
#[derive(Default)]
struct Group {
    count: u64,
    on_link: DVec3,
    link_torque: DVec3,
    on_nodes: [DVec3; 4],
    node_torque: DVec3,
    normals: DVec3,
    tangents: DVec3,
    first: u64,
    last: u64,
    slot: usize,
}
#[derive(Default)]
struct History {
    groups: BTreeMap<(usize, Vec<usize>), Group>,
    slots: [(DVec3, DVec3, Option<DVec3>); 2],
    hits: Vec<Value>,
    cuts: u64,
    multiple: u64,
    last_support_ns: u64,
    min_gap: Option<f64>,
}
impl History {
    fn add(&mut self, owner: &Owner) {
        let soft = owner.sample();
        for e in soft.impacts_this_step() {
            self.hits.push(json!({"time":e.time.as_nanos() as u64,"loads":e.material.loads.iter().map(|l|json!({"sphere":l.site.sphere,"point":l.site.point,"feature":l.site.feature.vertices(),"on_link":l.on_rigid,"on_nodes":l.on_particles,"normal":l.site.normal,"incoming":l.initial_normal_velocity,"outgoing":l.final_normal_velocity})).collect::<Vec<_>>(),"nodes":e.loaded_particles}));
        }
        for r in soft.support_this_step() {
            self.cuts += 1;
            self.last_support_ns = (r.start + r.duration).as_nanos() as u64;
            self.multiple += u64::from(r.source_features.windows(2).any(|w| w[0].0 == w[1].0));
            let low = r
                .region_clearances
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min);
            self.min_gap = Some(self.min_gap.map_or(low, |v| v.min(low)));
            for (t, application) in [
                (r.start, r.beginning.as_ref()),
                (r.start + r.duration, r.end.as_ref()),
            ] {
                let Some(a) = application else {
                    continue;
                };
                for l in &a.material {
                    let n = DVec3::from(l.site.normal);
                    let j = DVec3::from(l.on_rigid);
                    let at = DVec3::from(l.site.point);
                    let tangent = l
                        .friction
                        .map_or(DVec3::ZERO, |f| DVec3::from(f.on_material));
                    let normal = -j.dot(n) * n;
                    let id = owner.slot(l.site.sphere, l.site.feature.vertices(), n);
                    let group = self
                        .groups
                        .entry((l.site.sphere, l.site.feature.vertices().to_vec()))
                        .or_default();
                    if group.count == 0 {
                        group.first = t.as_nanos() as u64;
                    }
                    group.count += 1;
                    group.slot = id;
                    group.last = t.as_nanos() as u64;
                    group.on_link += j;
                    group.link_torque += at.cross(j);
                    group.normals += normal;
                    group.tangents += tangent;
                    for &(i, x, s) in &l.on_particles {
                        let s = DVec3::from(s);
                        group.on_nodes[i] += s;
                        group.node_torque += DVec3::from(x).cross(s);
                    }
                    if j.length_squared() > 0. {
                        let (nf, ft, p) = &mut self.slots[id];
                        *nf += normal;
                        *ft += tangent;
                        *p = Some(at);
                    }
                }
            }
        }
    }
    fn record(&self) -> Value {
        json!({"cuts":self.cuts,"multiple":self.multiple,"last_support_ns":self.last_support_ns,"minimum_region_gap":self.min_gap,"hits":self.hits,"features":self.groups.iter().map(|((s,f),v)|json!({"sphere":s,"vertices":f,"slot":v.slot,"count":v.count,"first_ns":v.first,"last_ns":v.last,"on_link":v.on_link.to_array(),"link_moment":v.link_torque.to_array(),"nodes":v.on_nodes.map(|v|v.to_array()),"node_moment":v.node_torque.to_array(),"normal":v.normals.to_array(),"tangent":v.tangents.to_array()})).collect::<Vec<_>>()})
    }
}
fn draw(
    f: &mut Frame,
    w: &Owner,
    seed: &[DVec3],
    lane: usize,
    name: &str,
    h: &History,
    dt: f64,
) -> Value {
    let fold = matches!(w, Owner::Fold(_));
    let q = DQuat::from_rotation_z(if fold { -0.72 } else { -0.55 });
    let shift = if fold {
        [0.66, -0.66][lane]
    } else {
        [0.85, -0.85][lane]
    };
    let display = |p: DVec3| q * p + shift * DVec3::Y;
    let ball = |f: &mut Frame, part: &str, p: DVec3, r: f64, color| {
        f.add(
            &format!("{name}_{part}"),
            display(p),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":r}),
            color,
        )
    };
    let edge = |f: &mut Frame, part: &str, a: DVec3, b: DVec3, len: f64, size: f64, color| {
        f.edge(
            &format!("{name}_{part}"),
            display(a),
            display(b),
            len,
            size,
            color,
        )
    };
    let bodies = w.bodies();
    let states=bodies.iter().map(|(b,id,r,m)|json!({"id":id,"x":b.position,"q":b.orientation,"v":b.velocity,"w":b.angular_velocity,"r":r,"m":m})).collect::<Vec<_>>();
    for (i, (b, _, r, _)) in bodies.iter().enumerate() {
        let c = DVec3::from(b.position);
        ball(f, &format!("body_{i}"), c, *r, [0.21, 0.47, 0.78, 1.]);
        ball(
            f,
            &format!("spin_{i}"),
            c + DQuat::from_array(b.orientation) * (0.96 * r * DVec3::X),
            0.013,
            [0.94, 0.78, 0.22, 1.],
        );
        if i > 0 && !fold {
            let s = bodies[0].0;
            let p = DVec3::from(s.position)
                + DQuat::from_array(s.orientation)
                    * DVec3::new(0.04, if i == 1 { 0.29 } else { -0.29 }, 0.);
            edge(
                f,
                &format!("arm_{i}"),
                p,
                c,
                0.60,
                0.040,
                [0.23, 0.36, 0.48, 1.],
            );
        }
    }
    let support = if fold {
        DVec3::new(0., 0., 0.64)
    } else {
        DVec3::new(-0.32, 0., 0.58)
    };
    edge(
        f,
        "pivot",
        DVec3::new(support.x, support.y, 0.04),
        support,
        support.z - 0.04,
        0.055,
        [0.28, 0.37, 0.48, 1.],
    );
    f.add(
        &format!("{name}_base"),
        display(DVec3::new(support.x, support.y, 0.02)),
        q,
        json!({"kind":"box","size":[0.42,0.36,0.04]}),
        [0.25, 0.29, 0.34, 1.],
    );
    let nodes = w.sample().particles();
    let pos = nodes
        .iter()
        .map(|p| DVec3::from(p.position))
        .collect::<Vec<_>>();
    let color = if lane == 0 {
        [0.12, 0.79, 0.60, 1.]
    } else {
        [0.97, 0.55, 0.18, 1.]
    };
    for (i, &p) in pos.iter().enumerate() {
        ball(f, &format!("node_{i}"), p, 0.016, color);
    }
    for (face, ids) in [[0, 1, 3], [0, 3, 2]].into_iter().enumerate() {
        for (dir, [o, p, t]) in [[0, 1, 2], [1, 0, 2], [2, 0, 1]].into_iter().enumerate() {
            let (o, p, t) = (ids[o], ids[p], ids[t]);
            for k in 1..=4 {
                let u = k as f64 / 4.;
                edge(
                    f,
                    &format!("weave_{face}_{dir}_{k}"),
                    pos[o].lerp(pos[p], u),
                    pos[o].lerp(pos[t], u),
                    u * seed[p].distance(seed[t]),
                    if k == 4 { 0.009 } else { 0.004 },
                    color,
                );
            }
        }
    }
    let mut origins = vec![];
    for (i, (normal, tangent, observed)) in h.slots.iter().enumerate() {
        let fallback = if fold {
            DVec3::from(bodies[0].0.position) + 0.12 * source::normal([-1., 1.][i])
        } else {
            DVec3::from(bodies[i + 1].0.position)
        };
        let at = observed.unwrap_or(fallback);
        origins.push(at.to_array());
        ball(
            f,
            &format!("contact_{i}"),
            at,
            0.009,
            [0.96, 0.92, 0.77, 1.],
        );
        for (key, v, scale, color) in [
            ("pressure", *normal, 0.058, [0.62, 0.85, 1., 1.]),
            ("traction", *tangent, 0.105, [1., 0.76, 0.13, 1.]),
        ] {
            for k in 1..=3 {
                ball(
                    f,
                    &format!("{key}_{i}_{k}"),
                    at + scale * (k as f64 / 3.) * v / dt,
                    if k == 3 { 0.016 } else { 0.009 },
                    color,
                );
            }
        }
    }
    json!({"bodies":states,"particles":nodes.iter().map(|p|json!({"x":p.position,"v":p.velocity,"m":1./p.inverse_mass})).collect::<Vec<_>>(),"history":h.record(),"origins":origins})
}
#[derive(Parser)]
struct Args {
    output: PathBuf,
    #[arg(long)]
    oblique: bool,
    #[arg(long)]
    measurements: PathBuf,
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    let (mut worlds, names, duration, step) = if a.oblique {
        (
            [
                Owner::Oblique(gripper::Gripper::new(&source::oblique(0.9, 250000))?),
                Owner::Oblique(gripper::Gripper::new(&source::oblique(0.9, 15625))?),
            ],
            ["coarse", "fine"],
            1800,
            10,
        )
    } else {
        (
            [
                Owner::Fold(rotor::Rotor::new(&source::sdf(0., 250000, false))?),
                Owner::Fold(rotor::Rotor::new(&source::sdf(0.32, 250000, true))?),
            ],
            ["symmetric", "driven"],
            600,
            4,
        )
    };
    let seed = worlds[0]
        .sample()
        .particles()
        .iter()
        .map(|p| p.position.into())
        .collect::<Vec<_>>();
    let mut out = BufWriter::new(File::create_new(a.measurements)?);
    let mut frames = Vec::new();
    for i in 0..=duration / step {
        let mut f = Frame::default();
        let mut row = json!({"time_ns":(i*step*1_000_000) as u64});
        for (lane, w) in worlds.iter_mut().enumerate() {
            let mut h = History::default();
            if i > 0 {
                for _ in 0..step {
                    w.next()?;
                    h.add(w);
                }
            }
            row[names[lane]] = draw(&mut f, w, &seed, lane, names[lane], &h, step as f64 * 0.001);
        }
        frames.push(f.finish(worlds[0].time(), i + 1));
        serde_json::to_writer(&mut out, &row)?;
        writeln!(out)?;
    }
    out.flush()?;
    gz_sim_cli::export_archive(&a.output, BTreeMap::new(), frames)?;
    println!(
        "{} chronological two-world physical frames, with per-feature momentum and moment sums.",
        1 + duration / step
    );
    Ok(())
}
