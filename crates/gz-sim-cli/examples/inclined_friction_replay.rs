// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two ordinary SDF worlds, rendered from all four measured pads and cloth particles.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/inclined_sheet.rs"]
mod sheet;
mod soft_replay;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_sim_scene::deformable::articulated_impact::ArticulatedSupportInterval;
use serde_json::{Value, json};
use sheet::*;
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};

#[derive(Parser)]
struct Args {
    output: PathBuf,
    #[arg(long)]
    moving: bool,
    #[arg(long)]
    measurements: PathBuf,
}
#[derive(Default)]
struct Forces {
    normal: [DVec3; 4],
    tangent: [DVec3; 4],
    at: [Option<DVec3>; 4],
}
fn interval(r: &ArticulatedSupportInterval, forces: &mut Forces) -> Value {
    let mut result = json!({"start_ns":r.start.as_nanos() as u64,"dt_ns":r.duration.as_nanos() as u64,"end_gaps":r.endpoint_clearances,"travel":r.tangential_travel,"trial_count":r.trajectory_trials});
    for (name, application) in [("head", &r.beginning), ("tail", &r.end)] {
        let Some(a) = application else {
            result[name] = Value::Null;
            continue;
        };
        let mut loads = vec![];
        for l in &a.material {
            let i = l.site.sphere;
            let j = -DVec3::from(l.on_rigid);
            let n = DVec3::from(l.site.normal);
            let f = l.friction.expect("frictional cloth");
            let jt = DVec3::from(f.on_material);
            forces.normal[i] += j.dot(n) * n;
            forces.tangent[i] += jt;
            if j.length_squared() > 0. {
                forces.at[i] = Some(DVec3::from(l.site.point));
            }
            loads.push(json!({"sphere":i,"point":l.site.point,"normal":l.site.normal,"vertices":l.site.feature.vertices(),"weights":l.site.weights,"j_on_link":l.on_rigid,"j_on_nodes":l.on_particles,"jt_on_skin":f.on_material,"slip_before":f.initial_slip,"slip_after":f.final_slip}));
        }
        result[name] = json!(loads);
    }
    result
}
fn draw(
    f: &mut Frame,
    g: &Incline,
    seed: &[DVec3],
    cold_pads: &[DVec3; 4],
    lane: usize,
    name: &str,
    forces: &Forces,
) -> Value {
    let rot = DQuat::from_rotation_z(-0.81);
    let display = |v: DVec3| rot * v + [0.75, -0.75][lane] * DVec3::Y;
    let tag = |key: &str| format!("{name}_{key}");
    let ball = |f: &mut Frame, key: &str, p: DVec3, r: f64, color| {
        f.add(
            &tag(key),
            display(p),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":r}),
            color,
        )
    };
    let edge = |f: &mut Frame, key: &str, a: DVec3, b: DVec3, length: f64, w: f64, color| {
        f.edge(&tag(key), display(a), display(b), length, w, color)
    };
    let pads = g.states();
    let nodes = g.material().particles();
    let x = nodes
        .iter()
        .map(|p| DVec3::from(p.position))
        .collect::<Vec<_>>();
    let sheet = if lane == 0 {
        [0.1, 0.82, 0.64, 1.]
    } else {
        [0.96, 0.54, 0.18, 1.]
    };
    let slate = [0.26, 0.34, 0.46, 1.];
    // Rings mark the actual native pivot positions; the wire frame connects
    // the four current sphere centers, while a light trace shows the start.
    for (i, p) in pads.iter().enumerate() {
        let center = DVec3::from(p.position);
        ball(
            f,
            &format!("pad_{i}"),
            center,
            0.085,
            [0.14, 0.43, 0.76, 1.],
        );
        let target = DVec3::from(pads[(i + 1) % 4].position);
        edge(
            f,
            &format!("rail_{i}"),
            center,
            target,
            cold_pads[i].distance(cold_pads[(i + 1) % 4]),
            0.035,
            slate,
        );

        ball(f, &format!("corner_{i}"), x[i], 0.014, sheet);
        let at = forces.at[i].unwrap_or(center + 0.085 * normal());
        for (key, vector, color, scale) in [
            ("pressure", forces.normal[i], [0.41, 0.74, 1., 1.], 0.105),
            ("friction", forces.tangent[i], [0.99, 0.83, 0.14, 1.], 0.36),
        ] {
            for k in 1..=3 {
                ball(
                    f,
                    &format!("{key}_{i}_{k}"),
                    at + k as f64 / 3. * scale * vector / 0.004,
                    if k == 3 { 0.014 } else { 0.008 },
                    color,
                );
            }
        }
    }
    edge(
        f,
        "world_pivot",
        0.035 * DVec3::Z,
        pads[0].position[2] * DVec3::Z,
        cold_pads[0].z - 0.035,
        0.075,
        slate,
    );
    for (i, [a, b]) in [[0, 1], [1, 3], [3, 2], [2, 0]].into_iter().enumerate() {
        edge(
            f,
            &format!("initial_{i}"),
            seed[a] - 0.012 * normal(),
            seed[b] - 0.012 * normal(),
            seed[b].distance(seed[a]),
            0.007,
            [0.57, 0.56, 0.57, 1.],
        );
    }
    for (t, ids) in [[0, 1, 3], [0, 3, 2]].into_iter().enumerate() {
        let [a, b, c] = ids.map(|i| x[i]);
        for (j, (o, p, q)) in [(a, b, c), (b, a, c), (c, a, b)].into_iter().enumerate() {
            for k in 1..=3 {
                let u = k as f64 / 3.;
                edge(
                    f,
                    &format!("mesh_{t}_{j}_{k}"),
                    o.lerp(p, u),
                    o.lerp(q, u),
                    u * seed[ids[[1, 0, 0][j]]].distance(seed[ids[[2, 2, 1][j]]]),
                    if k == 3 { 0.009 } else { 0.004 },
                    sheet,
                );
            }
        }
    }
    let (cm, v, _) = g.centroid();
    for k in 1..=3 {
        ball(
            f,
            &format!("velocity_{k}"),
            cm + 0.055 * normal() + 0.44 * (k as f64 / 3.) * v,
            if k == 3 { 0.021 } else { 0.014 },
            [0.94, 0.25, 0.20, 1.],
        );
    }
    f.add(
        &tag("base"),
        display(DVec3::new(0., 0., 0.015)),
        rot,
        json!({"kind":"box","size":[1.32,0.99,0.03]}),
        [0.24, 0.28, 0.33, 1.],
    );
    json!({"pads":pads.iter().zip(g.ids).map(|(p,(id,_))|json!({"id":id,"x":p.position,"v":p.velocity,"q":p.orientation,"w":p.angular_velocity})).collect::<Vec<_>>(),"particles":nodes.iter().map(|p|json!({"x":p.position,"v":p.velocity,"m":1./p.inverse_mass})).collect::<Vec<_>>(),"normal_n":forces.normal.map(|j|(j/0.004).to_array()),"tangent_n":forces.tangent.map(|j|(j/0.004).to_array()),"origins":forces.at.map(|p|p.map(|a|a.to_array()))})
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let lanes = if args.moving {
        [("arrest", 0.45, 0.32), ("reverse", 0.15, -0.42)]
    } else {
        [("hold", 0.45, 0.), ("slip", 0.2, 0.)]
    };
    let mut worlds = lanes
        .iter()
        .map(|&(_, mu, s)| Incline::new(&sdf(mu, s, 250000)))
        .collect::<Result<Vec<_>, _>>()?;
    let seed = worlds[0]
        .material()
        .particles()
        .iter()
        .map(|p| DVec3::from(p.position))
        .collect::<Vec<_>>();
    let cold_pads = worlds[0].states().map(|b| DVec3::from(b.position));
    let mut out = BufWriter::new(File::create_new(&args.measurements)?);
    let mut frames = Vec::new();
    for index in 0..=125 {
        let mut f = Frame::default();
        let mut measurement = json!({"time_ns":index as u64*4_000_000});
        for (i, (g, &(name, mu, v0))) in worlds.iter_mut().zip(&lanes).enumerate() {
            let mut forces = Forces::default();
            let mut cuts = Vec::new();
            if index > 0 {
                for _ in 0..4 {
                    g.step()?;
                    assert!(g.material().impacts_this_step().is_empty());
                    for r in g.material().support_this_step() {
                        cuts.push(interval(r, &mut forces));
                    }
                }
            }
            let mut row = draw(&mut f, g, &seed, &cold_pads, i, name, &forces);
            row["intervals"] = json!(cuts);
            row["mu"] = mu.into();
            row["initial_speed"] = v0.into();
            measurement[name] = row;
        }
        serde_json::to_writer(&mut out, &measurement)?;
        writeln!(out)?;
        frames.push(f.finish(worlds[0].sim.elapsed(), index + 1));
    }
    out.flush()?;
    gz_sim_cli::export_archive(&args.output, BTreeMap::new(), frames)?;
    println!(
        "126 genuine world cuts, 4 ms between frames, complete per-site support impulse, witnessed slip and Cartesian force history."
    );
    Ok(())
}
